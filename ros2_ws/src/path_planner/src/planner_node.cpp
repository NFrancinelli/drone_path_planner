// Flies the drone to a goal through the live OctoMap: A*, shortcut, then a smooth
// trajectory, sent to the trajectory server. Publishes the raw path and the trajectory for
// RViz. The goal is pursued until a trajectory to it has been flown:
//   - while flying, the rest of the trajectory is checked against each new map
//     (check_period); if it's blocked, a new plan replaces it at once;
//   - every improve_period, a new plan replaces it if it arrives clearly sooner
//     (improve_min_gain seconds and improve_min_ratio of the remaining time);
//   - if a blocked trajectory can't be replanned, the drone brakes to a stop, hovers, and
//     plans again every improve_period as the map grows;
//   - if the flown trajectory comes within emergency_radius of an obstacle sooner than a
//     new plan could take over, the drone brakes at once.
// Every voxel ever seen occupied stays an obstacle (StickyMap): the worlds are static, and
// OctoMap can open holes in thin walls.
//
// While the drone is flying a trajectory (`active_trajectory`, from the server), a new plan
// continues it: it starts replan_lead seconds ahead on that trajectory, in exactly its state
// there, and the server switches over at that moment. replan_lead must cover planning time.
// From rest, the server first turns the drone to face the first leg, then starts it.
//
// Goals come from RViz's "2D Goal Pose" tool, which sends z = 0, so the goal altitude is a
// parameter.
//
// RViz markers on `planner_markers`: the goal, the trajectory sent coloured by speed, and
// for a few seconds after a replan, the blocked trajectory in red with where it hit.

#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <drone_interfaces/msg/waypoint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "path_planner/astar.hpp"
#include "path_planner/octomap_map.hpp"
#include "path_planner/smoothing.hpp"
#include "path_planner/sticky_map.hpp"
#include "trajectory_server/msg_conversions.hpp"

namespace path_planner
{

using trajectory_server::WaypointTrajectory;
using Ms = std::chrono::duration<double, std::milli>;

// A trajectory sent to the server and the time it takes over.
struct Segment
{
  std::shared_ptr<const WaypointTrajectory> trajectory;
  rclcpp::Time start;
};

// What the drone flies, as one trajectory from `origin`: each segment from its start until
// the next one takes over. Mirrors the server's queue of continuations.
class Flown : public trajectory_server::Trajectory
{
public:
  Flown(std::deque<Segment> segments, const rclcpp::Time & origin)
  : segments_(std::move(segments)), origin_(origin) {}

  double duration() const override
  {
    const auto & last = segments_.back();
    return std::max((last.start - origin_).seconds() + last.trajectory->duration(), 0.0);
  }

  trajectory_server::TrajectoryPoint sample(double t) const override
  {
    const rclcpp::Time at = origin_ + rclcpp::Duration::from_seconds(t);
    const Segment * s = &segments_.front();
    for (const auto & segment : segments_) {
      if (segment.start <= at) {
        s = &segment;
      }
    }
    return s->trajectory->sample((at - s->start).seconds());
  }

private:
  std::deque<Segment> segments_;
  rclcpp::Time origin_;
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("path_planner")
  {
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));
    goal_altitude_ = declare_parameter("goal_altitude", 2.5);
    replan_lead_ = declare_parameter("replan_lead", 1.0);
    const double check_period = declare_parameter("check_period", 0.2);
    const double improve_period = declare_parameter("improve_period", 2.0);
    emergency_radius_ = declare_parameter("emergency_radius", 0.4);
    improve_min_gain_ = declare_parameter("improve_min_gain", 1.0);
    improve_min_ratio_ = declare_parameter("improve_min_ratio", 0.1);

    AStarPlanner::Params params;
    params.inflation_radius = declare_parameter("inflation_radius", params.inflation_radius);
    params.unknown_cost = declare_parameter("unknown_cost", params.unknown_cost);
    params.vertical_cost = declare_parameter("vertical_cost", params.vertical_cost);
    params.clearance_radius = declare_parameter("clearance_radius", params.clearance_radius);
    params.clearance_cost = declare_parameter("clearance_cost", params.clearance_cost);
    params.max_expansions =
      static_cast<int>(declare_parameter("max_expansions", params.max_expansions));
    const auto lo = declare_parameter("bounds_min", std::vector<double>{-20.0, -20.0, 0.5});
    const auto hi = declare_parameter("bounds_max", std::vector<double>{20.0, 20.0, 4.0});
    params.bounds = Eigen::AlignedBox3d(
      Eigen::Vector3d(lo.at(0), lo.at(1), lo.at(2)), Eigen::Vector3d(hi.at(0), hi.at(1), hi.at(2)));
    planner_ = std::make_unique<AStarPlanner>(params);

    smoothing_.max_vel = declare_parameter("max_vel", smoothing_.max_vel);
    smoothing_.max_acc = declare_parameter("max_acc", smoothing_.max_acc);
    smoothing_.max_yaw_rate = declare_parameter("max_yaw_rate", smoothing_.max_yaw_rate);
    smoothing_.max_yaw_acc = declare_parameter("max_yaw_acc", smoothing_.max_yaw_acc);
    smoothing_.collision_radius = declare_parameter("collision_radius", smoothing_.collision_radius);
    smoothing_.corner_distance = declare_parameter("corner_distance", smoothing_.corner_distance);
    if (smoothing_.collision_radius > params.inflation_radius) {
      throw std::invalid_argument("collision_radius must not exceed inflation_radius");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Latched so RViz shows the last path even if it subscribes later.
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "planned_path", rclcpp::QoS(1).transient_local());
    smoothed_pub_ = create_publisher<nav_msgs::msg::Path>(
      "smoothed_path", rclcpp::QoS(1).transient_local());
    // Not latched: a trajectory server that starts later must not fly a stale plan.
    trajectory_pub_ = create_publisher<drone_interfaces::msg::WaypointTrajectory>("trajectory", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "planner_markers", rclcpp::QoS(10).transient_local());
    // Only the latest map is kept; it's deserialized when needed.
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_binary", 1,
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr msg) {map_msg_ = msg;});
    // Latched by the server: the planner trajectory it's flying, stamped with its start.
    active_sub_ = create_subscription<drone_interfaces::msg::WaypointTrajectory>(
      "active_trajectory", rclcpp::QoS(1).transient_local(),
      [this](drone_interfaces::msg::WaypointTrajectory::ConstSharedPtr msg) {on_active(*msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});

    check_timer_ = create_wall_timer(
      std::chrono::duration<double>(check_period), [this]() {check();});
    improve_timer_ = create_wall_timer(
      std::chrono::duration<double>(improve_period), [this]() {improve();});
  }

private:
  enum class Reason { NewGoal, Blocked, Improve, Retry };

  static const char * name(Reason r)
  {
    switch (r) {
      case Reason::NewGoal: return "new goal";
      case Reason::Blocked: return "blocked";
      case Reason::Improve: return "improve";
      case Reason::Retry: return "retry";
    }
    return "?";
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (msg.header.frame_id != world_frame_) {
      RCLCPP_WARN(get_logger(), "Goal in frame '%s', expected '%s'; ignored",
        msg.header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    goal_ = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, goal_altitude_);
    RCLCPP_INFO(get_logger(), "Goal (%.2f, %.2f, %.2f)", goal_->x(), goal_->y(), goal_->z());
    publish_goal_marker();
    replan(Reason::NewGoal);
  }

  // --- drone state, as far as the planner knows ---------------------------------------

  // The server started a trajectory. Known if we sent it as a continuation (then the ones
  // before it are done); otherwise one sent from rest, which replaces everything. Latched
  // messages can arrive late: one older than what we know is stale.
  void on_active(const drone_interfaces::msg::WaypointTrajectory & msg)
  {
    const rclcpp::Time start(msg.header.stamp, get_clock()->get_clock_type());
    if (!flown_.empty() && start < flown_.front().start) {
      return;
    }
    for (size_t i = 0; i < flown_.size(); ++i) {
      if (flown_[i].start == start) {
        flown_.erase(flown_.begin(), flown_.begin() + i);
        return;
      }
    }
    flown_ = {{trajectory_server::from_msg(msg), start}};
    awaiting_since_.reset();
  }

  // Still flying at time t.
  bool flying(const rclcpp::Time & t) const
  {
    if (flown_.empty()) {
      return false;
    }
    const auto & last = flown_.back();
    return (t - last.start).seconds() < last.trajectory->duration();
  }

  // State of the drone's trajectory at time t (while flying).
  trajectory_server::TrajectoryPoint state_at(const rclcpp::Time & t) const
  {
    return Flown(flown_, t).sample(0.0);
  }

  static StartState start_state(const trajectory_server::TrajectoryPoint & s)
  {
    StartState state;
    state.velocity = s.velocity;
    state.acceleration = s.acceleration;
    state.yaw = s.yaw;
    state.yaw_rate = s.yaw_rate;
    return state;
  }

  // A trajectory sent from rest that the server hasn't started yet (it turns first).
  bool awaiting_start()
  {
    if (awaiting_since_ && (now() - *awaiting_since_).seconds() > 15.0) {
      RCLCPP_WARN(get_logger(), "Trajectory server never started the last trajectory");
      awaiting_since_.reset();
    }
    return awaiting_since_.has_value();
  }

  // --- timers -------------------------------------------------------------------------

  // Goal reached, or the rest of the active trajectory runs into the latest map.
  void check()
  {
    if (!goal_ || awaiting_start()) {
      return;
    }
    if (!flying(now())) {
      // Flown all the way to the goal, and actually there (not stopped short after a crash
      // or a brake): done. Otherwise the goal stays, and improve() tries again from here.
      const auto & last = flown_.empty() ? nullptr : flown_.back().trajectory;
      if (last && (last->sample(last->duration()).position - *goal_).norm() < 0.05) {
        try {
          const auto & t = tf_buffer_->lookupTransform(
            world_frame_, body_frame_, tf2::TimePointZero).transform.translation;
          const double off = (Eigen::Vector3d(t.x, t.y, t.z) - *goal_).norm();
          if (off > kReachedTolerance) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Trajectory to the goal flown, but the drone is %.2f m from it", off);
            return;
          }
        } catch (const tf2::TransformException &) {
          return;
        }
        RCLCPP_INFO(get_logger(), "Goal reached");
        goal_.reset();
        publish_goal_marker();
      }
      return;
    }
    const auto tree = latest_map();
    if (!tree) {
      return;
    }
    const OctomapMap octomap(tree);
    const StickyMap map(octomap, sticky_);
    const Flown flown(flown_, now());

    // Too close for a new plan to take over in time: brake now, no planning needed.
    const auto danger = first_collision(map, flown, 0.0, emergency_radius_);
    if (danger && *danger < replan_lead_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Obstacle within emergency_radius %.1f s ahead; braking now", *danger);
      publish_blocked_markers(flown, 0.0, *danger);
      const rclcpp::Time at = now() + rclcpp::Duration::from_seconds(kBrakeLead);
      const auto s = state_at(at);
      send(brake(s.position, start_state(s), smoothing_), at, true);
      return;
    }
    const auto hit = first_collision(map, flown, 0.0, smoothing_.collision_radius);
    if (!hit) {
      return;
    }
    // Closer than collision_radius but not emergency_radius: a margin over the drone that
    // squeezing through a gap can graze as the map fills in. Too soon to replan anyway.
    if (*hit < replan_lead_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Within collision_radius of an obstacle %.1f s ahead, too soon to replan", *hit);
      return;
    }
    RCLCPP_WARN(get_logger(), "Trajectory blocked %.1f s ahead, replanning", *hit);
    publish_blocked_markers(flown, 0.0, *hit);
    replan(Reason::Blocked);
  }

  // A shorter way while flying (if there's still a trajectory to take over from), or another
  // try once stopped short of the goal.
  void improve()
  {
    if (!goal_ || awaiting_start()) {
      return;
    }
    const rclcpp::Time takeover = now() + rclcpp::Duration::from_seconds(replan_lead_);
    if (flying(takeover)) {
      replan(Reason::Improve);
    } else if (!flying(now())) {
      replan(Reason::Retry);
    }
  }

  // --- planning -----------------------------------------------------------------------

  std::shared_ptr<const octomap::OcTree> latest_map()
  {
    if (!map_msg_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No map received yet");
      return nullptr;
    }
    if (map_msg_->header.frame_id != world_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Map in frame '%s', expected '%s'",
        map_msg_->header.frame_id.c_str(), world_frame_.c_str());
      return nullptr;
    }
    std::shared_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::binaryMsgToMap(*map_msg_));
    const auto tree = std::dynamic_pointer_cast<octomap::OcTree>(abstract);
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "Map message is not an OcTree");
      return nullptr;
    }
    OctomapMap(tree).collect_occupied(sticky_);
    return tree;
  }

  // Plans to goal_ and decides what to do with the result, depending on why.
  void replan(Reason reason)
  {
    // Only a new goal or a blocked path are worth reporting every time; the periodic
    // attempts are logged when they change something.
    const bool verbose = reason == Reason::NewGoal || reason == Reason::Blocked;
    const auto tree = latest_map();
    if (!tree) {
      return;
    }

    // Continue the trajectory being flown if it's still going at the takeover time, else
    // start at rest from where the drone is.
    Eigen::Vector3d start;
    StartState start_state;  // at rest
    const rclcpp::Time takeover = now() + rclcpp::Duration::from_seconds(replan_lead_);
    const bool continuing = flying(takeover);
    if (continuing) {
      const auto s = state_at(takeover);
      start = s.position;
      start_state = PlannerNode::start_state(s);
    } else {
      try {
        const auto tf = tf_buffer_->lookupTransform(world_frame_, body_frame_, tf2::TimePointZero);
        const auto & t = tf.transform.translation;
        start = {t.x, t.y, t.z};
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "No drone pose: %s", e.what());
        return;
      }
    }

    const auto planned = plan(tree, start, start_state, verbose);
    if (!planned.trajectory) {
      if (continuing && (reason == Reason::NewGoal || reason == Reason::Blocked)) {
        // From a fresh takeover point: a search that fails can take longer than replan_lead.
        const rclcpp::Time brake_at = now() + rclcpp::Duration::from_seconds(replan_lead_);
        RCLCPP_WARN(get_logger(), "No way to the goal (%s); braking to a stop", name(reason));
        if (flying(brake_at)) {
          const auto s = state_at(brake_at);
          send(brake(s.position, PlannerNode::start_state(s), smoothing_), brake_at, true);
        }
      } else if (verbose) {
        RCLCPP_WARN(get_logger(), "No way to the goal yet; hovering, will retry");
      }
      if (verbose) {
        clear_paths();
      }
      return;
    }

    if (reason == Reason::Improve) {
      const double remaining = Flown(flown_, takeover).duration();
      const double gain = remaining - planned.trajectory->duration();
      if (gain < improve_min_gain_ || gain < improve_min_ratio_ * remaining) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Shorter way found: %.1f s instead of %.1f s",
        planned.trajectory->duration(), remaining);
    } else if (!verbose) {
      RCLCPP_INFO(get_logger(), "Found a way to the goal (%s)", name(reason));
    }
    publish_path(*path_pub_, planned.path, {});
    send(planned.trajectory, continuing ? takeover : now(), continuing);
  }

  struct Planned
  {
    std::vector<Eigen::Vector3d> path;  // raw A* path
    std::shared_ptr<const WaypointTrajectory> trajectory;  // null on failure
  };

  Planned plan(
    std::shared_ptr<const octomap::OcTree> tree, const Eigen::Vector3d & start,
    const StartState & start_state, bool verbose)
  {
    Planned out;
    // One distance map for the search, shortcut and smoothing.
    const OctomapMap octomap(std::move(tree));
    const StickyMap sticky(octomap, sticky_);
    const auto t_map = std::chrono::steady_clock::now();
    const DistanceMap map(sticky, planner_->params().bounds, planner_->max_distance());
    const auto t0 = std::chrono::steady_clock::now();
    const auto result = planner_->plan(map, start, *goal_);
    const auto t1 = std::chrono::steady_clock::now();
    if (result.status != AStarPlanner::Status::Success) {
      if (verbose) {
        RCLCPP_WARN(get_logger(), "No path from (%.2f, %.2f, %.2f): %s (%d expansions, %.1f ms)",
          start.x(), start.y(), start.z(), AStarPlanner::to_string(result.status),
          result.expansions, Ms(t1 - t0).count());
      }
      return out;
    }
    const auto waypoints = shortcut(map, result.path, planner_->params());
    const auto t2 = std::chrono::steady_clock::now();
    const auto smoothed = smooth(map, waypoints, smoothing_, start_state);
    const auto t3 = std::chrono::steady_clock::now();
    if (!smoothed.trajectory) {
      RCLCPP_WARN(get_logger(), "Smoothing failed (%zu waypoints): %s", waypoints.size(),
        smoothed.failure);
      for (const auto & w : waypoints) {
        RCLCPP_WARN(get_logger(), "  waypoint (%.3f, %.3f, %.3f)", w.x(), w.y(), w.z());
      }
      const Eigen::Vector3d & v = start_state.velocity, & a = start_state.acceleration;
      RCLCPP_WARN(get_logger(), "  start velocity (%.3f, %.3f, %.3f), acceleration (%.3f, %.3f, "
        "%.3f), yaw %.3f, yaw rate %.3f", v.x(), v.y(), v.z(), a.x(), a.y(), a.z(),
        start_state.yaw.value_or(NAN), start_state.yaw_rate);
      return out;
    }
    const char * format = "Plan: %zu A* points, %zu waypoints, %d stopped at, %.1f s long; "
      "distance map %.0f ms, A* %.0f ms (%d expansions), shortcut %.1f ms, smoothing %.1f ms";
    const auto & traj = *smoothed.trajectory;
    if (verbose) {
      RCLCPP_INFO(get_logger(), format, result.path.size(), waypoints.size(),
        smoothed.stopped_knots, traj.duration(), Ms(t0 - t_map).count(), Ms(t1 - t0).count(),
        result.expansions, Ms(t2 - t1).count(), Ms(t3 - t2).count());
    } else {
      RCLCPP_DEBUG(get_logger(), format, result.path.size(), waypoints.size(),
        smoothed.stopped_knots, traj.duration(), Ms(t0 - t_map).count(), Ms(t1 - t0).count(),
        result.expansions, Ms(t2 - t1).count(), Ms(t3 - t2).count());
    }
    out.path = result.path;
    out.trajectory = smoothed.trajectory;
    return out;
  }

  // Sends a trajectory to the server: as a continuation taking over at `start_time`, or
  // from rest. A continuation is assumed accepted, so it's checked from now on.
  void send(
    std::shared_ptr<const WaypointTrajectory> trajectory, const rclcpp::Time & start_time,
    bool continuation)
  {
    if (continuation) {
      const double margin = (start_time - now()).seconds();
      if (margin < 0.05) {
        RCLCPP_WARN(get_logger(), "Planning overran replan_lead (%.2f s left); not sent", margin);
        return;
      }
      RCLCPP_INFO(get_logger(), "Continuing the current trajectory, takeover in %.2f s", margin);
      // As the server does: drop what would start at or after it, and what's done.
      while (!flown_.empty() && flown_.back().start >= start_time) {
        flown_.pop_back();
      }
      flown_.push_back({trajectory, start_time});
      while (flown_.size() > 1 && flown_[1].start <= now()) {
        flown_.pop_front();
      }
    } else {
      awaiting_since_ = now();
    }
    std::vector<Eigen::Vector3d> points;
    std::vector<double> yaws, speeds;
    for (double t = 0.0; ; t = std::min(t + 0.1, trajectory->duration())) {
      const auto sp = trajectory->sample(t);
      points.push_back(sp.position);
      yaws.push_back(sp.yaw);
      speeds.push_back(sp.velocity.norm());
      if (t >= trajectory->duration()) {
        break;
      }
    }
    publish_path(*smoothed_pub_, points, yaws);
    publish_trajectory_marker(points, speeds);

    auto msg = trajectory_server::to_msg(*trajectory);
    msg.header.stamp = start_time;
    msg.header.frame_id = world_frame_;
    trajectory_pub_->publish(msg);
  }

  // --- RViz ---------------------------------------------------------------------------

  visualization_msgs::msg::Marker marker(const std::string & ns, int type) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = now();
    m.ns = ns;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  static geometry_msgs::msg::Point point(const Eigen::Vector3d & p)
  {
    geometry_msgs::msg::Point out;
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    return out;
  }

  static std_msgs::msg::ColorRGBA color(double r, double g, double b, double a = 1.0)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = static_cast<float>(r);
    c.g = static_cast<float>(g);
    c.b = static_cast<float>(b);
    c.a = static_cast<float>(a);
    return c;
  }

  // Green sphere on the goal, removed once it's reached.
  void publish_goal_marker()
  {
    auto m = marker("goal", visualization_msgs::msg::Marker::SPHERE);
    if (goal_) {
      m.pose.position = point(*goal_);
      m.scale.x = m.scale.y = m.scale.z = 0.4;
      m.color = color(0.1, 0.9, 0.2, 0.8);
    } else {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(m);
    marker_pub_->publish(array);
  }

  // The trajectory sent, coloured by speed: blue at rest to orange at max_vel.
  void publish_trajectory_marker(
    const std::vector<Eigen::Vector3d> & points, const std::vector<double> & speeds)
  {
    auto m = marker("trajectory", visualization_msgs::msg::Marker::LINE_STRIP);
    m.scale.x = 0.06;
    for (size_t i = 0; i < points.size(); ++i) {
      const double s = std::clamp(speeds[i] / smoothing_.max_vel, 0.0, 1.0);
      m.points.push_back(point(points[i]));
      m.colors.push_back(color(0.1 + 0.9 * s, 0.4 + 0.1 * s, 1.0 - s));
    }
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(m);
    marker_pub_->publish(array);
  }

  // For a few seconds after a replan: the rest of the blocked trajectory in red, and a red
  // sphere where it came too close to the map.
  void publish_blocked_markers(
    const trajectory_server::Trajectory & trajectory, double from, double hit)
  {
    const auto lifetime = rclcpp::Duration::from_seconds(3.0);
    auto line = marker("blocked", visualization_msgs::msg::Marker::LINE_STRIP);
    line.id = 0;
    line.scale.x = 0.05;
    line.color = color(0.9, 0.1, 0.1, 0.8);
    line.lifetime = lifetime;
    for (double t = from; ; t = std::min(t + 0.1, trajectory.duration())) {
      line.points.push_back(point(trajectory.sample(t).position));
      if (t >= trajectory.duration()) {
        break;
      }
    }
    auto sphere = marker("blocked", visualization_msgs::msg::Marker::SPHERE);
    sphere.id = 1;
    sphere.pose.position = point(trajectory.sample(hit).position);
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * smoothing_.collision_radius;
    sphere.color = color(0.9, 0.1, 0.1, 0.5);
    sphere.lifetime = lifetime;
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(line);
    array.markers.push_back(sphere);
    marker_pub_->publish(array);
  }

  void clear_paths()
  {
    publish_path(*path_pub_, {}, {});
    publish_path(*smoothed_pub_, {}, {});
  }

  // yaws: one per point, or empty for identity orientations.
  void publish_path(
    rclcpp::Publisher<nav_msgs::msg::Path> & pub, const std::vector<Eigen::Vector3d> & points,
    const std::vector<double> & yaws)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = world_frame_;
    for (size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x();
      pose.pose.position.y = points[i].y();
      pose.pose.position.z = points[i].z();
      const double yaw = yaws.empty() ? 0.0 : yaws[i];
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(pose);
    }
    pub.publish(path);
  }

  std::string world_frame_, body_frame_;
  // How soon an emergency brake takes over: enough for the message to reach the server.
  static constexpr double kBrakeLead = 0.15;  // [s]
  // How close to the goal the drone must end up for it to count as reached.
  static constexpr double kReachedTolerance = 0.5;  // [m]

  double goal_altitude_, replan_lead_, improve_min_gain_, improve_min_ratio_, emergency_radius_;
  std::unique_ptr<AStarPlanner> planner_;
  SmoothingParams smoothing_;

  std::optional<Eigen::Vector3d> goal_;  // until a trajectory to it has been flown
  octomap_msgs::msg::Octomap::ConstSharedPtr map_msg_;
  std::deque<Segment> flown_;  // being flown, then continuations waiting to take over
  KeySet sticky_;  // every voxel ever seen occupied
  std::optional<rclcpp::Time> awaiting_since_;  // sent from rest, not started yet

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<drone_interfaces::msg::WaypointTrajectory>::SharedPtr active_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, smoothed_pub_;
  rclcpp::Publisher<drone_interfaces::msg::WaypointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_, improve_timer_;
};

}  // namespace path_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
