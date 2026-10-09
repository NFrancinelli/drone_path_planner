// Plans from the drone's current position to a goal on the live OctoMap: A*, shortcut,
// then a smooth trajectory. Sends the trajectory to the trajectory server to fly, and
// publishes the raw path and the trajectory for RViz. Plans once per goal.
//
// Goals come from RViz's "2D Goal Pose" tool, which sends z = 0, so the goal altitude is a
// parameter.

#include <chrono>
#include <cmath>
#include <memory>
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

#include "path_planner/astar.hpp"
#include "path_planner/octomap_map.hpp"
#include "path_planner/smoothing.hpp"
#include "trajectory_server/msg_conversions.hpp"

namespace path_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("path_planner")
  {
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));
    goal_altitude_ = declare_parameter("goal_altitude", 2.5);

    AStarPlanner::Params params;
    params.inflation_radius = declare_parameter("inflation_radius", params.inflation_radius);
    params.unknown_cost = declare_parameter("unknown_cost", params.unknown_cost);
    params.vertical_cost = declare_parameter("vertical_cost", params.vertical_cost);
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
    // Only the latest map is kept; it's deserialized when a plan is requested.
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_binary", 1,
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr msg) {map_msg_ = msg;});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (msg.header.frame_id != world_frame_) {
      RCLCPP_WARN(get_logger(), "Goal in frame '%s', expected '%s'; ignored",
        msg.header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    const Eigen::Vector3d goal(msg.pose.position.x, msg.pose.position.y, goal_altitude_);
    RCLCPP_INFO(get_logger(), "Goal (%.2f, %.2f, %.2f)", goal.x(), goal.y(), goal.z());
    plan(goal);
  }

  void plan(const Eigen::Vector3d & goal)
  {
    if (!map_msg_) {
      RCLCPP_WARN(get_logger(), "No map received yet");
      return;
    }
    if (map_msg_->header.frame_id != world_frame_) {
      RCLCPP_WARN(get_logger(), "Map in frame '%s', expected '%s'",
        map_msg_->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }

    Eigen::Vector3d start;
    try {
      const auto tf = tf_buffer_->lookupTransform(world_frame_, body_frame_, tf2::TimePointZero);
      start = {tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "No drone pose: %s", e.what());
      return;
    }

    std::shared_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::binaryMsgToMap(*map_msg_));
    const auto tree = std::dynamic_pointer_cast<octomap::OcTree>(abstract);
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "Map message is not an OcTree");
      return;
    }

    const OctomapMap map(tree);
    const auto t0 = std::chrono::steady_clock::now();
    const auto result = planner_->plan(map, start, goal);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (result.status != AStarPlanner::Status::Success) {
      RCLCPP_WARN(get_logger(), "No path from (%.2f, %.2f, %.2f): %s (%d expansions, %.1f ms)",
        start.x(), start.y(), start.z(), AStarPlanner::to_string(result.status),
        result.expansions, ms);
      clear_paths();
      return;
    }
    RCLCPP_INFO(get_logger(), "Path: %zu points, cost %.2f, %d expansions, %.1f ms",
      result.path.size(), result.cost, result.expansions, ms);
    publish_path(*path_pub_, result.path, {});

    const auto t1 = std::chrono::steady_clock::now();
    const auto waypoints = shortcut(map, result.path, planner_->params());
    const auto t2 = std::chrono::steady_clock::now();
    const auto smoothed = smooth(map, waypoints, smoothing_);
    const auto t3 = std::chrono::steady_clock::now();
    const double shortcut_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    const double smooth_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    if (!smoothed.trajectory) {
      RCLCPP_WARN(get_logger(), "Smoothing failed (%zu waypoints)", waypoints.size());
      publish_path(*smoothed_pub_, {}, {});
      return;
    }
    const auto & traj = *smoothed.trajectory;
    for (const auto & w : waypoints) {
      RCLCPP_DEBUG(get_logger(), "waypoint (%.3f, %.3f, %.3f)", w.x(), w.y(), w.z());
    }
    for (size_t k = 0; k < traj.knots().size(); ++k) {
      const auto & knot = traj.knots()[k];
      RCLCPP_DEBUG(get_logger(), "knot %zu at %.2f s: (%.2f, %.2f, %.2f), %.2f m/s",
        k, traj.knot_time(k), knot.position.x(), knot.position.y(), knot.position.z(),
        knot.velocity.norm());
    }
    for (const auto & turn : traj.turns()) {
      RCLCPP_DEBUG(get_logger(), "turn to %.2f rad from %.2f s to %.2f s",
        turn.yaw, turn.start, turn.start + turn.duration);
    }
    RCLCPP_INFO(get_logger(),
      "Trajectory: %zu waypoints, %d stopped at, %.1f s long (shortcut %.1f ms, smoothing %.1f ms)",
      waypoints.size(), smoothed.stopped_knots, traj.duration(), shortcut_ms, smooth_ms);
    std::vector<Eigen::Vector3d> points;
    std::vector<double> yaws;
    for (double t = 0.0; t < traj.duration(); t += 0.1) {
      const auto sp = traj.sample(t);
      points.push_back(sp.position);
      yaws.push_back(sp.yaw);
    }
    points.push_back(traj.sample(traj.duration()).position);
    yaws.push_back(traj.sample(traj.duration()).yaw);
    publish_path(*smoothed_pub_, points, yaws);

    auto msg = trajectory_server::to_msg(traj);
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    trajectory_pub_->publish(msg);
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
  double goal_altitude_;
  std::unique_ptr<AStarPlanner> planner_;
  SmoothingParams smoothing_;
  octomap_msgs::msg::Octomap::ConstSharedPtr map_msg_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, smoothed_pub_;
  rclcpp::Publisher<drone_interfaces::msg::WaypointTrajectory>::SharedPtr trajectory_pub_;
};

}  // namespace path_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
