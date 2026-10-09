// Streams trajectory setpoints to PX4 in offboard mode.
//
// Mission: wait for PX4, pre-stream setpoints, switch to offboard, arm, take off, then
// either fly a figure-eight and land, or (figure_eight.enabled = false) hover and fly the
// trajectories the planner sends on `trajectory`, holding position between them.
// A trajectory is only accepted while hovering, and must start where the drone holds;
// the drone first turns in place to face along it.
//
// Everything here is in the ROS world frame (odom, where the map lives). PX4's local frame
// can be shifted from it: EKF2 puts its origin where its estimate started, e.g. its height
// origin at the drone resting on the ground (~0.2 m below odom with mocap). The offset is
// measured from TF before takeoff, once steady while the drone is still, and kept up to
// date across PX4's estimator resets, so setpoints and the map agree.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <drone_interfaces/msg/waypoint_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "trajectory_server/frames.hpp"
#include "trajectory_server/msg_conversions.hpp"
#include "trajectory_server/trajectory.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace trajectory_server
{

class TrajectoryServerNode : public rclcpp::Node
{
public:
  TrajectoryServerNode()
  : Node("trajectory_server")
  {
    rate_hz_ = declare_parameter("rate_hz", 50.0);
    // World-frame z, like the planner's goal_altitude, so the first plan stays level.
    takeoff_altitude_ = declare_parameter("takeoff_altitude", 2.5);
    max_vel_ = declare_parameter("max_vel", 1.0);
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.8);
    max_yaw_acc_ = declare_parameter("max_yaw_acc", 1.0);
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));
    prestream_time_ = declare_parameter("prestream_time", 1.0);
    hover_time_ = declare_parameter("hover_time", 2.0);
    // false: take off, hold position and fly the planner's trajectories.
    fly_figure_eight_ = declare_parameter("figure_eight.enabled", true);
    // Planner trajectories must start this close to the hold point [m].
    max_start_error_ = declare_parameter("max_start_error", 0.3);
    fig8_.half_length = declare_parameter("figure_eight.half_length", fig8_.half_length);
    fig8_.half_width = declare_parameter("figure_eight.half_width", fig8_.half_width);
    fig8_.loop_period = declare_parameter("figure_eight.loop_period", fig8_.loop_period);
    fig8_.ramp_time = declare_parameter("figure_eight.ramp_time", fig8_.ramp_time);
    fig8_.loops = static_cast<int>(declare_parameter("figure_eight.loops", 2));
    fig8_.face_direction = declare_parameter("figure_eight.face_direction", true);

    const auto status_topic =
      declare_parameter("status_topic", std::string("/fmu/out/vehicle_status_v1"));
    const auto position_topic =
      declare_parameter("local_position_topic", std::string("/fmu/out/vehicle_local_position_v1"));

    // PX4 publishes best-effort; a reliable subscriber would silently get nothing.
    const auto px4_qos = rclcpp::SensorDataQoS();
    status_sub_ = create_subscription<VehicleStatus>(
      status_topic, px4_qos, [this](VehicleStatus::ConstSharedPtr msg) {status_ = msg;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      position_topic, px4_qos, [this](VehicleLocalPosition::ConstSharedPtr msg) {
        position_ = msg;
        track_resets(*msg);
      });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    trajectory_sub_ = create_subscription<drone_interfaces::msg::WaypointTrajectory>(
      "trajectory", 1,
      [this](drone_interfaces::msg::WaypointTrajectory::ConstSharedPtr msg) {on_trajectory(*msg);});

    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    // Latched so RViz shows the active trajectory even if it subscribes later.
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "planned_trajectory", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Waiting for PX4 on %s and %s", status_topic.c_str(),
      position_topic.c_str());
  }

private:
  enum class State
  {
    WaitForPx4, Prestream, Engaging, Takeoff, Hover, Turn, FigureEight, Follow, Landing, Done
  };

  static const char * name(State s)
  {
    switch (s) {
      case State::WaitForPx4: return "WAIT_FOR_PX4";
      case State::Prestream: return "PRESTREAM";
      case State::Engaging: return "ENGAGING";
      case State::Takeoff: return "TAKEOFF";
      case State::Hover: return "HOVER";
      case State::Turn: return "TURN";
      case State::FigureEight: return "FIGURE_EIGHT";
      case State::Follow: return "FOLLOW";
      case State::Landing: return "LANDING";
      case State::Done: return "DONE";
    }
    return "?";
  }

  void tick()
  {
    const double t = elapsed();

    switch (state_) {
      case State::WaitForPx4:
        if (px4_ready() && measure_px4_offset()) {
          hold(current_position(), current_yaw_enu());
          transition(State::Prestream);
        }
        break;

      case State::Prestream:
        // PX4 rejects the switch to offboard unless setpoints are already flowing.
        if (t > prestream_time_) {
          transition(State::Engaging);
        }
        break;

      case State::Engaging:
        if (offboard() && armed()) {
          const Eigen::Vector3d start = current_position();
          const Eigen::Vector3d top(start.x(), start.y(), takeoff_altitude_);
          follow(std::make_shared<QuinticLine>(start, top, hold_point_.yaw,
            QuinticLine::min_duration((top - start).norm(), max_vel_, max_acc_)), now());
          transition(State::Takeoff);
        } else if (t - last_command_time_ > 1.0) {
          // Re-send until PX4 accepts; commands are fire-and-forget over DDS.
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);  // custom, offboard
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          last_command_time_ = t;
        } else if (t > 15.0) {
          RCLCPP_ERROR(get_logger(), "PX4 did not enter offboard + armed within 15 s, aborting");
          transition(State::Done);
        }
        break;

      case State::Takeoff:
        if (trajectory_finished()) {
          transition(State::Hover);
        }
        break;

      case State::Hover:
        if (fly_figure_eight_ && t > hover_time_) {
          const auto fig8 =
            std::make_shared<FigureEight>(hold_point_.position, hold_point_.yaw, fig8_);
          turn_then_follow(fig8, fig8->start_yaw(), now());
        }
        break;

      case State::Turn:
        if (trajectory_finished()) {
          follow(pending_, std::max(now(), pending_start_));
          RCLCPP_INFO(get_logger(), "Flying trajectory: %.1f s", trajectory_->duration());
          transition(fly_figure_eight_ ? State::FigureEight : State::Follow);
        }
        break;

      case State::FigureEight:
        if (trajectory_finished()) {
          send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          transition(State::Landing);
        }
        break;

      case State::Follow:
        if (trajectory_finished()) {
          transition(State::Hover);
        }
        break;

      case State::Landing:
        // PX4 disarms automatically once the land detector triggers.
        if (status_ && status_->arming_state != VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Landed and disarmed. Mission complete.");
          transition(State::Done);
        }
        break;

      case State::Done:
        break;
    }

    // Offboard requires a continuous stream; stop once PX4's land mode has taken over.
    if (state_ != State::WaitForPx4 && state_ != State::Landing && state_ != State::Done) {
      publish_setpoint(current_setpoint());
    }
  }

  // --- PX4 state ---------------------------------------------------------------------

  // Offboard only needs a local position. Don't wait for pre_flight_checks_pass: it means
  // "can arm in the current mode", and PX4 boots into Hold, which needs a global (GPS)
  // position we don't have with mocap. Real arming refusals surface as the ENGAGING timeout.
  bool px4_ready() const
  {
    return status_ && position_ && position_->xy_valid && position_->z_valid;
  }

  bool offboard() const
  {
    return status_ && status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool armed() const
  {
    return status_ && status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  // Drone position in the world frame.
  Eigen::Vector3d current_position() const
  {
    return ned_to_enu({position_->x, position_->y, position_->z}) + px4_offset_;
  }

  // PX4 local frame -> world frame offset, from the TF pose and PX4's estimate of the same
  // pose. Only valid while the drone is still (on the ground, before arming), when the
  // latest of each describe the same instant. EKF2 reports a valid position before it
  // has fused the first vision heights, so wait until the offset has been steady for a
  // while. Returns true once it has.
  bool measure_px4_offset()
  {
    constexpr double kSteadyTime = 2.0;       // [s]
    constexpr double kSteadyTolerance = 0.01;  // [m]
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(world_frame_, body_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for TF: %s", e.what());
      return false;
    }
    const Eigen::Vector3d world(
      tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    const Eigen::Vector3d offset = world - ned_to_enu({position_->x, position_->y, position_->z});
    if (!steady_since_ || (offset - steady_offset_).norm() > kSteadyTolerance) {
      steady_offset_ = offset;
      steady_since_ = now();
    }
    if ((now() - *steady_since_).seconds() < kSteadyTime) {
      return false;
    }
    px4_offset_ = offset;
    xy_resets_ = position_->xy_reset_counter;
    z_resets_ = position_->z_reset_counter;
    have_offset_ = true;
    RCLCPP_INFO(get_logger(), "PX4 local frame -> %s offset: (%.3f, %.3f, %.3f) m",
      world_frame_.c_str(), px4_offset_.x(), px4_offset_.y(), px4_offset_.z());
    return true;
  }

  // An estimator reset jumps PX4's position by delta_xy / delta_z while the drone stays
  // put, so the offset moves the other way. Assumes at most one reset between messages.
  void track_resets(const VehicleLocalPosition & msg)
  {
    if (!have_offset_) {
      return;
    }
    if (msg.xy_reset_counter != xy_resets_) {
      xy_resets_ = msg.xy_reset_counter;
      px4_offset_.x() -= msg.delta_xy[1];  // NED (north, east) -> ENU (east, north)
      px4_offset_.y() -= msg.delta_xy[0];
      RCLCPP_WARN(get_logger(), "PX4 xy reset by (%.3f, %.3f) m NED, offset adjusted",
        msg.delta_xy[0], msg.delta_xy[1]);
    }
    if (msg.z_reset_counter != z_resets_) {
      z_resets_ = msg.z_reset_counter;
      px4_offset_.z() += msg.delta_z;  // NED down -> ENU up
      RCLCPP_WARN(get_logger(), "PX4 z reset by %.3f m NED, offset adjusted", msg.delta_z);
    }
  }

  double current_yaw_enu() const {return ned_yaw_to_enu(position_->heading);}

  // --- trajectory handling -------------------------------------------------------------

  void on_trajectory(const drone_interfaces::msg::WaypointTrajectory & msg)
  {
    if (fly_figure_eight_ || state_ != State::Hover) {
      RCLCPP_WARN(get_logger(), "Trajectory ignored: only accepted while hovering with "
        "figure_eight.enabled = false (state %s)", name(state_));
      return;
    }
    if (msg.header.frame_id != world_frame_) {
      RCLCPP_WARN(get_logger(), "Trajectory in frame '%s', expected '%s'; ignored",
        msg.header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    std::shared_ptr<const WaypointTrajectory> trajectory;
    try {
      trajectory = from_msg(msg);
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN(get_logger(), "Invalid trajectory ignored: %s", e.what());
      return;
    }
    const TrajectoryPoint start = trajectory->sample(0.0);
    const double error = (start.position - hold_point_.position).norm();
    if (error > max_start_error_) {
      RCLCPP_WARN(get_logger(), "Trajectory starts %.2f m from the hold point (max %.2f); ignored",
        error, max_start_error_);
      return;
    }
    turn_then_follow(
      trajectory, start.yaw, rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()));
  }

  // Face along the trajectory before moving, so the heading doesn't jump at the start.
  // The trajectory starts once the turn is done, but not before `start`.
  void turn_then_follow(TrajectoryPtr trajectory, double start_yaw, const rclcpp::Time & start)
  {
    pending_ = std::move(trajectory);
    pending_start_ = start;
    const double from = hold_point_.yaw;
    const double duration = YawTurn::min_duration(from, start_yaw, max_yaw_rate_, max_yaw_acc_);
    follow(std::make_shared<YawTurn>(hold_point_.position, from, start_yaw, duration), now());
    transition(State::Turn);
  }

  // Sampling holds the start point until `start`, so a start in the future just waits.
  void follow(TrajectoryPtr trajectory, const rclcpp::Time & start)
  {
    trajectory_ = std::move(trajectory);
    trajectory_start_ = start;
    publish_path(*trajectory_);
  }

  void publish_path(const Trajectory & trajectory)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = world_frame_;
    const int n = std::max(2, static_cast<int>(trajectory.duration() / 0.1) + 1);
    for (int i = 0; i < n; ++i) {
      const auto sp = trajectory.sample(trajectory.duration() * i / (n - 1));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = sp.position.x();
      pose.pose.position.y = sp.position.y();
      pose.pose.position.z = sp.position.z();
      pose.pose.orientation.z = std::sin(sp.yaw / 2.0);
      pose.pose.orientation.w = std::cos(sp.yaw / 2.0);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void hold(const Eigen::Vector3d & position, double yaw)
  {
    trajectory_.reset();
    hold_point_ = TrajectoryPoint{};
    hold_point_.position = position;
    hold_point_.yaw = yaw;
  }

  bool trajectory_finished() const
  {
    return !trajectory_ || (now() - trajectory_start_).seconds() >= trajectory_->duration();
  }

  TrajectoryPoint current_setpoint()
  {
    if (!trajectory_) {
      return hold_point_;
    }
    const double t = (now() - trajectory_start_).seconds();
    TrajectoryPoint sp = trajectory_->sample(t);
    if (t >= trajectory_->duration()) {
      // Finished trajectories become a hold at their end point.
      hold(sp.position, sp.yaw);
      return hold_point_;
    }
    hold_point_ = sp;  // remember where we are, so a later hold starts from here
    return sp;
  }

  // --- PX4 output ----------------------------------------------------------------------

  // PX4 converts incoming timestamps from the XRCE agent's system clock to its own, so these
  // must be system time even when the node runs on simulation time.
  uint64_t timestamp_us() {return system_clock_.now().nanoseconds() / 1000;}

  void publish_setpoint(const TrajectoryPoint & sp)
  {
    OffboardControlMode mode{};
    mode.timestamp = timestamp_us();
    mode.position = true;
    mode.velocity = true;
    mode.acceleration = true;
    mode_pub_->publish(mode);

    const Eigen::Vector3f pos = enu_to_ned(sp.position - px4_offset_).cast<float>();
    const Eigen::Vector3f vel = enu_to_ned(sp.velocity).cast<float>();
    const Eigen::Vector3f acc = enu_to_ned(sp.acceleration).cast<float>();
    TrajectorySetpoint msg{};
    msg.timestamp = mode.timestamp;
    msg.position = {pos.x(), pos.y(), pos.z()};
    msg.velocity = {vel.x(), vel.y(), vel.z()};
    msg.acceleration = {acc.x(), acc.y(), acc.z()};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.jerk = {nan, nan, nan};
    msg.yaw = static_cast<float>(enu_yaw_to_ned(sp.yaw));
    msg.yawspeed = static_cast<float>(-sp.yaw_rate);  // ENU counter-clockwise -> NED clockwise
    setpoint_pub_->publish(msg);
  }

  void send_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  // --- state machine helpers -----------------------------------------------------------

  void transition(State next)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s", name(state_), name(next));
    state_ = next;
    state_entered_ = now();
    last_command_time_ = -std::numeric_limits<double>::infinity();
  }

  double elapsed() const {return (now() - state_entered_).seconds();}

  // Parameters
  double rate_hz_, takeoff_altitude_, max_vel_, max_acc_, prestream_time_, hover_time_;
  double max_yaw_rate_, max_yaw_acc_, max_start_error_;
  bool fly_figure_eight_;
  std::string world_frame_, body_frame_;
  FigureEight::Params fig8_;

  // PX4 inputs
  VehicleStatus::ConstSharedPtr status_;
  VehicleLocalPosition::ConstSharedPtr position_;
  Eigen::Vector3d px4_offset_{Eigen::Vector3d::Zero()};  // world = PX4 local (ENU) + offset
  bool have_offset_{false};
  std::optional<rclcpp::Time> steady_since_;  // offset within tolerance of steady_offset_
  Eigen::Vector3d steady_offset_{Eigen::Vector3d::Zero()};
  uint8_t xy_resets_{0}, z_resets_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Mission state
  State state_{State::WaitForPx4};
  rclcpp::Time state_entered_{now()};
  double last_command_time_{-std::numeric_limits<double>::infinity()};
  TrajectoryPtr trajectory_;
  rclcpp::Time trajectory_start_{now()};
  TrajectoryPtr pending_;  // flown after the turn in State::Turn
  rclcpp::Time pending_start_{now()};
  TrajectoryPoint hold_point_;

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<drone_interfaces::msg::WaypointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
};

}  // namespace trajectory_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trajectory_server::TrajectoryServerNode>());
  rclcpp::shutdown();
  return 0;
}
