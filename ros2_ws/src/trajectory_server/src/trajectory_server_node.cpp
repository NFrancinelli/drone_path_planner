// Streams trajectory setpoints to PX4 in offboard mode.
//
// For now this runs a hardcoded mission: wait for PX4, pre-stream setpoints, switch to
// offboard, arm, take off, fly a figure-eight, land.
// TODO: take trajectories from the planner over a topic instead of the scripted figure-eight.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "trajectory_server/frames.hpp"
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
    takeoff_altitude_ = declare_parameter("takeoff_altitude", 2.5);
    max_vel_ = declare_parameter("max_vel", 1.0);
    max_acc_ = declare_parameter("max_acc", 1.0);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.8);
    max_yaw_acc_ = declare_parameter("max_yaw_acc", 1.0);
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    prestream_time_ = declare_parameter("prestream_time", 1.0);
    hover_time_ = declare_parameter("hover_time", 2.0);
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
      position_topic, px4_qos, [this](VehicleLocalPosition::ConstSharedPtr msg) {position_ = msg;});

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
  enum class State { WaitForPx4, Prestream, Engaging, Takeoff, Hover, Turn, FigureEight, Landing, Done };

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
        if (px4_ready()) {
          hold(current_position_enu(), current_yaw_enu());
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
          const Eigen::Vector3d start = current_position_enu();
          const Eigen::Vector3d top = start + Eigen::Vector3d(0, 0, takeoff_altitude_);
          follow(std::make_shared<QuinticLine>(start, top, hold_point_.yaw,
            QuinticLine::min_duration(takeoff_altitude_, max_vel_, max_acc_)));
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
        if (t > hover_time_) {
          // Face along the path before moving, so the heading doesn't jump at the start.
          figure_eight_ = std::make_shared<FigureEight>(
            hold_point_.position, hold_point_.yaw, fig8_);
          const double from = hold_point_.yaw, to = figure_eight_->start_yaw();
          follow(std::make_shared<YawTurn>(hold_point_.position, from, to,
            YawTurn::min_duration(from, to, max_yaw_rate_, max_yaw_acc_)));
          transition(State::Turn);
        }
        break;

      case State::Turn:
        if (trajectory_finished()) {
          follow(figure_eight_);
          RCLCPP_INFO(get_logger(), "Figure-eight: %.1f s", trajectory_->duration());
          transition(State::FigureEight);
        }
        break;

      case State::FigureEight:
        if (trajectory_finished()) {
          send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          transition(State::Landing);
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

  Eigen::Vector3d current_position_enu() const
  {
    return ned_to_enu({position_->x, position_->y, position_->z});
  }

  double current_yaw_enu() const {return ned_yaw_to_enu(position_->heading);}

  // --- trajectory handling -------------------------------------------------------------

  void follow(TrajectoryPtr trajectory)
  {
    trajectory_ = std::move(trajectory);
    trajectory_start_ = now();
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

    const Eigen::Vector3f pos = enu_to_ned(sp.position).cast<float>();
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
  double max_yaw_rate_, max_yaw_acc_;
  std::string world_frame_;
  FigureEight::Params fig8_;

  // PX4 inputs
  VehicleStatus::ConstSharedPtr status_;
  VehicleLocalPosition::ConstSharedPtr position_;

  // Mission state
  State state_{State::WaitForPx4};
  rclcpp::Time state_entered_{now()};
  double last_command_time_{-std::numeric_limits<double>::infinity()};
  TrajectoryPtr trajectory_;
  std::shared_ptr<const FigureEight> figure_eight_;
  rclcpp::Time trajectory_start_{now()};
  TrajectoryPoint hold_point_;

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
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
