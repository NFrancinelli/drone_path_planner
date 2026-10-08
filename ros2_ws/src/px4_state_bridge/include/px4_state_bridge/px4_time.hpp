#pragma once

#include <cstdint>

#include <px4_msgs/msg/timesync_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_state_bridge
{

// Converts between px4_msgs timestamps and ROS time.
//
// The XRCE client translates every px4_msgs timestamp into the agent's system clock. On real
// hardware ROS time is that same clock, so no conversion is needed. In lockstep SITL, ROS runs
// on Gazebo's sim time, which is also PX4's own clock; PX4 publishes the offset it uses:
//   timesync estimated_offset = PX4 (= sim) time - system time
// Converting through it is exact. Deriving it from now() instead jitters by up to hundreds of
// ms under load (sim time arrives over /clock), enough to make EKF2 reject mocap data.
class Px4Time
{
public:
  explicit Px4Time(rclcpp::Node & node)
  : sim_time_(node.get_parameter("use_sim_time").as_bool())
  {
    sub_ = node.create_subscription<px4_msgs::msg::TimesyncStatus>(
      "/fmu/out/timesync_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
        offset_us_ = msg->estimated_offset;
        have_offset_ = true;
      });
  }

  // False until the offset is known (sim time only).
  bool ready() const {return !sim_time_ || have_offset_;}

  rclcpp::Time to_ros(uint64_t px4_us) const
  {
    const int64_t us = static_cast<int64_t>(px4_us) + (sim_time_ ? offset_us_ : 0);
    return rclcpp::Time(us * 1000, RCL_ROS_TIME);
  }

  uint64_t to_px4(const rclcpp::Time & ros_time) const
  {
    return static_cast<uint64_t>(ros_time.nanoseconds() / 1000 - (sim_time_ ? offset_us_ : 0));
  }

private:
  bool sim_time_;
  bool have_offset_{false};
  int64_t offset_us_{0};
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr sub_;
};

}  // namespace px4_state_bridge
