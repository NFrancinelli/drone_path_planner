// Fake mocap: forwards the Gazebo ground-truth pose (nav_msgs/Odometry, ENU/FLU) to PX4 as
// external vision (VehicleOdometry, NED/FRD), same as a Vicon/OptiTrack setup.
// EKF2 is configured to fuse only EV position + yaw (GPS/mag/baro off, see sim.launch.py),
// so PX4's estimate tracks ground truth. Its local frame is shifted by a constant though
// (height origin at the drone resting on the ground); the trajectory server compensates.
// Also publishes odom -> base_link TF from these poses. They have exact sim-time stamps,
// which keeps depth clouds aligned with the pose during fast turns.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "px4_state_bridge/px4_time.hpp"
#include "trajectory_server/frames.hpp"

using px4_msgs::msg::VehicleOdometry;

namespace px4_state_bridge
{

class MocapBridgeNode : public rclcpp::Node
{
public:
  MocapBridgeNode()
  : Node("mocap_bridge")
  {
    // Noise reported to EKF2. Don't go much lower: EKF2's consistency checks start rejecting
    // sub-mm innovations and it refuses to arm.
    position_std_ = declare_parameter("position_std", 0.05);        // [m]
    orientation_std_ = declare_parameter("orientation_std", 0.05);  // [rad]
    // Offset from the Gazebo model origin to base_link, in body frame (x500: 0.24 m up).
    // Must match the model SDF.
    const auto offset = declare_parameter("base_link_offset", std::vector<double>{0.0, 0.0, 0.24});
    base_link_offset_ = Eigen::Vector3d(offset.at(0), offset.at(1), offset.at(2));
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));
    if (declare_parameter("publish_tf", true)) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    pub_ = create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "mocap/odometry", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_pose(*msg);});
  }

private:
  void on_pose(const nav_msgs::msg::Odometry & msg)
  {
    if (!px4_time_.ready()) {
      return;
    }
    using trajectory_server::enu_to_ned;
    const auto & o = msg.pose.pose.orientation;
    const Eigen::Quaterniond q_enu(o.w, o.x, o.y, o.z);
    const Eigen::Vector3d p_enu = Eigen::Vector3d(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) +
      q_enu * base_link_offset_;

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg.header.stamp;
      tf.header.frame_id = world_frame_;
      tf.child_frame_id = body_frame_;
      tf.transform.translation.x = p_enu.x();
      tf.transform.translation.y = p_enu.y();
      tf.transform.translation.z = p_enu.z();
      tf.transform.rotation = o;
      tf_broadcaster_->sendTransform(tf);
    }

    const Eigen::Vector3f pos = enu_to_ned(p_enu).cast<float>();
    const Eigen::Quaternionf q = trajectory_server::enu_flu_to_ned_frd(q_enu).cast<float>();

    VehicleOdometry out{};
    out.timestamp_sample = px4_time_.to_px4(rclcpp::Time(msg.header.stamp, RCL_ROS_TIME));
    out.timestamp = out.timestamp_sample;
    out.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    out.position = {pos.x(), pos.y(), pos.z()};
    out.q = {q.w(), q.x(), q.y(), q.z()};
    // Pose only: velocity is left for PX4 to estimate.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    out.velocity = {nan, nan, nan};
    out.angular_velocity = {nan, nan, nan};
    const float pv = static_cast<float>(position_std_ * position_std_);
    const float ov = static_cast<float>(orientation_std_ * orientation_std_);
    out.position_variance = {pv, pv, pv};
    out.orientation_variance = {ov, ov, ov};
    out.velocity_variance = {nan, nan, nan};
    pub_->publish(out);
  }

  double position_std_, orientation_std_;
  Eigen::Vector3d base_link_offset_;
  std::string world_frame_, body_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Px4Time px4_time_{*this};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_;
};

}  // namespace px4_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_state_bridge::MocapBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
