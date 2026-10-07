// Republishes PX4's vehicle state in standard ROS form (ENU/FLU):
//   /odom (nav_msgs/Odometry), TF odom -> base_link, /trail (nav_msgs/Path),
//   /drone_model (visualization_msgs/MarkerArray, a simple quadrotor drawn in base_link).
// The TF is also what the mapping stack will use to place depth points in the world.

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "trajectory_server/frames.hpp"

using px4_msgs::msg::VehicleOdometry;
using visualization_msgs::msg::Marker;

namespace px4_state_bridge
{

class Px4StateBridgeNode : public rclcpp::Node
{
public:
  Px4StateBridgeNode()
  : Node("px4_state_bridge")
  {
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));
    trail_spacing_ = declare_parameter("trail_spacing", 0.05);
    trail_max_points_ = static_cast<size_t>(declare_parameter("trail_max_points", 5000));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    trail_pub_ = create_publisher<nav_msgs::msg::Path>("trail", 10);
    model_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("drone_model", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](VehicleOdometry::ConstSharedPtr msg) {on_odometry(*msg);});

    trail_.header.frame_id = world_frame_;
    trail_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        trail_.header.stamp = now();
        trail_pub_->publish(trail_);
      });
    // Republished so RViz instances started later still get it.
    model_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_model();});
  }

private:
  void on_odometry(const VehicleOdometry & msg)
  {
    if (msg.pose_frame != VehicleOdometry::POSE_FRAME_NED || std::isnan(msg.q[0]) ||
      std::isnan(msg.position[0]))
    {
      return;
    }
    using trajectory_server::ned_to_enu;
    const Eigen::Vector3d p = ned_to_enu(Eigen::Vector3f(msg.position.data()).cast<double>());
    const Eigen::Quaterniond q = trajectory_server::ned_frd_to_enu_flu(
      Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]));

    const auto stamp = now();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = body_frame_;
    tf.transform.translation.x = p.x();
    tf.transform.translation.y = p.y();
    tf.transform.translation.z = p.z();
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Odometry odom;
    odom.header = tf.header;
    odom.child_frame_id = body_frame_;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation = tf.transform.rotation;
    // nav_msgs/Odometry twist is expressed in the child (body) frame.
    if (msg.velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED && !std::isnan(msg.velocity[0])) {
      const Eigen::Vector3d v_world =
        ned_to_enu(Eigen::Vector3f(msg.velocity.data()).cast<double>());
      const Eigen::Vector3d v_body = q.conjugate() * v_world;
      odom.twist.twist.linear.x = v_body.x();
      odom.twist.twist.linear.y = v_body.y();
      odom.twist.twist.linear.z = v_body.z();
    }
    if (!std::isnan(msg.angular_velocity[0])) {
      // FRD -> FLU body rates: negate y and z.
      odom.twist.twist.angular.x = msg.angular_velocity[0];
      odom.twist.twist.angular.y = -msg.angular_velocity[1];
      odom.twist.twist.angular.z = -msg.angular_velocity[2];
    }
    odom_pub_->publish(odom);

    if (trail_.poses.empty() || (p - last_trail_point_).norm() > trail_spacing_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = odom.header;
      pose.pose = odom.pose.pose;
      trail_.poses.push_back(pose);
      if (trail_.poses.size() > trail_max_points_) {
        trail_.poses.erase(trail_.poses.begin());
      }
      last_trail_point_ = p;
    }
  }

  // A rough x500: body, two crossed arms, four rotor discs (front pair orange to show heading).
  void publish_model()
  {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    auto make = [&](int type, double x, double y, double z, double sx, double sy, double sz,
        float r, float g, float b, double yaw = 0.0) {
        Marker m;
        m.header.frame_id = body_frame_;
        m.header.stamp = now();
        m.ns = "drone";
        m.id = id++;
        m.type = type;
        m.action = Marker::ADD;
        m.frame_locked = true;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.z = std::sin(yaw / 2.0);
        m.pose.orientation.w = std::cos(yaw / 2.0);
        m.scale.x = sx;
        m.scale.y = sy;
        m.scale.z = sz;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;
        array.markers.push_back(m);
      };

    constexpr double kArm = 0.175;  // motor offset along x and y [m]
    make(Marker::CUBE, 0, 0, 0, 0.18, 0.12, 0.08, 0.2f, 0.2f, 0.2f);
    make(Marker::CUBE, 0, 0, 0.02, 2.0 * M_SQRT2 * kArm, 0.025, 0.02, 0.3f, 0.3f, 0.3f, M_PI_4);
    make(Marker::CUBE, 0, 0, 0.02, 2.0 * M_SQRT2 * kArm, 0.025, 0.02, 0.3f, 0.3f, 0.3f, -M_PI_4);
    for (const double sx : {1.0, -1.0}) {
      for (const double sy : {1.0, -1.0}) {
        const bool front = sx > 0;
        make(Marker::CYLINDER, sx * kArm, sy * kArm, 0.05, 0.25, 0.25, 0.01,
          front ? 1.0f : 0.6f, front ? 0.5f : 0.6f, front ? 0.0f : 0.6f);
      }
    }
    model_pub_->publish(array);
  }

  std::string world_frame_, body_frame_;
  double trail_spacing_;
  size_t trail_max_points_;
  nav_msgs::msg::Path trail_;
  Eigen::Vector3d last_trail_point_{Eigen::Vector3d::Zero()};

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr model_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr trail_timer_, model_timer_;
};

}  // namespace px4_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_state_bridge::Px4StateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
