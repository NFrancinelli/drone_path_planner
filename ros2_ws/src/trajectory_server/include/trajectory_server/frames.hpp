#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace trajectory_server
{

// ENU <-> NED conversions. ROS side: ENU, yaw from +x (East), CCW.
// PX4 side: NED, yaw from North, CW. Keep all frame conversions in this file.

inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d & v)
{
  return {v.y(), v.x(), -v.z()};
}

// The swap is its own inverse.
inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d & v)
{
  return {v.y(), v.x(), -v.z()};
}

inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline double enu_yaw_to_ned(double yaw_enu)
{
  return wrap_angle(M_PI_2 - yaw_enu);
}

inline double ned_yaw_to_enu(double yaw_ned)
{
  return wrap_angle(M_PI_2 - yaw_ned);
}

// PX4 attitude (body FRD -> world NED) to ROS attitude (body FLU -> world ENU):
//   q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu
// Both fixed rotations are 180 deg turns, so each is its own inverse.
inline Eigen::Quaterniond ned_frd_to_enu_flu(const Eigen::Quaterniond & q_ned_frd)
{
  const Eigen::Quaterniond q_enu_ned(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);  // about (x+y)/sqrt2
  const Eigen::Quaterniond q_frd_flu(0.0, 1.0, 0.0, 0.0);              // about x
  return (q_enu_ned * q_ned_frd * q_frd_flu).normalized();
}

// ROS attitude (body FLU -> world ENU) to PX4 attitude (body FRD -> world NED).
// Same two fixed rotations, applied on the other sides.
inline Eigen::Quaterniond enu_flu_to_ned_frd(const Eigen::Quaterniond & q_enu_flu)
{
  const Eigen::Quaterniond q_ned_enu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  const Eigen::Quaterniond q_flu_frd(0.0, 1.0, 0.0, 0.0);
  return (q_ned_enu * q_enu_flu * q_flu_frd).normalized();
}

}  // namespace trajectory_server
