#pragma once

#include <memory>

#include <Eigen/Core>

namespace trajectory_server
{

// A kinematically consistent sample, in the ENU world frame.
struct TrajectoryPoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

// Time-parameterised trajectory starting at t = 0. Sampling outside [0, duration()]
// clamps to the endpoints, and both endpoints are at rest, so a finished trajectory
// degrades into a hover at its final point.
class Trajectory
{
public:
  virtual ~Trajectory() = default;
  virtual double duration() const = 0;
  virtual TrajectoryPoint sample(double t) const = 0;
};

using TrajectoryPtr = std::shared_ptr<const Trajectory>;

// Rest-to-rest straight segment with quintic (minimum-jerk) time scaling.
class QuinticLine : public Trajectory
{
public:
  QuinticLine(const Eigen::Vector3d & from, const Eigen::Vector3d & to, double yaw, double duration);

  // Shortest duration that respects the given speed and acceleration limits.
  static double min_duration(double distance, double max_vel, double max_acc);

  double duration() const override {return duration_;}
  TrajectoryPoint sample(double t) const override;

private:
  Eigen::Vector3d from_;
  Eigen::Vector3d delta_;
  double yaw_;
  double duration_;
};

// Horizontal figure-eight (lemniscate of Gerono) around `center`, starting and ending at
// rest at the center. Angular rate ramps linearly up and down so the acceleration
// stays bounded at both ends.
//   x = A sin(phi),  y = B sin(2 phi)
class FigureEight : public Trajectory
{
public:
  struct Params
  {
    double half_length{3.0};   // A [m]
    double half_width{1.5};    // B [m]
    double loop_period{14.0};  // seconds per loop at cruise rate
    double ramp_time{4.0};     // seconds to reach / leave cruise rate
    int loops{2};
  };

  FigureEight(const Eigen::Vector3d & center, double yaw, const Params & params);

  double duration() const override {return duration_;}
  TrajectoryPoint sample(double t) const override;

private:
  // Phase and its first two time derivatives.
  void phase(double t, double & phi, double & dphi, double & ddphi) const;

  Eigen::Vector3d center_;
  double yaw_;
  Params p_;
  double omega_;
  double cruise_time_;
  double duration_;
};

}  // namespace trajectory_server
