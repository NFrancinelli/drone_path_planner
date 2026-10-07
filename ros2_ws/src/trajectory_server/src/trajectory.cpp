#include "trajectory_server/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajectory_server
{

namespace
{
// Peak values of s'(tau) and s''(tau) for s = 10 tau^3 - 15 tau^4 + 6 tau^5.
constexpr double kQuinticPeakVel = 1.875;
constexpr double kQuinticPeakAcc = 5.773502691896258;  // 10 / sqrt(3)
}  // namespace

QuinticLine::QuinticLine(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to, double yaw, double duration)
: from_(from), delta_(to - from), yaw_(yaw), duration_(duration)
{
  if (duration_ <= 0.0) {
    throw std::invalid_argument("QuinticLine duration must be positive");
  }
}

double QuinticLine::min_duration(double distance, double max_vel, double max_acc)
{
  const double t_vel = kQuinticPeakVel * distance / max_vel;
  const double t_acc = std::sqrt(kQuinticPeakAcc * distance / max_acc);
  return std::max({t_vel, t_acc, 1e-3});
}

TrajectoryPoint QuinticLine::sample(double t) const
{
  const double tau = std::clamp(t / duration_, 0.0, 1.0);
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double s = 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2;
  const double ds = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau3 * tau;
  const double dds = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;

  TrajectoryPoint p;
  p.position = from_ + delta_ * s;
  p.velocity = delta_ * (ds / duration_);
  p.acceleration = delta_ * (dds / (duration_ * duration_));
  p.yaw = yaw_;
  return p;
}

FigureEight::FigureEight(const Eigen::Vector3d & center, double yaw, const Params & params)
: center_(center), yaw_(yaw), p_(params)
{
  if (p_.loops < 1 || p_.loop_period <= 0.0 || p_.ramp_time <= 0.0) {
    throw std::invalid_argument("FigureEight needs loops >= 1 and positive times");
  }
  omega_ = 2.0 * M_PI / p_.loop_period;
  // Each ramp covers omega * ramp_time / 2 of phase; the cruise covers the rest.
  const double total_phase = 2.0 * M_PI * p_.loops;
  const double ramp_phase = omega_ * p_.ramp_time / 2.0;
  if (2.0 * ramp_phase > total_phase) {
    throw std::invalid_argument("FigureEight ramp_time too long for the number of loops");
  }
  cruise_time_ = (total_phase - 2.0 * ramp_phase) / omega_;
  duration_ = 2.0 * p_.ramp_time + cruise_time_;
}

void FigureEight::phase(double t, double & phi, double & dphi, double & ddphi) const
{
  const double tr = p_.ramp_time;
  t = std::clamp(t, 0.0, duration_);
  if (t < tr) {
    phi = omega_ * t * t / (2.0 * tr);
    dphi = omega_ * t / tr;
    ddphi = omega_ / tr;
  } else if (t < tr + cruise_time_) {
    phi = omega_ * tr / 2.0 + omega_ * (t - tr);
    dphi = omega_;
    ddphi = 0.0;
  } else {
    const double u = duration_ - t;
    phi = 2.0 * M_PI * p_.loops - omega_ * u * u / (2.0 * tr);
    dphi = omega_ * u / tr;
    ddphi = -omega_ / tr;
  }
}

TrajectoryPoint FigureEight::sample(double t) const
{
  double phi, dphi, ddphi;
  phase(t, phi, dphi, ddphi);
  if (t >= duration_) {
    ddphi = 0.0;  // at rest after the end
  }

  const double a = p_.half_length;
  const double b = p_.half_width;
  const double s1 = std::sin(phi), c1 = std::cos(phi);
  const double s2 = std::sin(2.0 * phi), c2 = std::cos(2.0 * phi);

  TrajectoryPoint p;
  p.position = center_ + Eigen::Vector3d(a * s1, b * s2, 0.0);
  p.velocity = Eigen::Vector3d(a * c1 * dphi, 2.0 * b * c2 * dphi, 0.0);
  p.acceleration = Eigen::Vector3d(
    -a * s1 * dphi * dphi + a * c1 * ddphi,
    -4.0 * b * s2 * dphi * dphi + 2.0 * b * c2 * ddphi,
    0.0);
  p.yaw = yaw_;
  return p;
}

}  // namespace trajectory_server
