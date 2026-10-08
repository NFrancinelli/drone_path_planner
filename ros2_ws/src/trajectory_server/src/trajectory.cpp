#include "trajectory_server/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "trajectory_server/frames.hpp"

namespace trajectory_server
{

namespace
{
// Peak values of s'(tau) and s''(tau) for s = 10 tau^3 - 15 tau^4 + 6 tau^5.
constexpr double kQuinticPeakVel = 1.875;
constexpr double kQuinticPeakAcc = 5.773502691896258;  // 10 / sqrt(3)

// Minimum-jerk profile s(tau) on [0, 1] and its first two derivatives.
void quintic(double tau, double & s, double & ds, double & dds)
{
  tau = std::clamp(tau, 0.0, 1.0);
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  s = 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2;
  ds = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau3 * tau;
  dds = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;
}

double min_quintic_duration(double distance, double max_vel, double max_acc)
{
  const double t_vel = kQuinticPeakVel * distance / max_vel;
  const double t_acc = std::sqrt(kQuinticPeakAcc * distance / max_acc);
  return std::max({t_vel, t_acc, 1e-3});
}
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
  return min_quintic_duration(distance, max_vel, max_acc);
}

TrajectoryPoint QuinticLine::sample(double t) const
{
  double s, ds, dds;
  quintic(t / duration_, s, ds, dds);

  TrajectoryPoint p;
  p.position = from_ + delta_ * s;
  p.velocity = delta_ * (ds / duration_);
  p.acceleration = delta_ * (dds / (duration_ * duration_));
  p.yaw = yaw_;
  return p;
}

YawTurn::YawTurn(
  const Eigen::Vector3d & position, double from_yaw, double to_yaw, double duration)
: position_(position), from_yaw_(from_yaw), delta_(wrap_angle(to_yaw - from_yaw)), duration_(duration)
{
  if (duration_ <= 0.0) {
    throw std::invalid_argument("YawTurn duration must be positive");
  }
}

double YawTurn::min_duration(double from_yaw, double to_yaw, double max_rate, double max_acc)
{
  return min_quintic_duration(std::abs(wrap_angle(to_yaw - from_yaw)), max_rate, max_acc);
}

TrajectoryPoint YawTurn::sample(double t) const
{
  double s, ds, dds;
  quintic(t / duration_, s, ds, dds);

  TrajectoryPoint p;
  p.position = position_;
  p.yaw = wrap_angle(from_yaw_ + delta_ * s);
  p.yaw_rate = delta_ * ds / duration_;
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
  if (p_.face_direction) {
    // Heading of the tangent T = (tx, ty) = (A cos phi, 2B cos 2phi), and its rate:
    // d(atan2(ty, tx))/dphi = (tx ty' - ty tx') / |T|^2.
    const double tx = a * c1, ty = 2.0 * b * c2;
    const double dtx = -a * s1, dty = -4.0 * b * s2;
    p.yaw = std::atan2(ty, tx);
    p.yaw_rate = (tx * dty - ty * dtx) / (tx * tx + ty * ty) * dphi;
  } else {
    p.yaw = yaw_;
  }
  return p;
}

WaypointTrajectory::WaypointTrajectory(std::vector<Knot> knots, std::vector<double> durations)
: knots_(std::move(knots)), times_{0.0}
{
  if (knots_.size() < 2 || durations.size() != knots_.size() - 1) {
    throw std::invalid_argument("WaypointTrajectory needs n >= 2 knots and n - 1 durations");
  }
  for (const Knot * k : {&knots_.front(), &knots_.back()}) {
    if (!k->velocity.isZero() || !k->acceleration.isZero()) {
      throw std::invalid_argument("WaypointTrajectory must start and end at rest");
    }
  }
  for (size_t i = 0; i < durations.size(); ++i) {
    const double T = durations[i];
    if (T <= 0.0) {
      throw std::invalid_argument("WaypointTrajectory durations must be positive");
    }
    const Knot & a = knots_[i];
    const Knot & b = knots_[i + 1];
    const Eigen::Vector3d dp = b.position - a.position;
    const double T2 = T * T, T3 = T2 * T;
    // Quintic Hermite: matches position, velocity and acceleration at both ends.
    Segment s;
    s.c[0] = a.position;
    s.c[1] = a.velocity;
    s.c[2] = a.acceleration / 2.0;
    s.c[3] = (20.0 * dp - (8.0 * b.velocity + 12.0 * a.velocity) * T -
      (3.0 * a.acceleration - b.acceleration) * T2) / (2.0 * T3);
    s.c[4] = (-30.0 * dp + (14.0 * b.velocity + 16.0 * a.velocity) * T +
      (3.0 * a.acceleration - 2.0 * b.acceleration) * T2) / (2.0 * T3 * T);
    s.c[5] = (12.0 * dp - 6.0 * (b.velocity + a.velocity) * T +
      (b.acceleration - a.acceleration) * T2) / (2.0 * T3 * T2);
    s.yaw0 = a.yaw;
    s.dyaw = wrap_angle(b.yaw - a.yaw);
    segments_.push_back(s);
    times_.push_back(times_.back() + T);
  }
}

TrajectoryPoint WaypointTrajectory::sample(double t) const
{
  t = std::clamp(t, 0.0, duration());
  // Last segment whose start time is <= t.
  const size_t k = std::min<size_t>(
    std::upper_bound(times_.begin(), times_.end(), t) - times_.begin() - 1, segments_.size() - 1);
  const Segment & s = segments_[k];
  const double T = times_[k + 1] - times_[k];
  const double u = t - times_[k];

  TrajectoryPoint p;
  p.position = s.c[0] + u * (s.c[1] + u * (s.c[2] + u * (s.c[3] + u * (s.c[4] + u * s.c[5]))));
  p.velocity = s.c[1] + u * (2.0 * s.c[2] + u * (3.0 * s.c[3] + u * (4.0 * s.c[4] + u * 5.0 * s.c[5])));
  p.acceleration = 2.0 * s.c[2] + u * (6.0 * s.c[3] + u * (12.0 * s.c[4] + u * 20.0 * s.c[5]));

  double ys, dys, ddys;
  quintic(u / T, ys, dys, ddys);
  p.yaw = wrap_angle(s.yaw0 + s.dyaw * ys);
  p.yaw_rate = s.dyaw * dys / T;
  return p;
}

}  // namespace trajectory_server
