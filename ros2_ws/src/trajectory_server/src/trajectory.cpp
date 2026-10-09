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

QuinticHermite::QuinticHermite(
  const Eigen::Vector3d & p0, const Eigen::Vector3d & v0, const Eigen::Vector3d & a0,
  const Eigen::Vector3d & p1, const Eigen::Vector3d & v1, const Eigen::Vector3d & a1, double T)
: T_(T)
{
  const Eigen::Vector3d dp = p1 - p0;
  const double T2 = T * T, T3 = T2 * T;
  c_[0] = p0;
  c_[1] = v0;
  c_[2] = a0 / 2.0;
  c_[3] = (20.0 * dp - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) / (2.0 * T3);
  c_[4] = (-30.0 * dp + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T3 * T);
  c_[5] = (12.0 * dp - 6.0 * (v1 + v0) * T + (a1 - a0) * T2) / (2.0 * T3 * T2);
}

Eigen::Vector3d QuinticHermite::position(double u) const
{
  return c_[0] + u * (c_[1] + u * (c_[2] + u * (c_[3] + u * (c_[4] + u * c_[5]))));
}

Eigen::Vector3d QuinticHermite::velocity(double u) const
{
  return c_[1] + u * (2.0 * c_[2] + u * (3.0 * c_[3] + u * (4.0 * c_[4] + u * 5.0 * c_[5])));
}

Eigen::Vector3d QuinticHermite::acceleration(double u) const
{
  return 2.0 * c_[2] + u * (6.0 * c_[3] + u * (12.0 * c_[4] + u * 20.0 * c_[5]));
}

WaypointTrajectory::WaypointTrajectory(
  std::vector<Knot> knots, std::vector<double> durations, double initial_yaw,
  std::vector<Turn> turns)
: knots_(std::move(knots)), times_{0.0}, initial_yaw_(initial_yaw), turns_(std::move(turns))
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
    segments_.emplace_back(
      a.position, a.velocity, a.acceleration, b.position, b.velocity, b.acceleration, T);
    times_.push_back(times_.back() + T);
  }
  duration_ = times_.back();
  double free_from = 0.0;
  for (const Turn & turn : turns_) {
    if (turn.duration <= 0.0 || turn.start < free_from - 1e-9) {
      throw std::invalid_argument("WaypointTrajectory turns must be sorted, not overlap, last > 0");
    }
    free_from = turn.start + turn.duration;
  }
  duration_ = std::max(duration_, free_from);
}

TrajectoryPoint WaypointTrajectory::sample(double t) const
{
  t = std::clamp(t, 0.0, duration());
  const double tp = std::min(t, times_.back());  // past the last knot: hold it
  // Last segment whose start time is <= tp.
  const size_t k = std::min<size_t>(
    std::upper_bound(times_.begin(), times_.end(), tp) - times_.begin() - 1, segments_.size() - 1);
  const QuinticHermite & s = segments_[k];
  const double u = tp - times_[k];

  TrajectoryPoint p;
  p.position = s.position(u);
  p.velocity = s.velocity(u);
  p.acceleration = s.acceleration(u);

  // Heading before the first turn that hasn't finished yet, then interpolate if t is in it.
  p.yaw = initial_yaw_;
  for (const Turn & turn : turns_) {
    if (t < turn.start) {
      break;
    }
    const double delta = wrap_angle(turn.yaw - p.yaw);
    if (t < turn.start + turn.duration) {
      double ys, dys, ddys;
      quintic((t - turn.start) / turn.duration, ys, dys, ddys);
      p.yaw_rate = delta * dys / turn.duration;
      p.yaw = wrap_angle(p.yaw + delta * ys);
      break;
    }
    p.yaw = wrap_angle(turn.yaw);
  }
  return p;
}

}  // namespace trajectory_server
