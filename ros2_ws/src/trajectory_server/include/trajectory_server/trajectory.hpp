#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

namespace trajectory_server
{

// One trajectory sample, ENU world frame. vel/acc are the exact derivatives of position.
struct TrajectoryPoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};       // [rad], ENU: 0 = facing +x (east), counter-clockwise positive
  double yaw_rate{0.0};  // [rad/s]
};

// Trajectory parameterised by time, starting at t = 0. sample() clamps t to [0, duration()]
// and both endpoints are at rest, so sampling past the end just hovers at the last point.
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

// Rotate in place from one heading to another, quintic in angle, taking the short way round.
class YawTurn : public Trajectory
{
public:
  YawTurn(const Eigen::Vector3d & position, double from_yaw, double to_yaw, double duration);

  // Shortest duration that respects the given yaw rate and yaw acceleration limits.
  static double min_duration(double from_yaw, double to_yaw, double max_rate, double max_acc);

  double duration() const override {return duration_;}
  TrajectoryPoint sample(double t) const override;

private:
  Eigen::Vector3d position_;
  double from_yaw_;
  double delta_;
  double duration_;
};

// Horizontal figure-eight (lemniscate of Gerono) around `center`:
//   x = A sin(phi),  y = B sin(2 phi)
// Starts and ends at rest at the center; dphi/dt ramps linearly up/down to keep the
// acceleration bounded. With face_direction, yaw follows the tangent (A cos phi, 2B cos 2phi).
// The tangent is never zero, so yaw is well defined even at rest.
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
    bool face_direction{true};  // false: keep the constructor's fixed yaw
  };

  FigureEight(const Eigen::Vector3d & center, double yaw, const Params & params);

  // Heading of the path at the start (and end); turn to this before flying it.
  double start_yaw() const {return sample(0.0).yaw;}

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

// Quintic polynomial from (p0, v0, a0) to (p1, v1, a1) over [0, T], in 3D.
class QuinticHermite
{
public:
  QuinticHermite(
    const Eigen::Vector3d & p0, const Eigen::Vector3d & v0, const Eigen::Vector3d & a0,
    const Eigen::Vector3d & p1, const Eigen::Vector3d & v1, const Eigen::Vector3d & a1, double T);

  double duration() const {return T_;}
  // u in [0, T], not clamped.
  Eigen::Vector3d position(double u) const;
  Eigen::Vector3d velocity(double u) const;
  Eigen::Vector3d acceleration(double u) const;

private:
  Eigen::Vector3d c_[6];  // c_[i] * u^i
  double T_;
};

// Piecewise quintic through knots. Each segment is the quintic Hermite interpolant of
// position, velocity and acceleration at its two knots, so the trajectory is C2 and passes
// exactly through every knot. First and last knots must be at rest.
// Yaw has its own timeline, independent of the knots: it starts at initial_yaw and changes
// only during the given turns, each a rest-to-rest quintic taking the short way round.
// If the last turn ends after the last knot, the drone holds position until it's done.
class WaypointTrajectory : public Trajectory
{
public:
  struct Knot
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  };

  struct Turn
  {
    double start;     // [s] from the start of the trajectory
    double duration;  // [s]
    double yaw;       // heading reached at start + duration
  };

  // durations[k] is the time from knot k to knot k + 1. Turns sorted and not overlapping.
  WaypointTrajectory(
    std::vector<Knot> knots, std::vector<double> durations, double initial_yaw,
    std::vector<Turn> turns = {});

  double duration() const override {return duration_;}
  TrajectoryPoint sample(double t) const override;

  const std::vector<Knot> & knots() const {return knots_;}
  const std::vector<Turn> & turns() const {return turns_;}
  // Time at which the trajectory passes knot k.
  double knot_time(size_t k) const {return times_.at(k);}

private:
  std::vector<Knot> knots_;
  std::vector<double> times_;  // cumulative, times_[0] = 0
  std::vector<QuinticHermite> segments_;
  double initial_yaw_;
  std::vector<Turn> turns_;
  double duration_;
};

}  // namespace trajectory_server
