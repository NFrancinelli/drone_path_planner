#include <cmath>

#include <gtest/gtest.h>

#include "trajectory_server/frames.hpp"
#include "trajectory_server/trajectory.hpp"

using trajectory_server::FigureEight;
using trajectory_server::QuinticLine;
using trajectory_server::Trajectory;

namespace
{

// Velocity and acceleration must be the true derivatives of position, or PX4's
// feed-forward fights its own position loop.
void expect_consistent_derivatives(const Trajectory & traj)
{
  const double h = 1e-5;
  for (double t = h; t < traj.duration() - h; t += traj.duration() / 197.0) {
    const auto p = traj.sample(t);
    const auto before = traj.sample(t - h);
    const auto after = traj.sample(t + h);
    const Eigen::Vector3d num_vel = (after.position - before.position) / (2 * h);
    const Eigen::Vector3d num_acc = (after.velocity - before.velocity) / (2 * h);
    EXPECT_LT((num_vel - p.velocity).norm(), 1e-4) << "t = " << t;
    EXPECT_LT((num_acc - p.acceleration).norm(), 1e-3) << "t = " << t;
  }
}

void expect_at_rest(const Trajectory & traj, double t)
{
  const auto p = traj.sample(t);
  EXPECT_LT(p.velocity.norm(), 1e-9) << "t = " << t;
}

}  // namespace

TEST(Frames, EnuNedRoundTrip)
{
  const Eigen::Vector3d enu(1.0, 2.0, 3.0);
  const Eigen::Vector3d ned = trajectory_server::enu_to_ned(enu);
  EXPECT_EQ(ned, Eigen::Vector3d(2.0, 1.0, -3.0));  // north = y, east = x, down = -up
  EXPECT_EQ(trajectory_server::ned_to_enu(ned), enu);
}

TEST(Frames, YawConventions)
{
  // Facing east: 0 in ENU, +pi/2 in NED. Facing north: pi/2 in ENU, 0 in NED.
  EXPECT_NEAR(trajectory_server::enu_yaw_to_ned(0.0), M_PI_2, 1e-12);
  EXPECT_NEAR(trajectory_server::enu_yaw_to_ned(M_PI_2), 0.0, 1e-12);
  for (double yaw = -3.0; yaw < 3.0; yaw += 0.37) {
    EXPECT_NEAR(trajectory_server::ned_yaw_to_enu(trajectory_server::enu_yaw_to_ned(yaw)),
      yaw, 1e-12);
  }
}

TEST(Frames, AttitudeMatchesPositionAndYawConventions)
{
  for (double yaw_ned = -3.0; yaw_ned < 3.0; yaw_ned += 0.41) {
    for (double pitch = -0.6; pitch < 0.6; pitch += 0.3) {
      const Eigen::Quaterniond q_ned_frd =
        Eigen::AngleAxisd(yaw_ned, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
      const Eigen::Quaterniond q_enu_flu = trajectory_server::ned_frd_to_enu_flu(q_ned_frd);

      // The body's forward axis must point the same way in the world, whichever convention.
      const Eigen::Vector3d forward_ned = q_ned_frd * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d forward_enu = q_enu_flu * Eigen::Vector3d::UnitX();
      EXPECT_LT((trajectory_server::ned_to_enu(forward_ned) - forward_enu).norm(), 1e-12);
      // Body "down" (FRD +z) is body "up" (FLU +z) negated.
      const Eigen::Vector3d down_ned = q_ned_frd * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d up_enu = q_enu_flu * Eigen::Vector3d::UnitZ();
      EXPECT_LT((trajectory_server::ned_to_enu(down_ned) + up_enu).norm(), 1e-12);
    }
    // Level attitude: the yaw extracted in ENU matches the scalar yaw conversion.
    const Eigen::Quaterniond level(Eigen::AngleAxisd(yaw_ned, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d fwd = trajectory_server::ned_frd_to_enu_flu(level) * Eigen::Vector3d::UnitX();
    EXPECT_NEAR(trajectory_server::wrap_angle(std::atan2(fwd.y(), fwd.x()) -
      trajectory_server::ned_yaw_to_enu(yaw_ned)), 0.0, 1e-12);
  }
}

TEST(QuinticLine, EndpointsAndDerivatives)
{
  const Eigen::Vector3d from(0, 0, 0), to(1, 2, 3);
  const QuinticLine line(from, to, 0.3, 4.0);
  EXPECT_LT((line.sample(0.0).position - from).norm(), 1e-12);
  EXPECT_LT((line.sample(4.0).position - to).norm(), 1e-12);
  EXPECT_LT((line.sample(100.0).position - to).norm(), 1e-12);  // clamps after the end
  expect_at_rest(line, 0.0);
  expect_at_rest(line, 4.0);
  expect_consistent_derivatives(line);
}

TEST(QuinticLine, MinDurationRespectsLimits)
{
  const double distance = 5.0, max_vel = 1.0, max_acc = 0.5;
  const double duration = QuinticLine::min_duration(distance, max_vel, max_acc);
  const QuinticLine line({0, 0, 0}, {distance, 0, 0}, 0.0, duration);
  for (double t = 0; t <= duration; t += duration / 500.0) {
    const auto p = line.sample(t);
    EXPECT_LE(p.velocity.norm(), max_vel + 1e-9);
    EXPECT_LE(p.acceleration.norm(), max_acc + 1e-6);
  }
}

TEST(FigureEight, StartsAndEndsAtRestAtCenter)
{
  const Eigen::Vector3d center(1, -2, 2.5);
  const FigureEight fig(center, 0.0, FigureEight::Params{});
  EXPECT_LT((fig.sample(0.0).position - center).norm(), 1e-12);
  EXPECT_LT((fig.sample(fig.duration()).position - center).norm(), 1e-9);
  expect_at_rest(fig, 0.0);
  expect_at_rest(fig, fig.duration());
  expect_consistent_derivatives(fig);
}

TEST(FigureEight, StaysAtAltitudeAndInsideBounds)
{
  FigureEight::Params params;
  params.half_length = 3.0;
  params.half_width = 1.5;
  const FigureEight fig({0, 0, 2.0}, 0.0, params);
  for (double t = 0; t <= fig.duration(); t += 0.05) {
    const auto p = fig.sample(t);
    EXPECT_DOUBLE_EQ(p.position.z(), 2.0);
    EXPECT_LE(std::abs(p.position.x()), 3.0 + 1e-9);
    EXPECT_LE(std::abs(p.position.y()), 1.5 + 1e-9);
  }
}

TEST(FigureEight, RejectsRampLongerThanMission)
{
  FigureEight::Params params;
  params.loops = 1;
  params.loop_period = 4.0;
  params.ramp_time = 10.0;
  EXPECT_THROW(FigureEight({0, 0, 0}, 0.0, params), std::invalid_argument);
}
