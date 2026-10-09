#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "path_planner/astar.hpp"
#include "path_planner/inflated_map.hpp"
#include "path_planner/smoothing.hpp"
#include "trajectory_server/frames.hpp"
#include "test_map.hpp"

using path_planner::AStarPlanner;
using path_planner::InflatedMap;
using path_planner::SmoothingParams;
using path_planner::shortcut;
using path_planner::smooth;
using namespace test;  // NOLINT

namespace
{

using Path = std::vector<Eigen::Vector3d>;

// Every point along the polyline, a quarter voxel apart (start voxel excluded).
Path densify(const OccupancyMap & map, const Path & path)
{
  Path out;
  for (size_t i = 1; i < path.size(); ++i) {
    const double len = (path[i] - path[i - 1]).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / (0.25 * map.resolution()))));
    for (int j = 1; j <= n; ++j) {
      out.push_back(path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(j) / n));
    }
  }
  return out;
}

double length(const Path & path)
{
  double l = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    l += (path[i] - path[i - 1]).norm();
  }
  return l;
}

// L-shaped path at z = 2.125: east 3 m, then north 3 m. The first leg runs 1 cm inside its
// voxel row, so any outward rounding enters the next row.
const Path kCorner{{0.125, 0.01, 2.125}, {3.125, 0.01, 2.125}, {3.125, 3.125, 2.125}};

// Index of the knot sitting on `p`.
size_t knot_at(const trajectory_server::WaypointTrajectory & traj, const Eigen::Vector3d & p)
{
  const auto & knots = traj.knots();
  for (size_t k = 0; k < knots.size(); ++k) {
    if ((knots[k].position - p).norm() < 1e-9) {
      return k;
    }
  }
  ADD_FAILURE() << "no knot at " << p.transpose();
  return 0;
}

void expect_within_limits(const trajectory_server::Trajectory & traj, const SmoothingParams & p)
{
  for (double t = 0.0; t <= traj.duration(); t += 0.02) {
    const auto s = traj.sample(t);
    EXPECT_LE(s.velocity.norm(), p.max_vel * 1.001) << "t = " << t;
    EXPECT_LE(s.acceleration.norm(), p.max_acc * 1.001) << "t = " << t;
    EXPECT_LE(std::abs(s.yaw_rate), p.max_yaw_rate * 1.001) << "t = " << t;
  }
}

// Largest distance from the trajectory to the polyline through `path`.
double max_deviation(const trajectory_server::Trajectory & traj, const Path & path)
{
  double worst = 0.0;
  for (double t = 0.0; t <= traj.duration(); t += 0.01) {
    const Eigen::Vector3d p = traj.sample(t).position;
    double nearest = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector3d a = path[i - 1], d = path[i] - a;
      const double s = std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0);
      nearest = std::min(nearest, (a + s * d - p).norm());
    }
    worst = std::max(worst, nearest);
  }
  return worst;
}

// Yaw rate matches the change in yaw, including across the end of the position knots.
void expect_consistent_yaw(const trajectory_server::Trajectory & traj)
{
  const double h = 1e-5;
  for (double t = h; t < traj.duration() - h; t += 0.01) {
    const double rate =
      trajectory_server::wrap_angle(traj.sample(t + h).yaw - traj.sample(t - h).yaw) / (2 * h);
    EXPECT_NEAR(rate, traj.sample(t).yaw_rate, 1e-4) << "t = " << t;
  }
}

}  // namespace

TEST(Shortcut, StraightensZigZagInFreeSpace)
{
  const TestMap map(Occupancy::Free);
  const Path zigzag{{0, 0, 2}, {0.5, 0.5, 2}, {1, 0, 2}, {1.5, 0.5, 2}, {2, 0, 2}};
  const Path out = shortcut(map, zigzag, AStarPlanner::Params{});
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out.front(), zigzag.front());
  EXPECT_EQ(out.back(), zigzag.back());
}

TEST(Shortcut, KeepsClearanceThroughGap)
{
  const TestMap map = wall_with_gap(3);
  AStarPlanner::Params params;
  params.inflation_radius = 0.3;
  params.bounds = flat_bounds(-1.0, -2.0, 6.0, 3.0);
  const Eigen::Vector3d start(0.125, 0.125, kLayerZ), goal(5.125, 0.125, kLayerZ);
  const auto raw = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(raw.status, AStarPlanner::Status::Success);

  const Path out = shortcut(map, raw.path, params);
  EXPECT_LT(out.size(), raw.path.size());
  EXPECT_LE(out.size(), 5u);
  EXPECT_EQ(out.front(), start);
  EXPECT_EQ(out.back(), goal);
  EXPECT_LE(length(out), length(raw.path) + 1e-9);
  const InflatedMap inflated(map, params.inflation_radius);
  for (const auto & p : densify(map, out)) {
    EXPECT_FALSE(inflated.blocked(p)) << p.transpose();
  }
}

TEST(Shortcut, DoesNotCutThroughUnknownTheSearchAvoided)
{
  // Same U-shaped free corridor as the A* test, unknown everywhere else.
  TestMap map(Occupancy::Unknown);
  map.set_box({0.1, 0.1, kLayerZ}, {0.1, 2.1, kLayerZ}, Occupancy::Free);
  map.set_box({0.1, 2.1, kLayerZ}, {4.1, 2.1, kLayerZ}, Occupancy::Free);
  map.set_box({4.1, 0.1, kLayerZ}, {4.1, 2.1, kLayerZ}, Occupancy::Free);
  AStarPlanner::Params params;
  params.unknown_cost = 10.0;
  params.bounds = flat_bounds(-1.0, -1.0, 5.0, 3.0);
  const auto raw = AStarPlanner(params).plan(map, {0.125, 0.125, kLayerZ}, {4.125, 0.125, kLayerZ});
  ASSERT_EQ(raw.status, AStarPlanner::Status::Success);

  const Path out = shortcut(map, raw.path, params);
  EXPECT_LT(out.size(), raw.path.size());
  for (const auto & p : densify(map, out)) {
    EXPECT_EQ(map.at(map.key(p)), Occupancy::Free) << p.transpose();
  }
}

TEST(Smooth, KeepsMovingThroughFreeCorner)
{
  const TestMap map(Occupancy::Free);
  const SmoothingParams params;
  const auto result = smooth(map, kCorner, params);
  ASSERT_TRUE(result.trajectory);
  const auto & traj = *result.trajectory;
  EXPECT_EQ(result.stopped_knots, 0);
  const double t_corner = traj.knot_time(knot_at(traj, kCorner[1]));
  EXPECT_GT(traj.sample(t_corner).velocity.norm(), 0.1);
  EXPECT_LT((traj.sample(t_corner).position - kCorner[1]).norm(), 1e-9);
  EXPECT_LT((traj.sample(traj.duration()).position - kCorner[2]).norm(), 1e-9);
  EXPECT_LT(traj.sample(traj.duration()).velocity.norm(), 1e-9);
  expect_within_limits(traj, params);
}

TEST(Smooth, StopsAtCornerWhenTheCurveWouldHitAnObstacle)
{
  // Column two voxel rows outside the first leg, just before the corner: clear of the
  // straight legs (0.5 m), but the rounding swings slightly outward into the row next to it.
  TestMap map(Occupancy::Free);
  map.set_box({2.9, -0.3, 1.5}, {2.9, -0.3, 2.8}, Occupancy::Occupied);
  const SmoothingParams params;
  const auto result = smooth(map, kCorner, params);
  ASSERT_TRUE(result.trajectory);
  const auto & traj = *result.trajectory;
  EXPECT_EQ(result.stopped_knots, 1);
  EXPECT_EQ(traj.knots().size(), 3u);  // start, stopped corner, goal
  EXPECT_LT(traj.sample(traj.knot_time(knot_at(traj, kCorner[1]))).velocity.norm(), 1e-9);
  const InflatedMap clearance(map, params.collision_radius);
  for (double t = 0.0; t <= traj.duration(); t += 0.02) {
    EXPECT_FALSE(clearance.blocked(traj.sample(t).position)) << "t = " << t;
  }
  expect_within_limits(traj, params);
}

TEST(Smooth, YawFacesEachLegByItsCornerAndHoldsThroughClimbs)
{
  const TestMap map(Occupancy::Free);
  const Path path{{0, 0, 1}, {2, 0, 1}, {2, 0, 3}, {2, 2, 3}};
  const auto result = smooth(map, path, SmoothingParams{});
  ASSERT_TRUE(result.trajectory);
  const auto & traj = *result.trajectory;
  const auto yaw_at = [&](const Eigen::Vector3d & p) {
      return traj.sample(traj.knot_time(knot_at(traj, p))).yaw;
    };
  EXPECT_NEAR(yaw_at(path[0]), 0.0, 1e-12);
  EXPECT_NEAR(yaw_at(path[1]), 0.0, 1e-12);  // climb: keeps facing east
  EXPECT_NEAR(yaw_at(path[2]), M_PI_2, 1e-12);  // turned by the time it reaches the corner
  EXPECT_NEAR(traj.sample(traj.duration()).yaw, M_PI_2, 1e-12);
  ASSERT_EQ(traj.turns().size(), 1u);
}

TEST(Smooth, RejectsDegeneratePath)
{
  const TestMap map(Occupancy::Free);
  EXPECT_FALSE(smooth(map, {{1, 1, 1}}, SmoothingParams{}).trajectory);
  EXPECT_FALSE(smooth(map, {{1, 1, 1}, {1, 1, 1}}, SmoothingParams{}).trajectory);
}

TEST(Smooth, CornersStayCloseToTheStraightPath)
{
  // Rounding only happens within corner_distance (0.75 m) of the corner, so the curve
  // stays within ~0.1 m of the straight legs instead of drifting over the whole leg.
  const TestMap map(Occupancy::Free);
  const SmoothingParams params;
  const Path shallow{{0, 0, 2}, {4, 0, 2}, {7, 2.5, 2}};  // ~40 deg turn
  const auto sharp = smooth(map, kCorner, params);
  const auto gentle = smooth(map, shallow, params);
  ASSERT_TRUE(sharp.trajectory);
  ASSERT_TRUE(gentle.trajectory);
  EXPECT_LT(max_deviation(*sharp.trajectory, kCorner), 0.15);
  EXPECT_LT(max_deviation(*gentle.trajectory, shallow), 0.08);
  // A gentle turn is taken faster than a sharp one.
  const auto corner_speed = [](const auto & traj, const Eigen::Vector3d & p) {
      return traj.sample(traj.knot_time(knot_at(traj, p))).velocity.norm();
    };
  EXPECT_GT(corner_speed(*gentle.trajectory, shallow[1]), corner_speed(*sharp.trajectory, kCorner[1]));
  expect_within_limits(*sharp.trajectory, params);
  expect_within_limits(*gentle.trajectory, params);
}

TEST(Smooth, CloseCornersDoNotSlowTheFlightForYaw)
{
  // Two corners 0.57 m apart with a 46 deg heading change between them, as the planner
  // produced in the sim. The turn can't finish on that short stretch; it may run late,
  // but the corners keep their speed.
  const TestMap map(Occupancy::Free);
  const Path path{{0, 0, 2.77}, {1.1, 0.9, 2.7}, {1.7, 1.5, 2.5}, {2.1, 1.9, 2.5}, {2.0, 6.5, 2.5}};
  const SmoothingParams params;
  const auto result = smooth(map, path, params);
  ASSERT_TRUE(result.trajectory);
  const auto & traj = *result.trajectory;
  for (size_t k = 1; k + 1 < path.size(); ++k) {
    EXPECT_GT(traj.sample(traj.knot_time(knot_at(traj, path[k]))).velocity.norm(), 0.2) << k;
  }
  expect_within_limits(traj, params);
  expect_consistent_yaw(traj);
}
