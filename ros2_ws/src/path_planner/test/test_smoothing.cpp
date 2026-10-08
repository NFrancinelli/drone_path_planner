#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "path_planner/astar.hpp"
#include "path_planner/inflated_map.hpp"
#include "path_planner/smoothing.hpp"
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

// L-shaped path at z = 2.125: east 3 m, then north 3 m.
const Path kCorner{{0.125, 0.125, 2.125}, {3.125, 0.125, 2.125}, {3.125, 3.125, 2.125}};

void expect_within_limits(const trajectory_server::Trajectory & traj, const SmoothingParams & p)
{
  for (double t = 0.0; t <= traj.duration(); t += 0.02) {
    const auto s = traj.sample(t);
    EXPECT_LE(s.velocity.norm(), p.max_vel * 1.001) << "t = " << t;
    EXPECT_LE(s.acceleration.norm(), p.max_acc * 1.001) << "t = " << t;
    EXPECT_LE(std::abs(s.yaw_rate), p.max_yaw_rate * 1.001) << "t = " << t;
  }
}

}  // namespace

TEST(Shortcut, StraightensZigZagInFreeSpace)
{
  const TestMap map(Occupancy::Free);
  const Path zigzag{{0, 0, 2}, {0.5, 0.5, 2}, {1, 0, 2}, {1.5, 0.5, 2}, {2, 0, 2}};
  const Path out = shortcut(map, zigzag, 0.5, 1.0);
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

  const Path out = shortcut(map, raw.path, params.inflation_radius, params.unknown_cost);
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

  const Path out = shortcut(map, raw.path, params.inflation_radius, params.unknown_cost);
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
  EXPECT_GT(traj.sample(traj.knot_time(1)).velocity.norm(), 0.1);
  EXPECT_LT((traj.sample(traj.knot_time(1)).position - kCorner[1]).norm(), 1e-9);
  EXPECT_LT((traj.sample(traj.duration()).position - kCorner[2]).norm(), 1e-9);
  EXPECT_LT(traj.sample(traj.duration()).velocity.norm(), 1e-9);
  expect_within_limits(traj, params);
}

TEST(Smooth, StopsAtCornerWhenTheCurveWouldHitAnObstacle)
{
  // Column 0.5 m outside the first leg: clear of the straight path (> 0.4 m), but the
  // rounded corner bulges ~0.4 m outward and would pass right next to it.
  TestMap map(Occupancy::Free);
  map.set_box({2.2, -0.3, 1.5}, {2.2, -0.3, 2.8}, Occupancy::Occupied);
  const SmoothingParams params;
  const auto result = smooth(map, kCorner, params);
  ASSERT_TRUE(result.trajectory);
  const auto & traj = *result.trajectory;
  EXPECT_GE(result.stopped_knots, 1);
  EXPECT_LT(traj.sample(traj.knot_time(1)).velocity.norm(), 1e-9);
  const InflatedMap clearance(map, params.collision_radius);
  for (double t = 0.0; t <= traj.duration(); t += 0.02) {
    EXPECT_FALSE(clearance.blocked(traj.sample(t).position)) << "t = " << t;
  }
  expect_within_limits(traj, params);
}

TEST(Smooth, YawFacesNextSegmentAndHoldsThroughClimbs)
{
  const TestMap map(Occupancy::Free);
  const Path path{{0, 0, 1}, {2, 0, 1}, {2, 0, 3}, {2, 2, 3}};
  const auto result = smooth(map, path, SmoothingParams{});
  ASSERT_TRUE(result.trajectory);
  const auto & knots = result.trajectory->knots();
  EXPECT_NEAR(knots[0].yaw, 0.0, 1e-12);
  EXPECT_NEAR(knots[1].yaw, 0.0, 1e-12);  // climb: keeps facing east
  EXPECT_NEAR(knots[2].yaw, M_PI_2, 1e-12);
  EXPECT_NEAR(knots[3].yaw, M_PI_2, 1e-12);
}

TEST(Smooth, RejectsDegeneratePath)
{
  const TestMap map(Occupancy::Free);
  EXPECT_FALSE(smooth(map, {{1, 1, 1}}, SmoothingParams{}).trajectory);
  EXPECT_FALSE(smooth(map, {{1, 1, 1}, {1, 1, 1}}, SmoothingParams{}).trajectory);
}
