#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "path_planner/astar.hpp"
#include "test_map.hpp"

using path_planner::AStarPlanner;
using path_planner::Key;
using path_planner::Occupancy;
using Status = AStarPlanner::Status;
using namespace test;  // NOLINT

namespace
{

// Min distance from path voxel centers (start excluded) to an occupied voxel center.
double min_clearance(const TestMap & map, const std::vector<Eigen::Vector3d> & path)
{
  double clearance = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path.size(); ++i) {
    const Key k = map.key(path[i]);
    for (int dx = -4; dx <= 4; ++dx) {
      for (int dy = -4; dy <= 4; ++dy) {
        for (int dz = -4; dz <= 4; ++dz) {
          const Key n = k + Key(dx, dy, dz);
          if (map.at(n) == Occupancy::Occupied) {
            clearance = std::min(clearance, (map.center(n) - map.center(k)).norm());
          }
        }
      }
    }
  }
  return clearance;
}

AStarPlanner::Params wall_params()
{
  AStarPlanner::Params params;
  params.inflation_radius = 0.3;
  params.bounds = flat_bounds(-1.0, -2.0, 6.0, 3.0);
  return params;
}

const Eigen::Vector3d kWallStart(0.125, 0.125, kLayerZ);
const Eigen::Vector3d kWallGoal(5.125, 0.125, kLayerZ);

}  // namespace

TEST(AStar, StraightLineInFreeSpace)
{
  const TestMap map(Occupancy::Free);
  AStarPlanner::Params params;
  const Eigen::Vector3d start(0.125, 0.125, 2.125), goal(5.125, 0.125, 2.125);
  const auto result = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(result.status, Status::Success);
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
  EXPECT_EQ(result.path.size(), 21u);
  EXPECT_NEAR(result.cost, 5.0, 1e-9);
}

TEST(AStar, StartEqualsGoal)
{
  const TestMap map(Occupancy::Free);
  const Eigen::Vector3d p(1.0, 1.0, 2.0);
  const auto result = AStarPlanner(AStarPlanner::Params{}).plan(map, p, p);
  ASSERT_EQ(result.status, Status::Success);
  EXPECT_EQ(result.path.size(), 2u);
  EXPECT_DOUBLE_EQ(result.cost, 0.0);
}

TEST(AStar, GoesThroughGapWithClearance)
{
  const TestMap map = wall_with_gap(3);
  const auto params = wall_params();
  const auto result = AStarPlanner(params).plan(map, kWallStart, kWallGoal);
  ASSERT_EQ(result.status, Status::Success);
  EXPECT_GT(min_clearance(map, result.path), params.inflation_radius);
  bool crossed_in_gap = false;
  for (const auto & p : result.path) {
    EXPECT_TRUE(params.bounds.contains(p));
    if (map.key(p).x() == 10) {
      crossed_in_gap |= std::abs(p.y() - 1.125) < 1e-9;  // middle voxel of the gap
    }
  }
  EXPECT_TRUE(crossed_in_gap);
}

TEST(AStar, GapNarrowerThanInflationIsClosed)
{
  // 0.5 m gap: gap voxels are 0.25 m from the wall, < 0.3 m inflation.
  const TestMap map = wall_with_gap(2);
  const auto result = AStarPlanner(wall_params()).plan(map, kWallStart, kWallGoal);
  EXPECT_EQ(result.status, Status::NoPath);
}

TEST(AStar, CanLeaveStartInsideInflation)
{
  TestMap map(Occupancy::Free);
  map.set_box({0.6, -10.0, 0.0}, {0.6, 10.0, 3.0}, Occupancy::Occupied);  // x = 0.5..0.75
  AStarPlanner::Params params;
  params.inflation_radius = 0.3;
  params.bounds = flat_bounds(-3.0, -2.0, 3.0, 2.0);
  const Eigen::Vector3d start(0.375, 0.125, kLayerZ);  // 0.25 m from the wall
  const Eigen::Vector3d goal(-2.125, 0.125, kLayerZ);
  const auto result = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(result.status, Status::Success);
  EXPECT_GT(min_clearance(map, result.path), params.inflation_radius);
}

TEST(AStar, UnknownCostTradesDistanceForKnownSpace)
{
  // Unknown map with a U-shaped free corridor: 4 m straight through unknown vs ~8 m
  // around through free space.
  TestMap map(Occupancy::Unknown);
  map.set_box({0.1, 0.1, kLayerZ}, {0.1, 2.1, kLayerZ}, Occupancy::Free);
  map.set_box({0.1, 2.1, kLayerZ}, {4.1, 2.1, kLayerZ}, Occupancy::Free);
  map.set_box({4.1, 0.1, kLayerZ}, {4.1, 2.1, kLayerZ}, Occupancy::Free);
  const Eigen::Vector3d start(0.125, 0.125, kLayerZ), goal(4.125, 0.125, kLayerZ);

  AStarPlanner::Params params;
  params.bounds = flat_bounds(-1.0, -1.0, 5.0, 3.0);

  params.unknown_cost = 0.0;
  const auto direct = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(direct.status, Status::Success);
  EXPECT_NEAR(direct.cost, 4.0, 1e-9);

  params.unknown_cost = 10.0;
  const auto detour = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(detour.status, Status::Success);
  for (size_t i = 1; i + 1 < detour.path.size(); ++i) {
    EXPECT_EQ(map.at(map.key(detour.path[i])), Occupancy::Free) << "waypoint " << i;
  }
  EXPECT_LT(detour.cost, 8.0);
}

TEST(AStar, VerticalCostKeepsAltitude)
{
  // Free space with a 0.5 m unknown block across the 2 m layer. Crossing it costs 0.5 extra;
  // hopping a layer over it costs 0.21 extra in length, plus 0.5 of climb when charged.
  TestMap map(Occupancy::Free);
  map.set_box({2.1, -3.0, 2.0}, {2.4, 3.0, 2.0}, Occupancy::Unknown);
  const Eigen::Vector3d start(0.125, 0.125, 2.125), goal(5.125, 0.125, 2.125);
  AStarPlanner::Params params;

  params.vertical_cost = 0.0;
  const auto hop = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(hop.status, Status::Success);
  bool left_layer = false;
  for (const auto & p : hop.path) {
    left_layer |= std::abs(p.z() - start.z()) > 1e-9;
  }
  EXPECT_TRUE(left_layer);

  params.vertical_cost = 1.0;
  const auto level = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(level.status, Status::Success);
  for (const auto & p : level.path) {
    EXPECT_DOUBLE_EQ(p.z(), start.z()) << p.transpose();
  }
  EXPECT_NEAR(level.cost, 5.0 + params.unknown_cost * 0.5, 1e-9);
}

TEST(AStar, VerticalCostChangesAltitudeOnce)
{
  // Free space, goal one metre up: the climb is charged once, never more.
  const TestMap map(Occupancy::Free);
  const Eigen::Vector3d start(0.125, 0.125, 1.125), goal(5.125, 0.125, 2.125);
  AStarPlanner::Params params;
  const auto result = AStarPlanner(params).plan(map, start, goal);
  ASSERT_EQ(result.status, Status::Success);
  for (size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_GE(result.path[i].z(), result.path[i - 1].z() - 1e-9) << "waypoint " << i;
  }
  // 4 straight steps + 4 diagonal ones, plus 1 m of climb.
  EXPECT_NEAR(result.cost, (16.0 + 4.0 * std::sqrt(2.0)) * kRes + params.vertical_cost, 1e-9);
}

TEST(AStar, RejectsBadStartAndGoal)
{
  TestMap map(Occupancy::Free);
  map.set_box({3.0, 3.0, 2.0}, {3.0, 3.0, 2.0}, Occupancy::Occupied);
  const AStarPlanner planner(AStarPlanner::Params{});  // z bounds 0.5..5 m
  const Eigen::Vector3d ok(0.0, 0.0, 2.0);
  EXPECT_EQ(planner.plan(map, {0, 0, 0.1}, ok).status, Status::StartOutOfBounds);
  EXPECT_EQ(planner.plan(map, ok, {0, 0, 9.0}).status, Status::GoalOutOfBounds);
  EXPECT_EQ(planner.plan(map, ok, {3.0, 3.0, 2.0}).status, Status::GoalBlocked);
  EXPECT_EQ(planner.plan(map, ok, {3.3, 3.0, 2.0}).status, Status::GoalBlocked);  // inflated
}

TEST(AStar, StopsAtExpansionLimit)
{
  const TestMap map(Occupancy::Unknown);
  AStarPlanner::Params params;
  params.max_expansions = 50;
  const auto result = AStarPlanner(params).plan(map, {0, 0, 2}, {40, 40, 2});
  EXPECT_EQ(result.status, Status::ExpansionLimit);
  EXPECT_TRUE(result.path.empty());
}
