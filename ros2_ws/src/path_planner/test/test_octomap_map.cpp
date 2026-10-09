#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <octomap/OcTree.h>

#include "path_planner/astar.hpp"
#include "path_planner/octomap_map.hpp"

using path_planner::AStarPlanner;
using path_planner::Occupancy;
using path_planner::OctomapMap;

namespace
{

octomap::point3d pt(const Eigen::Vector3d & p) {return {float(p.x()), float(p.y()), float(p.z())};}

}  // namespace

TEST(OctomapMap, KeysAndCentersMatchTheOctree)
{
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  const OctomapMap map(tree);
  // Points away from voxel boundaries, on both sides of the origin.
  for (const Eigen::Vector3d & p : {Eigen::Vector3d(0.05, 0.05, 0.05), Eigen::Vector3d(1.03, -0.37, 2.11),
      Eigen::Vector3d(-3.09, -0.01, 0.53), Eigen::Vector3d(-12.31, 7.77, -1.45)})
  {
    const octomap::point3d c = tree->keyToCoord(tree->coordToKey(pt(p)));
    EXPECT_LT((map.center(map.key(p)) - Eigen::Vector3d(c.x(), c.y(), c.z())).norm(), 1e-5) << p.transpose();
  }
}

TEST(OctomapMap, ReportsOccupiedFreeAndUnknown)
{
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  const Eigen::Vector3d occupied(1.03, -0.37, 2.11), free(-3.09, 0.11, 0.53), unknown(5.1, 5.1, 1.1);
  tree->updateNode(pt(occupied), true);
  tree->updateNode(pt(free), false);
  const OctomapMap map(tree);
  EXPECT_EQ(map.at(map.key(occupied)), Occupancy::Occupied);
  EXPECT_EQ(map.at(map.key(free)), Occupancy::Free);
  EXPECT_EQ(map.at(map.key(unknown)), Occupancy::Unknown);
  EXPECT_EQ(map.at(path_planner::Key(40000, 0, 0)), Occupancy::Unknown);  // outside the octree
}

TEST(OctomapMap, PrunedFreeSpaceStaysFree)
{
  // A full 8-voxel block of free space gets pruned into one parent node.
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  for (double x : {0.1, 0.3}) {
    for (double y : {0.1, 0.3}) {
      for (double z : {0.1, 0.3}) {
        tree->updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }
  tree->prune();
  const OctomapMap map(tree);
  EXPECT_EQ(map.at(map.key({0.3, 0.1, 0.3})), Occupancy::Free);
}

TEST(OctomapMap, PlansAroundOctreeObstacle)
{
  // Free slab at z = 1.0..1.2 m with a wall across it, except for a gap at y = 1.0..1.6 m.
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  for (double x = -0.9; x < 5.0; x += 0.2) {
    for (double y = -1.9; y < 2.0; y += 0.2) {
      const bool wall = std::abs(x - 2.1) < 0.05 && !(y > 1.0 && y < 1.6);
      tree->updateNode(octomap::point3d(x, y, 1.1), wall);
    }
  }
  const OctomapMap map(tree);
  AStarPlanner::Params params;
  params.inflation_radius = 0.25;
  params.unknown_cost = 10.0;
  params.bounds = Eigen::AlignedBox3d(Eigen::Vector3d(-1.0, -2.0, 1.0), Eigen::Vector3d(5.0, 2.0, 1.2));
  const auto result = AStarPlanner(params).plan(map, {0.1, 0.1, 1.1}, {4.1, 0.1, 1.1});
  ASSERT_EQ(result.status, AStarPlanner::Status::Success);
  bool through_gap = false;
  for (const auto & p : result.path) {
    if (map.key(p).x() == 10) {
      through_gap |= p.y() > 1.0 && p.y() < 1.6;
    }
  }
  EXPECT_TRUE(through_gap);
}
