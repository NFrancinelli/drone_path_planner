#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <octomap/OcTree.h>

#include "path_planner/astar.hpp"
#include "path_planner/octomap_map.hpp"
#include "path_planner/sticky_map.hpp"

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

TEST(OctomapMap, FillMatchesPerVoxelLookups)
{
  // Pruned free blocks, scattered occupied voxels, unknown around them, negative keys, and a
  // box that cuts through coarse leaves.
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  for (double x = -1.5; x < 1.7; x += 0.2) {
    for (double y = -1.5; y < 1.7; y += 0.2) {
      for (double z = 0.1; z < 1.7; z += 0.2) {
        tree->updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }
  for (const auto & p : {octomap::point3d(0.3, 0.5, 0.7), octomap::point3d(-1.1, 0.1, 1.3),
      octomap::point3d(2.5, -0.3, 0.9)})
  {
    tree->updateNode(p, true);
  }
  tree->updateInnerOccupancy();
  tree->prune();
  const path_planner::OctomapMap map(tree);

  const path_planner::Key lo = map.key({-1.3, -0.9, -0.3});
  const path_planner::Key size(22, 17, 12);
  std::vector<path_planner::Occupancy> filled;
  map.fill(lo, size, filled);
  ASSERT_EQ(filled.size(), 22u * 17u * 12u);
  size_t i = 0, occupied = 0;
  for (int z = 0; z < size.z(); ++z) {
    for (int y = 0; y < size.y(); ++y) {
      for (int x = 0; x < size.x(); ++x, ++i) {
        const path_planner::Key k = lo + path_planner::Key(x, y, z);
        EXPECT_EQ(filled[i], map.at(k)) << k.transpose();
        occupied += filled[i] == path_planner::Occupancy::Occupied;
      }
    }
  }
  EXPECT_EQ(occupied, 3u);
}

TEST(StickyMap, KeepsObstaclesTheMapHasSinceErased)
{
  // A wall voxel seen occupied, then marked free by later rays: still an obstacle.
  auto tree = std::make_shared<octomap::OcTree>(0.2);
  const octomap::point3d wall(1.1, 0.7, 2.5), other(1.3, 0.7, 2.5);
  tree->updateNode(wall, true);
  tree->updateNode(other, true);
  const path_planner::OctomapMap map(tree);
  path_planner::KeySet seen;
  map.collect_occupied(seen);
  EXPECT_EQ(seen.size(), 2u);

  for (int i = 0; i < 20; ++i) {
    tree->updateNode(wall, false);
  }
  const path_planner::Key k = map.key({1.1, 0.7, 2.5});
  ASSERT_EQ(map.at(k), path_planner::Occupancy::Free);
  const path_planner::StickyMap sticky(map, seen);
  EXPECT_EQ(sticky.at(k), path_planner::Occupancy::Occupied);
  EXPECT_EQ(sticky.at(map.key({0.1, 0.1, 0.1})), path_planner::Occupancy::Unknown);

  // fill() agrees, inside the box only.
  const path_planner::Key lo = k - path_planner::Key(1, 1, 1), size(3, 3, 3);
  std::vector<path_planner::Occupancy> filled;
  sticky.fill(lo, size, filled);
  size_t occupied = 0;
  for (const auto o : filled) {
    occupied += o == path_planner::Occupancy::Occupied;
  }
  EXPECT_EQ(occupied, 2u);  // the erased wall voxel and its neighbour
  EXPECT_EQ(filled[(1 * 3 + 1) * 3 + 1], path_planner::Occupancy::Occupied);
}
