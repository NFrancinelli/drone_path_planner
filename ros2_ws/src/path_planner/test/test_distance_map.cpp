#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "path_planner/distance_map.hpp"
#include "test_map.hpp"

using path_planner::DistanceMap;
using namespace test;  // NOLINT

namespace
{

const Eigen::AlignedBox3d kRegion(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(3.0, 2.0, 1.5));

// Brute force: distance from k to the nearest of the given occupied voxels.
double nearest(const TestMap & map, const Key & k, const std::vector<Key> & occupied)
{
  double best = std::numeric_limits<double>::infinity();
  for (const Key & o : occupied) {
    best = std::min(best, (map.center(o) - map.center(k)).norm());
  }
  return best;
}

}  // namespace

TEST(DistanceMap, MatchesBruteForce)
{
  TestMap map(Occupancy::Free);
  // A few scattered voxels, one of them outside the region but within max_distance of it.
  const std::vector<Eigen::Vector3d> points{
    {0.3, 0.4, 0.2}, {2.6, 1.1, 1.3}, {1.4, 1.9, 0.6}, {3.4, 0.6, 0.7}};
  std::vector<Key> occupied;
  for (const auto & p : points) {
    map.set_box(p, p, Occupancy::Occupied);
    occupied.push_back(map.key(p));
  }
  const DistanceMap dist(map, kRegion, 1.0);
  const Key lo = map.key(kRegion.min()), hi = map.key(kRegion.max());
  for (int x = lo.x(); x <= hi.x(); ++x) {
    for (int y = lo.y(); y <= hi.y(); ++y) {
      for (int z = lo.z(); z <= hi.z(); ++z) {
        const Key k(x, y, z);
        const double expected = nearest(map, k, occupied);
        if (expected <= 1.0) {
          EXPECT_NEAR(dist.distance(k), expected, 1e-6) << k.transpose();
        } else {
          EXPECT_GT(dist.distance(k), 1.0 - 1e-6) << k.transpose();
        }
        EXPECT_EQ(dist.at(k), map.at(k));
      }
    }
  }
}

TEST(DistanceMap, EmptyAndOutsideGrid)
{
  const TestMap map(Occupancy::Unknown);
  const DistanceMap dist(map, kRegion, 0.5);
  EXPECT_TRUE(std::isinf(dist.distance(map.key({1.0, 1.0, 1.0}))));
  EXPECT_EQ(dist.at(map.key({1.0, 1.0, 1.0})), Occupancy::Unknown);
  // Far outside: no distance (treated as touching an obstacle), occupancy from the map.
  EXPECT_EQ(dist.distance(map.key({9.0, 9.0, 9.0})), 0.0);
  EXPECT_EQ(dist.at(map.key({9.0, 9.0, 9.0})), Occupancy::Unknown);
}
