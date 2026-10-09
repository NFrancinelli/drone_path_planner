#pragma once

#include <unordered_map>

#include <Eigen/Geometry>

#include "path_planner/occupancy_map.hpp"

namespace test
{

using path_planner::Key;
using path_planner::KeyHash;
using path_planner::Occupancy;
using path_planner::OccupancyMap;

// 0.25 is exact in binary, avoids floor() rounding issues on box edges.
constexpr double kRes = 0.25;
constexpr double kLayerZ = 1.125;  // center of the single z layer most tests plan in

// Sparse map: every voxel is `fill` unless set otherwise.
class TestMap : public OccupancyMap
{
public:
  explicit TestMap(Occupancy fill)
  : fill_(fill) {}

  double resolution() const override {return kRes;}

  Occupancy at(const Key & k) const override
  {
    const auto it = cells_.find(k);
    return it == cells_.end() ? fill_ : it->second;
  }

  // Sets every voxel from the one containing `lo` to the one containing `hi`.
  void set_box(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi, Occupancy occ)
  {
    const Key a = key(lo), b = key(hi);
    for (int x = a.x(); x <= b.x(); ++x) {
      for (int y = a.y(); y <= b.y(); ++y) {
        for (int z = a.z(); z <= b.z(); ++z) {
          cells_[Key(x, y, z)] = occ;
        }
      }
    }
  }

private:
  Occupancy fill_;
  std::unordered_map<Key, Occupancy, KeyHash> cells_;
};

// Bounds with a single voxel layer at kLayerZ (2D tests).
inline Eigen::AlignedBox3d flat_bounds(double x0, double y0, double x1, double y1)
{
  return Eigen::AlignedBox3d(Eigen::Vector3d(x0, y0, 1.0), Eigen::Vector3d(x1, y1, 1.25));
}

// Wall at x = 2.5..2.75 m spanning the whole test area, with a gap at y = 0.75 m and up.
inline TestMap wall_with_gap(int gap_voxels)
{
  TestMap map(Occupancy::Free);
  map.set_box({2.6, -10.0, 0.0}, {2.6, 10.0, 3.0}, Occupancy::Occupied);
  map.set_box({2.6, 0.8, 0.0}, {2.6, 0.8 + (gap_voxels - 1) * kRes, 3.0}, Occupancy::Free);
  return map;
}

}  // namespace test
