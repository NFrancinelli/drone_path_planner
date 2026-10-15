#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// Dense copy of a map over a box, with the distance from every voxel to the nearest occupied
// voxel (center to center, exact Euclidean up to max_distance). Built once per plan so
// the search, shortcut and smoothing get O(1) occupancy and clearance lookups instead of an
// octree search per neighbour voxel. The grid extends max_distance beyond the box, so
// obstacles just outside it still count. Holds a reference to the map.
class DistanceMap : public OccupancyMap
{
public:
  DistanceMap(const OccupancyMap & map, const Eigen::AlignedBox3d & region, double max_distance);

  double resolution() const override {return map_.resolution();}
  // The copy inside the grid, the underlying map outside it.
  Occupancy at(const Key & k) const override;

  // [m]. Exact up to max_distance; beyond it only known to be larger (obstacles past the
  // grid's margin aren't seen), infinite with none in the grid. Outside the grid: 0.
  double distance(const Key & k) const;
  double max_distance() const {return max_distance_;}

  // Flat index of k into the grid, or -1 outside it; for per-voxel arrays (see A*).
  long index(const Key & k) const;
  long cells() const {return static_cast<long>(distance_.size());}

private:
  const OccupancyMap & map_;
  double max_distance_;
  Key lo_;    // first voxel of the grid
  Key size_;  // voxels along each axis
  std::vector<Occupancy> occupancy_;
  // double, not float: grid distances are sqrt(n) * resolution, and a float 0.4 is just
  // over 0.4, so a voxel exactly at a radius would count as outside it.
  std::vector<double> distance_;
};

}  // namespace path_planner
