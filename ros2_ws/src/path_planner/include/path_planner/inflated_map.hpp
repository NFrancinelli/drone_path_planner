#pragma once

#include <unordered_map>
#include <vector>

#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// Obstacle inflation: a voxel is blocked if an occupied voxel center is within `radius` of
// its center. Computed on demand and cached, so only voxels actually queried cost anything.
// Holds a reference to the map; not thread-safe.
class InflatedMap
{
public:
  InflatedMap(const OccupancyMap & map, double radius);

  bool blocked(const Key & k) const;
  bool blocked(const Eigen::Vector3d & p) const {return blocked(map_.key(p));}

  const OccupancyMap & map() const {return map_;}

private:
  const OccupancyMap & map_;
  std::vector<Key> kernel_;
  mutable std::unordered_map<Key, bool, KeyHash> cache_;
};

}  // namespace path_planner
