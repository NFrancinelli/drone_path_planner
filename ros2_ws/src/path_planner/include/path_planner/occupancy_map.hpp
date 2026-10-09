#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace path_planner
{

enum class Occupancy { Free, Occupied, Unknown };

// Voxel index. Voxel k spans [k * res, (k + 1) * res). Same as OctoMap keys without the
// 2^15 offset, so centers match the octree's.
using Key = Eigen::Vector3i;

struct KeyHash
{
  std::size_t operator()(const Key & k) const
  {
    // Teschner et al. spatial hash.
    return (static_cast<std::size_t>(k.x()) * 73856093u) ^
           (static_cast<std::size_t>(k.y()) * 19349663u) ^
           (static_cast<std::size_t>(k.z()) * 83492791u);
  }
};

// Map interface the planner searches on (sparse grid in tests, OctoMap later).
class OccupancyMap
{
public:
  virtual ~OccupancyMap() = default;
  virtual double resolution() const = 0;
  virtual Occupancy at(const Key & key) const = 0;

  Key key(const Eigen::Vector3d & p) const
  {
    return (p / resolution()).array().floor().cast<int>().matrix();
  }

  Eigen::Vector3d center(const Key & k) const
  {
    return ((k.cast<double>().array() + 0.5) * resolution()).matrix();
  }
};

}  // namespace path_planner
