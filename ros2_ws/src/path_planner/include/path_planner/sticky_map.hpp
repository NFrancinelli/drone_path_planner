#pragma once

#include <vector>

#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// A map in which every voxel ever seen occupied stays occupied: `base` plus `occupied`.
// Our worlds are static, and OctoMap can erase a real obstacle: a surface in the middle of
// a voxel leaves it partly free, and sensor rays passing close along the surface mark it
// free. A wall mapped as one layer of voxels, with the unknown inside of the object behind
// it, then gets a hole the planner would fly through. Holds references to both.
class StickyMap : public OccupancyMap
{
public:
  StickyMap(const OccupancyMap & base, const KeySet & occupied)
  : base_(base), occupied_(occupied) {}

  double resolution() const override {return base_.resolution();}

  Occupancy at(const Key & k) const override
  {
    return occupied_.count(k) ? Occupancy::Occupied : base_.at(k);
  }

  void fill(const Key & lo, const Key & size, std::vector<Occupancy> & out) const override
  {
    base_.fill(lo, size, out);
    for (const Key & k : occupied_) {
      const Key d = k - lo;
      if ((d.array() >= 0).all() && (d.array() < size.array()).all()) {
        out[(static_cast<std::size_t>(d.z()) * size.y() + d.y()) * size.x() + d.x()] =
          Occupancy::Occupied;
      }
    }
  }

private:
  const OccupancyMap & base_;
  const KeySet & occupied_;
};

}  // namespace path_planner
