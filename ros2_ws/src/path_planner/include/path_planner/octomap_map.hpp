#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <octomap/OcTree.h>

#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// OccupancyMap on top of an OctoMap octree. Our keys are OcTreeKeys at full depth (16)
// shifted by the octree's center offset.
class OctomapMap : public OccupancyMap
{
public:
  explicit OctomapMap(std::shared_ptr<const octomap::OcTree> tree)
  : tree_(std::move(tree)) {}

  double resolution() const override {return tree_->getResolution();}

  Occupancy at(const Key & k) const override
  {
    if ((k.array() < -kCenter).any() || (k.array() >= kCenter).any()) {
      return Occupancy::Unknown;
    }
    const octomap::OcTreeKey key(k.x() + kCenter, k.y() + kCenter, k.z() + kCenter);
    // search() also finds pruned nodes, i.e. a coarser leaf covering this voxel.
    const octomap::OcTreeNode * node = tree_->search(key);
    if (!node) {
      return Occupancy::Unknown;
    }
    return tree_->isNodeOccupied(node) ? Occupancy::Occupied : Occupancy::Free;
  }

  // Iterates the octree's leaves in the box instead of searching every voxel: a large free
  // or unknown region is a single coarse leaf (or no node at all).
  void fill(const Key & lo, const Key & size, std::vector<Occupancy> & out) const override
  {
    const long nx = size.x(), ny = size.y(), nz = size.z();
    out.assign(nx * ny * nz, Occupancy::Unknown);
    const double res = resolution();
    const Key hi = lo + size - Key::Ones();
    const octomap::point3d min(lo.x() * res, lo.y() * res, lo.z() * res);
    const octomap::point3d max((hi.x() + 1) * res, (hi.y() + 1) * res, (hi.z() + 1) * res);
    for (auto it = tree_->begin_leafs_bbx(min, max), end = tree_->end_leafs_bbx(); it != end;
      ++it)
    {
      const Occupancy occ = tree_->isNodeOccupied(*it) ? Occupancy::Occupied : Occupancy::Free;
      // Voxels the leaf covers (several for a pruned leaf), clipped to the box. Keys from the
      // leaf's corner, offset by half a voxel to stay clear of rounding at the boundaries.
      const double half = it.getSize() / 2.0;
      const octomap::point3d c = it.getCoordinate();
      const Eigen::Vector3d corner(c.x() - half, c.y() - half, c.z() - half);
      const Key a = key(corner + Eigen::Vector3d::Constant(res / 2.0));
      const Key b = key(corner + Eigen::Vector3d::Constant(2.0 * half - res / 2.0));
      const Key from = a.cwiseMax(lo), to = b.cwiseMin(hi);
      for (int z = from.z(); z <= to.z(); ++z) {
        for (int y = from.y(); y <= to.y(); ++y) {
          long i = ((z - lo.z()) * ny + (y - lo.y())) * nx + (from.x() - lo.x());
          for (int x = from.x(); x <= to.x(); ++x) {
            out[i++] = occ;
          }
        }
      }
    }
  }

private:
  static constexpr int kCenter = 32768;  // tree_max_val for a depth-16 tree

  std::shared_ptr<const octomap::OcTree> tree_;
};

}  // namespace path_planner
