#pragma once

#include <memory>
#include <utility>

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

private:
  static constexpr int kCenter = 32768;  // tree_max_val for a depth-16 tree

  std::shared_ptr<const octomap::OcTree> tree_;
};

}  // namespace path_planner
