#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "path_planner/distance_map.hpp"
#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// 26-connected A* on the map's voxel grid.
// A voxel is blocked if an occupied voxel center is within inflation_radius of its center.
// Unknown voxels cost (1 + unknown_cost) per metre. Voxels closer than clearance_radius to
// an obstacle cost up to clearance_cost more per metre, rising linearly from 0 at
// clearance_radius to clearance_cost at inflation_radius, so paths run down the middle of
// gaps and corridors instead of hugging their edges. Climbing or descending costs an extra
// vertical_cost per metre of height change, so the drone keeps its altitude unless a climb
// actually pays off; without it, paths wander up and down between near-equal options.
// The search is clamped to `bounds`, which also keeps it from expanding forever into
// unknown space.
class AStarPlanner
{
public:
  struct Params
  {
    double inflation_radius{0.5};  // [m]
    double unknown_cost{1.0};      // extra cost per metre through unknown space, 0 = free
    double vertical_cost{1.0};     // extra cost per metre of height change, 0 = free
    double clearance_radius{1.0};  // [m] obstacles closer than this add clearance cost
    double clearance_cost{1.0};    // extra cost per metre right at inflation_radius
    Eigen::AlignedBox3d bounds{Eigen::Vector3d(-20.0, -20.0, 0.5), Eigen::Vector3d(20.0, 20.0, 5.0)};
    int max_expansions{200000};
  };

  enum class Status { Success, StartOutOfBounds, GoalOutOfBounds, GoalBlocked, NoPath, ExpansionLimit };

  struct Result
  {
    Status status{Status::NoPath};
    std::vector<Eigen::Vector3d> path;  // start, voxel centers in between, goal
    double cost{0.0};
    int expansions{0};
  };

  explicit AStarPlanner(const Params & params);

  // The start voxel is allowed to be blocked, in case the drone is already closer to a
  // wall than inflation_radius. The distance map must cover `bounds` up to max_distance().
  Result plan(const DistanceMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const;
  // Builds the distance map over `bounds` first.
  Result plan(const OccupancyMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const;

  // Distance the map has to resolve for these params.
  double max_distance() const {return std::max(p_.inflation_radius, p_.clearance_radius);}

  const Params & params() const {return p_;}

  static const char * to_string(Status s);

private:
  Params p_;
};

// Cost per metre of moving through voxel k, before the vertical cost: 1, plus unknown_cost
// if unknown, plus the clearance cost. Shared by A* and the shortcut so they always agree.
double step_weight(const AStarPlanner::Params & params, const DistanceMap & map, const Key & k);

}  // namespace path_planner
