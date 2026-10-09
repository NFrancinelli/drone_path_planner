#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "path_planner/occupancy_map.hpp"

namespace path_planner
{

// 26-connected A* on the map's voxel grid.
// A voxel is blocked if an occupied voxel center is within inflation_radius of its center.
// Unknown voxels cost (1 + unknown_cost) per metre. Climbing or descending costs an extra
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
    Eigen::AlignedBox3d bounds{Eigen::Vector3d(-50.0, -50.0, 0.5), Eigen::Vector3d(50.0, 50.0, 5.0)};
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
  // wall than inflation_radius.
  Result plan(const OccupancyMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const;

  const Params & params() const {return p_;}

  static const char * to_string(Status s);

private:
  Params p_;
};

}  // namespace path_planner
