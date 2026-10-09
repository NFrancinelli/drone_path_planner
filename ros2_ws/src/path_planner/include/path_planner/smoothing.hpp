#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "path_planner/astar.hpp"
#include "path_planner/occupancy_map.hpp"
#include "trajectory_server/trajectory.hpp"

namespace path_planner
{

// Greedy shortcut of an A* path: from each kept waypoint, jump to the farthest later one
// whose straight segment is clear of the inflated map and costs no more than the path it
// replaces (same inflation and cost weighting as the A* params, so it never cuts through
// unknown space the planner went around). The start voxel is exempt from the clearance
// check, as in A*.
std::vector<Eigen::Vector3d> shortcut(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & path,
  const AStarPlanner::Params & params);

struct SmoothingParams
{
  double max_vel{1.0};           // [m/s]
  double max_acc{1.0};           // [m/s^2]
  double max_yaw_rate{0.8};      // [rad/s]
  double max_yaw_acc{1.0};       // [rad/s^2]
  double collision_radius{0.4};  // [m] clearance on the uninflated map, <= inflation radius
  double corner_distance{0.75};  // [m] how far before and after a corner the turn starts
};

struct SmoothingResult
{
  std::shared_ptr<const trajectory_server::WaypointTrajectory> trajectory;  // null on failure
  int stopped_knots{0};  // corners where the drone stops instead of rounding them
};

// Smooth trajectory through shortcut waypoints, at rest at both ends. It passes exactly
// through every waypoint and follows the straight legs except within corner_distance of a
// corner, where it rounds the turn at a speed set by the turn angle. Each segment gets the
// shortest duration that respects the limits.
// The result is sampled against the map with collision_radius: a corner whose rounding
// comes too close to an obstacle is replaced by a stop, which leaves only the straight
// legs the shortcut already checked. Corners over the vel/acc limits are slowed down.
// Yaw starts facing the first leg and turns to face each next leg by the time the drone
// reaches its corner (or as soon as possible after, when corners are close); climbs keep
// the heading. The caller turns the drone to the first leg's heading beforehand.
SmoothingResult smooth(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & waypoints,
  const SmoothingParams & params);

}  // namespace path_planner
