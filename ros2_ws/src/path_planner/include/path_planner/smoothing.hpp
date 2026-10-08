#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "path_planner/occupancy_map.hpp"
#include "trajectory_server/trajectory.hpp"

namespace path_planner
{

// Greedy shortcut of an A* path: from each kept waypoint, jump to the farthest later one
// whose straight segment is clear of the inflated map and costs no more than the path it
// replaces (same unknown-space weighting as A*, so it never cuts through unknown space the
// planner went around). The start voxel is exempt from the clearance check, as in A*.
std::vector<Eigen::Vector3d> shortcut(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & path,
  double inflation_radius, double unknown_cost);

struct SmoothingParams
{
  double max_vel{1.0};           // [m/s]
  double max_acc{1.0};           // [m/s^2]
  double max_yaw_rate{0.8};      // [rad/s]
  double max_yaw_acc{1.0};       // [rad/s^2]
  double collision_radius{0.4};  // [m] clearance on the uninflated map, <= inflation radius
};

struct SmoothingResult
{
  std::shared_ptr<const trajectory_server::WaypointTrajectory> trajectory;  // null on failure
  int stopped_knots{0};  // corners where the drone stops to stay on the straight segments
};

// Smooth trajectory through shortcut waypoints, at rest at both ends.
// Interior knots get a velocity along the corner's bisector, slower for sharper turns.
// The result is sampled against the map with collision_radius: any segment that bulges
// too close to an obstacle gets both its knots stopped, which makes it exactly the
// straight segment the shortcut already checked. Same for segments over the vel/acc
// limits, whose knot speeds are reduced until they fit.
// Yaw at each knot faces the next segment; vertical segments keep the previous heading.
SmoothingResult smooth(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & waypoints,
  const SmoothingParams & params);

}  // namespace path_planner
