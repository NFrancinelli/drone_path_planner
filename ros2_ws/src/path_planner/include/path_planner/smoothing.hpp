#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "path_planner/astar.hpp"
#include "path_planner/distance_map.hpp"
#include "trajectory_server/trajectory.hpp"

namespace path_planner
{

// Greedy shortcut of an A* path: from each kept waypoint, jump to the farthest later one
// whose straight segment is clear of the inflated map and costs no more than the path it
// replaces (same inflation and cost weighting as the A* params, so it never cuts through
// unknown space the planner went around, or closer to obstacles than it went). The start
// voxel is exempt from the clearance check, as in A*.
std::vector<Eigen::Vector3d> shortcut(
  const DistanceMap & map, const std::vector<Eigen::Vector3d> & path,
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

// State a trajectory starts in: at rest by default, or the state of the trajectory being
// flown at the moment the new one takes over from it.
struct StartState
{
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  std::optional<double> yaw;  // [rad]; unset: already facing the first leg
  double yaw_rate{0.0};       // [rad/s]
};

struct SmoothingResult
{
  std::shared_ptr<const trajectory_server::WaypointTrajectory> trajectory;  // null on failure
  int stopped_knots{0};  // corners where the drone stops instead of rounding them
  const char * failure{""};  // why there's no trajectory
};

// Smooth trajectory through shortcut waypoints from `start` (at rest by default) to rest at
// the last waypoint. It passes exactly
// through every waypoint and follows the straight legs except within corner_distance of a
// corner, where it rounds the turn at a speed set by the turn angle. Each segment gets the
// shortest duration that respects the limits.
// The result is sampled against the map with collision_radius: a corner whose rounding
// comes too close to an obstacle is replaced by a stop, which leaves only the straight
// legs the shortcut already checked. Corners over the vel/acc limits are slowed down.
// Yaw starts facing the first leg and turns to face each next leg by the time the drone
// reaches its corner (or as soon as possible after, when corners are close); climbs keep
// the heading. From rest, the caller turns the drone to the first leg's heading beforehand;
// with start.yaw set, the trajectory turns to it first, as fast as the limits allow from
// start.yaw_rate. A moving start leads straight into the first segment, which may bulge
// off the first leg while the velocity swings round (it's collision-checked like the rest).
SmoothingResult smooth(
  const DistanceMap & map, const std::vector<Eigen::Vector3d> & waypoints,
  const SmoothingParams & params, const StartState & start = {});

}  // namespace path_planner
