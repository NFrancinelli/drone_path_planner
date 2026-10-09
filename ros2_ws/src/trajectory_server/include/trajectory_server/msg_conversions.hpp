#pragma once

#include <memory>
#include <vector>

#include <drone_interfaces/msg/waypoint_trajectory.hpp>

#include "trajectory_server/trajectory.hpp"

namespace trajectory_server
{

// WaypointTrajectory <-> drone_interfaces/WaypointTrajectory. Lossless both ways; the
// header is left to the caller.

inline drone_interfaces::msg::WaypointTrajectory to_msg(const WaypointTrajectory & trajectory)
{
  drone_interfaces::msg::WaypointTrajectory msg;
  const auto & knots = trajectory.knots();
  for (size_t k = 0; k < knots.size(); ++k) {
    drone_interfaces::msg::TrajectoryKnot knot;
    knot.position.x = knots[k].position.x();
    knot.position.y = knots[k].position.y();
    knot.position.z = knots[k].position.z();
    knot.velocity.x = knots[k].velocity.x();
    knot.velocity.y = knots[k].velocity.y();
    knot.velocity.z = knots[k].velocity.z();
    knot.acceleration.x = knots[k].acceleration.x();
    knot.acceleration.y = knots[k].acceleration.y();
    knot.acceleration.z = knots[k].acceleration.z();
    msg.knots.push_back(knot);
    if (k + 1 < knots.size()) {
      msg.durations.push_back(trajectory.segment_duration(k));
    }
  }
  msg.initial_yaw = trajectory.initial_yaw();
  for (const auto & turn : trajectory.turns()) {
    drone_interfaces::msg::YawTurn t;
    t.start = turn.start;
    t.duration = turn.duration;
    t.yaw = turn.yaw;
    msg.turns.push_back(t);
  }
  return msg;
}

// Throws std::invalid_argument if the message doesn't describe a valid trajectory.
inline std::shared_ptr<const WaypointTrajectory> from_msg(
  const drone_interfaces::msg::WaypointTrajectory & msg)
{
  std::vector<WaypointTrajectory::Knot> knots;
  for (const auto & k : msg.knots) {
    WaypointTrajectory::Knot knot;
    knot.position = {k.position.x, k.position.y, k.position.z};
    knot.velocity = {k.velocity.x, k.velocity.y, k.velocity.z};
    knot.acceleration = {k.acceleration.x, k.acceleration.y, k.acceleration.z};
    knots.push_back(knot);
  }
  std::vector<WaypointTrajectory::Turn> turns;
  for (const auto & t : msg.turns) {
    turns.push_back({t.start, t.duration, t.yaw});
  }
  return std::make_shared<const WaypointTrajectory>(
    std::move(knots), msg.durations, msg.initial_yaw, std::move(turns));
}

}  // namespace trajectory_server
