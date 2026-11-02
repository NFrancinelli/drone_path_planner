#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "path_planner/astar.hpp"
#include "path_planner/smoothing.hpp"

namespace path_planner
{

// Everything a plan depends on besides the OctoMap itself, to replay it offline (see
// tools/plan_replay.cpp): `<name>.txt` next to the map's `<name>.bt`. Plain text, one
// "field values..." line each.
struct PlanInput
{
  AStarPlanner::Params astar;
  SmoothingParams smoothing;
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  StartState start_state;
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> sticky;  // centers of the voxels kept occupied
};

inline bool write_plan_input(const std::string & path, const PlanInput & in)
{
  std::ofstream f(path);
  f.precision(17);
  const auto v = [&](const char * name, const Eigen::Vector3d & x) {
      f << name << ' ' << x.x() << ' ' << x.y() << ' ' << x.z() << '\n';
    };
  const auto & a = in.astar;
  f << "astar " << a.inflation_radius << ' ' << a.unknown_cost << ' ' << a.vertical_cost << ' '
    << a.clearance_radius << ' ' << a.clearance_cost << ' ' << a.max_expansions << '\n';
  f << "shortcut_slack " << a.shortcut_slack << '\n';
  f << "known_radius " << a.known_radius << '\n';
  v("bounds_min", a.bounds.min());
  v("bounds_max", a.bounds.max());
  const auto & s = in.smoothing;
  f << "smoothing " << s.max_vel << ' ' << s.max_acc << ' ' << s.max_yaw_rate << ' '
    << s.max_yaw_acc << ' ' << s.collision_radius << ' ' << s.corner_distance << '\n';
  v("start", in.start);
  v("velocity", in.start_state.velocity);
  v("acceleration", in.start_state.acceleration);
  if (in.start_state.yaw) {
    f << "yaw " << *in.start_state.yaw << '\n';
  }
  f << "yaw_rate " << in.start_state.yaw_rate << '\n';
  v("goal", in.goal);
  for (const auto & c : in.sticky) {
    v("sticky", c);
  }
  return static_cast<bool>(f);
}

inline bool read_plan_input(const std::string & path, PlanInput & in)
{
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  const auto v = [&](Eigen::Vector3d & x) {f >> x.x() >> x.y() >> x.z();};
  std::string field;
  Eigen::Vector3d lo, hi;
  while (f >> field) {
    auto & a = in.astar;
    auto & s = in.smoothing;
    if (field == "astar") {
      f >> a.inflation_radius >> a.unknown_cost >> a.vertical_cost >> a.clearance_radius >>
      a.clearance_cost >> a.max_expansions;
    } else if (field == "shortcut_slack") {
      f >> a.shortcut_slack;
    } else if (field == "known_radius") {
      f >> a.known_radius;
      in.smoothing.known_radius = a.known_radius;
    } else if (field == "bounds_min") {
      v(lo);
    } else if (field == "bounds_max") {
      v(hi);
    } else if (field == "smoothing") {
      f >> s.max_vel >> s.max_acc >> s.max_yaw_rate >> s.max_yaw_acc >> s.collision_radius >>
      s.corner_distance;
    } else if (field == "start") {
      v(in.start);
    } else if (field == "velocity") {
      v(in.start_state.velocity);
    } else if (field == "acceleration") {
      v(in.start_state.acceleration);
    } else if (field == "yaw") {
      double yaw;
      f >> yaw;
      in.start_state.yaw = yaw;
    } else if (field == "yaw_rate") {
      f >> in.start_state.yaw_rate;
    } else if (field == "goal") {
      v(in.goal);
    } else if (field == "sticky") {
      in.sticky.emplace_back();
      v(in.sticky.back());
    } else {
      return false;
    }
  }
  in.astar.bounds = Eigen::AlignedBox3d(lo, hi);
  return true;
}

}  // namespace path_planner
