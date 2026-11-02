// Replays a plan the planner node dumped (dump_dir parameter) through A*, the shortcut and
// smoothing, and prints every step, to debug a plan offline:
//
//     ros2 run path_planner plan_replay /tmp/plans/plan_0012
//
// reads plan_0012.bt (the OctoMap) and plan_0012.txt (start, goal, parameters, sticky
// voxels). Optional overrides after the name: unknown_cost=<v>, collision_radius=<v>, ...,
// start=<x>,<y>,<z> (from rest), goal=<x>,<y>,<z>, samples=<file> (writes the smoothed trajectory as t,x,y,z,
// map clearance CSV every 0.05 s).

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <octomap/OcTree.h>

#include "path_planner/astar.hpp"
#include "path_planner/octomap_map.hpp"
#include "path_planner/plan_dump.hpp"
#include "path_planner/smoothing.hpp"
#include "path_planner/sticky_map.hpp"

using namespace path_planner;  // NOLINT

namespace
{

void print(const char * label, const DistanceMap & map, const Eigen::Vector3d & p)
{
  const Key k = map.key(p);
  std::printf("  %s (%7.3f, %7.3f, %6.3f)  clearance %.2f%s\n", label, p.x(), p.y(), p.z(),
    map.distance(k), map.at(k) == Occupancy::Unknown ? "  unknown" : "");
}

bool override_param(PlanInput & in, std::string & samples, const std::string & arg)
{
  const auto eq = arg.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  const std::string name = arg.substr(0, eq);
  const double value = std::atof(arg.c_str() + eq + 1);
  if (name == "samples") {
    samples = arg.substr(eq + 1);
  } else if (name == "start") {
    Eigen::Vector3d p;
    if (std::sscanf(arg.c_str() + eq + 1, "%lf,%lf,%lf", &p.x(), &p.y(), &p.z()) != 3) {
      return false;
    }
    in.start = p;
    in.start_state = StartState{};
  } else if (name == "goal") {
    if (std::sscanf(arg.c_str() + eq + 1, "%lf,%lf,%lf", &in.goal.x(), &in.goal.y(),
      &in.goal.z()) != 3)
    {
      return false;
    }
  } else if (name == "unknown_cost") {
    in.astar.unknown_cost = value;
  } else if (name == "inflation_radius") {
    in.astar.inflation_radius = value;
  } else if (name == "shortcut_slack") {
    in.astar.shortcut_slack = value;
  } else if (name == "clearance_cost") {
    in.astar.clearance_cost = value;
  } else if (name == "vertical_cost") {
    in.astar.vertical_cost = value;
  } else if (name == "collision_radius") {
    in.smoothing.collision_radius = value;
  } else if (name == "corner_distance") {
    in.smoothing.corner_distance = value;
  } else if (name == "known_radius") {
    in.astar.known_radius = in.smoothing.known_radius = value;
  } else {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <dump name without extension> [param=value...]\n", argv[0]);
    return 2;
  }
  const std::string name = argv[1];
  PlanInput in;
  std::string samples;
  if (!read_plan_input(name + ".txt", in)) {
    std::fprintf(stderr, "can't read %s.txt\n", name.c_str());
    return 1;
  }
  for (int i = 2; i < argc; ++i) {
    if (!override_param(in, samples, argv[i])) {
      std::fprintf(stderr, "unknown override %s\n", argv[i]);
      return 2;
    }
  }
  auto tree = std::make_shared<octomap::OcTree>(0.1);
  if (!tree->readBinary(name + ".bt")) {
    std::fprintf(stderr, "can't read %s.bt\n", name.c_str());
    return 1;
  }
  const OctomapMap octomap(tree);
  KeySet sticky;
  for (const auto & c : in.sticky) {
    sticky.insert(octomap.key(c));
  }
  const StickyMap map_with_sticky(octomap, sticky);
  const AStarPlanner planner(in.astar);
  const DistanceMap map(map_with_sticky, in.astar.bounds, planner.max_distance());

  std::printf("start -> goal, %zu sticky voxels\n", in.sticky.size());
  print("start", map, in.start);
  print("goal ", map, in.goal);
  const auto result = planner.plan(map, in.start, in.goal);
  std::printf("A*: %s, %zu points, cost %.2f, %d expansions\n",
    AStarPlanner::to_string(result.status), result.path.size(), result.cost, result.expansions);
  if (result.status != AStarPlanner::Status::Success) {
    return 1;
  }
  for (const auto & p : result.path) {
    print("a*", map, p);
  }
  const auto waypoints = shortcut(map, result.path, in.astar);
  std::printf("shortcut: %zu waypoints\n", waypoints.size());
  for (const auto & w : waypoints) {
    print("wp", map, w);
  }
  const auto smoothed = smooth(map, waypoints, in.smoothing, in.start_state);
  if (!smoothed.trajectory) {
    std::printf("smoothing FAILED: %s\n", smoothed.failure);
    return 1;
  }
  const auto & traj = *smoothed.trajectory;
  std::printf("smoothing: %.1f s, %d stopped at\n", traj.duration(), smoothed.stopped_knots);
  double closest = 1e9;
  std::FILE * out = samples.empty() ? nullptr : std::fopen(samples.c_str(), "w");
  if (out) {
    std::fprintf(out, "t,x,y,z,clearance\n");
  }
  for (double t = 0.0; t <= traj.duration(); t += 0.05) {
    const auto s = traj.sample(t);
    const double d = map.distance(map.key(s.position));
    if (out) {
      std::fprintf(out, "%.2f,%.4f,%.4f,%.4f,%.3f\n", t, s.position.x(), s.position.y(),
        s.position.z(), d);
    }
    closest = std::min(closest, d);
    if (d <= in.astar.inflation_radius) {
      std::printf("  t %5.2f (%7.3f, %7.3f, %6.3f)  clearance %.2f  v (%.2f, %.2f, %.2f)\n", t,
        s.position.x(), s.position.y(), s.position.z(), d, s.velocity.x(), s.velocity.y(),
        s.velocity.z());
    }
  }
  for (size_t i = 0; i < traj.knots().size(); ++i) {
    const auto & k = traj.knots()[i];
    std::printf("  knot %zu at %5.2f s (%7.3f, %7.3f, %6.3f)  v (%.2f, %.2f, %.2f)\n", i,
      traj.knot_time(i), k.position.x(), k.position.y(), k.position.z(), k.velocity.x(),
      k.velocity.y(), k.velocity.z());
  }
  if (out) {
    std::fclose(out);
  }
  std::printf("closest sample clearance %.2f\n", closest);
  return 0;
}
