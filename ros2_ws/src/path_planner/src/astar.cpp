#include "path_planner/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>


namespace path_planner
{

namespace
{

struct Step
{
  Key offset;
  double length;  // in voxels
  double climb;   // |dz| in voxels
};

std::vector<Step> make_neighbours()
{
  std::vector<Step> steps;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx != 0 || dy != 0 || dz != 0) {
          steps.push_back(
            {Key(dx, dy, dz), std::sqrt(double(dx * dx + dy * dy + dz * dz)), double(std::abs(dz))});
        }
      }
    }
  }
  return steps;
}

struct OpenEntry
{
  double f;
  double g;
  Key key;
};

// Min-heap on f, ties broken on larger g (fewer expansions in open space).
struct OpenCompare
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const
  {
    return a.f > b.f || (a.f == b.f && a.g < b.g);
  }
};

}  // namespace

AStarPlanner::AStarPlanner(const Params & params)
: p_(params)
{
}

double step_weight(const AStarPlanner::Params & params, const DistanceMap & map, const Key & k)
{
  double weight = map.at(k) == Occupancy::Unknown ? 1.0 + params.unknown_cost : 1.0;
  const double band = params.clearance_radius - params.inflation_radius;
  if (band > 0.0) {
    const double closeness = (params.clearance_radius - map.distance(k)) / band;
    weight += params.clearance_cost * std::clamp(closeness, 0.0, 1.0);
  }
  return weight;
}

AStarPlanner::Result AStarPlanner::plan(
  const OccupancyMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const
{
  return plan(DistanceMap(map, p_.bounds, max_distance()), start, goal);
}

AStarPlanner::Result AStarPlanner::plan(
  const DistanceMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const
{
  static const std::vector<Step> kNeighbours = make_neighbours();
  if (map.max_distance() < max_distance() - 1e-9) {
    throw std::invalid_argument("Distance map too short-ranged for these planner params");
  }

  Result result;
  const double res = map.resolution();
  const Key start_key = map.key(start);
  const Key goal_key = map.key(goal);
  const auto in_bounds = [&](const Key & k) {return p_.bounds.contains(map.center(k));};

  if (!in_bounds(start_key)) {
    result.status = Status::StartOutOfBounds;
    return result;
  }
  if (!in_bounds(goal_key)) {
    result.status = Status::GoalOutOfBounds;
    return result;
  }

  const auto blocked = [&](const Key & k) {return map.distance(k) <= p_.inflation_radius;};
  if (blocked(goal_key)) {
    result.status = Status::GoalBlocked;
    return result;
  }

  // Euclidean plus the height change still to go: admissible, since every path covers at
  // least that distance and that height change, at no less than these costs.
  const auto heuristic = [&](const Key & k) {
      const Key d = goal_key - k;
      return (d.cast<double>().norm() + p_.vertical_cost * std::abs(d.z())) * res;
    };

  // Search state in flat arrays over the distance map's grid (which covers the bounds):
  // much faster than hash maps, and the grid is allocated per plan anyway.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> g(map.cells(), kInf);
  std::vector<int8_t> parent(map.cells(), -1);  // index into kNeighbours of the step in
  std::vector<uint8_t> closed(map.cells(), 0);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCompare> open;
  g[map.index(start_key)] = 0.0;
  open.push({heuristic(start_key), 0.0, start_key});

  while (!open.empty()) {
    const OpenEntry node = open.top();
    open.pop();
    const long i = map.index(node.key);
    if (closed[i]) {
      continue;  // stale entry
    }
    closed[i] = 1;

    if (node.key == goal_key) {
      std::vector<Key> keys{goal_key};
      while (keys.back() != start_key) {
        keys.push_back(keys.back() - kNeighbours[parent[map.index(keys.back())]].offset);
      }
      std::reverse(keys.begin(), keys.end());
      result.path.push_back(start);
      for (size_t i = 1; i + 1 < keys.size(); ++i) {
        result.path.push_back(map.center(keys[i]));
      }
      result.path.push_back(goal);
      result.cost = node.g;
      result.status = Status::Success;
      return result;
    }

    if (++result.expansions > p_.max_expansions) {
      result.status = Status::ExpansionLimit;
      return result;
    }

    for (size_t s = 0; s < kNeighbours.size(); ++s) {
      const Step & step = kNeighbours[s];
      const Key next = node.key + step.offset;
      if (!in_bounds(next)) {
        continue;  // also keeps index() inside the grid, which covers the bounds
      }
      const long j = map.index(next);
      if (closed[j] || blocked(next)) {
        continue;
      }
      const double weight = step_weight(p_, map, next);
      const double cost = node.g + (step.length * weight + p_.vertical_cost * step.climb) * res;
      if (cost >= g[j]) {
        continue;
      }
      g[j] = cost;
      parent[j] = static_cast<int8_t>(s);
      open.push({cost + heuristic(next), cost, next});
    }
  }

  result.status = Status::NoPath;
  return result;
}

const char * AStarPlanner::to_string(Status s)
{
  switch (s) {
    case Status::Success: return "SUCCESS";
    case Status::StartOutOfBounds: return "START_OUT_OF_BOUNDS";
    case Status::GoalOutOfBounds: return "GOAL_OUT_OF_BOUNDS";
    case Status::GoalBlocked: return "GOAL_BLOCKED";
    case Status::NoPath: return "NO_PATH";
    case Status::ExpansionLimit: return "EXPANSION_LIMIT";
  }
  return "?";
}

}  // namespace path_planner
