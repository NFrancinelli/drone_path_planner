#include "path_planner/astar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "path_planner/inflated_map.hpp"

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

AStarPlanner::Result AStarPlanner::plan(
  const OccupancyMap & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const
{
  static const std::vector<Step> kNeighbours = make_neighbours();

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

  const InflatedMap inflated(map, p_.inflation_radius);
  if (inflated.blocked(goal_key)) {
    result.status = Status::GoalBlocked;
    return result;
  }

  // Euclidean plus the height change still to go: admissible, since every path covers at
  // least that distance and that height change, at no less than these costs.
  const auto heuristic = [&](const Key & k) {
      const Key d = goal_key - k;
      return (d.cast<double>().norm() + p_.vertical_cost * std::abs(d.z())) * res;
    };

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCompare> open;
  std::unordered_map<Key, double, KeyHash> g{{start_key, 0.0}};
  std::unordered_map<Key, Key, KeyHash> parent;
  std::unordered_set<Key, KeyHash> closed;
  open.push({heuristic(start_key), 0.0, start_key});

  while (!open.empty()) {
    const OpenEntry node = open.top();
    open.pop();
    if (!closed.insert(node.key).second) {
      continue;  // stale entry
    }

    if (node.key == goal_key) {
      std::vector<Key> keys{goal_key};
      while (keys.back() != start_key) {
        keys.push_back(parent.at(keys.back()));
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

    for (const Step & step : kNeighbours) {
      const Key next = node.key + step.offset;
      if (closed.count(next) || !in_bounds(next) || inflated.blocked(next)) {
        continue;
      }
      const double weight = map.at(next) == Occupancy::Unknown ? 1.0 + p_.unknown_cost : 1.0;
      const double cost = node.g + (step.length * weight + p_.vertical_cost * step.climb) * res;
      auto [it, inserted] = g.try_emplace(next, cost);
      if (!inserted && cost >= it->second) {
        continue;
      }
      it->second = cost;
      parent[next] = node.key;
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
