#include "path_planner/astar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace path_planner
{

namespace
{

struct Step
{
  Key offset;
  double length;  // in voxels
};

std::vector<Step> make_neighbours()
{
  std::vector<Step> steps;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx != 0 || dy != 0 || dz != 0) {
          steps.push_back({Key(dx, dy, dz), std::sqrt(double(dx * dx + dy * dy + dz * dz))});
        }
      }
    }
  }
  return steps;
}

// Offsets to every voxel whose center is within `radius` of the origin voxel's center.
std::vector<Key> make_kernel(double radius, double resolution)
{
  std::vector<Key> kernel;
  const int r = static_cast<int>(std::ceil(radius / resolution));
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dz = -r; dz <= r; ++dz) {
        const Key d(dx, dy, dz);
        if (d.cast<double>().norm() * resolution <= radius) {
          kernel.push_back(d);
        }
      }
    }
  }
  return kernel;
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

  // Inflation computed on demand and cached.
  const std::vector<Key> kernel = make_kernel(p_.inflation_radius, res);
  std::unordered_map<Key, bool, KeyHash> blocked_cache;
  const auto blocked = [&](const Key & k) {
      auto [it, inserted] = blocked_cache.try_emplace(k, false);
      if (inserted) {
        it->second = std::any_of(kernel.begin(), kernel.end(), [&](const Key & d) {
              return map.at(k + d) == Occupancy::Occupied;
            });
      }
      return it->second;
    };

  if (blocked(goal_key)) {
    result.status = Status::GoalBlocked;
    return result;
  }

  // Euclidean, admissible since a step never costs less than its length.
  const auto heuristic = [&](const Key & k) {return (goal_key - k).cast<double>().norm() * res;};

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
      if (closed.count(next) || !in_bounds(next) || blocked(next)) {
        continue;
      }
      const double weight = map.at(next) == Occupancy::Unknown ? 1.0 + p_.unknown_cost : 1.0;
      const double cost = node.g + step.length * res * weight;
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
