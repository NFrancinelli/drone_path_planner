#include "path_planner/smoothing.hpp"

#include <algorithm>
#include <cmath>

#include "path_planner/inflated_map.hpp"

namespace path_planner
{

namespace
{

using trajectory_server::QuinticLine;
using trajectory_server::WaypointTrajectory;
using trajectory_server::YawTurn;

struct SegmentCheck
{
  bool clear{true};
  double cost{0.0};
};

// Samples a -> b every quarter voxel. Cost uses the A* weighting per sampled voxel.
SegmentCheck check_segment(
  const InflatedMap & inflated, const Key & exempt, const Eigen::Vector3d & a,
  const Eigen::Vector3d & b, double unknown_cost)
{
  const OccupancyMap & map = inflated.map();
  const double length = (b - a).norm();
  const int n = std::max(1, static_cast<int>(std::ceil(length / (0.25 * map.resolution()))));
  SegmentCheck check;
  for (int i = 1; i <= n; ++i) {
    const Key k = map.key(a + (b - a) * (static_cast<double>(i) / n));
    if (k != exempt && inflated.blocked(k)) {
      check.clear = false;
    }
    const double weight = map.at(k) == Occupancy::Unknown ? 1.0 + unknown_cost : 1.0;
    check.cost += weight * length / n;
  }
  return check;
}

}  // namespace

std::vector<Eigen::Vector3d> shortcut(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & path,
  double inflation_radius, double unknown_cost)
{
  if (path.size() <= 2) {
    return path;
  }
  const InflatedMap inflated(map, inflation_radius);
  const Key exempt = map.key(path.front());

  // Cost of the original path up to each waypoint, measured like the shortcuts.
  std::vector<double> cost_to{0.0};
  for (size_t i = 1; i < path.size(); ++i) {
    cost_to.push_back(
      cost_to.back() + check_segment(inflated, exempt, path[i - 1], path[i], unknown_cost).cost);
  }

  std::vector<Eigen::Vector3d> out{path.front()};
  size_t i = 0;
  while (i + 1 < path.size()) {
    size_t next = i + 1;
    for (size_t j = path.size() - 1; j > i + 1; --j) {
      const auto seg = check_segment(inflated, exempt, path[i], path[j], unknown_cost);
      if (seg.clear && seg.cost <= cost_to[j] - cost_to[i] + 1e-9) {
        next = j;
        break;
      }
    }
    out.push_back(path[next]);
    i = next;
  }
  return out;
}

SmoothingResult smooth(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & waypoints,
  const SmoothingParams & params)
{
  SmoothingResult result;

  std::vector<Eigen::Vector3d> pts;
  for (const auto & w : waypoints) {
    if (pts.empty() || (w - pts.back()).norm() > 1e-3) {
      pts.push_back(w);
    }
  }
  if (pts.size() < 2) {
    return result;
  }
  const size_t n = pts.size();

  // Heading per segment. Vertical segments keep the previous one (or the first real one).
  std::vector<double> heading(n - 1, 0.0);
  double last = 0.0;
  for (size_t k = n - 1; k-- > 0; ) {
    const Eigen::Vector2d d = (pts[k + 1] - pts[k]).head<2>();
    if (d.norm() > 1e-3) {
      last = std::atan2(d.y(), d.x());
    }
  }
  for (size_t k = 0; k + 1 < n; ++k) {
    const Eigen::Vector2d d = (pts[k + 1] - pts[k]).head<2>();
    if (d.norm() > 1e-3) {
      last = std::atan2(d.y(), d.x());
    }
    heading[k] = last;
  }

  std::vector<WaypointTrajectory::Knot> knots(n);
  std::vector<Eigen::Vector3d> direction(n, Eigen::Vector3d::Zero());
  std::vector<double> speed(n, 0.0);
  for (size_t k = 0; k < n; ++k) {
    knots[k].position = pts[k];
    knots[k].yaw = heading[std::min(k, n - 2)];
  }
  for (size_t k = 1; k + 1 < n; ++k) {
    const Eigen::Vector3d in = pts[k] - pts[k - 1], out = pts[k + 1] - pts[k];
    const Eigen::Vector3d bisector = in.normalized() + out.normalized();
    if (bisector.norm() < 1e-6) {
      continue;  // full reversal, stop there
    }
    direction[k] = bisector.normalized();
    const double turn = (1.0 + in.normalized().dot(out.normalized())) / 2.0;  // 1 straight, 0 reversal
    // v^2 <= a L: a speed the drone could build up or shed over the shorter neighbour.
    speed[k] = std::min(
      params.max_vel * turn, std::sqrt(params.max_acc * std::min(in.norm(), out.norm())));
  }

  std::vector<double> durations(n - 1);
  for (size_t k = 0; k + 1 < n; ++k) {
    durations[k] = std::max(
      QuinticLine::min_duration((pts[k + 1] - pts[k]).norm(), params.max_vel, params.max_acc),
      YawTurn::min_duration(knots[k].yaw, knots[k + 1].yaw, params.max_yaw_rate, params.max_yaw_acc));
  }

  const InflatedMap clearance(map, params.collision_radius);
  const Key exempt = map.key(pts.front());
  const double dt = 0.25 * map.resolution() / params.max_vel;
  std::vector<bool> stopped(n, false);

  for (int iteration = 0; iteration < 50; ++iteration) {
    for (size_t k = 0; k < n; ++k) {
      knots[k].velocity = speed[k] * direction[k];
    }
    auto traj = std::make_shared<const WaypointTrajectory>(knots, durations);

    std::vector<bool> too_fast(n - 1, false), collides(n - 1, false);
    size_t seg = 0;
    for (double t = 0.0; ; t = std::min(t + dt, traj->duration())) {
      while (seg + 2 < n && t >= traj->knot_time(seg + 1)) {
        ++seg;
      }
      const auto p = traj->sample(t);
      if (p.velocity.norm() > params.max_vel * 1.001 ||
        p.acceleration.norm() > params.max_acc * 1.001)
      {
        too_fast[seg] = true;
      }
      const Key key = map.key(p.position);
      if (key != exempt && clearance.blocked(key)) {
        collides[seg] = true;
      }
      if (t >= traj->duration()) {
        break;
      }
    }

    bool changed = false;
    for (size_t k = 0; k + 1 < n; ++k) {
      const bool moving = speed[k] > 0.0 || speed[k + 1] > 0.0;
      if (collides[k]) {
        if (!moving) {
          return result;  // even the straight segment is too close; the map changed under us
        }
        for (size_t j : {k, k + 1}) {
          stopped[j] = stopped[j] || speed[j] > 0.0;
          speed[j] = 0.0;
        }
        changed = true;
      } else if (too_fast[k] && moving) {
        speed[k] *= 0.7;
        speed[k + 1] *= 0.7;
        changed = true;
      }
    }
    if (!changed) {
      result.trajectory = traj;
      result.stopped_knots = static_cast<int>(std::count(stopped.begin(), stopped.end(), true));
      return result;
    }
  }
  return result;
}

}  // namespace path_planner
