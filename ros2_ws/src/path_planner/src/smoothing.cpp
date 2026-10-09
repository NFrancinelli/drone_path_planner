#include "path_planner/smoothing.hpp"

#include <algorithm>
#include <cmath>

#include "path_planner/inflated_map.hpp"
#include "trajectory_server/frames.hpp"

namespace path_planner
{

namespace
{

using trajectory_server::QuinticHermite;
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
  const Eigen::Vector3d & b, const AStarPlanner::Params & params)
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
    const double weight = map.at(k) == Occupancy::Unknown ? 1.0 + params.unknown_cost : 1.0;
    check.cost += weight * length / n;
  }
  check.cost += params.vertical_cost * std::abs(b.z() - a.z());
  return check;
}

// Below this a corner isn't worth rounding and the drone just stops there.
constexpr double kMinCornerSpeed = 0.05;  // [m/s]

// Shortest duration for a segment between two knots (zero acceleration at both) that keeps
// speed and acceleration within limits and never moves backwards along the segment.
// Candidates go from 3x the rest-to-rest duration down to a small fraction of it: moving
// knots can need longer (speeding up from rest to a fast corner) or much shorter. The
// rest-to-rest duration itself always fits when both knots are at rest. Returns 0 when
// none fits, i.e. the knot speeds are too high for this segment.
double segment_duration(
  const WaypointTrajectory::Knot & a, const WaypointTrajectory::Knot & b,
  const SmoothingParams & params)
{
  const Eigen::Vector3d delta = b.position - a.position;
  const Eigen::Vector3d dir = delta.normalized();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  // The true peak can fall between samples, so keep a margin; otherwise the full check in
  // smooth() sees it and slows the corner down for nothing.
  constexpr double kMargin = 0.99;
  const auto fits = [&](double T) {
      const QuinticHermite seg(a.position, a.velocity, zero, b.position, b.velocity, zero, T);
      constexpr int kSamples = 64;
      for (int i = 0; i <= kSamples; ++i) {
        const double u = T * i / kSamples;
        const Eigen::Vector3d v = seg.velocity(u);
        if (v.norm() > params.max_vel * kMargin * (1.0 + 1e-9) ||
          seg.acceleration(u).norm() > params.max_acc * kMargin * (1.0 + 1e-9) ||
          v.dot(dir) < -1e-9)
        {
          return false;
        }
      }
      return true;
    };

  const double rest = QuinticLine::min_duration(
    delta.norm(), params.max_vel * kMargin, params.max_acc * kMargin);
  double best = 0.0;
  for (int i = 0; i <= 120; ++i) {
    const double T = 3.0 * rest * std::pow(0.95, i);
    if (fits(T)) {
      best = T;
    } else if (best > 0.0) {
      break;  // the durations that fit form one interval, and we're past its short end
    }
  }
  return best;
}

}  // namespace

std::vector<Eigen::Vector3d> shortcut(
  const OccupancyMap & map, const std::vector<Eigen::Vector3d> & path,
  const AStarPlanner::Params & params)
{
  if (path.size() <= 2) {
    return path;
  }
  const InflatedMap inflated(map, params.inflation_radius);
  const Key exempt = map.key(path.front());

  // Cost of the original path up to each waypoint, measured like the shortcuts.
  std::vector<double> cost_to{0.0};
  for (size_t i = 1; i < path.size(); ++i) {
    cost_to.push_back(
      cost_to.back() + check_segment(inflated, exempt, path[i - 1], path[i], params).cost);
  }

  std::vector<Eigen::Vector3d> out{path.front()};
  size_t i = 0;
  while (i + 1 < path.size()) {
    size_t next = i + 1;
    for (size_t j = path.size() - 1; j > i + 1; --j) {
      const auto seg = check_segment(inflated, exempt, path[i], path[j], params);
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

  // Interior waypoints are corners. A rounded corner gets three knots: an entry point on
  // the incoming leg and an exit point on the outgoing leg, moving along their leg, plus the
  // corner itself, moving along the bisector. The legs between corners then stay exactly
  // straight and the turn happens within `reach` of the corner. A stopped corner is a
  // single knot at rest.
  struct Corner
  {
    Eigen::Vector3d in, out, bisector;
    double speed{0.0}, reach{0.0};
    bool rounded{false};
  };
  std::vector<Corner> corners(n);
  for (size_t k = 1; k + 1 < n; ++k) {
    Corner & c = corners[k];
    const Eigen::Vector3d in = pts[k] - pts[k - 1], out = pts[k + 1] - pts[k];
    c.in = in.normalized();
    c.out = out.normalized();
    if ((c.in + c.out).norm() < 1e-6) {
      continue;  // full reversal, stop there
    }
    c.bisector = (c.in + c.out).normalized();
    c.reach = std::min({params.corner_distance, 0.4 * in.norm(), 0.4 * out.norm()});
    const double turn = (1.0 + c.in.dot(c.out)) / 2.0;  // 1 straight, 0 reversal
    // v^2 <= a * reach: a speed the drone could build up or shed within the rounding.
    c.speed = std::min(params.max_vel * turn, std::sqrt(params.max_acc * c.reach));
    c.rounded = c.speed > kMinCornerSpeed;
  }

  const InflatedMap clearance(map, params.collision_radius);
  const Key exempt = map.key(pts.front());
  const double dt = 0.25 * map.resolution() / params.max_vel;
  std::vector<bool> stopped(n, false);

  for (int iteration = 0; iteration < 50; ++iteration) {
    std::vector<WaypointTrajectory::Knot> knots;
    std::vector<int> owner;  // corner each knot belongs to, -1 for the two ends
    std::vector<size_t> corner_knot(n, 0);  // index of the knot on each corner
    const auto add = [&](const Eigen::Vector3d & p, const Eigen::Vector3d & v, int k) {
        WaypointTrajectory::Knot knot;
        knot.position = p;
        knot.velocity = v;
        knots.push_back(knot);
        owner.push_back(k);
      };
    add(pts.front(), Eigen::Vector3d::Zero(), -1);
    for (size_t k = 1; k + 1 < n; ++k) {
      const Corner & c = corners[k];
      const int id = static_cast<int>(k);
      if (c.rounded) {
        add(pts[k] - c.reach * c.in, c.speed * c.in, id);
        corner_knot[k] = knots.size();
        add(pts[k], c.speed * c.bisector, id);
        add(pts[k] + c.reach * c.out, c.speed * c.out, id);
      } else {
        corner_knot[k] = knots.size();
        add(pts[k], Eigen::Vector3d::Zero(), id);
      }
    }
    add(pts.back(), Eigen::Vector3d::Zero(), -1);
    const size_t m = knots.size();

    // Corners to slow down this iteration. A set, because a corner owns up to three knots
    // and must only be slowed once per pass. Too slow a corner becomes a stop.
    std::vector<bool> slow(n, false);
    const auto mark_slow = [&](int k) {
        if (k >= 0 && corners[k].rounded) {
          slow[k] = true;
        }
      };
    const auto apply_slow = [&]() {
        bool any = false;
        for (size_t k = 0; k < n; ++k) {
          if (slow[k]) {
            corners[k].speed *= 0.7;
            corners[k].rounded = corners[k].speed > kMinCornerSpeed;
            any = true;
          }
        }
        return any;
      };

    std::vector<double> durations(m - 1);
    bool infeasible = false;
    for (size_t j = 0; j + 1 < m; ++j) {
      durations[j] = segment_duration(knots[j], knots[j + 1], params);
      if (durations[j] == 0.0) {
        mark_slow(owner[j]);
        mark_slow(owner[j + 1]);
        infeasible = true;
      }
    }
    if (infeasible) {
      apply_slow();
      continue;
    }

    // Yaw turns to each new leg's heading, timed to be done at the corner, or as soon as the
    // previous turn allows when corners are close. Heading only aims the camera, so a turn
    // that finishes a little late beats slowing the flight down for it.
    std::vector<double> knot_time{0.0};
    for (double T : durations) {
      knot_time.push_back(knot_time.back() + T);
    }
    std::vector<WaypointTrajectory::Turn> turns;
    double yaw = heading.front(), free_from = 0.0;
    for (size_t k = 1; k + 1 < n; ++k) {
      if (std::abs(trajectory_server::wrap_angle(heading[k] - yaw)) < 1e-6) {
        continue;
      }
      const double T = YawTurn::min_duration(yaw, heading[k], params.max_yaw_rate, params.max_yaw_acc);
      const double start = std::max(free_from, knot_time[corner_knot[k]] - T);
      turns.push_back({start, T, heading[k]});
      free_from = start + T;
      yaw = heading[k];
    }

    auto traj = std::make_shared<const WaypointTrajectory>(knots, durations, heading.front(), turns);

    std::vector<bool> too_fast(m - 1, false), collides(m - 1, false);
    size_t seg = 0;
    for (double t = 0.0; ; t = std::min(t + dt, traj->duration())) {
      while (seg + 2 < m && t >= traj->knot_time(seg + 1)) {
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
    for (size_t j = 0; j + 1 < m; ++j) {
      if (collides[j]) {
        bool stopped_any = false;
        for (int k : {owner[j], owner[j + 1]}) {
          if (k >= 0 && corners[k].rounded) {
            corners[k].rounded = false;
            stopped[k] = true;
            stopped_any = true;
          }
        }
        if (!stopped_any) {
          return result;  // a straight leg is too close; the map changed under us
        }
        changed = true;
      } else if (too_fast[j]) {
        mark_slow(owner[j]);
        mark_slow(owner[j + 1]);
      }
    }
    changed = apply_slow() || changed;
    if (!changed) {
      result.trajectory = traj;
      result.stopped_knots = static_cast<int>(std::count(stopped.begin(), stopped.end(), true));
      return result;
    }
  }
  return result;
}

}  // namespace path_planner
