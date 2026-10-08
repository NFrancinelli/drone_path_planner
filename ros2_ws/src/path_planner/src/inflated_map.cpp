#include "path_planner/inflated_map.hpp"

#include <algorithm>
#include <cmath>

namespace path_planner
{

InflatedMap::InflatedMap(const OccupancyMap & map, double radius)
: map_(map)
{
  const double res = map.resolution();
  const int r = static_cast<int>(std::ceil(radius / res));
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dz = -r; dz <= r; ++dz) {
        const Key d(dx, dy, dz);
        if (d.cast<double>().norm() * res <= radius) {
          kernel_.push_back(d);
        }
      }
    }
  }
}

bool InflatedMap::blocked(const Key & k) const
{
  auto [it, inserted] = cache_.try_emplace(k, false);
  if (inserted) {
    it->second = std::any_of(kernel_.begin(), kernel_.end(), [&](const Key & d) {
          return map_.at(k + d) == Occupancy::Occupied;
        });
  }
  return it->second;
}

}  // namespace path_planner
