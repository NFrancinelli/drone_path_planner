#include "path_planner/distance_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace path_planner
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// One line of the squared Euclidean distance transform (Felzenszwalb & Huttenlocher):
// d[q] = min over p of (q - p)^2 + f[p], via the lower envelope of the parabolas rooted at
// the finite f[p]. f and d are strided views into the grid; v and z are scratch space.
void transform_line(
  double * f, long n, long stride, std::vector<double> & d, std::vector<long> & v,
  std::vector<double> & z)
{
  long k = -1;
  for (long q = 0; q < n; ++q) {
    const double fq = f[q * stride];
    if (fq == kInf) {
      continue;  // contributes nothing to the envelope
    }
    if (k < 0) {
      k = 0;
      v[0] = q;
      z[0] = -kInf;
      z[1] = kInf;
      continue;
    }
    double s;
    while (true) {
      const long p = v[k];
      s = ((fq + double(q * q)) - (f[p * stride] + double(p * p))) / double(2 * (q - p));
      if (s > z[k]) {
        break;
      }
      --k;  // never below 0: z[0] = -inf
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  if (k < 0) {
    return;  // no obstacle on this line, all stay infinite
  }
  k = 0;
  for (long q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const long p = v[k];
    d[q] = double((q - p) * (q - p)) + f[p * stride];
  }
  for (long q = 0; q < n; ++q) {
    f[q * stride] = d[q];
  }
}

}  // namespace

DistanceMap::DistanceMap(
  const OccupancyMap & map, const Eigen::AlignedBox3d & region, double max_distance)
: map_(map), max_distance_(max_distance)
{
  if (max_distance < 0.0 || region.isEmpty()) {
    throw std::invalid_argument("DistanceMap needs a non-empty region and max_distance >= 0");
  }
  const Eigen::Vector3d margin = Eigen::Vector3d::Constant(max_distance);
  lo_ = map.key(region.min() - margin);
  size_ = map.key(region.max() + margin) - lo_ + Key::Ones();
  const long total = long(size_.x()) * size_.y() * size_.z();

  occupancy_.resize(total);
  std::vector<double> sq(total);
  long i = 0;
  for (int z = 0; z < size_.z(); ++z) {
    for (int y = 0; y < size_.y(); ++y) {
      for (int x = 0; x < size_.x(); ++x, ++i) {
        occupancy_[i] = map.at(lo_ + Key(x, y, z));
        sq[i] = occupancy_[i] == Occupancy::Occupied ? 0.0 : kInf;
      }
    }
  }

  // Separable: transform every line along x, then y, then z.
  const long nx = size_.x(), ny = size_.y(), nz = size_.z();
  const long longest = std::max({nx, ny, nz});
  std::vector<double> d(longest), z(longest + 1);
  std::vector<long> v(longest);
  for (long b = 0; b < ny * nz; ++b) {
    transform_line(&sq[b * nx], nx, 1, d, v, z);
  }
  for (long zi = 0; zi < nz; ++zi) {
    for (long xi = 0; xi < nx; ++xi) {
      transform_line(&sq[zi * nx * ny + xi], ny, nx, d, v, z);
    }
  }
  for (long b = 0; b < nx * ny; ++b) {
    transform_line(&sq[b], nz, nx * ny, d, v, z);
  }

  const double res = map.resolution();
  distance_.resize(total);
  for (long j = 0; j < total; ++j) {
    distance_[j] = static_cast<float>(std::sqrt(sq[j]) * res);
  }
}

long DistanceMap::index(const Key & k) const
{
  const Key d = k - lo_;
  if ((d.array() < 0).any() || (d.array() >= size_.array()).any()) {
    return -1;
  }
  return (long(d.z()) * size_.y() + d.y()) * size_.x() + d.x();
}

Occupancy DistanceMap::at(const Key & k) const
{
  const long i = index(k);
  return i < 0 ? map_.at(k) : occupancy_[i];
}

double DistanceMap::distance(const Key & k) const
{
  const long i = index(k);
  return i < 0 ? 0.0 : distance_[i];
}

}  // namespace path_planner
