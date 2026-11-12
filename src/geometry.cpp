#include "nav2_dynamic_footprint/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav2_dynamic_footprint
{
std::vector<Point> convex_hull(std::vector<Point> points)
{
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw std::invalid_argument("Non-finite footprint coordinate");
    }
  }
  std::sort(points.begin(), points.end(), [](const Point & a, const Point & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  points.erase(std::unique(points.begin(), points.end(), [](const Point & a, const Point & b) {
      return a.x == b.x && a.y == b.y;
  }), points.end());
  const auto cross = [](const Point & a, const Point & b, const Point & c) {
      return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };
  std::vector<Point> hull;
  for (const auto & p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin() + (points.empty() ? 0 : 1); it != points.rend(); ++it) {
    while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  if (hull.size() < 3) {
    throw std::invalid_argument("Footprint must have nonzero area");
  }
  return hull;
}
}  // namespace nav2_dynamic_footprint
