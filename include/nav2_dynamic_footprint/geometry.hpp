#ifndef NAV2_DYNAMIC_FOOTPRINT__GEOMETRY_HPP_
#define NAV2_DYNAMIC_FOOTPRINT__GEOMETRY_HPP_

#include <vector>

namespace nav2_dynamic_footprint
{
struct Point
{
  double x;
  double y;
};

// Returns a counterclockwise hull without a repeated closing point.
// Rejects non-finite coordinates and degenerate polygons.
std::vector<Point> convex_hull(std::vector<Point> points);
}  // namespace nav2_dynamic_footprint
#endif  // NAV2_DYNAMIC_FOOTPRINT__GEOMETRY_HPP_
