#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "nav2_dynamic_footprint/geometry.hpp"

TEST(Geometry, CounterclockwiseHullDiscardsDuplicatesAndInteriorPoints)
{
  const auto hull = nav2_dynamic_footprint::convex_hull(
    {{1, 1}, {0, 0}, {1, 0}, {0, 1}, {0.5, 0.5}, {0, 0}, {0.5, 0}});
  ASSERT_EQ(hull.size(), 4u);
  EXPECT_DOUBLE_EQ(hull[0].x, 0);
  EXPECT_DOUBLE_EQ(hull[0].y, 0);
  EXPECT_DOUBLE_EQ(hull[1].x, 1);
  EXPECT_DOUBLE_EQ(hull[1].y, 0);
  EXPECT_DOUBLE_EQ(hull[2].x, 1);
  EXPECT_DOUBLE_EQ(hull[2].y, 1);
  EXPECT_DOUBLE_EQ(hull[3].x, 0);
  EXPECT_DOUBLE_EQ(hull[3].y, 1);
}

TEST(Geometry, RejectsDegenerateAndNonFiniteInputs)
{
  using nav2_dynamic_footprint::convex_hull;
  EXPECT_THROW(convex_hull({}), std::invalid_argument);
  EXPECT_THROW(convex_hull({{0, 0}}), std::invalid_argument);
  EXPECT_THROW(convex_hull({{0, 0}, {1, 0}, {2, 0}}), std::invalid_argument);
  EXPECT_THROW(convex_hull({{0, 0}, {1, 0}, {0, std::numeric_limits<double>::infinity()}}),
    std::invalid_argument);
}
