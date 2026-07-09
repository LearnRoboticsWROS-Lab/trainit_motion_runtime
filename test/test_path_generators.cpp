// Unit tests: process path generators (geometry).
#include <gtest/gtest.h>
#include <cmath>
#include "trainit_motion_runtime/process_path_generator.hpp"

using namespace trainit;

namespace
{
geometry_msgs::msg::Point pt(double x, double y, double z)
{
  geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z; return p;
}
}

TEST(Generators, RectangleHasFiveClosedCorners)
{
  PathFrame f;
  RectanglePathGenerator g(f, pt(0.5, 0.0, 0.05), 0.20, 0.10);
  auto path = g.generate();
  ASSERT_EQ(path.waypoints.size(), 5u);          // 4 corners + close
  EXPECT_TRUE(path.closed_path);
  EXPECT_NEAR(path.waypoints.front().pose.position.x, path.waypoints.back().pose.position.x, 1e-9);
  EXPECT_NEAR(path.waypoints.front().pose.position.y, path.waypoints.back().pose.position.y, 1e-9);
}

TEST(Generators, CircleClosedWithCorrectRadius)
{
  PathFrame f;
  CirclePathGenerator g(f, pt(0.5, 0.0, 0.05), 0.10, 24);
  auto path = g.generate();
  ASSERT_EQ(path.waypoints.size(), 25u);         // segments + 1 (closed)
  for (const auto& wp : path.waypoints)
  {
    const double r = std::hypot(wp.pose.position.x - 0.5, wp.pose.position.y - 0.0);
    EXPECT_NEAR(r, 0.10, 1e-6);
    EXPECT_NEAR(wp.pose.position.z, 0.05, 1e-9);
  }
}

TEST(Generators, RasterRowsTimesTwo)
{
  PathFrame f;
  RasterPathGenerator g(f, pt(0.0, 0.0, 0.0), 0.20, 0.20, 4);
  auto path = g.generate();
  EXPECT_EQ(path.waypoints.size(), 8u);          // 4 rows * 2 endpoints, boustrophedon
}

TEST(Generators, PolylineCloseAddsPoint)
{
  PathFrame f;
  std::vector<geometry_msgs::msg::Point> pts = {pt(0, 0, 0), pt(1, 0, 0), pt(1, 1, 0)};
  PolylinePathGenerator g(f, pts, /*closed=*/true);
  auto path = g.generate();
  EXPECT_EQ(path.waypoints.size(), 4u);
  EXPECT_TRUE(path.closed_path);
}

TEST(Generators, DefaultOrientationIsIdentity)
{
  PathFrame f;  // orientation unset (all zero)
  PolylinePathGenerator g(f, {pt(0, 0, 0), pt(1, 0, 0)}, false);
  auto path = g.generate();
  EXPECT_NEAR(path.waypoints[0].pose.orientation.w, 1.0, 1e-9);
}
