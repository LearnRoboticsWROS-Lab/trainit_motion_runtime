// =============================================================================
// process_path_generators.cpp — Polyline / Circle / Rectangle / Raster
// =============================================================================
#include "trainit_motion_runtime/process_path_generator.hpp"

#include <cmath>

namespace trainit
{

namespace
{
geometry_msgs::msg::Quaternion orientationOf(const PathFrame& f)
{
  geometry_msgs::msg::Quaternion q = f.orientation;
  if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
    q.w = 1.0;  // identity if unset
  return q;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

CartesianProcessPath makePath(const PathFrame& f, const std::string& name,
                              const std::vector<geometry_msgs::msg::Point>& pts, bool closed)
{
  CartesianProcessPath path;
  path.path_name = name;
  path.reference_frame = f.reference_frame;
  path.tcp_link = f.tcp_link;
  path.closed_path = closed;
  path.maintain_orientation = true;
  const auto q = orientationOf(f);
  for (const auto& p : pts)
  {
    CartesianWaypoint wp;
    wp.pose.position = p;
    wp.pose.orientation = q;
    wp.path_velocity = f.velocity;
    wp.path_acceleration = f.acceleration;
    path.waypoints.push_back(wp);
  }
  return path;
}
}  // namespace

// --- Polyline ----------------------------------------------------------------
PolylinePathGenerator::PolylinePathGenerator(PathFrame frame,
                                             std::vector<geometry_msgs::msg::Point> points,
                                             bool closed, std::string name)
: frame_(std::move(frame)), points_(std::move(points)), closed_(closed), name_(std::move(name))
{
}

CartesianProcessPath PolylinePathGenerator::generate() const
{
  std::vector<geometry_msgs::msg::Point> pts = points_;
  if (closed_ && !pts.empty())
    pts.push_back(pts.front());
  return makePath(frame_, name_, pts, closed_);
}

// --- Circle ------------------------------------------------------------------
CirclePathGenerator::CirclePathGenerator(PathFrame frame, geometry_msgs::msg::Point center,
                                         double radius, int segments, std::string name)
: frame_(std::move(frame)), center_(center), radius_(radius),
  segments_(std::max(8, segments)), name_(std::move(name))
{
}

CartesianProcessPath CirclePathGenerator::generate() const
{
  std::vector<geometry_msgs::msg::Point> pts;
  for (int i = 0; i <= segments_; ++i)   // closed loop (last == first)
  {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments_);
    pts.push_back(makePoint(center_.x + radius_ * std::cos(a),
                            center_.y + radius_ * std::sin(a), center_.z));
  }
  return makePath(frame_, name_, pts, true);
}

// --- Rectangle ---------------------------------------------------------------
RectanglePathGenerator::RectanglePathGenerator(PathFrame frame, geometry_msgs::msg::Point center,
                                               double size_x, double size_y, std::string name)
: frame_(std::move(frame)), center_(center), size_x_(size_x), size_y_(size_y),
  name_(std::move(name))
{
}

CartesianProcessPath RectanglePathGenerator::generate() const
{
  const double hx = size_x_ * 0.5, hy = size_y_ * 0.5, z = center_.z;
  std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(center_.x - hx, center_.y - hy, z),
    makePoint(center_.x + hx, center_.y - hy, z),
    makePoint(center_.x + hx, center_.y + hy, z),
    makePoint(center_.x - hx, center_.y + hy, z),
    makePoint(center_.x - hx, center_.y - hy, z),   // close
  };
  return makePath(frame_, name_, pts, true);
}

// --- Raster ------------------------------------------------------------------
RasterPathGenerator::RasterPathGenerator(PathFrame frame, geometry_msgs::msg::Point center,
                                         double size_x, double size_y, int rows, std::string name)
: frame_(std::move(frame)), center_(center), size_x_(size_x), size_y_(size_y),
  rows_(std::max(2, rows)), name_(std::move(name))
{
}

CartesianProcessPath RasterPathGenerator::generate() const
{
  const double hx = size_x_ * 0.5, hy = size_y_ * 0.5, z = center_.z;
  std::vector<geometry_msgs::msg::Point> pts;
  for (int r = 0; r < rows_; ++r)
  {
    const double y = center_.y - hy + size_y_ * static_cast<double>(r) / static_cast<double>(rows_ - 1);
    const double x0 = center_.x - hx, x1 = center_.x + hx;
    if (r % 2 == 0)
    {
      pts.push_back(makePoint(x0, y, z));
      pts.push_back(makePoint(x1, y, z));
    }
    else
    {
      pts.push_back(makePoint(x1, y, z));
      pts.push_back(makePoint(x0, y, z));
    }
  }
  return makePath(frame_, name_, pts, false);
}

}  // namespace trainit
