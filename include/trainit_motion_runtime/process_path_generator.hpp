// =============================================================================
// process_path_generator.hpp — reusable cartesian path generators + loaders.
//
// Pure geometry: produce a CartesianProcessPath (ordered TCP poses) in a given
// reference frame with a shared orientation. Used for process applications
// (dispensing, inspection, contouring, ...). A spline-like curve is represented
// here by DENSE sampling — it is NOT a native robot spline.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PROCESS_PATH_GENERATOR_HPP_
#define TRAINIT_MOTION_RUNTIME__PROCESS_PATH_GENERATOR_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "trainit_motion_runtime/motion_task.hpp"

namespace trainit
{

// Frame + orientation shared by all generated waypoints.
struct PathFrame
{
  std::string reference_frame{"base_link"};
  std::string tcp_link{"tcp"};
  geometry_msgs::msg::Quaternion orientation;   // default identity (w=1) if unset
  double velocity{0.05};
  double acceleration{0.05};
};

class ProcessPathGenerator
{
public:
  virtual ~ProcessPathGenerator() = default;
  virtual CartesianProcessPath generate() const = 0;
};

// Ordered points -> waypoints (optionally closed back to the first point).
class PolylinePathGenerator : public ProcessPathGenerator
{
public:
  PolylinePathGenerator(PathFrame frame, std::vector<geometry_msgs::msg::Point> points,
                        bool closed = false, std::string name = "polyline");
  CartesianProcessPath generate() const override;
private:
  PathFrame frame_;
  std::vector<geometry_msgs::msg::Point> points_;
  bool closed_;
  std::string name_;
};

// Circle in the reference-frame XY plane at center.z, sampled into `segments` points.
class CirclePathGenerator : public ProcessPathGenerator
{
public:
  CirclePathGenerator(PathFrame frame, geometry_msgs::msg::Point center, double radius,
                      int segments = 36, std::string name = "circle");
  CartesianProcessPath generate() const override;
private:
  PathFrame frame_;
  geometry_msgs::msg::Point center_;
  double radius_;
  int segments_;
  std::string name_;
};

// Axis-aligned rectangle (XY plane) centered at `center`, closed.
class RectanglePathGenerator : public ProcessPathGenerator
{
public:
  RectanglePathGenerator(PathFrame frame, geometry_msgs::msg::Point center, double size_x,
                         double size_y, std::string name = "rectangle");
  CartesianProcessPath generate() const override;
private:
  PathFrame frame_;
  geometry_msgs::msg::Point center_;
  double size_x_, size_y_;
  std::string name_;
};

// Boustrophedon raster over a rectangle (XY plane), `rows` passes along X.
class RasterPathGenerator : public ProcessPathGenerator
{
public:
  RasterPathGenerator(PathFrame frame, geometry_msgs::msg::Point center, double size_x,
                      double size_y, int rows, std::string name = "raster");
  CartesianProcessPath generate() const override;
private:
  PathFrame frame_;
  geometry_msgs::msg::Point center_;
  double size_x_, size_y_;
  int rows_;
  std::string name_;
};

// File loaders -> CartesianProcessPath (throw std::runtime_error on error).
class YamlProcessPathLoader
{
public:
  static CartesianProcessPath fromFile(const std::string& path);
};

class CsvProcessPathLoader
{
public:
  // CSV columns: x,y,z[,vel,acc,process]. Orientation + frame given by args.
  static CartesianProcessPath fromFile(const std::string& path, const PathFrame& frame,
                                       const std::string& name = "csv_path");
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PROCESS_PATH_GENERATOR_HPP_
