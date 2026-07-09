// =============================================================================
// process_path_loaders.cpp — YAML / CSV -> CartesianProcessPath
// =============================================================================
#include "trainit_motion_runtime/process_path_generator.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace trainit
{

namespace
{
geometry_msgs::msg::Quaternion readQuat(const YAML::Node& n)
{
  geometry_msgs::msg::Quaternion q;
  q.x = n[0].as<double>(); q.y = n[1].as<double>();
  q.z = n[2].as<double>(); q.w = n[3].as<double>();
  return q;
}
}  // namespace

CartesianProcessPath YamlProcessPathLoader::fromFile(const std::string& path)
{
  YAML::Node root = YAML::LoadFile(path);

  CartesianProcessPath p;
  p.path_name = root["path_name"] ? root["path_name"].as<std::string>() : "yaml_path";
  p.reference_frame = root["reference_frame"] ? root["reference_frame"].as<std::string>() : "";
  p.tcp_link = root["tcp_link"] ? root["tcp_link"].as<std::string>() : "";
  p.closed_path = root["closed_path"] ? root["closed_path"].as<bool>() : false;
  p.maintain_orientation = root["maintain_orientation"] ? root["maintain_orientation"].as<bool>() : true;
  p.collision_checking_enabled = root["collision_checking"] ? root["collision_checking"].as<bool>() : true;
  if (root["waypoint_resolution"]) p.waypoint_resolution = root["waypoint_resolution"].as<double>();
  if (root["max_tcp_deviation"]) p.max_tcp_deviation = root["max_tcp_deviation"].as<double>();
  if (root["max_orientation_deviation"]) p.max_orientation_deviation = root["max_orientation_deviation"].as<double>();

  geometry_msgs::msg::Quaternion default_q;
  default_q.w = 1.0;
  if (root["orientation"]) default_q = readQuat(root["orientation"]);

  if (!root["waypoints"])
    throw std::runtime_error("process path '" + p.path_name + "' has no 'waypoints'");

  for (const auto& wn : root["waypoints"])
  {
    CartesianWaypoint wp;
    const YAML::Node pos = wn["position"];
    wp.pose.position.x = pos[0].as<double>();
    wp.pose.position.y = pos[1].as<double>();
    wp.pose.position.z = pos[2].as<double>();
    wp.pose.orientation = wn["orientation"] ? readQuat(wn["orientation"]) : default_q;
    wp.path_velocity = wn["velocity"] ? wn["velocity"].as<double>() : 0.05;
    wp.path_acceleration = wn["acceleration"] ? wn["acceleration"].as<double>() : 0.05;
    wp.process_enabled = wn["process"] ? wn["process"].as<bool>() : false;
    p.waypoints.push_back(wp);
  }

  if (p.closed_path && !p.waypoints.empty())
    p.waypoints.push_back(p.waypoints.front());

  return p;
}

CartesianProcessPath CsvProcessPathLoader::fromFile(const std::string& path, const PathFrame& frame,
                                                    const std::string& name)
{
  std::ifstream file(path);
  if (!file.is_open())
    throw std::runtime_error("cannot open CSV path file: " + path);

  CartesianProcessPath p;
  p.path_name = name;
  p.reference_frame = frame.reference_frame;
  p.tcp_link = frame.tcp_link;
  geometry_msgs::msg::Quaternion q = frame.orientation;
  if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
    q.w = 1.0;

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;
    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> cols;
    while (std::getline(ss, cell, ','))
      cols.push_back(cell);
    if (cols.size() < 3)
      continue;
    try
    {
      CartesianWaypoint wp;
      wp.pose.position.x = std::stod(cols[0]);
      wp.pose.position.y = std::stod(cols[1]);
      wp.pose.position.z = std::stod(cols[2]);
      wp.pose.orientation = q;
      wp.path_velocity = (cols.size() > 3 && !cols[3].empty()) ? std::stod(cols[3]) : frame.velocity;
      wp.path_acceleration = (cols.size() > 4 && !cols[4].empty()) ? std::stod(cols[4]) : frame.acceleration;
      wp.process_enabled = (cols.size() > 5 && (cols[5] == "1" || cols[5] == "true"));
      p.waypoints.push_back(wp);
    }
    catch (const std::exception&)
    {
      // header or malformed line -> skip
      continue;
    }
  }

  if (p.waypoints.size() < 2)
    throw std::runtime_error("CSV path '" + path + "' has fewer than 2 valid waypoints");

  return p;
}

}  // namespace trainit
