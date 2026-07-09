// =============================================================================
// bt_profiles.cpp
// =============================================================================
#include "trainit_motion_runtime/bt_profiles.hpp"

#include <yaml-cpp/yaml.h>

namespace trainit
{

namespace
{
MotionOptions optionsFromNode(const YAML::Node& n, const MotionOptions& base)
{
  MotionOptions o = base;
  if (n["velocity_scaling"])           o.velocity_scaling = n["velocity_scaling"].as<double>();
  if (n["acceleration_scaling"])       o.acceleration_scaling = n["acceleration_scaling"].as<double>();
  if (n["planning_time"])              o.planning_time = n["planning_time"].as<double>();
  if (n["planning_attempts"])          o.planning_attempts = n["planning_attempts"].as<int>();
  if (n["goal_position_tolerance"])    o.goal_position_tolerance = n["goal_position_tolerance"].as<double>();
  if (n["goal_orientation_tolerance"]) o.goal_orientation_tolerance = n["goal_orientation_tolerance"].as<double>();
  if (n["retime_with_totg"])           o.retime_with_totg = n["retime_with_totg"].as<bool>();
  if (n["smooth_with_ruckig"])         o.smooth_with_ruckig = n["smooth_with_ruckig"].as<bool>();
  return o;
}
}  // namespace

MotionProfileRegistry loadMotionProfiles(const std::string& yaml_path)
{
  MotionProfileRegistry reg;
  MotionOptions base;  // built-in default (vel/acc 0.1, planning_time 5.0)

  if (!yaml_path.empty())
  {
    try
    {
      YAML::Node root = YAML::LoadFile(yaml_path);
      const YAML::Node profiles = root["motion_profiles"] ? root["motion_profiles"] : root;
      if (profiles["default"])
        base = optionsFromNode(profiles["default"], base);
      reg["default"] = base;
      for (const auto& kv : profiles)
      {
        const std::string name = kv.first.as<std::string>();
        if (name == "default") continue;
        reg[name] = optionsFromNode(kv.second, base);
      }
    }
    catch (const std::exception&)
    {
      reg["default"] = base;  // unreadable file -> just the default
    }
  }

  if (reg.find("default") == reg.end())
    reg["default"] = base;
  return reg;
}

MotionOptions resolveProfile(const MotionProfileRegistry& registry, const std::string& name)
{
  auto it = registry.find(name);
  if (it != registry.end()) return it->second;
  auto def = registry.find("default");
  if (def != registry.end()) return def->second;
  return MotionOptions{};
}

}  // namespace trainit
