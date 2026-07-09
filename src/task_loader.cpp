// =============================================================================
// task_loader.cpp — YAML -> MotionTask
// =============================================================================
#include "trainit_motion_runtime/task_loader.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace trainit
{
namespace
{

geometry_msgs::msg::Point readPoint(const YAML::Node& n)
{
  geometry_msgs::msg::Point p;
  p.x = n[0].as<double>();
  p.y = n[1].as<double>();
  p.z = n[2].as<double>();
  return p;
}

geometry_msgs::msg::Quaternion readQuat(const YAML::Node& n)
{
  geometry_msgs::msg::Quaternion q;
  q.x = n[0].as<double>();
  q.y = n[1].as<double>();
  q.z = n[2].as<double>();
  q.w = n[3].as<double>();
  return q;
}

std::array<double, 3> readDims(const YAML::Node& n)
{
  return {{n[0].as<double>(), n[1].as<double>(), n[2].as<double>()}};
}

StepMotion parseMotion(const std::string& s)
{
  if (s == "free") return StepMotion::FREE;
  if (s == "ptp") return StepMotion::PTP;
  if (s == "linear" || s == "lin") return StepMotion::LINEAR;
  if (s == "circular" || s == "circ") return StepMotion::CIRCULAR;
  if (s == "process_path" || s == "process") return StepMotion::PROCESS_PATH;
  throw std::runtime_error("unknown motion '" + s + "'");
}

PathSource parsePathSource(const std::string& type, const std::string& format)
{
  if (type == "polyline")  return PathSource::POLYLINE;
  if (type == "circle")    return PathSource::CIRCLE;
  if (type == "rectangle") return PathSource::RECTANGLE;
  if (type == "raster")    return PathSource::RASTER;
  if (type == "file")      return (format == "csv") ? PathSource::FILE_CSV : PathSource::FILE_YAML;
  throw std::runtime_error("unknown process path type '" + type + "'");
}

ProcessEventType parseProcType(const std::string& s)
{
  if (s == "enable") return ProcessEventType::ENABLE_PROCESS;
  if (s == "disable") return ProcessEventType::DISABLE_PROCESS;
  if (s == "trigger") return ProcessEventType::TRIGGER;
  if (s == "set" || s == "set_analog") return ProcessEventType::SET_ANALOG_VALUE;
  throw std::runtime_error("unknown process type '" + s + "'");
}

}  // namespace

MotionTask TaskLoader::fromYamlFile(const std::string& path)
{
  YAML::Node root = YAML::LoadFile(path);

  // directory of the task file, to resolve relative process-path file references
  const std::size_t slash = path.find_last_of('/');
  const std::string task_dir = (slash == std::string::npos) ? "." : path.substr(0, slash);

  MotionTask task;
  task.name       = root["name"]       ? root["name"].as<std::string>()       : "task";
  task.group      = root["group"]      ? root["group"].as<std::string>()      : "";
  task.base_frame = root["base_frame"] ? root["base_frame"].as<std::string>() : "";
  task.tip_link   = root["tip_link"]   ? root["tip_link"].as<std::string>()   : "";

  geometry_msgs::msg::Quaternion default_orientation;
  default_orientation.w = 1.0;
  if (root["orientation"])
    default_orientation = readQuat(root["orientation"]);
  task.default_orientation = default_orientation;

  MotionOptions defaults;
  if (root["defaults"])
  {
    const YAML::Node d = root["defaults"];
    if (d["velocity_scaling"])     defaults.velocity_scaling = d["velocity_scaling"].as<double>();
    if (d["acceleration_scaling"]) defaults.acceleration_scaling = d["acceleration_scaling"].as<double>();
    if (d["planning_time"])        defaults.planning_time = d["planning_time"].as<double>();
    if (d["planning_attempts"])    defaults.planning_attempts = d["planning_attempts"].as<int>();
  }

  if (root["scene"])
  {
    for (const auto& s : root["scene"])
    {
      SceneObject obj;
      obj.id = s["id"].as<std::string>();
      obj.frame = s["frame"] ? s["frame"].as<std::string>() : task.base_frame;
      obj.dims = readDims(s["dims"]);
      obj.pose.orientation.w = 1.0;
      if (s["position"])    obj.pose.position = readPoint(s["position"]);
      if (s["orientation"]) obj.pose.orientation = readQuat(s["orientation"]);
      task.scene.push_back(obj);
    }
  }

  if (!root["steps"])
    throw std::runtime_error("task '" + task.name + "' has no 'steps'");

  for (const auto& sn : root["steps"])
  {
    MotionStep step;
    step.name = sn["name"] ? sn["name"].as<std::string>() : "step";
    step.motion = parseMotion(sn["motion"] ? sn["motion"].as<std::string>() : "free");

    step.options = defaults;
    if (sn["velocity_scaling"])     step.options.velocity_scaling = sn["velocity_scaling"].as<double>();
    if (sn["acceleration_scaling"]) step.options.acceleration_scaling = sn["acceleration_scaling"].as<double>();
    if (sn["planning_time"])        step.options.planning_time = sn["planning_time"].as<double>();
    if (sn["plan_only"])            step.options.plan_only = sn["plan_only"].as<bool>();

    if (sn["planner"])
    {
      auto m = plannerModeFromString(sn["planner"].as<std::string>());
      if (m) step.planner_override = *m;
    }

    // target: named | joints | pose
    if (sn["named"])
    {
      step.named_state = sn["named"].as<std::string>();
    }
    else if (sn["joints"])
    {
      for (const auto& j : sn["joints"])
        step.joints.push_back(j.as<double>());
    }
    else if (sn["position"])
    {
      geometry_msgs::msg::Pose pose;
      pose.position = readPoint(sn["position"]);
      pose.orientation = sn["orientation"] ? readQuat(sn["orientation"]) : default_orientation;
      step.pose = pose;
    }

    if (sn["circ_interim"])
    {
      step.circ_aux = readPoint(sn["circ_interim"]);
      step.circ_aux_is_center = false;
    }
    else if (sn["circ_center"])
    {
      step.circ_aux = readPoint(sn["circ_center"]);
      step.circ_aux_is_center = true;
    }

    if (sn["eef"])
    {
      const YAML::Node e = sn["eef"];
      if (e["gripper"])
      {
        step.eef.type = EefActionType::GRIPPER;
        step.eef.gripper_position = e["gripper"].as<double>();
      }
      else if (e["process"])
      {
        const YAML::Node p = e["process"];
        step.eef.type = EefActionType::PROCESS;
        step.eef.process_event.type =
          parseProcType(p["type"] ? p["type"].as<std::string>() : "trigger");
        step.eef.process_event.channel = p["channel"] ? p["channel"].as<std::string>() : "";
        step.eef.process_event.value = p["value"] ? p["value"].as<double>() : 0.0;
      }
    }

    if (sn["attach"])
    {
      const YAML::Node a = sn["attach"];
      step.payload_op = PayloadOp::ATTACH;
      step.payload.id = a["id"].as<std::string>();
      step.payload.frame = a["link"] ? a["link"].as<std::string>() : task.tip_link;
      step.payload.dims = readDims(a["dims"]);
      step.payload.pose.orientation.w = 1.0;
      if (a["position"]) step.payload.pose.position = readPoint(a["position"]);
    }
    else if (sn["detach"])
    {
      const YAML::Node a = sn["detach"];
      step.payload_op = PayloadOp::DETACH;
      step.payload.id = a["id"].as<std::string>();
      step.payload.frame = a["link"] ? a["link"].as<std::string>() : task.tip_link;
    }

    if (step.motion == StepMotion::PROCESS_PATH && sn["path"])
    {
      const YAML::Node pn = sn["path"];
      ProcessPathSpec spec;
      const std::string type = pn["type"] ? pn["type"].as<std::string>() : "polyline";
      const std::string format = pn["format"] ? pn["format"].as<std::string>() : "yaml";
      spec.source = parsePathSource(type, format);
      if (pn["center"])  spec.center = readPoint(pn["center"]);
      if (pn["radius"])  spec.radius = pn["radius"].as<double>();
      if (pn["size_x"])  spec.size_x = pn["size_x"].as<double>();
      if (pn["size_y"])  spec.size_y = pn["size_y"].as<double>();
      if (pn["segments"]) spec.segments = pn["segments"].as<int>();
      if (pn["rows"])    spec.rows = pn["rows"].as<int>();
      if (pn["closed"])  spec.closed = pn["closed"].as<bool>();
      if (pn["file"])
      {
        spec.file = pn["file"].as<std::string>();
        if (!spec.file.empty() && spec.file.front() != '/')
          spec.file = task_dir + "/" + spec.file;   // resolve relative to the task file
      }
      if (pn["velocity"]) spec.velocity = pn["velocity"].as<double>();
      if (pn["acceleration"]) spec.acceleration = pn["acceleration"].as<double>();
      if (pn["process_channel"]) spec.process_channel = pn["process_channel"].as<std::string>();
      if (pn["points"])
        for (const auto& p : pn["points"])
          spec.points.push_back(readPoint(p));
      step.process_path = spec;
    }

    task.steps.push_back(step);
  }

  return task;
}

}  // namespace trainit
