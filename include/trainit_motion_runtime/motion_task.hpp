// =============================================================================
// motion_task.hpp — generic, application-AGNOSTIC task description.
//
// A MotionTask is pure data: an ordered list of MotionSteps plus a static scene.
// It expresses "move the end effector through points, with per-segment motion
// type / planner, optionally triggering the end effector and attaching/detaching
// a payload". The SAME structure serves any multi-waypoint end-effector
// application. The framework executes it; the application (or a future setup
// assistant) only PROVIDES it (e.g. from YAML).
//
// Nothing here names a specific application, robot, tool or pose.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_TASK_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_TASK_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

// How to move TO a step's target.
enum class StepMotion
{
  FREE,         // collision-free transfer (planner per mode: OMPL / Pilz PTP / ...)
  PTP,          // point-to-point (joint space)
  LINEAR,       // straight cartesian line (Pilz LIN)
  CIRCULAR,     // cartesian arc (Pilz CIRC)
  PROCESS_PATH  // a dense cartesian process path (the step carries a ProcessPathSpec)
};

// Source of a process path for a PROCESS_PATH step.
enum class PathSource { POLYLINE, CIRCLE, RECTANGLE, RASTER, FILE_YAML, FILE_CSV };

// Declarative process-path: a generator (geometry) or a file. The runtime's
// configured process_path_backend executes it. If process_channel is set, the
// runner enables the process before the path and disables it after.
struct ProcessPathSpec
{
  PathSource source{PathSource::POLYLINE};
  geometry_msgs::msg::Point center;                 // CIRCLE/RECTANGLE/RASTER
  double radius{0.1};                               // CIRCLE
  double size_x{0.2};                               // RECTANGLE/RASTER
  double size_y{0.2};                               // RECTANGLE/RASTER
  int    segments{36};                              // CIRCLE
  int    rows{4};                                   // RASTER
  std::vector<geometry_msgs::msg::Point> points;    // POLYLINE
  bool   closed{false};                             // POLYLINE
  std::string file;                                 // FILE_YAML / FILE_CSV
  double velocity{0.05};
  double acceleration{0.05};
  std::string process_channel;                      // enable/disable around the path (optional)
};

// End-effector action performed AFTER reaching a step's target.
enum class EefActionType
{
  NONE,
  GRIPPER,    // GripperCommand position (open/close, on/off)
  PROCESS     // generic process event (a digital/analog tool output) via ProcessController
};

struct EefAction
{
  EefActionType type{EefActionType::NONE};
  double gripper_position{0.0};   // GRIPPER: 1.0 (close/on) .. 0.0 (open/off)
  ProcessEvent process_event;     // PROCESS
};

// Collision-payload operation at a step (attach grasped object / release it).
enum class PayloadOp { NONE, ATTACH, DETACH };

// A box collision object (world object, or payload attached to a link).
struct SceneObject
{
  std::string id;
  std::string frame;                       // world frame (scene) or link (payload)
  std::array<double, 3> dims{{0.0, 0.0, 0.0}};
  geometry_msgs::msg::Pose pose;
};

struct MotionStep
{
  std::string name;
  StepMotion motion{StepMotion::FREE};

  // Per-segment planner override for FREE/PTP transfers (the configurator choice
  // "go PTP/LIN/CIRC with planner Pilz/OMPL"). Empty => use the runtime mode.
  std::optional<PlannerMode> planner_override;

  // Target — exactly one is used: pose, else named_state, else joints.
  std::optional<geometry_msgs::msg::Pose> pose;
  std::string named_state;
  std::vector<double> joints;

  // CIRCULAR auxiliary point (interim/via or center).
  std::optional<geometry_msgs::msg::Point> circ_aux;
  bool circ_aux_is_center{false};

  EefAction eef;                            // executed after reaching the target

  PayloadOp payload_op{PayloadOp::NONE};
  SceneObject payload;                      // ATTACH dims/pose on link, or DETACH id/link

  std::optional<ProcessPathSpec> process_path;   // only for StepMotion::PROCESS_PATH

  MotionOptions options;
};

struct MotionTask
{
  std::string name;
  std::string group;                        // empty => runtime default
  std::string base_frame;                   // empty => runtime default
  std::string tip_link;                     // empty => runtime default
  geometry_msgs::msg::Quaternion default_orientation;   // shared TCP orientation (identity if unset)
  std::vector<SceneObject> scene;           // world collision objects added before run
  std::vector<MotionStep> steps;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_TASK_HPP_
