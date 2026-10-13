// =============================================================================
// task_runner.hpp — generic executor for a MotionTask.
//
// Application-agnostic: it runs whatever steps the task contains (transfers,
// linear/circular segments, end-effector triggers, payload attach/detach). The
// same runner serves any multi-waypoint end-effector application. It holds no
// poses and no application knowledge.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__TASK_RUNNER_HPP_
#define TRAINIT_MOTION_RUNTIME__TASK_RUNNER_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logger.hpp>

#include "trainit_motion_runtime/motion_runtime.hpp"
#include "trainit_motion_runtime/planning_scene_manager.hpp"
// the compatibility constructor's type (and, as before 2.1.0, for the files that include this one)
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"
#include "trainit_motion_runtime/motion_task.hpp"
#include "trainit_motion_runtime/process_path_generator.hpp"

namespace trainit
{

class TaskRunner
{
public:
  // vNext M2.7 (owner decision 11.2): the runner depends on the MotionRuntime door, the planning
  // scene it adds fixtures and payloads to, and the logger it reports with.
  // gripper / process may be null if the task uses no such end-effector actions.
  TaskRunner(MotionRuntime& runtime, PlanningSceneManager& scene, rclcpp::Logger logger,
             GripperController* gripper = nullptr,
             ProcessController* process = nullptr);
  // Compatibility (TMR <= 2.0.0): the concrete runtime's own scene() and its node's logger, i.e.
  // exactly what the runner used before 2.1.0. Delegates to the constructor above.
  TaskRunner(MoveItMotionRuntime& runtime,
             GripperController* gripper = nullptr,
             ProcessController* process = nullptr);

  bool run(const MotionTask& task);

private:
  bool moveStep(const MotionStep& step);
  bool processPathStep(const MotionStep& step);
  bool eefAction(const EefAction& eef);
  void payloadAction(const MotionStep& step);
  CartesianProcessPath buildProcessPath(const ProcessPathSpec& spec) const;

  MotionRuntime& runtime_;
  PlanningSceneManager& scene_;
  rclcpp::Logger log_;
  GripperController* gripper_;
  ProcessController* process_;
  geometry_msgs::msg::Quaternion task_orientation_;   // shared TCP orientation for generated paths
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__TASK_RUNNER_HPP_
