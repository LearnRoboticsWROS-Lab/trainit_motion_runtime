// =============================================================================
// task_runner.cpp
// =============================================================================
#include "trainit_motion_runtime/task_runner.hpp"

#include <rclcpp/rclcpp.hpp>

namespace trainit
{

TaskRunner::TaskRunner(MoveItMotionRuntime& runtime, GripperController* gripper,
                       ProcessController* process)
: runtime_(runtime), gripper_(gripper), process_(process)
{
}

bool TaskRunner::moveStep(const MotionStep& step)
{
  const std::string tip = runtime_.tipLink();

  switch (step.motion)
  {
    case StepMotion::LINEAR:
    {
      if (!step.pose.has_value())
        return false;
      CartesianTarget t;
      t.pose = *step.pose;
      t.tip_link = tip;
      return runtime_.moveLinear(t, step.options).success;
    }
    case StepMotion::CIRCULAR:
    {
      if (!step.pose.has_value() || !step.circ_aux.has_value())
        return false;
      CircularTarget t;
      t.goal = *step.pose;
      t.aux = *step.circ_aux;
      t.aux_is_center = step.circ_aux_is_center;
      t.tip_link = tip;
      return runtime_.moveCircular(t, step.options).success;
    }
    case StepMotion::PROCESS_PATH:
      return processPathStep(step);

    case StepMotion::FREE:
    case StepMotion::PTP:
    default:
    {
      // Free-space / point-to-point: planner follows the per-step override or the
      // runtime mode (Pilz PTP in pilz mode, OMPL / OMPL+CHOMP otherwise).
      MotionTarget t;
      t.tip_link = tip;
      if (step.pose.has_value())
      {
        t.pose = *step.pose;
      }
      else
      {
        JointTarget jt;
        jt.named_state = step.named_state;
        jt.positions = step.joints;
        t.joint = jt;
      }
      PlanningOptions po;
      po.motion = step.options;
      po.mode = step.planner_override.value_or(runtime_.plannerMode());
      return runtime_.moveCollisionFree(t, po).success;
    }
  }
}

CartesianProcessPath TaskRunner::buildProcessPath(const ProcessPathSpec& s) const
{
  PathFrame f;
  f.reference_frame = runtime_.baseFrame();
  f.tcp_link = runtime_.tipLink();
  f.orientation = task_orientation_;
  f.velocity = s.velocity;
  f.acceleration = s.acceleration;
  switch (s.source)
  {
    case PathSource::POLYLINE:  return PolylinePathGenerator(f, s.points, s.closed).generate();
    case PathSource::CIRCLE:    return CirclePathGenerator(f, s.center, s.radius, s.segments).generate();
    case PathSource::RECTANGLE: return RectanglePathGenerator(f, s.center, s.size_x, s.size_y).generate();
    case PathSource::RASTER:    return RasterPathGenerator(f, s.center, s.size_x, s.size_y, s.rows).generate();
    case PathSource::FILE_YAML: return YamlProcessPathLoader::fromFile(s.file);
    case PathSource::FILE_CSV:  return CsvProcessPathLoader::fromFile(s.file, f);
  }
  return {};
}

bool TaskRunner::processPathStep(const MotionStep& step)
{
  if (!step.process_path.has_value())
  {
    RCLCPP_ERROR(runtime_.node()->get_logger(), "process_path step '%s' has no path spec",
                 step.name.c_str());
    return false;
  }
  const ProcessPathSpec& spec = *step.process_path;

  CartesianProcessPath path;
  try
  {
    path = buildProcessPath(spec);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(runtime_.node()->get_logger(), "process path build failed: %s", e.what());
    return false;
  }

  // process ON (segment-boundary sync; see ProcessController docs)
  if (!spec.process_channel.empty() && process_)
  {
    ProcessEvent on;
    on.type = ProcessEventType::ENABLE_PROCESS;
    on.channel = spec.process_channel;
    on.value = 1.0;
    process_->executeEvent(on);
  }

  MotionResult r = runtime_.executeCartesianProcess(path, step.options);

  // process OFF
  if (!spec.process_channel.empty() && process_)
  {
    ProcessEvent off;
    off.type = ProcessEventType::DISABLE_PROCESS;
    off.channel = spec.process_channel;
    process_->executeEvent(off);
  }
  return r.success;
}

bool TaskRunner::eefAction(const EefAction& eef)
{
  switch (eef.type)
  {
    case EefActionType::GRIPPER:
      if (!gripper_)
      {
        RCLCPP_ERROR(runtime_.node()->get_logger(), "step requests gripper action but no gripper");
        return false;
      }
      return gripper_->command(eef.gripper_position);
    case EefActionType::PROCESS:
      if (!process_)
      {
        RCLCPP_ERROR(runtime_.node()->get_logger(), "step requests process event but no controller");
        return false;
      }
      return process_->executeEvent(eef.process_event);
    case EefActionType::NONE:
    default:
      return true;
  }
}

void TaskRunner::payloadAction(const MotionStep& step)
{
  const std::string link = step.payload.frame.empty() ? runtime_.tipLink() : step.payload.frame;
  switch (step.payload_op)
  {
    case PayloadOp::ATTACH:
      runtime_.scene().attachBox(step.payload.id, link, step.payload.dims, step.payload.pose);
      break;
    case PayloadOp::DETACH:
      runtime_.scene().detachObject(step.payload.id, link);
      break;
    case PayloadOp::NONE:
    default:
      break;
  }
}

bool TaskRunner::run(const MotionTask& task)
{
  auto log = runtime_.node()->get_logger();
  task_orientation_ = task.default_orientation;
  RCLCPP_INFO(log, "=== task '%s': %zu steps (mode=%s) ===",
              task.name.c_str(), task.steps.size(), toString(runtime_.plannerMode()).c_str());

  // static scene (fixtures / surfaces): MoveIt plans blind without it.
  for (const SceneObject& obj : task.scene)
    runtime_.scene().addCollisionBox(obj.id, obj.frame, obj.dims, obj.pose);

  for (std::size_t i = 0; i < task.steps.size(); ++i)
  {
    const MotionStep& step = task.steps[i];
    RCLCPP_INFO(log, "--- step %zu/%zu: %s ---", i + 1, task.steps.size(), step.name.c_str());

    if (!moveStep(step))
    {
      RCLCPP_ERROR(log, "step '%s': MOTION FAILED", step.name.c_str());
      return false;
    }
    if (!eefAction(step.eef))
    {
      RCLCPP_ERROR(log, "step '%s': EEF ACTION FAILED", step.name.c_str());
      return false;
    }
    payloadAction(step);
  }

  RCLCPP_INFO(log, "=== task '%s' COMPLETE ===", task.name.c_str());
  return true;
}

}  // namespace trainit
