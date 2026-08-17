// =============================================================================
// moveit_motion_runtime.cpp
// =============================================================================
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/pilz_sequence_backend.hpp"
#include "trainit_motion_runtime/cartesian_waypoint_backend.hpp"

#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>

namespace trainit
{

using moveit::planning_interface::MoveGroupInterface;

namespace
{
double durationToSec(const builtin_interfaces::msg::Duration& d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}
}  // namespace

MoveItMotionRuntime::MoveItMotionRuntime(rclcpp::Node::SharedPtr node, Config config)
: node_(std::move(node)), config_(std::move(config))
{
  arm_ = std::make_shared<MoveGroupInterface>(node_, config_.group);
  arm_->setPoseReferenceFrame(config_.base_frame);
  arm_->setEndEffectorLink(config_.tip_link);
  robot_model_ = arm_->getRobotModel();

  selector_ = std::make_unique<ManualMotionStrategySelector>(
    config_.mode, config_.process_backend, config_.available_pipelines);
  scene_ = std::make_unique<PlanningSceneManager>(node_);
  validator_ = std::make_unique<TrajectoryValidator>(robot_model_, config_.group);

  RCLCPP_INFO(node_->get_logger(),
              "MoveItMotionRuntime ready: group=%s tip=%s base=%s mode=%s backend=%s",
              config_.group.c_str(), config_.tip_link.c_str(), config_.base_frame.c_str(),
              toString(selector_->mode()).c_str(), toString(selector_->backend()).c_str());
}

void MoveItMotionRuntime::fillFreeSpaceMetrics(
  FreeSpaceMetrics& m, const PlannerProfile& profile, const std::string& stage,
  const moveit_msgs::msg::RobotTrajectory& traj, double planning_time, bool planning_ok) const
{
  m.planner_mode = toString(selector_->mode());
  m.segment_name = stage;
  m.pipeline_id = profile.pipeline_id;
  m.planner_id = profile.planner_id;
  m.planning_success = planning_ok;
  m.planning_duration = planning_time;

  const auto& pts = traj.joint_trajectory.points;
  m.waypoint_count = pts.size();
  if (!pts.empty())
    m.trajectory_duration = durationToSec(pts.back().time_from_start);

  for (std::size_t i = 1; i < pts.size(); ++i)
  {
    double l1 = 0.0, step = 0.0;
    const auto& a = pts[i - 1].positions;
    const auto& b = pts[i].positions;
    const std::size_t k = std::min(a.size(), b.size());
    for (std::size_t j = 0; j < k; ++j)
    {
      const double d = std::abs(b[j] - a[j]);
      l1 += d;
      step = std::max(step, d);
    }
    m.joint_path_length += l1;
    m.maximum_joint_step = std::max(m.maximum_joint_step, step);
  }
}

bool MoveItMotionRuntime::doPlan(const std::string& pipeline, const std::string& planner,
                                 const MotionOptions& options, MoveGroupInterface::Plan& plan,
                                 double& planning_time, int& error_code)
{
  arm_->setPlanningPipelineId(pipeline);
  arm_->setPlannerId(planner);
  arm_->setMaxVelocityScalingFactor(options.velocity_scaling);
  arm_->setMaxAccelerationScalingFactor(options.acceleration_scaling);
  arm_->setPlanningTime(options.planning_time);
  arm_->setNumPlanningAttempts(std::max(1, options.planning_attempts));

  moveit::core::MoveItErrorCode code = arm_->plan(plan);
  planning_time = plan.planning_time_;
  error_code = code.val;
  return static_cast<bool>(code);
}

std::optional<geometry_msgs::msg::Pose>
MoveItMotionRuntime::namedPose(const std::string& named) const
{
  if (!robot_model_) return std::nullopt;
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(config_.group);
  if (!jmg) return std::nullopt;
  moveit::core::RobotState rs(robot_model_);
  rs.setToDefaultValues();
  if (!rs.setToDefaultValues(jmg, named)) return std::nullopt;   // unknown named state
  rs.update();
  return tf2::toMsg(rs.getGlobalLinkTransform(config_.tip_link));
}

MotionResult MoveItMotionRuntime::planAndExecute(
  const PlannerProfile& profile, const MotionOptions& options,
  MotionIntent /*intent*/, const std::string& stage)
{
  MotionResult result;
  MoveGroupInterface::Plan plan;
  double plan_time = 0.0;
  int code_val = 0;
  bool planning_ok = false;

  // Effective pipeline/planner that actually produced the executed trajectory.
  std::string eff_pipeline = profile.pipeline_id;
  std::string eff_planner = profile.planner_id;

  // Retry stochastic planning failures. RRTConnect is randomized, and MoveIt can flag a
  // FOUND path as "invalid (possibly due to postprocessing)" when the time-parameterized
  // resampling grazes an obstacle — a fresh plan almost always clears it. (Pilz PTP/LIN are
  // deterministic, so a retry is harmless and simply won't change the outcome.)
  const int max_attempts = std::max(1, config_.plan_retries + 1);
  for (int attempt = 1; attempt <= max_attempts && !planning_ok; ++attempt)
  {
  if (attempt > 1)
    RCLCPP_WARN(node_->get_logger(), "%s: plan invalid, re-planning (attempt %d/%d)",
                stage.c_str(), attempt, max_attempts);
  if (profile.two_stage_optimize)
  {
    // ompl_chomp: try the CHOMP optimizer pipeline; fall back to OMPL on failure.
    // NOTE: MoveIt 2.5.9 has no CHOMP request-adapter and MotionPlanRequest has no
    // reference_trajectories field, so OMPL cannot SEED CHOMP. We run CHOMP
    // standalone (genuinely loaded/executed, visible in the move_group log) and
    // fall back to OMPL if it fails. See docs/planner_validation.md.
    RCLCPP_INFO(node_->get_logger(), "%s: ompl_chomp -> attempting CHOMP pipeline '%s'",
                stage.c_str(), profile.optimizer_pipeline_id.c_str());
    planning_ok = doPlan(profile.optimizer_pipeline_id, "", options, plan, plan_time, code_val);
    if (planning_ok)
    {
      eff_pipeline = profile.optimizer_pipeline_id;
      eff_planner = "CHOMP";
      RCLCPP_INFO(node_->get_logger(), "%s: CHOMP plan OK", stage.c_str());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "%s: CHOMP failed (code=%d) -> fallback to OMPL",
                  stage.c_str(), code_val);
      planning_ok = doPlan(profile.pipeline_id, profile.planner_id, options, plan, plan_time, code_val);
      eff_pipeline = profile.pipeline_id;
      eff_planner = profile.planner_id;
    }
  }
  else
  {
    planning_ok = doPlan(profile.pipeline_id, profile.planner_id, options, plan, plan_time, code_val);
  }
  }

  const PlannerProfile eff_profile{eff_pipeline, eff_planner, false, ""};
  fillFreeSpaceMetrics(result.free_space, eff_profile, stage, plan.trajectory_, plan_time, planning_ok);
  result.error_code.val = code_val;

  if (!planning_ok)
  {
    result.success = false;
    result.stage = stage + ": PLAN FAILED";
    RCLCPP_ERROR(node_->get_logger(), "%s (pipeline=%s planner=%s, code=%d)", result.stage.c_str(),
                 eff_pipeline.c_str(), eff_planner.c_str(), code_val);
    return result;
  }

  // --- validation (real checks) ---------------------------------------------
  planning_scene::PlanningScenePtr snap = scene_->snapshotForValidation();
  ValidationReport vr = validator_->validate(plan.trajectory_, snap, config_.max_joint_jump);
  RCLCPP_INFO(node_->get_logger(),
              "%s: validation %s (collision_checked=%d max_joint_step=%.3f)",
              stage.c_str(), vr.valid ? "OK" : ("FAILED: " + vr.reason).c_str(),
              vr.collision_checked, vr.max_joint_step);
  if (!vr.valid && config_.enforce_validation)
  {
    result.success = false;
    result.stage = stage + ": VALIDATION FAILED (" + vr.reason + ")";
    return result;
  }

  if (options.plan_only)
  {
    result.success = true;
    result.stage = stage + ": PLANNED (plan_only)";
    result.free_space.execution_success = false;
    return result;
  }

  moveit::core::MoveItErrorCode exec_code = arm_->execute(plan);
  const bool exec_ok = static_cast<bool>(exec_code);
  result.free_space.execution_success = exec_ok;
  result.free_space.error_code = exec_code.val;
  result.success = exec_ok;
  result.error_code.val = exec_code.val;
  result.stage = stage + (exec_ok ? ": OK" : ": EXEC FAILED");

  // --- metrics (Section 20) -------------------------------------------------
  const FreeSpaceMetrics& m = result.free_space;
  RCLCPP_INFO(node_->get_logger(),
              "%s | metrics: pipeline=%s planner=%s plan=%.3fs traj=%.2fs wp=%zu "
              "joint_path=%.3f max_step=%.3f exec=%d",
              result.stage.c_str(), m.pipeline_id.c_str(), m.planner_id.c_str(),
              m.planning_duration, m.trajectory_duration, m.waypoint_count,
              m.joint_path_length, m.maximum_joint_step, m.execution_success);
  return result;
}

MotionResult MoveItMotionRuntime::movePtp(const JointTarget& target, const MotionOptions& options)
{
  PlannerProfile profile = selector_->selectPlanner(MotionIntent::PTP, "ptp");
  arm_->clearPoseTargets();
  std::string stage = "PTP";
  if (!target.named_state.empty())
  {
    if (!arm_->setNamedTarget(target.named_state))
      return MotionResult::failure("PTP: unknown named state '" + target.named_state + "'");
    stage = "PTP:named=" + target.named_state;
  }
  else
  {
    arm_->setJointValueTarget(target.positions);
    stage = "PTP:joints";
  }
  return planAndExecute(profile, options, MotionIntent::PTP, stage);
}

MotionResult MoveItMotionRuntime::moveLinear(const CartesianTarget& target, const MotionOptions& options)
{
  PlannerProfile profile = selector_->selectPlanner(MotionIntent::LINEAR, "lin");
  const std::string tip = target.tip_link.empty() ? config_.tip_link : target.tip_link;
  arm_->clearPoseTargets();
  arm_->setPoseTarget(target.pose, tip);
  return planAndExecute(profile, options, MotionIntent::LINEAR, "LIN");
}

MotionResult MoveItMotionRuntime::moveCircular(const CircularTarget& target,
                                               const MotionOptions& options)
{
  // Pilz CIRC via a single-item sequence (carries the interim/center point).
  ProcessSegment seg;
  seg.name = "circ";
  seg.type = ProcessSegmentType::CIRCULAR;
  seg.target_pose = target.goal;
  if (target.aux_is_center)
    seg.circle_center = target.aux;
  else
    seg.interim_point = target.aux;
  seg.velocity_scaling = options.velocity_scaling;
  seg.acceleration_scaling = options.acceleration_scaling;
  return executeProcessSequence({seg}, options);
}

MotionResult MoveItMotionRuntime::moveCollisionFree(const MotionTarget& target,
                                                    const PlanningOptions& options)
{
  selector_->setMode(options.mode);
  const MotionIntent intent =
    (options.mode == PlannerMode::OMPL_CHOMP) ? MotionIntent::COLLISION_FREE_OPTIMIZED
                                              : MotionIntent::COLLISION_FREE;
  PlannerProfile profile = selector_->selectPlanner(intent, "collision_free");

  arm_->clearPoseTargets();
  if (target.pose.has_value())
  {
    const std::string tip = target.tip_link.empty() ? config_.tip_link : target.tip_link;
    arm_->setPoseTarget(*target.pose, tip);
  }
  else if (target.joint.has_value())
  {
    if (!target.joint->named_state.empty())
      arm_->setNamedTarget(target.joint->named_state);
    else
      arm_->setJointValueTarget(target.joint->positions);
  }
  else
  {
    return MotionResult::failure("moveCollisionFree: no target set");
  }
  return planAndExecute(profile, options.motion, intent, "collision_free");
}

MotionResult MoveItMotionRuntime::executeProcessSequence(
  const std::vector<ProcessSegment>& segments, const MotionOptions& options)
{
  PilzSequenceBackend backend(node_, config_.group, config_.tip_link, config_.base_frame);
  return backend.run(segments, options);
}

MotionResult MoveItMotionRuntime::executeCartesianProcess(
  const CartesianProcessPath& path, const MotionOptions& options)
{
  const ProcessPathBackend backend_kind = selector_->selectProcessPathBackend(path);

  if (backend_kind == ProcessPathBackend::CARTESIAN_WAYPOINTS)
  {
    CartesianWaypointBackend backend(node_, *arm_, robot_model_,
                                     config_.group, config_.tip_link, config_.base_frame);
    return backend.run(path, options, config_.cartesian_fraction_threshold,
                       config_.allow_partial_cartesian);
  }

  // pilz_sequence: execute the path as chained LIN segments (deterministic MoveL).
  std::vector<ProcessSegment> segments;
  segments.reserve(path.waypoints.size());
  for (const auto& wp : path.waypoints)
  {
    ProcessSegment seg;
    seg.name = "lin";
    seg.type = ProcessSegmentType::LINEAR;
    seg.target_pose = wp.pose;
    seg.velocity_scaling = wp.path_velocity > 0.0 ? wp.path_velocity : options.velocity_scaling;
    seg.acceleration_scaling =
      wp.path_acceleration > 0.0 ? wp.path_acceleration : options.acceleration_scaling;
    segments.push_back(seg);
  }
  return executeProcessSequence(segments, options);
}

MotionResult MoveItMotionRuntime::executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  moveit::core::MoveItErrorCode code = arm_->execute(plan);
  if (static_cast<bool>(code))
    return MotionResult::ok("executeTrajectory: OK");
  return MotionResult::failure("executeTrajectory: EXEC FAILED", code.val);
}

MotionResult MoveItMotionRuntime::stop()
{
  arm_->stop();
  return MotionResult::ok("stop");
}

void MoveItMotionRuntime::setPlannerMode(PlannerMode mode)
{
  selector_->setMode(mode);
  config_.mode = selector_->mode();
}

PlannerMode MoveItMotionRuntime::plannerMode() const
{
  return selector_->mode();
}

}  // namespace trainit
