// =============================================================================
// moveit_motion_runtime.cpp
// =============================================================================
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/pilz_sequence_backend.hpp"
#include "trainit_motion_runtime/cartesian_waypoint_backend.hpp"
#include "trainit_motion_runtime/scene_service_names.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/motion_failure.hpp"
#include "trainit_motion_runtime/pose_configuration_search.hpp"

#include <moveit/move_group/capability_names.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/rclcpp_utils.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace trainit
{

using moveit::planning_interface::MoveGroupInterface;

namespace
{
double durationToSec(const builtin_interfaces::msg::Duration& d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

// vNext M3.4: a CIRC aux point closer than this to the goal describes no circle [m]
constexpr double kCircleMinAuxDistance = 1e-6;
}  // namespace

MoveItMotionRuntime::MoveItMotionRuntime(rclcpp::Node::SharedPtr node, Config config)
: node_(std::move(node)), config_(std::move(config))
{
  // vNext M2.7 (namespace fix, ADR-0023 Decision 3): the node's namespace is the move_group
  // namespace. MoveGroupInterface(node, group) meant Options(group) -- an EMPTY move_group
  // namespace, every name at the root whatever the node's namespace. names::append("/", x) ==
  // names::append("", x), so a root runtime's eight names are byte-identical. The string
  // constructor's rclcpp::ok() check is kept (the Options constructor has none).
  if (!rclcpp::ok()) throw std::runtime_error("ROS does not seem to be running");
  if (!(std::isfinite(config_.sequence_result_timeout_s) && config_.sequence_result_timeout_s > 0.0))
    throw std::invalid_argument("sequence_result_timeout_s must be a finite number > 0 (got " +
                                std::to_string(config_.sequence_result_timeout_s) + ")");
  arm_ = std::make_shared<MoveGroupInterface>(
    node_, MoveGroupInterface::Options(config_.group, MoveGroupInterface::ROBOT_DESCRIPTION,
                                       node_->get_namespace()));
  arm_->setPoseReferenceFrame(config_.base_frame);
  arm_->setEndEffectorLink(config_.tip_link);
  robot_model_ = arm_->getRobotModel();
  mgi_goal_position_tolerance_ = arm_->getGoalPositionTolerance();
  mgi_goal_orientation_tolerance_ = arm_->getGoalOrientationTolerance();

  selector_ = std::make_unique<ManualMotionStrategySelector>(
    config_.mode, config_.process_backend, config_.available_pipelines);
  scene_ = std::make_unique<PlanningSceneManager>(node_);
  validator_ = std::make_unique<TrajectoryValidator>(robot_model_, config_.group);

  RCLCPP_INFO(node_->get_logger(),
              "MoveItMotionRuntime ready: group=%s tip=%s base=%s mode=%s backend=%s",
              config_.group.c_str(), config_.tip_link.c_str(), config_.base_frame.c_str(),
              toString(selector_->mode()).c_str(), toString(selector_->backend()).c_str());
}

// --- the runners' MoveIt start-up wait (vNext M2.7, owner decision D1, ADR-0023 Decision 5) -------
std::optional<double> MoveItMotionRuntime::moveItWaitTimeout(const rclcpp::Node::SharedPtr& node,
                                                              std::string& refused)
{
  // The runners declare every override with ITS type (automatically_declare_parameters_from_overrides),
  // so "-p moveit_wait_timeout_s:=60" is an integer and 60.0 a double: both are read here.
  if (!node->has_parameter(kMoveItWaitParam)) node->declare_parameter(kMoveItWaitParam, kMoveItWaitDefaultS);
  const rclcpp::Parameter p = node->get_parameter(kMoveItWaitParam);
  double v = std::nan("");
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) v = static_cast<double>(p.as_int());
  else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) v = p.as_double();
  if (v == kMoveItWaitForever || (std::isfinite(v) && v > 0.0)) return v;
  refused = p.value_to_string() + " (" + rclcpp::to_string(p.get_type()) + ")";
  return std::nullopt;
}

namespace
{
std::string boundText(double timeout_s)
{
  if (timeout_s == MoveItMotionRuntime::kMoveItWaitForever) return "-1, no bound";
  std::ostringstream os;
  os << timeout_s;
  return os.str();
}
}  // namespace

MoveItMotionRuntime::MoveItWait MoveItMotionRuntime::waitForMoveIt(const rclcpp::Node::SharedPtr& node,
                                                                   double timeout_s)
{
  const std::string ns = node->get_namespace();
  // The names the constructor resolves: MoveGroupInterface's names::append(move_group_namespace,
  // ...) with the node's namespace (move_group_interface.cpp:162-167), and PlanningSceneInterface's
  // RELATIVE service names on its own node in the same namespace (planning_scene_interface.cpp:62-65),
  // remapped like every name in this process -- so a service client here reports what it will use.
  const std::string move_name = rclcpp::names::append(ns, move_group::MOVE_ACTION);
  const std::string exec_name = rclcpp::names::append(ns, move_group::EXECUTE_ACTION_NAME);
  auto move = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, move_name);
  auto exec = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(node, exec_name);
  auto get = node->create_client<moveit_msgs::srv::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  auto apply = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);
  const std::vector<std::pair<std::string, std::function<bool()>>> endpoints{
    {"action " + move_name, [&] { return move->action_server_is_ready(); }},
    {"action " + exec_name, [&] { return exec->action_server_is_ready(); }},
    {std::string("service ") + get->get_service_name(), [&] { return get->service_is_ready(); }},
    {std::string("service ") + apply->get_service_name(), [&] { return apply->service_is_ready(); }}};
  const auto start = std::chrono::steady_clock::now();   // wall time: a sim clock may be stopped
  int warned = 0;
  for (;;)
  {
    std::string missing;
    for (const auto& [name, ready] : endpoints)
      if (!ready()) missing += (missing.empty() ? "" : ", ") + name;
    if (missing.empty())
    {
      if (warned > 0)
        RCLCPP_INFO(node->get_logger(), "MoveIt start-up: every endpoint is served (namespace %s)", ns.c_str());
      return MoveItWait::READY;
    }
    if (!rclcpp::ok()) return MoveItWait::INTERRUPTED;
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (timeout_s != kMoveItWaitForever && elapsed >= timeout_s)
    {
      RCLCPP_FATAL(node->get_logger(),
                   "MoveIt start-up: not served after %s s: %s (namespace %s; is this robot's move_group "
                   "running?) -- exit 2 (moveit_wait_timeout_s; -1 waits without a bound)",
                   boundText(timeout_s).c_str(), missing.c_str(), ns.c_str());
      return MoveItWait::TIMED_OUT;
    }
    if (elapsed >= kMoveItWarnPeriodS * (warned + 1))
    {
      ++warned;
      RCLCPP_WARN(node->get_logger(),
                  "MoveIt start-up: still waiting after %d s for %s (namespace %s, moveit_wait_timeout_s %s)",
                  static_cast<int>(kMoveItWarnPeriodS) * warned, missing.c_str(), ns.c_str(),
                  boundText(timeout_s).c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// --- the cross-namespace remap guard (vNext M2.7, ADR-0023 Decision 5; INTEGRATION NS-G) ----------
// The names live here, not in the runners: the M2.5 seam scan forbids MoveIt endpoint literals there.
// The 32 are every robot-scoped name a runner's MoveIt and scene clients resolve (MoveIt 2.5.9), with the
// parameters every generated runner gets (build_move_group_params: the PlanningSceneMonitor's publish_* flags on):
//   - MoveGroupInterface (move_group_interface.cpp:151-181): its four services, its publishers
//     trajectory_execution_event and attached_collision_object, its two actions, and its state
//     monitor's joint_states;
//   - PlanningSceneInterface (planning_scene_interface.cpp:61-65): get/apply_planning_scene, planning_scene;
//   - the PlanningSceneMonitor as PlanningSceneManager starts it (planning_scene_monitor.cpp:562, :1112,
//     :1200-1222, :1262-1280): get_planning_scene; its subscriptions planning_scene, collision_object,
//     planning_scene_world, attached_collision_object and joint_states; its scene publisher
//     monitored_planning_scene, which MoveIt itself starts on the monitor's private node when the
//     publish_planning_scene parameter is true (:259-270, :404, :637-641: TMR never asks for it, but
//     a generated runner's parameters are written under /** and reach that node too); and its octomap monitor's
//     save_map and load_map, services this runner SERVES (occupancy_map_monitor_middleware_handle.cpp:151-157);
//   - TMR's own: the Pilz backend's sequence_move_group action and the two scene-manager clients.
// trainit_run_bt adds ServoTrack's default servo node (bt_nodes.cpp, ServoTrack::tick: the servo_node port's
// reader default and the five names it builds under it; test_m2_6_eef_static_scan pins those literals there)
// and the end-effector contract's names (eefEndpoints, end_effector_registry.cpp). An action is five
// entities: Humble applies no remap of an action NAME (rcl_action resolves each entity on its own), but it
// applies one of each entity, so the five are checked. Not checked, by design: the root; the cell-global
// /tf, /tf_static, /parameter_events, /rosout and /clock (Decision 4); each node's own parameter services;
// the names a tree passes in its ports (a servo node other than the default, a detector, an I/O channel, a
// scene-manager service, an attached topic); the LEGACY absolute endpoints (Decision 6); robot_description
// (a parameter).
namespace
{
using ScopedIds = std::vector<std::pair<std::string, bool>>;   // {relative id, is a service}

void addActionEntities(ScopedIds& ids, const std::string& action)
{
  for (const auto& [entity, service] : std::vector<std::pair<std::string, bool>>{
         {"_action/send_goal", true}, {"_action/cancel_goal", true}, {"_action/get_result", true},
         {"_action/feedback", false}, {"_action/status", false}})
    ids.emplace_back(rclcpp::names::append(action, entity), service);
}

// the 32 both runners resolve, in the M2.7 order
ScopedIds runnerIds()
{
  using Monitor = planning_scene_monitor::PlanningSceneMonitor;
  ScopedIds ids{
    {move_group::CARTESIAN_PATH_SERVICE_NAME, true}, {move_group::QUERY_PLANNERS_SERVICE_NAME, true},
    {move_group::GET_PLANNER_PARAMS_SERVICE_NAME, true}, {move_group::SET_PLANNER_PARAMS_SERVICE_NAME, true},
    {move_group::GET_PLANNING_SCENE_SERVICE_NAME, true}, {move_group::APPLY_PLANNING_SCENE_SERVICE_NAME, true},
    {trainit_scene::kAttachedCollisionCheckServiceDefault, true}, {trainit_scene::kResetSceneServiceDefault, true},
    {"planning_scene", false}, {"joint_states", false},
    {"trajectory_execution_event", false},   // MoveIt's EXECUTION_EVENT_TOPIC; its library is not linked here
    {Monitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, false}, {Monitor::DEFAULT_COLLISION_OBJECT_TOPIC, false},
    {Monitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, false}, {Monitor::MONITORED_PLANNING_SCENE_TOPIC, false},
    {"save_map", true}, {"load_map", true}};
  for (const std::string& action : {move_group::MOVE_ACTION, move_group::EXECUTE_ACTION_NAME,
                                    std::string(PilzSequenceBackend::kActionName)})
    addActionEntities(ids, action);
  return ids;
}

std::vector<std::string> remapsOutOf(const rclcpp::Node::SharedPtr& node, const ScopedIds& ids)
{
  std::vector<std::string> out;
  const std::string ns = node->get_namespace();
  if (ns == "/") return out;
  const auto base = node->get_node_base_interface();
  for (const auto& [id, service] : ids)
  {
    // the name as this robot's runtime builds it, then what the process's remap rules make of it
    const std::string expected = rclcpp::names::append(ns, id);
    const std::string resolved = base->resolve_topic_or_service_name(expected, service);
    if (resolved.rfind(ns + "/", 0) != 0)
      out.push_back(std::string(service ? "service " : "topic ") + expected + " -> " + resolved);
  }
  return out;
}
}  // namespace

std::vector<std::string> MoveItMotionRuntime::crossNamespaceRemaps(const rclcpp::Node::SharedPtr& node)
{
  return remapsOutOf(node, runnerIds());
}

std::vector<std::string> MoveItMotionRuntime::crossNamespaceRemaps(const rclcpp::Node::SharedPtr& node,
                                                                   const eef::Contract& contract)
{
  ScopedIds ids = runnerIds();
  // ServoTrack's default servo node: its twist publisher and status subscription, its three Trigger clients
  ids.insert(ids.end(), {{"servo_node/delta_twist_cmds", false}, {"servo_node/status", false},
                         {"servo_node/start_servo", true}, {"servo_node/pause_servo", true},
                         {"servo_node/unpause_servo", true}});
  for (const EefEndpoint& e : eefEndpoints(contract))
  {
    if (e.kind == EefEndpoint::Kind::ACTION) addActionEntities(ids, e.name);
    else ids.emplace_back(e.name, e.kind == EefEndpoint::Kind::SERVICE);
  }
  return remapsOutOf(node, ids);
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
  // vNext M3.4 (owner D3): a REQUESTED goal tolerance reaches the MoveGroup request (a pose goal's
  // position/orientation constraints); otherwise MoveGroupInterface's own default, set again on every
  // plan so that no move inherits the previous one's. A joint goal keeps MoveGroupInterface's joint
  // tolerance (MoveWaypoint refuses a Cartesian tolerance on a joint target).
  arm_->setGoalPositionTolerance(options.goal_position_tolerance_requested ? options.goal_position_tolerance
                                                                           : mgi_goal_position_tolerance_);
  arm_->setGoalOrientationTolerance(options.goal_orientation_tolerance_requested
                                      ? options.goal_orientation_tolerance
                                      : mgi_goal_orientation_tolerance_);

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
    // NOTE: MoveIt 2.5 ships no CHOMP optimizer request-adapter, so the runtime does not
    // chain OMPL into CHOMP. We run CHOMP standalone (genuinely loaded/executed, visible
    // in the move_group log) and fall back to OMPL if it fails. See
    // config/planner_profiles.yaml (ompl_chomp).
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
    result.phase = MotionPhase::PLAN;   // vNext M2.7: the facts classifyMotionFailure() reads
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
    result.phase = MotionPhase::VALIDATE;
    result.cause = validationCause(vr);
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
  if (!exec_ok) result.phase = MotionPhase::EXECUTE;

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
  // vNext M3.4 (owner D3): a circle point MoveIt cannot use is refused here, before the sequence
  // goal: a non-finite one, or one ON the goal (no circle). What depends on the start (the start's
  // radius about a centre, a start/via/goal line) is Pilz CIRC's to refuse: it knows the start.
  const char* aux_name = target.aux_is_center ? "centre" : "via point";
  auto refused = [&](const std::string& why, const char* cause) {
    RCLCPP_ERROR(node_->get_logger(), "CIRC: %s -- nothing was sent", why.c_str());
    MotionResult r = MotionResult::failure("CIRC: " + why);
    r.phase = MotionPhase::REQUEST;
    r.cause = cause;
    return r;
  };
  if (!(std::isfinite(target.aux.x) && std::isfinite(target.aux.y) && std::isfinite(target.aux.z)))
    return refused(std::string("the ") + aux_name + " is not finite", kCauseInvalidPose);
  const auto& g = target.goal;
  for (const double v : {g.position.x, g.position.y, g.position.z, g.orientation.x, g.orientation.y,
                         g.orientation.z, g.orientation.w})
    if (!std::isfinite(v)) return refused("the goal pose is not finite", kCauseInvalidPose);
  const double dx = target.aux.x - g.position.x, dy = target.aux.y - g.position.y,
               dz = target.aux.z - g.position.z;
  if (!(std::sqrt(dx * dx + dy * dy + dz * dz) >= kCircleMinAuxDistance))
    return refused(std::string("the ") + aux_name + " is on the goal: no circle", kCauseCircleGeometry);

  // Pilz CIRC via a single-item sequence (carries the interim/center point).
  ProcessSegment seg;
  seg.name = "circ";
  seg.type = ProcessSegmentType::CIRCULAR;
  seg.target_pose = target.goal;
  seg.tip_link = target.tip_link;   // vNext M3.4 (owner D3): empty = the configured tip
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
  std::string stage = "collision_free";
  if (target.pose.has_value())
  {
    const std::string tip = target.tip_link.empty() ? config_.tip_link : target.tip_link;
    // TMR 2.6.0 (ADR-0028): the joint configuration is the runtime's choice, never the planner's IK
    // restart -- the same pose from the same start, the same configuration, every cycle
    const bool nearest = config_.pose_goal_configuration == PoseGoalConfiguration::NEAREST;
    const PoseResolution res = nearest ? resolvePoseGoal(*target.pose, tip, target.configuration) : PoseResolution{};
    const char* reference = target.configuration ? "the taught configuration" : "the start state";
    auto refused = [&](const std::string& why, const char* cause) {
      RCLCPP_ERROR(node_->get_logger(), "collision_free: %s -- nothing was sent", why.c_str());
      MotionResult r = MotionResult::failure("collision_free: " + why);
      r.phase = MotionPhase::REQUEST;
      r.cause = cause;
      return r;
    };
    using Status = PoseResolution::Status;
    if (!nearest || res.status == Status::NO_SOLVER)
    {
      if (target.configuration)        // a taught configuration that cannot be honoured: never dropped
        return refused("the pose names its taught configuration, and " +
                       (nearest ? res.why : std::string("pose_goal_configuration is 'planner'")),
                       kCauseConfigurationUnreachable);
      if (nearest && !warned_no_solver_)
      {
        warned_no_solver_ = true;
        RCLCPP_WARN(node_->get_logger(), "collision_free: %s -- a move to a pose leaves its joint "
                    "configuration to the planner (ADR-0028)", res.why.c_str());
      }
      arm_->setPoseTarget(*target.pose, tip);
    }
    else if (res.status == Status::RESOLVED)
    {
      if (!arm_->setJointValueTarget(res.joints))
        return refused("the configuration chosen for the pose is outside the joints' bounds",
                       kCauseInvalidConfiguration);
      if (res.unchecked && !warned_no_scene_)
      {
        warned_no_scene_ = true;
        RCLCPP_WARN(node_->get_logger(), "collision_free: no planning scene to check the pose's joint "
                    "solutions in: the planner checks the move (ADR-0028)");
      }
      RCLCPP_INFO(node_->get_logger(), "collision_free: the pose in the configuration nearest %s (max %.3f rad, "
                  "joint '%s'), %zu distinct solution(s)%s", reference, res.choice.max_difference,
                  res.worst_joint.c_str(), res.choice.distinct,
                  res.timed ? " -- from the time-limited IK search: the choice may differ between runs" : "");
      stage = "collision_free:configuration";
    }
    else if (res.status == Status::NO_STATE)
      return refused(res.why, kCauseStateUnavailable);
    else if (res.status == Status::NO_SOLUTION)
    {
      RCLCPP_ERROR(node_->get_logger(), "collision_free: %s", res.why.c_str());
      MotionResult r = MotionResult::failure("collision_free: " + res.why,
                                             moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
      r.phase = MotionPhase::PLAN;     // this target cannot be reached: the planner's own verdict
      return r;
    }
    else if (res.status == Status::INVALID)
      return refused(res.why, kCauseInvalidConfiguration);
    else
      return refused(res.why, kCauseConfigurationUnreachable);
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
  return planAndExecute(profile, options.motion, intent, stage);
}

bool MoveItMotionRuntime::poseConfigurationParams(const rclcpp::Node::SharedPtr& node, Config& cfg,
                                                  std::string& refused)
{
  if (!node->has_parameter(kPoseGoalConfigurationParam))
    node->declare_parameter(kPoseGoalConfigurationParam, std::string(toString(PoseGoalConfiguration::NEAREST)));
  if (!node->has_parameter(kConfigurationToleranceParam))
    node->declare_parameter(kConfigurationToleranceParam, kDefaultConfigurationTolerance);
  const rclcpp::Parameter mode = node->get_parameter(kPoseGoalConfigurationParam);
  const auto parsed = mode.get_type() == rclcpp::ParameterType::PARAMETER_STRING
                        ? poseGoalConfigurationFromString(mode.as_string()) : std::nullopt;
  if (!parsed)
  {
    refused = std::string(kPoseGoalConfigurationParam) + " " + mode.value_to_string() + " (" + mode.get_type_name() +
              "): 'nearest' or 'planner'";
    return false;
  }
  const rclcpp::Parameter tol = node->get_parameter(kConfigurationToleranceParam);
  double value = std::nan("");
  if (tol.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) value = tol.as_double();
  else if (tol.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) value = static_cast<double>(tol.as_int());
  if (!(std::isfinite(value) && value > 0.0))
  {
    refused = std::string(kConfigurationToleranceParam) + " " + tol.value_to_string() + " (" + tol.get_type_name() +
              "): a finite number of radians > 0";
    return false;
  }
  cfg.pose_goal_configuration = *parsed;
  cfg.configuration_tolerance = value;
  return true;
}

MoveItMotionRuntime::PoseResolution MoveItMotionRuntime::resolvePoseGoal(
  const geometry_msgs::msg::Pose& pose, const std::string& tip,
  const std::optional<std::vector<double>>& taught) const
{
  using Status = PoseResolution::Status;
  PoseResolution out;
  const moveit::core::JointModelGroup* jmg = robot_model_ ? robot_model_->getJointModelGroup(config_.group) : nullptr;
  if (!jmg || !jmg->getSolverInstance())
  {
    out.status = Status::NO_SOLVER;
    out.why = "planning group '" + config_.group + "' has no kinematics solver in this runner";
    return out;
  }
  // the group's variables, in the order copyJointGroupPositions gives them (a mimic one included), each
  // with its own bounds: never the active joints' list, which a mimic joint would shift
  const std::vector<std::string>& names = jmg->getVariableNames();
  const std::size_t n = names.size();
  std::vector<JointBounds> bounds;
  for (const std::string& name : names)
  {
    const moveit::core::VariableBounds& b = robot_model_->getVariableBounds(name);
    const moveit::core::JointModel* jm = robot_model_->getJointOfVariable(name);
    const auto* rev = dynamic_cast<const moveit::core::RevoluteJointModel*>(jm);
    bounds.push_back({b.min_position_, b.max_position_, rev != nullptr && rev->isContinuous(), rev != nullptr});
  }
  if (taught && (taught->size() != n || !allFinite(*taught)))
  {
    out.status = Status::INVALID;
    out.why = taught->size() != n
                ? "the taught configuration has " + std::to_string(taught->size()) + " value(s), and group '" +
                    config_.group + "' has " + std::to_string(n) + " variable(s)"
                : std::string("the taught configuration has a value that is not a finite number");
    return out;
  }
  moveit::core::RobotStatePtr start = arm_->getCurrentState(1.0);
  if (!start)
  {
    out.status = Status::NO_STATE;
    out.why = "no fresh robot state to choose the pose's joint configuration from";
    return out;
  }
  std::vector<double> start_q;
  start->copyJointGroupPositions(jmg, start_q);
  // the pose is given in the runtime's base frame; RobotState IK takes the model frame
  Eigen::Isometry3d goal;
  tf2::fromMsg(pose, goal);
  if (config_.base_frame != robot_model_->getModelFrame())
  {
    if (!start->knowsFrameTransform(config_.base_frame))
    {
      out.status = Status::INVALID;
      out.why = "the base frame '" + config_.base_frame + "' is not a frame of the robot model ('" +
                robot_model_->getModelFrame() + "')";
      return out;
    }
    goal = start->getFrameTransform(config_.base_frame) * goal;
  }

  planning_scene::PlanningScenePtr scene = scene_ ? scene_->snapshotForValidation() : nullptr;
  out.unchecked = !scene;
  const moveit::core::GroupStateValidityCallbackFn collision_free =
    [scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* values) {
      state->setJointGroupPositions(group, values);
      state->update();
      return !scene || !scene->isStateColliding(*state, group->getName());   // the group's links (and held parts)
    };
  const std::vector<double>& reference = taught ? *taught : start_q;
  std::vector<std::vector<double>> head;
  if (taught) head.push_back(*taught);
  head.push_back(start_q);
  // TMR 2.6.2 (D7): with no taught configuration, one KDL attempt per seed -- the choice no longer
  // depends on the clock; a taught configuration keeps the 2.6.0 search (pose_configuration_search.hpp)
  const ConfigurationSearch found =
    findConfigurations(*start, jmg, goal, tip, head, taught.has_value(), collision_free, reference, bounds);
  const std::vector<std::vector<double>>& solutions = found.solutions;
  out.timed = found.timed;
  out.choice = chooseConfiguration(solutions, reference, bounds,
                                   taught ? std::optional<double>(config_.configuration_tolerance) : std::nullopt);
  out.worst_joint = out.choice.worst_joint < n ? names[out.choice.worst_joint] : "?";
  if (out.choice.refused)
  {
    out.status = Status::REFUSED;
    std::ostringstream why;
    why << "the pose is not reached near its taught configuration: the nearest of " << out.choice.distinct
        << " collision-free solution(s) is " << out.choice.max_difference << " rad away on joint '"
        << out.worst_joint << "' (configuration_tolerance " << config_.configuration_tolerance << " rad)";
    out.why = why.str();
    return out;
  }
  if (!out.choice.joints)
  {
    out.status = Status::NO_SOLUTION;
    out.why = "no collision-free joint solution for the pose (its IK seeds, single attempts and timed)";
    return out;
  }
  out.status = Status::RESOLVED;
  out.joints = *out.choice.joints;
  return out;
}

MotionResult MoveItMotionRuntime::executeProcessSequence(
  const std::vector<ProcessSegment>& segments, const MotionOptions& options)
{
  PilzSequenceBackend backend(node_, config_.group, config_.tip_link, config_.base_frame,
                              config_.sequence_result_timeout_s);
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

geometry_msgs::msg::PoseStamped MoveItMotionRuntime::currentTipPose() const
{
  // The read the BT nodes made through moveGroup() before vNext M2.5 (getCurrentPose(tip)): the
  // same MoveGroupInterface and current-state monitor (started on first use), the same 1 s wait,
  // the same FK and stamp. vNext M3.4 (R-M3-20, owner D3): what getCurrentPose turned into the
  // identity pose is now a RobotStateUnavailable naming its reason.
  const moveit::core::RobotStatePtr state = arm_->getCurrentState(1.0);
  if (!state)
    throw RobotStateUnavailable("no fresh robot state within 1 s (group '" + config_.group +
                                "': is joint_states published?)");
  const moveit::core::LinkModel* link = state->getLinkModel(config_.tip_link);
  if (!link)
    throw RobotStateUnavailable("the tip link '" + config_.tip_link + "' is not a link of the robot model");
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = robot_model_->getModelFrame();
  pose.pose = tf2::toMsg(state->getGlobalLinkTransform(link));
  return pose;
}

}  // namespace trainit
