// =============================================================================
// bt_nodes.cpp — generic BT nodes wrapping the TrainIt runtime.
// =============================================================================
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/detection_candidates.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/motion_failure.hpp"
#include "trainit_motion_runtime/process_path_generator.hpp"
#include "trainit_motion_runtime/scene_service_names.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <set>

namespace trainit
{

// ─── helpers ────────────────────────────────────────────────────────────────
namespace
{
BtContext* ctxOf(const BT::NodeConfig& cfg)
{
  return cfg.blackboard->get<BtContext*>(BB_BT_CONTEXT);
}

MotionOptions profileOf(const BT::NodeConfig& cfg, const std::string& name)
{
  const auto& reg = cfg.blackboard->get<MotionProfileRegistry>(BB_MOTION_PROFILES);
  return resolveProfile(reg, name);
}

std::vector<double> parseVec(const std::string& s)
{
  std::vector<double> out;
  std::istringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ';'))
    if (!tok.empty()) out.push_back(std::stod(tok));
  return out;
}

std::array<double, 3> parseXyz3(const std::string& s)
{
  auto v = parseVec(s);
  if (v.size() != 3) throw std::runtime_error("expected 3 values, got " + std::to_string(v.size()));
  return {v[0], v[1], v[2]};
}

Eigen::Isometry3d isometryFromMsg(const geometry_msgs::msg::Pose& p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

geometry_msgs::msg::Pose makePose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.x(); p.position.y = t.y(); p.position.z = t.z();
  Eigen::Quaterniond qn = q.normalized();
  p.orientation.x = qn.x(); p.orientation.y = qn.y(); p.orientation.z = qn.z(); p.orientation.w = qn.w();
  return p;
}

Eigen::Quaterniond quatFromRpyDeg(const std::array<double, 3>& rpy_deg)
{
  const double r = rpy_deg[0] * M_PI / 180.0, p = rpy_deg[1] * M_PI / 180.0, y = rpy_deg[2] * M_PI / 180.0;
  Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
  q.normalize();
  return q;
}

geometry_msgs::msg::Pose applyTcpOffset(const geometry_msgs::msg::Pose& base,
                                        const std::array<double, 3>& xyz,
                                        const std::array<double, 3>& rpy_deg)
{
  Eigen::Isometry3d off = Eigen::Isometry3d::Identity();
  off.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  off.linear() = quatFromRpyDeg(rpy_deg).toRotationMatrix();
  Eigen::Isometry3d out = isometryFromMsg(base) * off;
  return makePose(Eigen::Vector3d(out.translation()), Eigen::Quaterniond(out.rotation()));
}

geometry_msgs::msg::Pose offsetInToolFrame(const geometry_msgs::msg::Pose& in, double dx, double dy, double dz)
{
  Eigen::Isometry3d T = isometryFromMsg(in);
  T.translation() += T.linear() * Eigen::Vector3d(dx, dy, dz);
  return makePose(Eigen::Vector3d(T.translation()), Eigen::Quaterniond(T.rotation()));
}

geometry_msgs::msg::Pose poseFromXyz(const std::array<double, 3>& p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2];
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Pose currentPose(BtContext* ctx)
{
  return ctx->runtime->currentTipPose().pose;
}

// read a string input port, returning a fallback if unresolved/empty
std::string in(BT::TreeNode& n, const char* port, const std::string& def = "")
{
  auto e = n.getInput<std::string>(port);
  return (e && !e.value().empty()) ? e.value() : def;
}

// Read an INTEGER input port, whether it is written as a literal (index="2") or as
// a blackboard reference (index="{i}") whose entry is held as an int (what a
// `Script` node writes) or as a string. Returns false when the port is absent or
// empty, so the caller keeps its own default.
//
// The port must be DECLARED int as well (see DetectObject/SetWaypointFromDetection):
// a std::string port REFUSES, at tree CREATION, to bind an entry already typed int
// — "the port [i] was initially created with type [int]". Reading a number through a
// string port is what forces that refusal, so a counter port does neither.
bool intIn(BT::TreeNode& n, const char* port, int& out)
{
  auto e = n.getInput<int>(port);
  if (!e) return false;
  out = e.value();
  return true;
}

rclcpp::Logger nlog(const BT::NodeConfig& cfg)
{
  return ctxOf(cfg)->node->get_logger();
}

// A logger that works WITHOUT a BtContext, so a node can report a DATA error
// (a bad waypoint field) before it touches the runtime stack — and so such a
// node is unit-testable with a bare tree.
rclcpp::Logger safeLog(const BT::NodeConfig& cfg)
{
  try { auto* c = cfg.blackboard->get<BtContext*>(BB_BT_CONTEXT); if (c && c->node) return c->node->get_logger(); }
  catch (const std::exception&) {}
  return rclcpp::get_logger("trainit_bt");
}

// --- the `motion` and `execution` vocabularies of a data-driven waypoint ------
// Both are CLOSED sets. An unrecognised value must FAIL the node, never fall
// through to a default move: a typo ("srvo", "linaer") would otherwise plan and
// EXECUTE a different motion than the project asked for, silently (ADR-0015).
// An ABSENT or EMPTY value is not an error — it is the documented default
// (motion=free, execution=plan), which is what a generated bundle relies on:
// the TSA emits the key only when it is set.
constexpr const char* kMotionAccepted    = "ptp | free | lin (linear) | circ (circular)";
constexpr const char* kExecutionAccepted = "plan | servo";

bool isKnownMotion(const std::string& m)
{
  return m.empty() || m == "ptp" || m == "free" ||
         m == "lin" || m == "linear" || m == "circ" || m == "circular";
}

// Resolve the motion options for a move: start from the named `profile`, then
// apply optional per-move speed/accel OVERRIDES given as PERCENT (1..100).
// speed=80 -> velocity_scaling 0.80. This is the application "Set_Speed(80)".
MotionOptions resolveOpts(BT::TreeNode& node, const BT::NodeConfig& cfg, const std::string& default_profile)
{
  MotionOptions o = profileOf(cfg, in(node, "profile", default_profile));
  const std::string sp = in(node, "speed");
  if (!sp.empty())
    try { o.velocity_scaling = std::clamp(std::stod(sp) / 100.0, 0.01, 1.0); } catch (const std::exception&) {}
  const std::string ac = in(node, "accel");
  if (!ac.empty())
    try { o.acceleration_scaling = std::clamp(std::stod(ac) / 100.0, 0.01, 1.0); } catch (const std::exception&) {}
  return o;
}
}  // namespace

// ═══ motion ═══════════════════════════════════════════════════════════════════
// Data-driven waypoint: the ONLY port is the waypoint NAME; the full spec
// (type, position/orientation or joints/named, motion, planner, speed) is read
// from the blackboard keys "<name>.<field>" loaded from bt_params task_parameters.
// This is what a setup assistant generates: a uniform tree + per-point data.
//
// The `motion` and `execution` fields are VALIDATED before anything is planned:
// the dispatch below ends in an `else` (moveCollisionFree), so without this guard a
// typo ("srvo") would plan and execute a free-space move instead of failing — a
// silent wrong move. Validation happens before the runtime is touched, so the
// failure is reportable (and testable) without a live MoveIt stack.
//
// vNext M2.7 (owner Q8.2): the optional output `fault` says a FAILED move was not this target's
// failure (motion_failure.hpp: the motion infrastructure, the robot or scene state, the configuration,
// an exception) -- "MOTION_FAULT: <CAUSE>[ during <phase>]". It is written ONLY then: never on SUCCESS,
// never when the target itself cannot be reached (no IK, no plan, the goal in collision, a joint jump).
// A tree that does not remap it is unaffected: nothing is written, nothing more is logged.
BT::PortsList MoveWaypoint::providedPorts()
{ return {BT::InputPort<std::string>("waypoint"),
          // two arguments = a DESCRIPTION and no default: an unremapped port writes nothing
          BT::OutputPort<std::string>("fault", "MOTION_FAULT: CAUSE during PHASE")}; }
BT::NodeStatus MoveWaypoint::tick()
{
  const std::string wp = in(*this, "waypoint");
  // vNext M2.7: every write of `fault` goes through here, and none throws out of tick(). An unremapped
  // port is refused without a write (false). setOutput THROWS when the remapped key already holds a
  // value of another type (BT.CPP Blackboard::set; e.g. a Script wrote a number into it): that write
  // is dropped with an ERROR line, and the node still returns FAILURE.
  auto writeFault = [this, &wp](const std::string& text) {
    try { return setOutput("fault", text).has_value(); }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(safeLog(config()), "MoveWaypoint '%s': its fault output was not written (%s): %s",
                   wp.c_str(), e.what(), text.c_str());
      return false;
    }
  };
  // vNext M2.7: a refusal before the runtime is asked is the configuration's, never the target's.
  auto refused = [&writeFault](const char* cause) {
    MotionResult r; r.phase = MotionPhase::REQUEST; r.cause = cause;
    writeFault(motionFaultText(r));                    // refused (no write) on an unremapped port
    return BT::NodeStatus::FAILURE;
  };
  if (wp.empty()) { RCLCPP_ERROR(safeLog(config()), "MoveWaypoint: empty 'waypoint'"); return refused(kCauseEmptyWaypoint); }

  auto field = [&](const std::string& f, const std::string& def = "") -> std::string {
    try { return config().blackboard->get<std::string>(wp + "." + f); }
    catch (const std::exception&) { return def; }
  };

  const std::string type    = field("type", "tcp");
  const std::string motion  = field("motion", "free");
  const std::string planner = field("planner");
  const std::string speed   = field("speed");
  const std::string accel   = field("accel");
  const std::string exec    = field("execution");

  if (!isKnownMotion(motion))
  {
    RCLCPP_ERROR(safeLog(config()), "MoveWaypoint '%s': unknown motion '%s' — accepted: %s. "
                 "Refusing to move (an unrecognised motion must not fall back to a free-space move).",
                 wp.c_str(), motion.c_str(), kMotionAccepted);
    return refused(kCauseUnknownMotion);
  }
  // `execution` is the ADR-0015 axis, and it is NOT this node's to run: a streamed
  // move belongs to ServoTrack, which owns the controller while it streams. Refuse
  // rather than plan it, so a bundle that asks for servo cannot silently get a plan.
  if (!exec.empty() && exec != "plan")
  {
    if (exec == "servo")
      RCLCPP_ERROR(safeLog(config()), "MoveWaypoint '%s': execution='servo' is streamed by the "
                   "ServoTrack node, not by MoveWaypoint — the tree must call "
                   "<ServoTrack waypoint=\"%s\"/> for this waypoint (ADR-0015)", wp.c_str(), wp.c_str());
    else
      RCLCPP_ERROR(safeLog(config()), "MoveWaypoint '%s': unknown execution '%s' — accepted: %s",
                   wp.c_str(), exec.c_str(), kExecutionAccepted);
    return refused(exec == "servo" ? kCauseServoExecution : kCauseUnknownExecution);
  }

  auto* ctx = ctxOf(config());

  // base profile by motion type, then per-waypoint speed/accel overrides (percent).
  const std::string base_profile = (motion == "lin" || motion == "circ") ? "approach" : "transit";
  MotionOptions opts = profileOf(config(), base_profile);
  if (!speed.empty()) try { opts.velocity_scaling = std::clamp(std::stod(speed) / 100.0, 0.01, 1.0); } catch (const std::exception&) {}
  if (!accel.empty()) try { opts.acceleration_scaling = std::clamp(std::stod(accel) / 100.0, 0.01, 1.0); } catch (const std::exception&) {}

  auto buildPose = [&]() -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    auto pos = parseXyz3(field("position"));
    p.position.x = pos[0]; p.position.y = pos[1]; p.position.z = pos[2];
    auto q = parseVec(field("orientation"));
    if (q.size() != 4) throw std::runtime_error("waypoint '" + wp + "' needs orientation qx;qy;qz;qw");
    p.orientation.x = q[0]; p.orientation.y = q[1]; p.orientation.z = q[2]; p.orientation.w = q[3];
    return p;
  };

  // LIN/CIRC need a TCP pose. A waypoint captured as JOINTS (blind mode, type=named)
  // has none -> resolve it via FK, so a linear approach to a captured pick works
  // instead of failing "needs orientation".
  auto cartesianGoal = [&]() -> geometry_msgs::msg::Pose {
    if (!field("position").empty()) return buildPose();
    const std::string named = field("named");
    if (!named.empty())
    {
      auto p = ctx->runtime->namedPose(named);
      if (p) return *p;
      throw std::runtime_error("waypoint '" + wp + "': LIN/CIRC to named state '" + named +
                               "' but FK failed (unknown in this config's SRDF)");
    }
    throw std::runtime_error("waypoint '" + wp + "': LIN/CIRC needs a TCP pose or a named "
                             "joint state — capture a TCP pose, or use motion ptp/free");
  };

  const std::string tip = ctx->runtime->tipLink();
  MotionResult r;
  try
  {
    if (motion == "lin" || motion == "linear")
    {
      CartesianTarget t; t.pose = cartesianGoal(); t.tip_link = tip;
      r = ctx->runtime->moveLinear(t, opts);
    }
    else if (motion == "circ" || motion == "circular")
    {
      CircularTarget t; t.goal = cartesianGoal(); t.tip_link = tip;
      auto a = parseXyz3(field("aux"));
      t.aux.x = a[0]; t.aux.y = a[1]; t.aux.z = a[2];
      t.aux_is_center = (field("aux_is_center", "false") == "true");
      r = ctx->runtime->moveCircular(t, opts);
    }
    else  // "ptp" or "free" -> moveCollisionFree (handles pose OR joint)
    {
      MotionTarget t; t.tip_link = tip;
      if (type == "joint")
      {
        JointTarget jt;
        const std::string named = field("named");
        if (!named.empty()) jt.named_state = named;
        else jt.positions = parseVec(field("joints"));
        t.joint = jt;
      }
      else { t.pose = buildPose(); }

      PlanningOptions po; po.motion = opts;
      std::optional<PlannerMode> mode;
      if (!planner.empty()) mode = plannerModeFromString(planner);   // explicit per-waypoint planner
      else if (motion == "ptp") mode = PlannerMode::PILZ;            // PTP defaults to Pilz PTP
      po.mode = mode.value_or(ctx->runtime->plannerMode());          // else the launch planner_mode
      r = ctx->runtime->moveCollisionFree(t, po);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(nlog(config()), "MoveWaypoint '%s': %s", wp.c_str(), e.what());
    MotionResult thrown; thrown.cause = kCauseException;   // vNext M2.7: never the target's
    writeFault(motionFaultText(thrown));
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(nlog(config()), "MoveWaypoint '%s' (type=%s motion=%s planner=%s speed=%s%%)",
              wp.c_str(), type.c_str(), motion.c_str(),
              planner.empty() ? "mode" : planner.c_str(), speed.empty() ? "profile" : speed.c_str());
  if (!r.success && classifyMotionFailure(r) == MotionFailureClass::FAULT)
  {
    const std::string why = motionFaultText(r);
    if (writeFault(why))   // only a tree that remaps `fault` gets it -- and only then is it logged
      RCLCPP_ERROR(nlog(config()), "MoveWaypoint '%s': %s -- not this target's failure (vNext M2.7)",
                   wp.c_str(), why.c_str());
  }
  return r.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MovePtp::providedPorts()
{ return {BT::InputPort<std::string>("joints", ""), BT::InputPort<std::string>("named", ""),
          BT::InputPort<std::string>("profile", "transit"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MovePtp::tick()
{
  auto* ctx = ctxOf(config());
  JointTarget jt;
  const std::string named = in(*this, "named");
  if (!named.empty()) jt.named_state = named;
  else jt.positions = parseVec(in(*this, "joints"));
  return ctx->runtime->movePtp(jt, resolveOpts(*this, config(), "transit")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveToNamed::providedPorts()
{ return {BT::InputPort<std::string>("target"), BT::InputPort<std::string>("profile", "transit"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MoveToNamed::tick()
{
  auto* ctx = ctxOf(config());
  JointTarget jt; jt.named_state = in(*this, "target");
  if (jt.named_state.empty()) return BT::NodeStatus::FAILURE;
  return ctx->runtime->movePtp(jt, resolveOpts(*this, config(), "transit")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveToJoint::providedPorts()
{ return {BT::InputPort<std::string>("joints"), BT::InputPort<std::string>("profile", "transit"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MoveToJoint::tick()
{
  auto* ctx = ctxOf(config());
  JointTarget jt; jt.positions = parseVec(in(*this, "joints"));   // radians
  if (jt.positions.empty()) return BT::NodeStatus::FAILURE;
  return ctx->runtime->movePtp(jt, resolveOpts(*this, config(), "transit")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveLinear::providedPorts()
{ return {BT::InputPort<std::string>("pose_key"), BT::InputPort<std::string>("tip_link", ""),
          BT::InputPort<std::string>("profile", "approach"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MoveLinear::tick()
{
  auto* ctx = ctxOf(config());
  CartesianTarget t;
  try { t.pose = config().blackboard->get<geometry_msgs::msg::Pose>(in(*this, "pose_key")); }
  catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "MoveLinear: bad pose_key: %s", e.what()); return BT::NodeStatus::FAILURE; }
  t.tip_link = in(*this, "tip_link");
  return ctx->runtime->moveLinear(t, resolveOpts(*this, config(), "approach")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveCircular::providedPorts()
{ return {BT::InputPort<std::string>("goal_key"), BT::InputPort<std::string>("aux"),
          BT::InputPort<std::string>("aux_is_center", "false"), BT::InputPort<std::string>("tip_link", ""),
          BT::InputPort<std::string>("profile", "approach"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MoveCircular::tick()
{
  auto* ctx = ctxOf(config());
  CircularTarget t;
  try { t.goal = config().blackboard->get<geometry_msgs::msg::Pose>(in(*this, "goal_key")); }
  catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "MoveCircular: bad goal_key: %s", e.what()); return BT::NodeStatus::FAILURE; }
  try { auto a = parseXyz3(in(*this, "aux")); t.aux.x = a[0]; t.aux.y = a[1]; t.aux.z = a[2]; }
  catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "MoveCircular: bad aux: %s", e.what()); return BT::NodeStatus::FAILURE; }
  t.aux_is_center = (in(*this, "aux_is_center", "false") == "true");
  t.tip_link = in(*this, "tip_link");
  return ctx->runtime->moveCircular(t, resolveOpts(*this, config(), "approach")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveCollisionFree::providedPorts()
{ return {BT::InputPort<std::string>("pose_key", ""), BT::InputPort<std::string>("joints", ""),
          BT::InputPort<std::string>("named", ""), BT::InputPort<std::string>("planner", "pilz"),
          BT::InputPort<std::string>("tip_link", ""), BT::InputPort<std::string>("profile", "transit"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus MoveCollisionFree::tick()
{
  auto* ctx = ctxOf(config());
  MotionTarget t;
  t.tip_link = in(*this, "tip_link");
  const std::string pose_key = in(*this, "pose_key"), named = in(*this, "named"), joints = in(*this, "joints");
  if (!pose_key.empty())
  {
    try { t.pose = config().blackboard->get<geometry_msgs::msg::Pose>(pose_key); }
    catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "MoveCollisionFree: bad pose_key: %s", e.what()); return BT::NodeStatus::FAILURE; }
  }
  else if (!named.empty()) { JointTarget jt; jt.named_state = named; t.joint = jt; }
  else if (!joints.empty()) { JointTarget jt; jt.positions = parseVec(joints); t.joint = jt; }
  else { RCLCPP_ERROR(nlog(config()), "MoveCollisionFree: no target"); return BT::NodeStatus::FAILURE; }

  PlanningOptions po;
  po.motion = resolveOpts(*this, config(), "transit");
  // empty/unset `planner` -> use the runtime's global planner_mode (launch param).
  auto mode = plannerModeFromString(in(*this, "planner", ""));
  po.mode = mode.value_or(ctx->runtime->plannerMode());
  return ctx->runtime->moveCollisionFree(t, po).success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList ExecuteProcessPath::providedPorts()
{ return {BT::InputPort<std::string>("path_file"), BT::InputPort<std::string>("profile", "process"),
          BT::InputPort<std::string>("speed", ""), BT::InputPort<std::string>("accel", "")}; }
BT::NodeStatus ExecuteProcessPath::tick()
{
  auto* ctx = ctxOf(config());
  const std::string file = in(*this, "path_file");
  if (file.empty()) return BT::NodeStatus::FAILURE;
  CartesianProcessPath path;
  try
  {
    if (file.size() > 4 && file.substr(file.size() - 4) == ".csv")
    {
      PathFrame f; f.reference_frame = ctx->runtime->baseFrame(); f.tcp_link = ctx->runtime->tipLink();
      path = CsvProcessPathLoader::fromFile(file, f);
    }
    else
    {
      path = YamlProcessPathLoader::fromFile(file);
    }
  }
  catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "ExecuteProcessPath load failed: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return ctx->runtime->executeCartesianProcess(path, resolveOpts(*this, config(), "process")).success
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══ end-effector ═════════════════════════════════════════════════════════════
// Grasp/release honour the two ADR-0010 axes: the ACTUATION (a GripperCommand to the
// open/closed target — 1.0/0.0 by default == the historical suction on/off) AND, when a
// gripper_cmd_topic is configured, the SIM GRASP ADAPTER signal (a latched Bool that fires
// the planning-scene attach + the active backend's adapter). Either may be absent: a cell
// with only a grasp adapter (no actuation controller) still welds; the "no gripper" error
// is kept only when NEITHER is configured.
namespace
{
void fireGraspSignal(const trainit::BtContext* ctx, bool close)
{
  if (!ctx->gripper_cmd_pub) return;
  std_msgs::msg::Bool m;
  m.data = close;
  ctx->gripper_cmd_pub->publish(m);
}

// --- the canonical end-effector contract (ADR-0022) -------------------------------------------
// The ports carry logical ids only; the registry (end_effector_registry.hpp) resolves, commands
// or judges, logs, and latches a FAULT. The outputs are written ONLY on FAILURE and never
// cleared: `reason` and `subject` always, `fault` only for a reason whose class is not VERDICT.
// On the LEGACY branch (no eef block) nothing here runs and no output is ever written.
BT::PortsList eefOutputs(BT::PortsList ports)
{
  ports.insert(BT::OutputPort<std::string>("reason", "on FAILURE: the ADR-0022 reason"));
  ports.insert(BT::OutputPort<std::string>("fault", "on FAILURE: the reason, unless it is a VERDICT"));
  ports.insert(BT::OutputPort<std::string>("subject", "on FAILURE: the identity the reason is about"));
  return ports;
}

BT::NodeStatus eefOutcome(BT::TreeNode& node, const EefResult& r)
{
  if (r.ok) return BT::NodeStatus::SUCCESS;
  node.setOutput("reason", std::string(eef::toToken(r.reason)));
  node.setOutput("subject", r.subject);
  if (eef::isFault(r.reason)) node.setOutput("fault", std::string(eef::toToken(r.reason)));
  return BT::NodeStatus::FAILURE;
}

// An optional integer port: absent -> nullopt (the contract's value); present but unreadable
// -> false.
bool optionalInt(const BT::TreeNode& node, const char* port, std::optional<int64_t>& out)
{
  out.reset();
  if (node.config().input_ports.count(port) == 0) return true;
  const auto e = node.getInput<int>(port);
  if (!e) return false;
  out = e.value();
  return true;
}

// An optional string port (eef, site, actuator, target, expect, accept): absent or an empty
// literal -> "" (the only device, the only site or tool level, no narrowing); present but
// unreadable -- a blackboard key never written, or not a string -> false, with the cause in
// `error`: a port error, as for an int port, never a silent "no site".
bool optionalStr(const BT::TreeNode& node, const char* port, std::string& out, std::string& error)
{
  out.clear();
  const auto& ports = node.config().input_ports;
  const auto it = ports.find(port);
  if (it == ports.end() || it->second.empty()) return true;
  const auto e = node.getInput<std::string>(port);
  if (!e)
  {
    error = std::string(port) + "=\"" + it->second + "\" cannot be read: " + e.error();
    return false;
  }
  out = e.value();
  return true;
}

BT::NodeStatus eefGrip(BT::TreeNode& node, const BT::NodeConfig& cfg, bool engage)
{
  const char* who = engage ? "CloseGripper" : "OpenGripper";
  std::string eef_id, site, error;
  if (!optionalStr(node, "eef", eef_id, error) || !optionalStr(node, "site", site, error))
    return eefOutcome(node, ctxOf(cfg)->eefs->portError(who, node.fullPath(), error));
  return eefOutcome(node, ctxOf(cfg)->eefs->grip(who, eef_id, site, engage));
}
}  // namespace

// LEGACY (ADR-0022, ADR-0023): after their canonical first line, Open/CloseGripper run the two
// ADR-0010 axes this section's head describes, with the gripper action and the grasp Bool as the
// project writes them; SetGripper is that path only (the runner refuses it in a canonical tree at
// start-up). The three bodies are pinned textually (ADR-0022 D9, test_m2_6_eef_static_scan).
BT::PortsList OpenGripper::providedPorts()
{ return eefOutputs({BT::InputPort<std::string>("eef", "ADR-0022: the device (default: the only one)"),
                     BT::InputPort<std::string>("site", "ADR-0022: the site (required on a multi-site device)")}); }
BT::NodeStatus OpenGripper::tick()
{
  if (ctxOf(config())->eefs) return eefGrip(*this, config(), false);   // the canonical contract (ADR-0022)
  auto* ctx = ctxOf(config());
  if (!ctx->gripper && !ctx->gripper_cmd_pub)
  { RCLCPP_ERROR(nlog(config()), "OpenGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  const bool ok = ctx->gripper ? ctx->gripper->command(ctx->gripper_open_pos) : true;
  if (ok) fireGraspSignal(ctx, false);
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CloseGripper::providedPorts()
{ return eefOutputs({BT::InputPort<std::string>("eef", "ADR-0022: the device (default: the only one)"),
                     BT::InputPort<std::string>("site", "ADR-0022: the site (required on a multi-site device)")}); }
BT::NodeStatus CloseGripper::tick()
{
  if (ctxOf(config())->eefs) return eefGrip(*this, config(), true);   // the canonical contract (ADR-0022)
  auto* ctx = ctxOf(config());
  if (!ctx->gripper && !ctx->gripper_cmd_pub)
  { RCLCPP_ERROR(nlog(config()), "CloseGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  const bool ok = ctx->gripper ? ctx->gripper->command(ctx->gripper_close_pos) : true;
  if (ok) fireGraspSignal(ctx, true);
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList SetGripper::providedPorts() { return {BT::InputPort<std::string>("position", "1.0")}; }
BT::NodeStatus SetGripper::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->gripper) { RCLCPP_ERROR(nlog(config()), "SetGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  return ctx->gripper->command(std::stod(in(*this, "position", "1.0"))) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// CommandActuator (ADR-0022): commands ONE actuator of a device to one of its NAMED targets on
// the actuator's own standard interface (SetBool engage/release, a GripperCommand position, a
// FollowJointTrajectory point with its tolerances). A grip actuator's engage or release target
// is refused (UNKNOWN_TARGET): use CloseGripper/OpenGripper. SUCCESS is actuation only.
// `timeout_ms` replaces the device's command_timeout_ms for this command.
BT::PortsList CommandActuator::providedPorts()
{ return eefOutputs({BT::InputPort<std::string>("eef", "the device (default: the only one)"),
                     BT::InputPort<std::string>("actuator", "the actuator id"),
                     BT::InputPort<std::string>("target", "a NAMED target of the actuator"),
                     BT::InputPort<int>("timeout_ms", "replaces the device's command_timeout_ms")}); }
BT::NodeStatus CommandActuator::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->eefs)
  { RCLCPP_ERROR(nlog(config()), "CommandActuator: the project has no end-effector contract (ADR-0022)"); return BT::NodeStatus::FAILURE; }
  std::string eef_id, actuator, target, error;
  if (!optionalStr(*this, "eef", eef_id, error) || !optionalStr(*this, "actuator", actuator, error) ||
      !optionalStr(*this, "target", target, error))
    return eefOutcome(*this, ctx->eefs->portError("CommandActuator", fullPath(), error));
  std::optional<int64_t> timeout_ms;
  if (!optionalInt(*this, "timeout_ms", timeout_ms))
    return eefOutcome(*this, ctx->eefs->portError("CommandActuator", fullPath(), "timeout_ms is not an integer"));
  return eefOutcome(*this, ctx->eefs->actuate("CommandActuator", eef_id, actuator, target, timeout_ms));
}

// VerifyGrasp (ADR-0022 D4): judges the TOOL, not an actuator -- held/released at `site`, or at
// tool level on a multi-site device with no `site`. There is no actuator port: a command's
// outcome never enters the verdict. `accept` may only NARROW the project's accepted_evidence;
// unset timeout_ms / max_age_ms take the project's verification block (3000 / 500 ms).
BT::PortsList VerifyGrasp::providedPorts()
{ return eefOutputs({BT::InputPort<std::string>("eef", "the device (default: the only one)"),
                     BT::InputPort<std::string>("site", "the site (empty: the only one, or tool level)"),
                     BT::InputPort<std::string>("expect", "held | released"),
                     BT::InputPort<int>("timeout_ms", "default: the project's verification.timeout_ms"),
                     BT::InputPort<int>("max_age_ms", "default: the project's verification.max_age_ms"),
                     BT::InputPort<std::string>("accept", "narrows the accepted evidence, e.g. sensor")}); }
BT::NodeStatus VerifyGrasp::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->eefs)
  { RCLCPP_ERROR(nlog(config()), "VerifyGrasp: the project has no end-effector contract (ADR-0022)"); return BT::NodeStatus::FAILURE; }
  std::string eef_id, site, expect, accept, error;
  if (!optionalStr(*this, "eef", eef_id, error) || !optionalStr(*this, "site", site, error) ||
      !optionalStr(*this, "expect", expect, error) || !optionalStr(*this, "accept", accept, error))
    return eefOutcome(*this, ctx->eefs->portError("VerifyGrasp", fullPath(), error));
  std::optional<int64_t> timeout_ms, max_age_ms;
  if (!optionalInt(*this, "timeout_ms", timeout_ms) || !optionalInt(*this, "max_age_ms", max_age_ms))
    return eefOutcome(*this, ctx->eefs->portError("VerifyGrasp", fullPath(), "timeout_ms / max_age_ms is not an integer"));
  return eefOutcome(*this, ctx->eefs->verifyGrasp("VerifyGrasp", eef_id, site, expect, timeout_ms, max_age_ms, accept));
}

BT::PortsList EnableProcess::providedPorts() { return {BT::InputPort<std::string>("channel")}; }
BT::NodeStatus EnableProcess::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->process) { RCLCPP_ERROR(nlog(config()), "EnableProcess: no process controller"); return BT::NodeStatus::FAILURE; }
  ProcessEvent e; e.type = ProcessEventType::ENABLE_PROCESS; e.channel = in(*this, "channel"); e.value = 1.0;
  return ctx->process->executeEvent(e) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList DisableProcess::providedPorts() { return {BT::InputPort<std::string>("channel")}; }
BT::NodeStatus DisableProcess::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->process) { RCLCPP_ERROR(nlog(config()), "DisableProcess: no process controller"); return BT::NodeStatus::FAILURE; }
  ProcessEvent e; e.type = ProcessEventType::DISABLE_PROCESS; e.channel = in(*this, "channel");
  return ctx->process->executeEvent(e) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══ planning scene ═══════════════════════════════════════════════════════════
BT::PortsList AddCollisionObject::providedPorts()
{ return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("frame", "base_link"),
          BT::InputPort<std::string>("dims"), BT::InputPort<std::string>("position", "0;0;0")}; }
BT::NodeStatus AddCollisionObject::tick()
{
  auto* ctx = ctxOf(config());
  try {
    ctx->scene->addCollisionBox(in(*this, "id"), in(*this, "frame", "base_link"),
      parseXyz3(in(*this, "dims")), poseFromXyz(parseXyz3(in(*this, "position", "0;0;0"))));
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "AddCollisionObject: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList RemoveCollisionObject::providedPorts() { return {BT::InputPort<std::string>("id")}; }
BT::NodeStatus RemoveCollisionObject::tick()
{
  ctxOf(config())->scene->removeCollisionObject(in(*this, "id"));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList AttachObject::providedPorts()
{ return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("link", "tcp"),
          BT::InputPort<std::string>("dims"), BT::InputPort<std::string>("position", "0;0;0")}; }
BT::NodeStatus AttachObject::tick()
{
  auto* ctx = ctxOf(config());
  try {
    ctx->scene->attachBox(in(*this, "id"), in(*this, "link", "tcp"),
      parseXyz3(in(*this, "dims")), poseFromXyz(parseXyz3(in(*this, "position", "0;0;0"))));
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "AttachObject: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList DetachObject::providedPorts()
{ return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("link", "tcp")}; }
BT::NodeStatus DetachObject::tick()
{
  ctxOf(config())->scene->detachObject(in(*this, "id"), in(*this, "link", "tcp"));
  return BT::NodeStatus::SUCCESS;
}

// ═══ dynamic-object runtime flags ═════════════════════════════════════════════
// A scene-manager service that did not answer. With NO scene_manager_node anywhere in the
// graph that is the documented no-op -- a cell with no scene loader (a from-scratch mock):
// a warning, SUCCESS. With one running it is a misconfiguration or a bring-up that is not
// up, and the change the tree asked for was NOT applied, so reporting success hid it: in a
// namespaced cell both nodes asked the root and silently did nothing (vNext M0C #5).
// Returns the fully qualified names of the scene managers that do run ("" = none).
static std::string runningSceneManagers(const rclcpp::Node::SharedPtr& node)
{
  static const std::string kSuffix = "/scene_manager_node";
  std::string found;
  for (const auto& n : node->get_node_names())     // fully qualified, e.g. /r3/scene_manager_node
    if (n.size() >= kSuffix.size() &&
        n.compare(n.size() - kSuffix.size(), kSuffix.size(), kSuffix) == 0)
      found += (found.empty() ? "" : ", ") + n;
  return found;
}

// Both nodes below create their client PER TICK, on the tree's own node. A static client
// kept the FIRST name it was built with for the life of the process, whatever a later port
// said, and outlived the node it was made on. They tick once per move or per cycle, and
// wait_for_service() already waits for this client to be matched with the server.
// SetAttachedCollisionCheck: flips the scene_manager_node planning-collision check
// for GRASPED objects (its SetBool service). ON => the planner routes the held
// payload around the static/actuated meshes (e.g. into the prewash); OFF => the
// payload is transparent to them (e.g. at the pick, where it sits on the belt).
// BLOCKS on the service response so the ACM is applied BEFORE the next move plans.
// The default service name is RELATIVE (trainit_scene::kAttachedCollisionCheckServiceDefault),
// so a tree in /r3 reaches /r3's scene manager. No scene manager at all is a no-op; one that
// runs but does not answer at this name is a FAILURE (runningSceneManagers above).
BT::PortsList SetAttachedCollisionCheck::providedPorts()
{ return {BT::InputPort<std::string>("value"),   // "true"|"false"
          BT::InputPort<std::string>("service",
                                     trainit_scene::kAttachedCollisionCheckServiceDefault)}; }
BT::NodeStatus SetAttachedCollisionCheck::tick()
{
  auto* ctx = ctxOf(config());
  const std::string v = in(*this, "value");
  const bool value = (v == "true" || v == "1");
  const std::string srv = in(*this, "service",
                             trainit_scene::kAttachedCollisionCheckServiceDefault);
  auto client = ctx->node->create_client<std_srvs::srv::SetBool>(srv);
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    const std::string running = runningSceneManagers(ctx->node);
    if (!running.empty())
    {
      RCLCPP_ERROR(nlog(config()), "SetAttachedCollisionCheck: '%s' does not answer, but a "
                   "scene manager runs (%s) -- the check was NOT changed. Set the `service` "
                   "port, or run the tree in that scene manager's namespace.",
                   client->get_service_name(), running.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN(nlog(config()), "SetAttachedCollisionCheck: no scene manager in the graph "
                "('%s' unavailable) -- skipping (no scene loader)", client->get_service_name());
    return BT::NodeStatus::SUCCESS;   // the documented no-op: nothing to configure
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = value;
  auto fut = client->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_ERROR(nlog(config()), "SetAttachedCollisionCheck: service timed out");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(nlog(config()), "SetAttachedCollisionCheck: %s", value ? "ON" : "OFF");
  return BT::NodeStatus::SUCCESS;
}

// SetReleasePolicy: tells the Isaac manipulation adapter what a grasped object does
// on the NEXT release — freeze (stay put; a PLC clamp is imagined to hold it) or
// gravity (fall). Latched Bool on /isaac_release_policy (freeze=true). Fire-and-
// forget: Isaac applies it when the grip opens, which the tree sequences after.
// A no-op in mock/real (no Isaac subscriber) — safe to leave in every tree.
// LEGACY (ADR-0022, ADR-0023): the default is an Isaac-named GLOBAL topic every robot shares,
// and the publisher is function-static: it binds the FIRST topic for the life of the process
// (M2.6 E4). Never emitted for a canonical project; kept unchanged for legacy trees.
BT::PortsList SetReleasePolicy::providedPorts()
{ return {BT::InputPort<std::string>("policy", "freeze"),   // "freeze"|"gravity"
          BT::InputPort<std::string>("topic", "/isaac_release_policy")}; }
BT::NodeStatus SetReleasePolicy::tick()
{
  auto* ctx = ctxOf(config());
  const std::string policy = in(*this, "policy", "freeze");
  const bool freeze = (policy != "gravity");   // freeze default; only "gravity" => fall
  const std::string topic = in(*this, "topic", "/isaac_release_policy");
  static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
  if (!pub) pub = ctx->node->create_publisher<std_msgs::msg::Bool>(
                    topic, rclcpp::QoS(1).transient_local());
  std_msgs::msg::Bool msg; msg.data = freeze;
  pub->publish(msg);
  RCLCPP_INFO(nlog(config()), "SetReleasePolicy: %s", freeze ? "freeze" : "gravity");
  return BT::NodeStatus::SUCCESS;
}

// ResetScene: put every dynamic object back at its scene.yaml pose (scene_manager
// ~/reset_scene). The cycle boundary of a looping app in SIMULATION — cycle N+1
// picks where cycle 1 did. The scene_manager also latches its scene_reset_topic (LEGACY
// default /isaac_scene_reset) so the Isaac adapter can teleport its prims. Same name rule
// as SetAttachedCollisionCheck: RELATIVE default, a no-op with no scene manager, a FAILURE
// with one that does not answer.
BT::PortsList ResetScene::providedPorts()
{ return {BT::InputPort<std::string>("service", trainit_scene::kResetSceneServiceDefault)}; }
BT::NodeStatus ResetScene::tick()
{
  auto* ctx = ctxOf(config());
  const std::string srv = in(*this, "service", trainit_scene::kResetSceneServiceDefault);
  auto client = ctx->node->create_client<std_srvs::srv::Trigger>(srv);
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    const std::string running = runningSceneManagers(ctx->node);
    if (!running.empty())
    {
      RCLCPP_ERROR(nlog(config()), "ResetScene: '%s' does not answer, but a scene manager "
                   "runs (%s) -- the scene was NOT reset. Set the `service` port, or run the "
                   "tree in that scene manager's namespace.",
                   client->get_service_name(), running.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN(nlog(config()), "ResetScene: no scene manager in the graph ('%s' "
                "unavailable) -- skipping (no scene loader)", client->get_service_name());
    return BT::NodeStatus::SUCCESS;   // the documented no-op: nothing to reset
  }
  auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
  {
    RCLCPP_ERROR(nlog(config()), "ResetScene: service timed out");
    return BT::NodeStatus::FAILURE;
  }
  auto resp = fut.get();
  RCLCPP_INFO(nlog(config()), "ResetScene: %s", resp->message.c_str());
  return resp->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══ pose math ════════════════════════════════════════════════════════════════
// Build a FULL TCP pose for one waypoint: position + orientation. Requires the
// orientation (quaternion or RPY) — a TCP waypoint must fill ALL its DOF; there
// is no shared/implicit orientation. This is the per-point primitive the
// application layer (and the future setup assistant) emits for a "TCP pose" point.
BT::PortsList MakePose::providedPorts()
{ return {BT::InputPort<std::string>("position"),            // "x;y;z"  (metres, in base frame)
          BT::InputPort<std::string>("orientation", ""),     // "qx;qy;qz;qw" (preferred)
          BT::InputPort<std::string>("rpy_deg", ""),         // "r;p;y"  (degrees, alternative)
          BT::InputPort<std::string>("out_key")}; }
BT::NodeStatus MakePose::tick()
{
  try {
    geometry_msgs::msg::Pose pose;
    auto p = parseXyz3(in(*this, "position"));
    pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2];

    const std::string quat = in(*this, "orientation");
    const std::string rpy  = in(*this, "rpy_deg");
    if (!quat.empty())
    {
      auto q = parseVec(quat);
      if (q.size() != 4) throw std::runtime_error("orientation needs 4 values qx;qy;qz;qw");
      pose.orientation.x = q[0]; pose.orientation.y = q[1]; pose.orientation.z = q[2]; pose.orientation.w = q[3];
    }
    else if (!rpy.empty())
    {
      auto qd = quatFromRpyDeg(parseXyz3(rpy));
      pose.orientation.x = qd.x(); pose.orientation.y = qd.y(); pose.orientation.z = qd.z(); pose.orientation.w = qd.w();
    }
    else
    {
      RCLCPP_ERROR(nlog(config()), "MakePose '%s': a TCP pose needs an orientation (orientation= or rpy_deg=) "
                   "— fill ALL degrees of freedom", in(*this, "out_key").c_str());
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set<geometry_msgs::msg::Pose>(in(*this, "out_key"), pose);
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "MakePose: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList StoreCurrentPose::providedPorts() { return {BT::InputPort<std::string>("pose_key")}; }
BT::NodeStatus StoreCurrentPose::tick()
{
  auto* ctx = ctxOf(config());
  config().blackboard->set<geometry_msgs::msg::Pose>(in(*this, "pose_key"), currentPose(ctx));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList ComputeTcpTarget::providedPorts()
{ return {BT::InputPort<std::string>("target_xyz"), BT::InputPort<std::string>("tcp_offset_xyz", "0;0;0"),
          BT::InputPort<std::string>("tcp_offset_rpy_deg", "0;0;0"),
          BT::InputPort<std::string>("use_current_orientation", "true"),
          BT::InputPort<std::string>("fixed_rpy_deg", "0;0;0"),
          BT::InputPort<std::string>("quat", ""),  // optional "x;y;z;w" for EXACT orientation
          BT::InputPort<std::string>("out_key")}; }
BT::NodeStatus ComputeTcpTarget::tick()
{
  auto* ctx = ctxOf(config());
  try {
    geometry_msgs::msg::Pose base;
    auto txyz = parseXyz3(in(*this, "target_xyz"));
    base.position.x = txyz[0]; base.position.y = txyz[1]; base.position.z = txyz[2];

    const std::string quat = in(*this, "quat");
    if (!quat.empty())
    {
      auto q = parseVec(quat);
      if (q.size() != 4) throw std::runtime_error("quat needs 4 values x;y;z;w");
      base.orientation.x = q[0]; base.orientation.y = q[1]; base.orientation.z = q[2]; base.orientation.w = q[3];
    }
    else if (in(*this, "use_current_orientation", "true") == "true")
    {
      base.orientation = currentPose(ctx).orientation;
    }
    else
    {
      auto qd = quatFromRpyDeg(parseXyz3(in(*this, "fixed_rpy_deg", "0;0;0")));
      base.orientation.x = qd.x(); base.orientation.y = qd.y(); base.orientation.z = qd.z(); base.orientation.w = qd.w();
    }

    auto out = applyTcpOffset(base, parseXyz3(in(*this, "tcp_offset_xyz", "0;0;0")),
                              parseXyz3(in(*this, "tcp_offset_rpy_deg", "0;0;0")));
    config().blackboard->set<geometry_msgs::msg::Pose>(in(*this, "out_key"), out);
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "ComputeTcpTarget: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList OffsetPoseInToolFrame::providedPorts()
{ return {BT::InputPort<std::string>("in_key"), BT::InputPort<std::string>("dx", "0.0"),
          BT::InputPort<std::string>("dy", "0.0"), BT::InputPort<std::string>("dz", "0.0"),
          BT::InputPort<std::string>("out_key")}; }
BT::NodeStatus OffsetPoseInToolFrame::tick()
{
  try {
    auto src = config().blackboard->get<geometry_msgs::msg::Pose>(in(*this, "in_key"));
    auto out = offsetInToolFrame(src, std::stod(in(*this, "dx", "0.0")), std::stod(in(*this, "dy", "0.0")),
                                 std::stod(in(*this, "dz", "0.0")));
    config().blackboard->set<geometry_msgs::msg::Pose>(in(*this, "out_key"), out);
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "OffsetPoseInToolFrame: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList OffsetPoseInBaseFrame::providedPorts()
{ return {BT::InputPort<std::string>("in_key"), BT::InputPort<std::string>("dx", "0.0"),
          BT::InputPort<std::string>("dy", "0.0"), BT::InputPort<std::string>("dz", "0.0"),
          BT::InputPort<std::string>("xyz", ""), BT::InputPort<std::string>("out_key")}; }
BT::NodeStatus OffsetPoseInBaseFrame::tick()
{
  try {
    double dx = 0, dy = 0, dz = 0;
    const std::string xyz = in(*this, "xyz");
    if (!xyz.empty()) { auto v = parseXyz3(xyz); dx = v[0]; dy = v[1]; dz = v[2]; }
    else { dx = std::stod(in(*this, "dx", "0.0")); dy = std::stod(in(*this, "dy", "0.0")); dz = std::stod(in(*this, "dz", "0.0")); }
    auto src = config().blackboard->get<geometry_msgs::msg::Pose>(in(*this, "in_key"));
    src.position.x += dx; src.position.y += dy; src.position.z += dz;
    config().blackboard->set<geometry_msgs::msg::Pose>(in(*this, "out_key"), src);
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "OffsetPoseInBaseFrame: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

// ═══ utility ══════════════════════════════════════════════════════════════════
BT::PortsList Wait::providedPorts() { return {BT::InputPort<int>("duration_ms", 100, "sleep [ms]")}; }
BT::NodeStatus Wait::tick()
{
  int ms = 100; auto e = getInput<int>("duration_ms"); if (e) ms = e.value();
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList Log::providedPorts() { return {BT::InputPort<std::string>("message")}; }
BT::NodeStatus Log::tick()
{
  RCLCPP_INFO(nlog(config()), "[BT] %s", in(*this, "message").c_str());
  return BT::NodeStatus::SUCCESS;
}

// ═══ perception ═══════════════════════════════════════════════════════════════
namespace
{
// safeLog() (a logger that works WITHOUT a BtContext, so pure-blackboard nodes are
// unit-testable with a bare tree) lives in the helper block at the top of this file.

std::string fmtVec(const std::vector<double>& v)
{
  std::ostringstream os;
  os.precision(6);
  for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
  return os.str();
}

// One subscription per topic, shared by every DetectObject that names it. Held as a
// static so the subscription outlives a tick (the node object is recreated per tree).
struct DetectionFeed
{
  std::mutex m;
  vision_msgs::msg::Detection3DArray::SharedPtr latest;
  uint64_t arrivals{0};
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub;
};

DetectionFeed& feedFor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
{
  static std::map<std::string, std::shared_ptr<DetectionFeed>> feeds;
  static std::mutex feeds_m;
  std::lock_guard<std::mutex> lk(feeds_m);
  auto& f = feeds[topic];
  if (!f)
  {
    f = std::make_shared<DetectionFeed>();
    // The perception contract publishes RELIABLE depth 1: a detection is an event the
    // consumer waited for and must not miss.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    DetectionFeed* raw = f.get();
    f->sub = node->create_subscription<vision_msgs::msg::Detection3DArray>(
      topic, qos, [raw](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> g(raw->m);
        raw->latest = std::move(msg);
        ++raw->arrivals;
      });
  }
  return *f;
}

std::shared_ptr<tf2_ros::Buffer> tfFor(BtContext* ctx)
{
  if (ctx->tf) return ctx->tf;
  // Older launcher without a buffer in the context: build one once, spun by the
  // node's own executor (spin_thread=false).
  static std::shared_ptr<tf2_ros::Buffer> buf;
  static std::shared_ptr<tf2_ros::TransformListener> lis;
  if (!buf)
  {
    buf = std::make_shared<tf2_ros::Buffer>(ctx->node->get_clock());
    lis = std::make_shared<tf2_ros::TransformListener>(*buf, ctx->node, false);
  }
  return buf;
}
}  // namespace

// DetectObject: wait for a detection FRESHER than this tick, transform it into
// target_frame, and write it to the blackboard as strings the rest of the tree reads.
//
//   detector      name -> topic /perception/<detector>/detections (or give `topic`)
//   class_id      keep only detections whose hypothesis.class_id matches ("" = any)
//   target_frame  planning frame, default "base_link" (TF from the message's frame)
//   timeout_ms    how long to wait for a fresh message
//   keep          how many CANDIDATES to publish, default 1 (ADR-0016 Decision 2)
//   out_key       prefix: writes <out_key>.position "x;y;z", .orientation "qx;qy;qz;qw",
//                 .score, .class_id, .frame, .size "sx;sy;sz"
//
// `keep` is the whole of ADR-0016 Decision 2 in this node:
//
//   keep = 1   the FIRST published match, on the FLAT keys above. Every bundle
//              generated before ADR-0016 emits <DetectObject .../> with no `keep`, so
//              this is the path they all take and its key LAYOUT does not move. (It
//              chose the highest SCORE until vNext M0C #4: a re-rank that overrode the
//              detector's ranking.)
//   keep = N   the first N matches on <out_key>.<i>.* (i = 0..N-1) plus
//              <out_key>.count, IN THE ORDER THE ARRAY WAS PUBLISHED — and NOT
//              sorted. ADR-0016 Decision 1 carries the rank in the publication
//              order, because the terms a bin-picking cell ranks by (score x pose_quality x
//              visible_fraction) are the detector's vocabulary and the perception
//              contract exists so the runtime never learns it. TMR learns "try them
//              in the order you were given", and no more; "try the next candidate"
//              is then a decorator over SetWaypointFromDetection's `index`, not a
//              new control node.
//
// Freshness is by ARRIVAL, not by stamp: a message that arrived after this tick began
// is fresh, whatever clock the detector used. That survives sim/system clock mixes.
BT::PortsList DetectObject::providedPorts()
{ return {BT::InputPort<std::string>("detector"),
          BT::InputPort<std::string>("topic", ""),
          BT::InputPort<std::string>("class_id", ""),
          BT::InputPort<std::string>("target_frame", "base_link"),
          BT::InputPort<std::string>("timeout_ms", "3000"),
          BT::InputPort<std::string>("min_score", "0"),
          // INT, not string, like every other port here: a count may arrive from the
          // blackboard ({tries}), and a string port REFUSES AT TREE CREATION to bind an
          // entry already typed int — "the port was initially created with type [int]".
          // An int port binds an int entry, a string entry and a literal alike (verified).
          BT::InputPort<int>("keep", 1, "how many candidates to publish (ADR-0016)"),
          BT::InputPort<std::string>("out_key", "detected"),
          // ADR-0022 (canonical bin picking): written ONLY when no fresh message arrived within
          // timeout_ms or a TF lookup failed -- never for a fresh message with no match, which
          // stays the "bin empty" stall. A tree that does not remap it is unaffected.
          BT::OutputPort<std::string>("fault", "DETECTION_TIMEOUT | TF_LOOKUP_FAILED")}; }
BT::NodeStatus DetectObject::tick()
{
  auto* ctx = ctxOf(config());
  const std::string det = in(*this, "detector");
  std::string topic = in(*this, "topic");
  if (topic.empty())
  {
    if (det.empty()) { RCLCPP_ERROR(nlog(config()), "DetectObject: need 'detector' or 'topic'"); return BT::NodeStatus::FAILURE; }
    // CELL-GLOBAL (ADR-0023): a detector serves a station, not one robot (absolute in any namespace).
    topic = "/perception/" + det + "/detections";
  }
  const std::string want_class = in(*this, "class_id");
  const std::string target = in(*this, "target_frame", "base_link");
  const std::string out = in(*this, "out_key", "detected");
  int timeout_ms = 3000; double min_score = 0.0;
  try { timeout_ms = std::stoi(in(*this, "timeout_ms", "3000")); } catch (const std::exception&) {}
  try { min_score = std::stod(in(*this, "min_score", "0")); } catch (const std::exception&) {}
  int keep = 1;
  if (intIn(*this, "keep", keep) && keep != clampDetectionKeep(keep))
  {
    RCLCPP_WARN(nlog(config()), "DetectObject: keep=%d is not a count; publishing the single "
                "best candidate (keep=1)", keep);
    keep = clampDetectionKeep(keep);
  }

  DetectionFeed& feed = feedFor(ctx->node, topic);
  uint64_t seen;
  { std::lock_guard<std::mutex> g(feed.m); seen = feed.arrivals; }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  vision_msgs::msg::Detection3DArray::SharedPtr msg;
  while (std::chrono::steady_clock::now() < deadline)
  {
    { std::lock_guard<std::mutex> g(feed.m); if (feed.arrivals > seen) { msg = feed.latest; break; } }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!msg)
  {
    RCLCPP_ERROR(nlog(config()), "DetectObject: no fresh detection on '%s' within %d ms "
                 "(is the detector running? is it publishing RELIABLE?)", topic.c_str(), timeout_ms);
    setOutput("fault", std::string("DETECTION_TIMEOUT"));
    return BT::NodeStatus::FAILURE;
  }

  // The candidates, by the ONE rule both nodes of the pair share: the best match at
  // keep=1, the first `keep` matches in PUBLISHED order above it. No sorting here —
  // see detection_candidates.hpp.
  const auto chosen = selectDetectionCandidates(*msg, want_class, min_score, keep);
  if (chosen.empty())
  {
    RCLCPP_WARN(nlog(config()), "DetectObject: fresh message on '%s' but no detection%s%s (had %zu)",
                topic.c_str(), want_class.empty() ? "" : " of class ", want_class.c_str(), msg->detections.size());
    return BT::NodeStatus::FAILURE;
  }

  // TF: the detector reports in its sensor's OPTICAL frame and knows nothing about
  // robots. ONE lookup per source frame, shared by every candidate from that frame:
  // two lookups at TimePointZero can return two different transforms (a wrist camera
  // moves between them), and the candidates of one message must land in one frame or
  // their order stops meaning anything.
  auto tf = tfFor(ctx);
  std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;
  auto transformTo = [&](const std::string& src, geometry_msgs::msg::TransformStamped& T) -> bool {
    auto it = transforms.find(src);
    if (it != transforms.end()) { T = it->second; return true; }
    try { T = tf->lookupTransform(target, src, tf2::TimePointZero, tf2::durationFromSec(2.0)); }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(nlog(config()), "DetectObject: TF %s -> %s failed: %s", src.c_str(), target.c_str(), e.what());
      return false;
    }
    transforms.emplace(src, T);
    return true;
  };

  auto bb = config().blackboard;
  for (std::size_t n = 0; n < chosen.size(); ++n)
  {
    const auto& d = msg->detections[chosen[n]];
    const auto& h = d.results.front();
    const std::string src = d.header.frame_id.empty() ? msg->header.frame_id : d.header.frame_id;
    geometry_msgs::msg::TransformStamped T;
    if (!transformTo(src, T))
    {
      setOutput("fault", std::string("TF_LOOKUP_FAILED"));
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped in_p, out_p;
    in_p.header.frame_id = src;
    in_p.pose = h.pose.pose;
    tf2::doTransform(in_p, out_p, T);

    const std::string pre = detectionCandidatePrefix(out, keep, n);
    const auto& P = out_p.pose;
    bb->set<std::string>(pre + ".position", fmtVec({P.position.x, P.position.y, P.position.z}));
    bb->set<std::string>(pre + ".orientation", fmtVec({P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w}));
    bb->set<std::string>(pre + ".score", fmtVec({h.hypothesis.score}));
    bb->set<std::string>(pre + ".class_id", h.hypothesis.class_id);
    bb->set<std::string>(pre + ".frame", target);
    bb->set<std::string>(pre + ".size", fmtVec({d.bbox.size.x, d.bbox.size.y, d.bbox.size.z}));
    RCLCPP_INFO(nlog(config()), "DetectObject: '%s' score %.2f at (%.4f, %.4f, %.4f) in %s -> BB[%s.*]",
                h.hypothesis.class_id.c_str(), h.hypothesis.score,
                P.position.x, P.position.y, P.position.z, target.c_str(), pre.c_str());
  }
  if (keep > 1)
  {
    // A STRING, like every other key this node writes — and an integer port binds it
    // anyway (num_attempts="{<out_key>.count}"): BT.CPP converts a string entry to
    // the port's number, and converts this string into an entry a port has already
    // declared int.
    bb->set<std::string>(detectionCountKey(out), std::to_string(chosen.size()));
    RCLCPP_INFO(nlog(config()), "DetectObject: %zu candidate(s) on BB[%s.0 .. %s.%zu] in the order "
                "they were PUBLISHED (that order IS the detector's rank, ADR-0016); BB[%s] = %zu",
                chosen.size(), out.c_str(), out.c_str(), chosen.size() - 1,
                detectionCountKey(out).c_str(), chosen.size());
  }
  return BT::NodeStatus::SUCCESS;
}

// SetWaypointFromDetection: make a data-driven waypoint point at a detection.
//
//   waypoint     the MoveWaypoint name to (re)define
//   from         the DetectObject out_key
//   index        WHICH candidate of the list, default 0 (ADR-0016 Decision 2).
//                DetectObject with keep>1 wrote "<from>.<i>.*"; with keep=1 -- and in
//                every bundle generated before ADR-0016 -- it wrote the FLAT
//                "<from>.*". index=0 accepts both, which is what makes "try the next
//                candidate" expressible without touching a single existing tree:
//                <SetWaypointFromDetection index="{i}" .../> inside a retry decorator.
//                A NON-ZERO index REQUIRES the indexed keys: falling back to the flat
//                single candidate would make a retry loop re-try the SAME candidate,
//                which is the exact failure the candidate list exists to remove.
//   dx dy dz     offset in the target frame (metres) -- dz=0.10 makes a pre-pick above
//   orientation  keep         leave <waypoint>.orientation as bt_params set it
//                from:<wp>    copy another waypoint's orientation (pre_pick from pick)
//                detected     use the detection's orientation (fiducial / learned model)
//                "qx;qy;qz;qw" literal
//   type         "tcp" (default): a waypoint captured as joints becomes a TCP target
//
// Pure blackboard: no ROS, no runtime -- so it is unit-tested by ticking it.
BT::PortsList SetWaypointFromDetection::providedPorts()
{ return {BT::InputPort<std::string>("waypoint"),
          BT::InputPort<std::string>("from", "detected"),
          // INT, not string: the index is driven by a counter, so it arrives as an int
          // blackboard entry ({i}, written by a Script node). A string port would
          // REFUSE AT TREE CREATION to bind an entry already typed int; an int port
          // binds an int entry, a string entry and a literal alike (verified).
          BT::InputPort<int>("index", 0, "which candidate of the list (ADR-0016)"),
          BT::InputPort<std::string>("dx", "0"), BT::InputPort<std::string>("dy", "0"),
          BT::InputPort<std::string>("dz", "0"),
          BT::InputPort<std::string>("orientation", "keep"),
          BT::InputPort<std::string>("type", "tcp")}; }
BT::NodeStatus SetWaypointFromDetection::tick()
{
  auto bb = config().blackboard;
  auto log = safeLog(config());
  const std::string wp = in(*this, "waypoint");
  const std::string from = in(*this, "from", "detected");
  if (wp.empty()) { RCLCPP_ERROR(log, "SetWaypointFromDetection: empty 'waypoint'"); return BT::NodeStatus::FAILURE; }

  int index = 0;
  if (intIn(*this, "index", index) && index < 0)
  {
    RCLCPP_ERROR(log, "SetWaypointFromDetection '%s': index=%d is not a candidate", wp.c_str(), index);
    return BT::NodeStatus::FAILURE;
  }

  // Resolve WHICH candidate, then read everything from that one prefix.
  auto bbGet = [&](const std::string& key, std::string& value) -> bool {
    try { value = bb->get<std::string>(key); return true; }
    catch (const std::exception&) { return false; }
  };
  std::string base = detectionIndexedPrefix(from, index);
  std::string src;
  if (!bbGet(base + ".position", src))
  {
    if (index != 0)
    {
      // Deliberately NOT a fallback: see the `index` note above.
      std::string count;
      const std::string published = bbGet(detectionCountKey(from), count)
                                      ? ("a list of " + count + " candidate(s)")
                                      : std::string("a single candidate (keep=1), not a list");
      RCLCPP_ERROR(log, "SetWaypointFromDetection '%s': no '%s.position' on the blackboard -- "
                   "DetectObject published %s, so there is no candidate %d (raise its `keep`, "
                   "or let the retry end here)",
                   wp.c_str(), base.c_str(), published.c_str(), index);
      return BT::NodeStatus::FAILURE;
    }
    base = from;   // the flat single candidate: keep=1, and every pre-ADR-0016 bundle
    if (!bbGet(base + ".position", src))
    {
      RCLCPP_ERROR(log, "SetWaypointFromDetection: no '%s.position' on the blackboard (run DetectObject first)", from.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  std::array<double, 3> p;
  double dx = 0, dy = 0, dz = 0;
  try
  {
    p = parseXyz3(src);
    dx = std::stod(in(*this, "dx", "0")); dy = std::stod(in(*this, "dy", "0")); dz = std::stod(in(*this, "dz", "0"));
  }
  catch (const std::exception& e) { RCLCPP_ERROR(log, "SetWaypointFromDetection: %s", e.what()); return BT::NodeStatus::FAILURE; }

  bb->set<std::string>(wp + ".position", fmtVec({p[0] + dx, p[1] + dy, p[2] + dz}));
  bb->set<std::string>(wp + ".type", in(*this, "type", "tcp"));

  const std::string ori = in(*this, "orientation", "keep");
  try
  {
    if (ori == "detected")
      bb->set<std::string>(wp + ".orientation", bb->get<std::string>(base + ".orientation"));
    else if (ori.rfind("from:", 0) == 0)
      bb->set<std::string>(wp + ".orientation", bb->get<std::string>(ori.substr(5) + ".orientation"));
    else if (ori != "keep")
    {
      if (parseVec(ori).size() != 4) throw std::runtime_error("orientation literal needs qx;qy;qz;qw");
      bb->set<std::string>(wp + ".orientation", ori);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "SetWaypointFromDetection '%s': orientation '%s': %s", wp.c_str(), ori.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  // a TCP waypoint MUST have an orientation, or MoveWaypoint will refuse it later
  try { bb->get<std::string>(wp + ".orientation"); }
  catch (const std::exception&)
  {
    RCLCPP_ERROR(log, "SetWaypointFromDetection '%s': no orientation -- bt_params has none for it; "
                 "use orientation=\"from:<wp>\", \"detected\" or a literal", wp.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(log, "SetWaypointFromDetection: %s.position = %s (from %s %+.3f %+.3f %+.3f), type=%s, orientation=%s",
              wp.c_str(), bb->get<std::string>(wp + ".position").c_str(), base.c_str(), dx, dy, dz,
              in(*this, "type", "tcp").c_str(), ori.c_str());
  return BT::NodeStatus::SUCCESS;
}

// ─── SetWaypointRelative ──────────────────────────────────────────────────────
// A waypoint DERIVED from another step's FINAL pose (D-017): reads the reference
// waypoint's blackboard pose — which vision may have overwritten this very cycle —
// and writes waypoint = reference (+ dx/dy/dz, + an rpy DELTA), promoted to tcp.
// The canonical use is the retreat: post_pick = pick + 8 cm, wherever the camera
// sent pick, with no second detection (at retreat time the arm occludes the object
// anyway; a relative pose is motion's business, not perception's).
//
// Frames: dx/dy/dz are metres along the BASE-frame axes (same convention as
// SetWaypointFromDetection). droll/dpitch/dyaw are DEGREES, an extrinsic
// base-frame rotation composed ONTO the reference orientation:
//   q_new = Rz(dyaw) * Ry(dpitch) * Rx(droll) * q_ref
// All-zero delta (the common case) copies the reference orientation verbatim.
//
// The reference must carry a pose on the blackboard: a tcp waypoint from
// bt_params, or one a vision/relative node promoted this cycle. Order the tree so
// the reference is set FIRST (the generator emits relatives after vision lines).
//
// Pure blackboard: no ROS, no runtime -- so it is unit-tested by ticking it.
BT::PortsList SetWaypointRelative::providedPorts()
{ return {BT::InputPort<std::string>("waypoint"),
          BT::InputPort<std::string>("from"),
          BT::InputPort<std::string>("dx", "0"), BT::InputPort<std::string>("dy", "0"),
          BT::InputPort<std::string>("dz", "0"),
          BT::InputPort<std::string>("droll", "0"),
          BT::InputPort<std::string>("dpitch", "0"),
          BT::InputPort<std::string>("dyaw", "0"),
          BT::InputPort<std::string>("type", "tcp")}; }
BT::NodeStatus SetWaypointRelative::tick()
{
  auto bb = config().blackboard;
  auto log = safeLog(config());
  const std::string wp = in(*this, "waypoint");
  const std::string from = in(*this, "from");
  if (wp.empty() || from.empty())
  { RCLCPP_ERROR(log, "SetWaypointRelative: 'waypoint' and 'from' are required"); return BT::NodeStatus::FAILURE; }

  std::string src_p, src_q;
  try
  {
    src_p = bb->get<std::string>(from + ".position");
    src_q = bb->get<std::string>(from + ".orientation");
  }
  catch (const std::exception&)
  {
    RCLCPP_ERROR(log, "SetWaypointRelative '%s': reference '%s' has no pose on the blackboard -- "
                 "it must be a tcp waypoint (or vision-promoted) and set BEFORE this node", wp.c_str(), from.c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    const auto p = parseXyz3(src_p);
    const double dx = std::stod(in(*this, "dx", "0"));
    const double dy = std::stod(in(*this, "dy", "0"));
    const double dz = std::stod(in(*this, "dz", "0"));
    const double dr = std::stod(in(*this, "droll", "0")) * M_PI / 180.0;
    const double dp = std::stod(in(*this, "dpitch", "0")) * M_PI / 180.0;
    const double dw = std::stod(in(*this, "dyaw", "0")) * M_PI / 180.0;

    const auto q = parseVec(src_q);
    if (q.size() != 4) throw std::runtime_error("reference orientation needs qx;qy;qz;qw");
    Eigen::Quaterniond q_ref(q[3], q[0], q[1], q[2]);
    Eigen::Quaterniond q_new = q_ref;
    if (dr != 0.0 || dp != 0.0 || dw != 0.0)
    {
      const Eigen::Quaterniond q_delta =
        Eigen::AngleAxisd(dw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(dp, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(dr, Eigen::Vector3d::UnitX());
      q_new = (q_delta * q_ref).normalized();
    }

    bb->set<std::string>(wp + ".position", fmtVec({p[0] + dx, p[1] + dy, p[2] + dz}));
    bb->set<std::string>(wp + ".orientation",
                         fmtVec({q_new.x(), q_new.y(), q_new.z(), q_new.w()}));
    bb->set<std::string>(wp + ".type", in(*this, "type", "tcp"));
    RCLCPP_INFO(log, "SetWaypointRelative: %s = %s %+.3f %+.3f %+.3f (rpy %+.1f %+.1f %+.1f deg) -> %s",
                wp.c_str(), from.c_str(), dx, dy, dz,
                dr * 180.0 / M_PI, dp * 180.0 / M_PI, dw * 180.0 / M_PI,
                bb->get<std::string>(wp + ".position").c_str());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  { RCLCPP_ERROR(log, "SetWaypointRelative '%s': %s", wp.c_str(), e.what()); return BT::NodeStatus::FAILURE; }
}

// ═══ learned-policy execution ═══════════════════════════════════════════════════
// Extension point (POLICY_EXECUTION.md, ADR-0005). These nodes keep TMR torch-free:
// they only speak a ROS contract to the separate Pro package trainit_policy_runtime.
namespace
{
// Generic "latest message on a topic" feed, shared across ticks (like DetectionFeed).
template <typename MsgT>
struct LatestFeed
{
  std::mutex m;
  typename MsgT::SharedPtr latest;
  uint64_t arrivals{0};
  typename rclcpp::Subscription<MsgT>::SharedPtr sub;
};

template <typename MsgT>
LatestFeed<MsgT>& latestFeedFor(const rclcpp::Node::SharedPtr& node,
                                const std::string& topic, const rclcpp::QoS& qos)
{
  static std::map<std::string, std::shared_ptr<LatestFeed<MsgT>>> feeds;
  static std::mutex feeds_m;
  std::lock_guard<std::mutex> lk(feeds_m);
  auto& f = feeds[topic];
  if (!f)
  {
    f = std::make_shared<LatestFeed<MsgT>>();
    LatestFeed<MsgT>* raw = f.get();
    f->sub = node->create_subscription<MsgT>(
      topic, qos, [raw](typename MsgT::SharedPtr msg) {
        std::lock_guard<std::mutex> g(raw->m);
        raw->latest = std::move(msg);
        ++raw->arrivals;
      });
  }
  return *f;
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr triggerClientFor(
  const rclcpp::Node::SharedPtr& node, const std::string& service)
{
  static std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients;
  static std::mutex m;
  std::lock_guard<std::mutex> lk(m);
  auto& c = clients[service];
  if (!c) c = node->create_client<std_srvs::srv::Trigger>(service);
  return c;
}
}  // namespace

// RunPolicy: trigger a run in trainit_policy_runtime (the Pro package that owns the
// policy) and wait for its part to finish. TMR stays torch-free -- it only calls a
// std_srvs/Trigger service and reads a status string; it never loads a policy.
//
//   service       the runtime's run service (default /trainit_policy_runtime/run)
//   status_topic  std_msgs/String JSON status (default /trainit_policy_runtime/status)
//   mode          pure|hybrid|residual (informational + completion criterion)
//   timeout_ms    max wait for pure/residual to run its window
//
// CELL-GLOBAL (ADR-0023): the policy runtime runs at the root until vNext M5, so both defaults
// stay absolute in any namespace; TSA refuses a canonical contract with a cell namespace and a
// policy step (ADR-0022 R14).
//
// hybrid returns as soon as the run service returns (the policy has published its
// decided target on the Detection3D contract -> the existing DetectObject/MoveWaypoint
// move there). pure/residual return once the runtime leaves the "running" state. This
// node only reports whether the policy RAN; the following CheckRobotState judges whether
// it ended in the right state (fail-safe, ADR-0005).
BT::PortsList RunPolicy::providedPorts()
{ return {BT::InputPort<std::string>("service", "/trainit_policy_runtime/run"),
          BT::InputPort<std::string>("status_topic", "/trainit_policy_runtime/status"),
          BT::InputPort<std::string>("mode", "hybrid"),
          BT::InputPort<std::string>("timeout_ms", "10000")}; }
BT::NodeStatus RunPolicy::tick()
{
  auto* ctx = ctxOf(config());
  const std::string service = in(*this, "service", "/trainit_policy_runtime/run");
  const std::string status_topic = in(*this, "status_topic", "/trainit_policy_runtime/status");
  const std::string mode = in(*this, "mode", "hybrid");
  int timeout_ms = 10000;
  try { timeout_ms = std::stoi(in(*this, "timeout_ms", "10000")); } catch (const std::exception&) {}

  auto client = triggerClientFor(ctx->node, service);
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(nlog(config()), "RunPolicy: service '%s' unavailable -- is trainit_policy_runtime launched?",
                 service.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Baseline the status feed BEFORE starting, so we detect the run leaving "running".
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  LatestFeed<std_msgs::msg::String>* status = nullptr;
  uint64_t status_base = 0;
  if (mode != "hybrid")
  {
    status = &latestFeedFor<std_msgs::msg::String>(ctx->node, status_topic, status_qos);
    std::lock_guard<std::mutex> g(status->m); status_base = status->arrivals;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) break;
  }
  if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    RCLCPP_ERROR(nlog(config()), "RunPolicy: run service '%s' did not respond within %d ms",
                 service.c_str(), timeout_ms);
    return BT::NodeStatus::FAILURE;
  }
  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(nlog(config()), "RunPolicy: runtime refused the run: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (mode == "hybrid")
  {
    RCLCPP_INFO(nlog(config()), "RunPolicy(hybrid): target ready (%s)", response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // pure / residual: wait until the runtime leaves "running" (its window elapsed).
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    std::string state;
    {
      std::lock_guard<std::mutex> g(status->m);
      if (status->arrivals > status_base && status->latest) state = status->latest->data;
    }
    if (!state.empty() && state.find("\"running\"") == std::string::npos)
    {
      RCLCPP_INFO(nlog(config()), "RunPolicy(%s): run ended (%s)", mode.c_str(), state.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_WARN(nlog(config()), "RunPolicy(%s): stopped waiting after %d ms (letting CheckRobotState judge)",
              mode.c_str(), timeout_ms);
  return BT::NodeStatus::SUCCESS;
}

// CheckRobotState: assert the robot is in the state a policy card promised (end_state),
// so the next step starts from a known configuration. Pure state check, no motion.
//
//   position           expected tcp position "x;y;z" in the planning frame ("" = skip)
//   position_tolerance metres (default 0.03, the card's success radius)
//   require_attached   "true" to require the suction/attached signal
//   attached_topic     std_msgs/Bool that is true while the object is held ("" = skip)
//   timeout_ms         how long to wait for a fresh attached message
//
// LEGACY (ADR-0022, ADR-0023): the attached half is the legacy grasp check (ADR-0022 F10), and
// attached_topic is used as the project writes it: an absolute name is GLOBAL, shared by every
// robot. TSA never emits the attached half for a canonical project, whose grasp check is
// VerifyGrasp (ADR-0022 D4).
BT::PortsList CheckRobotState::providedPorts()
{ return {BT::InputPort<std::string>("position", ""),
          BT::InputPort<std::string>("position_tolerance", "0.03"),
          BT::InputPort<std::string>("require_attached", "false"),
          BT::InputPort<std::string>("attached_topic", ""),
          BT::InputPort<std::string>("timeout_ms", "3000")}; }
BT::NodeStatus CheckRobotState::tick()
{
  auto* ctx = ctxOf(config());
  const std::string pos_s = in(*this, "position");
  double tol = 0.03;
  try { tol = std::stod(in(*this, "position_tolerance", "0.03")); } catch (const std::exception&) {}
  const bool require_attached = (in(*this, "require_attached", "false") == "true");
  const std::string attached_topic = in(*this, "attached_topic");
  int timeout_ms = 3000;
  try { timeout_ms = std::stoi(in(*this, "timeout_ms", "3000")); } catch (const std::exception&) {}

  // 1) tcp position near the expected pose
  if (!pos_s.empty())
  {
    if (!ctx->runtime)
    { RCLCPP_ERROR(nlog(config()), "CheckRobotState: no runtime to read the current pose"); return BT::NodeStatus::FAILURE; }
    std::array<double, 3> want{};
    try { want = parseXyz3(pos_s); }
    catch (const std::exception& e)
    { RCLCPP_ERROR(nlog(config()), "CheckRobotState: bad 'position': %s", e.what()); return BT::NodeStatus::FAILURE; }
    const auto cur = currentPose(ctx).position;
    const double dx = cur.x - want[0], dy = cur.y - want[1], dz = cur.z - want[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > tol)
    {
      RCLCPP_ERROR(nlog(config()), "CheckRobotState: tcp %.4f m from expected (tol %.4f) -- policy did not reach the end pose",
                   dist, tol);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(nlog(config()), "CheckRobotState: tcp within %.4f m of expected (tol %.4f)", dist, tol);
  }

  // 2) suction/attached
  if (require_attached)
  {
    if (attached_topic.empty())
    { RCLCPP_ERROR(nlog(config()), "CheckRobotState: require_attached set but no 'attached_topic'"); return BT::NodeStatus::FAILURE; }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto& feed = latestFeedFor<std_msgs::msg::Bool>(ctx->node, attached_topic, qos);
    uint64_t seen; { std::lock_guard<std::mutex> g(feed.m); seen = feed.arrivals; }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std_msgs::msg::Bool::SharedPtr msg;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      { std::lock_guard<std::mutex> g(feed.m); if (feed.arrivals > seen) { msg = feed.latest; break; } }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!msg)
    { RCLCPP_ERROR(nlog(config()), "CheckRobotState: no attached signal on '%s' within %d ms", attached_topic.c_str(), timeout_ms); return BT::NodeStatus::FAILURE; }
    if (!msg->data)
    { RCLCPP_ERROR(nlog(config()), "CheckRobotState: object NOT attached -- policy ended without the object"); return BT::NodeStatus::FAILURE; }
    RCLCPP_INFO(nlog(config()), "CheckRobotState: object attached");
  }

  return BT::NodeStatus::SUCCESS;
}

// ═══ online (servo) motion — ADR-0015 ═════════════════════════════════════════
// ServoTrack: reach a waypoint by STREAMING a cartesian velocity to an
// OUT-OF-PROCESS moveit_servo node, instead of planning a trajectory and
// executing it. This is the `execution: servo` axis of ADR-0015 — orthogonal to
// `motion` (ptp|lin|free|circ, which are plan SHAPES) and to PlannerMode.
//
// WHY A BT NODE AND NOT A BACKEND (ADR-0015 Decision 5). moveit_servo 2.5.10 on
// Humble is a NODE, not a library: ServoNode is driven entirely through ROS — four
// std_srvs/Trigger services (~/start_servo, ~/stop_servo, ~/pause_servo,
// ~/unpause_servo), a TwistStamped input and one std_msgs/Int8 status output
// (moveit_servo/servo_node.h, servo_calcs.cpp). So this node is a service client
// plus a publisher — structurally the same as SetAttachedCollisionCheck and
// ResetScene — and `moveit_servo` stays OFF the Community library's link line
// (ADR-0003: Community must build and run without proprietary or heavy extras).
//
// DATA. The single port is the waypoint NAME, exactly like MoveWaypoint, and it
// reads the same blackboard keys: <wp>.type, <wp>.position, <wp>.orientation,
// <wp>.frame. A waypoint whose pose is written at RUN TIME (vision, or a relative
// binding) therefore needs no second representation — and that is the only case
// where streaming buys anything over a planned move (ADR-0015 Decision 4).
// The orientation is tracked only if the waypoint carries one; a position-only
// waypoint is a position-only correction.
//
// NAMES AND NAMESPACES. `servo_node` is RELATIVE by default, so the five names are
// resolved against the node that runs the tree: a tree launched under /r3 talks to
// /r3/servo_node/* with no configuration. Pass an ABSOLUTE `servo_node`
// (e.g. "/r3/servo_node") to reach a servo node in another namespace.
//
// ARBITRATION — ONE OWNER PER CONTROLLER (ADR-0015, a requirement not a nicety).
// JointTrajectoryController accepts the servo topic while an action goal is in
// flight and the two then fight over the same controller; the goal aborts on a
// tolerance violation with no useful message. So:
//   * before streaming, this node REFUSES to run while a trajectory execution is
//     in flight (it watches move_group's execute_trajectory action status);
//   * it unpauses servo before streaming and PAUSES it on EVERY exit path
//     (success, failure, timeout, bad status), so servo never holds the controller
//     while a planned move executes. The planned path is untouched — that is how
//     `execution: plan` keeps emitting zero bytes;
//   * start_servo is called ONCE per process, never per motion: ServoNode::startCB
//     joins and respawns the servo calc thread and waits up to 10 s for a complete
//     robot state. Per-motion ownership is pause/unpause, which is cheap.
//
// A FRESH header.stamp ON EVERY MESSAGE IS NOT OPTIONAL. Verified by execution: a
// twist published without a live stamp never moves the robot and produces no error
// whatsoever. The mechanism is in servo_calcs.cpp: twistStampedCB only updates
// latest_twist_command_stamp_ when header.stamp != 0, and the main loop marks the
// command stale once (now - that stamp) >= incoming_command_timeout (0.1 s), after
// which it publishes halt messages instead of motion. A zero or reused stamp is
// therefore permanently stale — silently.
//
// MEASURED/DERIVED DEFAULTS (all are ports; these are the documented defaults).
// Two sources of truth, and the defaults satisfy BOTH: moveit_servo 2.5.10's own
// defaults, read in the installed servo_parameters.h (publish_period 0.034 s,
// incoming_command_timeout 0.1 s, command_in_type "unitless", linear_scale 0.4 m/s,
// rotational_scale 0.8 rad/s), and the servo.yaml the TSA generates for a cell that
// has a servo move (publish_period 0.01 s, incoming_command_timeout 0.2 s,
// command_in_type "speed_units", scale caps 0.2 m/s and 0.5 rad/s).
//   command_in_type = speed_units   what the generated servo.yaml sets, so a
//                       ServoTrack velocity means the same thing on every cell. The
//                       generated tree emits <ServoTrack waypoint="..."/> and nothing
//                       else, so every default here must match that file.
//   rate_hz = 50        20 ms between commands: a 10x margin on the generated
//                       0.2 s stale threshold (5x on the moveit_servo default
//                       0.1 s), so one late cycle cannot halt the stream.
//                       Below 10 Hz the stream is stale between commands.
//   max_linear_vel  = 0.05 m/s   conservative industrial default for a streamed
//   max_angular_vel = 0.30 rad/s correction; also the excursion bound below.
//   tolerance_m   = 0.002   one servo cycle at the speed cap travels
//                           0.05 * 0.034 = 1.7 mm with the moveit_servo default
//                           publish_period (0.5 mm with the generated 0.01 s), so a
//                           tolerance under ~2 mm cannot be hit reliably on either —
//                           the loop would chatter through it instead of converging.
//   tolerance_deg = 1.0     same arithmetic: 0.30 * 0.034 = 0.58 deg per cycle.
//   timeout_ms    = 5000    bounds the excursion as much as the wait:
//                           5 s * 0.05 m/s = 0.25 m of streamed travel, after
//                           which the node fails instead of creeping on.
//   gain = 2.0 [1/s]        proportional: speed = min(gain * error, cap), so the
//                           approach decelerates inside 25 mm and the last
//                           millimetres are walked, not overshot.
// `linear_scale`/`angular_scale` matter ONLY in "unitless" mode, where servo
// multiplies the command by them (servo_calcs.cpp, scaleCartesianCommand) and this
// node divides by them first so the caps still mean m/s and rad/s. With
// "speed_units" — the default, and what the generated cell uses — the twist IS m/s
// and rad/s and the two ports are unused.
//
// FRAMES. The error is computed in the frame MotionRuntime::currentTipPose() reports (the
// robot model frame), and the twist is stamped with THAT frame, which must be servo's
// `planning_frame` (servo then uses the twist verbatim) — override `command_frame`
// only if the two genuinely differ. Note servo resolves a command frame through
// getGlobalLinkTransform(), so it must be a LINK of the robot model.
//
// FAILURE MODE. Unlike SetAttachedCollisionCheck/ResetScene, an absent service is
// FATAL here: a missing flag service is a no-op, but a missing servo node means
// THE MOVE DID NOT HAPPEN, and returning SUCCESS for a move that never ran is the
// silent wrong move this ADR is fixing. Only the clean-up pause is non-fatal.
namespace
{
// start_servo is called once per process per servo node (see the header comment).
class ServoStartRegistry
{
public:
  static bool alreadyStarted(const std::string& service)
  { std::lock_guard<std::mutex> lk(mutex()); return services().count(service) != 0; }
  static void markStarted(const std::string& service)
  { std::lock_guard<std::mutex> lk(mutex()); services().insert(service); }

private:
  static std::set<std::string>& services() { static std::set<std::string> s; return s; }
  static std::mutex& mutex() { static std::mutex m; return m; }
};

// One publisher per topic, shared across ticks (like latestFeedFor): a publisher
// created per tick would spend the first commands in DDS discovery, unmatched.
template <typename MsgT>
typename rclcpp::Publisher<MsgT>::SharedPtr publisherFor(
  const rclcpp::Node::SharedPtr& node, const std::string& topic, const rclcpp::QoS& qos)
{
  static std::map<std::string, typename rclcpp::Publisher<MsgT>::SharedPtr> pubs;
  static std::mutex m;
  std::lock_guard<std::mutex> lk(m);
  auto& p = pubs[topic];
  if (!p) p = node->create_publisher<MsgT>(topic, qos);
  return p;
}

enum class TriggerOutcome { OK, REFUSED, ABSENT, NO_RESPONSE };

TriggerOutcome callTrigger(const rclcpp::Node::SharedPtr& node, const std::string& service,
                           std::chrono::milliseconds discover, std::chrono::milliseconds budget,
                           std::string& message)
{
  auto client = triggerClientFor(node, service);
  if (!client->wait_for_service(discover)) return TriggerOutcome::ABSENT;
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(budget) != std::future_status::ready) return TriggerOutcome::NO_RESPONSE;
  auto response = future.get();
  message = response->message;
  return response->success ? TriggerOutcome::OK : TriggerOutcome::REFUSED;
}

// moveit_servo::StatusCode (moveit_servo/status_codes.h) as the readable text a
// tree author needs. 2 | 4 | 5 are HALTS: servo stopped the robot, so the move
// cannot converge and the node must fail.
const char* servoStatusText(int code)
{
  switch (code)
  {
    case 0:  return "no warning";
    case 1:  return "decelerating: approaching a singularity";
    case 2:  return "SINGULARITY: too close to a singularity, servo halted the robot";
    case 3:  return "decelerating: close to a collision";
    case 4:  return "COLLISION: servo halted the robot";
    case 5:  return "JOINT_BOUND: at a joint position/velocity limit, servo halted the robot";
    case 6:  return "decelerating: leaving a singularity";
    case -1: return "invalid";
    default: return "unrecognised status code";
  }
}
bool servoStatusIsHalt(int code) { return code == 2 || code == 4 || code == 5; }

// Is a trajectory execution in flight? Reads move_group's ExecuteTrajectory action
// status (transient_local, so the last array is latched for a late subscriber).
// No publisher / no message == nothing in flight.
bool trajectoryExecutionActive(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                               std::chrono::milliseconds grace, std::string& detail)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto& feed = latestFeedFor<action_msgs::msg::GoalStatusArray>(node, topic, qos);
  const auto deadline = std::chrono::steady_clock::now() + grace;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    { std::lock_guard<std::mutex> g(feed.m); if (feed.latest) break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  action_msgs::msg::GoalStatusArray::SharedPtr msg;
  { std::lock_guard<std::mutex> g(feed.m); msg = feed.latest; }
  if (!msg) { detail = "no execute_trajectory status published"; return false; }
  for (const auto& s : msg->status_list)
  {
    if (s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        s.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
    { detail = "goal status " + std::to_string(static_cast<int>(s.status)); return true; }
  }
  detail = "no goal in flight";
  return false;
}
}  // namespace

BT::PortsList ServoTrack::providedPorts()
{ return {BT::InputPort<std::string>("waypoint"),
          // relative by default -> a tree under /r3 talks to /r3/servo_node/*
          BT::InputPort<std::string>("servo_node", "servo_node"),
          BT::InputPort<std::string>("command_frame", ""),            // "" = the current-pose frame
          // speed_units, because that is what the TSA's generated servo.yaml sets:
          // a ServoTrack velocity then means the same thing on every cell
          BT::InputPort<std::string>("command_in_type", "speed_units"),
          BT::InputPort<std::string>("linear_scale", "0.4"),          // ditto (unitless only)
          BT::InputPort<std::string>("angular_scale", "0.8"),         // ditto (unitless only)
          BT::InputPort<std::string>("tolerance_m", "0.002"),
          BT::InputPort<std::string>("tolerance_deg", "1.0"),
          BT::InputPort<std::string>("timeout_ms", "5000"),
          BT::InputPort<std::string>("rate_hz", "50"),
          BT::InputPort<std::string>("max_linear_vel", "0.05"),
          BT::InputPort<std::string>("max_angular_vel", "0.30"),
          BT::InputPort<std::string>("gain", "2.0"),
          BT::InputPort<std::string>("execution_status_topic", "execute_trajectory/_action/status"),
          BT::InputPort<std::string>("start_servo", "true")}; }
BT::NodeStatus ServoTrack::tick()
{
  auto log = safeLog(config());
  const std::string wp = in(*this, "waypoint");
  if (wp.empty()) { RCLCPP_ERROR(log, "ServoTrack: empty 'waypoint'"); return BT::NodeStatus::FAILURE; }

  auto field = [&](const std::string& f, const std::string& def = "") -> std::string {
    try { return config().blackboard->get<std::string>(wp + "." + f); }
    catch (const std::exception&) { return def; }
  };
  auto num = [&](const char* port, double def) -> double {
    try { return std::stod(in(*this, port, std::to_string(def))); }
    catch (const std::exception&) { return def; }
  };

  // --- the target: the SAME <wp>.* keys MoveWaypoint reads --------------------
  const std::string type = field("type", "tcp");
  const std::string pos_s = field("position");
  if (type == "joint" || pos_s.empty())
  {
    RCLCPP_ERROR(log, "ServoTrack '%s': needs a cartesian target in '%s.position' (type=%s). "
                 "Servo streams a TCP correction; a joint/named waypoint is a planned move "
                 "(execution: plan).", wp.c_str(), wp.c_str(), type.c_str());
    return BT::NodeStatus::FAILURE;
  }
  Eigen::Vector3d target_p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_q = Eigen::Quaterniond::Identity();
  bool track_orientation = false;
  try
  {
    const auto p = parseXyz3(pos_s);
    target_p = Eigen::Vector3d(p[0], p[1], p[2]);
    const std::string q_s = field("orientation");
    if (!q_s.empty())
    {
      const auto q = parseVec(q_s);
      if (q.size() != 4) throw std::runtime_error("orientation needs qx;qy;qz;qw");
      target_q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
      track_orientation = true;
    }
  }
  catch (const std::exception& e)
  { RCLCPP_ERROR(log, "ServoTrack '%s': %s", wp.c_str(), e.what()); return BT::NodeStatus::FAILURE; }

  auto* ctx = ctxOf(config());
  if (!ctx->runtime)
  { RCLCPP_ERROR(log, "ServoTrack '%s': no runtime to read the current pose", wp.c_str()); return BT::NodeStatus::FAILURE; }

  // --- parameters ------------------------------------------------------------
  const std::string servo_node = in(*this, "servo_node", "servo_node");
  const std::string cmd_type   = in(*this, "command_in_type", "speed_units");
  const double lin_scale       = std::max(1e-6, num("linear_scale", 0.4));
  const double ang_scale       = std::max(1e-6, num("angular_scale", 0.8));
  const double tol_m           = std::max(1e-6, num("tolerance_m", 0.002));
  const double tol_rad         = std::max(1e-6, num("tolerance_deg", 1.0)) * M_PI / 180.0;
  const double rate_hz         = std::clamp(num("rate_hz", 50.0), 10.0, 500.0);
  const double max_lin         = std::max(1e-6, num("max_linear_vel", 0.05));
  const double max_ang         = std::max(1e-6, num("max_angular_vel", 0.30));
  const double gain            = std::max(1e-6, num("gain", 2.0));
  const int    timeout_ms      = static_cast<int>(num("timeout_ms", 5000.0));
  const std::string exec_topic = in(*this, "execution_status_topic", "");
  const bool do_start          = (in(*this, "start_servo", "true") != "false");
  const bool unitless          = (cmd_type != "speed_units");

  const std::string twist_topic  = servo_node + "/delta_twist_cmds";
  const std::string status_topic = servo_node + "/status";
  const std::string start_srv    = servo_node + "/start_servo";
  const std::string pause_srv    = servo_node + "/pause_servo";
  const std::string unpause_srv  = servo_node + "/unpause_servo";

  if (unitless && max_lin > lin_scale)
    RCLCPP_WARN(log, "ServoTrack: max_linear_vel %.3f m/s exceeds the servo node's linear_scale "
                "%.3f m/s — a unitless command saturates at the scale", max_lin, lin_scale);
  if (unitless && max_ang > ang_scale)
    RCLCPP_WARN(log, "ServoTrack: max_angular_vel %.3f rad/s exceeds the servo node's "
                "rotational_scale %.3f rad/s — a unitless command saturates at the scale",
                max_ang, ang_scale);

  // --- the target must already be in the frame we measure in -----------------
  // The FULL read, not a frame accessor: it also waits (up to 1 s) for a fresh state before
  // start_servo, as the read through moveGroup() did before vNext M2.5.
  const std::string pose_frame = ctx->runtime->currentTipPose().header.frame_id;
  const std::string wp_frame = field("frame");
  if (!wp_frame.empty() && !pose_frame.empty() && wp_frame != pose_frame)
  {
    RCLCPP_ERROR(log, "ServoTrack '%s': the target is in frame '%s' but the arm is measured in "
                 "'%s' — no TF is applied here; make the detection publish into '%s' "
                 "(DetectObject target_frame)", wp.c_str(), wp_frame.c_str(),
                 pose_frame.c_str(), pose_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }
  const std::string cmd_frame = in(*this, "command_frame", pose_frame);

  // --- ARBITRATION: refuse to stream while a trajectory execution is in flight -
  if (!exec_topic.empty())
  {
    std::string detail;
    if (trajectoryExecutionActive(ctx->node, exec_topic, std::chrono::milliseconds(200), detail))
    {
      RCLCPP_ERROR(log, "ServoTrack '%s': a trajectory execution is in flight (%s on '%s') — "
                   "one owner per controller (ADR-0015); refusing to stream", wp.c_str(),
                   detail.c_str(), exec_topic.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // --- take ownership: start once, then unpause ------------------------------
  std::string srv_msg;
  if (do_start && !ServoStartRegistry::alreadyStarted(start_srv))
  {
    // 15 s budget: startCB joins and respawns the calc thread and waits up to 10 s
    // for a complete robot state.
    const auto outcome = callTrigger(ctx->node, start_srv, std::chrono::milliseconds(2000),
                                     std::chrono::milliseconds(15000), srv_msg);
    if (outcome == TriggerOutcome::ABSENT)
    {
      RCLCPP_ERROR(log, "ServoTrack '%s': '%s' unavailable — there is no servo node here. "
                   "`execution: servo` needs the servo_node launch entry (ADR-0015).",
                   wp.c_str(), start_srv.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (outcome != TriggerOutcome::OK)
    {
      RCLCPP_ERROR(log, "ServoTrack '%s': start_servo failed (%s) — a servo node started without "
                   "robot_description_planning emits NaN; check joint_limits.yaml", wp.c_str(),
                   srv_msg.empty() ? "no response" : srv_msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
    ServoStartRegistry::markStarted(start_srv);
    RCLCPP_INFO(log, "ServoTrack: servo started once via '%s'", start_srv.c_str());
  }

  switch (callTrigger(ctx->node, unpause_srv, std::chrono::milliseconds(2000),
                      std::chrono::milliseconds(5000), srv_msg))
  {
    case TriggerOutcome::OK:
      break;
    case TriggerOutcome::ABSENT:
      // non-fatal: a servo node without the pause pair is already streaming.
      RCLCPP_WARN(log, "ServoTrack: '%s' unavailable — streaming without unpause",
                  unpause_srv.c_str());
      break;
    default:
      RCLCPP_ERROR(log, "ServoTrack '%s': unpause_servo failed (%s) — servo would stay paused and "
                   "the arm would not move", wp.c_str(),
                   srv_msg.empty() ? "no response" : srv_msg.c_str());
      return BT::NodeStatus::FAILURE;
  }

  // servo subscribes with the rclcpp defaults (SystemDefaultsQoS == RELIABLE), so a
  // best-effort publisher here would never be matched.
  auto twist_pub = publisherFor<geometry_msgs::msg::TwistStamped>(
    ctx->node, twist_topic, rclcpp::SystemDefaultsQoS());
  auto& status = latestFeedFor<std_msgs::msg::Int8>(ctx->node, status_topic,
                                                    rclcpp::SystemDefaultsQoS());
  uint64_t status_base = 0;
  { std::lock_guard<std::mutex> g(status.m); status_base = status.arrivals; }

  // Every exit path hands the controller back: a zero twist (a deterministic stop,
  // not a stale-command halt) and then pause. ONE OWNER PER CONTROLLER.
  auto release = [&](BT::NodeStatus st) {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.frame_id = cmd_frame;
    zero.header.stamp = ctx->node->now();
    twist_pub->publish(zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));   // let the zero be consumed
    std::string m;
    const auto out = callTrigger(ctx->node, pause_srv, std::chrono::milliseconds(500),
                                 std::chrono::milliseconds(2000), m);
    if (out != TriggerOutcome::OK && out != TriggerOutcome::ABSENT)
      RCLCPP_WARN(log, "ServoTrack: pause_servo did not confirm (%s) — servo may still own the "
                  "controller and a planned move would fight it",
                  m.empty() ? "no response" : m.c_str());
    return st;
  };

  // The twist must reach servo: an unmatched publisher streams into the void, which
  // is exactly the silent no-move this node exists to prevent.
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (twist_pub->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline && rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (twist_pub->get_subscription_count() == 0)
    {
      RCLCPP_ERROR(log, "ServoTrack '%s': nothing subscribes to '%s' — the servo node is not "
                   "listening (wrong name or namespace?)", wp.c_str(), twist_topic.c_str());
      return release(BT::NodeStatus::FAILURE);
    }
  }

  // --- stream until converged, timed out, or halted by servo ------------------
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  const auto period = std::chrono::microseconds(static_cast<int64_t>(1.0e6 / rate_hz));
  const auto unitClamp = [](double v, double k, bool clamp_unit) {
    const double out = v * k;
    return clamp_unit ? std::clamp(out, -1.0, 1.0) : out;
  };
  double dist = 0.0, angle = 0.0;
  uint64_t cycles = 0;
  while (rclcpp::ok())
  {
    // 1) where are we? (currentTipPose copies the monitored state — fine at 50 Hz on
    //    the BT thread, which is not a real-time control path)
    const auto cur = ctx->runtime->currentTipPose().pose;
    const Eigen::Vector3d cur_p(cur.position.x, cur.position.y, cur.position.z);
    const Eigen::Vector3d err_p = target_p - cur_p;
    dist = err_p.norm();

    Eigen::Vector3d err_r = Eigen::Vector3d::Zero();
    angle = 0.0;
    if (track_orientation)
    {
      Eigen::Quaterniond cur_q(cur.orientation.w, cur.orientation.x, cur.orientation.y,
                               cur.orientation.z);
      if (cur_q.norm() < 1e-9) cur_q = Eigen::Quaterniond::Identity();
      cur_q.normalize();
      Eigen::Quaterniond dq = target_q * cur_q.inverse();
      if (dq.w() < 0.0) dq.coeffs() *= -1.0;          // shortest arc
      const Eigen::AngleAxisd aa(dq);
      angle = aa.angle();
      if (angle > 1e-9) err_r = aa.axis() * angle;
    }

    // 2) converged?
    if (dist <= tol_m && angle <= tol_rad)
    {
      RCLCPP_INFO(log, "ServoTrack '%s': converged in %llu cycles — %.4f m, %.2f deg "
                  "(tol %.4f m, %.2f deg)", wp.c_str(), static_cast<unsigned long long>(cycles),
                  dist, angle * 180.0 / M_PI, tol_m, tol_rad * 180.0 / M_PI);
      return release(BT::NodeStatus::SUCCESS);
    }

    // 3) what does servo say? (only what it published since we started streaming)
    {
      std::lock_guard<std::mutex> g(status.m);
      if (status.arrivals > status_base && status.latest)
      {
        const int code = static_cast<int>(status.latest->data);
        if (servoStatusIsHalt(code))
        {
          RCLCPP_ERROR(log, "ServoTrack '%s': servo status %d — %s (%.4f m short of the target)",
                       wp.c_str(), code, servoStatusText(code), dist);
          return release(BT::NodeStatus::FAILURE);
        }
        if (code != 0)
          RCLCPP_WARN_THROTTLE(log, *ctx->node->get_clock(), 1000,
                               "ServoTrack '%s': servo status %d — %s",
                               wp.c_str(), code, servoStatusText(code));
      }
    }

    // 4) timed out?
    if (std::chrono::steady_clock::now() >= deadline)
    {
      RCLCPP_ERROR(log, "ServoTrack '%s': did not converge within %d ms — %.4f m / %.2f deg short "
                   "(tol %.4f m / %.2f deg). Does the servo node's command_in_type match '%s', "
                   "and its scales %.2f / %.2f? Does its output controller hold the arm?",
                   wp.c_str(), timeout_ms, dist, angle * 180.0 / M_PI, tol_m,
                   tol_rad * 180.0 / M_PI, cmd_type.c_str(), lin_scale, ang_scale);
      return release(BT::NodeStatus::FAILURE);
    }

    // 5) proportional approach, capped. A unitless command is divided by the servo
    //    node's own scales, so the cap means m/s and rad/s either way.
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = cmd_frame;
    cmd.header.stamp = ctx->node->now();   // FRESH ON EVERY MESSAGE — see the comment above
    if (dist > tol_m)
    {
      const Eigen::Vector3d v = err_p.normalized() * std::min(gain * dist, max_lin);
      const double k = unitless ? 1.0 / lin_scale : 1.0;
      cmd.twist.linear.x = unitClamp(v.x(), k, unitless);
      cmd.twist.linear.y = unitClamp(v.y(), k, unitless);
      cmd.twist.linear.z = unitClamp(v.z(), k, unitless);
    }
    if (track_orientation && angle > tol_rad)
    {
      const Eigen::Vector3d w = err_r.normalized() * std::min(gain * angle, max_ang);
      const double k = unitless ? 1.0 / ang_scale : 1.0;
      cmd.twist.angular.x = unitClamp(w.x(), k, unitless);
      cmd.twist.angular.y = unitClamp(w.y(), k, unitless);
      cmd.twist.angular.z = unitClamp(w.z(), k, unitless);
    }
    twist_pub->publish(cmd);
    ++cycles;
    std::this_thread::sleep_for(period);
  }

  RCLCPP_ERROR(log, "ServoTrack '%s': interrupted (rclcpp shutting down), %.4f m short",
               wp.c_str(), dist);
  return release(BT::NodeStatus::FAILURE);
}

// ═══ registration ═════════════════════════════════════════════════════════════
void registerAllNodes(BT::BehaviorTreeFactory& f)
{
  f.registerNodeType<MoveWaypoint>("MoveWaypoint");
  f.registerNodeType<MovePtp>("MovePtp");
  f.registerNodeType<MoveToNamed>("MoveToNamed");
  f.registerNodeType<MoveToJoint>("MoveToJoint");
  f.registerNodeType<MoveLinear>("MoveLinear");
  f.registerNodeType<MoveCircular>("MoveCircular");
  f.registerNodeType<MoveCollisionFree>("MoveCollisionFree");
  f.registerNodeType<MoveCollisionFree>("MoveFree");   // alias
  f.registerNodeType<ServoTrack>("ServoTrack");        // ADR-0015: execution: servo
  f.registerNodeType<ExecuteProcessPath>("ExecuteProcessPath");
  f.registerNodeType<OpenGripper>("OpenGripper");
  f.registerNodeType<CloseGripper>("CloseGripper");
  f.registerNodeType<SetGripper>("SetGripper");
  f.registerNodeType<CommandActuator>("CommandActuator");   // ADR-0022
  f.registerNodeType<VerifyGrasp>("VerifyGrasp");           // ADR-0022
  f.registerNodeType<EnableProcess>("EnableProcess");
  f.registerNodeType<DisableProcess>("DisableProcess");
  f.registerNodeType<AddCollisionObject>("AddCollisionObject");
  f.registerNodeType<RemoveCollisionObject>("RemoveCollisionObject");
  f.registerNodeType<AttachObject>("AttachObject");
  f.registerNodeType<DetachObject>("DetachObject");
  f.registerNodeType<SetAttachedCollisionCheck>("SetAttachedCollisionCheck");
  f.registerNodeType<ResetScene>("ResetScene");
  f.registerNodeType<SetReleasePolicy>("SetReleasePolicy");
  f.registerNodeType<MakePose>("MakePose");
  f.registerNodeType<StoreCurrentPose>("StoreCurrentPose");
  f.registerNodeType<ComputeTcpTarget>("ComputeTcpTarget");
  f.registerNodeType<OffsetPoseInToolFrame>("OffsetPoseInToolFrame");
  f.registerNodeType<OffsetPoseInBaseFrame>("OffsetPoseInBaseFrame");
  f.registerNodeType<DetectObject>("DetectObject");
  f.registerNodeType<SetWaypointFromDetection>("SetWaypointFromDetection");
  f.registerNodeType<SetWaypointRelative>("SetWaypointRelative");
  f.registerNodeType<RunPolicy>("RunPolicy");
  f.registerNodeType<CheckRobotState>("CheckRobotState");
  f.registerNodeType<Wait>("Wait");
  f.registerNodeType<Log>("Log");
}

}  // namespace trainit
