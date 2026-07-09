// =============================================================================
// bt_nodes.cpp — generic BT nodes wrapping the TrainIt runtime.
// =============================================================================
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/process_path_generator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

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
  return ctx->runtime->moveGroup().getCurrentPose(ctx->runtime->tipLink()).pose;
}

// read a string input port, returning a fallback if unresolved/empty
std::string in(BT::TreeNode& n, const char* port, const std::string& def = "")
{
  auto e = n.getInput<std::string>(port);
  return (e && !e.value().empty()) ? e.value() : def;
}

rclcpp::Logger nlog(const BT::NodeConfig& cfg)
{
  return ctxOf(cfg)->node->get_logger();
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
BT::PortsList MoveWaypoint::providedPorts()
{ return {BT::InputPort<std::string>("waypoint")}; }
BT::NodeStatus MoveWaypoint::tick()
{
  auto* ctx = ctxOf(config());
  const std::string wp = in(*this, "waypoint");
  if (wp.empty()) { RCLCPP_ERROR(nlog(config()), "MoveWaypoint: empty 'waypoint'"); return BT::NodeStatus::FAILURE; }

  auto field = [&](const std::string& f, const std::string& def = "") -> std::string {
    try { return config().blackboard->get<std::string>(wp + "." + f); }
    catch (const std::exception&) { return def; }
  };

  const std::string type    = field("type", "tcp");
  const std::string motion  = field("motion", "free");
  const std::string planner = field("planner");
  const std::string speed   = field("speed");
  const std::string accel   = field("accel");

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

  const std::string tip = ctx->runtime->tipLink();
  MotionResult r;
  try
  {
    if (motion == "lin" || motion == "linear")
    {
      CartesianTarget t; t.pose = buildPose(); t.tip_link = tip;
      r = ctx->runtime->moveLinear(t, opts);
    }
    else if (motion == "circ" || motion == "circular")
    {
      CircularTarget t; t.goal = buildPose(); t.tip_link = tip;
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
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(nlog(config()), "MoveWaypoint '%s' (type=%s motion=%s planner=%s speed=%s%%)",
              wp.c_str(), type.c_str(), motion.c_str(),
              planner.empty() ? "mode" : planner.c_str(), speed.empty() ? "profile" : speed.c_str());
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
BT::PortsList OpenGripper::providedPorts() { return {}; }
BT::NodeStatus OpenGripper::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->gripper) { RCLCPP_ERROR(nlog(config()), "OpenGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  return ctx->gripper->open() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CloseGripper::providedPorts() { return {}; }
BT::NodeStatus CloseGripper::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->gripper) { RCLCPP_ERROR(nlog(config()), "CloseGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  return ctx->gripper->close() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList SetGripper::providedPorts() { return {BT::InputPort<std::string>("position", "1.0")}; }
BT::NodeStatus SetGripper::tick()
{
  auto* ctx = ctxOf(config());
  if (!ctx->gripper) { RCLCPP_ERROR(nlog(config()), "SetGripper: no gripper configured"); return BT::NodeStatus::FAILURE; }
  return ctx->gripper->command(std::stod(in(*this, "position", "1.0"))) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    ctx->runtime->scene().addCollisionBox(in(*this, "id"), in(*this, "frame", "base_link"),
      parseXyz3(in(*this, "dims")), poseFromXyz(parseXyz3(in(*this, "position", "0;0;0"))));
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "AddCollisionObject: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList RemoveCollisionObject::providedPorts() { return {BT::InputPort<std::string>("id")}; }
BT::NodeStatus RemoveCollisionObject::tick()
{
  ctxOf(config())->runtime->scene().removeCollisionObject(in(*this, "id"));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList AttachObject::providedPorts()
{ return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("link", "tcp"),
          BT::InputPort<std::string>("dims"), BT::InputPort<std::string>("position", "0;0;0")}; }
BT::NodeStatus AttachObject::tick()
{
  auto* ctx = ctxOf(config());
  try {
    ctx->runtime->scene().attachBox(in(*this, "id"), in(*this, "link", "tcp"),
      parseXyz3(in(*this, "dims")), poseFromXyz(parseXyz3(in(*this, "position", "0;0;0"))));
  } catch (const std::exception& e) { RCLCPP_ERROR(nlog(config()), "AttachObject: %s", e.what()); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList DetachObject::providedPorts()
{ return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("link", "tcp")}; }
BT::NodeStatus DetachObject::tick()
{
  ctxOf(config())->runtime->scene().detachObject(in(*this, "id"), in(*this, "link", "tcp"));
  return BT::NodeStatus::SUCCESS;
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
  f.registerNodeType<ExecuteProcessPath>("ExecuteProcessPath");
  f.registerNodeType<OpenGripper>("OpenGripper");
  f.registerNodeType<CloseGripper>("CloseGripper");
  f.registerNodeType<SetGripper>("SetGripper");
  f.registerNodeType<EnableProcess>("EnableProcess");
  f.registerNodeType<DisableProcess>("DisableProcess");
  f.registerNodeType<AddCollisionObject>("AddCollisionObject");
  f.registerNodeType<RemoveCollisionObject>("RemoveCollisionObject");
  f.registerNodeType<AttachObject>("AttachObject");
  f.registerNodeType<DetachObject>("DetachObject");
  f.registerNodeType<MakePose>("MakePose");
  f.registerNodeType<StoreCurrentPose>("StoreCurrentPose");
  f.registerNodeType<ComputeTcpTarget>("ComputeTcpTarget");
  f.registerNodeType<OffsetPoseInToolFrame>("OffsetPoseInToolFrame");
  f.registerNodeType<OffsetPoseInBaseFrame>("OffsetPoseInBaseFrame");
  f.registerNodeType<Wait>("Wait");
  f.registerNodeType<Log>("Log");
}

}  // namespace trainit
