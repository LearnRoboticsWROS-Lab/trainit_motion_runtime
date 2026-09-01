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
#include <future>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <map>
#include <memory>
#include <mutex>

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

// ═══ dynamic-object runtime flags ═════════════════════════════════════════════
// SetAttachedCollisionCheck: flips the scene_manager_node planning-collision check
// for GRASPED objects (its SetBool service). ON => the planner routes the held
// payload around the static/actuated meshes (e.g. into the prewash); OFF => the
// payload is transparent to them (e.g. at the pick, where it sits on the belt).
// BLOCKS on the service response so the ACM is applied BEFORE the next move plans.
// Missing service (e.g. mock with no scene loader) is non-fatal.
BT::PortsList SetAttachedCollisionCheck::providedPorts()
{ return {BT::InputPort<std::string>("value"),   // "true"|"false"
          BT::InputPort<std::string>("service", "/scene_manager_node/attached_collision_check")}; }
BT::NodeStatus SetAttachedCollisionCheck::tick()
{
  auto* ctx = ctxOf(config());
  const std::string v = in(*this, "value");
  const bool value = (v == "true" || v == "1");
  const std::string srv = in(*this, "service", "/scene_manager_node/attached_collision_check");
  static rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;
  if (!client) client = ctx->node->create_client<std_srvs::srv::SetBool>(srv);
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(nlog(config()), "SetAttachedCollisionCheck: service '%s' unavailable — skipping "
                "(no scene loader?)", srv.c_str());
    return BT::NodeStatus::SUCCESS;   // non-fatal
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
// picks where cycle 1 did. The scene_manager also latches /isaac_scene_reset so the
// Isaac adapter can teleport its prims. No-op (SUCCESS) without a scene loader.
BT::PortsList ResetScene::providedPorts()
{ return {BT::InputPort<std::string>("service", "/scene_manager_node/reset_scene")}; }
BT::NodeStatus ResetScene::tick()
{
  auto* ctx = ctxOf(config());
  const std::string srv = in(*this, "service", "/scene_manager_node/reset_scene");
  static rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  if (!client) client = ctx->node->create_client<std_srvs::srv::Trigger>(srv);
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(nlog(config()), "ResetScene: service '%s' unavailable — skipping "
                "(no scene loader?)", srv.c_str());
    return BT::NodeStatus::SUCCESS;   // non-fatal
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
// A logger that works WITHOUT a BtContext, so pure-blackboard nodes are unit-testable
// with a bare tree (no runtime stack).
rclcpp::Logger safeLog(const BT::NodeConfig& cfg)
{
  try { auto* c = cfg.blackboard->get<BtContext*>(BB_BT_CONTEXT); if (c && c->node) return c->node->get_logger(); }
  catch (const std::exception&) {}
  return rclcpp::get_logger("trainit_bt");
}

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
//   out_key       prefix: writes <out_key>.position "x;y;z", .orientation "qx;qy;qz;qw",
//                 .score, .class_id, .frame, .size "sx;sy;sz"
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
          BT::InputPort<std::string>("out_key", "detected")}; }
BT::NodeStatus DetectObject::tick()
{
  auto* ctx = ctxOf(config());
  const std::string det = in(*this, "detector");
  std::string topic = in(*this, "topic");
  if (topic.empty())
  {
    if (det.empty()) { RCLCPP_ERROR(nlog(config()), "DetectObject: need 'detector' or 'topic'"); return BT::NodeStatus::FAILURE; }
    topic = "/perception/" + det + "/detections";
  }
  const std::string want_class = in(*this, "class_id");
  const std::string target = in(*this, "target_frame", "base_link");
  const std::string out = in(*this, "out_key", "detected");
  int timeout_ms = 3000; double min_score = 0.0;
  try { timeout_ms = std::stoi(in(*this, "timeout_ms", "3000")); } catch (const std::exception&) {}
  try { min_score = std::stod(in(*this, "min_score", "0")); } catch (const std::exception&) {}

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
    return BT::NodeStatus::FAILURE;
  }

  // pick the best matching detection
  const vision_msgs::msg::Detection3D* best = nullptr; double best_score = -1.0;
  for (const auto& d : msg->detections)
  {
    if (d.results.empty()) continue;
    const auto& h = d.results.front();
    if (!want_class.empty() && h.hypothesis.class_id != want_class) continue;
    if (h.hypothesis.score < min_score) continue;
    if (h.hypothesis.score > best_score) { best = &d; best_score = h.hypothesis.score; }
  }
  if (!best)
  {
    RCLCPP_WARN(nlog(config()), "DetectObject: fresh message on '%s' but no detection%s%s (had %zu)",
                topic.c_str(), want_class.empty() ? "" : " of class ", want_class.c_str(), msg->detections.size());
    return BT::NodeStatus::FAILURE;
  }

  // TF: the detector reports in its sensor's OPTICAL frame and knows nothing about robots.
  const std::string src = best->header.frame_id.empty() ? msg->header.frame_id : best->header.frame_id;
  geometry_msgs::msg::PoseStamped in_p, out_p;
  in_p.header.frame_id = src;
  in_p.pose = best->results.front().pose.pose;
  try
  {
    auto tf = tfFor(ctx);
    const auto T = tf->lookupTransform(target, src, tf2::TimePointZero, tf2::durationFromSec(2.0));
    tf2::doTransform(in_p, out_p, T);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(nlog(config()), "DetectObject: TF %s -> %s failed: %s", src.c_str(), target.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  auto bb = config().blackboard;
  const auto& P = out_p.pose;
  bb->set<std::string>(out + ".position", fmtVec({P.position.x, P.position.y, P.position.z}));
  bb->set<std::string>(out + ".orientation", fmtVec({P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w}));
  bb->set<std::string>(out + ".score", fmtVec({best_score}));
  bb->set<std::string>(out + ".class_id", best->results.front().hypothesis.class_id);
  bb->set<std::string>(out + ".frame", target);
  bb->set<std::string>(out + ".size", fmtVec({best->bbox.size.x, best->bbox.size.y, best->bbox.size.z}));
  RCLCPP_INFO(nlog(config()), "DetectObject: '%s' score %.2f at (%.4f, %.4f, %.4f) in %s -> BB[%s.*]",
              best->results.front().hypothesis.class_id.c_str(), best_score,
              P.position.x, P.position.y, P.position.z, target.c_str(), out.c_str());
  return BT::NodeStatus::SUCCESS;
}

// SetWaypointFromDetection: make a data-driven waypoint point at a detection.
//
//   waypoint     the MoveWaypoint name to (re)define
//   from         the DetectObject out_key
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

  std::string src;
  try { src = bb->get<std::string>(from + ".position"); }
  catch (const std::exception&) {
    RCLCPP_ERROR(log, "SetWaypointFromDetection: no '%s.position' on the blackboard (run DetectObject first)", from.c_str());
    return BT::NodeStatus::FAILURE;
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
      bb->set<std::string>(wp + ".orientation", bb->get<std::string>(from + ".orientation"));
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
              wp.c_str(), bb->get<std::string>(wp + ".position").c_str(), from.c_str(), dx, dy, dz,
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
  f.registerNodeType<Wait>("Wait");
  f.registerNodeType<Log>("Log");
}

}  // namespace trainit
