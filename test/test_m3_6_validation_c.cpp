// test_m3_6_validation_c (vNext M3.6, ADR-0026) -- checkpoint M3-D: the MANIPULATION CONTRACT offline.
//
// The trees and parameters TSA generates for Validation C (TSA test/data/m3_6/validation_c*.yaml;
// byte copies in test/data/m3_6/validation_c/, sha256-pinned here and in TSA) are ticked the way
// trainit_run_bt ticks them -- every node production code: DetectCandidates, SetWaypointFromCandidate,
// MoveWaypoint on a REAL MoveItMotionRuntime against the M2.5 fake move_group, CloseGripper /
// OpenGripper / VerifyGrasp on the REAL end-effector registry against the M2.6 fake adapter -- with
// two SYNTHETIC producers, in ONE process:
//
//   A  a grasp-candidate producer: trainit_interfaces/GraspCandidateArray on
//      /perception/grasp_provider/grasp_candidates (the candidate in frame `cell`)
//   B  a detector: vision_msgs/Detection3DArray on /perception/part_cam/detections (the same object,
//      a fitted surface normal facing the camera, in cam_optical), projected onto the same contract
//
// The cell (the TF the runner would read): base_link; cam_optical at (0.4, 0, 0.6) looking down;
// `cell` = base_link turned 90 deg about Z and moved by (0.2, 0, 0). The object: (0.4, 0, 0.1) in
// base_link on a surface whose normal (-0.6, 0, 0.8) faces the camera, so its approach is (0.6, 0, -0.8).
// The designed goals (base_link), derived by hand in the project file and checked here numerically:
//   PreGrasp   PTP  (0.37, 0, 0.14)   = p - 0.05 a              orientation q* = (3, 0, 1, 0)/sqrt(10)
//   Approach   LIN  (0.4, 0, 0.1)     = p                        q*
//   Retract    LIN  (0.34, 0, 0.18)   = p - 0.1 a                q*
//   PreRelease PTP  (0.3, -0.3, 0.2)  Approach LIN (0.3, -0.3, 0.15)  Retract LIN (0.3, -0.3, 0.23)
// where q* = align_approach(pick_nominal = tool down (1, 0, 0, 0), a): the reference tilted by acos(0.8)
// about -Y (owner decision M3.6-1). The two producers must give the SAME goals.
//
//   VC-A / VC-B   the whole program, SUCCESS: the goals above, PTP / LIN as designed, the profiles'
//                 scalings, base_link, the tcp link; the end-effector commands in the idiom's order
//   VC-C          the grasp is not confirmed (NO_OBJECT): the idiom stops after the Approach -- no
//                 Retract, no release, the vacuum engaged only
//   VC-D          the release is not confirmed: the idiom stops after the opening -- no Retract
//   VC-E          an unusable candidate (a zero approach): nothing moves, nothing is commanded
//   VC-F          no candidate (an empty array): nothing moves
//   VC-G          a candidate whose approach opposes the reference's tool axis: refused, nothing moves
// The family is an AFTER record (new behaviour: no BEFORE): test/data/m3_6/captures/.
#include <rcutils/logging.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <trainit_interfaces/msg/grasp_candidate_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "m2_5/motion_harness.hpp"
#include "m2_6/capture26.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"

namespace
{
using m26::milliseconds;
using Candidates = trainit_interfaces::msg::GraspCandidateArray;
using Candidate = trainit_interfaces::msg::GraspCandidate;
using Detections = vision_msgs::msg::Detection3DArray;
using State = trainit_interfaces::msg::EndEffectorState;

const std::string kData = TRAINIT_M36_DATA_DIR;
const std::string kFixture = kData + "/validation_c";
// the byte copies of TSA's generation (the same digests are pinned in TSA's test_m3_6_validation_c.py)
const char* kCandidatesTreeSha = "0c954d63dc62f227fc7850d8d65ad3a3c482b18f254b553effb2cca419b4a757";
const char* kDetectionsTreeSha = "bfa1d674d5025801c040c728767135480ae002450c0ec71d444100f83a715a9d";
const char* kBtParamsSha = "e90956740ce6fff21ba79c81c47a89b3ace8e0817b9c0ba71a70bc8134b2ed9e";
const char* kProfilesSha = "507b77a7feca6be0dd0b25a512c20ac480e9445b0a8b04d0e01c4db30b5bc587";

// --- the log lines of the run's steps, through rcutils' output handler (the M2.7 cell's tap) -------
std::mutex g_tap_m;
std::vector<std::string> g_lines;
rcutils_logging_output_handler_t g_previous = nullptr;

void tap(const rcutils_log_location_t* location, int severity, const char* name, rcutils_time_point_value_t ts,
         const char* format, va_list* args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[4096];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  {
    const std::string line(buf);
    // the step a line reports, as "<node> <OK|FAIL>": the timeline of the idiom
    struct Head { const char* prefix; const char* node; };
    static const Head heads[] = {{"DetectCandidates: ", "DetectCandidates"}, {"DetectCandidates '", "DetectCandidates"},
                                 {"SetWaypointFromCandidate: ", "SetWaypointFromCandidate"},
                                 {"SetWaypointFromCandidate '", "SetWaypointFromCandidate"},
                                 {"MoveWaypoint '", "MoveWaypoint"}, {"CloseGripper ", "CloseGripper"},
                                 {"OpenGripper ", "OpenGripper"}, {"VerifyGrasp ", "VerifyGrasp"}};
    std::lock_guard<std::mutex> g(g_tap_m);
    for (const auto& h : heads)
      if (line.rfind(h.prefix, 0) == 0)
      {
        if (std::strcmp(h.node, "DetectCandidates") == 0 && line.find("candidate(s) on BB") == std::string::npos &&
            severity < RCUTILS_LOG_SEVERITY_WARN)
          break;   // one line per detection: its summary
        std::string what = h.node;
        if (std::strcmp(h.node, "MoveWaypoint") == 0 && severity < RCUTILS_LOG_SEVERITY_WARN)
        {
          const auto a = line.find('\''), b = line.find('\'', a + 1);
          what += " " + line.substr(a + 1, b - a - 1).substr(line.substr(a + 1, b - a - 1).find("__") + 2);
        }
        // a refusal or a fault is an ERROR, a verdict (NO_OBJECT) a WARN: both end the step
        g_lines.push_back(what + (severity >= RCUTILS_LOG_SEVERITY_WARN ? " FAIL" : " OK"));
        break;
      }
  }
  if (g_previous) g_previous(location, severity, name, ts, format, args);
}

std::vector<std::string> timeline()
{
  std::lock_guard<std::mutex> g(g_tap_m);
  return g_lines;
}

// --- the synthetic producers ------------------------------------------------------------------------
Candidate referenceCandidate(const std::string& id)
{
  Candidate c;   // the reference object in `cell`: (0, -0.2, 0.1), approach (0, -0.6, -0.8)
  c.header.frame_id = "cell";
  c.id = id;
  c.provider = "synthetic";
  c.grasp_pose.position.y = -0.2;
  c.grasp_pose.position.z = 0.1;
  c.grasp_pose.orientation.w = 1.0;
  c.approach.y = -0.6;
  c.approach.z = -0.8;
  c.standoff_m = 0.03;   // the step's 0.05 replaces it
  c.score = 0.9;
  c.confidence = 0.9;
  c.object_class = "part";
  return c;
}

Candidates candidateArray()
{
  Candidates a;
  a.header.frame_id = "cell";
  a.candidates.push_back(referenceCandidate("best"));
  Candidate second = referenceCandidate("second");   // never used: index 0 is the best
  second.grasp_pose.position.z = 0.3;
  a.candidates.push_back(second);
  return a;
}

Detections detectionArray()
{
  Detections a;
  a.header.frame_id = "cam_optical";
  vision_msgs::msg::Detection3D d;
  d.header.frame_id = "cam_optical";
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = "part";
  h.hypothesis.score = 0.9;
  h.pose.pose.position.z = 0.5;   // (0.4, 0, 0.1) in base_link
  // the perception contract: the orientation takes +Z onto the fitted normal, which faces the camera:
  // (0, 0.6, -0.8) in cam_optical, (-0.6, 0, 0.8) in base_link
  const Eigen::Quaterniond q =
    Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.0, 0.6, -0.8));
  h.pose.pose.orientation.x = q.x();
  h.pose.pose.orientation.y = q.y();
  h.pose.pose.orientation.z = q.z();
  h.pose.pose.orientation.w = q.w();
  d.results.push_back(h);
  a.detections.push_back(d);
  return a;
}

class Producers
{
public:
  Producers()
  {
    node_ = std::make_shared<rclcpp::Node>("m36_synthetic_producers", "/",
                                           rclcpp::NodeOptions().use_global_arguments(false));
    cpub_ = node_->create_publisher<Candidates>("/perception/grasp_provider/grasp_candidates",
                                                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    dpub_ = node_->create_publisher<Detections>("/perception/part_cam/detections",
                                                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    timer_ = node_->create_wall_timer(milliseconds(20), [this] {
      std::lock_guard<std::mutex> g(m_);
      if (cands_) cpub_->publish(*cands_);
      if (dets_) dpub_->publish(*dets_);
    });
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~Producers()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  Producers(const Producers&) = delete;
  Producers& operator=(const Producers&) = delete;
  void set(std::optional<Candidates> c, std::optional<Detections> d)
  {
    std::lock_guard<std::mutex> g(m_);
    cands_ = std::move(c);
    dets_ = std::move(d);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Candidates>::SharedPtr cpub_;
  rclcpp::Publisher<Detections>::SharedPtr dpub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex m_;
  std::optional<Candidates> cands_;
  std::optional<Detections> dets_;
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

// task_parameters.* (ROS params) -> blackboard STRINGS: trainit_run_bt's loadTaskParameters /
// putBlackboard (src/trainit_run_bt_node.cpp), copied (they are file-local there)
void putBlackboard(const BT::Blackboard::Ptr& bb, const std::string& key, const rclcpp::ParameterValue& p)
{
  using PT = rclcpp::ParameterType;
  switch (p.get_type())
  {
    case PT::PARAMETER_BOOL: bb->set<std::string>(key, p.get<bool>() ? "true" : "false"); break;
    case PT::PARAMETER_INTEGER: bb->set<std::string>(key, std::to_string(p.get<int64_t>())); break;
    case PT::PARAMETER_DOUBLE: bb->set<std::string>(key, std::to_string(p.get<double>())); break;
    case PT::PARAMETER_STRING: bb->set<std::string>(key, p.get<std::string>()); break;
    case PT::PARAMETER_DOUBLE_ARRAY:
    {
      std::ostringstream os;
      const auto v = p.get<std::vector<double>>();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str());
      break;
    }
    case PT::PARAMETER_INTEGER_ARRAY:
    {
      std::ostringstream os;
      const auto v = p.get<std::vector<int64_t>>();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str());
      break;
    }
    default: break;
  }
}

// --- one scenario ------------------------------------------------------------------------------------
struct Scenario
{
  std::string id;
  std::string pins;
  std::string tree;                               // candidates | detections
  std::function<void(Producers&)> produce;
  std::optional<m26::after::SiteScript> site{};   // the fake adapter's grasp decisions
};

struct Outcome
{
  std::string bt;
  std::vector<moveit_msgs::msg::MotionPlanRequest> goals;
  std::vector<std::string> vacuum;
  std::vector<std::string> steps;
  std::string text;   // the capture block
};

std::string num9(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9f", std::abs(v) < 5e-10 ? 0.0 : v);
  return buf;
}

std::string goalText(const moveit_msgs::msg::MotionPlanRequest& r)
{
  std::ostringstream os;
  os << r.pipeline_id << "/" << r.planner_id << " vel=" << num9(r.max_velocity_scaling_factor)
     << " acc=" << num9(r.max_acceleration_scaling_factor);
  if (r.goal_constraints.empty() || r.goal_constraints[0].position_constraints.empty() ||
      r.goal_constraints[0].orientation_constraints.empty())
    return os.str() + " (not a pose goal)";
  const auto& pc = r.goal_constraints[0].position_constraints[0];
  const auto& oc = r.goal_constraints[0].orientation_constraints[0];
  const auto& p = pc.constraint_region.primitive_poses[0].position;
  const auto& q = oc.orientation;
  os << " " << pc.link_name << "@" << pc.header.frame_id << " p=(" << num9(p.x) << ", " << num9(p.y) << ", "
     << num9(p.z) << ") q=(" << num9(q.x) << ", " << num9(q.y) << ", " << num9(q.z) << ", " << num9(q.w) << ")";
  return os.str();
}

class Cell
{
public:
  Cell()
  : mg_(motion_log_, "/"),
    js_("/"),
    params_(m26::after::loadParams(kFixture + "/bt_params.yaml")),
    contract_(trainit::eef::parseContract(params_))
  {
    node_ = std::make_shared<rclcpp::Node>(
      "trainit_run_bt", "/",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                           .parameter_overrides(m25::armParameters()));
    profiles_ = trainit::loadMotionProfiles(kFixture + "/motion_profiles.yaml");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = "cam_optical";
    T.transform.translation.x = 0.4;
    T.transform.translation.z = 0.6;
    T.transform.rotation.x = std::sqrt(0.5);   // 180 deg about (1, -1, 0): x -> -y, y -> -x, z -> -z
    T.transform.rotation.y = -std::sqrt(0.5);
    T.transform.rotation.z = 0.0;
    T.transform.rotation.w = 0.0;
    tf_->setTransform(T, "m36", true);
    T.child_frame_id = "cell";
    T.transform.translation.x = 0.2;
    T.transform.translation.z = 0.0;
    T.transform.rotation.x = 0.0;
    T.transform.rotation.y = 0.0;
    T.transform.rotation.z = std::sqrt(0.5);   // 90 deg about Z
    T.transform.rotation.w = std::sqrt(0.5);
    tf_->setTransform(T, "m36", true);
    tf_->setUsingDedicatedThread(true);
    rcutils_logging_output_handler_t current = rcutils_logging_get_output_handler();
    if (current != &tap)
    {
      g_previous = current;
      rcutils_logging_set_output_handler(&tap);
    }
  }
  ~Cell()
  {
    rcutils_logging_set_output_handler(g_previous);
    adapter_.reset();
    stopSpinning();
    runtime_.reset();
    registry_.reset();
  }
  Cell(const Cell&) = delete;
  Cell& operator=(const Cell&) = delete;

  Outcome run(const Scenario& s)
  {
    Outcome o;
    m26::watchdog().arm("build " + s.id, std::chrono::seconds(90));
    const std::string problem = begin(s);
    m26::watchdog().disarm();
    if (!problem.empty())
    {
      ADD_FAILURE() << s.id << ": the cell could not be set up: " << problem;
      return o;
    }
    motion_log_.reset();
    mg_.resetScenario();
    {
      std::lock_guard<std::mutex> g(g_tap_m);
      g_lines.clear();
    }
    m26::watchdog().arm(s.id, std::chrono::seconds(240));
    o.bt = tick(m26::after::readFile(kFixture + "/" + s.tree + ".xml"));
    m25::settleTopics(motion_log_);
    m26::watchdog().disarm();
    o.goals = mg_.moveGoals();
    for (const auto& c : adapter_->setBool("vacuum").calls()) o.vacuum.push_back(c.data ? "engage" : "release");
    o.steps = timeline();
    std::ostringstream os;
    os << "=== scenario " << s.id << "\n" << "pins: " << s.pins << "\n" << "tree: " << s.tree << ".xml\n"
       << "steps: " << m26::listOrNone(o.steps) << "\n" << "move_group goals:\n";
    if (o.goals.empty()) os << "  (none)\n";
    for (std::size_t i = 0; i < o.goals.size(); ++i) os << "  " << i + 1 << ". " << goalText(o.goals[i]) << "\n";
    os << "vacuum: " << m26::listOrNone(o.vacuum) << "\n" << "bt: " << o.bt << "\n\n";
    o.text = os.str();
    return o;
  }

private:
  std::string begin(const Scenario& s)
  {
    stopSpinning();
    runtime_.reset();
    registry_.reset();
    adapter_.reset();
    if (!m26::waitUntil([&] { return node_->count_publishers("/eef/main/state") == 0; }, milliseconds(6000)))
      return "the previous adapter's state publisher is still in the graph";
    mg_.start(m25::FakeMoveGroup::ALL);
    mg_.resetScenario();
    producers_.set(std::nullopt, std::nullopt);
    if (s.produce) s.produce(producers_);
    registry_ = std::make_unique<trainit::EndEffectorRegistry>(node_, contract_);
    startSpinning();
    runtime_ = std::make_unique<trainit::MoveItMotionRuntime>(node_, m25::defaultConfig());
    std::this_thread::sleep_for(milliseconds(300));
    adapter_ = std::make_unique<m26::after::FakeEefAdapter>(eef_log_, "/", *contract_.device("main"),
                                                           m26::after::AdapterOptions{});
    if (s.site) adapter_->setSite("main", *s.site);
    if (!m26::waitUntil([&] { return registry_->device("main")->latest().sample != nullptr; }, milliseconds(6000)))
      return "no state sample of eef/main reached the registry";
    if (!m26::waitUntil([&] {
          return node_->get_service_names_and_types().count("/eef/main/actuators/vacuum/set_engaged") == 1;
        }, milliseconds(6000)))
      return "the adapter's SetBool server is not in the graph";
    ctx_ = m25::makeContext(runtime_.get(), node_);
    ctx_.tf = tf_;
    ctx_.eefs = registry_.get();
    return "";
  }

  // as trainit_run_bt: the blackboard (context, profiles, node, task parameters), tickOnce while RUNNING
  std::string tick(const std::string& document)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    auto bb = BT::Blackboard::create();
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES, profiles_);
    bb->set<rclcpp::Node::SharedPtr>(trainit::BB_ROS_NODE, node_);
    const std::string prefix = "task_parameters.";
    for (const auto& [name, value] : params_)
      if (name.rfind(prefix, 0) == 0) putBlackboard(bb, name.substr(prefix.size()), value);
    BT::Tree tree;
    try
    {
      factory.registerBehaviorTreeFromText(document);
      tree = factory.createTree("MainTree", bb);
    }
    catch (const std::exception& e)
    {
      return std::string("LOAD FAILED: ") + e.what();
    }
    const auto problems = registry_->checkTree(tree);
    if (!problems.empty()) return "REFUSED AT START-UP: " + problems.front();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    try
    {
      while (status == BT::NodeStatus::RUNNING)
      {
        status = tree.tickOnce();
        if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(milliseconds(10));
      }
    }
    catch (const std::exception& e)
    {
      return std::string("THROW ") + e.what();
    }
    return BT::toStr(status);
  }

  void startSpinning()
  {
    spin_ = std::make_unique<m26::Spinner<rclcpp::executors::MultiThreadedExecutor>>(
      rclcpp::ExecutorOptions(), 2 + contract_.devices.size());
    spin_->executor().add_node(node_);
    spin_->start();
  }
  void stopSpinning()
  {
    if (!spin_) return;
    spin_->stop();
    spin_->executor().remove_node(node_);
    spin_.reset();
  }

  m25::Log motion_log_;
  m25::FakeMoveGroup mg_;
  m25::FakeJointStates js_;
  m26::Log eef_log_;
  trainit::eef::ParameterMap params_;
  trainit::eef::Contract contract_;
  rclcpp::Node::SharedPtr node_;
  trainit::MotionProfileRegistry profiles_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Producers producers_;
  std::unique_ptr<trainit::EndEffectorRegistry> registry_;
  std::unique_ptr<m26::Spinner<rclcpp::executors::MultiThreadedExecutor>> spin_;
  std::unique_ptr<trainit::MoveItMotionRuntime> runtime_;
  std::unique_ptr<m26::after::FakeEefAdapter> adapter_;
  trainit::BtContext ctx_{};
};

Cell& cell()
{
  static std::unique_ptr<Cell> c;   // test harness: one per process
  if (!c)
  {
    c = std::make_unique<Cell>();
    m26::teardowns().push_back([] { c.reset(); });
  }
  return *c;
}

m25::Family& family()
{
  static m25::Family f("m3_6_validation_c", kData + "/captures");   // test harness: one per process
  return f;
}

// --- the designed values ----------------------------------------------------------------------------
struct Goal
{
  const char* planner;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  double vel;
};

const Eigen::Quaterniond kAligned(0.0, 3.0 / std::sqrt(10.0), 0.0, 1.0 / std::sqrt(10.0));   // (w, x, y, z)
const Eigen::Quaterniond kDown(0.0, 1.0, 0.0, 0.0);

std::vector<Goal> designed()
{
  const Eigen::Vector3d p(0.4, 0.0, 0.1), a(0.6, 0.0, -0.8);
  return {{"PTP", p - 0.05 * a, kAligned, 0.5}, {"LIN", p, kAligned, 0.2}, {"LIN", p - 0.1 * a, kAligned, 0.2},
          {"PTP", {0.3, -0.3, 0.2}, kDown, 0.5}, {"LIN", {0.3, -0.3, 0.15}, kDown, 0.2},
          {"LIN", {0.3, -0.3, 0.23}, kDown, 0.2}};
}

void expectGoals(const Outcome& o, std::size_t count, const std::string& id)
{
  const auto want = designed();
  ASSERT_EQ(o.goals.size(), count) << id << "\n" << o.text;
  for (std::size_t i = 0; i < count; ++i)
  {
    const auto& r = o.goals[i];
    const auto& w = want[i];
    EXPECT_EQ(r.pipeline_id, "pilz_industrial_motion_planner") << id << " goal " << i + 1;
    EXPECT_EQ(r.planner_id, w.planner) << id << " goal " << i + 1;
    EXPECT_NEAR(r.max_velocity_scaling_factor, w.vel, 1e-12) << id << " goal " << i + 1;
    EXPECT_NEAR(r.max_acceleration_scaling_factor, w.vel, 1e-12) << id << " goal " << i + 1;
    ASSERT_EQ(r.goal_constraints.size(), 1u) << id << " goal " << i + 1;
    const auto& pc = r.goal_constraints[0].position_constraints.at(0);
    const auto& oc = r.goal_constraints[0].orientation_constraints.at(0);
    EXPECT_EQ(pc.link_name, "tcp") << id << " goal " << i + 1;
    EXPECT_EQ(pc.header.frame_id, "base_link") << id << " goal " << i + 1;
    const auto& p = pc.constraint_region.primitive_poses.at(0).position;
    EXPECT_NEAR((Eigen::Vector3d(p.x, p.y, p.z) - w.p).norm(), 0.0, 1e-9) << id << " goal " << i + 1;
    const Eigen::Quaterniond q(oc.orientation.w, oc.orientation.x, oc.orientation.y, oc.orientation.z);
    EXPECT_NEAR(q.angularDistance(w.q), 0.0, 1e-6) << id << " goal " << i + 1;
  }
}

Outcome runScenario(const Scenario& s)
{
  Outcome o = cell().run(s);
  family().add(o.text);
  return o;
}

std::optional<Outcome> g_candidates, g_detections;

const std::vector<std::string> kFullTimeline = {
  "DetectCandidates OK", "SetWaypointFromCandidate OK", "MoveWaypoint s2_pregrasp OK", "MoveWaypoint s2_grasp OK",
  "CloseGripper OK", "VerifyGrasp OK", "MoveWaypoint s2_retract OK", "MoveWaypoint s3_prerelease OK",
  "MoveWaypoint s3_release OK", "VerifyGrasp OK", "OpenGripper OK", "VerifyGrasp OK", "MoveWaypoint s3_retract OK"};
}  // namespace

TEST(M36ValidationC, the_fixtures_are_the_pinned_byte_copies_of_what_TSA_generates)
{
  EXPECT_EQ(m25::sha256(m26::after::readFile(kFixture + "/candidates.xml")), kCandidatesTreeSha);
  EXPECT_EQ(m25::sha256(m26::after::readFile(kFixture + "/detections.xml")), kDetectionsTreeSha);
  EXPECT_EQ(m25::sha256(m26::after::readFile(kFixture + "/bt_params.yaml")), kBtParamsSha);
  EXPECT_EQ(m25::sha256(m26::after::readFile(kFixture + "/motion_profiles.yaml")), kProfilesSha);
}

TEST(M36ValidationC, VC_A_a_grasp_candidate_producer_runs_the_designed_goals)
{
  g_candidates = runScenario({"VC-A", "producer A (GraspCandidateArray, frame cell, best first): SUCCESS; PreGrasp, "
                              "Approach, Retract and the release goals as designed", "candidates",
                              [](Producers& p) { p.set(candidateArray(), std::nullopt); }});
  EXPECT_EQ(g_candidates->bt, "SUCCESS") << g_candidates->text;
  expectGoals(*g_candidates, 6, "VC-A");
  EXPECT_EQ(g_candidates->vacuum, (std::vector<std::string>{"engage", "release"}));
  EXPECT_EQ(g_candidates->steps, kFullTimeline) << g_candidates->text;
}

TEST(M36ValidationC, VC_B_a_detector_projected_onto_the_contract_runs_the_same_goals)
{
  g_detections = runScenario({"VC-B", "producer B (Detection3DArray with a fitted normal, cam_optical), projected: "
                              "SUCCESS; the SAME goals as VC-A", "detections",
                              [](Producers& p) { p.set(std::nullopt, detectionArray()); }});
  EXPECT_EQ(g_detections->bt, "SUCCESS") << g_detections->text;
  expectGoals(*g_detections, 6, "VC-B");
  EXPECT_EQ(g_detections->vacuum, (std::vector<std::string>{"engage", "release"}));
  EXPECT_EQ(g_detections->steps, kFullTimeline) << g_detections->text;
}

TEST(M36ValidationC, the_two_producers_converge_onto_the_same_manipulation_contract)
{
  ASSERT_TRUE(g_candidates && g_detections);
  ASSERT_EQ(g_candidates->goals.size(), g_detections->goals.size());
  for (std::size_t i = 0; i < g_candidates->goals.size(); ++i)
    EXPECT_EQ(goalText(g_candidates->goals[i]), goalText(g_detections->goals[i])) << "goal " << i + 1;
  EXPECT_EQ(g_candidates->steps, g_detections->steps);
  EXPECT_EQ(g_candidates->vacuum, g_detections->vacuum);
}

TEST(M36ValidationC, VC_C_an_unconfirmed_grasp_stops_the_idiom_before_the_retract)
{
  m26::after::SiteScript no_object;
  no_object.on_engage = State::GRASP_NOT_HELD;
  const Outcome o = runScenario({"VC-C", "the grasp is NOT confirmed (NO_OBJECT): FAILURE after the Approach -- no "
                                 "Retract, no release", "candidates",
                                 [](Producers& p) { p.set(candidateArray(), std::nullopt); }, no_object});
  EXPECT_EQ(o.bt, "FAILURE");
  expectGoals(o, 2, "VC-C");   // PreGrasp, Approach -- and nothing after the failed verification
  EXPECT_EQ(o.vacuum, (std::vector<std::string>{"engage"}));
  ASSERT_FALSE(o.steps.empty());
  EXPECT_EQ(o.steps.back(), "VerifyGrasp FAIL") << o.text;
}

TEST(M36ValidationC, VC_D_an_unconfirmed_release_stops_the_idiom_before_the_retract)
{
  m26::after::SiteScript still_held;
  still_held.on_release = State::GRASP_HELD;
  const Outcome o = runScenario({"VC-D", "the release is NOT confirmed (still held): FAILURE after the opening -- no "
                                 "Retract", "candidates",
                                 [](Producers& p) { p.set(candidateArray(), std::nullopt); }, still_held});
  EXPECT_EQ(o.bt, "FAILURE");
  expectGoals(o, 5, "VC-D");
  EXPECT_EQ(o.vacuum, (std::vector<std::string>{"engage", "release"}));
  ASSERT_FALSE(o.steps.empty());
  EXPECT_EQ(o.steps.back(), "VerifyGrasp FAIL") << o.text;
}

TEST(M36ValidationC, VC_E_F_G_invalid_or_no_candidate_data_never_moves_the_robot)
{
  Candidates zero = candidateArray();
  zero.candidates[0].approach.y = 0.0;
  zero.candidates[0].approach.z = 0.0;
  Candidates opposed = candidateArray();
  opposed.candidates[0].approach.y = 0.0;
  opposed.candidates[0].approach.z = 1.0;   // up: exactly against pick_nominal's tool +Z (down)
  struct Case { const char* id; const char* pins; Candidates array; const char* last; };
  const Case cases[] = {
    {"VC-E", "an unusable candidate (a zero approach): the array is refused, nothing moves", zero, "DetectCandidates FAIL"},
    {"VC-F", "no candidate (an empty array): FAILURE, nothing moves", Candidates{}, "DetectCandidates FAIL"},
    {"VC-G", "an approach exactly against the reference's tool axis: align_approach refuses, nothing moves",
     opposed, "SetWaypointFromCandidate FAIL"},
  };
  for (const auto& c : cases)
  {
    const Candidates array = c.array;
    const Outcome o = runScenario({c.id, c.pins, "candidates", [array](Producers& p) { p.set(array, std::nullopt); }});
    EXPECT_EQ(o.bt, "FAILURE") << c.id;
    EXPECT_TRUE(o.goals.empty()) << c.id << "\n" << o.text;
    EXPECT_TRUE(o.vacuum.empty()) << c.id;
    ASSERT_FALSE(o.steps.empty()) << c.id;
    EXPECT_EQ(o.steps.back(), c.last) << c.id << "\n" << o.text;
  }
}

TEST(M36ValidationC, zz_the_family_is_the_committed_record)
{
  family().check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
