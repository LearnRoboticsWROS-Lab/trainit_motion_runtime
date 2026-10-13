// vNext M2.6 test harness -- TODAY's end-effector behaviour as deterministic text (tests BEFORE).
//
// ADR-0022 ("Order of work and test first") freezes, per end-effector (EEF) family, what the
// LEGACY paths do before the canonical EEF contract touches a production line. Owner brief M2.6
// section 13 names what is recorded: the COMMAND SENT, the ACK, the STATE, the GRASP
// CONFIRMATION, the BT RESULT, the TIMEOUT and the RELEASE RESULT.
//
// A FAMILY file (test/data/m2_6/<family>.txt) is a header (family; the BT.CPP, rmw and MoveIt
// versions this binary was built against) and one block per scenario. Every block ends with ONE
// `semantic:` line whose fields are COMPUTED from what the fakes and recorders observed (never
// typed by hand). Excluded on purpose: stamps, goal ids, wall-clock durations, PIDs, log text
// (except trainit_run_bt's documented 'BT finished:' line). Each run writes its capture to the
// build tree (TRAINIT_M26_ACTUAL_DIR) and compares it byte for byte with the committed one, which
// was recorded on TMR production 1dda503 before M2.6 touched a production line
// (test/data/m2_6/README.md).
//
// Reused from M2.5 (test/m2_5/capture.hpp): sha256, yaml17, num17, Log, Spinner, Watchdog.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_CAPTURE26_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_CAPTURE26_HPP_

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <cxxabi.h>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "m2_5/capture.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"

namespace m26
{
using m25::Log;
using m25::num17;
using m25::Spinner;
using m25::Watchdog;
using m25::yaml17;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

// The production sources every M2.6 capture was frozen on (test/data/m2_6/README.md).
inline constexpr const char* kFrozenOn = "1dda503";

inline std::string environmentLine()
{
  return std::string("behaviortree_cpp ") + TRAINIT_BTCPP_VERSION + " | rmw " +
         rmw_get_implementation_identifier() + " | moveit_ros_planning_interface " +
         TRAINIT_MOVEIT_VERSION;
}

inline std::string demangle(const char* name)
{
  int status = 0;
  std::unique_ptr<char, void (*)(void*)> r{abi::__cxa_demangle(name, nullptr, nullptr, &status),
                                           std::free};
  return status == 0 ? std::string(r.get()) : std::string(name);
}

inline std::string boolText(bool b) { return b ? "true" : "false"; }

inline std::string join(const std::vector<std::string>& v, const std::string& sep = ", ")
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? sep : "") + v[i];
  return s;
}

// --- a family file ---------------------------------------------------------------------------------
class Family
{
public:
  explicit Family(std::string name) : name_(std::move(name))
  {
    text_ = "m2_6 capture " + name_ + "\n" + environmentLine() + "\n\n";
  }

  void add(const std::string& block) { text_ += block; }

  // Writes the capture to the build tree, then compares it with the committed BEFORE capture.
  void check() const
  {
    const std::string actual = std::string(TRAINIT_M26_ACTUAL_DIR) + "/" + name_ + ".txt";
    {
      std::ofstream out(actual, std::ios::binary | std::ios::trunc);
      out << text_;
    }
    const std::string frozen = std::string(TRAINIT_M26_DATA_DIR) + "/" + name_ + ".txt";
    std::ifstream in(frozen, std::ios::binary);
    ASSERT_TRUE(in.is_open()) << "no committed capture " << frozen << " -- this run's capture is "
                              << actual << ". A capture is frozen only from the " << kFrozenOn
                              << " production sources, after the determinism gate "
                                 "(test/data/m2_6/README.md).";
    std::stringstream committed;
    committed << in.rdbuf();
    const std::string want = committed.str();
    const auto header_end = [](const std::string& s) {
      const auto a = s.find('\n');
      return a == std::string::npos ? s.size() : s.find('\n', a + 1);
    };
    ASSERT_EQ(want.substr(0, header_end(want)), text_.substr(0, header_end(text_)))
      << "the environment differs from the one the captures were recorded in (BT.CPP, the rmw or "
         "MoveIt): this is NOT a pass, and NOT a reason to re-capture from the current code. A "
         "capture is re-frozen only from the " << kFrozenOn << " production sources "
         "(test/data/m2_6/README.md). This run's capture: " << actual;
    EXPECT_EQ(want, text_) << "the " << name_ << " family's end-effector behaviour changed; compare "
                           << frozen << " with " << actual << ". A behaviour change is an owner "
                           << "decision, never a re-freeze (test/data/m2_6/README.md; frozen on "
                           << "production " << kFrozenOn << ").";
  }

  const std::string& text() const { return text_; }

private:
  std::string name_;
  std::string text_;
};

// --- the semantic line ---------------------------------------------------------------------------
// ONE per scenario (owner brief M2.6 section 13): command sent | ack | state | grasp confirmation |
// BT result | timeout | release result, plus the action result code when there is one. The caller
// fills each field from what its fakes and recorders observed; an empty field prints "none".
struct Semantic
{
  std::vector<std::string> command, ack, state, grasp, bt, timeout, release, code;

  std::string line() const
  {
    auto f = [](const std::vector<std::string>& v) { return v.empty() ? std::string("none") : join(v, "; "); };
    std::string s = "semantic: command sent=" + f(command) + " | ack=" + f(ack) + " | state=" +
                    f(state) + " | grasp confirmation=" + f(grasp) + " | BT result=" + f(bt) +
                    " | timeout=" + f(timeout) + " | release result=" + f(release);
    if (!code.empty()) s += " | result code=" + join(code, "; ");
    return s + "\n";
  }
};

// --- the graph ---------------------------------------------------------------------------------------
inline std::string fqn(const std::string& ns, const std::string& name)
{
  return (ns == "/" ? "" : ns) + "/" + name;
}

// The topics of type `type` a node subscribes to / publishes, from the graph (sorted). This is where
// a grasp STATE could enter the tree's node, or a command signal leave it.
inline std::vector<std::string> nodeTopics(const rclcpp::Node::SharedPtr& probe, const std::string& node,
                                           const std::string& ns, const std::string& type, bool subscriptions)
{
  std::vector<std::string> out;
  try
  {
    auto graph = probe->get_node_graph_interface();
    const auto m = subscriptions ? graph->get_subscriber_names_and_types_by_node(node, ns)
                                 : graph->get_publisher_names_and_types_by_node(node, ns);
    for (const auto& [topic, types] : m)
      for (const auto& t : types)
        if (t == type) out.push_back(topic);
  }
  catch (const std::exception&) {}   // the node is gone
  std::sort(out.begin(), out.end());
  return out;
}

inline std::string listOrNone(const std::vector<std::string>& v)
{
  return v.empty() ? std::string("none") : "[" + join(v) + "]";
}

// The grasp-confirmation field every family shares: which std_msgs/Bool state topics the node that
// runs the tree subscribes to. "none" means the tree's node could not have read any grasp state.
inline std::string boolStateSources(const rclcpp::Node::SharedPtr& probe, const std::string& node,
                                    const std::string& ns)
{
  const auto subs = nodeTopics(probe, node, ns, "std_msgs/msg/Bool", true);
  return "Bool state subscriptions of " + fqn(ns, node) + ": " + listOrNone(subs);
}

// --- one tree node per process, like trainit_run_bt --------------------------------------------------
// The BtContext node, spun by a 2-thread MultiThreadedExecutor started before anything uses it.
// One per process because RunPolicy's and CheckRobotState's process-static clients and feeds, and
// SetReleasePolicy's static publisher, bind to the FIRST node they see.
class TreeNode
{
public:
  explicit TreeNode(const std::string& name = "trainit_run_bt", const std::string& ns = "/")
  {
    node_ = std::make_shared<rclcpp::Node>(
      name, ns, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~TreeNode()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  TreeNode(const TreeNode&) = delete;
  TreeNode& operator=(const TreeNode&) = delete;
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  rclcpp::Node::SharedPtr node_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
};

// One tick of a tree, the way trainit_run_bt ticks (tickOnce while RUNNING, 10 ms apart), with the
// blackboard trainit_run_bt sets (context, profiles, node). Any std::exception is the result
// "THROW <dynamic type>: <what>".
inline std::string tickTree(const std::string& xml, trainit::BtContext* ctx, const rclcpp::Node::SharedPtr& node)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES, trainit::loadMotionProfiles(""));
  bb->set<rclcpp::Node::SharedPtr>(trainit::BB_ROS_NODE, node);
  try
  {
    factory.registerBehaviorTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + xml +
      "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(milliseconds(10));
    }
    return BT::toStr(status);
  }
  catch (const std::exception& e)
  {
    return "THROW " + demangle(typeid(e).name()) + ": " + e.what();
  }
}

// Polls `pred` every 10 ms until it holds or `budget` runs out; returns whether it held.
inline bool waitUntil(const std::function<bool()>& pred, milliseconds budget)
{
  const auto deadline = Clock::now() + budget;
  while (Clock::now() < deadline)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(milliseconds(10));
  }
  return pred();
}

// --- the isolated process ------------------------------------------------------------------------------
// Per-process fixtures register how to destroy themselves; runAll() runs them (last registered
// first) after the tests and before rclcpp::shutdown().
inline std::vector<std::function<void()>>& teardowns()
{
  static std::vector<std::function<void()>> t;
  return t;
}

// The private domain is set PROGRAMMATICALLY (authoritative even from a shell exporting another
// ROS_DOMAIN_ID), localhost only, and exported for the child processes a family starts; the ament
// ENV is a second guard. Fail closed: after discovery, any node this process did not create is a
// FAILURE -- these tests must never be able to reach a robot.
inline int runAll(int argc, char** argv, std::size_t domain, const std::function<void()>& before_shutdown = {})
{
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  ::setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
  ::testing::InitGoogleTest(&argc, argv);
  std::vector<const char*> args{argv[0]};
  rclcpp::InitOptions options;
  options.set_domain_id(domain);
  rclcpp::init(static_cast<int>(args.size()), args.data(), options);
  {
    auto probe = std::make_shared<rclcpp::Node>("m26_graph_guard", "/",
                                                rclcpp::NodeOptions().use_global_arguments(false));
    // Cross-process discovery took up to 2.54 s on this machine (measured by the M2.6 policy-runtime
    // suite), so a fixed 1.5 s wait can miss a node that is already running: poll for 6 s (two Fast
    // DDS announcement periods) and refuse on the first foreign node.
    std::vector<std::string> foreign;
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (foreign.empty() && std::chrono::steady_clock::now() < until)
    {
      std::this_thread::sleep_for(milliseconds(200));
      for (const auto& n : probe->get_node_names())
        if (n != "/m26_graph_guard" && n.rfind("/_", 0) != 0) foreign.push_back(n);
    }
    if (!foreign.empty())
    {
      std::fprintf(stderr, "M2.6 GRAPH GUARD: foreign nodes on private domain %zu:", domain);
      for (const auto& n : foreign) std::fprintf(stderr, " %s", n.c_str());
      std::fprintf(stderr, " -- refusing to run (no test may reach a robot)\n");
      rclcpp::shutdown();
      return 4;
    }
  }
  const int rc = RUN_ALL_TESTS();
  // A hang in teardown (MoveIt's destructors join their own threads) is a named failure too.
  std::thread([] {
    std::this_thread::sleep_for(std::chrono::seconds(60));
    std::fprintf(stderr, "\nM2.6 WATCHDOG: teardown did not finish in 60 s\n");
    std::fflush(stderr);
    std::_Exit(5);
  }).detach();
  for (auto it = teardowns().rbegin(); it != teardowns().rend(); ++it) (*it)();
  if (before_shutdown) before_shutdown();
  rclcpp::shutdown();
  return rc;
}

// The ONE tree node of a process (see TreeNode), destroyed by runAll() before shutdown.
inline TreeNode& treeNode()
{
  static std::unique_ptr<TreeNode> t;
  if (!t)
  {
    t = std::make_unique<TreeNode>();
    teardowns().push_back([] { t.reset(); });
  }
  return *t;
}

// One watchdog per process: a hang is a NAMED failure, never a silent ctest timeout.
inline Watchdog& watchdog()
{
  static Watchdog dog;
  return dog;
}

}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_CAPTURE26_HPP_
