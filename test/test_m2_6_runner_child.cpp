// vNext M2.6 (tests BEFORE, ADR-0022 family E7): eef_runner_child -- the REAL trainit_run_bt,
// started as a child process the way a launch starts it (its own process, a params file, a
// `-r __ns:=` remap for a namespaced cell), running a tiny tree (test/data/m2_6/eef_tree*.xml).
// Frozen on production 1dda503 before the canonical EEF contract touches a production line
// (test/data/m2_6/README.md).
//
// trainit_run_bt constructs a MoveItMotionRuntime first. Its MoveGroupInterface waits for
// move_action + execute_trajectory and its PlanningSceneInterface for get/apply_planning_scene,
// all in the child's namespace (until the vNext M2.7 namespace fix, ADR-0023 Decision 3, the two
// actions were the ROOT ones: M2.5 NS-03). The parent serves them with the M2.5 fakes
// (test/m2_5/fake_move_group.hpp), unchanged: the full set at the root, and the two actions and
// the scene services in every child namespace. The end-effector servers are
// ScriptedGrippers; the grasp Bool is recorded with attribution. Captured: what the fakes and
// recorders saw, the child's exit code, its 'BT finished:' line (the ONLY log line read), and the
// Bool topics the child's node published and subscribed (polled from the graph while it ran).
// Harness keys every params file carries: the inline M2.5 test arm and groot_monitor false (no TCP
// port is opened; two children would otherwise collide on 1667).
#include <sys/stat.h>

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"
#include "m2_6/eef_fakes.hpp"

using m26::BoolRecorder;
using m26::GripperReply;

namespace
{
struct Child
{
  std::string ns;          // "/" or "/r1"
  std::string eef_keys;    // the scenario's end-effector keys, YAML lines verbatim ("key: value" per line)
  std::string tree;        // eef_tree.xml | eef_tree_hold.xml
};

struct RcScenario
{
  std::string id;
  std::string pins;
  std::vector<Child> children;
  std::vector<std::pair<std::string, std::vector<GripperReply>>> servers;   // absolute action names
  std::string bool_topic;      // a VOLATILE recorder made before the children ("" = none)
  bool late_joiner{false};     // a LATCHED recorder made once every child has closed
};

std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/runner_child"; }

std::string paramsFile(const RcScenario& s, const Child& c)
{
  const std::string path = actualDir() + "/" + m26::tagOf(s.id) + m26::tagOf(c.ns == "/" ? "_root" : c.ns) + ".yaml";
  std::ostringstream y;
  y << "# vNext M2.6 " << s.id << ": trainit_run_bt params, written by test_m2_6_runner_child\n"
    << "/**:\n  ros__parameters:\n"
    << "    # --- harness keys: the tree, the inline M2.5 test arm, no Groot2 TCP server ---\n"
    << "    bt_tree_file: \"" << TRAINIT_M26_DATA_DIR << "/" << c.tree << "\"\n"
    << "    bt_tree_id: \"MainTree\"\n"
    << "    planner_mode: \"ompl\"\n"
    << "    process_path_backend: \"pilz_sequence\"\n"
    << "    group: \"arm\"\n"
    << "    base_frame: \"base_link\"\n"
    << "    tip_link: \"tcp\"\n"
    << "    process_controller: \"mock\"\n"
    << "    enforce_validation: true\n"
    << "    groot_monitor: false\n"
    << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
    << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6)
    << "    # --- the scenario's end-effector keys ---\n"
    << m26::indentBlock(c.eef_keys, 4);
  m26::writeFile(path, y.str());
  return path;
}

std::string childName(const Child& c) { return m26::fqn(c.ns, "trainit_run_bt"); }

std::string run(const RcScenario& s)
{
  ::mkdir(actualDir().c_str(), 0755);
  auto probe = std::make_shared<rclcpp::Node>("m26_e7_probe", "/", rclcpp::NodeOptions().use_global_arguments(false));
  // the M2.5 fakes MoveItMotionRuntime's constructor waits for: the root set, and in every child
  // namespace the move_group actions and the scene services. A /r1 or /r3 child's
  // MoveGroupInterface resolves move_action and execute_trajectory in its own namespace since the
  // vNext M2.7 namespace fix (ADR-0023 Decision 3); before the fix it used the root ones. The
  // trees never plan, so no goal reaches either set, and the capture is byte-identical on both
  // sides of the fix (a goal would land in its namespace's block, which keeps its frozen label).
  m26::Log mg_log;
  mg_log.setBodies(false);
  auto root = std::make_unique<m25::FakeMoveGroup>(mg_log, "/");
  std::map<std::string, std::unique_ptr<m26::Log>> ns_logs;
  std::vector<std::unique_ptr<m25::FakeMoveGroup>> ns_fakes;
  for (const auto& c : s.children)
    if (c.ns != "/" && !ns_logs.count(c.ns))
    {
      ns_logs[c.ns] = std::make_unique<m26::Log>();
      ns_logs[c.ns]->setBodies(false);
      ns_fakes.push_back(std::make_unique<m25::FakeMoveGroup>(
        *ns_logs[c.ns], c.ns,
        m25::FakeMoveGroup::MOVE_ACTION | m25::FakeMoveGroup::EXECUTE | m25::FakeMoveGroup::SCENE_SERVICES));
    }
  std::vector<std::unique_ptr<m26::Log>> eef_logs;
  std::vector<std::unique_ptr<m26::ScriptedGripper>> servers;
  for (const auto& [action, script] : s.servers)
  {
    eef_logs.push_back(std::make_unique<m26::Log>());
    servers.push_back(std::make_unique<m26::ScriptedGripper>(*eef_logs.back(), "/", action, script));
  }
  std::unique_ptr<BoolRecorder> rec;
  if (!s.bool_topic.empty())
    rec = std::make_unique<BoolRecorder>("m26_rec_" + m26::tagOf(s.id), s.bool_topic, BoolRecorder::Qos::VOLATILE);
  std::this_thread::sleep_for(m26::milliseconds(500));

  std::ostringstream os;
  os << "=== scenario " << s.id << "\n" << "pins: " << s.pins << "\n";
  for (const auto& c : s.children)
  {
    os << "child " << childName(c) << ": trainit_run_bt --ros-args --params-file <" << s.id << " params>"
       << (c.ns == "/" ? std::string() : " -r __ns:=" + c.ns) << " (tree test/data/m2_6/" << c.tree << ")\n"
       << "  end-effector keys:" << (c.eef_keys.empty() ? std::string(" (none)\n") : "\n" + m26::indentBlock(c.eef_keys, 4))
       << "  harness keys: bt_tree_file bt_tree_id group=arm base_frame=base_link tip_link=tcp planner_mode=ompl "
          "process_controller=mock enforce_validation=true groot_monitor=false robot_description(_semantic)=the inline M2.5 test arm\n";
  }
  for (const auto& sv : servers) os << "server " << sv->name() << ": script " << sv->scriptText() << "\n";
  if (rec) os << "recorder: " << s.bool_topic << " (" << rec->qosText() << "), made before the children\n";

  // --- the children, concurrently --------------------------------------------------------------------
  m26::watchdog().arm(s.id, std::chrono::seconds(120));
  std::vector<std::unique_ptr<m26::ChildProcess>> kids;
  for (const auto& c : s.children)
  {
    std::vector<std::string> args{"--ros-args", "--params-file", paramsFile(s, c)};
    if (c.ns != "/") { args.push_back("-r"); args.push_back("__ns:=" + c.ns); }
    const std::string out = actualDir() + "/" + m26::tagOf(s.id) + m26::tagOf(c.ns == "/" ? "_root" : c.ns) + ".log";
    kids.push_back(std::make_unique<m26::ChildProcess>(TRAINIT_RUN_BT_EXE, args,
                                                       m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), out));
  }
  // poll the graph while they run: the Bool topics each child's node publishes / subscribes
  std::map<std::string, std::set<std::string>> bool_pubs, bool_subs;
  std::unique_ptr<BoolRecorder> late;
  std::vector<m26::Clock::time_point> exited(kids.size());
  for (;;)
  {
    bool any = false;
    for (std::size_t i = 0; i < kids.size(); ++i)
    {
      const Child& c = s.children[i];
      for (const auto& t : m26::nodeTopics(probe, "trainit_run_bt", c.ns, "std_msgs/msg/Bool", false)) bool_pubs[childName(c)].insert(t);
      for (const auto& t : m26::nodeTopics(probe, "trainit_run_bt", c.ns, "std_msgs/msg/Bool", true)) bool_subs[childName(c)].insert(t);
      if (kids[i]->running()) any = true;
      else if (exited[i] == m26::Clock::time_point{}) exited[i] = m26::Clock::now();
    }
    if (s.late_joiner && !late && rec && rec->count() >= kids.size())
    {
      late = std::make_unique<BoolRecorder>("m26_rec_" + m26::tagOf(s.id) + "_late", s.bool_topic, BoolRecorder::Qos::LATCHED);
      late->settle(kids.size(), m26::milliseconds(3000));
    }
    if (!any) break;
    std::this_thread::sleep_for(m26::milliseconds(100));
  }
  if (rec) rec->settle(0, m26::milliseconds(1000));
  for (auto& sv : servers) sv->joinWorkers();
  m26::watchdog().disarm();

  // --- the block -----------------------------------------------------------------------------------------
  for (std::size_t i = 0; i < servers.size(); ++i)
    os << "events " << servers[i]->name() << ":\n" << (eef_logs[i]->events().empty() ? std::string("(none)\n") : eef_logs[i]->events());
  os << "events of the root move_group fakes (summaries):\n" << (mg_log.events().empty() ? std::string("(none)\n") : mg_log.events());
  for (const auto& [ns, log] : ns_logs)
    os << "events of the " << ns << " scene fakes (summaries):\n" << (log->events().empty() ? std::string("(none)\n") : log->events());
  if (rec) os << "recorder received: " << rec->byPublisher() << "\n";
  if (late) os << "late joiner (" << late->qosText() << ", made once every child had closed) received: " << late->multiset() << "\n";
  m26::Semantic sem;
  for (std::size_t i = 0; i < kids.size(); ++i)
  {
    const Child& c = s.children[i];
    const std::string who = childName(c);
    const auto pubs = std::vector<std::string>(bool_pubs[who].begin(), bool_pubs[who].end());
    const auto subs = std::vector<std::string>(bool_subs[who].begin(), bool_subs[who].end());
    const std::string bt = kids[i]->btFinished();
    os << "child " << who << ": " << kids[i]->exitText() << ", 'BT finished: " << (bt.empty() ? "(absent)" : bt)
       << "', Bool publishers " << m26::listOrNone(pubs) << ", Bool subscriptions " << m26::listOrNone(subs) << "\n";
    sem.bt.push_back(who + ": BT finished " + (bt.empty() ? "(absent)" : bt) + ", " + kids[i]->exitText());
    sem.grasp.push_back("Bool state subscriptions of " + who + ": " + m26::listOrNone(subs));
  }
  const char* leaves[] = {"CloseGripper", "OpenGripper"};
  for (const auto& sv : servers)
  {
    for (const auto& g : sv->goals())
    {
      const std::string what = sv->name() + " " + (g.ordinal <= 2 ? leaves[g.ordinal - 1] : "goal") + ": ";
      const auto returned = exited.empty() ? m26::Clock::now() : *std::max_element(exited.begin(), exited.end());
      sem.command.push_back(what + m26::goalCommand(g));
      sem.ack.push_back(what + m26::goalAck(g, returned));
      sem.state.push_back(what + m26::goalState(g, returned));
      sem.code.push_back(what + m26::goalCode(g, returned));
      const std::string t = m26::goalTimeout(g, returned);
      if (t != "none") sem.timeout.push_back(what + t);
      if (g.ordinal == 2) sem.release.push_back(what + m26::goalCode(g, returned));
    }
    if (sv->goals().empty()) sem.ack.push_back(sv->name() + ": no goal arrived");
  }
  if (servers.empty()) sem.ack.push_back("no server: none serves the configured gripper_action");
  for (std::size_t i = 0; i < kids.size(); ++i)
    if (kids[i]->btFinished() == "FAILURE" && servers.empty())
      sem.timeout.push_back(childName(s.children[i]) + ": server wait (no server)");
  if (rec) sem.command.push_back("Bool on " + s.bool_topic + ": " + rec->byPublisher());
  if (late) sem.state.push_back("late joiner on " + s.bool_topic + ": " + late->multiset());
  os << sem.line() << "\n";
  return os.str();
}

const char* kRos2mlKeys =
  "gripper_action: \"/gripper_position_controller/gripper_cmd\"\n"
  "gripper_close_position: 0.7691\n"
  "gripper_open_position: 0.0\n"
  "gripper_cmd_topic: \"/gripper_cmd\"\n";

std::string nsKeys(const std::string& ns)
{
  return "gripper_action: \"" + ns + "/gripper_position_controller/gripper_cmd\"\n"
         "gripper_close_position: 0.7691\n"
         "gripper_open_position: 0.0\n"
         "gripper_cmd_topic: \"/gripper_cmd\"\n";
}
}  // namespace

TEST(M26RunnerChild, eef_runner_child)
{
  const std::string golden_action = "/suctioncup_controller/gripper_command";
  const std::string golden_keys = "gripper_action: \"" + golden_action + "\"\n";
  const std::vector<RcScenario> scenarios{
    {"RC-01", "golden-style keys (gripper_action only) against a shim-like server that ABORTS every goal: "
              "the runner exits 0 and publishes no grasp Bool",
     {{"/", golden_keys, "eef_tree.xml"}}, {{golden_action, {GripperReply::shimAbort(), GripperReply::shimAbort()}}},
     "/gripper_cmd"},
    {"RC-02", "ROS2ML-style keys (action + joint targets + gripper_cmd_topic) against a ros2_control-like server "
              "(close stalls SUCCEEDED, open reaches): goals 0.7691/0, Bool true then false, exit 0",
     {{"/", kRos2mlKeys, "eef_tree.xml"}},
     {{"/gripper_position_controller/gripper_cmd", {GripperReply::succeed(false, true).at(0.5, 0.0), GripperReply::succeed()}}},
     "/gripper_cmd"},
    {"RC-03", "MISSPELT keys (gripper_close_positon, gripper_cmd_topc) are accepted silently: the close sends the "
              "1.0 default and no grasp Bool is published; exit 0",
     {{"/", "gripper_action: \"/gripper_position_controller/gripper_cmd\"\ngripper_close_positon: 0.7691\n"
            "gripper_open_position: 0.0\ngripper_cmd_topc: \"/gripper_cmd\"\n", "eef_tree.xml"}},
     {{"/gripper_position_controller/gripper_cmd", {}}}, "/gripper_cmd"},
    {"RC-04", "two children in /r1 and /r3 (TSA-style namespaced actions, ONE global gripper_cmd_topic): each "
              "reaches its own server; both robots' Bools land on the one topic",
     {{"/r1", nsKeys("/r1"), "eef_tree.xml"}, {"/r3", nsKeys("/r3"), "eef_tree.xml"}},
     {{"/r1/gripper_position_controller/gripper_cmd", {}}, {"/r3/gripper_position_controller/gripper_cmd", {}}},
     "/gripper_cmd"},
    {"RC-05", "two children in /r1 and /r3 close and HOLD: a late joiner on the global grasp topic receives BOTH "
              "robots' latched true",
     {{"/r1", nsKeys("/r1"), "eef_tree_hold.xml"}, {"/r3", nsKeys("/r3"), "eef_tree_hold.xml"}},
     {{"/r1/gripper_position_controller/gripper_cmd", {}}, {"/r3/gripper_position_controller/gripper_cmd", {}}},
     "/gripper_cmd", true},
    {"RC-06", "golden-style keys and NO end-effector server (the golden's real mode): the first CloseGripper fails "
              "after the 5 s server wait; exit 1",
     {{"/", golden_keys, "eef_tree.xml"}}, {}, "/gripper_cmd"},
  };
  m26::Family f("eef_runner_child");
  for (const auto& s : scenarios) f.add(run(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
