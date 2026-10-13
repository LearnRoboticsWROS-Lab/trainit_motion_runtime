// vNext M2.7 (owner decision D1, test T8; WP_NS 7.3, INTEGRATION G-2): the runners' MoveIt start-up
// wait, on the REAL trainit_run_bt / trainit_run_task as child processes (a params file, `-r __ns:=`
// for a namespaced robot), domain 90. BEFORE (TMR 2.0.0) a runner with no move_group waited for ever
// and in silence, and Ctrl-C in that wait aborted it (SIGABRT, an RCLError) -- recorded in the M2.7
// forensics. Pinned here:
//   W01-W03  MoveIt absent: a WARN every 5 s naming the endpoints still missing, under the runner's
//            namespace (/r3/..., never a root name); at moveit_wait_timeout_s a FATAL naming them,
//            exit 2, no tick; the root names at the root; trainit_run_task waits the same way;
//   W04      moveit_wait_timeout_s -1: it keeps waiting (still WARNing), and goes on once MoveIt appears;
//   W05      only what is missing is named (the actions served, the scene services not);
//   W06      an invalid value (0, -2, -0.5, a string, nan, inf, a bool, an array) is a configuration
//            error: exit 2 at once, before any wait, in both runners;
//   W07      an integer, a double or no value at all, with MoveIt up: no start-up line at all;
//   W08      Ctrl-C in the wait: exit 1, one INFO line, no abort;
//   W09      trainit_run_task refuses a cross-namespace remap at once, as trainit_run_bt does (NS-G,
//            ADR-0023 Decision 5; trainit_run_bt's cases are in test_m2_7_namespace_runner);
//   MW-07    the literal default (no parameter at all): a WARN every 5 s, the FATAL at 60 s, exit 2;
//   MW-08    use_sim_time true and nobody publishes /clock: the bound runs on a steady clock and fires.
// What is pinned is the text and the exit; the wall-clock ranges are wide on purpose (a loaded
// machine), and where a test must wait for the child it waits for the child's own line, not a time.
#include <signal.h>
#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"
#include "m2_6/eef_fakes.hpp"

namespace
{
using m26::milliseconds;

std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/m2_7_moveit_wait"; }

struct Run
{
  std::string exit, finished;
  double wall_s{0.0};   // from the child's start to its exit, steady clock
  std::vector<std::string> lines;
  int count(const std::string& text) const
  {
    int n = 0;
    for (const auto& l : lines) if (l.find(text) != std::string::npos) ++n;
    return n;
  }
  std::string dump() const { std::string s; for (const auto& l : lines) s += l + "\n"; return s; }
};

const char* kAllNames = "action /r3/move_action, action /r3/execute_trajectory, service /r3/get_planning_scene, "
                        "service /r3/apply_planning_scene";

bool fileHas(const std::string& path, const std::string& text)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str().find(text) != std::string::npos;
}

// Starts the runner; `during` runs while it is alive (it may start fakes or send a signal); `extra`
// goes on the command line after the parameters (e.g. a remap).
Run run(const std::string& id, const std::string& exe, const std::string& ns, const std::vector<std::string>& p,
        milliseconds budget, const std::function<void(m26::ChildProcess&)>& during = {},
        const std::vector<std::string>& extra = {})
{
  ::mkdir(actualDir().c_str(), 0755);
  const std::string stem = actualDir() + "/" + m26::tagOf(id);
  m26::writeFile(stem + ".xml", "<?xml version=\"1.0\"?>\n<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
                                "  <BehaviorTree ID=\"MainTree\">\n    <AlwaysSuccess/>\n  </BehaviorTree>\n</root>\n");
  std::ostringstream y;
  y << "/**:\n  ros__parameters:\n"
    << "    bt_tree_file: \"" << stem << ".xml\"\n    task_file: \"" << TRAINIT_M27_TASK_FILE << "\"\n"
    << "    planner_mode: \"ompl\"\n    group: \"arm\"\n    base_frame: \"base_link\"\n    tip_link: \"tcp\"\n"
    << "    process_controller: \"mock\"\n    groot_monitor: false\n"
    << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
    << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6);
  m26::writeFile(stem + ".yaml", y.str());
  // The previous run's log of this id is still in the build tree, and the child truncates it only after
  // fork(), once it runs: empty it first, so `during` polling c.output() can only read this run's lines.
  m26::writeFile(stem + ".log", "");
  std::vector<std::string> args{"--ros-args", "--params-file", stem + ".yaml"};
  if (ns != "/") { args.push_back("-r"); args.push_back("__ns:=" + ns); }
  for (const auto& kv : p) { args.push_back("-p"); args.push_back(kv); }
  args.insert(args.end(), extra.begin(), extra.end());
  Run r;
  {
    const auto t0 = std::chrono::steady_clock::now();
    m26::ChildProcess child(exe, args, m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), stem + ".log");
    m26::watchdog().arm(id, std::chrono::seconds(120));
    if (during) during(child);
    child.wait(budget);
    r.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    child.stop();
    m26::watchdog().disarm();
    r.exit = child.exitText();
    r.finished = child.btFinished();
  }
  std::ifstream in(stem + ".log");
  std::string line;
  while (std::getline(in, line)) r.lines.push_back(line);
  return r;
}
}  // namespace

TEST(M27MoveItWait, absent_moveit_is_named_every_5_s_and_ends_in_a_fatal_and_exit_2)
{
  const auto r = run("W01 r3", TRAINIT_RUN_BT_EXE, "/r3", {"moveit_wait_timeout_s:=6"}, milliseconds(30000));
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_EQ(r.finished, "");
  EXPECT_EQ(r.count(std::string("MoveIt start-up: still waiting after 5 s for ") + kAllNames +
                    " (namespace /r3, moveit_wait_timeout_s 6)"), 1) << r.dump();
  EXPECT_EQ(r.count("still waiting after 10 s"), 0);
  EXPECT_EQ(r.count(std::string("MoveIt start-up: not served after 6 s: ") + kAllNames +
                    " (namespace /r3; is this robot's move_group running?) -- exit 2"), 1) << r.dump();
  EXPECT_EQ(r.count(" /move_action"), 0) << "under /r3 no ROOT name is waited for";
  EXPECT_GE(r.wall_s, 5.5) << "the bound is 6 s";
  EXPECT_LE(r.wall_s, 12.0) << "the bound is 6 s";
  // the root: the root names; a double value
  const auto root = run("W02 root", TRAINIT_RUN_BT_EXE, "/", {"moveit_wait_timeout_s:=6.0"}, milliseconds(30000));
  EXPECT_EQ(root.exit, "exit code 2") << root.dump();
  EXPECT_EQ(root.count("MoveIt start-up: not served after 6 s: action /move_action, action /execute_trajectory, "
                       "service /get_planning_scene, service /apply_planning_scene (namespace /;"), 1) << root.dump();
  // trainit_run_task waits the same way
  const auto task = run("W03 task r3", TRAINIT_RUN_TASK_EXE, "/r3", {"moveit_wait_timeout_s:=6"}, milliseconds(30000));
  EXPECT_EQ(task.exit, "exit code 2") << task.dump();
  EXPECT_EQ(task.count(std::string("MoveIt start-up: still waiting after 5 s for ") + kAllNames +
                       " (namespace /r3, moveit_wait_timeout_s 6)"), 1) << task.dump();
  EXPECT_EQ(task.count(std::string("MoveIt start-up: not served after 6 s: ") + kAllNames), 1) << task.dump();
}

TEST(M27MoveItWait, minus_one_keeps_waiting_and_goes_on_once_moveit_appears)
{
  m25::Log log;
  std::unique_ptr<m25::FakeMoveGroup> late;
  bool saw_10_s = false, alive = false;
  const auto r = run("W04 -1", TRAINIT_RUN_BT_EXE, "/r3", {"moveit_wait_timeout_s:=-1"}, milliseconds(60000),
                     [&](m26::ChildProcess& c) {
                       // the child's own 10 s WARN: -1 waited past it; only then does MoveIt appear
                       saw_10_s = m26::waitUntil([&] { return fileHas(c.output(), "still waiting after 10 s"); },
                                                 milliseconds(30000));
                       alive = c.running();
                       late = std::make_unique<m25::FakeMoveGroup>(log, "/r3");
                     });
  EXPECT_TRUE(saw_10_s) << r.dump();
  EXPECT_TRUE(alive) << "-1 is an unbounded wait";
  EXPECT_EQ(r.exit, "exit code 0") << r.dump();
  EXPECT_EQ(r.finished, "SUCCESS");
  EXPECT_EQ(r.count("MoveIt start-up: still waiting after 5 s for"), 1) << r.dump();
  EXPECT_EQ(r.count("MoveIt start-up: still waiting after 10 s for"), 1) << r.dump();
  const int warns = r.count("MoveIt start-up: still waiting after ");
  // a third WARN (15 s) only if discovering the late move_group took about 5 s: a loaded machine
  EXPECT_GE(warns, 2) << r.dump();
  EXPECT_LE(warns, 3) << r.dump();
  EXPECT_EQ(r.count("moveit_wait_timeout_s -1, no bound)"), warns) << "every WARN names the unbounded wait";
  EXPECT_EQ(r.count("MoveIt start-up: every endpoint is served (namespace /r3)"), 1) << r.dump();
  EXPECT_EQ(r.count("not served"), 0);
}

TEST(M27MoveItWait, only_the_missing_endpoints_are_named)
{
  m25::Log log;
  m25::FakeMoveGroup actions(log, "/r3", m25::FakeMoveGroup::MOVE_ACTION | m25::FakeMoveGroup::EXECUTE);
  const auto r = run("W05 services missing", TRAINIT_RUN_BT_EXE, "/r3", {"moveit_wait_timeout_s:=6"}, milliseconds(30000));
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_EQ(r.count("MoveIt start-up: still waiting after 5 s for service /r3/get_planning_scene, "
                    "service /r3/apply_planning_scene (namespace /r3, moveit_wait_timeout_s 6)"), 1) << r.dump();
  EXPECT_EQ(r.count("MoveIt start-up: not served after 6 s: service /r3/get_planning_scene, "
                    "service /r3/apply_planning_scene (namespace /r3;"), 1) << r.dump();
  EXPECT_EQ(r.count("action /r3/"), 0) << "the served actions are not named";
}

TEST(M27MoveItWait, an_invalid_value_is_a_configuration_error)
{
  const std::vector<std::pair<std::string, std::string>> cases{
    {"moveit_wait_timeout_s:=0", "invalid moveit_wait_timeout_s 0 (integer)"},
    {"moveit_wait_timeout_s:=-2", "invalid moveit_wait_timeout_s -2 (integer)"},
    {"moveit_wait_timeout_s:=-0.5", "invalid moveit_wait_timeout_s -0.500000 (double)"},
    {"moveit_wait_timeout_s:=abc", "invalid moveit_wait_timeout_s abc (string)"},
    {"moveit_wait_timeout_s:=.nan", "invalid moveit_wait_timeout_s nan (double)"},
    {"moveit_wait_timeout_s:=.inf", "invalid moveit_wait_timeout_s inf (double)"},
    {"moveit_wait_timeout_s:=true", "invalid moveit_wait_timeout_s true (bool)"},
    {"moveit_wait_timeout_s:=[1,2]", "invalid moveit_wait_timeout_s [1, 2] (integer_array)"}};
  for (const auto& [arg, line] : cases)
    for (const char* exe : {TRAINIT_RUN_BT_EXE, TRAINIT_RUN_TASK_EXE})
    {
      const auto r = run("W06 " + arg, exe, "/r3", {arg}, milliseconds(20000));
      EXPECT_EQ(r.exit, "exit code 2") << arg << " " << exe << "\n" << r.dump();
      EXPECT_EQ(r.count(line + ": a finite number of seconds > 0, or -1 to wait without a bound"), 1) << arg << "\n" << r.dump();
      EXPECT_EQ(r.count("MoveIt start-up"), 0) << arg << ": refused before any wait";
    }
}

TEST(M27MoveItWait, with_moveit_up_there_is_no_start_up_line)
{
  m25::Log log;
  m25::FakeMoveGroup mg(log, "/r3");
  std::this_thread::sleep_for(milliseconds(500));
  const std::vector<std::vector<std::string>> values{{}, {"moveit_wait_timeout_s:=60"}, {"moveit_wait_timeout_s:=60.0"},
                                                     {"moveit_wait_timeout_s:=-1"}};
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    const auto& p = values[i];
    // the id names no parameter: the runner logs its tree path, which carries the id
    const auto r = run("W07 up " + std::to_string(i), TRAINIT_RUN_BT_EXE, "/r3", p, milliseconds(40000));
    EXPECT_EQ(r.exit, "exit code 0") << r.dump();
    EXPECT_EQ(r.finished, "SUCCESS");
    EXPECT_EQ(r.count("MoveIt start-up"), 0) << r.dump();
    EXPECT_EQ(r.count("moveit_wait_timeout_s"), 0) << r.dump();
  }
}

TEST(M27MoveItWait, ctrl_c_in_the_wait_exits_1_without_an_abort)
{
  bool waiting = false;
  const auto r = run("W08 sigint", TRAINIT_RUN_BT_EXE, "/r3", {"moveit_wait_timeout_s:=-1"}, milliseconds(30000),
                     [&](m26::ChildProcess& c) {
                       // Ctrl-C once the child is in the wait (its own 5 s WARN says so)
                       waiting = m26::waitUntil([&] { return fileHas(c.output(), "still waiting after 5 s"); },
                                                milliseconds(20000));
                       c.signal(SIGINT);
                     });
  EXPECT_TRUE(waiting) << r.dump();
  EXPECT_EQ(r.exit, "exit code 1") << r.dump();
  EXPECT_EQ(r.count("interrupted while waiting for MoveIt: exit 1"), 1) << r.dump();
  EXPECT_EQ(r.count("terminate called"), 0) << r.dump();
}

TEST(M27MoveItWait, trainit_run_task_refuses_a_cross_namespace_remap_too)
{
  // W09 (NS-G in the second runner): the scene service of /r3 sent to /r2 -- refused before any wait
  const auto r = run("W09 task cross remap", TRAINIT_RUN_TASK_EXE, "/r3", {"moveit_wait_timeout_s:=6"},
                     milliseconds(20000), {}, {"-r", "get_planning_scene:=/r2/get_planning_scene"});
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_EQ(r.count("MoveIt start-up: a remap sends this robot's resources out of its namespace /r3: "
                    "service /r3/get_planning_scene -> /r2/get_planning_scene "
                    "(ADR-0023: a cross-namespace remap) -- exit 2"), 1) << r.dump();
  EXPECT_EQ(r.count("still waiting"), 0) << "refused before the wait";
  EXPECT_EQ(r.count("Loading robot model"), 0) << "refused before any MoveIt client exists";
}

TEST(M27MoveItWait, the_default_bound_is_60_s)
{
  // MW-07 (INTEGRATION G-2): no moveit_wait_timeout_s at all, no move_group, /r3
  const auto r = run("MW07 default", TRAINIT_RUN_BT_EXE, "/r3", {}, milliseconds(100000));
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_EQ(r.finished, "");
  EXPECT_GE(r.wall_s, 55.0) << "the default bound is 60 s";
  EXPECT_LE(r.wall_s, 70.0) << "the default bound is 60 s";
  // one WARN per 5 s boundary, 5 .. 55 s (11; the FATAL comes first at 60 s), each naming the four
  // /r3 endpoints and the bound
  const int warns = r.count("MoveIt start-up: still waiting after ");
  EXPECT_GE(warns, 10) << r.dump();
  EXPECT_LE(warns, 11) << r.dump();
  for (int k = 1; k <= warns; ++k)
    EXPECT_EQ(r.count("MoveIt start-up: still waiting after " + std::to_string(5 * k) + " s for " + kAllNames +
                      " (namespace /r3, moveit_wait_timeout_s 60)"), 1) << "the WARN at " << 5 * k << " s\n" << r.dump();
  EXPECT_EQ(r.count(std::string("MoveIt start-up: not served after 60 s: ") + kAllNames +
                    " (namespace /r3; is this robot's move_group running?) -- exit 2 "
                    "(moveit_wait_timeout_s; -1 waits without a bound)"), 1) << r.dump();
  EXPECT_EQ(r.count(" /move_action"), 0) << "under /r3 no ROOT name is waited for";
}

TEST(M27MoveItWait, a_stopped_sim_clock_does_not_stop_the_bound)
{
  // MW-08 (INTEGRATION G-2): use_sim_time true and nobody publishes /clock, so the runner's ROS
  // time stays 0 (Isaac mode starts that way): the bound runs on a steady clock and still fires
  auto probe = std::make_shared<rclcpp::Node>("m27_clock_probe", "/", rclcpp::NodeOptions().use_global_arguments(false));
  std::size_t subscribers = 0, publishers = 0;
  const auto r = run("MW08 sim time", TRAINIT_RUN_BT_EXE, "/r3", {"use_sim_time:=true", "moveit_wait_timeout_s:=6"},
                     milliseconds(30000), [&](m26::ChildProcess& c) {
                       // while it waits: its /clock subscription (use_sim_time took effect), and no publisher
                       m26::waitUntil([&] { subscribers = probe->count_subscribers("/clock");
                                            return subscribers > 0 || !c.running(); }, milliseconds(5000));
                       publishers = probe->count_publishers("/clock");
                     });
  EXPECT_GE(subscribers, 1u) << "the child subscribes to /clock: use_sim_time is on";
  EXPECT_EQ(publishers, 0u) << "nobody publishes /clock: the ROS time is stopped";
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_GE(r.wall_s, 5.5) << "the bound is 6 s of wall time";
  EXPECT_LE(r.wall_s, 12.0) << "the bound is 6 s of wall time";
  EXPECT_EQ(r.count(std::string("MoveIt start-up: still waiting after 5 s for ") + kAllNames +
                    " (namespace /r3, moveit_wait_timeout_s 6)"), 1) << r.dump();
  EXPECT_EQ(r.count(std::string("MoveIt start-up: not served after 6 s: ") + kAllNames + " (namespace /r3;"), 1)
    << r.dump();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
