// test_m3_4_runner_stop (vNext M3.4, owner decision D8 -- the MINIMAL stop scope): the REAL
// trainit_run_bt, started as a child process the way a launch starts it, is blocked inside a planned
// motion (the fake move_group holds the ExecuteTrajectory goal) when SIGINT arrives. Captured: what
// the root move_group fakes received (summaries) and the topics they heard -- since M3.4 the runner's
// SIGINT/shutdown handling calls MotionRuntime::stop(), so move_group's trajectory_execution_event
// carries "stop" -- and how the child ended.
//
// The motion nodes stay synchronous (D8): the tick blocked in MoveGroupInterface::execute() is not
// interrupted, so after the stop the child does not return from it; the launch's escalation ends it
// (here: SIGKILL after a 3 s grace, recorded). The same scenario is recorded BEFORE (TMR 2.2.0,
// 9418694: no stop event) and AFTER (test/data/m3_4/{before,after}/m3_4_runner_stop.txt). Domain 85,
// localhost only, graph-guarded.
#include <sys/stat.h>

#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"

namespace
{
const std::string kData = TRAINIT_M34_DATA_DIR;

std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/m3_4_runner_stop"; }

std::string paramsFile()
{
  const std::string path = actualDir() + "/stop.yaml";
  std::ostringstream y;
  y << "# vNext M3.4: trainit_run_bt params, written by test_m3_4_runner_stop\n"
    << "/**:\n  ros__parameters:\n"
    << "    bt_tree_file: \"" << kData << "/runner_stop_tree.xml\"\n"
    << "    bt_tree_id: \"MainTree\"\n"
    << "    planner_mode: \"ompl\"\n"
    << "    process_path_backend: \"pilz_sequence\"\n"
    << "    group: \"arm\"\n"
    << "    base_frame: \"base_link\"\n"
    << "    tip_link: \"tcp\"\n"
    << "    available_pipelines: [\"ompl\", \"pilz_industrial_motion_planner\", \"chomp\"]\n"
    << "    process_controller: \"mock\"\n"
    << "    enforce_validation: true\n"
    << "    groot_monitor: false\n"
    << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
    << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6);
  m26::writeFile(path, y.str());
  return path;
}
}  // namespace

TEST(M34RunnerStop, sigint_during_a_motion_sends_the_stop_event)
{
  ::mkdir(actualDir().c_str(), 0755);
  m25::Log log;
  log.setBodies(false);
  auto mg = std::make_unique<m25::FakeMoveGroup>(log, "/");
  mg->ex_script.push_back(m25::Reply::hold());     // the execution never ends by itself
  auto js = std::make_unique<m25::FakeJointStates>("/");
  std::this_thread::sleep_for(m26::milliseconds(500));

  std::ostringstream os;
  os << "=== scenario STOP-RUNNER-01\n"
     << "pins: trainit_run_bt blocked in MoveToNamed's execution (held by the fake) receives SIGINT: the motion "
        "in flight gets move_group's stop event (2.2.0: nothing was sent); the blocked tick does not return, the "
        "launch's escalation ends the process\n"
     << "child /trainit_run_bt: trainit_run_bt --ros-args --params-file <stop params> (tree "
        "test/data/m3_4/runner_stop_tree.xml: one MoveToNamed ready)\n"
     << "script: " << mg->scriptText() << "\n";

  m26::watchdog().arm("STOP-RUNNER-01", std::chrono::seconds(120));
  m26::ChildProcess child(TRAINIT_RUN_BT_EXE, {"--ros-args", "--params-file", paramsFile()},
                          m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), actualDir() + "/stop.log");
  ASSERT_TRUE(child.started());
  const bool executing = m26::waitUntil([&] { return log.count("/execute_trajectory") >= 1; },
                                        m26::milliseconds(60000));
  os << "execution in flight before SIGINT: " << (executing ? "yes" : "NO") << "\n";
  log.reset();                                      // what SIGINT causes, alone
  child.signal(SIGINT);
  m25::settleTopics(log);
  std::this_thread::sleep_for(m26::milliseconds(1000));
  child.stop(m26::milliseconds(3000));             // SIGINT again, then SIGKILL after the grace
  mg->releaseHeld();
  m26::watchdog().disarm();

  const std::string topics = log.topics();
  os << "after SIGINT, events of the root move_group fakes (summaries):\n"
     << (log.events().empty() ? std::string("(none)\n") : log.events())
     << "after SIGINT, topics heard (sorted):\n" << (topics.empty() ? std::string("(none)\n") : topics)
     << "child /trainit_run_bt: " << child.exitText() << ", 'BT finished: "
     << (child.btFinished().empty() ? std::string("(absent)") : child.btFinished()) << "'\n\n";

  m25::Family f("m3_4_runner_stop",
#ifdef TRAINIT_M34_AFTER
                kData + "/after"
#else
                kData + "/before"
#endif
  );
  f.add(os.str());
  EXPECT_TRUE(executing) << os.str();
#ifdef TRAINIT_M34_AFTER
  EXPECT_NE(topics.find("/trajectory_execution_event String: stop"), std::string::npos) << os.str();
#endif
  f.check();
  js.reset();
  mg.reset();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
