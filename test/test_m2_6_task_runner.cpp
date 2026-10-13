// vNext M2.6 (tests BEFORE, ADR-0022 family E6): eef_task_runner -- TaskRunner's GRIPPER step
// (src/task_runner.cpp eefAction), frozen on production 1dda503 before the canonical EEF contract
// touches a production line (test/data/m2_6/README.md).
//
// TaskRunner needs a real MoveItMotionRuntime (every step moves first), so this family reuses the
// M2.5 rig and fake move_group (test/m2_5/), unchanged: the MoveIt traffic is printed as summaries
// only (it is what the M2.5 captures pin), interleaved in order with the ScriptedGripper's events.
// What is pinned: the step sends GripperCommand(position=<eef.gripper>, max_effort=0) after its
// move; ABORTED counts as done (the run goes on); REJECTED, or no gripper, ends the run false at
// that step.
#include <memory>
#include <string>
#include <vector>

#include "m2_5/motion_harness.hpp"
#include "m2_6/eef_fakes.hpp"
#include "trainit_motion_runtime/task_loader.hpp"
#include "trainit_motion_runtime/task_runner.hpp"

using m26::GripperReply;

namespace
{
struct TrScenario
{
  std::string id;
  std::string pins;
  bool serve;                        // a ScriptedGripper at the action
  bool gripper;                      // TaskRunner gets a GripperController
  std::vector<GripperReply> script;
};

std::string runTask(const TrScenario& s)
{
  m25::World& w = m25::world();
  w.log.setBodies(false);
  std::string tag = s.id;
  std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c) { return c == '-' ? '_' : std::tolower(c); });
  const std::string action = "/m26_" + tag + "/gripper_command";
  std::unique_ptr<m26::ScriptedGripper> server;
  if (s.serve) server = std::make_unique<m26::ScriptedGripper>(w.log, "/", action, s.script);
  m25::Config cfg = m25::defaultConfig();
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.mg.resetScenario();
  w.dog.arm("build " + s.id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();
  std::unique_ptr<trainit::GripperController> gc;
  if (s.gripper) gc = std::make_unique<trainit::GripperController>(w.rig.node(), action);
  if (gc && server) gc->waitForServer(5.0);
  trainit::TaskRunner runner(*w.rig.runtime(), gc.get(), nullptr);
  trainit::MotionTask task = trainit::TaskLoader::fromYamlFile(std::string(TRAINIT_M26_DATA_DIR) + "/eef_task.yaml");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  w.log.reset();
  w.dog.arm(s.id, std::chrono::seconds(120));
  std::string result;
  const auto t0 = m26::Clock::now();
  try { result = runner.run(task) ? "true" : "false"; }
  catch (const std::exception& e) { result = "THROW " + m26::demangle(typeid(e).name()) + ": " + e.what(); }
  const auto t1 = m26::Clock::now();
  m25::settleTopics(w.log);
  w.dog.disarm();
  std::ostringstream os;
  os << "=== scenario " << s.id << "\n"
     << "pins: " << s.pins << "\n"
     << "config: ns=/ " << m25::describe(cfg) << "\n"
     << "state: " << w.js.describe() << "\n"
     << "task: test/data/m2_6/eef_task.yaml (grip: ptp ready + gripper 1.0; release: ptp home + gripper 0.0)\n"
     << "gripper: " << (s.gripper ? "GripperController(" + action + ")" : std::string("none (TaskRunner gets nullptr)"))
     << "; server: " << (server ? server->name() + " script: " + server->scriptText() : std::string("none")) << "\n"
     << "events (MoveIt summaries only):\n" << w.log.events();
  const std::string topics = w.log.topics();
  if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;
  os << "run: " << result << "\n";

  m26::Semantic sem;
  const auto goals = server ? server->goals() : std::vector<m26::GoalRecord>{};
  const char* steps[] = {"grip", "release"};
  for (const auto& g : goals)
  {
    const std::string step = g.ordinal <= 2 ? steps[g.ordinal - 1] : "step " + std::to_string(g.ordinal);
    sem.command.push_back(step + ": " + m26::goalCommand(g));
    sem.ack.push_back(step + ": " + m26::goalAck(g, t1));
    sem.state.push_back(step + ": " + m26::goalState(g, t1));
    sem.code.push_back(step + ": " + m26::goalCode(g, t1));
    const std::string t = m26::goalTimeout(g, t1);
    if (t != "none") sem.timeout.push_back(step + ": " + t);
  }
  if (goals.empty()) sem.ack.push_back(s.gripper ? "no goal reached a server" : "no GripperCommand (no gripper)");
  (void)t0;
  sem.grasp.push_back(m26::boolStateSources(w.rig.node(), "trainit_run_bt", "/"));
  sem.bt.push_back("TaskRunner::run=" + result);
  sem.release.push_back(goals.size() >= 2 ? "release step: GripperCommand " + m26::goalCode(goals[1], t1)
                                          : std::string("release step not reached"));
  os << sem.line() << "counts: " << w.log.counts() << "\n\n";
  return os.str();
}
}  // namespace

TEST(M26TaskRunner, eef_task_runner)
{
  const std::vector<TrScenario> scenarios{
    {"TR-01", "both gripper steps SUCCEEDED: run true", true, true, {GripperReply::succeed(), GripperReply::succeed()}},
    {"TR-02", "the grip ABORTED (the shim's answer): counted as done, the run goes on to the release and ends true",
     true, true, {GripperReply::shimAbort(), GripperReply::shimAbort()}},
    {"TR-03", "the grip REJECTED: the run ends false at the grip step, the release never runs", true, true,
     {GripperReply::reject()}},
    {"TR-04", "no gripper configured (nullptr): the run ends false at the grip step, after its move", false, false, {}},
  };
  m26::Family f("eef_task_runner");
  for (const auto& s : scenarios) f.add(runTask(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN, [] {
    for (auto& t : m25::teardowns()) t();
    m25::worldSlot().reset();
  });
}
