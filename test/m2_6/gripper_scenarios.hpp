// vNext M2.6 test harness -- one GripperCommand / grasp-Bool scenario: the runner's EEF context (built
// like trainit_run_bt), a ScriptedGripper, optional Bool recorders, trees ticked in order, then a
// watch window after the last tick (a late goal response, a held goal, a cancel request that never
// comes). Used by the eef_gripper_command (E1) and eef_grasp_signal (E2) families.
//
// The capture block: the scenario's keys and script; per tick its tree, the server events that
// happened while it ran, its BT result; the events of the watch window; the recorders; and the
// semantic line, computed from the ScriptedGripper's goal records (which tick each goal arrived
// in, whether its response and its result came before that tick returned), the recorders'
// samples, and the graph (the tree node's Bool subscriptions).
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_GRIPPER_SCENARIOS_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_GRIPPER_SCENARIOS_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "capture26.hpp"
#include "eef_fakes.hpp"

namespace m26
{

struct GripperScenario
{
  std::string id{};
  std::string pins{};                         // one line: what this scenario pins
  RunnerKeys keys{};
  bool serve{true};                           // a ScriptedGripper at keys.gripper_action
  std::vector<GripperReply> script{};
  std::vector<std::string> trees{};           // ticked in order on ONE context
  int watch_after_ms{1000};                   // after the last tick, the server is watched this long
  std::vector<std::pair<std::string, BoolRecorder::Qos>> recorders{};   // on keys.gripper_cmd_topic, made BEFORE the ticks
  bool late_joiner{false};                    // a LATCHED recorder made AFTER the ticks
  int budget_s{60};
};

inline std::string treeLeaf(const std::string& xml)
{
  // "<CloseGripper/>" -> "CloseGripper"; "<SetGripper position=..."/> -> "SetGripper"
  const auto a = xml.find('<');
  const auto b = xml.find_first_of(" />", a + 1);
  return (a == std::string::npos || b == std::string::npos) ? xml : xml.substr(a + 1, b - a - 1);
}

inline std::string runGripperScenario(const GripperScenario& s)
{
  Log log;
  auto node = treeNode().node();
  Watchdog& dog = watchdog();
  std::unique_ptr<ScriptedGripper> server;
  if (s.serve && !s.keys.gripper_action.empty())
    server = std::make_unique<ScriptedGripper>(log, "/", s.keys.gripper_action, s.script);
  auto rc = makeRunnerContext(node, s.keys);
  std::vector<std::unique_ptr<BoolRecorder>> recs;
  for (std::size_t i = 0; i < s.recorders.size(); ++i)
    recs.push_back(std::make_unique<BoolRecorder>("m26_rec_" + tagOf(s.id) + "_" + std::to_string(i),
                                                  s.keys.gripper_cmd_topic, s.recorders[i].second));
  // trainit_run_bt sleeps 3 s for DDS before the first tick; here the conditions it waits for:
  // the client sees its server, and the grasp publisher is matched by every recorder.
  if (server && rc->gripper) rc->gripper->waitForServer(5.0);
  if (rc->ctx.gripper_cmd_pub && !recs.empty())
    waitUntil([&] { return rc->ctx.gripper_cmd_pub->get_subscription_count() >= recs.size(); }, milliseconds(5000));

  std::ostringstream os;
  os << "=== scenario " << s.id << "\n"
     << "pins: " << s.pins << "\n"
     << "runner keys: " << s.keys.describe() << "\n"
     << "server: " << (server ? server->name() + " script: " + server->scriptText() : std::string("none")) << "\n";
  for (std::size_t i = 0; i < recs.size(); ++i)
    os << "recorder " << i << ": " << s.keys.gripper_cmd_topic << " (" << recs[i]->qosText() << "), made before the ticks\n";

  std::string before;
  std::vector<std::pair<Clock::time_point, Clock::time_point>> windows;
  std::vector<std::string> results;
  for (std::size_t i = 0; i < s.trees.size(); ++i)
  {
    dog.arm(s.id + " tick " + std::to_string(i + 1), std::chrono::seconds(s.budget_s));
    const auto t0 = Clock::now();
    const std::string bt = tickTree(s.trees[i], &rc->ctx, node);
    const auto t1 = Clock::now();
    dog.disarm();
    windows.emplace_back(t0, t1);
    results.push_back(bt);
    const std::string all = log.events();
    os << "--- tick " << (i + 1) << ": " << s.trees[i] << "\n"
       << "events:\n" << (all.size() > before.size() ? all.substr(before.size()) : std::string("(none)\n"))
       << "bt: " << bt << "\n";
    before = all;
  }

  // the watch window: a late response gets executed, a held goal stays held, a cancel never comes
  dog.arm(s.id + " watch", std::chrono::seconds(s.budget_s));
  if (server)
  {
    waitUntil([&] {
      for (const auto& g : server->goals())
        if (g.response.empty() || (g.response != "REJECT" && g.terminal.empty() &&
                                   g.reply.kind != GripperReply::Kind::WITHHOLD))
          return false;
      return true;
    }, milliseconds(15000));
    server->joinWorkers();
  }
  std::this_thread::sleep_for(milliseconds(s.watch_after_ms));
  for (auto& r : recs) r->settle(0, milliseconds(2000));
  std::unique_ptr<BoolRecorder> late;
  if (s.late_joiner && !s.keys.gripper_cmd_topic.empty())
  {
    late = std::make_unique<BoolRecorder>("m26_rec_" + tagOf(s.id) + "_late", s.keys.gripper_cmd_topic,
                                          BoolRecorder::Qos::LATCHED);
    late->settle(rc->ctx.gripper_cmd_pub ? 1u : 0u, milliseconds(3000));
  }
  dog.disarm();
  const std::string all = log.events();
  os << "--- after the last tick (watched " << s.watch_after_ms << " ms more):\n"
     << "events:\n" << (all.size() > before.size() ? all.substr(before.size()) : std::string("(none)\n"));
  for (std::size_t i = 0; i < recs.size(); ++i)
    os << "recorder " << i << " received: " << recs[i]->byPublisher() << "\n";
  if (late) os << "late joiner (" << late->qosText() << ", made after the ticks) received: " << late->byPublisher() << "\n";

  // --- the semantic line, from the records -------------------------------------------------------
  Semantic sem;
  const std::vector<GoalRecord> goals = server ? server->goals() : std::vector<GoalRecord>{};
  auto tickOf = [&](const GoalRecord& g) -> int {
    for (std::size_t i = 0; i < windows.size(); ++i)
      if (g.received >= windows[i].first && g.received <= windows[i].second) return static_cast<int>(i);
    return -1;
  };
  for (std::size_t i = 0; i < s.trees.size(); ++i)
  {
    const std::string leaf = treeLeaf(s.trees[i]);
    const GoalRecord* goal = nullptr;
    for (const auto& g : goals)
      if (tickOf(g) == static_cast<int>(i)) goal = &g;
    const auto returned = windows[i].second;
    if (goal)
    {
      sem.command.push_back(leaf + ": " + goalCommand(*goal));
      sem.ack.push_back(leaf + ": " + goalAck(*goal, returned));
      sem.state.push_back(leaf + ": " + goalState(*goal, returned));
      sem.code.push_back(leaf + ": " + goalCode(*goal, returned));
      const std::string t = goalTimeout(*goal, returned);
      if (t != "none") sem.timeout.push_back(leaf + ": " + t);
    }
    else if (!s.keys.gripper_action.empty())
    {
      sem.ack.push_back(leaf + ": no goal reached a server" + (server ? "" : " (none serves " + s.keys.gripper_action + ")"));
      if (!server && results[i] != "SUCCESS") sem.timeout.push_back(leaf + ": server wait (no server)");
    }
    else
    {
      sem.ack.push_back(leaf + ": no GripperCommand (no gripper_action)");
    }
    sem.bt.push_back(leaf + "=" + results[i]);
    if (leaf == "OpenGripper")
      sem.release.push_back("OpenGripper=" + results[i] + (goal ? " (goal " + goalCode(*goal, returned) + ")" : ""));
  }
  if (!recs.empty())
  {
    const auto values = recs.front()->values();
    sem.command.push_back("Bool on " + s.keys.gripper_cmd_topic + ": " + (values.empty() ? std::string("none") : join(values)));
  }
  if (late)
    sem.state.push_back("late joiner on " + s.keys.gripper_cmd_topic + ": " +
                        (late->values().empty() ? std::string("nothing") : join(late->values())));
  sem.grasp.push_back(boolStateSources(node, node->get_name(), node->get_namespace()));
  if (server && server->cancelRequests() > 0)
    sem.ack.push_back("cancel requests received by the server: " + std::to_string(server->cancelRequests()) +
                      " (the harness's second client sent " + std::to_string(server->preemptsSent()) + ")");
  os << sem.line() << "counts: " << log.counts() << "\n\n";
  return os.str();
}

}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_GRIPPER_SCENARIOS_HPP_
