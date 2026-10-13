// vNext M2.6 (tests BEFORE, ADR-0022 family E5): eef_run_policy -- RunPolicy, TMR's door to the
// policy runtime (the consumer of the policy runtime's EEF adapter, ADR-0022 F9), frozen on
// production 1dda503 before the canonical EEF contract touches a production line
// (test/data/m2_6/README.md).
//
// The policy runtime is a FakePolicyRuntime with policy_runtime_node.py's ROS contract (85053b3):
// ~/run (std_srvs/Trigger), ~/status (std_msgs/String JSON, RELIABLE + TRANSIENT_LOCAL depth 1,
// "running" published inside the run handler). What is pinned (bt_nodes.cpp RunPolicy): a missing
// service FAILS after the 2 s wait; a refused run FAILS; an answer later than timeout_ms FAILS;
// hybrid returns SUCCESS on the answer; pure/residual return SUCCESS once the status leaves
// "running" -- whatever it says ("success", "failure", "timeout") -- and ALSO when it never does
// (RunPolicy only reports that the policy RAN; CheckRobotState is left to judge). RunPolicy reads no
// grasp state. Each scenario uses its own service and status names: RunPolicy's clients and status
// feeds are process-static, keyed by name.
#include <memory>
#include <string>
#include <vector>

#include "m2_6/gripper_scenarios.hpp"

using m26::PolicyScript;

namespace
{
struct RpScenario
{
  std::string id;
  std::string pins;
  std::string node_name;     // the fake's node ("" = no policy runtime at all)
  std::string mode;
  PolicyScript script;
  std::string ports;         // RunPolicy's ports, verbatim
};

std::string run(const RpScenario& s)
{
  m26::Log log;
  auto node = m26::treeNode().node();
  auto rc = m26::makeRunnerContext(node, m26::RunnerKeys{});
  std::unique_ptr<m26::FakePolicyRuntime> fake;
  if (!s.node_name.empty())
  {
    fake = std::make_unique<m26::FakePolicyRuntime>(log, s.node_name, s.mode, s.script);
    // the client RunPolicy will create sees the service before the tick (trainit_run_bt's 3 s settle)
    auto probe = node->create_client<std_srvs::srv::Trigger>(fake->runName());
    probe->wait_for_service(std::chrono::seconds(5));
  }
  const std::string tree = "<RunPolicy " + s.ports + "/>";
  std::ostringstream os;
  os << "=== scenario " << s.id << "\n" << "pins: " << s.pins << "\n"
     << "policy runtime: " << (fake ? fake->runName() + " + " + fake->statusName() + " (mode " + s.mode + "): " +
                                        s.script.describe()
                                   : std::string("none")) << "\n";
  m26::watchdog().arm(s.id, std::chrono::seconds(60));
  const auto t0 = m26::Clock::now();
  const std::string bt = m26::tickTree(tree, &rc->ctx, node);
  const auto t1 = m26::Clock::now();
  const std::string during = log.events();
  // the watch window: a late answer and the states after it
  if (fake)
  {
    m26::waitUntil([&] { return fake->calls() == 0 || fake->answered(); }, m26::milliseconds(10000));
    fake->waitWorkers();
  }
  std::this_thread::sleep_for(m26::milliseconds(500));
  m26::watchdog().disarm();
  const std::string all = log.events();
  os << "--- tick 1: " << tree << "\n"
     << "events:\n" << (during.empty() ? std::string("(none)\n") : during)
     << "bt: " << bt << "\n"
     << "--- after the tick (watched 500 ms more):\n"
     << "events:\n" << (all.size() > during.size() ? all.substr(during.size()) : std::string("(none)\n"));

  m26::Semantic sem;
  if (fake && fake->calls() > 0)
  {
    sem.command.push_back("Trigger on " + fake->runName() + " x" + std::to_string(fake->calls()));
    sem.ack.push_back(fake->answered() && fake->answeredAt() <= t1
                        ? "answered success=" + m26::boolText(s.script.success) + " message='" + s.script.message + "'"
                        : std::string("no answer before RunPolicy returned"));
    std::vector<std::string> before;
    for (const auto& [state, at] : fake->states())
      if (at >= t0 && at <= t1) before.push_back(state);
    sem.state.push_back("status published before RunPolicy returned: " + (before.empty() ? std::string("nothing") : m26::join(before, " -> ")));
    const bool ended = !before.empty() && before.back() != "running";
    if (bt == "FAILURE" && !(fake->answered() && fake->answeredAt() <= t1))
      sem.timeout.push_back("run-answer wait (timeout_ms) expired");
    else if (bt == "SUCCESS" && s.mode != "hybrid" && !ended)
      sem.timeout.push_back("status wait (timeout_ms) expired with the status still running");
  }
  else
  {
    sem.ack.push_back("no run service" + std::string(fake ? " call" : " in the graph"));
    if (bt == "FAILURE") sem.timeout.push_back("service wait (2 s) expired");
  }
  sem.grasp.push_back(m26::boolStateSources(node, node->get_name(), node->get_namespace()));
  sem.bt.push_back("RunPolicy=" + bt);
  os << sem.line() << "\n";
  return os.str();
}

PolicyScript accepted(std::vector<std::string> then_states)
{
  PolicyScript p;
  p.then_states = std::move(then_states);
  return p;
}
}  // namespace

TEST(M26RunPolicy, eef_run_policy)
{
  PolicyScript hybrid;   // policy_runtime_node.py: _run_hybrid publishes hybrid_target_ready, then answers
  hybrid.message = "hybrid target published";
  hybrid.inline_states = {"hybrid_target_ready"};
  PolicyScript refused;
  refused.success = false;
  refused.message = "already running";
  PolicyScript late;
  late.respond_after_ms = 2500;
  const std::vector<RpScenario> scenarios{
    {"RP-01", "pure, the DEFAULT ports (/trainit_policy_runtime/run + /status): accepted, status running -> success: SUCCESS",
     "trainit_policy_runtime", "pure", accepted({"success"}), "mode=\"pure\" timeout_ms=\"5000\""},
    {"RP-02", "pure: accepted, status running -> failure: RunPolicy still reports SUCCESS (it only says the policy RAN)",
     "m26_policy_rp02", "pure", accepted({"failure"}),
     "service=\"/m26_policy_rp02/run\" status_topic=\"/m26_policy_rp02/status\" mode=\"pure\" timeout_ms=\"5000\""},
    {"RP-03", "pure: accepted, the runtime's own window expires (status timeout): SUCCESS",
     "m26_policy_rp03", "pure", accepted({"timeout"}),
     "service=\"/m26_policy_rp03/run\" status_topic=\"/m26_policy_rp03/status\" mode=\"pure\" timeout_ms=\"5000\""},
    {"RP-04", "pure: accepted, the status never leaves running: RunPolicy stops waiting after timeout_ms and reports SUCCESS",
     "m26_policy_rp04", "pure", accepted({}),
     "service=\"/m26_policy_rp04/run\" status_topic=\"/m26_policy_rp04/status\" mode=\"pure\" timeout_ms=\"1500\""},
    {"RP-05", "hybrid: SUCCESS on the answer, no status is read",
     "m26_policy_rp05", "hybrid", hybrid,
     "service=\"/m26_policy_rp05/run\" status_topic=\"/m26_policy_rp05/status\" mode=\"hybrid\" timeout_ms=\"5000\""},
    {"RP-06", "the runtime refuses the run (success=false): FAILURE",
     "m26_policy_rp06", "pure", refused,
     "service=\"/m26_policy_rp06/run\" status_topic=\"/m26_policy_rp06/status\" mode=\"pure\" timeout_ms=\"5000\""},
    {"RP-07", "no policy runtime in the graph: FAILURE after the 2 s service wait",
     "", "pure", PolicyScript{},
     "service=\"/m26_policy_rp07/run\" status_topic=\"/m26_policy_rp07/status\" mode=\"pure\" timeout_ms=\"5000\""},
    {"RP-08", "the run service answers after timeout_ms (2500 ms > 1000 ms): FAILURE; the late answer is discarded",
     "m26_policy_rp08", "pure", late,
     "service=\"/m26_policy_rp08/run\" status_topic=\"/m26_policy_rp08/status\" mode=\"pure\" timeout_ms=\"1000\""},
  };
  m26::Family f("eef_run_policy");
  for (const auto& s : scenarios) f.add(run(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
