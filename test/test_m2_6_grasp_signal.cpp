// vNext M2.6 (tests BEFORE, ADR-0022 family E2): eef_grasp_signal -- the latched std_msgs/Bool the
// runner publishes on gripper_cmd_topic (ADR-0010's sim grasp adapter signal), frozen on production
// 1dda503 before the canonical EEF contract touches a production line (test/data/m2_6/README.md).
//
// What is pinned: CloseGripper/OpenGripper publish it ONLY when GripperController::command()
// returned true -- so an ABORTED goal still publishes `true` (the result code is never read) and a
// REJECTED one publishes nothing; a Bool-only cell (no gripper action) publishes unconditionally;
// SetGripper never publishes; the publisher is QoS(1).transient_local(), so a late joiner receives
// the LAST sample only; and two runners publishing on ONE global name replay BOTH robots' last
// samples to a late joiner (each sample attributed to its publishing node through the graph).
#include <memory>
#include <string>
#include <vector>

#include "m2_6/gripper_scenarios.hpp"

using m26::BoolRecorder;
using m26::GripperReply;
using m26::GripperScenario;
using m26::RunnerKeys;

namespace
{
RunnerKeys keys(const std::string& action, const std::string& topic)
{
  RunnerKeys k;
  k.gripper_action = action;
  k.gripper_cmd_topic = topic;
  return k;
}

const std::vector<std::pair<std::string, BoolRecorder::Qos>> kBoth{
  {"volatile", BoolRecorder::Qos::VOLATILE}, {"latched", BoolRecorder::Qos::LATCHED}};
const char* kClose = "<CloseGripper/>";
const char* kOpen = "<OpenGripper/>";

// GS-07: two runners (two contexts, /r1 and /r3), each a Bool-only cell publishing on ONE global
// name. Its own block: two tree nodes, one recorder made before, one late joiner made after.
std::string crossRobotReplay()
{
  const std::string topic = "/m26_gs07/gripper_cmd";
  m26::TreeNode r1("trainit_run_bt", "/r1");
  m26::TreeNode r3("trainit_run_bt", "/r3");
  const RunnerKeys k = keys("", topic);
  auto c1 = m26::makeRunnerContext(r1.node(), k);
  auto c3 = m26::makeRunnerContext(r3.node(), k);
  BoolRecorder before("m26_rec_gs07_before", topic, BoolRecorder::Qos::VOLATILE);
  m26::waitUntil([&] {
    return c1->ctx.gripper_cmd_pub->get_subscription_count() >= 1 && c3->ctx.gripper_cmd_pub->get_subscription_count() >= 1;
  }, m26::milliseconds(5000));
  std::ostringstream os;
  os << "=== scenario GS-07\n"
     << "pins: cross-robot replay: two runners (/r1, /r3) publish the grasp Bool on ONE global name; a late joiner "
        "receives BOTH robots' last samples (sorted multiset, attributed to the publishing node)\n"
     << "runner keys (both): " << k.describe() << "\n"
     << "recorder 0: " << topic << " (" << before.qosText() << "), made before the ticks\n";
  struct Tick { m26::TreeNode* node; m26::RunnerContext* ctx; std::string tree; };
  const std::vector<Tick> ticks{{&r1, c1.get(), kClose}, {&r3, c3.get(), kClose}, {&r3, c3.get(), kOpen}};
  std::vector<std::string> bts;
  for (std::size_t i = 0; i < ticks.size(); ++i)
  {
    const std::string who = ticks[i].node->node()->get_fully_qualified_name();
    const std::string bt = m26::tickTree(ticks[i].tree, &ticks[i].ctx->ctx, ticks[i].node->node());
    os << "--- tick " << (i + 1) << " on " << who << ": " << ticks[i].tree << "\nbt: " << bt << "\n";
    bts.push_back(who + " " + m26::treeLeaf(ticks[i].tree) + "=" + bt);
  }
  before.settle(3, m26::milliseconds(3000));
  BoolRecorder late("m26_rec_gs07_late", topic, BoolRecorder::Qos::LATCHED);
  late.settle(2, m26::milliseconds(3000));
  os << "recorder 0 received: " << before.byPublisher() << "\n"
     << "late joiner (" << late.qosText() << ", made after the ticks) received: " << late.multiset() << "\n";
  m26::Semantic sem;
  sem.command.push_back("Bool on " + topic + ": " + before.byPublisher());
  sem.ack.push_back("none (a topic has no ack)");
  sem.state.push_back("late joiner on " + topic + ": " + late.multiset());
  sem.grasp.push_back(m26::boolStateSources(r1.node(), "trainit_run_bt", "/r1"));
  sem.grasp.push_back(m26::boolStateSources(r3.node(), "trainit_run_bt", "/r3"));
  sem.bt = bts;
  sem.release.push_back(bts.back());
  os << sem.line() << "\n";
  return os.str();
}
}  // namespace

TEST(M26GraspSignal, eef_grasp_signal)
{
  std::vector<GripperScenario> scenarios{
    {"GS-01", "SUCCEEDED close and open: true then false, on the configured name; a late joiner receives the last sample only",
     keys("/m26_gs01/gripper_command", "/m26_gs01/gripper_cmd"), true, {GripperReply::succeed(), GripperReply::succeed()},
     {kClose, kOpen}, 300, kBoth, true},
    {"GS-02", "ABORTED close and open (the shim's answer): command() returned true, so true and false are STILL published",
     keys("/m26_gs02/gripper_command", "/m26_gs02/gripper_cmd"), true, {GripperReply::shimAbort(), GripperReply::shimAbort()},
     {kClose, kOpen}, 300, kBoth, true},
    {"GS-03", "a REJECTED close publishes nothing; the following SUCCEEDED open publishes false",
     keys("/m26_gs03/gripper_command", "/m26_gs03/gripper_cmd"), true, {GripperReply::reject(), GripperReply::succeed()},
     {kClose, kOpen}, 300, kBoth, true},
    {"GS-04", "a Bool-only cell (gripper_cmd_topic, no gripper_action): Close/Open publish unconditionally; SetGripper fails and publishes nothing",
     keys("", "/m26_gs04/gripper_cmd"), false, {}, {kClose, kOpen, "<SetGripper position=\"0.5\"/>"}, 300, kBoth, true},
    {"GS-05", "no server at the gripper action: the close fails after the 5 s server wait and publishes nothing",
     keys("/m26_gs05/gripper_command", "/m26_gs05/gripper_cmd"), false, {}, {kClose}, 300, kBoth, true},
    {"GS-06", "latched vs volatile vs late joiner: close, open, close -- both early recorders receive all three, the late joiner the last (true)",
     keys("/m26_gs06/gripper_command", "/m26_gs06/gripper_cmd"), true, {}, {kClose, kOpen, kClose}, 300, kBoth, true},
  };
  m26::Family f("eef_grasp_signal");
  for (const auto& s : scenarios) f.add(m26::runGripperScenario(s));
  f.add(crossRobotReplay());
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
