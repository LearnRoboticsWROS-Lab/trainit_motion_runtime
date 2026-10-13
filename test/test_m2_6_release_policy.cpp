// vNext M2.6 (tests BEFORE, ADR-0022 family E4): eef_release_policy -- SetReleasePolicy, the
// simulator release metadata (ADR-0022 F7), frozen on production 1dda503 before the canonical EEF
// contract touches a production line (test/data/m2_6/README.md).
//
// What is pinned (bt_nodes.cpp SetReleasePolicy): its publisher is FUNCTION-STATIC, created on the
// first tick on the first node, so the FIRST topic binds for the life of the process -- a later
// tick naming another topic (or the default /isaac_release_policy) still publishes on the first;
// the value is true (freeze) for anything but "gravity"; QoS(1).transient_local(). This binary must
// be the only one ticking SetReleasePolicy in its process (it is: one family per binary).
#include <memory>
#include <string>
#include <vector>

#include "m2_6/gripper_scenarios.hpp"

using m26::BoolRecorder;

TEST(M26ReleasePolicy, eef_release_policy)
{
  auto node = m26::treeNode().node();
  auto rc = m26::makeRunnerContext(node, m26::RunnerKeys{});
  const std::string first = "/m26_rel/first", second = "/m26_rel/second", dflt = "/isaac_release_policy";
  BoolRecorder r_first("m26_rec_rel_first", first, BoolRecorder::Qos::LATCHED);
  BoolRecorder r_second("m26_rec_rel_second", second, BoolRecorder::Qos::LATCHED);
  BoolRecorder r_default("m26_rec_rel_default", dflt, BoolRecorder::Qos::LATCHED);
  const std::vector<std::string> trees{
    "<SetReleasePolicy policy=\"freeze\" topic=\"" + first + "\"/>",
    "<SetReleasePolicy policy=\"gravity\" topic=\"" + second + "\"/>",
    "<SetReleasePolicy policy=\"gravity\"/>",
    "<SetReleasePolicy policy=\"drop\" topic=\"" + second + "\"/>",
  };
  std::ostringstream os;
  os << "=== scenario REL-01\n"
     << "pins: the function-static publisher binds the FIRST topic: ticks naming another topic, or the default, "
        "still publish on it; anything but \"gravity\" is freeze (true)\n"
     << "recorders (" << r_first.qosText() << ", made before the ticks): " << first << ", " << second << ", " << dflt << "\n";
  m26::Semantic sem;
  sem.ack.push_back("none (fire-and-forget topic)");
  for (std::size_t i = 0; i < trees.size(); ++i)
  {
    m26::watchdog().arm("REL-01 tick " + std::to_string(i + 1), std::chrono::seconds(30));
    const std::string bt = m26::tickTree(trees[i], &rc->ctx, node);
    // every tick publishes ONE sample somewhere: wait until the first recorder has i+1, or 2 s
    r_first.settle(i + 1, m26::milliseconds(2000), m26::milliseconds(200));
    m26::watchdog().disarm();
    os << "--- tick " << (i + 1) << ": " << trees[i] << "\nbt: " << bt << "\n";
    sem.command.push_back(m26::treeLeaf(trees[i]) + "(" + trees[i].substr(18, trees[i].size() - 20) + ")");
    sem.bt.push_back("tick " + std::to_string(i + 1) + "=" + bt);
  }
  r_second.settle(0, m26::milliseconds(1000));
  r_default.settle(0, m26::milliseconds(1000));
  const auto pubs = [&](const std::string& t) { return std::to_string(node->count_publishers(t)); };
  os << "received on " << first << ": " << r_first.byPublisher() << "\n"
     << "received on " << second << ": " << r_second.byPublisher() << "\n"
     << "received on " << dflt << ": " << r_default.byPublisher() << "\n"
     << "publishers in the graph: " << first << "=" << pubs(first) << " " << second << "=" << pubs(second) << " "
     << dflt << "=" << pubs(dflt) << "\n";
  sem.state.push_back(first + ": " + r_first.byPublisher());
  sem.state.push_back(second + ": " + r_second.byPublisher());
  sem.state.push_back(dflt + ": " + r_default.byPublisher());
  sem.grasp.push_back(m26::boolStateSources(node, node->get_name(), node->get_namespace()));
  sem.release.push_back("every release policy landed on " + first + ": [" + m26::join(r_first.values()) + "]");
  os << sem.line() << "\n";
  m26::Family f("eef_release_policy");
  f.add(os.str());
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
