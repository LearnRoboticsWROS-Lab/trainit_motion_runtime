// vNext M2.6 (tests BEFORE, ADR-0022 family E3): eef_check_robot_state -- CheckRobotState's
// require_attached, the ONE grasp verification primitive TrainIt has today (ADR-0022 F10), frozen on
// production 1dda503 before the canonical EEF contract touches a production line
// (test/data/m2_6/README.md).
//
// What is pinned (bt_nodes.cpp CheckRobotState): only the exact string "true" turns the check on
// ("True" and "1" SKIP it, i.e. SUCCESS); no attached_topic is a FAILURE before any wait; the feed
// is a process-static BEST_EFFORT VOLATILE KeepLast(1) subscription, so a latched provider's sample
// never reaches it; and only a sample ARRIVING DURING the tick counts (the fresh-arrival rule), so a
// provider that publishes on change cannot satisfy it. Each scenario uses its own topic, because the
// feeds are cached per topic name for the life of the process.
#include <memory>
#include <string>
#include <vector>

#include "m2_6/gripper_scenarios.hpp"

namespace
{
enum class Provide { NONE, PERIODIC, ONCE_LATCHED_BEFORE };

struct CrsScenario
{
  std::string id;
  std::string pins;
  std::string topic;                  // the scenario's attached topic ("" = the port left empty)
  Provide provide{Provide::NONE};
  bool value{true};                   // what the provider publishes
  std::vector<std::string> trees;     // CheckRobotState ticks, in order
  // between tick 1 and tick 2: stop the periodic provider, publish ONE sample, let it arrive
  bool one_sample_between_ticks{false};
};

bool contains(const std::vector<std::string>& v, const std::string& s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

std::string run(const CrsScenario& s)
{
  auto node = m26::treeNode().node();
  auto rc = m26::makeRunnerContext(node, m26::RunnerKeys{});
  std::unique_ptr<m26::BoolProvider> provider;
  if (s.provide != Provide::NONE)
    provider = std::make_unique<m26::BoolProvider>("m26_provider" + m26::tagOf(s.topic), "/", s.topic,
                                                   s.provide == Provide::ONCE_LATCHED_BEFORE);
  std::ostringstream os;
  os << "=== scenario " << s.id << "\n" << "pins: " << s.pins << "\n";
  if (provider)
  {
    if (s.provide == Provide::PERIODIC)
    {
      provider->startPeriodic(s.value, m26::milliseconds(50));
      os << "provider: " << provider->name() << " (" << provider->qosText() << "): " << m26::boolText(s.value)
         << " every 50 ms, started before the tick\n";
    }
    else
    {
      provider->publish(s.value);
      std::this_thread::sleep_for(m26::milliseconds(300));
      os << "provider: " << provider->name() << " (" << provider->qosText() << "): ONE " << m26::boolText(s.value)
         << " published before the tick, then silent\n";
    }
  }
  else
  {
    os << "provider: none\n";
  }
  m26::Semantic sem;
  sem.command.push_back("none (a state check sends nothing)");
  sem.ack.push_back("none");
  for (std::size_t i = 0; i < s.trees.size(); ++i)
  {
    if (i == 1 && s.one_sample_between_ticks && provider)
    {
      provider->stopPeriodic();
      std::this_thread::sleep_for(m26::milliseconds(300));
      provider->publish(s.value);
      std::this_thread::sleep_for(m26::milliseconds(500));   // it arrives at the feed BEFORE tick 2
      os << "between the ticks: the periodic provider stopped, then ONE " << m26::boolText(s.value)
         << " was published and had 500 ms to arrive\n";
    }
    m26::watchdog().arm(s.id + " tick " + std::to_string(i + 1), std::chrono::seconds(30));
    const auto t0 = m26::Clock::now();
    const std::string bt = m26::tickTree(s.trees[i], &rc->ctx, node);
    const auto t1 = m26::Clock::now();
    m26::watchdog().disarm();
    const auto subs = m26::nodeTopics(node, node->get_name(), node->get_namespace(), "std_msgs/msg/Bool", true);
    const bool subscribed = !s.topic.empty() && contains(subs, s.topic);
    const auto during = provider ? provider->publishedBetween(t0, t1) : std::vector<bool>{};
    std::vector<std::string> during_s;
    for (bool b : during) during_s.push_back(m26::boolText(b));
    os << "--- tick " << (i + 1) << ": " << s.trees[i] << "\n"
       << "bt: " << bt << "\n"
       << "the tree node subscribes to " << (s.topic.empty() ? std::string("(no topic)") : s.topic) << " after the tick: "
       << (subscribed ? "yes" : "no") << "\n";
    const std::string tick = "tick " + std::to_string(i + 1);
    // a check that made no subscription read nothing: what the provider did meanwhile is not observed
    sem.state.push_back(tick + ": " + (!subscribed ? std::string(provider ? "provider not read" : "no provider")
                                       : "provider published during the tick: " +
                                           (during_s.empty() ? std::string("nothing") : m26::join(during_s))));
    sem.grasp.push_back(tick + ": " + (subscribed ? "checked through a Bool subscription on " + s.topic
                                                  : std::string("no subscription made (no check reached the topic)")));
    sem.bt.push_back(tick + ": CheckRobotState=" + bt);
    if (bt == "FAILURE")
      sem.timeout.push_back(tick + ": " + (!subscribed ? std::string("none (failed before any wait)")
                                           : during.empty() ? std::string("attached wait expired (nothing arrived during the tick)")
                                                            : std::string("none (a sample arrived and was not true)")));
  }
  if (provider) provider->stopPeriodic();
  os << sem.line() << "\n";
  return os.str();
}

std::string check(const std::string& require, const std::string& topic, const std::string& timeout_ms = "")
{
  std::string x = "<CheckRobotState require_attached=\"" + require + "\" attached_topic=\"" + topic + "\"";
  if (!timeout_ms.empty()) x += " timeout_ms=\"" + timeout_ms + "\"";
  return x + "/>";
}
}  // namespace

TEST(M26CheckRobotState, eef_check_robot_state)
{
  const std::vector<CrsScenario> scenarios{
    {"CRS-01", "require_attached=\"true\" with no attached_topic: FAILURE before any wait", "", Provide::NONE, true,
     {check("true", "")}},
    {"CRS-02", "a topic nobody publishes, timeout_ms 300: FAILURE when the wait expires", "/m26_crs02/attached",
     Provide::NONE, true, {check("true", "/m26_crs02/attached", "300")}},
    {"CRS-03", "a provider publishing true every 50 ms: SUCCESS on the first fresh sample", "/m26_crs03/attached",
     Provide::PERIODIC, true, {check("true", "/m26_crs03/attached")}},
    {"CRS-04", "a provider publishing false every 50 ms: FAILURE (object NOT attached)", "/m26_crs04/attached",
     Provide::PERIODIC, false, {check("true", "/m26_crs04/attached")}},
    {"CRS-05", "a LATCHED provider that published true ONCE before the tick: its sample never reaches the volatile "
               "feed -- FAILURE when the wait expires", "/m26_crs05/attached", Provide::ONCE_LATCHED_BEFORE, true,
     {check("true", "/m26_crs05/attached", "500")}},
    {"CRS-06", "the fresh-arrival rule: tick 1 succeeds on a periodic provider; a true that ARRIVED before tick 2 "
               "does not count -- tick 2 FAILS when its wait expires", "/m26_crs06/attached", Provide::PERIODIC, true,
     {check("true", "/m26_crs06/attached"), check("true", "/m26_crs06/attached", "500")}, true},
    {"CRS-07", "require_attached=\"True\" (not the exact \"true\"): the check is SKIPPED -- SUCCESS although the "
               "provider says false", "/m26_crs07/attached", Provide::PERIODIC, false,
     {check("True", "/m26_crs07/attached")}},
    {"CRS-08", "require_attached=\"1\": SKIPPED too -- SUCCESS although the provider says false",
     "/m26_crs08/attached", Provide::PERIODIC, false, {check("1", "/m26_crs08/attached")}},
  };
  m26::Family f("eef_check_robot_state");
  for (const auto& s : scenarios) f.add(run(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
