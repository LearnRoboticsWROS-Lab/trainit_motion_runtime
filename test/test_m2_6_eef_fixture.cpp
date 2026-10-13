// vNext M2.6 AFTER (ADR-0022), M4: the R1/R2-like synthetic fixture (composite_tool: 6 actuators
// on 3 interfaces, 2 sites) ticked by REAL BT nodes through the REAL EndEffectorRegistry, in /r1
// (robot_a_* joints) and /r2 (robot_b_* joints, IDENTICAL eef/actuator/site ids) AT ONCE, against
// in-process fakes (test/m2_6/eef_after_fakes.hpp: ScriptedSetBool, the BEFORE harness's
// ScriptedGripper, ScriptedTrajectoryServer, a FakeEefAdapter that publishes the state by the
// adapter's rules). Domain 97, localhost only, graph-guarded (m26::runAll).
//
//   P1  representable: every actuator reached on its standard interface, the goals exact
//   P2  (TMR's half) every endpoint the runner reaches is in its own namespace
//   P3  actuator failure != grasp failure      P4  command success != grasp
//   P5  aggregation and correlation (FAULT, BUSY, LOST and its clearing, a rejected release)
//   P6  the same ids on two robots: WRONG_EEF, the command cross-remap, two writers
//   P7  the in-tool handoff                    P8  the guard
//   P10 the refusal half; the mock-level -5 naming the joint; a one-joint goal REJECTED
//   the switch confirmation and ACTUATION_NOT_CONFIRMED; the scene signal TMR publishes (and does
//   not, for an adapter or none device); the no-cancel rule for a third-party GripperCommand server;
//   the must-reach rule without the runner's own adapter and with a silent one; the waits (a SetBool
//   with no answer, a late accept, CommandActuator's timeout_ms, FJT -1 with no state, a shut-down
//   context); the start-up refusals one by one; an unreadable blackboard port; an unobservable site.
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "m2_6/capture26.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "m2_6/eef_after_runner.hpp"
#include "m2_6/eef_fakes.hpp"

namespace
{
namespace eef = trainit::eef;
using m26::after::Act;
using m26::after::CanonicalRunner;
using m26::after::FakeEefAdapter;
using m26::after::SetBoolReply;
using m26::after::SiteScript;
using m26::after::State;
using m26::after::TickResult;
using m26::after::TrajectoryReply;
using m26::GripperReply;
using m26::milliseconds;

const char* kTool = "composite_tool";

struct Robot
{
  std::string ns;
  std::string prefix;   // the robot's TF prefix: robot_a_ / robot_b_
  eef::Contract contract;
  std::unique_ptr<FakeEefAdapter> adapter;
  std::unique_ptr<CanonicalRunner> runner;
  std::string id(const std::string& tail = "") const { return ns + "/eef/composite_tool" + tail; }
};

class Rig
{
public:
  Rig()
  {
    for (auto* r : {&r1, &r2})
    {
      r->ns = r == &r1 ? "/r1" : "/r2";
      r->prefix = r == &r1 ? "robot_a_" : "robot_b_";
      r->contract = m26::after::onlyDevices(m26::after::fixtureContract(r->prefix), {kTool});
      r->adapter = std::make_unique<FakeEefAdapter>(log, r->ns, *r->contract.device(kTool));
      r->runner = std::make_unique<CanonicalRunner>(r->ns, r->contract);
    }
  }
  // Back to the start: both tools empty and idle, every script default, exactly one state writer,
  // and both runners holding a fresh sample.
  void reset()
  {
    for (auto* r : {&r1, &r2})
    {
      r->adapter->reset();
      r->adapter->resetGripper("gripper_a", {});
      EXPECT_TRUE(m26::waitUntil([&] { return r->runner->device(kTool).statePublishers() == 1; }, milliseconds(6000)))
        << r->ns << ": not exactly one state writer";
      const auto seq = r->runner->device(kTool).latest().seq;
      EXPECT_TRUE(r->runner->waitForState(kTool, milliseconds(6000), seq + 3)) << r->ns << ": no fresh state";
    }
  }
  m26::Log log;
  Robot r1, r2;
};

Rig& rig()
{
  static std::unique_ptr<Rig> rig;   // one per process (test harness): built on first use
  if (!rig)
  {
    rig = std::make_unique<Rig>();
    m26::teardowns().push_back([] { rig.reset(); });
  }
  return *rig;
}

// Both robots tick `xml` at the same time.
std::pair<TickResult, TickResult> tickBoth(Rig& r, const std::string& xml)
{
  auto f1 = std::async(std::launch::async, [&] { return r.r1.runner->tick(xml); });
  auto f2 = std::async(std::launch::async, [&] { return r.r2.runner->tick(xml); });
  return {f1.get(), f2.get()};
}

std::string close(const std::string& site, const std::string& extra = "")
{
  return "<CloseGripper eef=\"composite_tool\" site=\"" + site + "\" " + extra + "/>";
}
std::string open(const std::string& site, const std::string& extra = "")
{
  return "<OpenGripper eef=\"composite_tool\" site=\"" + site + "\" " + extra + "/>";
}
std::string actuate(const std::string& a, const std::string& t, const std::string& extra = "")
{
  return "<CommandActuator eef=\"composite_tool\" actuator=\"" + a + "\" target=\"" + t + "\" " + extra + "/>";
}
std::string verify(const std::string& site, const std::string& expect, const std::string& extra = "")
{
  return "<VerifyGrasp eef=\"composite_tool\" " + (site.empty() ? std::string() : "site=\"" + site + "\" ") +
         "expect=\"" + expect + "\" " + extra + "/>";
}
const char* kOutputs = "reason=\"{r}\" fault=\"{f}\" subject=\"{s}\"";
std::string seq(const std::vector<std::string>& children)
{
  std::string s = "<Sequence>";
  for (const auto& c : children) s += c;
  return s + "</Sequence>";
}

// Waits until `runner` HOLDS a sample in which `site` reads `grasp` (a scripted change has reached it).
bool seen(CanonicalRunner& runner, const std::string& site, uint8_t grasp, uint8_t evidence)
{
  return m26::waitUntil([&] {
    const auto s = runner.device(kTool).latest().sample;
    if (!s) return false;
    for (std::size_t i = 0; i < s->sites.size(); ++i)
      if (s->sites[i] == site) return s->site_grasp[i] == grasp && s->site_evidence[i] == evidence;
    return false;
  }, milliseconds(2000));
}

struct Guard
{
  explicit Guard(const std::string& what, int seconds = 120) { m26::watchdog().arm(what, std::chrono::seconds(seconds)); }
  ~Guard() { m26::watchdog().disarm(); }
};
}  // namespace

// --- P1 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P1_every_actuator_is_representable_and_reached_on_its_standard_interface)
{
  Guard g("P1");
  auto& r = rig();
  r.reset();
  for (auto* robot : {&r.r1, &r.r2})
  {
    auto& d = robot->runner->device(kTool);
    EXPECT_EQ(d.identity(), robot->id());
    EXPECT_EQ(d.stateTopic(), robot->id("/state"));
    const auto snap = d.latest();
    ASSERT_TRUE(snap.sample);
    const State& s = *snap.sample;
    EXPECT_EQ(s.end_effector, robot->id());
    std::vector<std::string> ids;
    std::vector<uint8_t> ifaces;
    for (const auto& a : s.actuators)
    {
      ids.push_back(a.actuator);
      ifaces.push_back(a.command_interface);
    }
    EXPECT_EQ(ids, (std::vector<std::string>{"linear_axis", "rotary_axis", "wrist_mechanism", "gripper_a", "gripper_b", "binary_lock"}));
    EXPECT_EQ(ifaces, (std::vector<uint8_t>{3, 3, 3, 2, 1, 1}));
    EXPECT_EQ(s.actuators[2].joints, (std::vector<std::string>{robot->prefix + "wrist_pitch_joint", robot->prefix + "wrist_roll_joint"}));
    EXPECT_EQ(s.site_grip, (std::vector<std::string>{"gripper_a", "gripper_b"}));
  }
  // every actuator, once, on BOTH robots at the same time
  const std::string all = seq({close("site_a"), close("site_b"), actuate("linear_axis", "extended"),
                               actuate("rotary_axis", "transfer"), actuate("wrist_mechanism", "present"),
                               actuate("binary_lock", "engage"), open("site_b"), open("site_a"),
                               actuate("binary_lock", "release"), actuate("linear_axis", "home")});
  const auto [t1, t2] = tickBoth(r, all);
  EXPECT_EQ(t1.status, "SUCCESS");
  EXPECT_EQ(t2.status, "SUCCESS");
  for (auto* robot : {&r.r1, &r.r2})
  {
    auto& fa = *robot->adapter;
    const auto ga = fa.gripper("gripper_a").goals();
    ASSERT_EQ(ga.size(), 2u) << robot->ns;
    EXPECT_EQ(ga[0].position, 0.0);    // closed
    EXPECT_EQ(ga[0].max_effort, 20.0); // the declared effort, never 0.0 on an effort-flavour server
    EXPECT_EQ(ga[1].position, 0.02);   // open
    const auto gb = fa.setBool("gripper_b").calls();
    ASSERT_EQ(gb.size(), 2u);
    EXPECT_TRUE(gb[0].data);
    EXPECT_FALSE(gb[1].data);
    const auto lock = fa.setBool("binary_lock").calls();
    ASSERT_EQ(lock.size(), 2u);
    const auto lin = fa.trajectory("linear_axis").goals();
    ASSERT_EQ(lin.size(), 2u);
    EXPECT_EQ(lin[0].joints, (std::vector<std::string>{robot->prefix + "linear_axis_joint"}));
    EXPECT_EQ(lin[0].positions, (std::vector<double>{0.08}));
    EXPECT_EQ(lin[1].positions, (std::vector<double>{0.0}));
    EXPECT_EQ(lin[0].points, 1u);
    EXPECT_TRUE(lin[0].start_now);
    EXPECT_EQ(lin[0].velocities + lin[0].accelerations + lin[0].effort, 0u) << "positions only: the JTC rejects effort";
    EXPECT_DOUBLE_EQ(lin[0].time_from_start_s, 1.0);
    EXPECT_DOUBLE_EQ(lin[0].goal_time_tolerance_s, 0.5);
    ASSERT_EQ(lin[0].goal_tolerance.size(), 1u);
    EXPECT_EQ(lin[0].goal_tolerance[0].first, robot->prefix + "linear_axis_joint");
    EXPECT_DOUBLE_EQ(lin[0].goal_tolerance[0].second, 0.002);
    EXPECT_TRUE(lin[0].path_tolerance.empty());
    const auto rot = fa.trajectory("rotary_axis").goals();
    ASSERT_EQ(rot.size(), 1u);
    ASSERT_EQ(rot[0].path_tolerance.size(), 1u);
    EXPECT_DOUBLE_EQ(rot[0].path_tolerance[0].second, 0.2);
    const auto wrist = fa.trajectory("wrist_mechanism").goals();
    ASSERT_EQ(wrist.size(), 1u);
    EXPECT_EQ(wrist[0].joints.size(), 2u);   // ONE goal for the two-joint mechanism
    EXPECT_EQ(wrist[0].positions, (std::vector<double>{0.3, -0.3}));
    EXPECT_DOUBLE_EQ(wrist[0].time_from_start_s, 1.5);
  }
  // the node reads its contract from its parameter OVERRIDES, as trainit_run_bt does
  const auto fixture = m26::after::loadParams(m26::after::contractFile("eef_conformance.yaml"));
  std::vector<rclcpp::Parameter> overrides;
  for (const auto& [k, v] : fixture) overrides.emplace_back(k, v);
  auto node = std::make_shared<rclcpp::Node>("m26_overrides_probe", "/r9",
                                             rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(overrides)
                                               .automatically_declare_parameters_from_overrides(true));
  node->declare_parameter("gripper_cmd_topic", std::string());   // a runner declares legacy keys with defaults ...
  EXPECT_EQ(eef::parseContract(*node).dump(), m26::after::readFile(m26::after::contractFile("eef_conformance.expected.txt")))
    << "... and a declared default is never taken for a key the project set";
}

// --- P2 (TMR's half) --------------------------------------------------------------------------------
TEST(M26EefFixture, P2_every_endpoint_the_runner_reaches_is_in_its_own_namespace)
{
  Guard g("P2");
  auto& r = rig();
  r.reset();
  auto probe = std::make_shared<rclcpp::Node>("m26_p2_probe", "/", rclcpp::NodeOptions().use_global_arguments(false));
  // the SetBool servers each runner's clients match, per namespace, and none outside one
  std::map<std::string, int> set_engaged;
  for (const auto& [name, types] : probe->get_service_names_and_types())
    if (name.find("/set_engaged") != std::string::npos) ++set_engaged[name];
  for (const char* ns : {"/r1", "/r2"})
    for (const char* a : {"gripper_b", "binary_lock"})
      EXPECT_EQ(set_engaged[std::string(ns) + "/eef/composite_tool/actuators/" + a + "/set_engaged"], 1) << ns << " " << a;
  EXPECT_EQ(set_engaged.size(), 4u) << "an EEF service outside /r1 and /r2";
  // a late joiner on /r1's state receives exactly one sample, /r1's
  int got = 0;
  std::string who;
  auto sub = probe->create_subscription<State>("/r1/eef/composite_tool/state",
                                               rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                                               [&](std::shared_ptr<const State> m) {
                                                 if (got++ == 0) who = m->end_effector;
                                               });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(probe);
  const auto until = m26::Clock::now() + milliseconds(3000);
  while (got == 0 && m26::Clock::now() < until) ex.spin_some(milliseconds(20));
  EXPECT_EQ(who, "/r1/eef/composite_tool");
  // the action servers too, by their status topics
  std::set<std::string> actions;
  for (const auto& [name, types] : probe->get_topic_names_and_types())
    if (name.size() > 15 && name.substr(name.size() - 15) == "/_action/status") actions.insert(name);
  for (const char* ns : {"/r1", "/r2"})
    for (const char* c : {"linear_axis_controller/follow_joint_trajectory", "rotary_axis_controller/follow_joint_trajectory",
                          "wrist_mechanism_controller/follow_joint_trajectory", "gripper_a_controller/gripper_cmd"})
      EXPECT_TRUE(actions.count(std::string(ns) + "/" + c + "/_action/status")) << ns << "/" << c;
}

// --- P3 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P3_an_actuator_failure_is_not_a_grasp_failure)
{
  Guard g("P3");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  // gripper_b answers success:false: CloseGripper fails with the ACTUATOR's reason; VerifyGrasp is
  // never reached
  const uint8_t last_b = fa.lastCommand("gripper_b");
  fa.setBool("gripper_b").script({SetBoolReply::reject("no stage subscriber")});
  const auto t = r.r1.runner->tick(seq({close("site_b", kOutputs), "<Script code=\"verified := 1\"/>", verify("site_b", "held")}));
  EXPECT_EQ(t.status, "FAILURE");
  EXPECT_EQ(t.get("r"), "COMMAND_REJECTED");
  EXPECT_EQ(t.get("f"), "COMMAND_REJECTED");
  EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/actuators/gripper_b");
  EXPECT_EQ(eef::classOf(eef::Reason::COMMAND_REJECTED), eef::ReasonClass::ACTUATOR_FAILURE);
  EXPECT_FALSE(t.has("verified")) << "VerifyGrasp was reached after a failed command";
  // the rejection is named in the device's detail, and NOT recorded as an applied command
  EXPECT_TRUE(m26::waitUntil([&] { return fa.latestPublished().detail.find("gripper_b: ENGAGE rejected") != std::string::npos; },
                             milliseconds(2000)));
  EXPECT_EQ(fa.lastCommand("gripper_b"), last_b) << "a rejected command is not an applied one";

  // linear_axis ABORTED -5: GOAL_TOLERANCE_VIOLATED about the axis; the sites do not change
  const auto before = fa.latestPublished();
  fa.trajectory("linear_axis").script({TrajectoryReply::abort(-5, "Aborted due to goal tolerance violation")});
  const auto a = r.r1.runner->tick(actuate("linear_axis", "extended", kOutputs));
  EXPECT_EQ(a.status, "FAILURE");
  EXPECT_EQ(a.get("r"), "GOAL_TOLERANCE_VIOLATED");
  EXPECT_EQ(a.get("s"), "/r1/eef/composite_tool/actuators/linear_axis");
  EXPECT_EQ(fa.latestPublished().site_grasp, before.site_grasp);

  // in contrast: gripper_a SUCCEEDED with site_a NOT_HELD is a VERDICT about the SITE, and `fault`
  // stays unset
  fa.setSite("site_a", SiteScript{State::GRASP_NOT_HELD});
  const auto v = r.r1.runner->tick(seq({close("site_a", kOutputs), verify("site_a", "held", kOutputs)}));
  EXPECT_EQ(v.status, "FAILURE");
  EXPECT_EQ(v.get("r"), "NO_OBJECT");
  EXPECT_EQ(v.get("s"), "/r1/eef/composite_tool/sites/site_a");
  EXPECT_EQ(v.get("f"), "") << "a VERDICT never writes fault";
}

// --- P4 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P4_command_success_is_not_a_grasp)
{
  Guard g("P4");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  // an EMPTY close: the jaw reaches its goal (SUCCEEDED, reached_goal) and nothing is held
  fa.setSite("site_a", SiteScript{State::GRASP_NOT_HELD});
  const auto empty = r.r1.runner->tick(seq({close("site_a"), "<Script code=\"closed := 1\"/>", verify("site_a", "held", kOutputs)}));
  EXPECT_EQ(empty.status, "FAILURE");
  EXPECT_TRUE(empty.has("closed")) << "CloseGripper is SUCCESS: actuation only";
  EXPECT_EQ(empty.get("r"), "NO_OBJECT");
  EXPECT_EQ(fa.gripper("gripper_a").goals().back().reply.reached_goal, true);
  // HELD by ACTUATION_INFERENCE: not accepted by default ([sensor])
  fa.setSite("site_a", SiteScript{State::GRASP_HELD, State::GRASP_NOT_HELD, State::EVIDENCE_ACTUATION_INFERENCE});
  const auto inferred = r.r1.runner->tick(seq({close("site_a"), verify("site_a", "held", kOutputs)}));
  EXPECT_EQ(inferred.get("r"), "EVIDENCE_NOT_ACCEPTED");
  EXPECT_EQ(inferred.get("f"), "EVIDENCE_NOT_ACCEPTED");
  // ... and SUCCESS, labelled as inference, only with the opt-in (a project that lists it)
  auto params = m26::after::loadParams(m26::after::contractFile("eef_conformance.yaml"));
  params["eef.composite_tool.verification.accepted_evidence"] =
    rclcpp::ParameterValue(std::vector<std::string>{"sensor", "actuation_inference"});
  {
    CanonicalRunner opted("/r1", m26::after::onlyDevices(eef::parseContract(params), {kTool}), {}, "trainit_run_bt_opt_in");
    ASSERT_TRUE(opted.waitForState(kTool, milliseconds(6000)));
    const auto res = opted.registry().verifyGrasp("VerifyGrasp", kTool, "site_a", "held", std::nullopt, std::nullopt, "");
    EXPECT_TRUE(res.ok) << eef::toToken(res.reason) << " " << res.detail;
    EXPECT_TRUE(res.inferred) << "the success says: confirmed by inference";
    // `accept` may only NARROW: accept="sensor" refuses the inference again
    const auto narrowed = opted.registry().verifyGrasp("VerifyGrasp", kTool, "site_a", "held", std::nullopt, std::nullopt, "sensor");
    EXPECT_EQ(narrowed.reason, eef::Reason::EVIDENCE_NOT_ACCEPTED);
    // ... and cannot widen
    const auto widened = r.r1.runner->registry().verifyGrasp("VerifyGrasp", kTool, "site_a", "held", std::nullopt, std::nullopt,
                                                             "actuation_inference");
    EXPECT_EQ(widened.reason, eef::Reason::CONTRACT_MISMATCH);
  }
  // PENDING from an unaccepted class: EVIDENCE_NOT_ACCEPTED, never a wait for a verdict
  fa.setSite("site_a", SiteScript{State::GRASP_PENDING, State::GRASP_NOT_HELD, State::EVIDENCE_ACTUATION_INFERENCE});
  EXPECT_EQ(r.r1.runner->tick(seq({close("site_a"), verify("site_a", "held", kOutputs)})).get("r"), "EVIDENCE_NOT_ACCEPTED");
  // LOST from an unaccepted class: EVIDENCE_NOT_ACCEPTED, not OBJECT_LOST
  fa.setSite("site_a", SiteScript{State::GRASP_HELD, State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR});
  ASSERT_EQ(r.r1.runner->tick(seq({close("site_a"), verify("site_a", "held")})).status, "SUCCESS");
  fa.observe("site_a", State::GRASP_LOST, State::EVIDENCE_ACTUATION_INFERENCE);
  ASSERT_TRUE(seen(*r.r1.runner, "site_a", State::GRASP_LOST, State::EVIDENCE_ACTUATION_INFERENCE));
  EXPECT_EQ(r.r1.runner->tick(verify("site_a", "held", kOutputs)).get("r"), "EVIDENCE_NOT_ACCEPTED");
  // COMMAND_ECHO can never be accepted (TSA refuses it; the parser refuses it: the runner exits 2)
  fa.observe("site_a", State::GRASP_HELD, State::EVIDENCE_COMMAND_ECHO);
  ASSERT_TRUE(seen(*r.r1.runner, "site_a", State::GRASP_HELD, State::EVIDENCE_COMMAND_ECHO));
  EXPECT_EQ(r.r1.runner->tick(verify("site_a", "held", kOutputs)).get("r"), "EVIDENCE_NOT_ACCEPTED");
}

// --- P5 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P5_the_tool_state_aggregates_and_correlates_the_actuators)
{
  Guard g("P5");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  auto& runner = *r.r1.runner;
  // healthy actuators, a site NOT_HELD: READY
  EXPECT_EQ(fa.latestPublished().status, State::STATUS_READY);
  // binary_lock FAULT: the tool is FAULT, detail names it, the sites do not change, and a
  // VerifyGrasp of site_a is a FAULT about binary_lock -- not a verdict
  ASSERT_EQ(runner.tick(close("site_a")).status, "SUCCESS");
  fa.setActuatorStatus("binary_lock", Act::STATUS_FAULT, "FAULT (scripted)");
  ASSERT_TRUE(m26::waitUntil([&] { return fa.latestPublished().status == State::STATUS_FAULT; }, milliseconds(1000)));
  EXPECT_NE(fa.latestPublished().detail.find("binary_lock"), std::string::npos);
  EXPECT_TRUE(m26::waitUntil([&] { return fa.siteGrasp("site_a") == State::GRASP_HELD; }, milliseconds(1000)))
    << "an actuator FAULT never changes a site";
  const auto fault = runner.tick(verify("site_a", "held", kOutputs));
  EXPECT_EQ(fault.get("r"), "HARDWARE_FAULT");
  EXPECT_EQ(fault.get("f"), "HARDWARE_FAULT");
  EXPECT_EQ(fault.get("s"), "/r1/eef/composite_tool/actuators/binary_lock");
  fa.setActuatorStatus("binary_lock", Act::STATUS_IDLE);
  // an axis MOVING: BUSY -- and a site can still be verified meanwhile
  fa.trajectory("linear_axis").script({TrajectoryReply::succeed(1500)});
  auto moving = std::async(std::launch::async, [&] { return runner.tick(actuate("linear_axis", "extended")); });
  EXPECT_TRUE(m26::waitUntil([&] {
    const auto s = runner.device(kTool).latest().sample;
    return s && s->status == State::STATUS_BUSY;
  }, milliseconds(1400)));
  const auto busy = runner.registry().verifyGrasp("VerifyGrasp", kTool, "site_a", "held", std::nullopt, std::nullopt, "");
  EXPECT_TRUE(busy.ok) << eef::toToken(busy.reason) << " " << busy.detail;
  EXPECT_EQ(moving.get().status, "SUCCESS");
  // HELD -> NOT_HELD after an APPLIED release of gripper_a: NOT_HELD (released), not LOST
  EXPECT_EQ(runner.tick(seq({open("site_a"), verify("site_a", "released")})).status, "SUCCESS");
  EXPECT_EQ(fa.siteGrasp("site_a"), State::GRASP_NOT_HELD);
  // the same edge with NO command: LOST, latched ...
  ASSERT_EQ(runner.tick(seq({close("site_a"), verify("site_a", "held")})).status, "SUCCESS");
  fa.drop("site_a");
  ASSERT_TRUE(seen(runner, "site_a", State::GRASP_LOST, State::EVIDENCE_SENSOR));
  EXPECT_EQ(runner.tick(verify("site_a", "held", kOutputs)).get("r"), "OBJECT_LOST");
  // ... a REJECTED release does not disarm it, and is named in detail ...
  fa.resetGripper("gripper_a", {GripperReply::reject()});
  EXPECT_EQ(runner.tick(open("site_a", kOutputs)).get("r"), "COMMAND_REJECTED");
  EXPECT_TRUE(m26::waitUntil([&] { return fa.latestPublished().detail.find("gripper_a: RELEASE rejected") != std::string::npos; },
                             milliseconds(2000)));
  EXPECT_EQ(fa.siteGrasp("site_a"), State::GRASP_LOST);
  // ... and the next APPLIED engage clears it
  EXPECT_EQ(runner.tick(seq({close("site_a"), verify("site_a", "held")})).status, "SUCCESS");
  // an actuator set that differs from the contract: CONTRACT_MISMATCH
  m26::after::AdapterOptions o;
  o.serve = false;
  o.node_name = "m26_fake_eef_adapter_mismatch";
  o.published_actuators = {"linear_axis", "rotary_axis", "wrist_mechanism", "gripper_a", "gripper_b", "lock"};
  {
    FakeEefAdapter odd(r.log, "/r7", *r.r1.contract.device(kTool), o);
    CanonicalRunner r7("/r7", r.r1.contract);
    ASSERT_TRUE(r7.waitForState(kTool, milliseconds(6000)));
    const auto m = r7.registry().verifyGrasp("VerifyGrasp", kTool, "site_a", "held", std::nullopt, std::nullopt, "");
    EXPECT_EQ(m.reason, eef::Reason::CONTRACT_MISMATCH) << m.detail;
    EXPECT_EQ(m.subject, "/r7/eef/composite_tool");
  }
}

// --- P6 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P6_the_same_ids_on_two_robots)
{
  Guard g("P6", 180);
  auto& r = rig();
  r.reset();
  // VerifyGrasp in /r1 with its state remapped to /r2's: WRONG_EEF
  {
    CanonicalRunner remapped("/r1", r.r1.contract, {"eef/composite_tool/state:=/r2/eef/composite_tool/state"},
                             "trainit_run_bt_state_remap");
    ASSERT_TRUE(remapped.waitForState(kTool, milliseconds(6000)));
    EXPECT_EQ(remapped.device(kTool).stateTopic(), "/r2/eef/composite_tool/state");
    const auto t = remapped.tick(verify("site_b", "held", kOutputs));
    EXPECT_EQ(t.get("r"), "WRONG_EEF");
    EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool");
  }
  // CloseGripper in /r1 with its SetBool client remapped to /r2's: CONTRACT_MISMATCH, robot_b's
  // last_command changes, and no NO_OBJECT
  {
    CanonicalRunner remapped("/r1", r.r1.contract,
                             {"eef/composite_tool/actuators/gripper_b/set_engaged:=/r2/eef/composite_tool/actuators/gripper_b/set_engaged"},
                             "trainit_run_bt_command_remap");
    ASSERT_TRUE(remapped.waitForState(kTool, milliseconds(6000)));
    r.r2.adapter->setSite("site_b", SiteScript{State::GRASP_NOT_HELD});
    const auto r1_calls = r.r1.adapter->setBool("gripper_b").calls().size();
    const auto r2_calls = r.r2.adapter->setBool("gripper_b").calls().size();
    const auto t = remapped.tick(seq({close("site_b", kOutputs), verify("site_b", "held", kOutputs)}));
    EXPECT_EQ(t.status, "FAILURE");
    EXPECT_EQ(t.get("r"), "CONTRACT_MISMATCH");
    EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/actuators/gripper_b");
    EXPECT_NE(t.get("r"), "NO_OBJECT");
    EXPECT_EQ(r.r2.adapter->setBool("gripper_b").calls().size(), r2_calls + 1) << "the command reached robot_b";
    EXPECT_EQ(r.r2.adapter->lastCommand("gripper_b"), Act::COMMAND_ENGAGE);
    EXPECT_EQ(r.r1.adapter->setBool("gripper_b").calls().size(), r1_calls) << "and never robot_a";
  }
  r.reset();
  // CommandActuator linear_axis in /r1 changes only the /r1 entry
  const auto r2_goals = r.r2.adapter->trajectory("linear_axis").goals().size();
  const auto r2_last = r.r2.adapter->lastCommand("linear_axis");
  ASSERT_EQ(r.r1.runner->tick(actuate("linear_axis", "extended")).status, "SUCCESS");
  EXPECT_EQ(r.r1.adapter->lastCommand("linear_axis"), Act::COMMAND_TARGET);
  EXPECT_EQ(r.r2.adapter->trajectory("linear_axis").goals().size(), r2_goals);
  EXPECT_EQ(r.r2.adapter->lastCommand("linear_axis"), r2_last);
  // two adapters writing one device's state: CONTRACT_MISMATCH, at VerifyGrasp and at start-up
  {
    m26::after::AdapterOptions o;
    o.serve = false;
    o.node_name = "m26_second_writer";
    FakeEefAdapter second(r.log, "/r1", *r.r1.contract.device(kTool), o);
    ASSERT_TRUE(m26::waitUntil([&] { return r.r1.runner->device(kTool).statePublishers() == 2; }, milliseconds(6000)));
    const auto t = r.r1.runner->tick(verify("site_a", "released", kOutputs));
    EXPECT_EQ(t.get("r"), "CONTRACT_MISMATCH");
    EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool");
    EXPECT_EQ(r.r1.runner->registry().checkStateWriters().size(), 1u);
  }
}

// --- one-device contracts for the cases below ---------------------------------------------------------
namespace
{
// One-device contracts (synthetic names): a set_bool `vacuum` with its command_timeout_ms; a
// GripperCommand jaw with a third, non-grip target `half`; an axis with no TrainIt adapter.
std::string valveYaml(const std::string& id, int command_timeout_ms)
{
  return "/**:\n  ros__parameters:\n    eef:\n      ids: [\"" + id + "\"]\n      " + id + ":\n        actuator_ids: [\"vacuum\"]\n"
         "        site_ids: [\"main\"]\n        command_timeout_ms: " + std::to_string(command_timeout_ms) + "\n"
         "        scene_signal: \"none\"\n        actuators:\n          vacuum: {command_interface: \"set_bool\"}\n"
         "        sites:\n          main: {grip: \"vacuum\", engage: \"engage\", release: \"release\"}\n"
         "        verification:\n          require_grasp_confirmation: false\n          require_release_confirmation: false\n";
}
std::string jawYaml(const std::string& id)
{
  return "/**:\n  ros__parameters:\n    eef:\n      ids: [\"" + id + "\"]\n      " + id + ":\n        actuator_ids: [\"gripper\"]\n"
         "        site_ids: [\"main\"]\n        scene_signal: \"none\"\n        actuators:\n"
         "          gripper:\n            command_interface: \"gripper_command\"\n            action: \"" + id + "_controller/gripper_cmd\"\n"
         "            joints: [\"finger_joint\"]\n            targets: {open: 0.0, closed: 0.7, half: 0.35}\n"
         "            result_policy: \"succeeded_or_stalled\"\n        sites:\n          main: {grip: \"gripper\", engage: \"closed\", release: \"open\"}\n"
         "        verification:\n          require_grasp_confirmation: false\n          require_release_confirmation: false\n";
}
std::string axisYaml(const std::string& id)
{
  return "/**:\n  ros__parameters:\n    eef:\n      ids: [\"" + id + "\"]\n      " + id + ":\n        actuator_ids: [\"slide\", \"clamp\"]\n"
         "        site_ids: [\"main\"]\n        scene_signal: \"none\"\n        actuators:\n          slide:\n"
         "            command_interface: \"follow_joint_trajectory\"\n            action: \"slide_controller/follow_joint_trajectory\"\n"
         "            joints: [\"robot_f_slide_joint\"]\n            targets: {home: [0.0], out: [0.05]}\n"
         "            goal_tolerance: 0.001\n            duration_s: 0.2\n          clamp: {command_interface: \"set_bool\"}\n"
         "        sites:\n          main: {grip: \"clamp\", engage: \"engage\", release: \"release\"}\n"
         "        verification:\n          require_grasp_confirmation: false\n          require_release_confirmation: false\n";
}
int64_t msSince(m26::Clock::time_point t0)
{
  return std::chrono::duration_cast<milliseconds>(m26::Clock::now() - t0).count();
}
}  // namespace

// ADR-0022 D2, "the command must reach THIS device", holds for EVERY set_bool actuator whatever the
// graph shows: with the runner's own adapter absent, another robot's SetBool success:true is never
// DONE -- no sample of the device's own state ever showed it (STATE_MISSING, about the device).
TEST(M26EefFixture, P6_a_cross_remapped_set_bool_with_no_own_adapter_is_never_SUCCESS)
{
  Guard g("P6 no own adapter");
  auto& r = rig();
  r.reset();
  CanonicalRunner lone("/r6", r.r1.contract,
                       {"eef/composite_tool/actuators/gripper_b/set_engaged:=/r2/eef/composite_tool/actuators/gripper_b/set_engaged"},
                       "trainit_run_bt_no_adapter");
  EXPECT_EQ(lone.device(kTool).statePublishers(), std::optional<std::size_t>(0)) << "no adapter in /r6";
  const auto r2_calls = r.r2.adapter->setBool("gripper_b").calls().size();
  const auto t = lone.tick(seq({close("site_b", kOutputs), "<Script code=\"closed := 1\"/>"}));
  EXPECT_EQ(t.status, "FAILURE");
  EXPECT_FALSE(t.has("closed")) << "a command that never reached THIS device is never DONE";
  EXPECT_EQ(t.get("r"), "STATE_MISSING");
  EXPECT_EQ(t.get("f"), "STATE_MISSING");
  EXPECT_EQ(t.get("s"), "/r6/eef/composite_tool");
  EXPECT_EQ(r.r2.adapter->setBool("gripper_b").calls().size(), r2_calls + 1) << "the command reached robot_b";
  EXPECT_EQ(r.r2.adapter->lastCommand("gripper_b"), Act::COMMAND_ENGAGE);
  EXPECT_TRUE(lone.registry().faultLatched());
}

// The same split for a GripperCommand actuator while a TrainIt adapter is visible: a server that is
// NOT the device's adapter answers the goal (SUCCEEDED), and the adapter keeps publishing samples
// none of which shows it -- CONTRACT_MISMATCH, never a silent DONE. (An action name cannot be
// cross-remapped on Humble, so the foreign server stands at the device's own action name.)
TEST(M26EefFixture, P6_a_gripper_goal_another_server_answers_is_CONTRACT_MISMATCH)
{
  Guard g("P6 foreign gripper server");
  m26::Log log;
  const auto jaw = m26::after::contractFromYaml(jawYaml("jaw_foreign"), "m26_jaw_foreign");
  m26::after::AdapterOptions o;
  o.serve = false;   // the device's adapter publishes its state and serves nothing
  FakeEefAdapter fa(log, "/r17", *jaw.device("jaw_foreign"), o);
  m26::ScriptedGripper foreign(log, "/r17", "jaw_foreign_controller/gripper_cmd");
  CanonicalRunner runner("/r17", jaw);
  ASSERT_TRUE(runner.waitForState("jaw_foreign", milliseconds(6000)));
  const auto t = runner.tick(std::string("<CloseGripper ") + kOutputs + "/>");
  EXPECT_EQ(t.status, "FAILURE");
  EXPECT_EQ(t.get("r"), "CONTRACT_MISMATCH");
  EXPECT_EQ(t.get("s"), "/r17/eef/jaw_foreign/actuators/gripper");
  ASSERT_EQ(foreign.goals().size(), 1u);
  EXPECT_EQ(foreign.goals()[0].terminal, "SUCCEEDED") << "the foreign server succeeded: actuation elsewhere";
  EXPECT_EQ(fa.lastCommand("gripper"), Act::COMMAND_NONE) << "and the device's own state never showed it";
}

// A visible adapter that went silent is not a remap: no NEW sample since the command is STATE_STALE
// ("the state publisher sent nothing since the command"), for a set_bool, a GripperCommand and a
// FollowJointTrajectory actuator alike; with actuation_feedback the reflection still comes first.
TEST(M26EefFixture, a_paused_adapter_is_STATE_STALE_at_the_command_not_CONTRACT_MISMATCH)
{
  Guard g("paused adapter");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  auto& runner = *r.r1.runner;
  fa.pause(true);
  ASSERT_TRUE(runner.waitForQuiet(kTool, milliseconds(200), milliseconds(3000)));
  const std::vector<std::pair<std::string, std::string>> cases{
    {close("site_b", kOutputs), "/r1/eef/composite_tool/actuators/gripper_b"},
    {close("site_a", kOutputs), "/r1/eef/composite_tool/actuators/gripper_a"},
    {actuate("linear_axis", "extended", kOutputs), "/r1/eef/composite_tool/actuators/linear_axis"}};
  for (const auto& [xml, subject] : cases)
  {
    const auto t = runner.tick(xml);
    EXPECT_EQ(t.status, "FAILURE") << xml;
    EXPECT_EQ(t.get("r"), "STATE_STALE") << xml;
    EXPECT_EQ(t.get("s"), subject) << xml;
  }
  const auto lock = runner.registry().actuate("CommandActuator", kTool, "binary_lock", "engage", std::nullopt);
  EXPECT_EQ(lock.reason, eef::Reason::STATE_STALE) << lock.detail << ": the reflection before the switch";
  EXPECT_NE(lock.detail.find("the state publisher sent nothing since the command"), std::string::npos) << lock.detail;
  fa.pause(false);
}

// --- P7 and P8 --------------------------------------------------------------------------------------
TEST(M26EefFixture, P7_the_in_tool_handoff_on_both_robots_at_once)
{
  Guard g("P7");
  auto& r = rig();
  r.reset();
  // ADR-0022 D2's HandoffAtoB (without the arm's clearance move: no motion runtime here)
  const std::string handoff = seq({close("site_a"), verify("site_a", "held"),
                                   actuate("rotary_axis", "transfer"), actuate("linear_axis", "extended"),
                                   close("site_b"), verify("site_b", "held"),   // guard: site_a opens only if site_b holds
                                   open("site_a"), verify("site_a", "released"),
                                   verify("", "held"),                          // tool level: held through site_b
                                   actuate("linear_axis", "home")});
  const auto [t1, t2] = tickBoth(r, handoff);
  EXPECT_EQ(t1.status, "SUCCESS");
  EXPECT_EQ(t2.status, "SUCCESS");
  for (auto* robot : {&r.r1, &r.r2})
  {
    EXPECT_EQ(robot->adapter->siteGrasp("site_a"), State::GRASP_NOT_HELD) << robot->ns;
    EXPECT_EQ(robot->adapter->siteGrasp("site_b"), State::GRASP_HELD) << robot->ns;
    const auto res = robot->runner->registry().verifyGrasp("VerifyGrasp", kTool, "", "held", std::nullopt, std::nullopt, "");
    EXPECT_TRUE(res.ok);
    EXPECT_EQ(res.subject, robot->id("/sites/site_b")) << "held through site_b; site_a, released, is skipped";
  }
}

TEST(M26EefFixture, P8_the_guard_stops_the_sequence_before_the_other_jaw_opens)
{
  Guard g("P8");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  fa.setSite("site_b", SiteScript{State::GRASP_NOT_HELD});
  const auto t = r.r1.runner->tick(seq({close("site_a"), verify("site_a", "held"), close("site_b"),
                                        verify("site_b", "held", kOutputs), open("site_a")}));
  EXPECT_EQ(t.status, "FAILURE");
  EXPECT_EQ(t.get("r"), "NO_OBJECT");
  EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/sites/site_b");
  EXPECT_EQ(fa.gripper("gripper_a").goals().size(), 1u) << "gripper_a received no goal after the failed guard";
  EXPECT_EQ(fa.lastCommand("gripper_a"), Act::COMMAND_ENGAGE);
  EXPECT_EQ(fa.siteGrasp("site_a"), State::GRASP_HELD) << "the tool holds (HOLD, EEF-D11)";
}

// --- P10 ---------------------------------------------------------------------------------------------
TEST(M26EefFixture, P10_axis_refusals_and_tolerance_outcomes)
{
  Guard g("P10");
  // the refusal half: no tolerance or no duration is refused (the runner exits 2)
  for (const char* key : {"goal_tolerance", "duration_s"})
  {
    auto params = m26::after::loadParams(m26::after::contractFile("eef_conformance.yaml"));
    params.erase(std::string("eef.composite_tool.actuators.linear_axis.") + key);
    EXPECT_THROW(eef::parseContract(params), eef::Refused) << key;
  }
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  // within tolerance: SUCCEEDED; one joint of the two-joint mechanism off: the WHOLE goal -5,
  // subject the mechanism, the failing joint named from the server's feedback
  fa.trajectory("wrist_mechanism").script({TrajectoryReply::abort(-5, "Aborted due to goal tolerance violation").withFeedback({0.0, 0.05})});
  const auto t = r.r1.runner->tick(actuate("wrist_mechanism", "present", kOutputs));
  EXPECT_EQ(t.get("r"), "GOAL_TOLERANCE_VIOLATED");
  EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/actuators/wrist_mechanism");
  const auto direct = r.r1.runner->registry().actuate("CommandActuator", kTool, "wrist_mechanism", "stow", std::nullopt);
  EXPECT_TRUE(direct.ok) << direct.detail;
  fa.trajectory("wrist_mechanism").script({TrajectoryReply::abort(-5, "").withFeedback({0.0, 0.05})});
  const auto named = r.r1.runner->registry().actuate("CommandActuator", kTool, "wrist_mechanism", "present", std::nullopt);
  EXPECT_EQ(named.reason, eef::Reason::GOAL_TOLERANCE_VIOLATED);
  EXPECT_NE(named.detail.find("robot_a_wrist_roll_joint (error 0.05, tolerance 0.01)"), std::string::npos) << named.detail;
  EXPECT_EQ(named.detail.find("robot_a_wrist_pitch_joint (error"), std::string::npos) << named.detail;
  // -4 is the path tolerance; CANCELED by a second client is never a success
  fa.trajectory("rotary_axis").script({TrajectoryReply::abort(-4, "Aborted due to path tolerance violation")});
  EXPECT_EQ(r.r1.runner->tick(actuate("rotary_axis", "transfer", kOutputs)).get("r"), "PATH_TOLERANCE_VIOLATED");
  // a goal naming only one of the mechanism's two joints is REJECTED by the controller
  const auto mech = m26::after::contractFromYaml(
    "/**:\n  ros__parameters:\n    eef:\n      ids: [\"mech\"]\n      mech:\n        actuator_ids: [\"wrist_mechanism\", \"clamp\"]\n"
    "        site_ids: [\"main\"]\n        scene_signal: \"none\"\n        actuators:\n          wrist_mechanism:\n"
    "            command_interface: \"follow_joint_trajectory\"\n            action: \"wrist_mechanism_controller/follow_joint_trajectory\"\n"
    "            joints: [\"robot_e_wrist_pitch_joint\"]\n            targets: {present: [0.3]}\n            goal_tolerance: 0.01\n"
    "            duration_s: 1.0\n          clamp: {command_interface: \"set_bool\"}\n        sites:\n"
    "          main: {grip: \"clamp\", engage: \"engage\", release: \"release\"}\n        verification:\n"
    "          require_grasp_confirmation: false\n          require_release_confirmation: false\n");
  m26::Log log;
  m26::after::ScriptedTrajectoryServer jtc(log, "/r5", "wrist_mechanism_controller/follow_joint_trajectory",
                                           {"robot_e_wrist_pitch_joint", "robot_e_wrist_roll_joint"}, {});
  CanonicalRunner r5("/r5", mech);
  const auto one = r5.registry().actuate("CommandActuator", "mech", "wrist_mechanism", "present", std::nullopt);
  EXPECT_EQ(one.reason, eef::Reason::COMMAND_REJECTED) << one.detail;
  ASSERT_EQ(jtc.goals().size(), 1u);
  EXPECT_EQ(jtc.goals()[0].response, "REJECT");
}

// --- the switch confirmation ----------------------------------------------------------------------------
TEST(M26EefFixture, the_switch_confirms_a_binary_actuator_or_ACTUATION_NOT_CONFIRMED)
{
  Guard g("switch");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  EXPECT_EQ(r.r1.runner->tick(actuate("binary_lock", "engage", kOutputs)).status, "SUCCESS");
  EXPECT_TRUE(fa.latestPublished().actuators[5].binary_confirmed);
  fa.setSwitchConfirms("binary_lock", false);
  const auto t0 = m26::Clock::now();
  const auto t = r.r1.runner->tick(actuate("binary_lock", "release", kOutputs));
  const auto waited = std::chrono::duration_cast<milliseconds>(m26::Clock::now() - t0).count();
  EXPECT_EQ(t.get("r"), "ACTUATION_NOT_CONFIRMED");
  EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/actuators/binary_lock");
  EXPECT_GE(waited, 1900) << "within confirm_timeout_ms (2000)";
  // a grip actuator's engage/release targets are refused by CommandActuator
  const auto grip = r.r1.runner->registry().actuate("CommandActuator", kTool, "gripper_b", "engage", std::nullopt);
  EXPECT_EQ(grip.reason, eef::Reason::UNKNOWN_TARGET);
  // ... and a literal one is refused at start-up (the runner exits 2)
  EXPECT_EQ(r.r1.runner->tick(actuate("gripper_a", "closed")).status.rfind("REFUSED AT START-UP", 0), 0u);
  EXPECT_EQ(r.r1.runner->tick(actuate("linear_axis", "nowhere")).status.rfind("REFUSED AT START-UP", 0), 0u);
  EXPECT_EQ(r.r1.runner->tick(close("site_z")).status.rfind("REFUSED AT START-UP", 0), 0u);
  EXPECT_EQ(r.r1.runner->tick("<SetGripper position=\"0.5\"/>").status.rfind("REFUSED AT START-UP", 0), 0u);
}

// --- the scene signal and the cancel rules (single-site jaws) -------------------------------------------
TEST(M26EefFixture, the_scene_signal_follows_a_DONE_command_when_TMR_is_its_source)
{
  Guard g("scene signal");
  const auto jaw = m26::after::onlyDevices(m26::after::fixtureContract(), {"jaw_simple"});
  m26::Log log;
  FakeEefAdapter fa(log, "/r3", *jaw.device("jaw_simple"));
  CanonicalRunner runner("/r3", jaw);
  // LATCHED, like the runner's writer: whenever the two match, the recorder receives the newest
  // sample, and each command is ticked alone and its sample awaited -- no match timing matters
  m26::BoolRecorder rec("m26_rec_r3_engaged", "/r3/eef/jaw_simple/engaged", m26::BoolRecorder::Qos::LATCHED);
  ASSERT_TRUE(runner.waitForState("jaw_simple", milliseconds(6000)));
  EXPECT_EQ(runner.tick("<CloseGripper/>").status, "SUCCESS");
  rec.settle(1, milliseconds(6000));
  EXPECT_EQ(rec.values(), (std::vector<std::string>{"true"}));
  EXPECT_EQ(runner.tick("<OpenGripper/>").status, "SUCCESS");
  rec.settle(2, milliseconds(6000));
  EXPECT_EQ(rec.values(), (std::vector<std::string>{"true", "false"}));
  // a failed command publishes nothing
  fa.resetGripper("gripper", {GripperReply::reject()});
  EXPECT_EQ(runner.tick("<CloseGripper/>").status, "FAILURE");
  rec.settle(2, milliseconds(1000));
  EXPECT_EQ(rec.values().size(), 2u);
}

TEST(M26EefFixture, a_third_party_gripper_goal_is_never_cancelled_a_mock_one_is)
{
  Guard g("cancel rules");
  auto jaw_yaml = [](const std::string& id) {
    return "/**:\n  ros__parameters:\n    eef:\n      ids: [\"" + id + "\"]\n      " + id + ":\n        actuator_ids: [\"gripper\"]\n"
           "        site_ids: [\"main\"]\n        command_timeout_ms: 1500\n        scene_signal: \"tmr\"\n        actuators:\n"
           "          gripper:\n            command_interface: \"gripper_command\"\n            action: \"" + id + "_controller/gripper_cmd\"\n"
           "            joints: [\"finger_joint\"]\n            targets: {open: 0.0, closed: 0.7}\n"
           "            result_policy: \"succeeded_or_stalled\"\n        sites:\n          main: {grip: \"gripper\", engage: \"closed\", release: \"open\"}\n"
           "        verification:\n          require_grasp_confirmation: false\n          require_release_confirmation: false\n";
  };
  // a ros2_control-like server: no TrainIt state on the device
  m26::Log log;
  const auto third = m26::after::contractFromYaml(jaw_yaml("jaw_ros2"));
  m26::ScriptedGripper gac(log, "/r4", "jaw_ros2_controller/gripper_cmd", {GripperReply::withhold()});
  CanonicalRunner r4("/r4", third);
  const auto t = r4.registry().grip("CloseGripper", "jaw_ros2", "", true);
  EXPECT_EQ(t.reason, eef::Reason::COMMAND_TIMEOUT) << t.detail;
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_EQ(gac.cancelRequests(), 0) << "a ros2_control GripperCommand goal is NEVER cancelled";
  // the TrainIt mock (a state publisher on the device): the goal IS cancelled
  const auto mock = m26::after::contractFromYaml(jaw_yaml("jaw_mock"));
  FakeEefAdapter fa(log, "/r4", *mock.device("jaw_mock"));
  fa.resetGripper("gripper", {GripperReply::withhold()});
  CanonicalRunner r4m("/r4", mock, {}, "trainit_run_bt_mock");
  ASSERT_TRUE(r4m.waitForState("jaw_mock", milliseconds(6000)));
  const auto m = r4m.registry().grip("CloseGripper", "jaw_mock", "", true);
  EXPECT_EQ(m.reason, eef::Reason::COMMAND_TIMEOUT) << m.detail;
  EXPECT_TRUE(m26::waitUntil([&] { return fa.gripper("gripper").cancelRequests() == 1; }, milliseconds(3000)));
}

// scene_signal adapter (or none): the ADAPTER is the signal's source, so TMR never publishes
// eef/<id>/engaged, not even on a DONE command.
TEST(M26EefFixture, TMR_publishes_no_scene_signal_for_an_adapter_or_none_device)
{
  Guard g("no scene signal");
  const auto suction = m26::after::onlyDevices(m26::after::fixtureContract(), {"suction_simple"});
  m26::Log log;
  FakeEefAdapter fa(log, "/r14", *suction.device("suction_simple"));   // its state, never the signal
  CanonicalRunner runner("/r14", suction);
  m26::BoolRecorder rec("m26_rec_r14_engaged", "/r14/eef/suction_simple/engaged", m26::BoolRecorder::Qos::LATCHED);
  ASSERT_TRUE(runner.waitForState("suction_simple", milliseconds(6000)));
  EXPECT_FALSE(runner.device("suction_simple").tmrPublishesSceneSignal());
  EXPECT_EQ(runner.tick("<Sequence><CloseGripper/><OpenGripper/></Sequence>").status, "SUCCESS");
  rec.settle(1, milliseconds(1500));
  EXPECT_EQ(rec.count(), 0u) << "TMR published the scene signal of an adapter-sourced device";
  EXPECT_EQ(runner.node()->count_publishers("/r14/eef/suction_simple/engaged"), 0u);
  // scene_signal none (the multi-site composite_tool): no publisher either
  auto& r = rig();
  EXPECT_FALSE(r.r1.runner->device(kTool).tmrPublishesSceneSignal());
  EXPECT_EQ(r.r1.runner->node()->count_publishers("/r1/eef/composite_tool/engaged"), 0u);
}

// --- the waits, at node level ------------------------------------------------------------------------
// A SetBool that never answers: COMMAND_TIMEOUT within the device's command_timeout_ms, the pending
// request dropped; applied but unanswered, the same reason -- and the STATE shows that it applied.
TEST(M26EefFixture, a_SetBool_with_no_answer_is_COMMAND_TIMEOUT_within_command_timeout_ms)
{
  Guard g("SetBool timeouts");
  const auto valve = m26::after::contractFromYaml(valveYaml("valve", 1200), "m26_valve");
  m26::Log log;
  FakeEefAdapter fa(log, "/r10", *valve.device("valve"));
  CanonicalRunner runner("/r10", valve);
  ASSERT_TRUE(runner.waitForState("valve", milliseconds(6000)));
  fa.setBool("vacuum").script({SetBoolReply::noAnswer(), SetBoolReply::appliedUnanswered()});
  const auto t0 = m26::Clock::now();
  const auto none = runner.registry().grip("CloseGripper", "valve", "", true);
  const auto waited = msSince(t0);
  EXPECT_EQ(none.reason, eef::Reason::COMMAND_TIMEOUT) << none.detail;
  EXPECT_EQ(none.subject, "/r10/eef/valve/actuators/vacuum");
  EXPECT_GE(waited, 1200);
  EXPECT_LT(waited, 1200 + 1500) << "within command_timeout_ms (1200), not the 10 s default";
  EXPECT_EQ(fa.lastCommand("vacuum"), Act::COMMAND_NONE) << "not applied";
  const auto applied = runner.registry().grip("CloseGripper", "valve", "", true);
  EXPECT_EQ(applied.reason, eef::Reason::COMMAND_TIMEOUT) << applied.detail;
  EXPECT_NE(applied.detail.find("the state shows whether it applied"), std::string::npos) << applied.detail;
  EXPECT_EQ(fa.lastCommand("vacuum"), Act::COMMAND_ENGAGE) << "applied, never answered";
  EXPECT_TRUE(m26::waitUntil([&] {
    const auto s = runner.device("valve").latest().sample;
    return s && !s->actuators.empty() && s->actuators[0].last_command == Act::COMMAND_ENGAGE;
  }, milliseconds(2000))) << "the device's state shows the applied engage";
  // the dropped requests leave no trace: the next command is answered and DONE
  const auto next = runner.registry().grip("CloseGripper", "valve", "", true);
  EXPECT_TRUE(next.ok) << eef::toToken(next.reason) << " " << next.detail;
  EXPECT_EQ(fa.setBool("vacuum").calls().size(), 3u);
}

// ADR-0022 TMR-5: the parser accepts any positive int64 in an ms key, as the reference does; the
// registry saturates every one before a chrono or nanosecond conversion. Unsaturated, 1e13 ms
// wraps negative: every SetBool would end COMMAND_TIMEOUT at once, every VerifyGrasp STATE_STALE.
TEST(M26EefFixture, a_huge_millisecond_value_is_saturated_never_overflowed)
{
  Guard g("saturated ms");
  const std::string huge = "10000000000000";   // "practically forever"
  auto yaml = valveYaml("valve_big", 1200);
  const std::string cto = "command_timeout_ms: 1200\n";
  yaml.replace(yaml.find(cto), cto.size(), "command_timeout_ms: " + huge + "\n");
  const std::string vac = "vacuum: {command_interface: \"set_bool\"}";
  yaml.replace(yaml.find(vac), vac.size(), "vacuum: {command_interface: \"set_bool\", confirm_timeout_ms: " + huge + "}");
  yaml += "          timeout_ms: " + huge + "\n          max_age_ms: " + huge + "\n          clock_tolerance_ms: " + huge + "\n";
  const auto big = m26::after::contractFromYaml(yaml, "m26_valve_big");
  ASSERT_EQ(big.device("valve_big")->command_timeout_ms, 10000000000000LL);
  m26::Log log;
  FakeEefAdapter fa(log, "/r18", *big.device("valve_big"));
  CanonicalRunner runner("/r18", big);
  ASSERT_TRUE(runner.waitForState("valve_big", milliseconds(6000)));
  const auto t = runner.tick(seq({"<CloseGripper/>", "<VerifyGrasp expect=\"held\"/>", "<OpenGripper/>",
                                  "<VerifyGrasp expect=\"released\"/>"}));
  EXPECT_EQ(t.status, "SUCCESS");
  EXPECT_FALSE(runner.registry().faultLatched());
  EXPECT_EQ(fa.setBool("vacuum").calls().size(), 2u);
}

// A goal accepted after the 5 s goal-response wait: SEND_TIMEOUT, and the late goal is cancelled
// exactly once while a TrainIt adapter serves it -- never on a ros2_control server (no state).
TEST(M26EefFixture, a_late_accepted_goal_is_SEND_TIMEOUT_and_cancelled_exactly_once)
{
  Guard g("late accept");
  m26::Log log;
  const auto mock = m26::after::contractFromYaml(jawYaml("jaw_late"), "m26_jaw_late");
  FakeEefAdapter fa(log, "/r11", *mock.device("jaw_late"));
  fa.resetGripper("gripper", {GripperReply::withhold().acceptAfter(5500)});
  CanonicalRunner runner("/r11", mock);
  ASSERT_TRUE(runner.waitForState("jaw_late", milliseconds(6000)));
  const auto t0 = m26::Clock::now();
  const auto late = runner.registry().grip("CloseGripper", "jaw_late", "", true);
  const auto waited = msSince(t0);
  EXPECT_EQ(late.reason, eef::Reason::SEND_TIMEOUT) << late.detail;
  EXPECT_GE(waited, 5000);
  EXPECT_LT(waited, 5000 + 2500);
  EXPECT_TRUE(m26::waitUntil([&] { return fa.gripper("gripper").cancelRequests() == 1; }, milliseconds(4000)))
    << "the late-accepted goal is cancelled";
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_EQ(fa.gripper("gripper").cancelRequests(), 1) << "exactly once";
  EXPECT_EQ(fa.gripper("gripper").goals().size(), 1u);
  // a ros2_control-like server: no TrainIt state on the device, so the late goal is left alone
  const auto third = m26::after::contractFromYaml(jawYaml("jaw_late_ros2"), "m26_jaw_late_ros2");
  m26::ScriptedGripper gac(log, "/r11", "jaw_late_ros2_controller/gripper_cmd", {GripperReply::withhold().acceptAfter(5500)});
  CanonicalRunner r11c("/r11", third, {}, "trainit_run_bt_ros2");
  EXPECT_EQ(r11c.registry().grip("CloseGripper", "jaw_late_ros2", "", true).reason, eef::Reason::SEND_TIMEOUT);
  ASSERT_TRUE(m26::waitUntil([&] { return !gac.goals().empty() && !gac.goals()[0].response.empty(); }, milliseconds(4000)));
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_EQ(gac.goals()[0].response, "ACCEPT_AND_EXECUTE") << "accepted late";
  EXPECT_EQ(gac.cancelRequests(), 0) << "a ros2_control GripperCommand goal is NEVER cancelled";
}

// CommandActuator's timeout_ms replaces the device's command_timeout_ms (10 s) for that command.
TEST(M26EefFixture, CommandActuator_timeout_ms_replaces_command_timeout_ms)
{
  Guard g("timeout_ms");
  m26::Log log;
  const auto jaw = m26::after::contractFromYaml(jawYaml("jaw_half"), "m26_jaw_half");
  FakeEefAdapter fa(log, "/r12", *jaw.device("jaw_half"));
  fa.resetGripper("gripper", {GripperReply::succeed(), GripperReply::withhold()});
  CanonicalRunner runner("/r12", jaw);
  ASSERT_TRUE(runner.waitForState("jaw_half", milliseconds(6000)));
  const std::string half = "<CommandActuator eef=\"jaw_half\" actuator=\"gripper\" target=\"half\" ";
  ASSERT_EQ(runner.tick(half + "/>").status, "SUCCESS") << "a named target, reflected as TARGET";
  const auto t0 = m26::Clock::now();
  const auto t = runner.tick(half + "timeout_ms=\"300\" " + kOutputs + "/>");
  const auto waited = msSince(t0);
  EXPECT_EQ(t.get("r"), "COMMAND_TIMEOUT");
  EXPECT_GE(waited, 300);
  EXPECT_LT(waited, 1300) << "the port's 300 ms, not command_timeout_ms (10000)";
  EXPECT_TRUE(m26::waitUntil([&] { return fa.gripper("gripper").cancelRequests() == 1; }, milliseconds(3000)))
    << "the TrainIt mock's goal is cancelled at the deadline";
}

// FollowJointTrajectory ABORTED -1 from a server with no TrainIt state on the device (a
// joint_trajectory_controller, deactivated): ACTUATOR_UNAVAILABLE; from the TrainIt mock,
// COMMAND_ABORTED with its text.
TEST(M26EefFixture, FJT_minus_1_from_a_server_with_no_state_is_ACTUATOR_UNAVAILABLE)
{
  Guard g("FJT -1");
  m26::Log log;
  const auto axis = m26::after::contractFromYaml(axisYaml("slide_tool"), "m26_slide_tool");
  m26::after::ScriptedTrajectoryServer jtc(log, "/r13", "slide_controller/follow_joint_trajectory", {"robot_f_slide_joint"}, {});
  jtc.script({TrajectoryReply::abort(-1, "")});
  CanonicalRunner r13("/r13", axis);
  const auto res = r13.registry().actuate("CommandActuator", "slide_tool", "slide", "out", std::nullopt);
  EXPECT_EQ(res.reason, eef::Reason::ACTUATOR_UNAVAILABLE) << res.detail;
  EXPECT_EQ(res.subject, "/r13/eef/slide_tool/actuators/slide");
  ASSERT_EQ(jtc.goals().size(), 1u);
  EXPECT_EQ(jtc.goals()[0].terminal, "ABORTED");
  auto& r = rig();
  r.reset();
  r.r1.adapter->trajectory("linear_axis").script({TrajectoryReply::abort(-1, "mock: refused")});
  const auto mock = r.r1.runner->registry().actuate("CommandActuator", kTool, "linear_axis", "extended", std::nullopt);
  EXPECT_EQ(mock.reason, eef::Reason::COMMAND_ABORTED) << mock.detail;
  EXPECT_NE(mock.detail.find("mock: refused"), std::string::npos) << mock.detail;
}

// The start-up checks (the runner exits with code 2), rule by rule, before any tick.
TEST(M26EefFixture, the_start_up_refusals_one_by_one)
{
  Guard g("start-up refusals");
  auto& runner = *rig().r1.runner;
  const std::vector<std::pair<std::string, std::string>> cases{
    {"<CloseGripper eef=\"composite_tool\"/>", "composite_tool has several sites: name one with the site port"},
    {"<OpenGripper eef=\"composite_tool\"/>", "composite_tool has several sites: name one with the site port"},
    {"<CloseGripper eef=\"composite_tool\" site=\"\"/>", "composite_tool has several sites: name one with the site port"},
    {"<CommandActuator eef=\"composite_tool\" actuator=\"nozzle\" target=\"open\"/>",
     "actuator=\"nozzle\": no such actuator of composite_tool"},
    {"<CommandActuator eef=\"composite_tool\" target=\"extended\"/>", "the actuator port is required"},
    {"<CommandActuator eef=\"composite_tool\" actuator=\"linear_axis\"/>", "the target port is required"},
    {"<VerifyGrasp eef=\"composite_tool\" site=\"site_a\"/>", "the expect port is required (held | released)"},
    {"<CommandActuator eef=\"composite_tool\" actuator=\"linear_axis\" target=\"extended\" timeout_ms=\"-300\"/>",
     "timeout_ms=\"-300\": a positive integer"},
    {"<VerifyGrasp eef=\"composite_tool\" site=\"site_a\" expect=\"held\" timeout_ms=\"1.5\"/>",
     "timeout_ms=\"1.5\": a positive integer"},
    {"<VerifyGrasp eef=\"composite_tool\" site=\"site_a\" expect=\"held\" max_age_ms=\"0\"/>",
     "max_age_ms=\"0\": a positive integer"},
    {"<VerifyGrasp eef=\"composite_tool\" site=\"site_a\" expect=\"held\" accept=\"sensor;actuation_inference\"/>",
     "'actuation_inference' is not in composite_tool's accepted_evidence (accept may only narrow it)"},
    {"<VerifyGrasp eef=\"composite_tool\" site=\"site_a\" expect=\"held\" accept=\";\"/>", "names no evidence class"},
  };
  for (const auto& [xml, problem] : cases)
  {
    const auto t = runner.tick(xml);
    EXPECT_EQ(t.status.rfind("REFUSED AT START-UP", 0), 0u) << xml << " -> " << t.status;
    EXPECT_NE(t.status.find(problem), std::string::npos) << xml << " -> " << t.status;
  }
  // a literal an int port cannot convert at all is refused earlier still, when the tree is built
  // (the runner's "failed to load BT", exit 2 too)
  const auto fast = runner.tick("<CommandActuator eef=\"composite_tool\" actuator=\"linear_axis\" target=\"extended\" "
                                "timeout_ms=\"fast\"/>");
  EXPECT_NE(fast.status.find("THROW The port with name \"timeout_ms\" and value \"fast\" can not be converted to int"),
            std::string::npos) << fast.status;
  // no eef port while the contract has several devices
  CanonicalRunner all("/r15", m26::after::fixtureContract(), {}, "trainit_run_bt_all_devices");
  const auto t = all.tick("<CloseGripper site=\"main\"/>");
  EXPECT_NE(t.status.find("no eef port, and the contract has several devices "
                          "[composite_tool, suction_simple, jaw_simple, jaw_binary]"), std::string::npos)
    << t.status;
}

// A blackboard port that cannot be read is a port error -- CONTRACT_MISMATCH about the node --
// never "no site" (tool level) or "no narrowing"; a key that holds a value is read as that value.
TEST(M26EefFixture, an_unreadable_blackboard_string_port_is_a_port_error)
{
  Guard g("blackboard ports");
  auto& r = rig();
  r.reset();
  auto& fa = *r.r1.adapter;
  auto& runner = *r.r1.runner;
  // ADR-0022 D2's handoff guard, hand-written, its key never written: site_a HELD, site_b NOT --
  // read as tool level, it would succeed "held through site_a"
  fa.setSite("site_b", SiteScript{State::GRASP_NOT_HELD});
  ASSERT_EQ(runner.tick(seq({close("site_a"), verify("site_a", "held"), close("site_b")})).status, "SUCCESS");
  const auto guard = runner.tick("<VerifyGrasp eef=\"composite_tool\" site=\"{guard_site}\" expect=\"held\" " +
                                 std::string(kOutputs) + "/>");
  EXPECT_EQ(guard.status, "FAILURE");
  EXPECT_EQ(guard.get("r"), "CONTRACT_MISMATCH");
  EXPECT_NE(guard.get("s").find("VerifyGrasp"), std::string::npos) << guard.get("s") << ": the node is the subject";
  const auto accept = runner.tick("<VerifyGrasp eef=\"composite_tool\" site=\"site_a\" expect=\"held\" accept=\"{strict}\" " +
                                  std::string(kOutputs) + "/>");
  EXPECT_EQ(accept.get("r"), "CONTRACT_MISMATCH") << "never the project's full accepted_evidence";
  // a command node sends nothing
  const auto b_calls = fa.setBool("gripper_b").calls().size();
  const auto a_goals = fa.gripper("gripper_a").goals().size();
  EXPECT_EQ(runner.tick("<OpenGripper eef=\"composite_tool\" site=\"{s}\" " + std::string(kOutputs) + "/>").get("r"),
            "CONTRACT_MISMATCH");
  EXPECT_EQ(runner.tick("<CommandActuator eef=\"{e}\" actuator=\"linear_axis\" target=\"extended\" " +
                        std::string(kOutputs) + "/>").get("r"), "CONTRACT_MISMATCH");
  EXPECT_EQ(fa.setBool("gripper_b").calls().size(), b_calls);
  EXPECT_EQ(fa.gripper("gripper_a").goals().size(), a_goals);
  // written, the same key is read as its value
  EXPECT_EQ(runner.tick(seq({"<Script code=\"guard_site := 'site_a'\"/>",
                             "<VerifyGrasp eef=\"composite_tool\" site=\"{guard_site}\" expect=\"held\"/>"})).status,
            "SUCCESS");
  const auto b = runner.tick(seq({"<Script code=\"guard_site := 'site_b'\"/>",
                                  "<VerifyGrasp eef=\"composite_tool\" site=\"{guard_site}\" expect=\"held\" " +
                                    std::string(kOutputs) + "/>"}));
  EXPECT_EQ(b.get("r"), "NO_OBJECT") << "the guard judges site_b, never the tool";
  EXPECT_EQ(b.get("s"), "/r1/eef/composite_tool/sites/site_b");
}

// Check 5 against the adapters' stamp convention (a site with no deciding source is UNAVAILABLE /
// NONE stamped 0, never fresh): VerifyGrasp fails at once, never STATE_STALE at its deadline.
TEST(M26EefFixture, an_unobservable_site_is_VERIFICATION_UNAVAILABLE_at_once)
{
  Guard g("unobservable site");
  auto& r = rig();
  r.reset();
  SiteScript blind;
  blind.observable = false;
  r.r1.adapter->setSite("site_b", blind);
  ASSERT_TRUE(seen(*r.r1.runner, "site_b", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE));
  ASSERT_EQ(r.r1.runner->tick(close("site_b")).status, "SUCCESS");
  const auto t0 = m26::Clock::now();
  const auto t = r.r1.runner->tick(verify("site_b", "held", kOutputs));
  const auto waited = msSince(t0);
  EXPECT_EQ(t.get("r"), "VERIFICATION_UNAVAILABLE");
  EXPECT_EQ(t.get("f"), "VERIFICATION_UNAVAILABLE");
  EXPECT_EQ(t.get("s"), "/r1/eef/composite_tool/sites/site_b");
  EXPECT_LT(waited, 1500) << "at once, not at the 3000 ms deadline";
  const auto s = r.r1.runner->device(kTool).latest().sample;
  ASSERT_TRUE(s);
  EXPECT_EQ(m26::after::toNs(s->site_stamp[1]), 0) << "stamped 0, as the adapters stamp it";
}

// ADR-0022 TMR-2: once the ROS context shuts down (SIGINT, a launch shutdown), every wait ends at
// once with COMMAND_ABORTED "interrupted: the ROS context shut down" -- never a throw out of a node
// (count_publishers throws on a shut-down context), and never "0 publishers". The runner's node is
// on its OWN context here, so this process's default context lives on.
TEST(M26EefFixture, a_shut_down_context_ends_every_wait_COMMAND_ABORTED)
{
  Guard g("shutdown");
  m26::Log log;
  const auto valve = m26::after::contractFromYaml(valveYaml("valve_sd", 10000), "m26_valve_sd");
  FakeEefAdapter fa(log, "/r16", *valve.device("valve_sd"));
  fa.setBool("vacuum").script({SetBoolReply::noAnswer()});
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(TRAINIT_M26_DOMAIN);
  io.auto_initialize_logging(false);
  context->init(0, nullptr, io);
  auto node = std::make_shared<rclcpp::Node>("trainit_run_bt", "/r16",
                                             rclcpp::NodeOptions().context(context).use_global_arguments(false));
  auto registry = std::make_unique<trainit::EndEffectorRegistry>(node, valve);
  rclcpp::ExecutorOptions eo;
  eo.context = context;
  auto exec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(eo, 3);
  exec->add_node(node);
  std::thread spinner([&exec] { exec->spin(); });
  auto& dev = *registry->device("valve_sd");
  ASSERT_TRUE(m26::waitUntil([&] { return dev.latest().sample != nullptr; }, milliseconds(6000)));
  ASSERT_TRUE(m26::waitUntil([&] {
    return node->get_service_names_and_types().count("/r16/eef/valve_sd/actuators/vacuum/set_engaged") == 1;
  }, milliseconds(6000)));
  EXPECT_EQ(dev.statePublishers(), std::optional<std::size_t>(1));
  // a SetBool that is never answered (command_timeout_ms 10 s), the context shut down after 300 ms
  auto stopper = std::async(std::launch::async, [&context] {
    std::this_thread::sleep_for(milliseconds(300));
    context->shutdown("m26: the test interrupts the run");
  });
  const auto t0 = m26::Clock::now();
  const auto res = registry->grip("CloseGripper", "valve_sd", "", true);
  const auto waited = msSince(t0);
  stopper.get();
  EXPECT_EQ(res.reason, eef::Reason::COMMAND_ABORTED) << res.detail;
  EXPECT_EQ(res.detail, "interrupted: the ROS context shut down");
  EXPECT_EQ(res.subject, "/r16/eef/valve_sd/actuators/vacuum");
  EXPECT_LT(waited, 3000) << "at once, not at command_timeout_ms (10000)";
  EXPECT_EQ(fa.setBool("vacuum").calls().size(), 1u) << "the shutdown ended the RESPONSE wait";
  EXPECT_THROW(node->count_publishers("/r16/eef/valve_sd/state"), std::exception)
    << "what the registry must never let out of a tick";
  EXPECT_EQ(dev.statePublishers(), std::nullopt);
  EXPECT_EQ(registry->grip("OpenGripper", "valve_sd", "", false).reason, eef::Reason::COMMAND_ABORTED);
  const auto v = registry->verifyGrasp("VerifyGrasp", "valve_sd", "", "released", std::nullopt, std::nullopt, "");
  EXPECT_EQ(v.reason, eef::Reason::COMMAND_ABORTED) << v.detail;
  EXPECT_EQ(v.subject, "/r16/eef/valve_sd");
  EXPECT_TRUE(registry->faultLatched());
  spinner.join();   // spin() returns once its context is shut down
  exec.reset();
  registry.reset();
  node.reset();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
