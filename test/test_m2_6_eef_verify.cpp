// vNext M2.6 AFTER (ADR-0022), M3: the VerifyGrasp judge on synthetic samples (pure, no ROS
// graph), single- and multi-site, on the conformance fixture's devices:
//   - the check order (WRONG_EEF first; evidence before LOST; a fault is never a verdict);
//   - the reference rule: a sample measured between this runner's send and the adapter's apply is
//     never judged; a runner clock 50 ms behind the adapter's is fine;
//   - the deadline precedence STATE_MISSING > STATE_STALE > STATE_UNKNOWN > NOT_CONFIRMED_TIMEOUT;
//   - the tool-level rules (some site held and none LOST; every site released; skip the sites
//     with no current engage);
//   - single writer, WRONG_EEF, CONTRACT_MISMATCH on ids, order, site_grip, interfaces and
//     unequal arrays, CLOCK_MISMATCH;
//   - checks 4 and 5 run as soon as the reference rule and check 3 passed, before the freshness
//     gate: a site the adapter cannot observe is stamped 0, as the real adapters stamp it, and a
//     FAULT tool is a fault even while the site's samples are old;
//   - the millisecond values are saturated: no tolerance or age overflows into nanoseconds.
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <trainit_interfaces/msg/end_effector_actuator_state.hpp>
#include <trainit_interfaces/msg/end_effector_state.hpp>

#include "m2_6/eef_after_contract.hpp"
#include "trainit_motion_runtime/verify_grasp.hpp"

namespace
{
namespace eef = trainit::eef;
using State = trainit_interfaces::msg::EndEffectorState;
using Act = trainit_interfaces::msg::EndEffectorActuatorState;
using R = eef::Reason;
using V = eef::Verdict;

constexpr int64_t MS = 1000000;
constexpr int64_t T0 = 1000000LL * MS;   // 1000 s: every stamp below is T0 + something

builtin_interfaces::msg::Time stamp(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

const eef::Contract& contract()
{
  static const eef::Contract c = m26::after::fixtureContract();   // test data, built once
  return c;
}
const eef::DeviceSpec& suction() { return *contract().device("suction_simple"); }
const eef::DeviceSpec& composite() { return *contract().device("composite_tool"); }

const char* kSuction = "/r1/eef/suction_simple";
const char* kComposite = "/r1/eef/composite_tool";

// A well-formed sample of `d`, as its adapter publishes it: every actuator IDLE with no command,
// every site NOT_HELD (sensor), all observed at `at`.
State sample(const eef::DeviceSpec& d, const std::string& identity, int64_t at)
{
  State s;
  s.end_effector = identity;
  s.adapter = "m26_synthetic";
  s.status = State::STATUS_READY;
  for (const auto& site : d.sites)
  {
    s.sites.push_back(site.id);
    s.site_grip.push_back(site.grip);
    s.site_grasp.push_back(State::GRASP_NOT_HELD);
    s.site_evidence.push_back(State::EVIDENCE_SENSOR);
    s.site_stamp.push_back(stamp(at));
  }
  for (const auto& a : d.actuators)
  {
    Act e;
    e.actuator = a.id;
    e.command_interface = static_cast<uint8_t>(a.command_interface);
    e.joints = a.joints;
    e.status = Act::STATUS_IDLE;
    e.binary = a.command_interface == eef::CommandInterface::SET_BOOL ? Act::BINARY_RELEASED : Act::BINARY_UNKNOWN;
    e.stamp = stamp(at);
    s.actuators.push_back(e);
  }
  s.header.stamp = stamp(at);
  return s;
}

std::size_t actIndex(const State& s, const std::string& a)
{
  for (std::size_t i = 0; i < s.actuators.size(); ++i)
    if (s.actuators[i].actuator == a) return i;
  ADD_FAILURE() << "no actuator " << a;
  return 0;
}
std::size_t siteIndex(const State& s, const std::string& id)
{
  for (std::size_t i = 0; i < s.sites.size(); ++i)
    if (s.sites[i] == id) return i;
  ADD_FAILURE() << "no site " << id;
  return 0;
}

// the adapter APPLIED `command` to `actuator` at `at`
void applied(State& s, const std::string& actuator, uint8_t command, int64_t at)
{
  auto& e = s.actuators[actIndex(s, actuator)];
  e.last_command = command;
  e.last_command_stamp = stamp(at);
}
// site `id` observed `grasp` (with `evidence`) at `at`
void observed(State& s, const std::string& id, uint8_t grasp, uint8_t evidence, int64_t at)
{
  const auto i = siteIndex(s, id);
  s.site_grasp[i] = grasp;
  s.site_evidence[i] = evidence;
  s.site_stamp[i] = stamp(at);
}

eef::JudgeInput input(const eef::DeviceSpec& d, const std::string& identity, const State* s, int64_t now,
                      eef::Expect expect = eef::Expect::HELD, const std::string& site = "")
{
  eef::JudgeInput in;
  in.device = &d;
  in.identity = identity;
  in.sample = s;
  in.now_ns = now;
  in.site = site;
  in.expect = expect;
  in.max_age_ms = d.verification.max_age_ms;
  in.clock_tolerance_ms = d.verification.clock_tolerance_ms;
  in.accepted = d.verification.accepted_evidence;
  in.publishers = 1;
  return in;
}

eef::SentCommand sent(uint8_t command, int64_t at)
{
  eef::SentCommand c;
  c.command = command;
  c.stamp_ns = at;
  return c;
}

eef::JudgeInput atDeadline(eef::JudgeInput in)
{
  in.at_deadline = true;
  return in;
}

// a suction sample after this runner's engage at T0 (applied at T0 + 10 ms), the site observed
// `grasp` at T0 + 40 ms; judged at T0 + 50 ms
State suctionAfterEngage(uint8_t grasp, uint8_t evidence = State::EVIDENCE_SENSOR)
{
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 10 * MS);
  observed(s, "main", grasp, evidence, T0 + 40 * MS);
  return s;
}

#define EXPECT_RESULT(j, verdict_, reason_, subject_)                                                      \
  do                                                                                                        \
  {                                                                                                         \
    const auto jr_ = (j);                                                                                   \
    EXPECT_EQ(jr_.verdict, verdict_) << "detail: " << jr_.detail;                                          \
    EXPECT_EQ(jr_.reason, reason_) << "got " << eef::toToken(jr_.reason) << " detail: " << jr_.detail;      \
    EXPECT_EQ(jr_.subject, std::string(subject_)) << "detail: " << jr_.detail;                            \
  } while (0)
}  // namespace

TEST(M26EefVerify, no_sample_waits_then_STATE_MISSING_about_the_device)
{
  auto in = input(suction(), kSuction, nullptr, T0);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT);
  EXPECT_RESULT(eef::judge(atDeadline(in)), V::FAILURE, R::STATE_MISSING, kSuction);
  EXPECT_EQ(eef::classOf(R::STATE_MISSING), eef::ReasonClass::STATE_PROVIDER_MISSING);
}

TEST(M26EefVerify, a_held_fresh_sensor_sample_after_the_engage_is_SUCCESS)
{
  const State s = suctionAfterEngage(State::GRASP_HELD);
  const auto j = eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS));
  EXPECT_EQ(j.verdict, V::SUCCESS) << j.detail;
  EXPECT_FALSE(j.inferred);
}

TEST(M26EefVerify, check_1_WRONG_EEF_comes_before_everything)
{
  State s = suctionAfterEngage(State::GRASP_HELD);
  s.end_effector = "/r2/eef/suction_simple";
  s.site_grasp.clear();   // also malformed: identity is checked first
  auto in = input(suction(), kSuction, &s, T0 + 50 * MS);
  in.publishers = 2;
  EXPECT_RESULT(eef::judge(in), V::FAILURE, R::WRONG_EEF, kSuction);
}

TEST(M26EefVerify, check_2_a_second_writer_is_CONTRACT_MISMATCH)
{
  const State s = suctionAfterEngage(State::GRASP_HELD);
  auto in = input(suction(), kSuction, &s, T0 + 50 * MS);
  in.publishers = 2;
  EXPECT_RESULT(eef::judge(in), V::FAILURE, R::CONTRACT_MISMATCH, kSuction);
}

TEST(M26EefVerify, check_2_the_structure_must_be_the_contract)
{
  const int64_t now = T0 + 50 * MS;
  auto mismatch = [&](const std::string& what, const std::function<void(State&)>& edit) {
    State s = sample(composite(), kComposite, T0);
    applied(s, "gripper_a", Act::COMMAND_ENGAGE, T0 + 10 * MS);
    observed(s, "site_a", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
    edit(s);
    const auto j = eef::judge(input(composite(), kComposite, &s, now, eef::Expect::HELD, "site_a"));
    EXPECT_EQ(j.reason, R::CONTRACT_MISMATCH) << what << ": " << j.detail;
    EXPECT_EQ(j.subject, kComposite) << what;
  };
  mismatch("an actuator id", [](State& s) { s.actuators[5].actuator = "lock"; });
  mismatch("the actuator order", [](State& s) { std::swap(s.actuators[0], s.actuators[1]); });
  mismatch("an actuator missing", [](State& s) { s.actuators.pop_back(); });
  mismatch("a site id", [](State& s) { s.sites[1] = "site_c"; });
  mismatch("the site order", [](State& s) {
    std::swap(s.sites[0], s.sites[1]);
    std::swap(s.site_grip[0], s.site_grip[1]);
  });
  mismatch("site_grip", [](State& s) { s.site_grip[1] = "gripper_a"; });
  mismatch("an interface", [](State& s) { s.actuators[4].command_interface = Act::INTERFACE_GRIPPER_COMMAND; });
  mismatch("unequal site arrays (site_grasp)", [](State& s) { s.site_grasp.pop_back(); });
  mismatch("unequal site arrays (site_stamp)", [](State& s) { s.site_stamp.push_back(s.site_stamp[0]); });
  // and the same sample untouched is SUCCESS
  State ok = sample(composite(), kComposite, T0);
  applied(ok, "gripper_a", Act::COMMAND_ENGAGE, T0 + 10 * MS);
  observed(ok, "site_a", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
  EXPECT_EQ(eef::judge(input(composite(), kComposite, &ok, now, eef::Expect::HELD, "site_a")).verdict, V::SUCCESS);
}

TEST(M26EefVerify, check_3_a_judged_stamp_in_the_future_is_CLOCK_MISMATCH)
{
  State s = suctionAfterEngage(State::GRASP_HELD);
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 260 * MS);
  // now T0 + 50 ms: the observation is 210 ms ahead, the tolerance 100 ms
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)), V::FAILURE, R::CLOCK_MISMATCH,
                "/r1/eef/suction_simple/sites/main");
  // within the tolerance it is judged
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 120 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)).verdict, V::SUCCESS);
  // the engage's own stamp in the future
  State t = suctionAfterEngage(State::GRASP_HELD);
  applied(t, "vacuum", Act::COMMAND_ENGAGE, T0 + 400 * MS);
  observed(t, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 410 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &t, T0 + 50 * MS)).reason, R::CLOCK_MISMATCH);
}

TEST(M26EefVerify, check_4_a_FAULT_or_UNAVAILABLE_tool_is_an_actuator_failure)
{
  State s = suctionAfterEngage(State::GRASP_HELD);
  s.status = State::STATUS_FAULT;
  s.actuators[0].status = Act::STATUS_FAULT;
  s.detail = "vacuum: FAULT";
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)), V::FAILURE, R::HARDWARE_FAULT,
                "/r1/eef/suction_simple/actuators/vacuum");
  s.actuators[0].status = Act::STATUS_IDLE;   // a tool-level fault: the device is the subject
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)), V::FAILURE, R::HARDWARE_FAULT, kSuction);
  s.status = State::STATUS_UNAVAILABLE;
  s.actuators[0].status = Act::STATUS_UNAVAILABLE;
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)), V::FAILURE, R::EEF_UNAVAILABLE,
                "/r1/eef/suction_simple/actuators/vacuum");
  EXPECT_TRUE(eef::isFault(R::HARDWARE_FAULT));
  // BUSY and UNKNOWN tools are not failures
  s.status = State::STATUS_BUSY;
  s.actuators[0].status = Act::STATUS_MOVING;
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS)).verdict, V::SUCCESS);
}

TEST(M26EefVerify, checks_5_and_6_unavailable_then_evidence_before_LOST)
{
  // 5: a site that cannot be observed
  State u = suctionAfterEngage(State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE);
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &u, T0 + 50 * MS)), V::FAILURE, R::VERIFICATION_UNAVAILABLE,
                "/r1/eef/suction_simple/sites/main");
  // 6 before 7: LOST from an unaccepted class is EVIDENCE_NOT_ACCEPTED, never OBJECT_LOST
  State l = suctionAfterEngage(State::GRASP_LOST, State::EVIDENCE_ACTUATION_INFERENCE);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &l, T0 + 50 * MS)).reason, R::EVIDENCE_NOT_ACCEPTED);
  // ... and PENDING from an unaccepted class too
  State p = suctionAfterEngage(State::GRASP_PENDING, State::EVIDENCE_ACTUATION_INFERENCE);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &p, T0 + 50 * MS)).reason, R::EVIDENCE_NOT_ACCEPTED);
  // HELD by inference is not accepted by default ...
  State h = suctionAfterEngage(State::GRASP_HELD, State::EVIDENCE_ACTUATION_INFERENCE);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &h, T0 + 50 * MS)).reason, R::EVIDENCE_NOT_ACCEPTED);
  // ... and accepted, LABELLED as inference, only with the opt-in
  auto in = input(suction(), kSuction, &h, T0 + 50 * MS);
  in.accepted.push_back(eef::EvidenceClass::ACTUATION_INFERENCE);
  const auto j = eef::judge(in);
  EXPECT_EQ(j.verdict, V::SUCCESS);
  EXPECT_TRUE(j.inferred);
  // 7: LOST from an accepted class
  State ls = suctionAfterEngage(State::GRASP_LOST, State::EVIDENCE_SENSOR);
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &ls, T0 + 50 * MS)), V::FAILURE, R::OBJECT_LOST,
                "/r1/eef/suction_simple/sites/main");
}

// Check 5 on the adapters' own stamp convention: a site with no deciding source on its backend is
// published UNAVAILABLE / NONE with site_stamp 0 -- never fresh -- and still fails at once with
// VERIFICATION_UNAVAILABLE once the grip's entry shows the reference command, never STATE_STALE
// at the deadline.
TEST(M26EefVerify, check_5_an_unobservable_site_stamped_0_is_VERIFICATION_UNAVAILABLE_at_once)
{
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 10 * MS);
  observed(s, "main", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  auto in = input(suction(), kSuction, &s, T0 + 50 * MS);
  in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0);   // this runner's engage, reflected
  const auto j = eef::judge(in);
  EXPECT_RESULT(j, V::FAILURE, R::VERIFICATION_UNAVAILABLE, "/r1/eef/suction_simple/sites/main");
  EXPECT_EQ(eef::classOf(j.reason), eef::ReasonClass::VERIFICATION_ERROR);
  // ... and for `released`, after this runner's applied release
  State r = sample(suction(), kSuction, T0);
  applied(r, "vacuum", Act::COMMAND_RELEASE, T0 + 10 * MS);
  observed(r, "main", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  auto rin = input(suction(), kSuction, &r, T0 + 50 * MS, eef::Expect::RELEASED);
  rin.sent["vacuum"] = sent(Act::COMMAND_RELEASE, T0);
  EXPECT_RESULT(eef::judge(rin), V::FAILURE, R::VERIFICATION_UNAVAILABLE, "/r1/eef/suction_simple/sites/main");
  // the reference rule still comes first: an entry that does not show this runner's engage waits,
  // and the deadline says where the command went
  State w = sample(suction(), kSuction, T0);
  observed(w, "main", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  auto win = input(suction(), kSuction, &w, T0 + 50 * MS);
  win.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0);
  EXPECT_EQ(eef::judge(win).verdict, V::WAIT);
  EXPECT_EQ(eef::judge(atDeadline(win)).reason, R::CONTRACT_MISMATCH);
  // ... and check 3: a future last_command_stamp is CLOCK_MISMATCH, before check 5
  State c = sample(suction(), kSuction, T0);
  applied(c, "vacuum", Act::COMMAND_ENGAGE, T0 + 400 * MS);
  observed(c, "main", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &c, T0 + 50 * MS)).reason, R::CLOCK_MISMATCH);
}

// Check 4 before the freshness gate: a FAULT tool is HARDWARE_FAULT at once, while the site's only
// samples are older than the reference (between the send and the apply) or than max_age.
TEST(M26EefVerify, check_4_a_FAULT_tool_with_a_stale_site_is_HARDWARE_FAULT_at_once)
{
  for (const int64_t site_at : {T0 + 5 * MS, T0 - 2000 * MS})   // before the apply; 2 s old
  {
    State s = sample(suction(), kSuction, T0);
    applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 10 * MS);
    observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, site_at);
    s.status = State::STATUS_FAULT;
    s.actuators[0].status = Act::STATUS_FAULT;
    s.detail = "vacuum: FAULT";
    auto in = input(suction(), kSuction, &s, T0 + 50 * MS);
    in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0);
    EXPECT_RESULT(eef::judge(in), V::FAILURE, R::HARDWARE_FAULT, "/r1/eef/suction_simple/actuators/vacuum");
    // an UNAVAILABLE tool likewise; and check 4 comes before check 5
    s.status = State::STATUS_UNAVAILABLE;
    s.actuators[0].status = Act::STATUS_UNAVAILABLE;
    observed(s, "main", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
    EXPECT_RESULT(eef::judge(in), V::FAILURE, R::EEF_UNAVAILABLE, "/r1/eef/suction_simple/actuators/vacuum");
  }
}

// ADR-0022 TMR-5: every millisecond value is saturated before it becomes nanoseconds -- the parser
// accepts any positive int64, as the reference does; unsaturated, 1e13 ms wraps negative and every
// sample reads stale.
TEST(M26EefVerify, the_millisecond_values_are_saturated)
{
  const State s = suctionAfterEngage(State::GRASP_HELD);   // observed at T0 + 40 ms
  auto in = input(suction(), kSuction, &s, T0 + 2000 * MS);   // 1960 ms old
  EXPECT_EQ(eef::judge(atDeadline(in)).reason, R::STATE_STALE) << "older than the 500 ms default";
  in.max_age_ms = 10000000000000LL;   // "practically forever"
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS) << eef::judge(in).detail;
  in.max_age_ms = std::numeric_limits<int64_t>::max();
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS) << eef::judge(in).detail;
  in.clock_tolerance_ms = std::numeric_limits<int64_t>::max();
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS) << eef::judge(in).detail;
}

TEST(M26EefVerify, an_echo_or_a_weld_never_confirms)
{
  for (const uint8_t never : {State::EVIDENCE_COMMAND_ECHO, State::EVIDENCE_SIM_ATTACHMENT})
  {
    State s = suctionAfterEngage(State::GRASP_HELD, never);
    auto in = input(suction(), kSuction, &s, T0 + 50 * MS);
    in.accepted = {eef::EvidenceClass::SENSOR, eef::EvidenceClass::SIM_GROUND_TRUTH, eef::EvidenceClass::ACTUATION_INFERENCE};
    EXPECT_EQ(eef::judge(in).reason, R::EVIDENCE_NOT_ACCEPTED) << eef::evidenceToken(never);
  }
}

TEST(M26EefVerify, check_8_a_fresh_NOT_HELD_after_the_engage_is_the_verdict_NO_OBJECT)
{
  const State s = suctionAfterEngage(State::GRASP_NOT_HELD);
  const auto j = eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS));
  EXPECT_RESULT(j, V::FAILURE, R::NO_OBJECT, "/r1/eef/suction_simple/sites/main");
  EXPECT_FALSE(eef::isFault(j.reason));   // a VERDICT
}

TEST(M26EefVerify, the_reference_rule_a_sample_between_send_and_apply_is_never_judged)
{
  // this runner sent ENGAGE at T0; the adapter applied it at T0 + 100 ms; the deciding source
  // observed HELD at T0 + 50 ms -- between the send and the apply
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 100 * MS);
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 50 * MS);
  auto in = input(suction(), kSuction, &s, T0 + 120 * MS);
  in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT) << "a sample older than the apply is never judged";
  EXPECT_RESULT(eef::judge(atDeadline(in)), V::FAILURE, R::STATE_STALE, "/r1/eef/suction_simple/sites/main");
  // the same verdict-worthy NOT_HELD between send and apply is never a verdict either
  observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 50 * MS);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT);
  // the next observation, after the apply, is judged
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 110 * MS);
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS);
}

TEST(M26EefVerify, the_reference_rule_a_command_that_never_reaches_the_state_is_CONTRACT_MISMATCH)
{
  // the entry still shows an older release: this runner's engage at T0 + 500 ms went elsewhere
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_RELEASE, T0 + 10 * MS);
  observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 600 * MS);
  auto in = input(suction(), kSuction, &s, T0 + 610 * MS);
  in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0 + 500 * MS);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT) << "never NO_OBJECT on a state that does not show the command";
  const auto j = eef::judge(atDeadline(in));
  EXPECT_RESULT(j, V::FAILURE, R::CONTRACT_MISMATCH, "/r1/eef/suction_simple/actuators/vacuum");
  EXPECT_EQ(eef::classOf(j.reason), eef::ReasonClass::CONTRACT_FAILURE);
}

// Two applies of the same command within the clock tolerance (a re-engage, 60 ms apart): the
// earlier one is not a reflection of the later send -- the entry must be NEWER than the one this
// runner saw when it sent, or the previous engage's decision would be judged.
TEST(M26EefVerify, the_reference_rule_a_re_engage_is_judged_on_its_own_apply)
{
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 10 * MS);                         // the first engage
  observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 60 * MS);   // its verdict
  auto in = input(suction(), kSuction, &s, T0 + 75 * MS);
  in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0 + 70 * MS);
  in.sent["vacuum"].prior_ns = T0 + 10 * MS;   // what this runner saw when it re-engaged
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT) << "the first engage's NOT_HELD is not this engage's verdict";
  EXPECT_EQ(eef::judge(atDeadline(in)).reason, R::CONTRACT_MISMATCH);
  // the adapter applies the re-engage at T0 + 80 ms; its own decision is judged
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 80 * MS);
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 90 * MS);
  in.now_ns = T0 + 95 * MS;
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS);
}

TEST(M26EefVerify, a_runner_clock_50_ms_behind_the_adapter_is_fine)
{
  // adapter clock = runner clock + 50 ms. The runner sends at its T0 (adapter T0 + 50); the
  // adapter applies at its T0 + 60 and observes HELD at its T0 + 90; the runner judges at its
  // T0 + 60 (adapter T0 + 110).
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_ENGAGE, T0 + 60 * MS);
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 90 * MS);
  auto in = input(suction(), kSuction, &s, T0 + 60 * MS);
  in.sent["vacuum"] = sent(Act::COMMAND_ENGAGE, T0);
  EXPECT_EQ(eef::judge(in).verdict, V::SUCCESS) << eef::judge(in).detail;
  // 200 ms behind is beyond the 100 ms tolerance: the observation reads as future
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 260 * MS);
  EXPECT_EQ(eef::judge(in).reason, R::CLOCK_MISMATCH);
}

TEST(M26EefVerify, held_needs_a_current_engage)
{
  State none = sample(suction(), kSuction, T0);   // no command at all
  EXPECT_RESULT(eef::judge(input(suction(), kSuction, &none, T0 + 50 * MS)), V::FAILURE, R::NO_ENGAGE_COMMAND,
                "/r1/eef/suction_simple/sites/main");
  State released = suctionAfterEngage(State::GRASP_NOT_HELD);
  applied(released, "vacuum", Act::COMMAND_RELEASE, T0 + 20 * MS);   // a newer applied release
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &released, T0 + 50 * MS)).reason, R::NO_ENGAGE_COMMAND);
}

TEST(M26EefVerify, released_rules)
{
  // after an applied release: NOT_HELD at or after it
  State s = sample(suction(), kSuction, T0);
  applied(s, "vacuum", Act::COMMAND_RELEASE, T0 + 10 * MS);
  observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 30 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS, eef::Expect::RELEASED)).verdict, V::SUCCESS);
  // a NOT_HELD older than the release is not judged
  observed(s, "main", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 5 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &s, T0 + 50 * MS, eef::Expect::RELEASED)).verdict, V::WAIT);
  // still HELD after the release: RELEASE_NOT_CONFIRMED at the deadline, a FAULT
  observed(s, "main", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 30 * MS);
  auto in = input(suction(), kSuction, &s, T0 + 50 * MS, eef::Expect::RELEASED);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT);
  const auto j = eef::judge(atDeadline(in));
  EXPECT_RESULT(j, V::FAILURE, R::RELEASE_NOT_CONFIRMED, "/r1/eef/suction_simple/sites/main");
  EXPECT_TRUE(eef::isFault(j.reason));
  // the grip still engaged
  State e = suctionAfterEngage(State::GRASP_HELD);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &e, T0 + 50 * MS, eef::Expect::RELEASED)).reason,
            R::NO_RELEASE_COMMAND);
  // no command at all: a fresh NOT_HELD is enough ("the tool is empty")
  State empty = sample(suction(), kSuction, T0 + 40 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &empty, T0 + 50 * MS, eef::Expect::RELEASED)).verdict, V::SUCCESS);
}

TEST(M26EefVerify, the_deadline_precedence)
{
  const int64_t now = T0 + 2000 * MS;
  // STATE_MISSING: no sample at all
  EXPECT_EQ(eef::judge(atDeadline(input(suction(), kSuction, nullptr, now))).reason, R::STATE_MISSING);
  // STATE_STALE: only old samples (older than max_age 500 ms) -- a PENDING among them
  State old = suctionAfterEngage(State::GRASP_PENDING);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &old, now)).verdict, V::WAIT);
  EXPECT_EQ(eef::judge(atDeadline(input(suction(), kSuction, &old, now))).reason, R::STATE_STALE)
    << "an old PENDING is never a verdict";
  // STATE_UNKNOWN: fresh, still UNKNOWN (a fault)
  State unknown = suctionAfterEngage(State::GRASP_UNKNOWN, State::EVIDENCE_NONE);
  observed(unknown, "main", State::GRASP_UNKNOWN, State::EVIDENCE_NONE, now - 10 * MS);
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &unknown, now)).verdict, V::WAIT);
  const auto u = eef::judge(atDeadline(input(suction(), kSuction, &unknown, now)));
  EXPECT_EQ(u.reason, R::STATE_UNKNOWN);
  EXPECT_EQ(eef::classOf(u.reason), eef::ReasonClass::STALE_STATE);
  // NOT_CONFIRMED_TIMEOUT: fresh, still PENDING (a verdict)
  State pending = suctionAfterEngage(State::GRASP_PENDING);
  observed(pending, "main", State::GRASP_PENDING, State::EVIDENCE_SENSOR, now - 10 * MS);
  const auto p = eef::judge(atDeadline(input(suction(), kSuction, &pending, now)));
  EXPECT_EQ(p.reason, R::NOT_CONFIRMED_TIMEOUT);
  EXPECT_FALSE(eef::isFault(p.reason));
  // an unknown constant reads as UNKNOWN
  State weird = suctionAfterEngage(State::GRASP_HELD);
  observed(weird, "main", 9, State::EVIDENCE_NONE, now - 10 * MS);
  EXPECT_EQ(eef::judge(atDeadline(input(suction(), kSuction, &weird, now))).reason, R::STATE_UNKNOWN);
}

// --- tool level (a multi-site device, no site) ------------------------------------------------------
namespace
{
// composite_tool with gripper_a and gripper_b engaged at T0 + 10 ms, both sites observed at T0 + 40
State compositeEngaged(uint8_t grasp_a, uint8_t grasp_b)
{
  State s = sample(composite(), kComposite, T0);
  applied(s, "gripper_a", Act::COMMAND_ENGAGE, T0 + 10 * MS);
  applied(s, "gripper_b", Act::COMMAND_ENGAGE, T0 + 10 * MS);
  observed(s, "site_a", grasp_a, grasp_a == State::GRASP_UNAVAILABLE ? State::EVIDENCE_NONE : State::EVIDENCE_SENSOR, T0 + 40 * MS);
  observed(s, "site_b", grasp_b, grasp_b == State::GRASP_UNAVAILABLE ? State::EVIDENCE_NONE : State::EVIDENCE_SENSOR, T0 + 40 * MS);
  return s;
}
}  // namespace

TEST(M26EefVerify, tool_level_held_skips_the_sites_with_no_current_engage)
{
  // the handoff's end: site_a released (its grip's newest applied command is a RELEASE), site_b held
  State s = compositeEngaged(State::GRASP_NOT_HELD, State::GRASP_HELD);
  applied(s, "gripper_a", Act::COMMAND_RELEASE, T0 + 20 * MS);
  const auto j = eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS));
  EXPECT_EQ(j.verdict, V::SUCCESS) << j.detail;
  EXPECT_EQ(j.subject, "/r1/eef/composite_tool/sites/site_b");
}

TEST(M26EefVerify, tool_level_held_with_no_evaluated_site_is_NO_ENGAGE_COMMAND_about_the_device)
{
  State s = sample(composite(), kComposite, T0 + 40 * MS);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS)), V::FAILURE, R::NO_ENGAGE_COMMAND,
                kComposite);
}

TEST(M26EefVerify, tool_level_held_fails_on_LOST_at_another_site)
{
  State s = compositeEngaged(State::GRASP_HELD, State::GRASP_LOST);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS)), V::FAILURE, R::OBJECT_LOST,
                "/r1/eef/composite_tool/sites/site_b");
}

TEST(M26EefVerify, tool_level_held_waits_while_a_site_is_undecided)
{
  State s = compositeEngaged(State::GRASP_PENDING, State::GRASP_HELD);
  auto in = input(composite(), kComposite, &s, T0 + 50 * MS);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT);
  EXPECT_EQ(eef::judge(atDeadline(in)).verdict, V::SUCCESS) << "held through site_b, none LOST";
}

TEST(M26EefVerify, tool_level_held_fails_with_the_first_fault_else_the_first_verdict)
{
  State v = compositeEngaged(State::GRASP_NOT_HELD, State::GRASP_UNAVAILABLE);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &v, T0 + 50 * MS)), V::FAILURE,
                R::VERIFICATION_UNAVAILABLE, "/r1/eef/composite_tool/sites/site_b");
  State n = compositeEngaged(State::GRASP_NOT_HELD, State::GRASP_NOT_HELD);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &n, T0 + 50 * MS)), V::FAILURE, R::NO_OBJECT,
                "/r1/eef/composite_tool/sites/site_a");
}

TEST(M26EefVerify, tool_level_released_needs_every_site_released)
{
  State s = sample(composite(), kComposite, T0);
  applied(s, "gripper_a", Act::COMMAND_RELEASE, T0 + 10 * MS);
  applied(s, "gripper_b", Act::COMMAND_RELEASE, T0 + 10 * MS);
  observed(s, "site_a", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
  observed(s, "site_b", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
  EXPECT_EQ(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS, eef::Expect::RELEASED)).verdict, V::SUCCESS);
  // site_b still HELD after its release: the first site not released, at the deadline
  observed(s, "site_b", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
  auto in = input(composite(), kComposite, &s, T0 + 50 * MS, eef::Expect::RELEASED);
  EXPECT_EQ(eef::judge(in).verdict, V::WAIT);
  EXPECT_RESULT(eef::judge(atDeadline(in)), V::FAILURE, R::RELEASE_NOT_CONFIRMED, "/r1/eef/composite_tool/sites/site_b");
  // site_b's grip still engaged
  applied(s, "gripper_b", Act::COMMAND_ENGAGE, T0 + 20 * MS);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS, eef::Expect::RELEASED)), V::FAILURE,
                R::NO_RELEASE_COMMAND, "/r1/eef/composite_tool/sites/site_b");
}

TEST(M26EefVerify, tool_level_checks_1_to_4_run_once_for_the_device)
{
  State s = compositeEngaged(State::GRASP_HELD, State::GRASP_HELD);
  s.status = State::STATUS_FAULT;
  s.actuators[5].status = Act::STATUS_FAULT;   // binary_lock
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS)), V::FAILURE, R::HARDWARE_FAULT,
                "/r1/eef/composite_tool/actuators/binary_lock");
}

// P5: an actuator's FAULT is a fault about that actuator, not a verdict about a site.
TEST(M26EefVerify, a_fault_elsewhere_on_the_tool_is_a_fault_not_a_verdict)
{
  State s = compositeEngaged(State::GRASP_HELD, State::GRASP_NOT_HELD);
  s.status = State::STATUS_FAULT;
  s.actuators[5].status = Act::STATUS_FAULT;
  s.detail = "binary_lock: FAULT";
  const auto j = eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS, eef::Expect::HELD, "site_a"));
  EXPECT_RESULT(j, V::FAILURE, R::HARDWARE_FAULT, "/r1/eef/composite_tool/actuators/binary_lock");
  EXPECT_EQ(eef::classOf(j.reason), eef::ReasonClass::ACTUATOR_FAILURE);
}

TEST(M26EefVerify, an_unknown_site_is_UNKNOWN_SITE)
{
  State s = compositeEngaged(State::GRASP_HELD, State::GRASP_HELD);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &s, T0 + 50 * MS, eef::Expect::HELD, "site_z")),
                V::FAILURE, R::UNKNOWN_SITE, "/r1/eef/composite_tool/sites/site_z");
}

TEST(M26EefVerify, a_default_constructed_sample_reads_as_cannot_tell)
{
  const State blank;   // "cannot tell": an empty end_effector
  EXPECT_EQ(eef::judge(input(suction(), kSuction, &blank, T0)).reason, R::WRONG_EEF);
}

// Checks 4 and 5 at tool level, on the adapters' stamp convention (an unobservable site stamped 0).
TEST(M26EefVerify, tool_level_checks_4_and_5_run_before_the_freshness_gate)
{
  // both sites engaged, neither observable: VERIFICATION_UNAVAILABLE about the first, at once
  State u = compositeEngaged(State::GRASP_UNAVAILABLE, State::GRASP_UNAVAILABLE);
  observed(u, "site_a", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  observed(u, "site_b", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &u, T0 + 50 * MS)), V::FAILURE,
                R::VERIFICATION_UNAVAILABLE, "/r1/eef/composite_tool/sites/site_a");
  // site_a NOT_HELD (a verdict), site_b unobservable: the fault, at once
  State n = compositeEngaged(State::GRASP_NOT_HELD, State::GRASP_UNAVAILABLE);
  observed(n, "site_b", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &n, T0 + 50 * MS)), V::FAILURE,
                R::VERIFICATION_UNAVAILABLE, "/r1/eef/composite_tool/sites/site_b");
  // site_a HELD: the tool holds through it; an unobservable site_b is not LOST
  State h = compositeEngaged(State::GRASP_HELD, State::GRASP_UNAVAILABLE);
  observed(h, "site_b", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  const auto held = eef::judge(input(composite(), kComposite, &h, T0 + 50 * MS));
  EXPECT_EQ(held.verdict, V::SUCCESS) << held.detail;
  EXPECT_EQ(held.subject, "/r1/eef/composite_tool/sites/site_a");
  // released, after both releases: an unobservable site is VERIFICATION_UNAVAILABLE at once
  State r = sample(composite(), kComposite, T0);
  applied(r, "gripper_a", Act::COMMAND_RELEASE, T0 + 10 * MS);
  applied(r, "gripper_b", Act::COMMAND_RELEASE, T0 + 10 * MS);
  observed(r, "site_a", State::GRASP_NOT_HELD, State::EVIDENCE_SENSOR, T0 + 40 * MS);
  observed(r, "site_b", State::GRASP_UNAVAILABLE, State::EVIDENCE_NONE, 0);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &r, T0 + 50 * MS, eef::Expect::RELEASED)), V::FAILURE,
                R::VERIFICATION_UNAVAILABLE, "/r1/eef/composite_tool/sites/site_b");
  // a FAULT tool while every site's samples are old: HARDWARE_FAULT about the actuator, at once,
  // for `held` and for `released`
  State f = compositeEngaged(State::GRASP_HELD, State::GRASP_HELD);
  observed(f, "site_a", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 - 2000 * MS);
  observed(f, "site_b", State::GRASP_HELD, State::EVIDENCE_SENSOR, T0 - 2000 * MS);
  f.status = State::STATUS_FAULT;
  f.actuators[5].status = Act::STATUS_FAULT;   // binary_lock
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &f, T0 + 50 * MS)), V::FAILURE, R::HARDWARE_FAULT,
                "/r1/eef/composite_tool/actuators/binary_lock");
  applied(f, "gripper_a", Act::COMMAND_RELEASE, T0 + 20 * MS);
  applied(f, "gripper_b", Act::COMMAND_RELEASE, T0 + 20 * MS);
  EXPECT_RESULT(eef::judge(input(composite(), kComposite, &f, T0 + 50 * MS, eef::Expect::RELEASED)), V::FAILURE,
                R::HARDWARE_FAULT, "/r1/eef/composite_tool/actuators/binary_lock");
}

// Exhaustively, over every grasp x evidence x last_command x stamp offset x expect, single-site and
// tool-level, with the site stamped as observed or 0 (the adapters' "cannot observe"): at the
// deadline the judge never waits; every FAILURE names a reason and a subject; an observation from
// an unaccepted class is never turned into a VERDICT; a VERDICT always rests on an accepted class
// at a site; and NEVER a SUCCESS without a fresh observation from an accepted class.
TEST(M26EefVerify, properties_over_every_combination)
{
  const int64_t now = T0 + 1000 * MS;
  int judged = 0;
  for (const bool tool_level : {false, true})
    for (uint8_t grasp = 0; grasp <= 6; ++grasp)
      for (uint8_t evidence = 0; evidence <= 6; ++evidence)
        for (uint8_t command = 0; command <= 3; ++command)
          for (const bool zero : {false, true})
            for (const int64_t offset_ms : {-700, -100, -5, 30, 250})
              for (const auto expect : {eef::Expect::HELD, eef::Expect::RELEASED})
              {
                const eef::DeviceSpec& d = tool_level ? composite() : suction();
                const std::string id = tool_level ? kComposite : kSuction;
                const int64_t site_at = zero ? 0 : now + offset_ms * MS;
                State s = sample(d, id, now + offset_ms * MS);
                for (const auto& site : d.sites)
                {
                  if (command != 0) applied(s, site.grip, command, now - 50 * MS);
                  observed(s, site.id, grasp, evidence, site_at);
                }
                auto in = atDeadline(input(d, id, &s, now, expect));
                const auto j = eef::judge(in);
                ++judged;
                const std::string what = std::string(tool_level ? "tool " : "site ") + eef::graspToken(grasp) + "/" +
                                         eef::evidenceToken(evidence) + "/" + eef::commandToken(command) + "/" +
                                         (zero ? std::string("stamp 0") : std::to_string(offset_ms)) +
                                         (expect == eef::Expect::HELD ? " held" : " released");
                ASSERT_NE(j.verdict, V::WAIT) << what;
                if (j.verdict == V::FAILURE)
                {
                  EXPECT_NE(j.reason, R::NONE) << what;
                  EXPECT_EQ(j.reason_class, eef::classOf(j.reason)) << what;
                  EXPECT_FALSE(j.subject.empty()) << what;
                }
                const bool unaccepted = evidence != State::EVIDENCE_NONE && evidence <= State::EVIDENCE_SIM_ATTACHMENT &&
                                        !d.verification.accepts(static_cast<eef::EvidenceClass>(evidence));
                if (unaccepted && grasp != State::GRASP_UNAVAILABLE)
                {
                  EXPECT_TRUE(j.verdict == V::FAILURE && eef::isFault(j.reason)) << what << ": got " << eef::toToken(j.reason);
                }
                if (j.verdict == V::FAILURE && !eef::isFault(j.reason))
                {
                  EXPECT_TRUE(evidence == State::EVIDENCE_SENSOR || evidence == State::EVIDENCE_SIM_GROUND_TRUTH) << what;
                  EXPECT_NE(j.subject.find("/sites/"), std::string::npos) << what << ": a verdict names a site";
                }
                if (j.verdict == V::SUCCESS && expect == eef::Expect::HELD)
                {
                  EXPECT_EQ(grasp, State::GRASP_HELD) << what;
                }
                if (j.verdict == V::SUCCESS)
                {
                  // fresh(s): at or after the reference (the applied command, when it is one), no
                  // older than max_age, not ahead of the clock; and an accepted class
                  const bool reference = (expect == eef::Expect::HELD && command == Act::COMMAND_ENGAGE) ||
                                         (expect == eef::Expect::RELEASED && command == Act::COMMAND_RELEASE);
                  const int64_t age_ms = (now - site_at) / MS;
                  const bool fresh = age_ms <= d.verification.max_age_ms && -age_ms <= d.verification.clock_tolerance_ms &&
                                     (!reference || site_at >= now - 50 * MS);
                  const bool accepted = evidence != State::EVIDENCE_NONE && evidence <= State::EVIDENCE_SIM_ATTACHMENT &&
                                        d.verification.accepts(static_cast<eef::EvidenceClass>(evidence));
                  EXPECT_TRUE(fresh) << what << ": a SUCCESS on an observation that is not fresh";
                  EXPECT_TRUE(accepted) << what << ": a SUCCESS on an unaccepted class";
                  EXPECT_FALSE(zero) << what << ": a SUCCESS on a site stamped 0";
                }
              }
  EXPECT_EQ(judged, 2 * 7 * 7 * 4 * 2 * 5 * 2);
}
