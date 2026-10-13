// =============================================================================
// verify_grasp.cpp — the ADR-0022 D4 judge: the reference rule, the check order 1-8, the
// deadline precedence and the tool-level rules, on ONE sample. Pure: no ROS graph, no clock.
//
// Checks 4 (the tool's status) and 5 (a site its backend cannot observe) are about the device
// and its binding, not about a grasp observation: they run as soon as the reference rule and
// check 3 passed, before the site's freshness gate -- an adapter honestly stamps an unobservable
// site 0, so it is never fresh. Checks 6-8, and every verdict, rest on a FRESH site sample.
// =============================================================================
#include "trainit_motion_runtime/verify_grasp.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

#include <trainit_interfaces/msg/end_effector_actuator_state.hpp>

namespace trainit
{
namespace eef
{

namespace
{
using State = trainit_interfaces::msg::EndEffectorState;
using Act = trainit_interfaces::msg::EndEffectorActuatorState;

constexpr char kSep = '/';
constexpr int64_t kNsPerMs = 1000000;

// A millisecond tolerance or age, saturated at INT32_MAX ms before it is turned into nanoseconds
// (the parser accepts any positive int64, as the reference does).
int64_t saturatedMs(int64_t ms) { return std::clamp<int64_t>(ms, 0, std::numeric_limits<int32_t>::max()); }

int64_t toNs(const builtin_interfaces::msg::Time& t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

std::string msText(int64_t ns) { return std::to_string(ns / kNsPerMs) + " ms"; }

std::string join(const std::vector<std::string>& v)
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + v[i];
  return "[" + s + "]";
}

JudgeResult make(Verdict v, Reason r, std::string subject, std::string detail)
{
  JudgeResult j;
  j.verdict = v;
  j.reason = r;
  j.reason_class = classOf(r);
  j.subject = std::move(subject);
  j.detail = std::move(detail);
  return j;
}

JudgeResult failure(Reason r, std::string subject, std::string detail)
{
  return make(Verdict::FAILURE, r, std::move(subject), std::move(detail));
}

// How one site reads in one sample.
enum class Kind : uint8_t
{
  SKIP,               // tool-level `held`: no current engage of its grip
  OK,                 // held(s) / released(s)
  FAIL,               // a decided failure
  WAIT_REFLECT,       // its grip's entry does not show this runner's command yet
  WAIT_STALE,         // not fresh: older than the reference, or older than max_age
  WAIT_UNKNOWN,       // fresh, UNKNOWN
  WAIT_PENDING,       // fresh, PENDING (held)
  WAIT_NOT_RELEASED,  // fresh, not released yet (released)
};

struct SiteEval
{
  Kind kind{Kind::SKIP};
  Reason reason{Reason::NONE};
  std::string subject{};
  std::string detail{};
  bool inferred{false};
  bool judged{false};  // passed the reference rule and check 3: checks 4 and 5 may run
  bool fresh{false};   // ... and the freshness gate: checks 6-8 may run
};

bool isWait(Kind k)
{
  return k == Kind::WAIT_REFLECT || k == Kind::WAIT_STALE || k == Kind::WAIT_UNKNOWN ||
         k == Kind::WAIT_PENDING || k == Kind::WAIT_NOT_RELEASED;
}

Reason deadlineReason(Kind k)
{
  switch (k)
  {
    case Kind::WAIT_REFLECT: return Reason::CONTRACT_MISMATCH;
    case Kind::WAIT_STALE: return Reason::STATE_STALE;
    case Kind::WAIT_UNKNOWN: return Reason::STATE_UNKNOWN;
    case Kind::WAIT_PENDING: return Reason::NOT_CONFIRMED_TIMEOUT;
    case Kind::WAIT_NOT_RELEASED: return Reason::RELEASE_NOT_CONFIRMED;
    default: return Reason::NONE;
  }
}

std::size_t indexOf(const std::vector<std::string>& v, const std::string& x)
{
  return static_cast<std::size_t>(std::find(v.begin(), v.end(), x) - v.begin());
}

std::string acceptedText(const std::vector<EvidenceClass>& a)
{
  std::vector<std::string> t;
  for (const auto e : a) t.emplace_back(toToken(e));
  return join(t);
}

bool isAccepted(const std::vector<EvidenceClass>& a, uint8_t ev)
{
  return std::any_of(a.begin(), a.end(), [ev](EvidenceClass e) { return static_cast<uint8_t>(e) == ev; });
}

SiteEval evalSite(const JudgeInput& in, const State& s, std::size_t i, bool tool_level)
{
  const SiteSpec& sp = in.device->sites[i];
  const std::string site_subject = siteIdentity(in.identity, sp.id);
  const std::string grip_subject = actuatorIdentity(in.identity, sp.grip);
  const Act& e = s.actuators[indexOf(in.device->actuator_ids, sp.grip)];
  const int64_t tol = saturatedMs(in.clock_tolerance_ms) * kNsPerMs;
  const int64_t last_stamp = toNs(e.last_command_stamp);
  SiteEval r;
  r.subject = site_subject;
  auto fail = [&r](Reason why, std::string detail) {
    r.kind = Kind::FAIL;
    r.reason = why;
    r.detail = std::move(detail);
    return r;
  };

  // --- the reference rule: the grip's entry must show what THIS runner sent ---------------------
  const auto sent = in.sent.find(sp.grip);
  if (sent != in.sent.end())
  {
    const bool shows = e.last_command != Act::COMMAND_NONE && last_stamp >= sent->second.stamp_ns - tol &&
                       last_stamp > sent->second.prior_ns;
    if (!shows)
    {
      r.kind = Kind::WAIT_REFLECT;
      r.subject = grip_subject;
      r.detail = "the device's state never showed the " + std::string(commandToken(sent->second.command)) +
                 " this runner sent to " + sp.grip + " (its newest applied command: " +
                 commandToken(e.last_command) + "): the command went somewhere else";
      return r;
    }
  }
  bool has_ref = false;
  int64_t ref = 0;
  if (in.expect == Expect::HELD)
  {
    if (e.last_command != Act::COMMAND_ENGAGE)
    {
      if (tool_level) return r;   // SKIP
      return fail(Reason::NO_ENGAGE_COMMAND, "the newest applied command of " + sp.grip + " is " +
                                               commandToken(e.last_command) + ", not an engage");
    }
    has_ref = true;
    ref = last_stamp;
  }
  else if (e.last_command == Act::COMMAND_RELEASE)
  {
    has_ref = true;
    ref = last_stamp;
  }
  else if (e.last_command != Act::COMMAND_NONE)
  {
    return fail(Reason::NO_RELEASE_COMMAND, sp.grip + " is still engaged (its newest applied command: " +
                                              commandToken(e.last_command) + ")");
  }
  // with no command at all, a fresh NOT_HELD is enough: "the tool is empty"

  // --- check 3: a judged stamp in the future ----------------------------------------------------
  const int64_t site_stamp = toNs(s.site_stamp[i]);
  if (site_stamp > in.now_ns + tol)
    return fail(Reason::CLOCK_MISMATCH, "site_stamp is " + msText(site_stamp - in.now_ns) +
                                          " ahead of this runner's clock (tolerance " +
                                          std::to_string(in.clock_tolerance_ms) + " ms)");
  if (has_ref && ref > in.now_ns + tol)
    return fail(Reason::CLOCK_MISMATCH, "the last_command_stamp of " + sp.grip + " is " +
                                          msText(ref - in.now_ns) + " ahead of this runner's clock (tolerance " +
                                          std::to_string(in.clock_tolerance_ms) + " ms)");
  r.judged = true;   // check 4 is the device's: the caller runs it before using this evaluation

  // --- check 5: a site its backend cannot observe (a property of the binding, stamped 0) --------
  const uint8_t grasp = s.site_grasp[i];
  if (grasp == State::GRASP_UNAVAILABLE)
    return fail(Reason::VERIFICATION_UNAVAILABLE,
                "the adapter '" + s.adapter + "' cannot observe a grasp at " + sp.id + " on this backend");

  // --- fresh(s): at or after the reference, and no older than max_age -----------------------------
  if (has_ref && site_stamp < ref)
  {
    r.kind = Kind::WAIT_STALE;
    r.detail = "no observation at or after the " + std::string(commandToken(e.last_command)) + " of " +
               sp.grip + " (site_stamp " + msText(ref - site_stamp) + " before it)";
    return r;
  }
  if (in.now_ns - site_stamp > saturatedMs(in.max_age_ms) * kNsPerMs)
  {
    r.kind = Kind::WAIT_STALE;
    r.detail = "site_stamp is " + msText(in.now_ns - site_stamp) + " old (max_age " +
               std::to_string(in.max_age_ms) + " ms)";
    return r;
  }
  r.fresh = true;

  // --- checks 6-8, on a fresh sample ------------------------------------------------------------
  // a constant the token file does not know reads as UNKNOWN (grasp) or NONE (evidence)
  const uint8_t evidence = s.site_evidence[i] <= State::EVIDENCE_SIM_ATTACHMENT ? s.site_evidence[i]
                                                                               : State::EVIDENCE_NONE;
  if (evidence != State::EVIDENCE_NONE && !isAccepted(in.accepted, evidence))
    return fail(Reason::EVIDENCE_NOT_ACCEPTED, std::string(graspToken(grasp)) + " rests on " + evidenceToken(evidence) +
                                                 " evidence; accepted: " + acceptedText(in.accepted));
  if (grasp == State::GRASP_UNKNOWN || grasp > State::GRASP_LOST)
  {
    r.kind = Kind::WAIT_UNKNOWN;
    r.detail = sp.id + " is UNKNOWN (no valid observation)";
    return r;
  }
  if (evidence == State::EVIDENCE_NONE)   // a decided grasp must name its provenance
    return fail(Reason::EVIDENCE_NOT_ACCEPTED,
                std::string(graspToken(grasp)) + " with evidence none (a malformed sample)");
  if (in.expect == Expect::HELD)
  {
    switch (grasp)
    {
      case State::GRASP_LOST:
        return fail(Reason::OBJECT_LOST, sp.id + " LOST its object after the engage of " + sp.grip);
      case State::GRASP_NOT_HELD:
        return fail(Reason::NO_OBJECT, sp.id + " observed NOT_HELD (" + evidenceToken(evidence) + ") after the engage of " + sp.grip);
      case State::GRASP_PENDING:
        r.kind = Kind::WAIT_PENDING;
        r.detail = sp.id + " still PENDING (" + evidenceToken(evidence) + ")";
        return r;
      default:   // HELD
        r.kind = Kind::OK;
        r.inferred = evidence == State::EVIDENCE_ACTUATION_INFERENCE;
        return r;
    }
  }
  if (grasp == State::GRASP_NOT_HELD)
  {
    r.kind = Kind::OK;
    return r;
  }
  r.kind = Kind::WAIT_NOT_RELEASED;
  r.detail = sp.id + " still " + graspToken(grasp) + " (" + evidenceToken(evidence) + ")";
  return r;
}

// Check 4, once for the device.
std::optional<JudgeResult> toolStatus(const JudgeInput& in, const State& s)
{
  auto culprit = [&](uint8_t status) {
    for (std::size_t j = 0; j < s.actuators.size(); ++j)
      if (s.actuators[j].status == status) return actuatorIdentity(in.identity, s.actuators[j].actuator);
    return in.identity;
  };
  if (s.status == State::STATUS_FAULT)
    return failure(Reason::HARDWARE_FAULT, culprit(Act::STATUS_FAULT), "tool status FAULT: " + s.detail);
  if (s.status == State::STATUS_UNAVAILABLE)
    return failure(Reason::EEF_UNAVAILABLE, culprit(Act::STATUS_UNAVAILABLE), "tool status UNAVAILABLE: " + s.detail);
  return std::nullopt;
}

// Checks 1 and 2.
std::optional<JudgeResult> identityAndStructure(const JudgeInput& in, const State& s)
{
  const DeviceSpec& d = *in.device;
  if (s.end_effector != in.identity)
    return failure(Reason::WRONG_EEF, in.identity,
                   "the sample identifies '" + s.end_effector + "'; this runner expects '" + in.identity + "'");
  if (in.publishers > 1)
    return failure(Reason::CONTRACT_MISMATCH, in.identity,
                   std::to_string(in.publishers) + " publishers on the device's state topic; ONE adapter writes it");
  const std::size_t n = s.sites.size();
  if (s.site_grip.size() != n || s.site_grasp.size() != n || s.site_evidence.size() != n || s.site_stamp.size() != n)
    return failure(Reason::CONTRACT_MISMATCH, in.identity,
                   "the site arrays have unequal lengths (sites " + std::to_string(n) + ", site_grip " +
                     std::to_string(s.site_grip.size()) + ", site_grasp " + std::to_string(s.site_grasp.size()) +
                     ", site_evidence " + std::to_string(s.site_evidence.size()) + ", site_stamp " +
                     std::to_string(s.site_stamp.size()) + "): a malformed sample");
  if (s.sites != d.site_ids)
    return failure(Reason::CONTRACT_MISMATCH, in.identity,
                   "sites " + join(s.sites) + " differ from the contract's " + join(d.site_ids));
  std::vector<std::string> grips;
  for (const auto& site : d.sites) grips.push_back(site.grip);
  if (s.site_grip != grips)
    return failure(Reason::CONTRACT_MISMATCH, in.identity,
                   "site_grip " + join(s.site_grip) + " differs from the contract's " + join(grips));
  std::vector<std::string> ids;
  for (const auto& a : s.actuators) ids.push_back(a.actuator);
  if (ids != d.actuator_ids)
    return failure(Reason::CONTRACT_MISMATCH, in.identity,
                   "actuators " + join(ids) + " differ from the contract's " + join(d.actuator_ids));
  for (std::size_t j = 0; j < d.actuators.size(); ++j)
    if (s.actuators[j].command_interface != static_cast<uint8_t>(d.actuators[j].command_interface))
      return failure(Reason::CONTRACT_MISMATCH, in.identity,
                     d.actuators[j].id + " is published with command_interface " +
                       std::to_string(s.actuators[j].command_interface) + ", the contract says " +
                       toToken(d.actuators[j].command_interface));
  return std::nullopt;
}

JudgeResult fromEval(const SiteEval& e, bool at_deadline)
{
  switch (e.kind)
  {
    case Kind::OK:
    {
      JudgeResult j = make(Verdict::SUCCESS, Reason::NONE, e.subject, e.detail);
      j.inferred = e.inferred;
      return j;
    }
    case Kind::FAIL:
      return failure(e.reason, e.subject, e.detail);
    default:
      if (at_deadline && isWait(e.kind)) return failure(deadlineReason(e.kind), e.subject, e.detail);
      return make(Verdict::WAIT, Reason::NONE, e.subject, e.detail);
  }
}

JudgeResult singleSite(const JudgeInput& in, const State& s, std::size_t i)
{
  const SiteEval e = evalSite(in, s, i, false);
  if (e.judged)   // check 4 before check 5 and the freshness gate
    if (auto st = toolStatus(in, s)) return *st;
  return fromEval(e, in.at_deadline);
}

JudgeResult toolLevelHeld(const JudgeInput& in, const State& s)
{
  std::vector<SiteEval> evals;
  for (std::size_t i = 0; i < in.device->sites.size(); ++i)
  {
    SiteEval e = evalSite(in, s, i, true);
    if (e.kind != Kind::SKIP) evals.push_back(std::move(e));
  }
  if (evals.empty())
    return failure(Reason::NO_ENGAGE_COMMAND, in.identity, "no site's grip actuator has a current engage");
  for (const auto& e : evals)   // check 3, once for the device
    if (e.kind == Kind::FAIL && !e.judged) return failure(e.reason, e.subject, e.detail);
  if (std::any_of(evals.begin(), evals.end(), [](const SiteEval& e) { return e.judged; }))
    if (auto st = toolStatus(in, s)) return *st;   // check 4, once for the device
  for (const auto& e : evals)   // a LOST site fails the check
    if (e.kind == Kind::FAIL && e.reason == Reason::OBJECT_LOST) return failure(e.reason, e.subject, e.detail);
  const bool waiting = std::any_of(evals.begin(), evals.end(), [](const SiteEval& e) { return isWait(e.kind); });
  if (waiting && !in.at_deadline)
  {
    const auto w = std::find_if(evals.begin(), evals.end(), [](const SiteEval& e) { return isWait(e.kind); });
    return make(Verdict::WAIT, Reason::NONE, w->subject, w->detail);
  }
  const auto held = std::find_if(evals.begin(), evals.end(), [](const SiteEval& e) { return e.kind == Kind::OK; });
  if (held != evals.end())
  {
    JudgeResult j = make(Verdict::SUCCESS, Reason::NONE, held->subject, "held through " + held->subject);
    j.inferred = held->inferred;
    return j;
  }
  // no site holds: the first non-VERDICT reason in contract order, otherwise the first VERDICT
  const SiteEval* first_verdict = nullptr;
  for (const auto& e : evals)
  {
    const Reason r = e.kind == Kind::FAIL ? e.reason : deadlineReason(e.kind);
    if (isFault(r)) return failure(r, e.subject, e.detail);
    if (r != Reason::NONE && first_verdict == nullptr) first_verdict = &e;
  }
  if (first_verdict)
    return failure(first_verdict->kind == Kind::FAIL ? first_verdict->reason : deadlineReason(first_verdict->kind),
                   first_verdict->subject, first_verdict->detail);
  return failure(Reason::NOT_CONFIRMED_TIMEOUT, in.identity, "no site confirmed a grasp");
}

JudgeResult toolLevelReleased(const JudgeInput& in, const State& s)
{
  std::vector<SiteEval> evals;
  for (std::size_t i = 0; i < in.device->sites.size(); ++i) evals.push_back(evalSite(in, s, i, false));
  for (const auto& e : evals)   // check 3 and the command rule, before any observation is judged
    if (e.kind == Kind::FAIL && !e.judged) return failure(e.reason, e.subject, e.detail);
  if (std::any_of(evals.begin(), evals.end(), [](const SiteEval& e) { return e.judged; }))
    if (auto st = toolStatus(in, s)) return *st;   // check 4, once for the device
  for (const auto& e : evals)   // a decided failure: the device cannot read as released
    if (e.kind == Kind::FAIL) return failure(e.reason, e.subject, e.detail);
  const bool waiting = std::any_of(evals.begin(), evals.end(), [](const SiteEval& e) { return isWait(e.kind); });
  if (waiting && !in.at_deadline)
  {
    const auto w = std::find_if(evals.begin(), evals.end(), [](const SiteEval& e) { return isWait(e.kind); });
    return make(Verdict::WAIT, Reason::NONE, w->subject, w->detail);
  }
  for (const auto& e : evals)   // the first site in contract order that is not released(s)
    if (e.kind != Kind::OK) return failure(deadlineReason(e.kind), e.subject, e.detail);
  return make(Verdict::SUCCESS, Reason::NONE, in.identity, "every site released");
}
}  // namespace

std::string actuatorIdentity(const std::string& device_identity, const std::string& actuator)
{
  return device_identity + kSep + "actuators" + kSep + actuator;
}

std::string siteIdentity(const std::string& device_identity, const std::string& site)
{
  return device_identity + kSep + "sites" + kSep + site;
}

JudgeResult judge(const JudgeInput& in)
{
  if (in.device == nullptr) return failure(Reason::UNKNOWN_EEF, in.identity, "no such device in the contract");
  if (in.sample == nullptr)
  {
    if (in.at_deadline) return failure(Reason::STATE_MISSING, in.identity, "no EndEffectorState sample arrived");
    return make(Verdict::WAIT, Reason::NONE, in.identity, "no sample yet");
  }
  const State& s = *in.sample;
  if (auto bad = identityAndStructure(in, s)) return *bad;

  if (!in.site.empty())
  {
    const auto i = indexOf(in.device->site_ids, in.site);
    if (i >= in.device->site_ids.size())
      return failure(Reason::UNKNOWN_SITE, siteIdentity(in.identity, in.site), "no such site in the contract");
    return singleSite(in, s, i);
  }
  if (in.device->sites.size() == 1) return singleSite(in, s, 0);
  return in.expect == Expect::HELD ? toolLevelHeld(in, s) : toolLevelReleased(in, s);
}

// --- the message constants' tokens --------------------------------------------------------------
const char* graspToken(uint8_t v)
{
  switch (v)
  {
    case State::GRASP_UNAVAILABLE: return "unavailable";
    case State::GRASP_UNKNOWN: return "unknown";
    case State::GRASP_NOT_HELD: return "not_held";
    case State::GRASP_PENDING: return "pending";
    case State::GRASP_HELD: return "held";
    case State::GRASP_LOST: return "lost";
    default: return "?";
  }
}

const char* evidenceToken(uint8_t v)
{
  switch (v)
  {
    case State::EVIDENCE_NONE: return "none";
    case State::EVIDENCE_COMMAND_ECHO: return "command_echo";
    case State::EVIDENCE_ACTUATION_INFERENCE: return "actuation_inference";
    case State::EVIDENCE_SENSOR: return "sensor";
    case State::EVIDENCE_SIM_GROUND_TRUTH: return "sim_ground_truth";
    case State::EVIDENCE_SIM_ATTACHMENT: return "sim_attachment";
    default: return "?";
  }
}

const char* toolStatusToken(uint8_t v)
{
  switch (v)
  {
    case State::STATUS_UNKNOWN: return "unknown";
    case State::STATUS_READY: return "ready";
    case State::STATUS_BUSY: return "busy";
    case State::STATUS_FAULT: return "fault";
    case State::STATUS_UNAVAILABLE: return "unavailable";
    default: return "?";
  }
}

const char* actuatorStatusToken(uint8_t v)
{
  switch (v)
  {
    case Act::STATUS_UNKNOWN: return "unknown";
    case Act::STATUS_IDLE: return "idle";
    case Act::STATUS_MOVING: return "moving";
    case Act::STATUS_FAULT: return "fault";
    case Act::STATUS_UNAVAILABLE: return "unavailable";
    default: return "?";
  }
}

const char* binaryToken(uint8_t v)
{
  switch (v)
  {
    case Act::BINARY_UNKNOWN: return "unknown";
    case Act::BINARY_RELEASED: return "released";
    case Act::BINARY_ENGAGED: return "engaged";
    default: return "?";
  }
}

const char* commandToken(uint8_t v)
{
  switch (v)
  {
    case Act::COMMAND_NONE: return "none";
    case Act::COMMAND_ENGAGE: return "engage";
    case Act::COMMAND_RELEASE: return "release";
    case Act::COMMAND_TARGET: return "target";
    default: return "?";
  }
}

}  // namespace eef
}  // namespace trainit
