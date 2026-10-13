// =============================================================================
// verify_grasp.hpp — the VerifyGrasp judge (ADR-0022 D4), as one pure function.
//
// VerifyGrasp asks the TOOL, never an actuator: "is the object held (or released) at this site,
// or anywhere on this device?". The judge reads the newest EndEffectorState sample of the
// device, this runner's own send stamps, `now` (the runner's ROS clock) and the node's ports,
// and answers SUCCESS, WAIT or FAILURE(reason, subject). The caller loops on WAIT until its
// steady-clock deadline, then asks once more with `at_deadline`, which turns the wait into its
// reason: STATE_MISSING (no sample at all) > STATE_STALE (only old samples) > STATE_UNKNOWN
// (still UNKNOWN) > NOT_CONFIRMED_TIMEOUT (still PENDING and fresh); RELEASE_NOT_CONFIRMED for a
// release; CONTRACT_MISMATCH when the device's state never showed the command THIS runner sent.
//
// The reference rule, on the ADAPTER's clock: a sample is judged for site s only when the
// entry of s's grip actuator REFLECTS the reference command (last_command ENGAGE, or RELEASE
// for `released`, with last_command_stamp >= this runner's send stamp - clock tolerance, and
// newer than the entry this runner saw when it sent, when this runner sent it), and its
// site_stamp is compared with that entry's last_command_stamp --
// so a sample measured between the send and the adapter's apply is never judged, and the two
// clocks are compared only through the tolerance.
//
// Checks, in order. Checks 4 and 5 (the device and its binding) run once the reference rule and
// check 3 passed; checks 6-8, and every verdict, also need the site's sample FRESH (an adapter
// stamps an unobservable site 0, which is never fresh):
//   1 WRONG_EEF   2 CONTRACT_MISMATCH (publishers, ids, order, site_grip, interfaces, arrays)
//   3 CLOCK_MISMATCH   4 HARDWARE_FAULT / EEF_UNAVAILABLE   5 VERIFICATION_UNAVAILABLE
//   6 EVIDENCE_NOT_ACCEPTED (PENDING and LOST included)   7 OBJECT_LOST   8 NO_OBJECT
// Tool level (no site on a multi-site device): checks 1-4 once for the device; `held` skips
// the sites with no current engage and succeeds iff some evaluated site is held and none is
// LOST, waiting while one is undecided; `released` needs every site released.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__VERIFY_GRASP_HPP_
#define TRAINIT_MOTION_RUNTIME__VERIFY_GRASP_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <trainit_interfaces/msg/end_effector_state.hpp>

#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_reason.hpp"

namespace trainit
{
namespace eef
{

enum class Expect : uint8_t
{
  HELD,
  RELEASED,
};

// The newest command THIS runner sent to one actuator and the server applied (SetBool
// success:true, or an accepted goal), stamped on the runner's clock when it was sent.
struct SentCommand
{
  uint8_t command{0};   // EndEffectorActuatorState::COMMAND_ENGAGE | COMMAND_RELEASE | COMMAND_TARGET
  int64_t stamp_ns{0};
  // The actuator entry's last_command_stamp in the newest sample this runner held when it sent
  // (0: none). A reflection of THIS send must be strictly newer: two applies within the clock
  // tolerance (a re-engage) are otherwise indistinguishable, and the earlier one would be judged.
  int64_t prior_ns{0};
};

struct JudgeInput
{
  const DeviceSpec* device{nullptr};
  std::string identity{};                                            // this runner's expansion of eef/<id>
  const trainit_interfaces::msg::EndEffectorState* sample{nullptr};  // the newest; null = none yet
  std::map<std::string, SentCommand> sent{};                         // per actuator id
  int64_t now_ns{0};                                                 // the runner's ROS clock
  std::string site{};                                                // "" = the only site, or tool level
  Expect expect{Expect::HELD};
  int64_t max_age_ms{500};
  int64_t clock_tolerance_ms{100};
  std::vector<EvidenceClass> accepted{};                             // the effective set
  std::size_t publishers{1};                                         // on eef/<id>/state
  bool at_deadline{false};
};

enum class Verdict : uint8_t
{
  SUCCESS,
  WAIT,
  FAILURE,
};

struct JudgeResult
{
  Verdict verdict{Verdict::WAIT};
  Reason reason{Reason::NONE};
  ReasonClass reason_class{ReasonClass::NONE};   // classOf(reason): VERDICT, or the FAULT's class
  std::string subject{};
  std::string detail{};
  bool inferred{false};   // SUCCESS rests on ACTUATION_INFERENCE evidence (EEF-D8: said so)
};

JudgeResult judge(const JudgeInput& in);

// "<device>/actuators/<id>" and "<device>/sites/<id>" (ADR-0022 D3).
std::string actuatorIdentity(const std::string& device_identity, const std::string& actuator);
std::string siteIdentity(const std::string& device_identity, const std::string& site);

// The canonical token of a message constant ("held", "sensor", "ready", ...), as in
// eef_contract_tokens.txt; "?" for a value the token file does not know.
const char* graspToken(uint8_t v);
const char* evidenceToken(uint8_t v);
const char* toolStatusToken(uint8_t v);
const char* actuatorStatusToken(uint8_t v);
const char* binaryToken(uint8_t v);
const char* commandToken(uint8_t v);

}  // namespace eef
}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__VERIFY_GRASP_HPP_
