// =============================================================================
// end_effector_reason.hpp — the failure vocabulary of the end-effector contract (ADR-0022 D5).
//
// Every canonical end-effector node (CloseGripper, OpenGripper, CommandActuator, VerifyGrasp)
// fails with exactly ONE of these reasons, and the reason's CLASS decides what a tree may do
// with it: a VERDICT ("the object is not there") may be absorbed by bin picking's candidate
// retry; every other class is a FAULT, which no generated retry absorbs and which the runner
// latches (exit code 3 when the tree still ends SUCCESS).
//
// The tokens are the canonical ones of `eef_contract_tokens.txt` (a byte copy is pinned in
// test/data/m2_6_contract/ and checked against these tables by test_m2_6_eef_contract).
// Header-only and pure: no ROS, no state.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REASON_HPP_
#define TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REASON_HPP_

#include <array>
#include <cstdint>

namespace trainit
{
namespace eef
{

enum class Reason : uint8_t
{
  NONE = 0,   // not a failure (DONE / SUCCESS)
  // VERDICT
  NO_OBJECT,
  NOT_CONFIRMED_TIMEOUT,
  OBJECT_LOST,
  // ACTUATOR_FAILURE
  SERVER_MISSING,
  SEND_TIMEOUT,
  COMMAND_REJECTED,
  COMMAND_TIMEOUT,
  COMMAND_ABORTED,
  COMMAND_CANCELED,
  PATH_TOLERANCE_VIOLATED,
  GOAL_TOLERANCE_VIOLATED,
  ACTUATOR_BLOCKED,
  ACTUATOR_UNAVAILABLE,
  ACTUATION_NOT_CONFIRMED,
  HARDWARE_FAULT,
  EEF_UNAVAILABLE,
  // STATE_PROVIDER_MISSING
  STATE_MISSING,
  // STALE_STATE
  STATE_STALE,
  STATE_UNKNOWN,
  // VERIFICATION_ERROR
  VERIFICATION_UNAVAILABLE,
  EVIDENCE_NOT_ACCEPTED,
  CLOCK_MISMATCH,
  NO_ENGAGE_COMMAND,
  NO_RELEASE_COMMAND,
  RELEASE_NOT_CONFIRMED,
  // CONTRACT_FAILURE
  WRONG_EEF,
  CONTRACT_MISMATCH,
  UNKNOWN_EEF,
  UNKNOWN_SITE,
  UNKNOWN_ACTUATOR,
  UNKNOWN_TARGET,
};

enum class ReasonClass : uint8_t
{
  NONE = 0,
  VERDICT,
  ACTUATOR_FAILURE,
  STATE_PROVIDER_MISSING,
  STALE_STATE,
  VERIFICATION_ERROR,
  CONTRACT_FAILURE,
};

// Every reason, in the order of the canonical token file.
inline constexpr std::array<Reason, 31> kAllReasons = {
  Reason::NO_OBJECT, Reason::NOT_CONFIRMED_TIMEOUT, Reason::OBJECT_LOST,
  Reason::SERVER_MISSING, Reason::SEND_TIMEOUT, Reason::COMMAND_REJECTED,
  Reason::COMMAND_TIMEOUT, Reason::COMMAND_ABORTED, Reason::COMMAND_CANCELED,
  Reason::PATH_TOLERANCE_VIOLATED, Reason::GOAL_TOLERANCE_VIOLATED, Reason::ACTUATOR_BLOCKED,
  Reason::ACTUATOR_UNAVAILABLE, Reason::ACTUATION_NOT_CONFIRMED, Reason::HARDWARE_FAULT,
  Reason::EEF_UNAVAILABLE, Reason::STATE_MISSING, Reason::STATE_STALE, Reason::STATE_UNKNOWN,
  Reason::VERIFICATION_UNAVAILABLE, Reason::EVIDENCE_NOT_ACCEPTED, Reason::CLOCK_MISMATCH,
  Reason::NO_ENGAGE_COMMAND, Reason::NO_RELEASE_COMMAND, Reason::RELEASE_NOT_CONFIRMED,
  Reason::WRONG_EEF, Reason::CONTRACT_MISMATCH, Reason::UNKNOWN_EEF, Reason::UNKNOWN_SITE,
  Reason::UNKNOWN_ACTUATOR, Reason::UNKNOWN_TARGET};

constexpr const char* toToken(Reason r)
{
  switch (r)
  {
    case Reason::NONE: return "";
    case Reason::NO_OBJECT: return "NO_OBJECT";
    case Reason::NOT_CONFIRMED_TIMEOUT: return "NOT_CONFIRMED_TIMEOUT";
    case Reason::OBJECT_LOST: return "OBJECT_LOST";
    case Reason::SERVER_MISSING: return "SERVER_MISSING";
    case Reason::SEND_TIMEOUT: return "SEND_TIMEOUT";
    case Reason::COMMAND_REJECTED: return "COMMAND_REJECTED";
    case Reason::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    case Reason::COMMAND_ABORTED: return "COMMAND_ABORTED";
    case Reason::COMMAND_CANCELED: return "COMMAND_CANCELED";
    case Reason::PATH_TOLERANCE_VIOLATED: return "PATH_TOLERANCE_VIOLATED";
    case Reason::GOAL_TOLERANCE_VIOLATED: return "GOAL_TOLERANCE_VIOLATED";
    case Reason::ACTUATOR_BLOCKED: return "ACTUATOR_BLOCKED";
    case Reason::ACTUATOR_UNAVAILABLE: return "ACTUATOR_UNAVAILABLE";
    case Reason::ACTUATION_NOT_CONFIRMED: return "ACTUATION_NOT_CONFIRMED";
    case Reason::HARDWARE_FAULT: return "HARDWARE_FAULT";
    case Reason::EEF_UNAVAILABLE: return "EEF_UNAVAILABLE";
    case Reason::STATE_MISSING: return "STATE_MISSING";
    case Reason::STATE_STALE: return "STATE_STALE";
    case Reason::STATE_UNKNOWN: return "STATE_UNKNOWN";
    case Reason::VERIFICATION_UNAVAILABLE: return "VERIFICATION_UNAVAILABLE";
    case Reason::EVIDENCE_NOT_ACCEPTED: return "EVIDENCE_NOT_ACCEPTED";
    case Reason::CLOCK_MISMATCH: return "CLOCK_MISMATCH";
    case Reason::NO_ENGAGE_COMMAND: return "NO_ENGAGE_COMMAND";
    case Reason::NO_RELEASE_COMMAND: return "NO_RELEASE_COMMAND";
    case Reason::RELEASE_NOT_CONFIRMED: return "RELEASE_NOT_CONFIRMED";
    case Reason::WRONG_EEF: return "WRONG_EEF";
    case Reason::CONTRACT_MISMATCH: return "CONTRACT_MISMATCH";
    case Reason::UNKNOWN_EEF: return "UNKNOWN_EEF";
    case Reason::UNKNOWN_SITE: return "UNKNOWN_SITE";
    case Reason::UNKNOWN_ACTUATOR: return "UNKNOWN_ACTUATOR";
    case Reason::UNKNOWN_TARGET: return "UNKNOWN_TARGET";
  }
  return "";
}

constexpr ReasonClass classOf(Reason r)
{
  switch (r)
  {
    case Reason::NONE:
      return ReasonClass::NONE;
    case Reason::NO_OBJECT:
    case Reason::NOT_CONFIRMED_TIMEOUT:
    case Reason::OBJECT_LOST:
      return ReasonClass::VERDICT;
    case Reason::SERVER_MISSING:
    case Reason::SEND_TIMEOUT:
    case Reason::COMMAND_REJECTED:
    case Reason::COMMAND_TIMEOUT:
    case Reason::COMMAND_ABORTED:
    case Reason::COMMAND_CANCELED:
    case Reason::PATH_TOLERANCE_VIOLATED:
    case Reason::GOAL_TOLERANCE_VIOLATED:
    case Reason::ACTUATOR_BLOCKED:
    case Reason::ACTUATOR_UNAVAILABLE:
    case Reason::ACTUATION_NOT_CONFIRMED:
    case Reason::HARDWARE_FAULT:
    case Reason::EEF_UNAVAILABLE:
      return ReasonClass::ACTUATOR_FAILURE;
    case Reason::STATE_MISSING:
      return ReasonClass::STATE_PROVIDER_MISSING;
    case Reason::STATE_STALE:
    case Reason::STATE_UNKNOWN:
      return ReasonClass::STALE_STATE;
    case Reason::VERIFICATION_UNAVAILABLE:
    case Reason::EVIDENCE_NOT_ACCEPTED:
    case Reason::CLOCK_MISMATCH:
    case Reason::NO_ENGAGE_COMMAND:
    case Reason::NO_RELEASE_COMMAND:
    case Reason::RELEASE_NOT_CONFIRMED:
      return ReasonClass::VERIFICATION_ERROR;
    case Reason::WRONG_EEF:
    case Reason::CONTRACT_MISMATCH:
    case Reason::UNKNOWN_EEF:
    case Reason::UNKNOWN_SITE:
    case Reason::UNKNOWN_ACTUATOR:
    case Reason::UNKNOWN_TARGET:
      return ReasonClass::CONTRACT_FAILURE;
  }
  return ReasonClass::NONE;
}

constexpr const char* toToken(ReasonClass c)
{
  switch (c)
  {
    case ReasonClass::NONE: return "";
    case ReasonClass::VERDICT: return "VERDICT";
    case ReasonClass::ACTUATOR_FAILURE: return "ACTUATOR_FAILURE";
    case ReasonClass::STATE_PROVIDER_MISSING: return "STATE_PROVIDER_MISSING";
    case ReasonClass::STALE_STATE: return "STALE_STATE";
    case ReasonClass::VERIFICATION_ERROR: return "VERIFICATION_ERROR";
    case ReasonClass::CONTRACT_FAILURE: return "CONTRACT_FAILURE";
  }
  return "";
}

// A FAULT is any failure whose class is not VERDICT (ADR-0022 D5).
constexpr bool isFault(Reason r)
{
  return r != Reason::NONE && classOf(r) != ReasonClass::VERDICT;
}

}  // namespace eef
}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REASON_HPP_
