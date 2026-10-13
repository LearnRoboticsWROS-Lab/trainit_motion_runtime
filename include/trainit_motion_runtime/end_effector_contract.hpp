// =============================================================================
// end_effector_contract.hpp — the canonical end-effector contract, as TMR reads it (ADR-0022).
//
// A canonical project's bt_params carry an `eef` parameter block (ADR-0022 D6 "Emission"):
// devices -> actuators (one standard command interface each: std_srvs/SetBool,
// control_msgs GripperCommand or FollowJointTrajectory) -> grasp sites (one grip actuator
// each) -> a verification policy. This header is the typed form of that block and its parser.
//
// The parser is PURE (no ROS graph): it reads a flat {dotted name: value} map, the way a node
// sees its parameter overrides, and either returns the typed contract or throws Refused{key,
// why}. Its behaviour is the reference normaliser's (`normalize_eef.py`, a byte copy pinned in
// test/data/m2_6_contract/): it accepts what that accepts, refuses what that refuses with the
// SAME refusal key, checks in the same order, and dump() prints the same normalised text, byte
// for byte (test_m2_6_eef_contract and test_m2_6_eef_reference prove it on the conformance
// fixture and on a table of mutations of it).
//
// LEGACY iff the block is absent (no `eef.*` parameter at all): the contract is then empty and
// every legacy path runs exactly as before (ADR-0022 D9). The runner exits with code 2 on any
// Refused (ADR-0022 D5).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__END_EFFECTOR_CONTRACT_HPP_
#define TRAINIT_MOTION_RUNTIME__END_EFFECTOR_CONTRACT_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/parameter_value.hpp>

namespace rclcpp
{
class Node;
}

namespace trainit
{
namespace eef
{

// The values are the EndEffectorActuatorState INTERFACE_* constants (trainit_interfaces).
enum class CommandInterface : uint8_t
{
  UNKNOWN = 0,
  SET_BOOL = 1,
  GRIPPER_COMMAND = 2,
  FOLLOW_JOINT_TRAJECTORY = 3,
};

enum class ResultPolicy : uint8_t
{
  SUCCEEDED,              // the engage target is DONE on SUCCEEDED only
  SUCCEEDED_OR_STALLED,   // ... or on ABORTED with `stalled`
};

// Who publishes the planning-scene signal eef/<id>/engaged (ADR-0022 D8).
enum class SceneSignal : uint8_t
{
  ADAPTER,   // the device's TrainIt adapter (a set_bool grip)
  TMR,       // this runner, on command DONE (a gripper_command grip)
  NONE,      // no scene signal (a multi-site device)
};

// The values are the EndEffectorState EVIDENCE_* constants (trainit_interfaces).
enum class EvidenceClass : uint8_t
{
  NONE = 0,
  COMMAND_ECHO = 1,
  ACTUATION_INFERENCE = 2,
  SENSOR = 3,
  SIM_GROUND_TRUTH = 4,
  SIM_ATTACHMENT = 5,
};

const char* toToken(CommandInterface i);   // "set_bool" | "gripper_command" | "follow_joint_trajectory" | "unknown"
const char* toToken(ResultPolicy p);       // "succeeded" | "succeeded_or_stalled"
const char* toToken(SceneSignal s);        // "adapter" | "tmr" | "none"
const char* toToken(EvidenceClass e);      // "none" | "command_echo" | ... | "sim_attachment"

// The evidence classes a project may ACCEPT (ADR-0022 D4, EEF-D8): never command_echo or
// sim_attachment.
std::optional<EvidenceClass> acceptedEvidenceFromToken(const std::string& token);

// A malformed contract (ADR-0022 D5: the runner exits with code 2). `key` is the dotted
// parameter name the reference normaliser names; what() is "<key>: <why>".
class Refused : public std::runtime_error
{
public:
  Refused(std::string key, std::string why);
  std::string key;
  std::string why;
};

// ONE independently commandable part of a device (ADR-0022 D0, D2).
struct ActuatorSpec
{
  std::string id;
  CommandInterface command_interface{CommandInterface::UNKNOWN};
  std::vector<std::string> joints;            // the joints it drives; may be empty for set_bool

  // set_bool
  std::vector<double> engaged_positions;      // one per joint, or empty (no joints)
  std::vector<double> released_positions;
  bool actuation_feedback{false};             // a switch ON the actuator confirms `binary`
  int64_t confirm_timeout_ms{2000};

  // gripper_command and follow_joint_trajectory
  std::string action;                         // relative
  // named targets: gripper_command one position each; follow_joint_trajectory one per joint
  std::map<std::string, std::vector<double>> targets;

  // gripper_command
  ResultPolicy result_policy{ResultPolicy::SUCCEEDED};
  std::optional<double> max_effort;

  // follow_joint_trajectory
  std::vector<double> goal_tolerance;         // one per joint
  double duration_s{0.0};
  double goal_time_tolerance_s{0.5};
  std::vector<double> path_tolerance;         // one per joint, or empty (not declared)

  // The named targets this actuator can be commanded to: "engage"/"release" for set_bool.
  bool hasTarget(const std::string& name) const;
  std::vector<std::string> targetNames() const;
};

// Where the device holds an object (ADR-0022 D0): exactly one grip actuator.
struct SiteSpec
{
  std::string id;
  std::string grip;      // the actuator that grips this site
  std::string engage;    // that actuator's engage target
  std::string release;   // that actuator's release target
};

struct VerificationSpec
{
  bool require_grasp_confirmation{false};
  bool require_release_confirmation{false};
  std::vector<EvidenceClass> accepted_evidence;   // in the declared order
  int64_t timeout_ms{3000};
  int64_t max_age_ms{500};
  int64_t clock_tolerance_ms{100};

  bool accepts(EvidenceClass e) const;
};

// ONE end-effector device on this robot's flange (ADR-0022 D0).
struct DeviceSpec
{
  std::string id;
  std::vector<std::string> actuator_ids;   // contract order
  std::vector<std::string> site_ids;       // contract order
  int64_t command_timeout_ms{10000};
  SceneSignal scene_signal{SceneSignal::NONE};
  std::vector<ActuatorSpec> actuators;     // in actuator_ids order
  std::vector<SiteSpec> sites;             // in site_ids order
  VerificationSpec verification;

  const ActuatorSpec* actuator(const std::string& id) const;
  const SiteSpec* site(const std::string& id) const;
  const SiteSpec* siteGrippedBy(const std::string& actuator_id) const;   // null: grips no site
};

class Contract
{
public:
  // false: LEGACY (no eef block)
  bool canonical() const { return !devices.empty(); }
  const DeviceSpec* device(const std::string& id) const;
  std::vector<std::string> ids() const;
  // The normalised form, exactly normalize_eef.py's dump(): one "eef.<key> = <value>" line per
  // leaf, sorted; defaults filled in; "" for a LEGACY contract.
  std::string dump() const;

  std::vector<DeviceSpec> devices;              // in eef.ids order
  std::map<std::string, std::string> normalized;   // key (no "eef." prefix) -> formatted value
};

using ParameterMap = std::map<std::string, rclcpp::ParameterValue>;

// Parses the flat {dotted name: value} map. Throws Refused.
Contract parseContract(const ParameterMap& params);

// Parses the node's parameter OVERRIDES (what its params files and -p arguments set), not its
// declared parameters: a key a runner declares with a default (gripper_cmd_topic) is not "set".
// Throws Refused.
Contract parseContract(rclcpp::Node& node);

}  // namespace eef
}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__END_EFFECTOR_CONTRACT_HPP_
