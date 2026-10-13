// =============================================================================
// end_effector_registry.hpp — the runner's end-effector devices (ADR-0022 D2, D4, D7, D8).
//
// Built once at runner start-up from the canonical contract (end_effector_contract.hpp), ONE
// EndEffectorDevice per device, each with:
//   - a MutuallyExclusive callback group for everything below;
//   - one standard client per actuator, at its RELATIVE name (resolved in the runner's cell
//     namespace): std_srvs/SetBool on eef/<id>/actuators/<a>/set_engaged, or the declared
//     control_msgs GripperCommand / FollowJointTrajectory action;
//   - ONE subscription to eef/<id>/state (RELIABLE, TRANSIENT_LOCAL, depth 1): the newest
//     sample, its arrival and a sequence number;
//   - the scene-signal publisher eef/<id>/engaged (latched Bool) when the block says
//     scene_signal: tmr;
//   - the send stamps of the commands THIS runner had applied (the runner's clock).
// The registry holds the devices, the fault latch (any FAULT reason -> exit code 3 when the
// tree still ends SUCCESS) and the start-up checks (tree literals, one writer per state topic).
//
// What a canonical node does is here, not in bt_nodes.cpp: CloseGripper/OpenGripper (grip),
// CommandActuator (actuate) and VerifyGrasp (verify) resolve their ports against the contract,
// command or judge, publish the scene signal on DONE when TMR is its source, log, and latch a
// FAULT. Command success is ACTUATION only; VerifyGrasp alone judges a grasp.
//
// No function-static cache and no absolute name: every name is relative to the runner's node
// (test_m2_6_eef_static_scan).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REGISTRY_HPP_
#define TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REGISTRY_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trainit_interfaces/msg/end_effector_state.hpp>

#include "trainit_motion_runtime/actuator_outcome.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_reason.hpp"
#include "trainit_motion_runtime/verify_grasp.hpp"

namespace BT
{
class Tree;
}

namespace trainit
{

// What one canonical end-effector node ended as.
struct EefResult
{
  bool ok{false};
  eef::Reason reason{eef::Reason::NONE};
  std::string subject{};   // the fully-qualified identity the reason is about
  std::string detail{};
  bool inferred{false};    // VerifyGrasp: SUCCESS confirmed by inference (actuation_inference)
};

class EndEffectorDevice
{
public:
  using State = trainit_interfaces::msg::EndEffectorState;
  using SetBool = std_srvs::srv::SetBool;
  using GripperCommand = control_msgs::action::GripperCommand;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  EndEffectorDevice(const rclcpp::Node::SharedPtr& node, const eef::DeviceSpec& spec);
  EndEffectorDevice(const EndEffectorDevice&) = delete;
  EndEffectorDevice& operator=(const EndEffectorDevice&) = delete;

  const eef::DeviceSpec& spec() const { return spec_; }
  // "<cell namespace>/eef/<id>": this runner's own expansion of eef/<id>, never configured.
  const std::string& identity() const { return identity_; }
  // The state topic as the subscription resolved it (remaps applied).
  std::string stateTopic() const;
  // The publishers on it; nullopt once the ROS context is shut down (or the graph query failed),
  // never read as "0 publishers", which would also read a TrainIt adapter as a third-party server.
  std::optional<std::size_t> statePublishers() const;

  // The newest sample (null: none yet), its arrival on the steady clock, its sequence number.
  struct Snapshot
  {
    std::shared_ptr<const State> sample{};
    std::chrono::steady_clock::time_point arrival{};
    uint64_t seq{0};
  };
  Snapshot latest() const;

  // Commands `actuator` to its named `target` and waits for DONE under its interface's rule
  // (ADR-0022 D2), then for "the command reached THIS device" and the switch confirmation.
  // `timeout_ms` replaces the device's command_timeout_ms when set (CommandActuator's port).
  // Every wait ends COMMAND_ABORTED ("interrupted: ...") once the ROS context shuts down.
  EefResult command(const std::string& actuator, const std::string& target, std::optional<int64_t> timeout_ms);

  // Waits up to `timeout_ms` (steady clock) for the judge (ADR-0022 D4).
  EefResult verify(const std::string& site, eef::Expect expect, int64_t timeout_ms, int64_t max_age_ms,
                   const std::vector<eef::EvidenceClass>& accepted);

  // eef/<id>/engaged, when scene_signal is tmr.
  void publishSceneSignal(bool engaged);
  bool tmrPublishesSceneSignal() const { return static_cast<bool>(scene_pub_); }

  // The commands this runner had applied, per actuator.
  std::map<std::string, eef::SentCommand> sent() const;

private:
  struct Feed;          // the state subscription's shared slot
  struct GoalWatch;     // a goal's late-accept cancel and its newest feedback
  // What the device's state showed when a command was sent: the actuator entry's
  // last_command_stamp (0: no sample, or not this device's) and the feed's sequence number.
  struct Prior
  {
    int64_t stamp_ns{0};
    uint64_t seq{0};
  };

  int64_t nowNs() const;
  bool rosOk() const;   // the node's ROS context is still valid
  EefResult interrupted(const std::string& subject) const;   // COMMAND_ABORTED: the context shut down
  uint8_t commandKind(const eef::ActuatorSpec& a, const std::string& target) const;
  EefResult failure(eef::Reason r, std::string subject, std::string detail) const;
  EefResult commandSetBool(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms);
  EefResult commandGripper(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms);
  EefResult commandTrajectory(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms);
  // the must-reach rule and, for set_bool with actuation_feedback, the switch confirmation;
  // `prior`: what the state showed when the command was sent (a reflection is newer)
  EefResult confirm(const eef::ActuatorSpec& a, uint8_t kind, int64_t sent_ns, const Prior& prior);
  void recordSent(const std::string& actuator, uint8_t kind, int64_t sent_ns, int64_t prior_ns);
  Prior priorOf(const std::string& actuator) const;

  rclcpp::Node::SharedPtr node_;
  eef::DeviceSpec spec_;
  std::string identity_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<Feed> feed_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  std::string state_topic_;   // as the subscription resolved it, read once
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scene_pub_;
  std::map<std::string, rclcpp::Client<SetBool>::SharedPtr> set_bool_;
  std::map<std::string, rclcpp_action::Client<GripperCommand>::SharedPtr> gripper_;
  std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> trajectory_;
  mutable std::mutex sent_m_;
  std::map<std::string, eef::SentCommand> sent_;
};

class EndEffectorRegistry
{
public:
  EndEffectorRegistry(const rclcpp::Node::SharedPtr& node, eef::Contract contract);
  EndEffectorRegistry(const EndEffectorRegistry&) = delete;
  EndEffectorRegistry& operator=(const EndEffectorRegistry&) = delete;

  const eef::Contract& contract() const { return contract_; }
  std::size_t size() const { return devices_.size(); }
  EndEffectorDevice* device(const std::string& id) const;

  // --- the canonical nodes ----------------------------------------------------------------------
  // `who` names the node in the log ("CloseGripper"). Empty `eef`: the only device; empty
  // `site`: the device's only site.
  EefResult grip(const std::string& who, const std::string& eef, const std::string& site, bool engage);
  EefResult actuate(const std::string& who, const std::string& eef, const std::string& actuator,
                    const std::string& target, std::optional<int64_t> timeout_ms);
  // `expect` "held" | "released"; `accept` a ';'- or ','-separated list that may only NARROW the
  // device's accepted_evidence; unset timeouts take the device's verification block.
  EefResult verifyGrasp(const std::string& who, const std::string& eef, const std::string& site,
                        const std::string& expect, std::optional<int64_t> timeout_ms,
                        std::optional<int64_t> max_age_ms, const std::string& accept);
  // A port the node could not read (e.g. a non-integer timeout_ms from the blackboard):
  // CONTRACT_MISMATCH about `path` (the node's tree path), logged and latched like any fault.
  EefResult portError(const std::string& who, const std::string& path, const std::string& detail);

  // --- the start-up checks (the runner exits with code 2 on any problem) -------------------------
  // Literal eef/site/actuator/target/expect/accept/timeout ports against the contract, and no
  // SetGripper in a canonical tree.
  std::vector<std::string> checkTree(const BT::Tree& tree) const;
  // More than one publisher on a device's state topic.
  std::vector<std::string> checkStateWriters() const;

  // --- the fault latch ----------------------------------------------------------------------------
  void latch(eef::Reason reason, const std::string& subject, const std::string& detail);
  bool faultLatched() const;
  std::vector<std::string> latchedFaults() const;

private:
  // resolves `eef` (empty: the only device) or fills `out` with UNKNOWN_EEF
  EndEffectorDevice* resolve(const std::string& eef, EefResult& out) const;
  EefResult finish(const std::string& who, const std::string& what, EefResult r);

  rclcpp::Node::SharedPtr node_;
  eef::Contract contract_;
  std::vector<std::unique_ptr<EndEffectorDevice>> devices_;
  mutable std::mutex latch_m_;
  std::vector<std::string> latched_;
};

// The ONE line the runner logs per run naming the contract (pinned by test_m2_6_eef_runner_child):
//   "end-effector contract: LEGACY (ADR-0022)"
//   "end-effector contract: canonical, devices [<id>, <id>] (ADR-0022)"
std::string contractLine(const eef::Contract& contract);

// One robot-scoped name a canonical device creates on the runner's node, RELATIVE: it resolves in
// the runner's namespace (ADR-0022 D7). An ACTION is checked as its five entities.
struct EefEndpoint
{
  enum class Kind : uint8_t
  {
    TOPIC,
    SERVICE,
    ACTION,
  };
  Kind kind{Kind::TOPIC};
  std::string name;
};

// Every name EndEffectorDevice's constructor creates for `contract`, in its order: per device (eef.ids
// order) its state subscription eef/<id>/state, its scene-signal publisher eef/<id>/engaged when the
// block says scene_signal: tmr, then per actuator (actuator_ids order) its SetBool client
// eef/<id>/actuators/<a>/set_engaged or its action client <action>. Empty for a LEGACY contract.
// trainit_run_bt's cross-namespace remap guard checks them (ADR-0023 Decision 5;
// MoveItMotionRuntime::crossNamespaceRemaps).
std::vector<EefEndpoint> eefEndpoints(const eef::Contract& contract);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__END_EFFECTOR_REGISTRY_HPP_
