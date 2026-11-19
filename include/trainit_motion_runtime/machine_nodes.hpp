// =============================================================================
// machine_nodes.hpp -- the two machine-handshake BT nodes of P0.b (P0B_DESIGN.md §3.4; the TSA <-> TMR contract of
// P0B_IMPLEMENTATION_PLAN.md, decision D1). TMR's first production StatefulActionNodes: they never block a tick
// (RUNNING until decided), never throw out of onStart / onRunning, never emit a wake-up signal, and a halt is
// immediate and never touches the machine (onHalted: the machine's own cycle continues).
//
//   <RequestMachine channel="turner" [fault="{trainit__fault}"]/>
//     A NEW request on the logical machine service: synchronises with its status (D8), then decides in ONE tick --
//     310 / 306 / 313 (nothing written) or the request written (SUCCESS, "MACHINE turner REQUESTED").
//   <WaitForMachine channel="turner" for="done|ready" timeout_ms="8000" [fault="{trainit__fault}"]/>
//     done: the DONE of this job's request on the service; ready: the machine READY or DONE (after this job's request
//     completed, when one is pending). RUNNING while waiting (WAITING FOR MACHINE / MACHINE turner BUSY), SUCCESS, or
//     FAILURE with a STATION code (305 / 306 / 310 / 313) or without one (D4: no request in this job; D10: the robot's
//     clock is not running / stopped).
// `channel` is a LOGICAL id the runner prepared before the first tick (a literal in the tree: never a topic, a PLC
// address or an I/O); `timeout_ms` an integer 1..600000, on the runner's clock (D10). Every FAILURE writes `fault`
// (when the tree remaps it: the program's fault gate) and latches the job's machine failure in the registry (the RTI
// result code, D3); every machine-state change is logged as `MACHINE: <text>` and reaches the job's activity (the RTI
// `detail` while RUNNING, D5). NOT a safety function (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MACHINE_NODES_HPP_
#define TRAINIT_MOTION_RUNTIME__MACHINE_NODES_HPP_

#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "trainit_motion_runtime/machine_channel.hpp"

namespace trainit
{

class RequestMachine : public BT::StatefulActionNode
{
public:
  RequestMachine(const std::string& name, const BT::NodeConfig& cfg) : BT::StatefulActionNode(name, cfg) {}
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus step();
  std::string service_;
  machine::SteadyTime started_{};
};

class WaitForMachine : public BT::StatefulActionNode
{
public:
  WaitForMachine(const std::string& name, const BT::NodeConfig& cfg) : BT::StatefulActionNode(name, cfg) {}
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus step();
  std::string service_;
  bool ready_{false};
  std::optional<machine::WaitTimer> timer_;
  std::string last_text_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MACHINE_NODES_HPP_
