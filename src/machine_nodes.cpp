// machine_nodes.cpp -- RequestMachine and WaitForMachine, the P0.b machine-handshake nodes. See the header.
#include "trainit_motion_runtime/machine_nodes.hpp"

#include <chrono>
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/machine_channels.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace trainit
{

namespace
{
constexpr int kTimeoutMaxMs = 600000;   // TSA's bound (R-P0B-02): 1..600000 ms

// The runner's context, or null -- never an exception out of a tick.
BtContext* contextOf(const BT::NodeConfig& cfg) noexcept
{
  try
  {
    return cfg.blackboard->get<BtContext*>(BB_BT_CONTEXT);
  }
  catch (...)
  {
    return nullptr;
  }
}

rclcpp::Logger loggerOf(const BtContext* ctx)
{
  return ctx && ctx->node ? ctx->node->get_logger() : rclcpp::get_logger("trainit_bt");
}

std::string stringPort(const BT::TreeNode& node, const char* port)
{
  try
  {
    const auto v = node.getInput<std::string>(port);
    return v ? v.value() : std::string();
  }
  catch (...)
  {
    return std::string();
  }
}

// A machine step FAILS: the text logged (`MACHINE: `), the job's failure latched (its STATION code, or 0), the job's
// activity, and `fault` written when the tree remaps it (an unremapped port writes nothing).
BT::NodeStatus failWith(BT::TreeNode& node, MachineChannels* machines, const rclcpp::Logger& log, uint16_t code,
                        const std::string& text)
{
  RCLCPP_ERROR(log, "MACHINE: %s", text.c_str());
  if (machines)
  {
    machines->latchFailure(code, text);
    machines->activity(text);
  }
  try
  {
    (void)node.setOutput("fault", text);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "MACHINE: its fault output was not written (%s)", e.what());
  }
  return BT::NodeStatus::FAILURE;
}

// The service a machine node names must be a LOGICAL id the runner prepared before the first tick (empty: usable).
std::string serviceProblem(const MachineChannels* machines, const std::string& service)
{
  if (!machines) return "this runner has no machine-channel registry";
  if (!isLogicalChannel(service))
    return "the channel must be a LOGICAL machine service id (a letter, then letters, digits or _), never a topic, a "
           "PLC address or an I/O";
  if (!machines->prepared(service))
    return "the runner did not prepare it before the first tick (a machine service is named literally in the tree)";
  return "";
}

std::optional<int64_t> runnerClockNs(const BtContext* ctx)
{
  if (!ctx || !ctx->node) return std::nullopt;
  const auto clock = ctx->node->get_clock();
  if (!clock->started()) return std::nullopt;   // sim time before the first /clock (D10)
  return clock->now().nanoseconds();
}
}  // namespace

// =================================================================================================================
// RequestMachine
// =================================================================================================================
BT::PortsList RequestMachine::providedPorts()
{
  return {BT::InputPort<std::string>("channel", "a logical machine service id (never a topic, a PLC address or an I/O)"),
          BT::OutputPort<std::string>("fault", "on FAILURE: MACHINE <service> FAULT / UNAVAILABLE (its STATION code)")};
}

BT::NodeStatus RequestMachine::onStart()
{
  started_ = std::chrono::steady_clock::now();
  service_ = stringPort(*this, "channel");
  return step();
}

BT::NodeStatus RequestMachine::onRunning()
{
  return step();
}

void RequestMachine::onHalted()
{
  // halted while synchronising: nothing was written. A request already written is never taken back (the machine is
  // asked, never commanded: it consumes the request by its own rules).
}

BT::NodeStatus RequestMachine::step()
{
  BtContext* ctx = contextOf(config());
  const auto log = loggerOf(ctx);
  MachineChannels* machines = ctx ? ctx->machines : nullptr;
  try
  {
    if (const std::string why = serviceProblem(machines, service_); !why.empty())
      return failWith(*this, machines, log, machine::station::NONE, machine::programErrorText(service_, why));
    const auto now = std::chrono::steady_clock::now();
    const auto feed = machines->feed(service_);
    if (!feed)
      return failWith(*this, machines, log, machine::station::NONE,
                      machine::programErrorText(service_, "the runner did not prepare it before the first tick"));
    const machine::Decision d =
      machine::decideRequest(service_, *feed, now, started_, machines->lastWritten(service_));
    switch (d.verdict)
    {
      case machine::Verdict::RUNNING:
        return BT::NodeStatus::RUNNING;   // synchronising (D8): bounded by the max age
      case machine::Verdict::FAILURE:
        return failWith(*this, machines, log, d.code, d.text);
      case machine::Verdict::WRITE:
      {
        if (!machines->write(service_, d.source_session, now, runnerClockNs(ctx)))
          return failWith(*this, machines, log, machine::station::NONE,
                          machine::programErrorText(service_, "the runner did not prepare it before the first tick"));
        RCLCPP_INFO(log, "MACHINE: %s", d.text.c_str());
        machines->activity(d.text);
        return BT::NodeStatus::SUCCESS;
      }
      case machine::Verdict::SUCCESS:
        break;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "RequestMachine '%s': %s", service_.c_str(), e.what());
    return failWith(*this, machines, log, machine::station::NONE,
                    machine::programErrorText(service_, "the request could not be written (see the runner's log)"));
  }
  return failWith(*this, machines, log, machine::station::NONE,
                  machine::programErrorText(service_, "no decision (a rule is missing)"));
}

// =================================================================================================================
// WaitForMachine
// =================================================================================================================
BT::PortsList WaitForMachine::providedPorts()
{
  return {BT::InputPort<std::string>("channel", "a logical machine service id (never a topic, a PLC address or an I/O)"),
          BT::InputPort<std::string>("for", "done | ready"),
          BT::InputPort<int>("timeout_ms", "the wait's bound on the runner's clock: an integer 1..600000 (no default)"),
          BT::OutputPort<std::string>("fault",
                                      "on FAILURE: MACHINE <service> FAULT / TIMEOUT / UNAVAILABLE (its STATION code)")};
}

BT::NodeStatus WaitForMachine::onStart()
{
  BtContext* ctx = contextOf(config());
  const auto log = loggerOf(ctx);
  MachineChannels* machines = ctx ? ctx->machines : nullptr;
  service_ = stringPort(*this, "channel");
  timer_.reset();
  last_text_.clear();
  try
  {
    const std::string what = stringPort(*this, "for");
    if (what != "done" && what != "ready")
      return failWith(*this, machines, log, machine::station::NONE,
                      machine::programErrorText(service_, "a wait is for done or ready, not '" + what + "'"));
    ready_ = what == "ready";
    int timeout_ms = 0;
    const auto t = getInput<int>("timeout_ms");
    if (!t || t.value() < 1 || t.value() > kTimeoutMaxMs)
      return failWith(*this, machines, log, machine::station::NONE,
                      machine::programErrorText(service_, "a wait needs its timeout_ms, an integer 1..600000 (no "
                                                          "default)"));
    timeout_ms = t.value();
    if (const std::string why = serviceProblem(machines, service_); !why.empty())
      return failWith(*this, machines, log, machine::station::NONE, machine::programErrorText(service_, why));
    timer_.emplace(std::chrono::milliseconds(timeout_ms), machines->maxAge());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "WaitForMachine '%s': %s", service_.c_str(), e.what());
    return failWith(*this, machines, log, machine::station::NONE,
                    machine::programErrorText(service_, "the wait could not start (see the runner's log)"));
  }
  return step();
}

BT::NodeStatus WaitForMachine::onRunning()
{
  return step();
}

void WaitForMachine::onHalted()
{
  // a cancel / stop halts the wait at once; the machine's own cycle is never touched (§3.4)
  RCLCPP_INFO(loggerOf(contextOf(config())), "MACHINE: the wait for machine %s was stopped (the machine's own cycle "
              "continues)", service_.c_str());
}

BT::NodeStatus WaitForMachine::step()
{
  BtContext* ctx = contextOf(config());
  const auto log = loggerOf(ctx);
  MachineChannels* machines = ctx ? ctx->machines : nullptr;
  try
  {
    if (!machines || !timer_)
      return failWith(*this, machines, log, machine::station::NONE,
                      machine::programErrorText(service_, "this runner has no machine-channel registry"));
    const auto now = std::chrono::steady_clock::now();
    timer_->sample(runnerClockNs(ctx), now);
    if (timer_->restartedLastSample())
      RCLCPP_WARN(log, "wait for machine %s: the robot's clock jumped back; its timeout restarts from the new time",
                  service_.c_str());
    const auto feed = machines->feed(service_);
    if (!feed)
      return failWith(*this, machines, log, machine::station::NONE,
                      machine::programErrorText(service_, "the runner did not prepare it before the first tick"));
    const auto outstanding = machines->outstanding(service_);
    // the ready-wait also consults this PROCESS's last written request (D19: not yet taken -> keep waiting)
    const machine::Decision d =
      ready_ ? machine::decideWaitReady(service_, *feed, now, outstanding, machines->lastWritten(service_),
                                        timer_->clock())
             : machine::decideWaitDone(service_, *feed, now, outstanding, timer_->clock());
    switch (d.verdict)
    {
      case machine::Verdict::RUNNING:
        if (d.text != last_text_)   // a machine-state change, or the elapsed whole seconds (at most once a second)
        {
          last_text_ = d.text;
          RCLCPP_INFO(log, "MACHINE: %s", d.text.c_str());
          machines->activity(d.text);
        }
        return BT::NodeStatus::RUNNING;
      case machine::Verdict::SUCCESS:
        if (d.completes) machines->markCompleted(service_);
        RCLCPP_INFO(log, "MACHINE: %s", d.text.c_str());
        machines->activity(d.text);
        return BT::NodeStatus::SUCCESS;
      case machine::Verdict::FAILURE:
        return failWith(*this, machines, log, d.code, d.text);
      case machine::Verdict::WRITE:
        break;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "WaitForMachine '%s': %s", service_.c_str(), e.what());
    return failWith(*this, machines, log, machine::station::NONE,
                    machine::programErrorText(service_, "the wait could not continue (see the runner's log)"));
  }
  return failWith(*this, machines, log, machine::station::NONE,
                  machine::programErrorText(service_, "no decision (a rule is missing)"));
}

}  // namespace trainit
