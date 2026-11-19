// =============================================================================
// machine_channels.hpp -- the per-process MACHINE-CHANNEL REGISTRY of the P0.b handshake (P0B_DESIGN.md §3.4;
// P0B_IMPLEMENTATION_PLAN.md D2, D5, D8-D10): the ROS binding of the pure rules in machine_channel.hpp.
//
// Owned by the runner's stack (RunnerStack), reached by the machine nodes through BtContext::machines -- real ROS
// whatever `process_controller` says (D2: a request never goes through the ProcessController). Per machine service
// the loaded trees name (prepare(), before the first tick):
//   - a status subscription `<service>/status` and a request publisher `<service>/request` (relative: the robot's
//     slot, ADR-0023; RELIABLE, TRANSIENT_LOCAL -- the request writer depth 1, level-held; the status reader KEEP_LAST
//     20, every image applied in order), the subscription in the registry's OWN mutually-exclusive callback group --
//     created with the registry, before the executor collects the node's groups;
//   - a feed under a mutex (the latest image, its steady arrival, liveness: machine::Feed), and the publisher counts of
//     `<service>/status` and `<service>/request` on their RESOLVED names, every 100 ms (a wall timer in that group).
// PROCESS scope (never cleared by a job): the requester session (random, non-zero, new at every start of the process),
// the seq per (service, source session), the last request written per service (the 313 "not yet consumed" rule).
// JOB scope (the tick thread; cleared by beginJob): the outstanding request per service (+ completed), the job's
// first machine-step failure (its STATION code and text: the RTI's station latch, D3) and the activity sink (D5).
// A real write is also a PROCESS_EVENT of kind `request` (ADR-0030), through the runner's shared process observer,
// on the tick thread. NOT a safety function (ADR-0033): the machine is never commanded, only asked.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MACHINE_CHANNELS_HPP_
#define TRAINIT_MOTION_RUNTIME__MACHINE_CHANNELS_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trainit_interfaces/msg/channel_request.hpp>
#include <trainit_interfaces/msg/channel_status.hpp>

#include "trainit_motion_runtime/execution_events.hpp"
#include "trainit_motion_runtime/machine_channel.hpp"

namespace trainit
{

class MachineChannels
{
public:
  using StatusMsg = trainit_interfaces::msg::ChannelStatus;
  using RequestMsg = trainit_interfaces::msg::ChannelRequest;
  using ActivitySink = std::function<void(const std::string&)>;

  struct Options
  {
    std::chrono::milliseconds max_age{1000};       // channel_status_max_age_ms (§3.4: LIVE; D8: the request's bound)
    std::chrono::milliseconds count_period{100};   // the publisher counts' period
    uint32_t initial_seq{1};                       // a source session's first seq (PB-P6 injects 2^32-1)
    uint32_t requester_session{0};                 // 0: a random, non-zero one
  };

  static rclcpp::QoS qos();         // the REQUEST writer, the wire's profile (§3.2): RELIABLE, TRANSIENT_LOCAL, depth 1
  // The STATUS reader: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 20 (protocol review fix 3) -- two images arriving between
  // two deliveries are both applied; compatible with the depth-1 writers, the wire unchanged.
  static rclcpp::QoS statusQos();
  static machine::Image imageOf(const StatusMsg& msg);

  // Creates the registry's callback group: construct it BEFORE the node joins its executor (RunnerStack::configure).
  MachineChannels(const rclcpp::Node::SharedPtr& node, Options options);
  ~MachineChannels();
  MachineChannels(const MachineChannels&) = delete;
  MachineChannels& operator=(const MachineChannels&) = delete;

  // --- set-up (before the first tick) ---------------------------------------------------------------------------
  // The service's status subscription and request publisher (idempotent). `service` is a LOGICAL id.
  void prepare(const std::string& service);
  bool prepared(const std::string& service) const;
  std::vector<std::string> services() const;
  void setProcessObserver(std::shared_ptr<ObservedProcessController::Observer> observer);
  // After the runner's settle: per service, INFO "status seen" or WARN "no status for machine service 'x' -- is the
  // machine simulation (or the PLC) serving it?" (never an exit: the first request ends 310, PB-P8).
  void reportStart() const;

  uint32_t requesterSession() const { return requester_session_; }
  std::chrono::milliseconds maxAge() const { return options_.max_age; }
  rclcpp::CallbackGroup::SharedPtr group() const { return group_; }
  rclcpp::Node::SharedPtr node() const { return node_; }

  // --- the decisions' inputs ------------------------------------------------------------------------------------
  std::optional<machine::Feed> feed(const std::string& service) const;              // a COPY; nullopt: not prepared
  std::optional<machine::Written> lastWritten(const std::string& service) const;    // PROCESS scope

  // --- the write (tick thread) ----------------------------------------------------------------------------------
  // The next identity for that source session, published as the service's request image, recorded as this process's
  // last written request and as the JOB's outstanding one, then the PROCESS_EVENT `request`. nullopt: not prepared.
  // A publish that throws records nothing (the feed's undo point restored, machine::Feed::unwrote) and propagates; an
  // observer that throws is logged, never propagated (the request IS written: observation never fails the step).
  // `runner_ns`: the runner's clock at the write (the clock of the waits' timeouts, D10; nullopt: not started), kept with
  // the job's outstanding request for the DONE text.
  std::optional<machine::Identity> write(const std::string& service, uint32_t source_session, machine::SteadyTime now,
                                         std::optional<int64_t> runner_ns = std::nullopt);

  // --- the JOB scope (tick thread) ------------------------------------------------------------------------------
  void beginJob(ActivitySink sink);   // clears the outstanding requests and the failure latch; the job's activity sink
  void endJob();                      // the sink is dropped (the latch stays readable until the next beginJob)
  void activity(const std::string& text) const;
  std::optional<machine::Outstanding> outstanding(const std::string& service) const;
  void markCompleted(const std::string& service);
  // The job's FIRST machine-step failure (a STATION code replaces an earlier code-0 one, D3/D4).
  void latchFailure(uint16_t code, const std::string& text);
  uint16_t jobStationCode() const;
  std::string jobStationText() const;

private:
  struct Slot;     // a service's feed (shared with its callbacks: never `this`)
  struct Shared;   // the slots (shared with the counting timer)
  struct Endpoint
  {
    rclcpp::Subscription<StatusMsg>::SharedPtr sub;
    rclcpp::Publisher<RequestMsg>::SharedPtr pub;
  };

  rclcpp::Node::SharedPtr node_;
  Options options_;
  uint32_t requester_session_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<Shared> shared_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex m_;   // endpoints_, observer_
  std::map<std::string, Endpoint> endpoints_;
  std::shared_ptr<ObservedProcessController::Observer> observer_;

  mutable std::mutex process_m_;   // the PROCESS scope
  std::map<std::string, machine::SeqCounter> seq_;
  std::map<std::string, machine::Written> last_written_;

  mutable std::mutex job_m_;   // the JOB scope
  std::map<std::string, machine::Outstanding> outstanding_;
  uint16_t failure_code_{0};
  std::string failure_text_;
  bool failed_{false};
  ActivitySink sink_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MACHINE_CHANNELS_HPP_
