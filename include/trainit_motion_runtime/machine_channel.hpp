// =============================================================================
// machine_channel.hpp -- the robot's rules of the P0.b machine handshake (P0B_DESIGN.md §3.4, the FROZEN contract;
// implementation decisions D4, D8, D10 of P0B_IMPLEMENTATION_PLAN.md), PURE: no ROS, no threads, no clock of its
// own. Every input carries the STEADY time it is applied at (and, for a wait, the runner clock's sample), so the
// rules are deterministic and unit-tested with a fake clock (test_p0b_channel_rules). The ROS binding is
// machine_channels.hpp (the registry); the BT nodes are machine_nodes.hpp.
//
// The wire (trainit_interfaces 0.5.0): `<service>/status` (ChannelStatus: source_session, heartbeat, state, the
// cycle's and the last rejection's request identity, fault_code) and `<service>/request` (ChannelRequest:
// source_session, requester_session, seq). A request's IDENTITY is (requester_session, seq); matching is by identity
// EQUALITY, never by ordering.
//
//   Feed           -- what the robot knows of one service's status: the last image, when it arrived, the last sign of
//                     life (a heartbeat CHANGE of the current session: §3.4 literally), the heartbeat changes of the
//                     CURRENT session, the publisher counts of `<service>/status` and `<service>/request`.
//   decideRequest  -- RequestMachine: synchronise (D8), then ONE decision in the §3.4 order (310, 306, 313, write).
//   decideWaitDone / decideWaitReady -- WaitForMachine, each tick in the §3.4 order.
//   WaitTimer      -- a wait's timeout on the runner's clock (D10: not started, a backward jump, a frozen clock).
//   SeqCounter     -- the requester's seq for one source session: 1 .. 2^32-1, wrapping to 1, 0 never (PB-P6).
// Every text here is a USER-FACING text (owner decision 10, §3.5): WAITING FOR MACHINE, MACHINE <s> BUSY / DONE /
// FAULT / TIMEOUT / UNAVAILABLE / REQUESTED -- never a topic or a counter. NOT a safety function (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MACHINE_CHANNEL_HPP_
#define TRAINIT_MOTION_RUNTIME__MACHINE_CHANNEL_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace trainit::machine
{

using SteadyTime = std::chrono::steady_clock::time_point;

// ChannelStatus.state (trainit_interfaces 0.5.0; the ROS binding static_asserts the values).
enum class State : uint8_t { NONE = 0, READY = 1, BUSY = 2, DONE = 3, FAULT = 4, HOMING = 5 };
// ChannelStatus.reject_code
enum class Reject : uint8_t { NONE = 0, BUSY = 1, FAULTED = 2, TWO_REQUESTERS = 3 };
// ChannelStatus.fault_code
enum class FaultCode : uint16_t { NONE = 0, ACTUATOR_TIMEOUT = 1, SENSOR_FAULT = 2, STATION_FAULT = 3 };

// The STATION codes a machine step ends a job with (ADR-0032 Decision 7; 313 appended by P0.b). The same values as
// rti::code (the ROS binding static_asserts it); 0 = a failure that is not a machine condition (D4, D10).
namespace station
{
constexpr uint16_t NONE = 0;
constexpr uint16_t TIMEOUT = 305;       // STATION_TIMEOUT
constexpr uint16_t FAULT = 306;         // STATION_FAULT
constexpr uint16_t UNAVAILABLE = 310;   // STATION_UNAVAILABLE
constexpr uint16_t BUSY = 313;          // STATION_BUSY
}  // namespace station

// A service's two topics, RELATIVE to the runner's namespace (the robot's slot, ADR-0023): "<service>/status" and
// "<service>/request".
std::string statusTopic(const std::string& service);
std::string requestTopic(const std::string& service);

const char* stateWord(uint8_t state);        // "READY", "BUSY", "DONE", "FAULT", "HOMING"; "NO STATE" for 0; "?"
const char* faultWord(uint16_t fault_code);  // "ACTUATOR_TIMEOUT", "SENSOR_FAULT", "STATION_FAULT"; "" for 0; "?"
bool validState(uint8_t state);              // READY .. HOMING

// A request's identity: (requester_session, seq). (0, 0) = none.
struct Identity
{
  uint32_t requester{0};
  uint32_t seq{0};
  bool none() const { return requester == 0 && seq == 0; }
  bool operator==(const Identity& o) const { return requester == o.requester && seq == o.seq; }
  bool operator!=(const Identity& o) const { return !(*this == o); }
};

// The status image's contract fields (`detail` is diagnosis only: never read by a rule).
struct Image
{
  uint32_t source_session{0};
  uint16_t heartbeat{0};
  uint8_t state{0};
  Identity cycle;
  bool cycle_done{false};
  Identity reject;
  uint8_t reject_code{0};
  uint16_t fault_code{0};
};

// A request this process wrote: the source session it was FOR, its identity, and when (steady; the 313 "not yet taken"
// text says how long ago -- protocol review MINOR-2; nullopt: not known).
struct Written
{
  uint32_t source_session{0};
  Identity id;
  std::optional<SteadyTime> at{};
};

// The JOB's outstanding request on a service (cleared at every job start, §3.4): what was written, when (steady), when
// on the runner's clock (the clock of a wait's timeout, D10; nullopt: not started -- sim time before the first /clock),
// and whether a wait already saw its cycle complete.
struct Outstanding
{
  Written written;
  SteadyTime at{};
  std::optional<int64_t> runner_ns{};
  bool completed{false};
};

// What the robot knows of one service's status (§3.4, D8). Fed by the registry under its lock, COPIED for a decision.
//   - the first image received is the BASELINE: a late subscriber receives the latched image whatever the source's
//     state, so it is no sign of life by itself;
//   - after it, ONLY a heartbeat change of the current session is a sign of life (§3.4 literally: LIVE = the heartbeat
//     CHANGED within channel_status_max_age_ms; architecture review F2). A session change is neither a sign of life nor
//     a heartbeat change: it resets the CURRENT session's heartbeat-change count (the request's synchronisation needs
//     2 of the new session) and leaves the last sign of life where it was -- a healthy restart stays live through the
//     new session's first heartbeat change (one period) only while the old session's last change is within the max age;
//   - an image with source_session 0 says nothing (a default image): ignored.
// LIVE = a sign of life within the max age (steady) AND exactly one status publisher AND a valid state.
class Feed
{
public:
  explicit Feed(std::chrono::milliseconds max_age = std::chrono::milliseconds(1000));

  void subscribed(SteadyTime at);                // when the status subscription was created
  void image(const Image& img, SteadyTime at);   // an image arrived
  void publishers(int status, int request);      // the graph's publisher counts (every period)

  std::chrono::milliseconds maxAge() const { return max_age_; }
  bool hasImage() const { return has_image_; }   // an image with a source session was received
  const Image& last() const { return last_; }
  int statusPublishers() const { return status_pubs_; }
  int requestPublishers() const { return request_pubs_; }
  int heartbeatChanges() const { return changes_; }   // of the CURRENT session
  std::optional<SteadyTime> lastArrival() const;
  std::optional<SteadyTime> lastLife() const;
  std::optional<SteadyTime> subscribedAt() const { return subscribed_; }

  bool live(SteadyTime now) const;
  // D8: a request decides AT ONCE (never synchronises) on a source known dead: no image ever and the subscription
  // older than the max age, or the last image older than the max age.
  bool knownDead(SteadyTime now) const;
  // D8: synchronised = the current session's heartbeat changed at least twice AND its status publisher is in the
  // graph (a count not yet refreshed is not a "two sources").
  bool synchronised() const { return has_image_ && changes_ >= 2 && status_pubs_ >= 1; }
  // Why the service is not LIVE, in plain words (empty when it is).
  std::string whyNotLive(const std::string& service, SteadyTime now) const;
  // The operator's log when this image is applied (call BEFORE image()): the status of a source starts to arrive --
  // the first image, a NEW source session, or an image after more than the max age without one ("status arriving
  // ..."); empty otherwise. A session is told at most once per max age (two sources alternating their sessions --
  // a misconfiguration the robot answers 310 -- never flood the log); with more than one status publisher counted the
  // line says "ANOTHER source", never a restart. Diagnosis only (a launch tier waits for it after a source restart:
  // the ROBOT's own view); it records what it told, so it is not const.
  std::string arrival(const Image& img, SteadyTime at);

  // This process's last written request to the service (§3.4 request rule 3). CONSUMED is an event: once an image of
  // its source session names it (`cycle_*` or `reject_*`), it stays consumed -- a later image naming another identity
  // (a second requester's, PB-P11 / PB-P20) never makes it "not yet taken" again (the P0.b M-tier finding: without
  // the latch, one rejection of another requester locked the robot out with 313 until the source restarted).
  // `wrote` is called BEFORE the image is published, so the image that names it cannot be missed.
  void wrote(const Written& w);
  bool consumed(const Written& w) const;
  // The publish that followed wrote() THREW (C++ review M2): the request never reached the wire, and the registry still
  // names the PREVIOUS request as its last written one -- unwrote() restores the snapshot wrote() took of the previous
  // written state (else consumed(previous) stays false for ever: a 313 lock-out). The previous request stays watched
  // until the publish's outcome is known, so an image naming it that was applied in between still counts. Only right
  // after the wrote() whose publish failed; without a pending wrote() it changes nothing.
  void unwrote();
  // The publish that followed wrote() SUCCEEDED: the snapshot is dropped -- a later unwrote() changes nothing.
  void kept();

private:
  struct WroteSnapshot
  {
    std::optional<Written> wrote;
    bool consumed{false};
  };

  std::chrono::milliseconds max_age_;
  std::optional<SteadyTime> subscribed_;
  std::optional<Written> wrote_;
  bool wrote_consumed_{false};
  std::optional<WroteSnapshot> undo_;   // the written state before the last wrote() (unwrote() restores it)
  std::array<std::pair<uint32_t, SteadyTime>, 4> told_{};   // the sessions arrival() last told, and when
  bool has_image_{false};
  Image last_;
  SteadyTime last_arrival_{};
  bool has_life_{false};
  SteadyTime last_life_{};
  int changes_{0};
  int status_pubs_{0};
  int request_pubs_{0};
};

// A decision of one tick.
enum class Verdict : uint8_t { RUNNING, SUCCESS, FAILURE, WRITE };
// RUNNING: what the robot is waiting for (the activity's state word).
enum class Phase : uint8_t { NONE, SYNC, WAITING, BUSY };

struct Decision
{
  Verdict verdict{Verdict::RUNNING};
  Phase phase{Phase::NONE};
  uint16_t code{station::NONE};   // FAILURE: the STATION code, or 0 (not a machine condition: D4, D10)
  uint32_t source_session{0};     // WRITE: the source session the request is FOR
  bool completes{false};          // SUCCESS of a wait on the job's outstanding request: its cycle completed
  std::string text;               // the user-facing text (the activity, the fault, the job's detail)
};

// RequestMachine, each tick (`started`: the node's first tick, steady). D8: a source known dead -> decided at once;
// otherwise RUNNING (synchronising) while not synchronised and within the max age of `started`; then in ONE tick:
//   1. not LIVE (or not synchronised when the bound expired), or more than one publisher on `<service>/request`
//      -> 310, nothing written;
//   2. state FAULT -> 306, nothing written;
//   3. state BUSY or HOMING, or `last` (this process's last written request) is FOR this source session and neither
//      cycle_* nor reject_* names it (not yet CONSUMED) -> 313, nothing written (the text says how long ago `last` was
//      written);
//   4. WRITE (for the image's source session).
Decision decideRequest(const std::string& service, const Feed& feed, SteadyTime now, SteadyTime started,
                       const std::optional<Written>& last);

// A wait's clock (D10): the timer's view of this tick.
struct WaitClock
{
  bool timed_out{false};
  double elapsed_s{0.0};      // on the runner's clock
  double timeout_s{0.0};
  std::optional<int64_t> now_ns;   // the runner's clock (the node clock's ns) at this tick; nullopt: not running now
  std::string problem;        // non-empty: the robot's clock is not running / stopped (a FAILURE without a code)
};

// WaitForMachine for="done", each tick in this order (§3.4):
//   1. no outstanding request on the service in this job -> FAILURE without a code (D4: a program error);
//   2. the image's source session differs from the request's -> 310 (the machine restarted; with more than one status
//      publisher counted: two sources publish its status -- a second source with its own session);
//   3. not LIVE -> 310;
//   4. reject_* == the request -> 313 (BUSY) / 306 (FAULTED) / 310 (TWO_REQUESTERS);
//   5. cycle_* == the request AND cycle_done -> SUCCESS (before any fault: a completed cycle is a success);
//   6. state FAULT -> 306; cycle_* == the request, not done, state READY / HOMING / DONE -> 306 (its cycle aborted);
//   7. the robot's clock not running / stopped (D10) -> FAILURE without a code; `timeout_ms` elapsed on the runner's
//      clock -> 305 with the last state;
//   8. RUNNING: WAITING (not yet taken) or BUSY (taken, executing).
Decision decideWaitDone(const std::string& service, const Feed& feed, SteadyTime now,
                        const std::optional<Outstanding>& outstanding, const WaitClock& clock);

// WaitForMachine for="ready", each tick:
//   with an outstanding request of this job NOT yet completed (`outstanding`, completed == false):
//     a. the image's source session differs from the request's -> 310 (restarted, or two sources as for done);
//     b. not LIVE -> 310;
//     c. reject_* == it -> 313 / 306 / 310; d. cycle_* == it AND cycle_done AND state READY or DONE -> SUCCESS
//     (it completes the request); e. state FAULT -> 306; f. cycle_* == it, not done, state READY / HOMING / DONE ->
//     306 (aborted); g. the clock (D10) / the timeout -> 305; h. RUNNING (WAITING, BUSY, or HOMING after it);
//   otherwise (none, or already completed): a. not LIVE -> 310 (a source restart alone is no failure: a restarted
//     source that reports READY is ready); b. D19 (the owner's amendment of §3.4, 2026-11-19): this PROCESS's last
//     written request (`last`, MachineChannels::lastWritten) to the CURRENT source session not yet taken -- the same
//     condition as request rule 3's 313 (no image named it in cycle_* / reject_*, the latch) -- e.g. written by a job
//     cancelled at once: state FAULT -> 306, the clock / the timeout -> 305, else RUNNING (WAITING): the status of
//     before that request is taken is never "ready" (the machine may start that request's cycle at any time);
//     c. state READY or DONE -> SUCCESS; d. state FAULT -> 306; e. the clock / the timeout -> 305; f. RUNNING (BUSY
//     or HOMING: keep waiting).
Decision decideWaitReady(const std::string& service, const Feed& feed, SteadyTime now,
                         const std::optional<Outstanding>& outstanding, const std::optional<Written>& last,
                         const WaitClock& clock);

// A wait's timeout on the runner's clock (D10). sample() once per tick with the runner clock (nullopt: not started --
// sim time before the first /clock) and the steady time.
//   - not started: the timer waits for it; after `max_age` (steady) the problem "the robot's clock is not running";
//   - a backward jump restarts the timer from the new time;
//   - a clock that has not advanced for more than `max_age` (steady) -> the problem "the robot's clock stopped"
//     (the rules consult it only once the source passed every machine check: a dead source is 310 first).
class WaitTimer
{
public:
  WaitTimer(std::chrono::milliseconds timeout, std::chrono::milliseconds max_age);
  void sample(std::optional<int64_t> runner_ns, SteadyTime steady);
  WaitClock clock() const;
  bool restartedLastSample() const { return restarted_; }

private:
  std::chrono::milliseconds timeout_;
  std::chrono::milliseconds max_age_;
  bool first_{true};
  SteadyTime steady_start_{};
  bool started_{false};
  int64_t t0_{0};
  int64_t last_runner_{0};
  SteadyTime last_change_{};
  SteadyTime last_steady_{};
  std::optional<SteadyTime> not_started_since_;
  bool restarted_{false};
};

// The requester's seq for ONE source session (§3.2): `initial` first (PB-P6 injects 2^32-1), then +1, wrapping from
// 2^32-1 to 1; 0 is never used. A new source session starts again from `initial`.
class SeqCounter
{
public:
  explicit SeqCounter(uint32_t initial = 1);
  uint32_t next(uint32_t source_session);

private:
  uint32_t initial_;
  uint32_t session_{0};
  uint32_t last_{0};
};

// --- the user-facing texts (owner decision 10, §3.5) ---------------------------------------------------------------
std::string requestedText(const std::string& service);
std::string waitingText(const std::string& service, const std::string& why, double elapsed_s, double timeout_s);
std::string busyText(const std::string& service, double elapsed_s, double timeout_s);
// "MACHINE <service> DONE in <the request's age on the runner's clock>", with the wall time in brackets when the two
// read differently (a simulation slower or faster than real time; acceptance finding 1, 2026-11-19); the wall time
// alone, marked so, when the runner's clock has no age for the request (not started at the request, or jumped back).
std::string doneText(const std::string& service, std::optional<double> runner_s, double wall_s);
std::string readyText(const std::string& service, double waited_s);
std::string failureText(uint16_t code, const std::string& service, const std::string& why);
std::string programErrorText(const std::string& service, const std::string& why);
// The registry's start diagnostic of one service (after the runner's settle): empty `warning` = a status is seen.
struct StartDiagnostic
{
  bool warning{false};
  std::string text;
};
StartDiagnostic startDiagnostic(const std::string& service, const Feed& feed, SteadyTime now);

}  // namespace trainit::machine

#endif  // TRAINIT_MOTION_RUNTIME__MACHINE_CHANNEL_HPP_
