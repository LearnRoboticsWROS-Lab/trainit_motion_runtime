// machine_channel.cpp -- the robot's rules of the P0.b machine handshake (P0B_DESIGN.md §3.4). Pure: see the header.
#include "trainit_motion_runtime/machine_channel.hpp"

#include <cmath>
#include <cstdio>

#include "trainit_motion_runtime/rti_contract.hpp"

namespace trainit::machine
{

// One vocabulary: the STATION codes ARE the RTI's (ADR-0032 Decision 7; 313 appended by P0.b).
static_assert(station::TIMEOUT == rti::code::STATION_TIMEOUT);
static_assert(station::FAULT == rti::code::STATION_FAULT);
static_assert(station::UNAVAILABLE == rti::code::STATION_UNAVAILABLE);
static_assert(station::BUSY == rti::code::STATION_BUSY);

namespace
{
std::string secondsText(double s)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f s", s < 0.0 ? 0.0 : s);
  return buf;
}

std::string wholeSeconds(double s)
{
  return std::to_string(static_cast<long long>(std::floor(s < 0.0 ? 0.0 : s)));
}

std::string maxAgeText(std::chrono::milliseconds max_age)
{
  return secondsText(static_cast<double>(max_age.count()) / 1000.0);
}

double secondsBetween(SteadyTime from, SteadyTime to)
{
  return std::chrono::duration<double>(to - from).count();
}

Decision failure(uint16_t code, const std::string& service, const std::string& why)
{
  Decision d;
  d.verdict = Verdict::FAILURE;
  d.code = code;
  d.text = failureText(code, service, why);
  return d;
}

Decision problem(const std::string& service, const std::string& why)
{
  Decision d;
  d.verdict = Verdict::FAILURE;
  d.code = station::NONE;
  d.text = programErrorText(service, why);
  return d;
}

Decision running(Phase phase, std::string text)
{
  Decision d;
  d.verdict = Verdict::RUNNING;
  d.phase = phase;
  d.text = std::move(text);
  return d;
}

// The request's rejection (reject_* == it): 313 / 306 / 310 by its reason.
Decision rejected(const std::string& service, uint8_t reject_code)
{
  switch (static_cast<Reject>(reject_code))
  {
    case Reject::BUSY:
      return failure(station::BUSY, service, "the machine was still busy and did not take the request");
    case Reject::FAULTED:
      return failure(station::FAULT, service, "the machine was in FAULT and did not take the request");
    case Reject::TWO_REQUESTERS:
      return failure(station::UNAVAILABLE, service,
                     "the machine saw two requesters and did not take the request (one robot per machine service)");
    case Reject::NONE:
      break;
  }
  return failure(station::FAULT, service, "the machine did not take the request (an unknown reason)");
}

std::string faultWhy(const Image& img)
{
  const std::string word = faultWord(img.fault_code);
  return "the machine reports FAULT" + (word.empty() ? std::string() : " (" + word + ")");
}

// Protocol review N3: a RESET from BUSY with no fault aborts a cycle too -- the text says "aborted", never "faulted".
constexpr const char* kAborted = "its cycle was aborted (the machine was reset before completing it)";
constexpr const char* kRestarted = "the machine restarted (a new session) before completing the request";
// Protocol review MINOR-3: whenever two publishers are counted, the hint that a crashed process's endpoint lingers in
// the ROS graph (the DDS lease: about 20 s) before it leaves by itself.
constexpr const char* kTwoSources =
  "two sources publish its status (a crashed source's endpoint leaves the ROS graph within about 20 s)";
constexpr const char* kTwoRequesters =
  "another requester also writes its requests (one robot per machine service; a crashed requester's endpoint leaves "
  "the ROS graph within about 20 s)";

// A wait's rule 2 (the image's source session is not the request's): with more than one status publisher counted it is
// a SECOND source with its own session, not a restart (protocol review MINOR-1) -- the same 310, the order unchanged.
std::string sessionChangedWhy(const Feed& feed)
{
  return feed.statusPublishers() > 1 ? kTwoSources : kRestarted;
}

// The 313 of a request this process wrote that is not yet consumed: how long ago it was written (protocol review
// MINOR-2), when known.
std::string writtenAgo(const Written& w, SteadyTime now)
{
  if (!w.at) return "";
  return ", written " + secondsText(secondsBetween(*w.at, now)) + " ago";
}

// The image names this written request: its source session, and its identity in cycle_* or reject_* (a consumption).
bool names(const Image& img, const std::optional<Written>& w)
{
  return w && img.source_session == w->source_session && (img.cycle == w->id || img.reject == w->id);
}

// This process's last written request to the image's (the CURRENT) source session is not yet taken: the image does not
// name it and no image did since it was written (the latch, Feed::consumed). One condition, two rules: request rule 3
// refuses a new request on it (313); the ready-wait keeps waiting on it (D19, the owner's amendment of §3.4).
bool notYetTaken(const Feed& feed, const Image& img, const std::optional<Written>& last)
{
  return last && last->source_session == img.source_session && img.cycle != last->id && img.reject != last->id &&
         !feed.consumed(*last);
}
}  // namespace

// --- vocabulary -----------------------------------------------------------------------------------------------------
std::string statusTopic(const std::string& service)
{
  return service + '/' + "status";
}

std::string requestTopic(const std::string& service)
{
  return service + '/' + "request";
}

const char* stateWord(uint8_t state)
{
  switch (static_cast<State>(state))
  {
    case State::NONE: return "NO STATE";
    case State::READY: return "READY";
    case State::BUSY: return "BUSY";
    case State::DONE: return "DONE";
    case State::FAULT: return "FAULT";
    case State::HOMING: return "HOMING";
  }
  return "?";
}

const char* faultWord(uint16_t fault_code)
{
  switch (static_cast<FaultCode>(fault_code))
  {
    case FaultCode::NONE: return "";
    case FaultCode::ACTUATOR_TIMEOUT: return "ACTUATOR_TIMEOUT";
    case FaultCode::SENSOR_FAULT: return "SENSOR_FAULT";
    case FaultCode::STATION_FAULT: return "STATION_FAULT";
  }
  return "?";
}

bool validState(uint8_t state)
{
  return state >= static_cast<uint8_t>(State::READY) && state <= static_cast<uint8_t>(State::HOMING);
}

// --- Feed -----------------------------------------------------------------------------------------------------------
Feed::Feed(std::chrono::milliseconds max_age) : max_age_(max_age)
{
}

void Feed::subscribed(SteadyTime at)
{
  subscribed_ = at;
}

void Feed::image(const Image& img, SteadyTime at)
{
  if (img.source_session == 0) return;   // a default image: no source says anything
  // consumed: an event, latched (see wrote()) -- for the last written request, and for the previous one an unwrote()
  // would restore (an image applied between wrote() and a failed publish never loses its consumption)
  if (names(img, wrote_)) wrote_consumed_ = true;
  if (undo_ && names(img, undo_->wrote)) undo_->consumed = true;
  if (!has_image_)
  {
    // the BASELINE: a latched image reaches a late subscriber whatever the source's state -- no sign of life by itself
    has_image_ = true;
    last_ = img;
    last_arrival_ = at;
    changes_ = 0;
    return;
  }
  if (img.source_session != last_.source_session)
  {
    // a NEW session: neither a sign of life nor a heartbeat change (§3.4: LIVE = the heartbeat CHANGED within the max
    // age; architecture review F2) -- the count restarts (the request synchronises on 2 of the new session) and the
    // last sign of life stays the old session's last heartbeat change until the new session's heartbeat changes
    changes_ = 0;
  }
  else if (img.heartbeat != last_.heartbeat)
  {
    if (changes_ < 1000000) ++changes_;
    has_life_ = true;
    last_life_ = at;
  }
  last_ = img;
  last_arrival_ = at;
}

void Feed::wrote(const Written& w)
{
  undo_ = WroteSnapshot{wrote_, wrote_consumed_};   // the snapshot unwrote() restores when the publish fails
  wrote_ = w;
  wrote_consumed_ = false;
}

void Feed::unwrote()
{
  if (!undo_) return;
  wrote_ = undo_->wrote;
  wrote_consumed_ = undo_->consumed;
  undo_.reset();
}

void Feed::kept()
{
  undo_.reset();
}

bool Feed::consumed(const Written& w) const
{
  return wrote_ && wrote_->source_session == w.source_session && wrote_->id == w.id && wrote_consumed_;
}

void Feed::publishers(int status, int request)
{
  status_pubs_ = status;
  request_pubs_ = request;
}

std::optional<SteadyTime> Feed::lastArrival() const
{
  if (!has_image_) return std::nullopt;
  return last_arrival_;
}

std::optional<SteadyTime> Feed::lastLife() const
{
  if (!has_life_) return std::nullopt;
  return last_life_;
}

bool Feed::live(SteadyTime now) const
{
  return has_image_ && has_life_ && (now - last_life_) <= max_age_ && status_pubs_ == 1 && validState(last_.state);
}

bool Feed::knownDead(SteadyTime now) const
{
  if (!has_image_) return subscribed_.has_value() && (now - *subscribed_) > max_age_;
  return (now - last_arrival_) > max_age_;
}

std::string Feed::arrival(const Image& img, SteadyTime at)
{
  if (img.source_session == 0) return "";
  // a session told within the max age is not told again (two sources alternating their sessions); the oldest of the
  // remembered tellings is replaced
  const auto tell = [&](std::string text) {
    auto* slot = &told_[0];
    for (auto& t : told_)
    {
      if (t.first == img.source_session)
      {
        slot = &t;
        break;
      }
      if (t.second < slot->second) slot = &t;
    }
    *slot = {img.source_session, at};
    return text;
  };
  const auto toldWithinMaxAge = [&]() {
    for (const auto& t : told_)
      if (t.first == img.source_session && (at - t.second) <= max_age_) return true;
    return false;
  };
  char session[16];
  std::snprintf(session, sizeof(session), "%08x", img.source_session);
  if (!has_image_) return tell(std::string("status arriving (source session ") + session + ")");
  if (img.source_session != last_.source_session)
  {
    if (toldWithinMaxAge()) return "";
    char was[16];
    std::snprintf(was, sizeof(was), "%08x", last_.source_session);
    if (status_pubs_ > 1)
      return tell(std::string("status arriving from ANOTHER source session ") + session +
                  " (two sources publish its status; the previous image was of " + was + ")");
    return tell(std::string("status arriving from a NEW source session ") + session + " (was " + was + ")");
  }
  if ((at - last_arrival_) > max_age_)
    return tell("status arriving again after " + secondsText(secondsBetween(last_arrival_, at)) +
                " without one (source session " + session + ")");
  return "";
}

std::string Feed::whyNotLive(const std::string& service, SteadyTime now) const
{
  if (!has_image_)
    return "no status for service '" + service + "' (is the machine simulation, or the PLC, serving it?)";
  if (status_pubs_ > 1) return kTwoSources;
  if ((now - last_arrival_) > max_age_)
    return "no status for more than " + maxAgeText(max_age_) + " (the machine stopped publishing)";
  if (status_pubs_ < 1) return "no status source (the machine simulation, or the PLC, is gone)";
  // images may still be arriving (a frozen source republishes its image), or the last one came after the last sign of
  // life (an on-change image of the same heartbeat, just before the source stopped): either way no LIVE status
  if (!has_life_ || (now - last_life_) > max_age_)
    return "no status update for more than " + maxAgeText(max_age_) +
           " (the machine stopped publishing, or its status is frozen)";
  if (!validState(last_.state)) return "it reports no valid state";
  return "";
}

// --- RequestMachine -------------------------------------------------------------------------------------------------
Decision decideRequest(const std::string& service, const Feed& feed, SteadyTime now, SteadyTime started,
                       const std::optional<Written>& last)
{
  // D8: synchronise (RUNNING) on a source that may be alive, never on one known dead
  if (!feed.knownDead(now) && !feed.synchronised() && (now - started) <= feed.maxAge())
    return running(Phase::SYNC, "");

  const std::string nothing = "; nothing was requested";
  // 1. not LIVE (or not synchronised when the bound expired), or two requesters -> 310
  std::string why;
  if (!feed.live(now)) why = feed.whyNotLive(service, now);
  else if (!feed.synchronised())
    why = "its status did not show two signs of life within " + maxAgeText(feed.maxAge());
  else if (feed.requestPublishers() > 1)
    why = kTwoRequesters;
  if (!why.empty()) return failure(station::UNAVAILABLE, service, why + nothing);

  const Image& img = feed.last();
  // 2. FAULT -> 306
  if (img.state == static_cast<uint8_t>(State::FAULT)) return failure(station::FAULT, service, faultWhy(img) + nothing);
  // 3. BUSY / HOMING, or this process's previous request to this source session not yet consumed -> 313
  if (img.state == static_cast<uint8_t>(State::BUSY))
    return failure(station::BUSY, service, "the machine was still busy (BUSY)" + nothing);
  if (img.state == static_cast<uint8_t>(State::HOMING))
    return failure(station::BUSY, service, "the machine was still busy (HOMING after a reset)" + nothing);
  if (notYetTaken(feed, img, last))
    return failure(station::BUSY, service,
                   "the machine was still busy (it has not yet taken the previous request" + writtenAgo(*last, now) +
                     ")" + nothing);
  // 4. write
  Decision d;
  d.verdict = Verdict::WRITE;
  d.source_session = img.source_session;
  d.text = requestedText(service);
  return d;
}

// --- WaitForMachine for="done" --------------------------------------------------------------------------------------
Decision decideWaitDone(const std::string& service, const Feed& feed, SteadyTime now,
                        const std::optional<Outstanding>& outstanding, const WaitClock& clock)
{
  // 1. no request of this job on the service (D4)
  if (!outstanding)
    return problem(service, "a wait for done with no request on it in this job (a wait for done follows a request "
                            "on the same machine service)");
  const Written& w = outstanding->written;
  const Image& img = feed.last();
  // 2. a source restart -> 310 at once (a second source with its own session: "two sources", MINOR-1)
  if (feed.hasImage() && img.source_session != w.source_session)
    return failure(station::UNAVAILABLE, service, sessionChangedWhy(feed));
  // 3. not LIVE -> 310
  if (!feed.live(now)) return failure(station::UNAVAILABLE, service, feed.whyNotLive(service, now));
  // 4. the request was rejected
  if (img.reject == w.id) return rejected(service, img.reject_code);
  // 5. its cycle completed: SUCCESS, before any fault
  if (img.cycle == w.id && img.cycle_done)
  {
    Decision d;
    d.verdict = Verdict::SUCCESS;
    d.completes = true;
    // the request's age on the runner's clock (the BUSY counter's and the timeout's, D10); a clock behind the request
    // (a simulation reset) gives the wall age instead. Accepted (C++ review, MINOR): a reset that then runs PAST the
    // request's time before the DONE prints a short runner age -- a log text only, never a verdict.
    std::optional<double> runner_s;
    if (outstanding->runner_ns && clock.now_ns && *clock.now_ns >= *outstanding->runner_ns)
      runner_s = static_cast<double>(*clock.now_ns - *outstanding->runner_ns) / 1e9;
    d.text = doneText(service, runner_s, secondsBetween(outstanding->at, now));
    return d;
  }
  // 6. a fault, or its cycle aborted -> 306
  if (img.state == static_cast<uint8_t>(State::FAULT)) return failure(station::FAULT, service, faultWhy(img));
  const bool taken = img.cycle == w.id;
  if (taken && (img.state == static_cast<uint8_t>(State::READY) || img.state == static_cast<uint8_t>(State::HOMING) ||
                img.state == static_cast<uint8_t>(State::DONE)))
    return failure(station::FAULT, service, kAborted);
  // 7. the robot's clock (D10), then the timeout -> 305 with the last state
  if (!clock.problem.empty()) return problem(service, clock.problem + ": the wait cannot time out");
  if (clock.timed_out)
    return failure(station::TIMEOUT, service,
                   taken ? std::string("still ") + stateWord(img.state) + " after " + secondsText(clock.timeout_s)
                         : "the request not yet taken after " + secondsText(clock.timeout_s) + " (the machine is " +
                             stateWord(img.state) + ")");
  // 8. RUNNING
  if (taken) return running(Phase::BUSY, busyText(service, clock.elapsed_s, clock.timeout_s));
  return running(Phase::WAITING, waitingText(service, "", clock.elapsed_s, clock.timeout_s));
}

// --- WaitForMachine for="ready" -------------------------------------------------------------------------------------
Decision decideWaitReady(const std::string& service, const Feed& feed, SteadyTime now,
                         const std::optional<Outstanding>& outstanding, const std::optional<Written>& last,
                         const WaitClock& clock)
{
  const Image& img = feed.last();
  const bool ready_state =
    img.state == static_cast<uint8_t>(State::READY) || img.state == static_cast<uint8_t>(State::DONE);
  if (outstanding && !outstanding->completed)
  {
    // the job's request is not completed yet: ready REQUIRES its completion (B1) -- never the status of before it
    const Written& w = outstanding->written;
    if (feed.hasImage() && img.source_session != w.source_session)
      return failure(station::UNAVAILABLE, service, sessionChangedWhy(feed));
    if (!feed.live(now)) return failure(station::UNAVAILABLE, service, feed.whyNotLive(service, now));
    if (img.reject == w.id) return rejected(service, img.reject_code);
    if (img.cycle == w.id && img.cycle_done && ready_state)
    {
      Decision d;
      d.verdict = Verdict::SUCCESS;
      d.completes = true;
      d.text = readyText(service, clock.elapsed_s);
      return d;
    }
    if (img.state == static_cast<uint8_t>(State::FAULT)) return failure(station::FAULT, service, faultWhy(img));
    const bool taken = img.cycle == w.id;
    if (taken && !img.cycle_done &&
        (img.state == static_cast<uint8_t>(State::READY) || img.state == static_cast<uint8_t>(State::HOMING) ||
         img.state == static_cast<uint8_t>(State::DONE)))
      return failure(station::FAULT, service, kAborted);
    if (!clock.problem.empty()) return problem(service, clock.problem + ": the wait cannot time out");
    if (clock.timed_out)
      return failure(station::TIMEOUT, service,
                     taken ? std::string("still ") + stateWord(img.state) + " after " + secondsText(clock.timeout_s)
                           : "the request not yet taken after " + secondsText(clock.timeout_s) + " (the machine is " +
                               stateWord(img.state) + ")");
    if (taken && !img.cycle_done) return running(Phase::BUSY, busyText(service, clock.elapsed_s, clock.timeout_s));
    return running(Phase::WAITING,
                   waitingText(service, taken ? std::string("it is ") + stateWord(img.state) : "", clock.elapsed_s,
                               clock.timeout_s));
  }
  // no request of this job pending: ready = LIVE and READY or DONE (a restarted source that reports READY is ready)
  if (!feed.live(now)) return failure(station::UNAVAILABLE, service, feed.whyNotLive(service, now));
  // D19 (the owner's amendment of §3.4, 2026-11-19; protocol review MAJOR-2): this process's last request to the
  // current source session is not yet taken -- e.g. a job cancelled right after its request: the status of before that
  // request is taken is never "ready" for THIS job (the machine may start that request's cycle at any time). Wait for
  // it to be taken; then the rules below (its cycle's BUSY keeps waiting, DONE / READY is ready).
  if (notYetTaken(feed, img, last))
  {
    if (img.state == static_cast<uint8_t>(State::FAULT)) return failure(station::FAULT, service, faultWhy(img));
    if (!clock.problem.empty()) return problem(service, clock.problem + ": the wait cannot time out");
    if (clock.timed_out)
      return failure(station::TIMEOUT, service,
                     "still not ready after " + secondsText(clock.timeout_s) +
                       " (the machine has not yet taken the previous request; it is " + stateWord(img.state) + ")");
    return running(Phase::WAITING, waitingText(service, "it has not yet taken the previous request", clock.elapsed_s,
                                               clock.timeout_s));
  }
  if (ready_state)
  {
    Decision d;
    d.verdict = Verdict::SUCCESS;
    d.text = readyText(service, clock.elapsed_s);
    return d;
  }
  if (img.state == static_cast<uint8_t>(State::FAULT)) return failure(station::FAULT, service, faultWhy(img));
  if (!clock.problem.empty()) return problem(service, clock.problem + ": the wait cannot time out");
  if (clock.timed_out)
    return failure(station::TIMEOUT, service,
                   "still not ready after " + secondsText(clock.timeout_s) + " (the machine is " +
                     stateWord(img.state) + ")");
  return running(Phase::WAITING, waitingText(service, std::string("it is ") + stateWord(img.state), clock.elapsed_s,
                                             clock.timeout_s));
}

// --- WaitTimer (D10) ------------------------------------------------------------------------------------------------
WaitTimer::WaitTimer(std::chrono::milliseconds timeout, std::chrono::milliseconds max_age)
: timeout_(timeout), max_age_(max_age)
{
}

void WaitTimer::sample(std::optional<int64_t> runner_ns, SteadyTime steady)
{
  restarted_ = false;
  if (first_)
  {
    first_ = false;
    steady_start_ = steady;
  }
  last_steady_ = steady;
  if (!runner_ns)
  {
    if (!not_started_since_) not_started_since_ = steady;
    return;
  }
  not_started_since_.reset();
  if (!started_)
  {
    started_ = true;
    t0_ = *runner_ns;
    last_runner_ = *runner_ns;
    last_change_ = steady;
    return;
  }
  if (*runner_ns < last_runner_)
  {
    t0_ = *runner_ns;   // a backward jump (a simulation reset): the timer restarts from the new time
    restarted_ = true;
  }
  if (*runner_ns != last_runner_)
  {
    last_runner_ = *runner_ns;
    last_change_ = steady;
  }
}

WaitClock WaitTimer::clock() const
{
  WaitClock c;
  c.timeout_s = static_cast<double>(timeout_.count()) / 1000.0;
  if (started_) c.elapsed_s = static_cast<double>(last_runner_ - t0_) / 1e9;
  if (started_ && !not_started_since_) c.now_ns = last_runner_;
  if (first_) return c;
  if (!started_ || not_started_since_)
  {
    const SteadyTime since = not_started_since_ ? *not_started_since_ : steady_start_;
    if (last_steady_ - since > max_age_)
      c.problem = "the robot's clock is not running (a simulation clock that has not started)";
    return c;
  }
  c.timed_out = last_runner_ - t0_ >= std::chrono::duration_cast<std::chrono::nanoseconds>(timeout_).count();
  if (last_steady_ - last_change_ > max_age_)
    c.problem = "the robot's clock stopped (it has not advanced for more than " + maxAgeText(max_age_) + ")";
  return c;
}

// --- SeqCounter -----------------------------------------------------------------------------------------------------
SeqCounter::SeqCounter(uint32_t initial) : initial_(initial == 0 ? 1u : initial)
{
}

uint32_t SeqCounter::next(uint32_t source_session)
{
  if (last_ == 0 || source_session != session_)
  {
    session_ = source_session;
    last_ = initial_;
    return last_;
  }
  last_ = rti::nextSeq(last_);   // +1, 2^32-1 -> 1, 0 never
  return last_;
}

// --- the user-facing texts ------------------------------------------------------------------------------------------
std::string requestedText(const std::string& service)
{
  return "MACHINE " + service + " REQUESTED";
}

std::string waitingText(const std::string& service, const std::string& why, double elapsed_s, double timeout_s)
{
  return "WAITING FOR MACHINE " + service + (why.empty() ? std::string() : " (" + why + ")") + ": " +
         wholeSeconds(elapsed_s) + " s of " + secondsText(timeout_s);
}

std::string busyText(const std::string& service, double elapsed_s, double timeout_s)
{
  return "MACHINE " + service + " BUSY: " + wholeSeconds(elapsed_s) + " s of " + secondsText(timeout_s);
}

std::string doneText(const std::string& service, std::optional<double> runner_s, double wall_s)
{
  if (!runner_s) return "MACHINE " + service + " DONE in " + secondsText(wall_s) + " (wall)";
  const std::string runner = secondsText(*runner_s);
  const std::string wall = secondsText(wall_s);
  return "MACHINE " + service + " DONE in " + runner + (runner == wall ? std::string() : " (" + wall + " wall)");
}

std::string readyText(const std::string& service, double waited_s)
{
  return "MACHINE " + service + " DONE: ready after " + secondsText(waited_s);
}

std::string failureText(uint16_t code, const std::string& service, const std::string& why)
{
  const char* word = code == station::TIMEOUT       ? "TIMEOUT"
                     : code == station::UNAVAILABLE ? "UNAVAILABLE"
                                                    : "FAULT";   // 306, and 313 ("the machine was still busy")
  return "MACHINE " + service + " " + word + " -- " + rti::codeName(code) + " (" + std::to_string(code) + "): " + why;
}

std::string programErrorText(const std::string& service, const std::string& why)
{
  return "machine service '" + service + "': " + why;
}

StartDiagnostic startDiagnostic(const std::string& service, const Feed& feed, SteadyTime now)
{
  StartDiagnostic s;
  if (!feed.hasImage())
  {
    s.warning = true;
    s.text = "no status for machine service '" + service +
             "' -- is the machine simulation (or the PLC) serving it?";
    return s;
  }
  if (!feed.live(now))
  {
    s.warning = true;
    s.text = "machine service '" + service + "': a status is seen but it is not live -- " +
             feed.whyNotLive(service, now);
    return s;
  }
  s.text = "machine service '" + service + "': status seen (" + stateWord(feed.last().state) + ", live)";
  return s;
}

}  // namespace trainit::machine
