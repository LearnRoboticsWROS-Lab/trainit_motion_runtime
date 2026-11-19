// test_p0b_channel_rules (P0.b, P0B_DESIGN.md §3.4 -- the FROZEN contract; P0B_IMPLEMENTATION_PLAN.md D4, D8, D10):
// the robot's machine-handshake rules, PURE (U level): no ROS, no threads, a fake STEADY clock and a fake runner
// clock. Every row of the three decision tables in their §3.4 ORDER (each row also proves the rows before it win), the
// liveness rules (§3.4, D8), the wait's clock (D10), the seq wrap from an injected counter (PB-P6) and the user-facing
// texts (owner decision 10: never a topic, a counter, or a word of the P0.a NEVER list). The S cases run the same
// rules through the real registry and nodes in test_p0b_machine_nodes.
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "trainit_motion_runtime/machine_channel.hpp"
#include "trainit_motion_runtime/rti_contract.hpp"

using namespace trainit::machine;
using std::chrono::milliseconds;

namespace
{
constexpr uint32_t kSession = 0xA1A1A1A1u;
constexpr uint32_t kMe = 0x0000BEEFu;   // this process's requester session
const std::string kSvc = "turner";

// The texts pinned word for word (protocol review MINOR-3: every count of two says that a crashed process's endpoint
// leaves the ROS graph within about 20 s; N3: an aborted cycle is "aborted", never "faulted").
const std::string kTwoSourcesWhy =
  "two sources publish its status (a crashed source's endpoint leaves the ROS graph within about 20 s)";
const std::string kTwoRequestersWhy =
  "another requester also writes its requests (one robot per machine service; a crashed requester's endpoint leaves "
  "the ROS graph within about 20 s)";
const std::string kAbortedWhy = "its cycle was aborted (the machine was reset before completing it)";

// The P0.a NEVER list (TSA's test_p0a_robot_tasks_launch.py): no machine text may carry one of these words.
const std::vector<std::string> kNever = {"REFUSED", "JOB LOST", "Robot LOST", "exit 2", "internal error",
                                         "PLAN FAILED", "VALIDATION FAILED"};

struct Rig
{
  SteadyTime now{std::chrono::steady_clock::now()};
  Feed feed{milliseconds(1000)};
  Image img;

  void advance(int ms) { now += milliseconds(ms); }
  void subscribe() { feed.subscribed(now); }
  void send() { feed.image(img, now); }
  void beat()
  {
    ++img.heartbeat;
    send();
  }
  void pubs(int status = 1, int request = 1) { feed.publishers(status, request); }
  // the usual start: subscribed, the baseline image, two heartbeat changes 100 ms apart, one publisher of each
  void live(State s = State::READY)
  {
    img.source_session = kSession;
    img.state = static_cast<uint8_t>(s);
    subscribe();
    pubs();
    send();
    advance(100);
    beat();
    advance(100);
    beat();
  }
  void state(State s)
  {
    img.state = static_cast<uint8_t>(s);
    send();
  }
  Decision request(std::optional<Written> last = std::nullopt, std::optional<SteadyTime> started = std::nullopt)
  {
    return decideRequest(kSvc, feed, now, started ? *started : now, last);
  }
};

Written mine(uint32_t seq, uint32_t session = kSession, std::optional<SteadyTime> at = std::nullopt)
{
  return Written{session, Identity{kMe, seq}, at};
}

Outstanding outstandingOf(uint32_t seq, SteadyTime at, bool completed = false, uint32_t session = kSession)
{
  Outstanding o;
  o.written = mine(seq, session);
  o.at = at;
  o.completed = completed;
  return o;
}

WaitClock clockAt(double elapsed_s, double timeout_s = 8.0)
{
  WaitClock c;
  c.elapsed_s = elapsed_s;
  c.timeout_s = timeout_s;
  c.timed_out = elapsed_s >= timeout_s;
  return c;
}

void expectNoNeverWord(const std::string& text)
{
  for (const auto& w : kNever) EXPECT_EQ(text.find(w), std::string::npos) << "'" << w << "' in: " << text;
  EXPECT_EQ(text.find('/'), std::string::npos) << "a topic-like name in: " << text;
}

void expectFailure(const Decision& d, uint16_t code, const std::string& word, const std::string& why_part)
{
  EXPECT_EQ(d.verdict, Verdict::FAILURE) << d.text;
  EXPECT_EQ(d.code, code) << d.text;
  if (code != 0)
  {
    EXPECT_NE(d.text.find("MACHINE " + kSvc + " " + word), std::string::npos) << d.text;
    EXPECT_NE(d.text.find(std::string(trainit::rti::codeName(code)) + " (" + std::to_string(code) + ")"),
              std::string::npos)
      << d.text;
  }
  EXPECT_NE(d.text.find(why_part), std::string::npos) << "'" << why_part << "' not in: " << d.text;
  expectNoNeverWord(d.text);
}
}  // namespace

// =====================================================================================================================
// Liveness (§3.4: the heartbeat CHANGED within the max age; D8)
// =====================================================================================================================
TEST(P0bLiveness, TheFirstImageIsABaselineNotASignOfLife)
{
  Rig r;
  r.img.source_session = kSession;
  r.img.state = static_cast<uint8_t>(State::READY);
  r.subscribe();
  r.pubs();
  r.send();   // a latched image reaches a late subscriber whatever the source's state
  EXPECT_TRUE(r.feed.hasImage());
  EXPECT_FALSE(r.feed.live(r.now)) << "a latched image alone is no sign of life";
  EXPECT_EQ(r.feed.heartbeatChanges(), 0);
  r.advance(100);
  r.beat();
  EXPECT_TRUE(r.feed.live(r.now));
  EXPECT_EQ(r.feed.heartbeatChanges(), 1);
  EXPECT_FALSE(r.feed.synchronised());
  r.advance(100);
  r.beat();
  EXPECT_TRUE(r.feed.synchronised());
}

TEST(P0bLiveness, LiveWhileTheHeartbeatChangesWithinTheMaxAge)
{
  Rig r;
  r.live();
  r.advance(1000);
  EXPECT_TRUE(r.feed.live(r.now)) << "exactly the max age: still live";
  r.advance(1);
  EXPECT_FALSE(r.feed.live(r.now));
  EXPECT_NE(r.feed.whyNotLive(kSvc, r.now).find("no status for more than 1.0 s"), std::string::npos)
    << r.feed.whyNotLive(kSvc, r.now);
}

// IM7: a source that still republishes a FROZEN image (its heartbeat stopped) is not live.
TEST(P0bLiveness, IM7_AFrozenImageIsNotLive)
{
  Rig r;
  r.live();
  for (int k = 0; k < 11; ++k)
  {
    r.advance(100);
    r.send();   // the same heartbeat, every period
  }
  EXPECT_FALSE(r.feed.live(r.now));
  EXPECT_NE(r.feed.whyNotLive(kSvc, r.now).find("frozen"), std::string::npos) << r.feed.whyNotLive(kSvc, r.now);
}

// PB-P5 (U): exactly ONE status publisher.
TEST(P0bLiveness, PB_P5_ExactlyOneStatusPublisher)
{
  Rig r;
  r.live();
  r.pubs(2, 1);
  EXPECT_FALSE(r.feed.live(r.now));
  EXPECT_EQ(r.feed.whyNotLive(kSvc, r.now), kTwoSourcesWhy);
  r.pubs(0, 1);
  EXPECT_FALSE(r.feed.live(r.now));
  EXPECT_NE(r.feed.whyNotLive(kSvc, r.now).find("no status source"), std::string::npos);
  r.pubs(1, 1);
  EXPECT_TRUE(r.feed.live(r.now));
}

// §3.4 literally (architecture review F2; it REPLACES D9, which counted a session change as a sign of life): LIVE = the
// heartbeat CHANGED within the max age. A session change is NEITHER a sign of life NOR a heartbeat change -- it only
// restarts the current session's change count (the request synchronises on two of the NEW session); the last sign of
// life stays the old session's last heartbeat change, so a healthy restart stays live through the new session's first
// heartbeat change (PB-P17(a)) only while that is within the max age; images whose session keeps changing (two sources
// with their own sessions, alternating) never show a sign of life.
TEST(P0bLiveness, ASessionChangeIsNeitherASignOfLifeNorAHeartbeatChange)
{
  Rig r;
  r.live(State::BUSY);   // the last heartbeat change: now
  r.advance(900);
  r.img.source_session = kSession + 1;
  r.img.state = static_cast<uint8_t>(State::READY);
  r.img.heartbeat = 0;
  r.send();              // a NEW session, 900 ms after the old one's last heartbeat change
  EXPECT_TRUE(r.feed.live(r.now)) << "the old session's last heartbeat change is within the max age";
  EXPECT_EQ(r.feed.heartbeatChanges(), 0);
  EXPECT_FALSE(r.feed.synchronised());
  r.advance(101);        // 1001 ms after the last heartbeat change: the session change refreshed nothing
  EXPECT_FALSE(r.feed.live(r.now)) << "a session change is no sign of life (D9 said it was)";
  EXPECT_NE(r.feed.whyNotLive(kSvc, r.now).find("no status update for more than 1.0 s"), std::string::npos)
    << r.feed.whyNotLive(kSvc, r.now);
  r.beat();              // the NEW session's first heartbeat change: a sign of life
  EXPECT_TRUE(r.feed.live(r.now));
  EXPECT_EQ(r.feed.heartbeatChanges(), 1);
  EXPECT_FALSE(r.feed.synchronised());
  r.advance(100);
  r.beat();
  EXPECT_TRUE(r.feed.synchronised());
  EXPECT_TRUE(r.feed.lastLife() == r.now) << "the last sign of life is the new session's heartbeat change";

  // two sources, each with its OWN session and a moving heartbeat, alternating: every image is a session change
  Rig t;
  t.live(State::BUSY);
  Image other = t.img;
  other.source_session = kSession + 7;
  for (int k = 0; k < 11; ++k)
  {
    t.advance(50);
    ++other.heartbeat;
    t.feed.image(other, t.now);
    t.advance(50);
    t.beat();
  }
  EXPECT_FALSE(t.feed.live(t.now)) << "1100 ms of session changes: no sign of life";
  EXPECT_EQ(t.feed.heartbeatChanges(), 0);
}

TEST(P0bLiveness, ADefaultImageSaysNothingAndAnInvalidStateIsNotLive)
{
  Rig r;
  r.subscribe();
  r.pubs();
  r.img.source_session = 0;   // a default image
  r.img.state = static_cast<uint8_t>(State::READY);
  r.send();
  EXPECT_FALSE(r.feed.hasImage());
  r.live(State::NONE);
  EXPECT_FALSE(r.feed.live(r.now));
  EXPECT_EQ(r.feed.whyNotLive(kSvc, r.now), "it reports no valid state");
  r.img.state = 9;
  r.advance(100);
  r.beat();
  EXPECT_FALSE(r.feed.live(r.now));
}

TEST(P0bLiveness, TheHeartbeatWrapIsAChange)
{
  Rig r;
  r.img.heartbeat = 65534;
  r.live();
  EXPECT_EQ(r.img.heartbeat, 0);   // 65534 -> 65535 -> 0
  EXPECT_EQ(r.feed.heartbeatChanges(), 2);
  EXPECT_TRUE(r.feed.live(r.now));
}

// D8: a source KNOWN dead -- no status ever and the subscription older than the max age, or the last image older
// than the max age.
TEST(P0bLiveness, D8_KnownDead)
{
  Rig r;
  r.subscribe();
  r.advance(1000);
  EXPECT_FALSE(r.feed.knownDead(r.now)) << "a subscription of exactly the max age may still receive";
  r.advance(1);
  EXPECT_TRUE(r.feed.knownDead(r.now));
  Rig s;
  s.live();
  s.advance(1001);
  EXPECT_TRUE(s.feed.knownDead(s.now));
  Rig t;
  t.live();
  for (int k = 0; k < 20; ++k)
  {
    t.advance(100);
    t.send();   // frozen but fresh: not KNOWN dead (the request synchronises, bounded)
  }
  EXPECT_FALSE(t.feed.knownDead(t.now));
  EXPECT_FALSE(t.feed.live(t.now));
}

// =====================================================================================================================
// RequestMachine: synchronise (D8), then 1. 310, 2. 306, 3. 313, 4. write -- in this order
// =====================================================================================================================
// F9 (U): no live status source -> 310 at once, nothing written.
TEST(P0bRequest, F9_NoStatusEverDecidesAtOnce)
{
  Rig r;
  r.subscribe();
  r.advance(3000);   // the runner's settle
  const Decision d = r.request();
  expectFailure(d, station::UNAVAILABLE, "UNAVAILABLE", "no status for service 'turner'");
  EXPECT_NE(d.text.find("nothing was requested"), std::string::npos);
}

TEST(P0bRequest, D8_AStaleSourceDecidesAtOnce)
{
  Rig r;
  r.live();
  r.advance(1500);   // the source stopped publishing 1.5 s ago
  expectFailure(r.request(), station::UNAVAILABLE, "UNAVAILABLE", "no status for more than 1.0 s");
}

// PB-P18 (U): a request right after the subscription synchronises (RUNNING) and then writes -- never a start-up 310.
TEST(P0bRequest, PB_P18_SynchronisesOnAFreshSourceThenWrites)
{
  Rig r;
  r.img.source_session = kSession;
  r.img.state = static_cast<uint8_t>(State::READY);
  r.subscribe();
  const SteadyTime started = r.now;
  EXPECT_EQ(r.request(std::nullopt, started).verdict, Verdict::RUNNING) << "no image yet, a young subscription";
  r.advance(30);
  r.send();
  EXPECT_EQ(r.request(std::nullopt, started).phase, Phase::SYNC);
  r.pubs();
  r.advance(100);
  r.beat();
  EXPECT_EQ(r.request(std::nullopt, started).verdict, Verdict::RUNNING) << "one heartbeat change";
  r.advance(100);
  r.beat();
  const Decision d = r.request(std::nullopt, started);
  EXPECT_EQ(d.verdict, Verdict::WRITE) << d.text;
  EXPECT_EQ(d.source_session, kSession);
  EXPECT_EQ(d.text, "MACHINE turner REQUESTED");
}

// D8: the synchronisation is bounded by the max age (steady) from the request's start.
TEST(P0bRequest, D8_TheSynchronisationIsBounded)
{
  Rig r;
  r.img.source_session = kSession;
  r.img.state = static_cast<uint8_t>(State::READY);
  r.subscribe();
  r.pubs();
  r.send();
  const SteadyTime started = r.now;
  for (int k = 0; k < 10; ++k)
  {
    r.advance(100);
    r.send();   // fresh, frozen
    EXPECT_EQ(r.request(std::nullopt, started).verdict, Verdict::RUNNING) << k;
  }
  r.advance(1);
  expectFailure(r.request(std::nullopt, started), station::UNAVAILABLE, "UNAVAILABLE", "frozen");
  // ... and a source with ONE heartbeat change in the bound is not synchronised: 310, never a write
  Rig s;
  s.img.source_session = kSession;
  s.img.state = static_cast<uint8_t>(State::READY);
  s.subscribe();
  s.pubs();
  s.send();
  const SteadyTime s0 = s.now;
  s.advance(600);
  s.beat();
  s.advance(401);
  expectFailure(s.request(std::nullopt, s0), station::UNAVAILABLE, "UNAVAILABLE", "two signs of life within 1.0 s");
}

// D8: the publisher count not yet refreshed (0) is waited for within the bound -- never read as "no source".
TEST(P0bRequest, D8_AnUncountedStatusPublisherIsWaitedFor)
{
  Rig r;
  r.live();
  r.pubs(0, 0);
  const SteadyTime started = r.now;
  EXPECT_EQ(r.request(std::nullopt, started).verdict, Verdict::RUNNING);
  r.pubs(1, 1);
  EXPECT_EQ(r.request(std::nullopt, started).verdict, Verdict::WRITE);
}

TEST(P0bRequest, Row1_NotLiveWinsOverEverything)
{
  Rig r;
  r.live(State::FAULT);   // 2. would say 306
  r.advance(1100);
  r.send();               // fresh but frozen for 1.1 s
  const SteadyTime started = r.now - milliseconds(1001);
  expectFailure(r.request(std::nullopt, started), station::UNAVAILABLE, "UNAVAILABLE", "frozen");
}

TEST(P0bRequest, Row1_TwoSourcesAndTwoRequestersAre310)
{
  Rig r;
  r.live(State::FAULT);
  r.pubs(2, 1);
  Decision d = r.request();
  expectFailure(d, station::UNAVAILABLE, "UNAVAILABLE", kTwoSourcesWhy);
  EXPECT_EQ(d.text, "MACHINE turner UNAVAILABLE -- STATION_UNAVAILABLE (310): " + kTwoSourcesWhy +
                      "; nothing was requested");
  r.pubs(1, 2);   // PB-P11 (U): a second publisher on the service's request
  d = r.request();
  expectFailure(d, station::UNAVAILABLE, "UNAVAILABLE", kTwoRequestersWhy);
  EXPECT_EQ(d.text, "MACHINE turner UNAVAILABLE -- STATION_UNAVAILABLE (310): " + kTwoRequestersWhy +
                      "; nothing was requested");
}

TEST(P0bRequest, Row2_FaultIs306)
{
  Rig r;
  r.live(State::FAULT);
  r.img.fault_code = static_cast<uint16_t>(FaultCode::ACTUATOR_TIMEOUT);
  r.send();
  r.pubs(1, 1);
  expectFailure(r.request(mine(3)), station::FAULT, "FAULT", "the machine reports FAULT (ACTUATOR_TIMEOUT); nothing");
}

// PB-P2(a) / PB-P23 (U): BUSY or HOMING -> 313, nothing written (not a fault of the machine: it was still busy).
TEST(P0bRequest, Row3_PB_P2a_PB_P23_BusyAndHomingAre313)
{
  Rig r;
  r.live(State::BUSY);
  Decision d = r.request();
  expectFailure(d, station::BUSY, "FAULT", "the machine was still busy (BUSY)");
  EXPECT_NE(d.verdict, Verdict::WRITE);
  r.state(State::HOMING);
  d = r.request();
  expectFailure(d, station::BUSY, "FAULT", "the machine was still busy (HOMING after a reset)");
}

// PB-P6 / PB-P14 (U): this process's previous request to THIS source session not yet consumed -> 313 (never a second
// request with one seq, never a depth-1 overwrite).
TEST(P0bRequest, Row3_PB_P14_TheUnconsumedPreviousRequestIs313)
{
  Rig r;
  r.live(State::READY);
  expectFailure(r.request(mine(41)), station::BUSY, "FAULT", "it has not yet taken the previous request");
  // protocol review MINOR-2: the words of §3.5 ("the machine was still busy"), the parenthetical, and how long ago the
  // previous request was written
  EXPECT_EQ(r.request(mine(41, kSession, r.now - milliseconds(12300))).text,
            "MACHINE turner FAULT -- STATION_BUSY (313): the machine was still busy (it has not yet taken the previous "
            "request, written 12.3 s ago); nothing was requested");
  EXPECT_EQ(r.request(mine(41)).text,
            "MACHINE turner FAULT -- STATION_BUSY (313): the machine was still busy (it has not yet taken the previous "
            "request); nothing was requested")
    << "a write time not known: the parenthetical alone";
  // consumed as the cycle -> writes
  r.img.cycle = Identity{kMe, 41};
  r.img.cycle_done = true;
  r.state(State::DONE);
  EXPECT_EQ(r.request(mine(41)).verdict, Verdict::WRITE);
  // consumed as a rejection -> writes
  Rig s;
  s.live(State::READY);
  s.img.reject = Identity{kMe, 41};
  s.img.reject_code = static_cast<uint8_t>(Reject::BUSY);
  s.send();
  EXPECT_EQ(s.request(mine(41)).verdict, Verdict::WRITE);
  // the same seq of ANOTHER requester never consumes it (identity equality)
  Rig t;
  t.live(State::READY);
  t.img.cycle = Identity{kMe + 1, 41};
  t.send();
  expectFailure(t.request(mine(41)), station::BUSY, "FAULT", "it has not yet taken the previous request");
}

// Rule 3, the M-tier finding (P0.b WP6, M run 1): CONSUMED is an EVENT. The robot's previous request was rejected and
// the robot SAW it (an image named it); a SECOND requester's later rejection then displaced it from `reject_*` (PB-P11's
// aftermath). It stays consumed: the next request is written -- before the latch the robot was locked out with 313
// ("still busy, not yet taken") until the source restarted. A request no image ever named stays "not yet taken" (313):
// the latch never invents a consumption, and a new write re-arms it.
TEST(P0bRequest, Row3_ConsumedIsLatchedAcrossAnotherRequestersLaterRejection)
{
  Rig r;
  r.live(State::READY);
  r.feed.wrote(mine(26));
  expectFailure(r.request(mine(26)), station::BUSY, "FAULT", "it has not yet taken the previous request");
  r.img.reject = Identity{kMe, 26};
  r.img.reject_code = static_cast<uint8_t>(Reject::TWO_REQUESTERS);
  r.send();                                                    // the robot sees its request consumed (rejected)
  EXPECT_TRUE(r.feed.consumed(mine(26)));
  r.img.reject = Identity{0x5C1D0011u, 1};                     // another requester's rejection displaces it
  r.send();
  EXPECT_TRUE(r.feed.consumed(mine(26)));
  EXPECT_EQ(r.request(mine(26)).verdict, Verdict::WRITE);
  // the same through a CYCLE later displaced by another requester's cycle
  Rig c;
  c.live(State::READY);
  c.feed.wrote(mine(7));
  c.img.cycle = Identity{kMe, 7};
  c.img.cycle_done = true;
  c.state(State::DONE);
  c.img.cycle = Identity{0x5C1D0011u, 2};
  c.state(State::DONE);
  EXPECT_EQ(c.request(mine(7)).verdict, Verdict::WRITE);
  // never named (displaced before it was consumed): still 313 -- the latch invents nothing
  Rig n;
  n.live(State::READY);
  n.feed.wrote(mine(30));
  n.img.reject = Identity{0x5C1D0011u, 3};
  n.send();
  EXPECT_FALSE(n.feed.consumed(mine(30)));
  expectFailure(n.request(mine(30)), station::BUSY, "FAULT", "it has not yet taken the previous request");
  // a new write re-arms it; an image of ANOTHER session never consumes it
  Rig w;
  w.live(State::READY);
  w.feed.wrote(mine(5));
  w.img.cycle = Identity{kMe, 5};
  w.send();
  EXPECT_TRUE(w.feed.consumed(mine(5)));
  w.feed.wrote(mine(6));
  EXPECT_FALSE(w.feed.consumed(mine(6)));
  EXPECT_FALSE(w.feed.consumed(mine(5)));                      // only the LAST written one is tracked
  Image other = w.img;
  other.source_session = kSession + 1;
  other.cycle = Identity{kMe, 6};
  w.feed.image(other, w.now);
  EXPECT_FALSE(w.feed.consumed(mine(6)));
}

// Rule 3, C++ review M2: wrote() is called BEFORE the publish (the consumed latch stays race-free); when the publish
// THROWS, the request never reached the wire and the registry still names the PREVIOUS request as its last written one.
// The Feed's snapshot of the previous written state is restored (unwrote): the previous request is watched again WITH
// its consumption -- before the fix the Feed kept watching the never-published identity, consumed(previous) stayed
// false for ever, and every later request was refused 313 until the source restarted.
TEST(P0bRequest, Row3_AFailedPublishRestoresThePreviousWrittenRequest)
{
  Rig r;
  r.live(State::READY);
  r.feed.wrote(mine(5));
  r.img.cycle = Identity{kMe, 5};
  r.img.cycle_done = true;
  r.state(State::DONE);                       // 5 consumed: its cycle completed
  EXPECT_TRUE(r.feed.consumed(mine(5)));
  r.img.cycle = Identity{0x5C1D0011u, 1};     // ... then displaced by another requester's cycle
  r.state(State::DONE);
  r.feed.wrote(mine(6));                      // the next request: its publish throws
  EXPECT_FALSE(r.feed.consumed(mine(5)));     // (only the LAST written one is tracked)
  r.feed.unwrote();                           // the snapshot restored
  EXPECT_TRUE(r.feed.consumed(mine(5)));
  EXPECT_FALSE(r.feed.consumed(mine(6))) << "never on the wire, never consumed";
  EXPECT_EQ(r.request(mine(5)).verdict, Verdict::WRITE) << "the last written request (5) is consumed: no lock-out";
  // an image naming the previous request applied BETWEEN wrote() and the failed publish keeps its consumption
  Rig w;
  w.live(State::READY);
  w.feed.wrote(mine(7));                      // written, not yet consumed
  w.feed.wrote(mine(8));                      // the next one: its publish will throw
  w.img.reject = Identity{kMe, 7};            // meanwhile the source consumed 7 (rejected) ...
  w.img.reject_code = static_cast<uint8_t>(Reject::BUSY);
  w.send();
  w.img.reject = Identity{0x5C1D0011u, 2};    // ... and another requester's rejection displaced it
  w.send();
  w.feed.unwrote();
  EXPECT_TRUE(w.feed.consumed(mine(7)));
  EXPECT_FALSE(w.feed.consumed(mine(8)));
  EXPECT_EQ(w.request(mine(7)).verdict, Verdict::WRITE);
  // not consumed before the failed publish: still not consumed after the restore (it invents nothing) -> 313
  Rig n;
  n.live(State::READY);
  n.feed.wrote(mine(9));
  n.feed.wrote(mine(10));
  n.feed.unwrote();
  EXPECT_FALSE(n.feed.consumed(mine(9)));
  expectFailure(n.request(mine(9)), station::BUSY, "FAULT", "it has not yet taken the previous request");
  // without a pending wrote() an unwrote() changes nothing; the first wrote() restores to "nothing written"
  n.feed.unwrote();
  n.img.cycle = Identity{kMe, 9};
  n.send();
  EXPECT_TRUE(n.feed.consumed(mine(9))) << "9 is still the one watched";
  Rig f;
  f.live(State::READY);
  f.feed.wrote(mine(1));
  f.feed.unwrote();
  f.img.cycle = Identity{kMe, 1};
  f.send();
  EXPECT_FALSE(f.feed.consumed(mine(1))) << "a request whose publish failed is never watched";
  EXPECT_EQ(f.request(std::nullopt).verdict, Verdict::WRITE);
}

// PB-P19 (U): a request written for an OLD source session never blocks one to the restarted source.
TEST(P0bRequest, Row3_PB_P19_ARequestForAnotherSessionDoesNotBlock)
{
  Rig r;
  r.live(State::READY);
  EXPECT_EQ(r.request(mine(9, kSession - 5)).verdict, Verdict::WRITE);
}

// PB-P1 (U): the DONE of the previous request still visible: a new request is written (DONE is idle).
TEST(P0bRequest, Row4_PB_P1_ReadyOrDoneWrites)
{
  Rig r;
  r.live(State::DONE);
  r.img.cycle = Identity{kMe, 7};
  r.img.cycle_done = true;
  r.send();
  const Decision d = r.request(mine(7));
  EXPECT_EQ(d.verdict, Verdict::WRITE);
  EXPECT_EQ(d.source_session, kSession);
  Rig s;
  s.live(State::READY);
  EXPECT_EQ(s.request().verdict, Verdict::WRITE);
}

// =====================================================================================================================
// WaitForMachine for="done": 1 .. 8, in this order
// =====================================================================================================================
// PB-P7 / PB-P16 (U): no request of this job on the service -> FAILURE WITHOUT a station code (D4), whatever the
// status shows (job A's DONE never matches job B's wait).
TEST(P0bWaitDone, Row1_PB_P7_PB_P16_NoRequestInThisJobFailsWithoutACode)
{
  Rig r;
  r.live(State::DONE);
  r.img.cycle = Identity{kMe, 3};
  r.img.cycle_done = true;
  r.send();
  const Decision d = decideWaitDone(kSvc, r.feed, r.now, std::nullopt, clockAt(0));
  expectFailure(d, 0, "", "a wait for done with no request on it in this job");
  EXPECT_EQ(d.text.rfind("machine service 'turner': ", 0), 0u) << d.text;
}

// PB-P3 (U): the source restarted after the request -> 310 at once ("restarted"), before liveness, rejection, DONE.
TEST(P0bWaitDone, Row2_PB_P3_ARestartedSourceIs310AtOnce)
{
  Rig r;
  r.live(State::DONE);
  const auto o = outstandingOf(5, r.now);
  r.img.source_session = kSession + 1;   // a new session, READY, no cycle
  r.img.state = static_cast<uint8_t>(State::READY);
  r.img.cycle = Identity{kMe, 5};        // even a (forged) DONE of the same identity
  r.img.cycle_done = true;
  r.send();
  expectFailure(decideWaitDone(kSvc, r.feed, r.now, o, clockAt(1)), station::UNAVAILABLE, "UNAVAILABLE",
                "the machine restarted");
  // protocol review MINOR-1: the other session comes from a SECOND source (two status publishers counted) -- the same
  // 310 by rule 2, before liveness, but its words say two sources, never "restarted"
  r.pubs(2, 1);
  const Decision d = decideWaitDone(kSvc, r.feed, r.now, o, clockAt(1));
  expectFailure(d, station::UNAVAILABLE, "UNAVAILABLE", kTwoSourcesWhy);
  EXPECT_EQ(d.text.find("restarted"), std::string::npos) << d.text;
}

TEST(P0bWaitDone, Row3_NotLiveIs310BeforeItsDone)
{
  Rig r;
  r.live(State::BUSY);
  const auto o = outstandingOf(5, r.now);
  r.img.cycle = Identity{kMe, 5};
  r.img.cycle_done = true;
  r.state(State::DONE);
  r.advance(1100);   // ... then the source died
  expectFailure(decideWaitDone(kSvc, r.feed, r.now, o, clockAt(1.1)), station::UNAVAILABLE, "UNAVAILABLE",
                "no status for more than 1.0 s");
  Rig s;   // F7 (U): two sources
  s.live(State::BUSY);
  s.pubs(2, 1);
  expectFailure(decideWaitDone(kSvc, s.feed, s.now, outstandingOf(5, s.now), clockAt(0)), station::UNAVAILABLE,
                "UNAVAILABLE", "two sources");
}

// PB-P2(b) / PB-P22 (U): the request REJECTED -> 313 (BUSY) / 306 (FAULTED) / 310 (TWO_REQUESTERS), at once.
TEST(P0bWaitDone, Row4_PB_P2b_TheRejectionAnswersAtOnce)
{
  struct Case
  {
    Reject reason;
    uint16_t code;
    const char* word;
    const char* why;
  };
  const Case cases[] = {
    {Reject::BUSY, station::BUSY, "FAULT", "the machine was still busy and did not take the request"},
    {Reject::FAULTED, station::FAULT, "FAULT", "the machine was in FAULT and did not take the request"},
    {Reject::TWO_REQUESTERS, station::UNAVAILABLE, "UNAVAILABLE", "the machine saw two requesters"},
    {static_cast<Reject>(9), station::FAULT, "FAULT", "an unknown reason"},
  };
  for (const auto& c : cases)
  {
    Rig r;
    r.live(State::BUSY);
    r.img.cycle = Identity{0x77, 1};    // the RUNNING cycle is another one
    r.img.reject = Identity{kMe, 5};
    r.img.reject_code = static_cast<uint8_t>(c.reason);
    r.send();
    expectFailure(decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(0)), c.code, c.word, c.why);
  }
  // ... and the waiter of the RUNNING cycle is not disturbed by another identity's rejection
  Rig r;
  r.live(State::BUSY);
  r.img.cycle = Identity{kMe, 6};
  r.img.reject = Identity{0x77, 2};
  r.img.reject_code = static_cast<uint8_t>(Reject::BUSY);
  r.send();
  EXPECT_EQ(decideWaitDone(kSvc, r.feed, r.now, outstandingOf(6, r.now), clockAt(0)).verdict, Verdict::RUNNING);
}

TEST(P0bWaitDone, Row4_ARejectionWinsOverACycleNamingTheSameIdentity)
{
  Rig r;
  r.live(State::DONE);
  r.img.cycle = Identity{kMe, 5};
  r.img.cycle_done = true;
  r.img.reject = Identity{kMe, 5};
  r.img.reject_code = static_cast<uint8_t>(Reject::BUSY);
  r.send();
  expectFailure(decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(0)), station::BUSY, "FAULT",
                "still busy");
}

// N1 / N4 / PB-P15 (U): the cycle of THIS request completed -> SUCCESS, checked BEFORE any fault; a DONE of an earlier
// request (PB-P1) or of another requester never matches.
TEST(P0bWaitDone, Row5_N4_PB_P15_ItsDoneWinsEvenOverALaterFault)
{
  Rig r;
  r.live(State::BUSY);
  const SteadyTime written = r.now;
  r.img.cycle = Identity{kMe, 5};
  r.img.cycle_done = true;
  r.state(State::DONE);
  r.advance(2300);
  r.beat();
  Decision d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, written), clockAt(0));
  EXPECT_EQ(d.verdict, Verdict::SUCCESS);
  EXPECT_TRUE(d.completes);
  EXPECT_EQ(d.text, "MACHINE turner DONE in 2.3 s (wall)") << "no runner time for the request: the wall time, marked";
  r.img.state = static_cast<uint8_t>(State::FAULT);   // PB-P15: an idle fault after the completion
  r.img.fault_code = static_cast<uint16_t>(FaultCode::STATION_FAULT);
  r.beat();
  d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, written), clockAt(0));
  EXPECT_EQ(d.verdict, Verdict::SUCCESS) << d.text;
}

// Acceptance finding 1 (2026-11-19): "DONE in" is the request's age on the runner's clock -- the clock of the BUSY
// counter and of the timeout (D10) -- with the wall time in brackets only when the two read differently (an Isaac
// window at RTF 0.24: "BUSY 5 s of 12.0 s" then "DONE in 9.6 s" read as a contradiction).
TEST(P0bWaitDone, Row5_DoneInIsOnTheTimeoutsClockWithTheWallTimeInBrackets)
{
  Rig r;
  r.live(State::BUSY);
  const SteadyTime written = r.now;
  r.img.cycle = Identity{kMe, 5};
  r.img.cycle_done = true;
  r.state(State::DONE);
  r.advance(9600);
  r.beat();
  Outstanding o = outstandingOf(5, written);
  o.runner_ns = 10'000'000'000;
  WaitClock c = clockAt(5.0, 12.0);
  c.now_ns = 15'430'000'000;   // a slow simulation: 5.43 s of runner time in 9.6 s of wall time
  EXPECT_EQ(decideWaitDone(kSvc, r.feed, r.now, o, c).text, "MACHINE turner DONE in 5.4 s (9.6 s wall)");
  c.now_ns = 19'570'000'000;   // the runner's clock IS wall time (no simulation): no brackets
  EXPECT_EQ(decideWaitDone(kSvc, r.feed, r.now, o, c).text, "MACHINE turner DONE in 9.6 s");
  c.now_ns = 9'000'000'000;    // the runner's clock jumped back after the request (a simulation reset)
  EXPECT_EQ(decideWaitDone(kSvc, r.feed, r.now, o, c).text, "MACHINE turner DONE in 9.6 s (wall)");
  c.now_ns.reset();            // the runner's clock not running at this tick
  EXPECT_EQ(decideWaitDone(kSvc, r.feed, r.now, o, c).text, "MACHINE turner DONE in 9.6 s (wall)");
}

TEST(P0bWaitDone, Row5_PB_P1_ADoneOfAnotherIdentityNeverMatches)
{
  for (const Identity other : {Identity{kMe, 4}, Identity{kMe + 1, 5}, Identity{0, 0}})
  {
    Rig r;
    r.live(State::DONE);
    r.img.cycle = other;
    r.img.cycle_done = true;
    r.send();
    const Decision d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(0.5));
    EXPECT_EQ(d.verdict, Verdict::RUNNING) << d.text;
    EXPECT_EQ(d.phase, Phase::WAITING);
    EXPECT_EQ(d.text, "WAITING FOR MACHINE turner: 0 s of 8.0 s");
  }
}

// F1 / F3 (U): a FAULT while the request is outstanding -> 306; PB-P22(a): its cycle aborted (a fault, then a RESET
// before the wait looked) -> 306 at once, never a timeout.
TEST(P0bWaitDone, Row6_F1_F3_PB_P22a_FaultAndAbortedAre306)
{
  Rig r;
  r.live(State::BUSY);
  r.img.cycle = Identity{kMe, 5};
  r.img.fault_code = static_cast<uint16_t>(FaultCode::ACTUATOR_TIMEOUT);
  r.state(State::FAULT);
  expectFailure(decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(9)), station::FAULT, "FAULT",
                "the machine reports FAULT (ACTUATOR_TIMEOUT)");
  for (const State idle : {State::READY, State::HOMING, State::DONE})
  {
    Rig s;
    s.live(State::BUSY);
    s.img.cycle = Identity{kMe, 5};
    s.img.cycle_done = false;
    s.state(idle);
    const Decision d = decideWaitDone(kSvc, s.feed, s.now, outstandingOf(5, s.now), clockAt(9));
    expectFailure(d, station::FAULT, "FAULT", "its cycle was aborted");
    // protocol review N3: a RESET from BUSY with no fault aborts a cycle too -- "aborted", never "faulted"
    EXPECT_EQ(d.text, "MACHINE turner FAULT -- STATION_FAULT (306): " + kAbortedWhy);
  }
}

// F2 / F5 (U): the timeout on the runner's clock -> 305 with the last state; the clock problems of D10 first.
TEST(P0bWaitDone, Row7_F2_TheTimeoutNamesTheLastState)
{
  Rig r;
  r.live(State::BUSY);
  r.img.cycle = Identity{kMe, 5};
  r.send();
  expectFailure(decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(8.0)), station::TIMEOUT,
                "TIMEOUT", "still BUSY after 8.0 s");
  Rig s;
  s.live(State::READY);   // never taken
  expectFailure(decideWaitDone(kSvc, s.feed, s.now, outstandingOf(5, s.now), clockAt(8.0)), station::TIMEOUT,
                "TIMEOUT", "the request not yet taken after 8.0 s (the machine is READY)");
  WaitClock stopped = clockAt(3);
  stopped.problem = "the robot's clock stopped (it has not advanced for more than 1.0 s)";
  const Decision d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), stopped);
  expectFailure(d, 0, "", "the robot's clock stopped");
}

TEST(P0bWaitDone, Row8_RunningIsWaitingOrBusy)
{
  Rig r;
  r.live(State::READY);
  Decision d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(0.4));
  EXPECT_EQ(d.verdict, Verdict::RUNNING);
  EXPECT_EQ(d.phase, Phase::WAITING);
  EXPECT_EQ(d.text, "WAITING FOR MACHINE turner: 0 s of 8.0 s");
  r.img.cycle = Identity{kMe, 5};
  r.state(State::BUSY);
  d = decideWaitDone(kSvc, r.feed, r.now, outstandingOf(5, r.now), clockAt(2.7));
  EXPECT_EQ(d.phase, Phase::BUSY);
  EXPECT_EQ(d.text, "MACHINE turner BUSY: 2 s of 8.0 s");
}

// =====================================================================================================================
// WaitForMachine for="ready"
// =====================================================================================================================
// N3 (U): with no request of this job pending, READY and DONE (idle) are both ready.
TEST(P0bWaitReady, N3_ReadyAndDoneAreReady)
{
  for (const State s : {State::READY, State::DONE})
  {
    Rig r;
    r.live(s);
    const Decision d = decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(0, 2.0));
    EXPECT_EQ(d.verdict, Verdict::SUCCESS) << d.text;
    EXPECT_FALSE(d.completes);
    EXPECT_EQ(d.text, "MACHINE turner DONE: ready after 0.0 s");
  }
}

TEST(P0bWaitReady, NoPending_BusyAndHomingKeepWaitingFaultIs306NotLiveIs310TimeoutIs305)
{
  Rig r;
  r.live(State::HOMING);
  Decision d = decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(1.2, 2.0));
  EXPECT_EQ(d.verdict, Verdict::RUNNING);
  EXPECT_EQ(d.text, "WAITING FOR MACHINE turner (it is HOMING): 1 s of 2.0 s");
  r.state(State::BUSY);
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(1.2, 2.0)).verdict,
            Verdict::RUNNING);
  expectFailure(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(2.0, 2.0)), station::TIMEOUT,
                "TIMEOUT",
                "still not ready after 2.0 s (the machine is BUSY)");
  r.state(State::FAULT);
  expectFailure(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(0, 2.0)), station::FAULT,
                "FAULT",
                "the machine reports FAULT");
  r.advance(1200);
  expectFailure(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(0, 2.0)), station::UNAVAILABLE,
                "UNAVAILABLE", "no status for more than 1.0 s");
}

// PB-P17(a) (U): a source restart with no request pending is no failure -- the restarted, homed source that reports
// READY is ready.
TEST(P0bWaitReady, PB_P17a_ARestartWithoutARequestIsNoFailure)
{
  Rig r;
  r.live(State::BUSY);
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(0, 2.0)).verdict,
            Verdict::RUNNING);
  r.advance(300);
  r.img.source_session = kSession + 1;
  r.img.state = static_cast<uint8_t>(State::READY);
  r.img.cycle = Identity{};
  r.send();
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, std::nullopt, clockAt(0.3, 2.0)).verdict,
            Verdict::SUCCESS);
}

// PB-P17(b) / PB-P12 (U): with this job's request pending, a restart is 310; the READY of BEFORE the request never
// counts (B1); only its completion + READY / DONE.
TEST(P0bWaitReady, PB_P12_PB_P17b_APendingRequestMustComplete)
{
  Rig r;
  r.live(State::READY);   // the status of BEFORE the request
  const auto o = outstandingOf(5, r.now);
  Decision d = decideWaitReady(kSvc, r.feed, r.now, o, std::nullopt, clockAt(0.1, 2.0));
  EXPECT_EQ(d.verdict, Verdict::RUNNING) << "B1: never the status of before the request";
  EXPECT_EQ(d.phase, Phase::WAITING);
  r.img.cycle = Identity{kMe, 5};
  r.state(State::BUSY);
  d = decideWaitReady(kSvc, r.feed, r.now, o, std::nullopt, clockAt(0.5, 2.0));
  EXPECT_EQ(d.phase, Phase::BUSY);
  EXPECT_EQ(d.text, "MACHINE turner BUSY: 0 s of 2.0 s");
  r.img.cycle_done = true;
  r.state(State::DONE);
  d = decideWaitReady(kSvc, r.feed, r.now, o, std::nullopt, clockAt(0.9, 2.0));
  EXPECT_EQ(d.verdict, Verdict::SUCCESS);
  EXPECT_TRUE(d.completes) << "the ready-wait completes the request";
  // its cycle done, then a fault and a RESET: HOMING keeps waiting, READY is ready
  r.state(State::HOMING);
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, o, std::nullopt, clockAt(1.0, 2.0)).verdict, Verdict::RUNNING);
  // PB-P17(b): a restart while it is pending
  Rig s;
  s.live(State::BUSY);
  const auto p = outstandingOf(5, s.now);
  s.img.source_session = kSession + 1;
  s.img.state = static_cast<uint8_t>(State::READY);
  s.send();
  expectFailure(decideWaitReady(kSvc, s.feed, s.now, p, std::nullopt, clockAt(0.2, 2.0)), station::UNAVAILABLE,
                "UNAVAILABLE",
                "the machine restarted");
  // protocol review MINOR-1: the other session from a SECOND source (two status publishers counted): two sources
  s.pubs(2, 1);
  const Decision two = decideWaitReady(kSvc, s.feed, s.now, p, std::nullopt, clockAt(0.2, 2.0));
  expectFailure(two, station::UNAVAILABLE, "UNAVAILABLE", kTwoSourcesWhy);
  EXPECT_EQ(two.text.find("restarted"), std::string::npos) << two.text;
}

// PB-P22(b) (U): a ready-wait on a REJECTED request -> 313 / 306 / 310 at once; its aborted cycle -> 306.
TEST(P0bWaitReady, PB_P22b_ARejectedOrAbortedPendingRequest)
{
  const std::pair<Reject, uint16_t> rows[] = {
    {Reject::BUSY, station::BUSY}, {Reject::FAULTED, station::FAULT}, {Reject::TWO_REQUESTERS, station::UNAVAILABLE}};
  for (const auto& [reason, code] : rows)
  {
    Rig r;
    r.live(State::READY);
    r.img.reject = Identity{kMe, 5};
    r.img.reject_code = static_cast<uint8_t>(reason);
    r.send();
    const Decision d = decideWaitReady(kSvc, r.feed, r.now, outstandingOf(5, r.now), std::nullopt, clockAt(0, 2.0));
    EXPECT_EQ(d.verdict, Verdict::FAILURE);
    EXPECT_EQ(d.code, code) << d.text;
  }
  Rig a;
  a.live(State::BUSY);
  a.img.cycle = Identity{kMe, 5};
  a.state(State::READY);   // aborted: taken, not done, idle
  expectFailure(decideWaitReady(kSvc, a.feed, a.now, outstandingOf(5, a.now), std::nullopt, clockAt(0, 2.0)),
                station::FAULT, "FAULT",
                kAbortedWhy);
}

// A request already COMPLETED in this job (by a done-wait) no longer gates readiness: the general rule.
TEST(P0bWaitReady, ACompletedRequestFallsBackToTheGeneralRule)
{
  Rig r;
  r.live(State::READY);
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, outstandingOf(5, r.now, true), std::nullopt, clockAt(0, 2.0)).verdict,
            Verdict::SUCCESS);
}

// The operator's log line "status arriving" (Feed::arrival, computed before the image is applied): the first image, a
// NEW session, an image after more than the max age without one -- never for an ordinary period's image, never for a
// default image. A launch tier waits for it after a source restart (the ROBOT's own view of the new source).
TEST(P0bFeed, ArrivalIsToldForTheFirstImageANewSessionAndAfterAGap)
{
  Rig r;
  r.img.source_session = kSession;
  r.img.state = static_cast<uint8_t>(State::READY);
  EXPECT_EQ(r.feed.arrival(Image{}, r.now), "") << "a default image says nothing";
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "status arriving (source session a1a1a1a1)");
  r.send();
  r.advance(100);
  ++r.img.heartbeat;
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "") << "an ordinary period";
  r.send();
  r.advance(1000);
  ++r.img.heartbeat;
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "") << "exactly the max age: not yet a gap";
  r.send();
  r.advance(21300);
  ++r.img.heartbeat;
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "status arriving again after 21.3 s without one (source session a1a1a1a1)");
  r.send();
  r.img.source_session = kSession + 1;
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "status arriving from a NEW source session a1a1a1a2 (was a1a1a1a1)");
}

// Delta review R3: two sources alternating their sessions never flood the log -- each session is told at most once
// per max age, and with two status publishers counted the line says ANOTHER source, never a restart.
TEST(P0bFeed, ArrivalTellsASessionOncePerMaxAgeAndTwoSourcesAsAnother)
{
  Rig r;
  r.img.source_session = kSession;
  r.img.state = static_cast<uint8_t>(State::READY);
  EXPECT_EQ(r.feed.arrival(r.img, r.now), "status arriving (source session a1a1a1a1)");
  r.send();
  r.pubs(2, 1);
  Image other = r.img;
  other.source_session = kSession + 1;
  int lines = 0;
  for (int k = 0; k < 18; ++k)   // 0.9 s of two sources alternating, an image every 50 ms
  {
    r.advance(50);
    Image& img = (k % 2 == 0) ? other : r.img;
    ++img.heartbeat;
    const std::string line = r.feed.arrival(img, r.now);
    if (!line.empty())
    {
      ++lines;
      EXPECT_EQ(line, "status arriving from ANOTHER source session a1a1a1a2 (two sources publish its status; the "
                      "previous image was of a1a1a1a1)");
      EXPECT_EQ(line.find("NEW"), std::string::npos);
    }
    r.feed.image(img, r.now);
  }
  EXPECT_EQ(lines, 1) << "the second source told once, the first never again within the max age";
  r.advance(200);   // past the max age since each was told: each is told once more
  ++other.heartbeat;
  EXPECT_FALSE(r.feed.arrival(other, r.now).empty());
  r.feed.image(other, r.now);
  r.advance(50);
  ++r.img.heartbeat;
  EXPECT_FALSE(r.feed.arrival(r.img, r.now).empty());
}

// Delta review coverage R3: kept() after a successful publish drops the snapshot -- a later unwrote() changes nothing.
TEST(P0bRequest, Row3_KeptDropsThePublishSnapshot)
{
  Rig r;
  r.live(State::READY);
  r.feed.wrote(mine(5));
  r.img.cycle = Identity{kMe, 5};
  r.send();
  r.feed.wrote(mine(6));
  r.feed.kept();
  r.feed.unwrote();   // after kept(): nothing to restore
  EXPECT_FALSE(r.feed.consumed(mine(5))) << "6 is still the watched request (5 was not restored)";
  r.img.cycle = Identity{kMe, 6};
  r.send();
  EXPECT_TRUE(r.feed.consumed(mine(6)));
}

// PB-P24 (U), D19 -- the owner's amendment of §3.4 (2026-11-19; protocol review MAJOR-2): job A wrote its request
// and was cancelled at once; job B's FIRST step is a ready-wait (no request of B pending). While this PROCESS's last
// request to the current source session is not yet taken, the READY of before it is never "ready": B waits (it says
// why); FAULT is 306; the robot's clock problem stops it; the timeout is 305 (it says why); not LIVE is 310 first.
// Once taken -- its cycle (BUSY keeps waiting, DONE is ready) or its rejection -- the general rule applies; the latch
// keeps it taken when another identity displaces it; a request to an OLDER session never blocks (PB-P17(a): the
// restarted source that reports READY is ready). One condition with request rule 3: the ready-wait waits exactly when
// a request would be 313.
TEST(P0bWaitReady, PB_P24_ThisProcesssRequestNotYetTakenIsNeverReady)
{
  Rig r;
  r.live(State::READY);
  const Written a = mine(9, kSession, r.now);   // job A's request; A was cancelled at once
  r.feed.wrote(a);
  Decision d = decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(0.0, 2.0));
  EXPECT_EQ(d.verdict, Verdict::RUNNING) << d.text;
  EXPECT_EQ(d.phase, Phase::WAITING);
  EXPECT_EQ(d.text, "WAITING FOR MACHINE turner (it has not yet taken the previous request): 0 s of 2.0 s");
  EXPECT_EQ(r.request(a).code, station::BUSY) << "the same condition as request rule 3";
  expectFailure(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(2.0, 2.0)), station::TIMEOUT, "TIMEOUT",
                "still not ready after 2.0 s (the machine has not yet taken the previous request; it is READY)");
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(2.0, 2.0)).text,
            "MACHINE turner TIMEOUT -- STATION_TIMEOUT (305): still not ready after 2.0 s (the machine has not yet "
            "taken the previous request; it is READY)");
  WaitClock stopped = clockAt(0.5, 2.0);
  stopped.problem = "the robot's clock stopped (it has not advanced for more than 1.0 s)";
  expectFailure(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, stopped), 0, "", "the robot's clock stopped");
  // taken as its CYCLE: BUSY keeps waiting (the general rule), its DONE is ready; B completes nothing (not B's request)
  r.img.cycle = Identity{kMe, 9};
  r.state(State::BUSY);
  d = decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(0.4, 2.0));
  EXPECT_EQ(d.verdict, Verdict::RUNNING);
  EXPECT_EQ(d.text, "WAITING FOR MACHINE turner (it is BUSY): 0 s of 2.0 s");
  r.img.cycle_done = true;
  r.state(State::DONE);
  d = decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(0.9, 2.0));
  EXPECT_EQ(d.verdict, Verdict::SUCCESS) << d.text;
  EXPECT_FALSE(d.completes) << "A's request is not B's: the ready-wait completes nothing";
  EXPECT_EQ(d.text, "MACHINE turner DONE: ready after 0.9 s");
  // latched: another requester's identity displaces it -- still taken, the general rule
  r.img.cycle = Identity{0x5C1D0011u, 4};
  r.state(State::DONE);
  EXPECT_EQ(decideWaitReady(kSvc, r.feed, r.now, std::nullopt, a, clockAt(1.0, 2.0)).verdict, Verdict::SUCCESS);

  // not yet taken and FAULT -> 306; not LIVE -> 310 first
  Rig f;
  f.live(State::READY);
  const Written fa = mine(3);
  f.feed.wrote(fa);
  f.state(State::FAULT);
  expectFailure(decideWaitReady(kSvc, f.feed, f.now, std::nullopt, fa, clockAt(0.1, 2.0)), station::FAULT, "FAULT",
                "the machine reports FAULT");
  f.advance(1200);
  expectFailure(decideWaitReady(kSvc, f.feed, f.now, std::nullopt, fa, clockAt(1.3, 2.0)), station::UNAVAILABLE,
                "UNAVAILABLE", "no status for more than 1.0 s");

  // taken as a REJECTION: the general rule (READY is ready)
  Rig j;
  j.live(State::READY);
  const Written ja = mine(4);
  j.feed.wrote(ja);
  j.img.reject = Identity{kMe, 4};
  j.img.reject_code = static_cast<uint8_t>(Reject::BUSY);
  j.send();
  EXPECT_EQ(decideWaitReady(kSvc, j.feed, j.now, std::nullopt, ja, clockAt(0, 2.0)).verdict, Verdict::SUCCESS);

  // a request to an OLDER session (the source restarted since): never blocks
  Rig o;
  o.live(State::READY);
  const Written old = mine(4, kSession + 7);
  o.feed.wrote(old);
  EXPECT_EQ(decideWaitReady(kSvc, o.feed, o.now, std::nullopt, old, clockAt(0, 2.0)).verdict, Verdict::SUCCESS);

  // this job's OWN request pending: the first branch decides (B1), unchanged
  Rig p;
  p.live(State::READY);
  const Written pa = mine(5);
  p.feed.wrote(pa);
  d = decideWaitReady(kSvc, p.feed, p.now, outstandingOf(5, p.now), pa, clockAt(0.1, 2.0));
  EXPECT_EQ(d.verdict, Verdict::RUNNING);
  EXPECT_EQ(d.text, "WAITING FOR MACHINE turner: 0 s of 2.0 s");
}

// =====================================================================================================================
// The wait's clock (D10)
// =====================================================================================================================
// CL2 (U): the timeout runs on the RUNNER's clock -- at 0.5x it fires after timeout_ms of runner time (2x steady).
TEST(P0bWaitTimer, CL2_TheTimeoutRunsOnTheRunnersClock)
{
  WaitTimer t(milliseconds(1000), milliseconds(1000));
  SteadyTime steady = std::chrono::steady_clock::now();
  int64_t sim = 5'000'000'000;
  for (int k = 0; k <= 200; ++k)
  {
    t.sample(sim, steady);
    const WaitClock c = t.clock();
    if (k < 200)
    {
      EXPECT_FALSE(c.timed_out) << k;
    }
    EXPECT_TRUE(c.problem.empty()) << c.problem;
    steady += milliseconds(10);
    sim += 5'000'000;   // 0.5x
  }
  EXPECT_TRUE(t.clock().timed_out) << "1000 ms of runner time = 2000 ms steady";
  EXPECT_DOUBLE_EQ(t.clock().elapsed_s, 1.0);
  ASSERT_TRUE(t.clock().now_ns.has_value()) << "the runner's clock at the last sample (the DONE text's age)";
  EXPECT_EQ(*t.clock().now_ns, int64_t{6'000'000'000});
}

TEST(P0bWaitTimer, D10_AClockThatNeverStartsIsAProblemAfterTheMaxAge)
{
  WaitTimer t(milliseconds(8000), milliseconds(1000));
  SteadyTime steady = std::chrono::steady_clock::now();
  t.sample(std::nullopt, steady);
  EXPECT_TRUE(t.clock().problem.empty());
  steady += milliseconds(1000);
  t.sample(std::nullopt, steady);
  EXPECT_TRUE(t.clock().problem.empty());
  steady += milliseconds(1);
  t.sample(std::nullopt, steady);
  EXPECT_NE(t.clock().problem.find("the robot's clock is not running"), std::string::npos) << t.clock().problem;
  EXPECT_FALSE(t.clock().timed_out);
  EXPECT_FALSE(t.clock().now_ns.has_value()) << "no runner time while the clock is not running";
  // it starts: the timer starts from that moment, the problem is gone
  t.sample(int64_t{42'000'000'000}, steady);
  EXPECT_TRUE(t.clock().problem.empty());
  EXPECT_DOUBLE_EQ(t.clock().elapsed_s, 0.0);
  ASSERT_TRUE(t.clock().now_ns.has_value());
  EXPECT_EQ(*t.clock().now_ns, int64_t{42'000'000'000});
  // it stops reporting after having started: no runner time at this tick (the DONE text falls back to the wall time)
  t.sample(std::nullopt, steady);
  EXPECT_FALSE(t.clock().now_ns.has_value());
}

TEST(P0bWaitTimer, D10_ABackwardJumpRestartsTheTimer)
{
  WaitTimer t(milliseconds(1000), milliseconds(1000));
  SteadyTime steady = std::chrono::steady_clock::now();
  t.sample(int64_t{10'000'000'000}, steady);
  steady += milliseconds(800);
  t.sample(int64_t{10'800'000'000}, steady);
  EXPECT_NEAR(t.clock().elapsed_s, 0.8, 1e-9);
  steady += milliseconds(10);
  t.sample(int64_t{1'000'000'000}, steady);   // a simulation reset
  EXPECT_TRUE(t.restartedLastSample());
  EXPECT_DOUBLE_EQ(t.clock().elapsed_s, 0.0);
  steady += milliseconds(900);
  t.sample(int64_t{1'900'000'000}, steady);
  EXPECT_FALSE(t.restartedLastSample());
  EXPECT_FALSE(t.clock().timed_out);
  steady += milliseconds(100);
  t.sample(int64_t{2'000'000'000}, steady);
  EXPECT_TRUE(t.clock().timed_out);
}

TEST(P0bWaitTimer, D10_AFrozenClockIsAProblemAfterTheMaxAge)
{
  WaitTimer t(milliseconds(8000), milliseconds(1000));
  SteadyTime steady = std::chrono::steady_clock::now();
  t.sample(int64_t{7'000'000'000}, steady);
  for (int k = 0; k < 100; ++k)
  {
    steady += milliseconds(10);
    t.sample(int64_t{7'000'000'000}, steady);
  }
  EXPECT_TRUE(t.clock().problem.empty()) << "exactly the max age";
  steady += milliseconds(1);
  t.sample(int64_t{7'000'000'000}, steady);
  EXPECT_NE(t.clock().problem.find("the robot's clock stopped"), std::string::npos);
  steady += milliseconds(10);
  t.sample(int64_t{7'001'000'000}, steady);   // it moves again
  EXPECT_TRUE(t.clock().problem.empty());
}

// =====================================================================================================================
// The seq (PB-P6) and the texts
// =====================================================================================================================
// PB-P6 (U): from an injected counter: 2^32-1, then 1 (0 skipped), in order across the wrap; a new source session
// starts again from the injected value.
TEST(P0bSeq, PB_P6_TheSeqWrapsSkippingZero)
{
  SeqCounter seq(UINT32_MAX);
  EXPECT_EQ(seq.next(kSession), UINT32_MAX);
  EXPECT_EQ(seq.next(kSession), 1u);
  EXPECT_EQ(seq.next(kSession), 2u);
  EXPECT_EQ(seq.next(kSession + 1), UINT32_MAX) << "a new source session";
  SeqCounter plain;
  EXPECT_EQ(plain.next(kSession), 1u);
  EXPECT_EQ(plain.next(kSession), 2u);
  SeqCounter zero(0);
  EXPECT_EQ(zero.next(kSession), 1u) << "0 is never a seq";
}

TEST(P0bTexts, TheUserFacingWordsAreThePinnedOnes)
{
  EXPECT_EQ(statusTopic("turner"), "turner/status");
  EXPECT_EQ(requestTopic("turner"), "turner/request");
  EXPECT_EQ(requestedText("turner"), "MACHINE turner REQUESTED");
  EXPECT_EQ(waitingText("turner", "", 0.9, 8.0), "WAITING FOR MACHINE turner: 0 s of 8.0 s");
  EXPECT_EQ(busyText("turner", 2.1, 8.0), "MACHINE turner BUSY: 2 s of 8.0 s");
  EXPECT_EQ(doneText("turner", 5.43, 5.41), "MACHINE turner DONE in 5.4 s");
  EXPECT_EQ(doneText("turner", 5.43, 9.6), "MACHINE turner DONE in 5.4 s (9.6 s wall)");
  EXPECT_EQ(doneText("turner", std::nullopt, 9.6), "MACHINE turner DONE in 9.6 s (wall)");
  EXPECT_EQ(failureText(305, "turner", "still BUSY after 8.0 s"),
            "MACHINE turner TIMEOUT -- STATION_TIMEOUT (305): still BUSY after 8.0 s");
  EXPECT_EQ(failureText(306, "turner", "x"), "MACHINE turner FAULT -- STATION_FAULT (306): x");
  EXPECT_EQ(failureText(313, "turner", "x"), "MACHINE turner FAULT -- STATION_BUSY (313): x");
  EXPECT_EQ(failureText(310, "turner", "x"), "MACHINE turner UNAVAILABLE -- STATION_UNAVAILABLE (310): x");
  for (const std::string& t : {requestedText("indexer"), waitingText("indexer", "it is BUSY", 1, 2),
                               busyText("indexer", 1, 2), doneText("indexer", 1, 2.5),
                               doneText("indexer", std::nullopt, 1), readyText("indexer", 0)})
    expectNoNeverWord(t);
}

// PB-P8 (U): the start diagnostic names an unserved service; the first request's 310 says "no status for service 'x'".
TEST(P0bTexts, PB_P8_TheStartDiagnosticNamesAnUnservedService)
{
  Rig r;
  r.subscribe();
  r.advance(3000);
  const StartDiagnostic none = startDiagnostic("turnr", r.feed, r.now);
  EXPECT_TRUE(none.warning);
  EXPECT_EQ(none.text, "no status for machine service 'turnr' -- is the machine simulation (or the PLC) serving it?");
  const Decision d = decideRequest("turnr", r.feed, r.now, r.now, std::nullopt);
  EXPECT_EQ(d.code, station::UNAVAILABLE);
  EXPECT_NE(d.text.find("no status for service 'turnr'"), std::string::npos) << d.text;
  Rig s;
  s.live(State::READY);
  const StartDiagnostic seen = startDiagnostic("turner", s.feed, s.now);
  EXPECT_FALSE(seen.warning);
  EXPECT_EQ(seen.text, "machine service 'turner': status seen (READY, live)");
  s.advance(2000);
  EXPECT_TRUE(startDiagnostic("turner", s.feed, s.now).warning);
}

TEST(P0bTexts, TheVocabularyWords)
{
  EXPECT_STREQ(stateWord(0), "NO STATE");
  EXPECT_STREQ(stateWord(1), "READY");
  EXPECT_STREQ(stateWord(5), "HOMING");
  EXPECT_STREQ(stateWord(6), "?");
  EXPECT_STREQ(faultWord(0), "");
  EXPECT_STREQ(faultWord(1), "ACTUATOR_TIMEOUT");
  EXPECT_STREQ(faultWord(2), "SENSOR_FAULT");
  EXPECT_STREQ(faultWord(3), "STATION_FAULT");
  EXPECT_STREQ(faultWord(4), "?");
}
