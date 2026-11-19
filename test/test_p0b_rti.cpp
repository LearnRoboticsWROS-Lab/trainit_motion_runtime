// test_p0b_rti (P0.b, P0B_IMPLEMENTATION_PLAN.md D3, D5): the Robot Task Interface's side of the machine handshake,
// PURE (the state machine with a fake steady clock, as test_p0a_rti_state_machine):
//   - onActivity (D5): a machine step's plain-words state becomes the status `detail` ONLY while ITS job is RUNNING
//     (another job's counter, STOPPING, IDLE, FAULTED, UNAVAILABLE: ignored); a START clears it; the job's end replaces
//     it with the result's detail;
//   - the STATION code (D3): a job whose tree did not succeed ends FAILED with the machine step's code (305 / 306 / 310
//     / 313), checked BEFORE TERMINAL_FAULT (300), the *_EXHAUSTED outcomes and the 301 fallback, but AFTER an
//     end-effector fault (308: the tool's state unconfirmed outranks it, C++ review M4); a stop CAUSE keeps precedence
//     (a cancel MASKS it: ADR-0032 Decision 8); a tree that succeeded keeps its own result and its detail never
//     carries a recovered machine failure's stale text (C++ review M3); a machine failure without a code (D4, D10) is
//     TASK_FAILED 301 and says why;
//   - 313 STATION_BUSY appended to the vocabulary (never renumbered).
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "trainit_motion_runtime/rti_contract.hpp"
#include "trainit_motion_runtime/rti_state_machine.hpp"

using namespace trainit::rti;
using std::chrono::milliseconds;

namespace
{
ExportTable table()
{
  ExportTable t;
  ExportEntry e;
  e.number = 5;
  e.name = "turn_part";
  e.timeout_s = 30.0;
  e.timeout_ms = 30000;
  t.entries = {e};
  t.id = exportTableId(t.entries);
  return t;
}

struct Rig
{
  StateMachine sm{Config{}, table()};
  SteadyTime now{std::chrono::steady_clock::now()};
  Command img{};

  Rig() { img.requester_session = 9; }
  void advance(int ms) { now += milliseconds(ms); }
  Effects feed(std::optional<int> publishers = std::nullopt) { return sm.onCommand(img, now, publishers); }
  Effects beat()
  {
    ++img.heartbeat;
    return feed();
  }
  Effects check(bool runtime = true, int pubs = 1) { return sm.onCheck(runtime, runtime ? "" : "MoveIt lost", pubs, now); }
  Effects send(Cmd c, uint32_t job = 0, uint16_t task = 0)
  {
    img.cmd_seq = nextSeq(img.cmd_seq);
    img.cmd = static_cast<uint8_t>(c);
    img.job_id = job;
    img.task = task;
    return feed();
  }
  void operational()
  {
    feed(1);
    beat();
    beat();
    check();
    send(Cmd::RESET);
    ASSERT_EQ(sm.status().state, State::IDLE);
  }
  uint64_t start(uint32_t job)
  {
    const Effects fx = send(Cmd::START, job, 5);
    EXPECT_TRUE(fx.start.has_value());
    return fx.start ? fx.start->counter : 0;
  }
  std::optional<Decided> end(uint64_t counter, JobEnd::Tree tree, uint16_t station = 0, std::string station_text = "",
                             std::string outcome = "", std::optional<bool> holding = false, bool eef = false,
                             std::string reason = "")
  {
    JobEnd e;
    e.tree = tree;
    e.outcome = std::move(outcome);
    e.holding = holding;
    e.eef_fault = eef;
    e.reason = std::move(reason);
    e.station_code = station;
    e.station_text = std::move(station_text);
    return sm.onJobEnded(counter, e, now);
  }
  const Status& st() const { return sm.status(); }
};

const char* kBusyText = "MACHINE turner BUSY: 2 s of 8.0 s";
const char* kTimeoutText = "MACHINE turner TIMEOUT -- STATION_TIMEOUT (305): still BUSY after 8.0 s";
}  // namespace

// ---------------------------------------------------------------------------------------------------- the vocabulary
TEST(P0bRti, StationBusy313IsAppended)
{
  EXPECT_EQ(code::STATION_BUSY, 313);
  EXPECT_STREQ(codeName(code::STATION_BUSY), "STATION_BUSY");
  EXPECT_STREQ(codeName(code::REQUESTER_CONFLICT), "REQUESTER_CONFLICT");   // 312 unchanged
  EXPECT_STREQ(codeName(314), "?");
  // every result code of 100..313 has one distinct name (never reused)
  std::set<std::string> names;
  for (const uint16_t c : {code::OK, code::BIN_EMPTY, code::NOTHING_PICKABLE, code::PICKABLE_EXHAUSTED,
                           code::RECOVERY_EXHAUSTED, code::CANCELLED, code::TERMINAL_FAULT, code::TASK_FAILED,
                           code::TASK_TIMEOUT, code::REQUESTER_LOST, code::RUNTIME_LOST, code::STATION_TIMEOUT,
                           code::STATION_FAULT, code::CANCELLED_WHILE_HOLDING, code::EEF_FAULT,
                           code::CANCELLED_HOLD_UNKNOWN, code::STATION_UNAVAILABLE, code::SERVER_STARTED,
                           code::REQUESTER_CONFLICT, code::STATION_BUSY})
    EXPECT_TRUE(names.insert(codeName(c)).second) << c;
  EXPECT_EQ(names.size(), 20u);
}

// ---------------------------------------------------------------------------------------------------- onActivity (D5)
TEST(P0bRti, D5_TheRunningJobsActivityIsTheDetail)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  EXPECT_TRUE(r.st().detail.empty()) << "a START clears the detail";
  r.sm.onActivity(c, "WAITING FOR MACHINE turner: 0 s of 8.0 s");
  EXPECT_EQ(r.st().detail, "WAITING FOR MACHINE turner: 0 s of 8.0 s");
  r.sm.onActivity(c, kBusyText);
  EXPECT_EQ(r.st().detail, kBusyText);
  EXPECT_EQ(r.st().state, State::RUNNING);
  EXPECT_EQ(r.st().result, Result::NONE);
  EXPECT_EQ(r.st().result_code, 0);
  EXPECT_EQ(r.st().fault_code, 0);
  // the periodic check while RUNNING never rewrites it
  for (int k = 0; k < 5; ++k)
  {
    r.advance(100);
    r.beat();
    r.check();
  }
  EXPECT_EQ(r.st().detail, kBusyText);
  // another job's counter: ignored
  r.sm.onActivity(c + 1, "MACHINE indexer BUSY: 0 s of 8.0 s");
  EXPECT_EQ(r.st().detail, kBusyText);
  // the end replaces it with the result's detail (a success: empty)
  r.end(c, JobEnd::Tree::SUCCESS);
  EXPECT_EQ(r.st().state, State::IDLE);
  EXPECT_TRUE(r.st().detail.empty()) << r.st().detail;
  // IDLE: a late activity of the finished job is ignored
  r.sm.onActivity(c, "MACHINE turner DONE in 5.4 s");
  EXPECT_TRUE(r.st().detail.empty());
  // the next job starts clean
  const auto c2 = r.start(2);
  EXPECT_TRUE(r.st().detail.empty());
  r.sm.onActivity(c2, "MACHINE turner REQUESTED");
  EXPECT_EQ(r.st().detail, "MACHINE turner REQUESTED");
}

TEST(P0bRti, D5_StoppingFaultedAndUnavailableIgnoreTheActivity)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  r.sm.onActivity(c, kBusyText);
  r.send(Cmd::CANCEL, 1);
  ASSERT_EQ(r.st().state, State::STOPPING);
  EXPECT_EQ(r.st().detail, "stopping: CANCEL");
  r.sm.onActivity(c, "MACHINE turner BUSY: 3 s of 8.0 s");
  EXPECT_EQ(r.st().detail, "stopping: CANCEL") << "STOPPING keeps its cause";
  r.end(c, JobEnd::Tree::HALTED, 0, "", "", false);
  EXPECT_EQ(r.st().result, Result::CANCELLED);
  const std::string after = r.st().detail;
  r.sm.onActivity(c, kBusyText);
  EXPECT_EQ(r.st().detail, after);

  Rig f;   // FAULTED
  f.operational();
  const auto fc = f.start(1);
  f.end(fc, JobEnd::Tree::FAILURE, code::STATION_TIMEOUT, kTimeoutText);
  ASSERT_EQ(f.st().state, State::FAULTED);
  const std::string faulted = f.st().detail;
  f.sm.onActivity(fc, kBusyText);
  EXPECT_EQ(f.st().detail, faulted);

  Rig u;   // UNAVAILABLE (adopted, the start-up acknowledged, the runtime not ready)
  u.feed(1);
  u.beat();
  u.beat();
  u.check(false);
  u.send(Cmd::RESET);
  ASSERT_EQ(u.st().state, State::UNAVAILABLE);
  const std::string unavailable = u.st().detail;
  u.sm.onActivity(1, kBusyText);
  EXPECT_EQ(u.st().detail, unavailable);
}

// ---------------------------------------------------------------------------------------------------- the station code
TEST(P0bRti, D3_AFailedTreeEndsWithItsStationCode)
{
  for (const uint16_t station : {code::STATION_TIMEOUT, code::STATION_FAULT, code::STATION_UNAVAILABLE,
                                 code::STATION_BUSY})
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    const auto d = r.end(c, JobEnd::Tree::FAILURE, station, "MACHINE turner ...");
    ASSERT_TRUE(d);
    EXPECT_EQ(d->result, Result::FAILED) << station;
    EXPECT_EQ(d->result_code, station);
    EXPECT_EQ(r.st().state, State::FAULTED) << "FAILED -> FAULTED (ADR-0032 Decision 7): a RESET acknowledges it";
    EXPECT_EQ(r.st().fault_code, station);
    EXPECT_NE(r.st().detail.find("MACHINE turner ..."), std::string::npos) << r.st().detail;
    r.send(Cmd::RESET);
    EXPECT_EQ(r.st().state, State::IDLE) << "a RESET returns it to the operational state";
  }
}

// D3: before TERMINAL_FAULT (the gate + outcomes report would say 300) and the *_EXHAUSTED outcomes -- but AFTER an
// end-effector fault latched in the same job (C++ review M4): the tool's physical state unconfirmed is the more severe
// uncertainty (ADR-0032 8a.4), so the result is EEF_FAULT 308; the machine's words stay in the detail. (Before M4 the
// station code won over the end-effector fault: the eef rows below expected 306 and lost the tool's state.)
TEST(P0bRti, D3_TheStationCodeIsCheckedBeforeTheOutcomesButAfterAnEefFault)
{
  struct Case
  {
    const char* outcome;
    bool eef;
  };
  for (const Case k : {Case{"TERMINAL_FAULT", false}, Case{"PICKABLE_EXHAUSTED", false}, Case{"RECOVERY_EXHAUSTED", false},
                       Case{"", false}, Case{"", true}, Case{"TERMINAL_FAULT", true}, Case{"PICKABLE_EXHAUSTED", true},
                       Case{"RECOVERY_EXHAUSTED", true}})
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    const auto d = r.end(c, JobEnd::Tree::FAILURE, code::STATION_FAULT, "MACHINE turner FAULT -- ...", k.outcome, false,
                         k.eef);
    ASSERT_TRUE(d);
    const uint16_t want = k.eef ? code::EEF_FAULT : code::STATION_FAULT;
    EXPECT_EQ(d->result, Result::FAILED) << k.outcome << " eef " << k.eef;
    EXPECT_EQ(d->result_code, want) << k.outcome << " eef " << k.eef;
    EXPECT_EQ(r.st().state, State::FAULTED) << k.outcome << " eef " << k.eef;
    EXPECT_EQ(r.st().fault_code, want) << k.outcome << " eef " << k.eef;
    EXPECT_NE(r.st().detail.find("MACHINE turner FAULT -- ..."), std::string::npos)
      << "the machine's words stay in the detail: " << r.st().detail;
    if (k.eef)
    {
      EXPECT_FALSE(r.sm.heldState().has_value()) << "an end-effector fault leaves the held state unknown";
    }
  }
  // without a station code the P0.a mapping is unchanged
  Rig r;
  r.operational();
  auto c = r.start(1);
  EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, 0, "", "TERMINAL_FAULT")->result_code, code::TERMINAL_FAULT);
  r.send(Cmd::RESET);
  c = r.start(2);
  EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, 0, "", "")->result_code, code::TASK_FAILED);
}

// A tree that SUCCEEDED keeps its own result (a hand-written tree recovered a machine failure: a retry, a fallback) and
// its detail never carries that recovered failure's stale text (C++ review M3; before it, this test pinned "the
// absorbed failure is still told", which reported a failure on a job that succeeded); an EEF fault is still 308; a
// cancel that came too late says so, without the machine's text either. A tree that did NOT succeed keeps D3: the
// job's FIRST machine failure stands (its code and its words), whatever failed after it.
TEST(P0bRti, D3_ASucceededTreeKeepsItsResultWithoutAStaleMachineText)
{
  Rig r;
  r.operational();
  auto c = r.start(1);
  auto d = r.end(c, JobEnd::Tree::SUCCESS, code::STATION_TIMEOUT, kTimeoutText);
  EXPECT_EQ(d->result, Result::SUCCEEDED);
  EXPECT_EQ(d->result_code, code::OK);
  EXPECT_EQ(r.st().state, State::IDLE);
  EXPECT_TRUE(r.st().detail.empty()) << "no stale machine text on a success: " << r.st().detail;
  c = r.start(2);
  d = r.end(c, JobEnd::Tree::SUCCESS, code::STATION_TIMEOUT, kTimeoutText, "", false, true);
  EXPECT_EQ(d->result_code, code::EEF_FAULT);
  EXPECT_EQ(r.st().detail.find(kTimeoutText), std::string::npos) << r.st().detail;
  r.send(Cmd::RESET);
  c = r.start(3);   // a cancel that came too late: the tree succeeded
  r.send(Cmd::CANCEL, 3);
  d = r.end(c, JobEnd::Tree::SUCCESS, code::STATION_TIMEOUT, kTimeoutText);
  EXPECT_EQ(d->result_code, code::OK);
  EXPECT_EQ(r.st().detail, "stopped: CANCEL (too late: the task had completed)");
  c = r.start(4);   // D3 kept: a tree that FAILED after a recovered machine failure ends with that first failure
  d = r.end(c, JobEnd::Tree::FAILURE, code::STATION_TIMEOUT, kTimeoutText, "", false, false, "MOTION_FAULT: x");
  EXPECT_EQ(d->result_code, code::STATION_TIMEOUT);
  EXPECT_EQ(r.st().detail, std::string(kTimeoutText) + "; MOTION_FAULT: x");
}

// D4 / D10: a machine step's failure WITHOUT a code (a wait with no request in this job, the robot's clock not running)
// is TASK_FAILED 301 -- a program error, never a machine condition -- and the detail says why.
TEST(P0bRti, D4_AFailureWithoutAStationCodeIs301AndSaysWhy)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  const std::string why = "machine service 'turner': a wait for done with no request on it in this job";
  const auto d = r.end(c, JobEnd::Tree::FAILURE, 0, why);
  EXPECT_EQ(d->result_code, code::TASK_FAILED);
  EXPECT_NE(r.st().detail.find(why), std::string::npos) << r.st().detail;
}

// The causes keep precedence (first cause wins, Decision 8a.3): a cancel MASKS the machine failure (Decision 8) and says
// so; a timeout / loss / conflict is its own code.
TEST(P0bRti, D3_ACauseKeepsPrecedenceOverTheStationCode)
{
  {
    Rig r;   // CANCEL, the tree failed (a request refused at the same moment) with nothing held
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    const auto d = r.end(c, JobEnd::Tree::FAILURE, code::STATION_BUSY, "MACHINE turner FAULT -- STATION_BUSY (313): x");
    EXPECT_EQ(d->result, Result::CANCELLED);
    EXPECT_EQ(d->result_code, code::CANCELLED);
    EXPECT_NE(r.st().detail.find("the machine failure STATION_BUSY is masked by the cancel"), std::string::npos)
      << r.st().detail;
  }
  {
    Rig r;   // CANCEL while holding a part
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    EXPECT_EQ(r.end(c, JobEnd::Tree::HALTED, code::STATION_FAULT, "x", "", true)->result_code,
              code::CANCELLED_WHILE_HOLDING);
  }
  {
    Rig r;   // CANCEL, the held state unknown
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, code::STATION_FAULT, "x", "", std::nullopt)->result_code,
              code::CANCELLED_HOLD_UNKNOWN);
  }
  {
    Rig r;   // CANCEL too late: the tree succeeded -> its own result
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    EXPECT_EQ(r.end(c, JobEnd::Tree::SUCCESS, code::STATION_FAULT, "x")->result_code, code::OK);
  }
  {
    Rig r;   // TASK_TIMEOUT (302) fires before the station's timeout (X9's warning)
    r.operational();
    const auto c = r.start(1);
    for (int k = 0; k < 301; ++k)
    {
      r.advance(100);
      r.beat();
      r.check();
    }
    ASSERT_EQ(r.st().state, State::STOPPING);
    EXPECT_EQ(r.end(c, JobEnd::Tree::HALTED, code::STATION_TIMEOUT, kTimeoutText)->result_code, code::TASK_TIMEOUT);
  }
  {
    Rig r;   // RUNTIME_LOST (304)
    r.operational();
    const auto c = r.start(1);
    r.check(false);
    EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, code::STATION_FAULT, "x")->result_code, code::RUNTIME_LOST);
  }
  {
    Rig r;   // REQUESTER_LOST (303)
    r.operational();
    const auto c = r.start(1);
    r.advance(1100);
    r.check();
    EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, code::STATION_FAULT, "x")->result_code, code::REQUESTER_LOST);
  }
  {
    Rig r;   // REQUESTER_CONFLICT (312)
    r.operational();
    const auto c = r.start(1);
    r.check(true, 2);
    EXPECT_EQ(r.end(c, JobEnd::Tree::FAILURE, code::STATION_FAULT, "x")->result_code, code::REQUESTER_CONFLICT);
  }
}

// The detail tells the machine step's words once, whether or not the fault gate carried the same text.
TEST(P0bRti, D3_TheResultDetailCarriesTheMachineTextOnce)
{
  Rig r;
  r.operational();
  auto c = r.start(1);
  r.end(c, JobEnd::Tree::FAILURE, code::STATION_TIMEOUT, kTimeoutText, "TERMINAL_FAULT", false, false,
        kTimeoutText);   // the gate's reason IS the node's fault text
  EXPECT_EQ(r.st().detail, kTimeoutText);
  r.send(Cmd::RESET);
  c = r.start(2);
  r.end(c, JobEnd::Tree::FAILURE, code::STATION_TIMEOUT, kTimeoutText);   // no gate (a tree without faults: stop)
  EXPECT_EQ(r.st().detail, kTimeoutText);
  r.send(Cmd::RESET);
  c = r.start(3);
  r.end(c, JobEnd::Tree::FAILURE, code::STATION_TIMEOUT, kTimeoutText, "", false, false, "MOTION_FAULT: x");
  EXPECT_EQ(r.st().detail, std::string(kTimeoutText) + "; MOTION_FAULT: x");
}
