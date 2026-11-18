// test_p0a_rti_state_machine (P0.a, ADR-0032): the PURE Robot Task Interface state machine, driven with a fake
// steady clock -- deterministic. Every rule of Decisions 5, 6, 7, 8, 8a, 9, 10 and every race ORDERING of 8a
// (the binding serializes the inputs; this pins that each order gives one defined result). The frozen matrix's S
// cases run again end to end through the ROS binding in test_p0a_rti_server.
#include <fstream>
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
  ExportEntry home;
  home.number = 7;
  home.name = "home";
  home.timeout_s = 5.0;
  home.timeout_ms = 5000;
  ExportEntry place;
  place.number = 12;
  place.name = "place_part";
  place.timeout_s = 30.0;
  place.timeout_ms = 30000;
  t.entries = {home, place};
  t.id = exportTableId(t.entries);
  return t;
}

struct Rig
{
  StateMachine sm{Config{}, table()};
  SteadyTime now{std::chrono::steady_clock::now()};
  Command img{};
  Effects last;

  Rig() { img.requester_session = 7; }

  void advance(int ms) { now += milliseconds(ms); }
  Effects feed(std::optional<int> publishers = std::nullopt) { return last = sm.onCommand(img, now, publishers); }
  Effects beat()
  {
    ++img.heartbeat;
    return feed();
  }
  Effects check(bool runtime = true, int pubs = 1) { return last = sm.onCheck(runtime, runtime ? "" : "MoveIt lost", pubs, now); }
  void adopt()
  {
    feed(1);   // a new session: the graph counts its publisher (Decision 10)
    beat();
    beat();
  }
  Effects send(Cmd c, uint32_t job = 0, uint16_t task = 0)
  {
    img.cmd_seq = nextSeq(img.cmd_seq);
    img.cmd = static_cast<uint8_t>(c);
    img.job_id = job;
    img.task = task;
    return feed();
  }
  // the usual start: adopted, the runtime ready, the start-up acknowledged
  void operational()
  {
    adopt();
    check();
    send(Cmd::RESET);
    ASSERT_EQ(sm.status().state, State::IDLE);
  }
  uint64_t start(uint32_t job, uint16_t task = 12)
  {
    const Effects fx = send(Cmd::START, job, task);
    EXPECT_TRUE(fx.start.has_value()) << "START " << job << " not accepted: " << codeName(sm.status().ack_code);
    return fx.start ? fx.start->counter : 0;
  }
  std::optional<Decided> end(uint64_t counter, JobEnd::Tree tree, std::string outcome = "",
                             std::optional<bool> holding = false, bool eef = false)
  {
    JobEnd e;
    e.tree = tree;
    e.outcome = std::move(outcome);
    e.holding = holding;
    e.eef_fault = eef;
    return sm.onJobEnded(counter, e, now);
  }
  const Status& st() const { return sm.status(); }
};
}  // namespace

// ---------------------------------------------------------------------------------------------------- contract
TEST(RtiContract, Crc32IsZlibs)
{
  EXPECT_EQ(crc32("123456789"), 0xCBF43926u);   // the standard check value (zlib.crc32(b"123456789"))
  EXPECT_EQ(crc32(""), 0u);
}

TEST(RtiContract, TheExportTableTextIsSortedByNumberWithMilliseconds)
{
  EXPECT_EQ(exportTableText(table().entries), "7:home:5000\n12:place_part:30000\n");
  EXPECT_EQ(table().id, crc32("7:home:5000\n12:place_part:30000\n"));
}

// The SAME vectors are pinned in TSA's test_p0a_task_export.py (SHARED_VECTORS, computed there with zlib.crc32): the
// two implementations of the export-table rule agree beyond the served fixture (review: architecture, finding 8).
TEST(RtiContract, TheSharedVectorsAreTheSameOnBothSides)
{
  auto entry = [](uint16_t number, const char* name, int64_t ms) {
    ExportEntry e;
    e.number = number;
    e.name = name;
    e.timeout_ms = ms;
    e.timeout_s = static_cast<double>(ms) / 1000.0;
    return e;
  };
  // the served fixture (TSA FIXTURE_TABLE_TEXT / FIXTURE_TABLE_ID; the R test's kTableId)
  const std::vector<ExportEntry> served = {entry(12, "place_part", 30000), entry(7, "home", 5000),
                                           entry(4, "carry", 10000), entry(3, "pick_part", 10000)};
  EXPECT_EQ(exportTableText(served), "3:pick_part:10000\n4:carry:10000\n7:home:5000\n12:place_part:30000\n");
  EXPECT_EQ(exportTableId(served), 3767158916u);
  const std::vector<ExportEntry> one = {entry(1, "a", 1)};
  EXPECT_EQ(exportTableText(one), "1:a:1\n");
  EXPECT_EQ(exportTableId(one), 429640448u);
  const std::vector<ExportEntry> bounds = {entry(65535, "z", 86400000), entry(300, "m", 2500)};
  EXPECT_EQ(exportTableText(bounds), "300:m:2500\n65535:z:86400000\n");
  EXPECT_EQ(exportTableId(bounds), 3443090808u);
  const std::vector<ExportEntry> jobs = {entry(2, "job_b", 1500), entry(1, "job_a", 1)};
  EXPECT_EQ(exportTableText(jobs), "1:job_a:1\n2:job_b:1500\n");
  EXPECT_EQ(exportTableId(jobs), 2330640754u);
}

TEST(RtiContract, NextSeqWrapsSkippingZero)
{
  EXPECT_EQ(nextSeq(0), 1u);
  EXPECT_EQ(nextSeq(41), 42u);
  EXPECT_EQ(nextSeq(UINT32_MAX), 1u);
}

namespace
{
std::string writeTasks(const std::string& body)
{
  const std::string path = std::string(::testing::TempDir()) + "/p0a_tasks_" +
                           ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".yaml";
  std::ofstream(path) << body;
  return path;
}
std::string tasksYaml(const std::string& id_line, const std::string& place_export = "{number: 12, timeout_s: 30}")
{
  return "schema_version: 1\nmain_task: place_part\n" + id_line +
         "tasks:\n  home:\n    tree: task_home\n    inputs: []\n    outputs: []\n    export: {number: 7, timeout_s: 5}\n"
         "  place_part:\n    tree: task_place_part\n    inputs: []\n    outputs: []\n    export: " +
         place_export + "\n  inspect:\n    tree: task_inspect\n    inputs: []\n    outputs: []\n";
}
}  // namespace

TEST(RtiContract, LoadsTheExportedTasksOfATasksFile)
{
  const auto t = loadExportTable(writeTasks(tasksYaml("export_table_id: " + std::to_string(table().id) + "\n")));
  ASSERT_EQ(t.entries.size(), 2u);   // inspect is not exported: internal to the application
  EXPECT_EQ(t.entries[0].name, "home");
  EXPECT_EQ(t.entries[1].treeId(), "job_place_part");
  EXPECT_EQ(t.id, table().id);
  EXPECT_EQ(t.byNumber(12)->name, "place_part");
  EXPECT_EQ(t.byNumber(13), nullptr);
}

TEST(RtiContract, RefusesABadTasksFile)
{
  const std::string id = "export_table_id: " + std::to_string(table().id) + "\n";
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(""))), TableRefused);                         // id missing
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml("export_table_id: 12345\n"))), TableRefused);  // id differs (V6)
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 7, timeout_s: 30}"))), TableRefused);   // dup
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 0, timeout_s: 30}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 65536, timeout_s: 30}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12, timeout_s: 0}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12, timeout_s: 86401}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12, timeout_s: 30, recipe: 1}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks("schema_version: 1\nmain_task: a\ntasks:\n  a:\n    tree: task_a\n")),
               TableRefused);   // nothing exported: nothing to serve
  EXPECT_THROW(loadExportTable(writeTasks("schema_version: 2\ntasks: {}\n")), TableRefused);
  EXPECT_THROW(loadExportTable("/nonexistent/tasks.yaml"), TableRefused);
}

// ---------------------------------------------------------------------------------------------------- start-up
TEST(RtiStateMachine, SU1_StartsFaultedServerStartedUntilAReset)
{
  Rig r;
  EXPECT_EQ(r.st().state, State::FAULTED);
  EXPECT_EQ(r.st().fault_code, code::SERVER_STARTED);
  EXPECT_EQ(r.st().job_id, 0u);
  EXPECT_EQ(r.st().result, Result::NONE);
  EXPECT_FALSE(r.st().ready);
  r.adopt();
  r.check();
  EXPECT_EQ(r.st().state, State::FAULTED);   // ready conditions alone never leave the start-up fault
  EXPECT_FALSE(r.send(Cmd::START, 1, 12).start);
  EXPECT_EQ(r.st().ack, Ack::REJECTED);
  EXPECT_EQ(r.st().ack_code, code::FAULTED);
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().state, State::IDLE);
  EXPECT_EQ(r.st().fault_code, 0);
  EXPECT_TRUE(r.st().ready);
  EXPECT_FALSE(r.sm.heldState().has_value());   // a RESET never makes the held state known
}

TEST(RtiStateMachine, ResetFromFaultedWithTheRuntimeNotReadyGoesUnavailable)   // K5(b), C2(b)
{
  Rig r;
  r.adopt();
  r.check(false);
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().state, State::UNAVAILABLE);
  EXPECT_EQ(r.st().fault_code, 0);
  r.send(Cmd::START, 1, 12);
  EXPECT_EQ(r.st().ack_code, code::NOT_READY);
  r.check(true);
  EXPECT_EQ(r.st().state, State::IDLE);
}

// ---------------------------------------------------------------------------------------------------- normal
TEST(RtiStateMachine, N1_AStartIsAcceptedAtomicallyAndCompletes)
{
  Rig r;
  r.operational();
  const Effects fx = r.send(Cmd::START, 41, 12);
  ASSERT_TRUE(fx.start);
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().state, State::RUNNING);   // the same image
  EXPECT_EQ(r.st().job_id, 41u);
  EXPECT_EQ(r.st().task, 12);
  EXPECT_EQ(r.st().task_name, "place_part");
  EXPECT_EQ(fx.start->tree_id, "job_place_part");
  EXPECT_FALSE(fx.start->holding.has_value());   // first job after a start: unknown
  const auto d = r.end(fx.start->counter, JobEnd::Tree::SUCCESS);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->result, Result::SUCCEEDED);
  EXPECT_EQ(r.st().result_code, code::OK);
  EXPECT_EQ(r.st().state, State::IDLE);
  EXPECT_TRUE(r.st().ready);
}

TEST(RtiStateMachine, OutcomesMapToResultsAndCodes)   // N2 N3 F1 F2 F10
{
  struct Case
  {
    JobEnd::Tree tree;
    const char* outcome;
    bool eef;
    Result result;
    uint16_t code;
    State after;
  };
  const Case cases[] = {
    {JobEnd::Tree::SUCCESS, "", false, Result::SUCCEEDED, code::OK, State::IDLE},
    {JobEnd::Tree::SUCCESS, "BIN_EMPTY", false, Result::SUCCEEDED, code::BIN_EMPTY, State::IDLE},
    {JobEnd::Tree::SUCCESS, "NOTHING_PICKABLE", false, Result::SUCCEEDED, code::NOTHING_PICKABLE, State::IDLE},
    {JobEnd::Tree::FAILURE, "PICKABLE_EXHAUSTED", false, Result::ENDED, code::PICKABLE_EXHAUSTED, State::IDLE},
    {JobEnd::Tree::FAILURE, "RECOVERY_EXHAUSTED", false, Result::ENDED, code::RECOVERY_EXHAUSTED, State::IDLE},
    {JobEnd::Tree::FAILURE, "TERMINAL_FAULT", false, Result::FAILED, code::TERMINAL_FAULT, State::FAULTED},
    {JobEnd::Tree::FAILURE, "", false, Result::FAILED, code::TASK_FAILED, State::FAULTED},
    {JobEnd::Tree::SUCCESS, "", true, Result::FAILED, code::EEF_FAULT, State::FAULTED},
    {JobEnd::Tree::FAILURE, "PICKABLE_EXHAUSTED", true, Result::FAILED, code::EEF_FAULT, State::FAULTED},
  };
  uint32_t job = 1;
  for (const auto& c : cases)
  {
    Rig r;
    r.operational();
    const auto counter = r.start(job++);
    const auto d = r.end(counter, c.tree, c.outcome, false, c.eef);
    ASSERT_TRUE(d);
    EXPECT_EQ(d->result, c.result) << c.outcome;
    EXPECT_EQ(r.st().result_code, c.code) << c.outcome;
    EXPECT_EQ(r.st().state, c.after) << c.outcome;
    EXPECT_EQ(r.st().fault_code, c.after == State::FAULTED ? c.code : 0) << c.outcome;
  }
}

// ---------------------------------------------------------------------------------------------------- identity
TEST(RtiStateMachine, I1_I2_I4_DuplicateJobIdsAreRejectedNewOnesAccepted)
{
  Rig r;
  r.operational();
  const auto c1 = r.start(5);
  r.send(Cmd::START, 5, 12);   // I1: the running job's id
  EXPECT_EQ(r.st().ack_code, code::DUPLICATE_JOB);
  EXPECT_EQ(r.st().state, State::RUNNING);
  r.end(c1, JobEnd::Tree::SUCCESS);
  r.send(Cmd::START, 5, 12);   // I2: a finished job's id
  EXPECT_EQ(r.st().ack_code, code::DUPLICATE_JOB);
  EXPECT_EQ(r.st().state, State::IDLE);
  r.start(6);                  // I4
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
}

TEST(RtiStateMachine, I3_I5_K1_CancelOfAJobThatIsNotCurrentAndZeroIds)
{
  Rig r;
  r.operational();
  r.send(Cmd::CANCEL, 99);           // K1: IDLE, never started
  EXPECT_EQ(r.st().ack_code, code::NOT_CURRENT_JOB);
  const auto c1 = r.start(1);
  r.end(c1, JobEnd::Tree::SUCCESS);
  r.start(2);
  r.send(Cmd::CANCEL, 1);            // I3: a finished job while another runs
  EXPECT_EQ(r.st().ack_code, code::NOT_CURRENT_JOB);
  EXPECT_EQ(r.st().state, State::RUNNING);
  r.send(Cmd::START, 0, 12);         // I5
  EXPECT_EQ(r.st().ack_code, code::INVALID_REQUEST);
  r.send(Cmd::CANCEL, 0);
  EXPECT_EQ(r.st().ack_code, code::INVALID_REQUEST);
}

TEST(RtiStateMachine, I6_TheHistoryHoldsTheLast64Jobs)
{
  Rig r;
  r.operational();
  uint32_t id = 1000;
  r.end(r.start(1), JobEnd::Tree::SUCCESS);                 // job id 1, then 63 others: 64 jobs
  for (int k = 0; k < 63; ++k) r.end(r.start(++id), JobEnd::Tree::SUCCESS);
  r.send(Cmd::START, 1, 12);
  EXPECT_EQ(r.st().ack_code, code::DUPLICATE_JOB);          // (a) 64 jobs earlier
  r.end(r.start(++id), JobEnd::Tree::SUCCESS);              // the 65th
  const auto fx = r.send(Cmd::START, 1, 12);
  EXPECT_TRUE(fx.start);                                    // (b) 65 jobs earlier: out of the history
}

// ---------------------------------------------------------------------------------------------------- validation
TEST(RtiStateMachine, V1_V2_P11_UnknownTaskUnknownCommandAndNoneWithANewSeq)
{
  Rig r;
  r.operational();
  r.send(Cmd::START, 1, 99);
  EXPECT_EQ(r.st().ack_code, code::UNKNOWN_TASK);
  r.img.cmd_seq = nextSeq(r.img.cmd_seq);
  r.img.cmd = 7;
  r.feed();
  EXPECT_EQ(r.st().ack_code, code::INVALID_REQUEST);
  const uint32_t before = r.st().ack_seq;
  r.send(Cmd::NONE);
  EXPECT_EQ(r.st().ack_code, code::INVALID_REQUEST);
  EXPECT_EQ(r.st().ack_seq, nextSeq(before));               // advanced, nothing executed
  EXPECT_EQ(r.st().state, State::IDLE);
}

// ---------------------------------------------------------------------------------------------------- concurrency
TEST(RtiStateMachine, C1_C2_StartWhileRunningStoppingOrFaulted)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  r.send(Cmd::START, 2, 12);
  EXPECT_EQ(r.st().ack_code, code::BUSY);
  r.send(Cmd::CANCEL, 1);
  ASSERT_EQ(r.st().state, State::STOPPING);
  r.send(Cmd::START, 3, 12);
  EXPECT_EQ(r.st().ack_code, code::BUSY);
  r.end(c, JobEnd::Tree::HALTED, "", std::nullopt);         // unknown hold: FAULTED (309)
  ASSERT_EQ(r.st().state, State::FAULTED);
  r.send(Cmd::START, 4, 12);
  EXPECT_EQ(r.st().ack_code, code::FAULTED);
}

// ---------------------------------------------------------------------------------------------------- control
TEST(RtiStateMachine, K2_CancelWithTheHeldStateKnownFalseIsCancelled)
{
  Rig r;
  r.operational();
  const auto c0 = r.start(1);
  r.end(c0, JobEnd::Tree::SUCCESS, "", false);   // a tracking program: the held state known false
  const auto c = r.start(2);
  const Effects fx = r.send(Cmd::CANCEL, 2);
  ASSERT_TRUE(fx.stop);
  EXPECT_EQ(*fx.stop, c);
  EXPECT_EQ(r.st().state, State::STOPPING);
  const auto d = r.end(c, JobEnd::Tree::HALTED, "", false);
  EXPECT_EQ(d->result, Result::CANCELLED);
  EXPECT_EQ(r.st().result_code, code::CANCELLED);
  EXPECT_EQ(r.st().state, State::IDLE);
}

TEST(RtiStateMachine, K4_CancelWhileHoldingOrUnknownIsFaulted)
{
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    r.end(c, JobEnd::Tree::HALTED, "", true);
    EXPECT_EQ(r.st().result_code, code::CANCELLED_WHILE_HOLDING);
    EXPECT_EQ(r.st().state, State::FAULTED);
  }
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    r.end(c, JobEnd::Tree::HALTED, "", std::nullopt);
    EXPECT_EQ(r.st().result_code, code::CANCELLED_HOLD_UNKNOWN);
    EXPECT_EQ(r.st().state, State::FAULTED);
  }
}

TEST(RtiStateMachine, K6_K7_K9_K10_ResetAndCancelInEveryState)
{
  Rig r;
  r.operational();
  r.send(Cmd::RESET);                      // K7: IDLE
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().state, State::IDLE);
  const auto c = r.start(1);
  r.send(Cmd::RESET);                      // K6: RUNNING
  EXPECT_EQ(r.st().ack_code, code::INVALID_STATE);
  r.send(Cmd::CANCEL, 1);
  r.send(Cmd::RESET);                      // K6: STOPPING
  EXPECT_EQ(r.st().ack_code, code::INVALID_STATE);
  r.send(Cmd::CANCEL, 1);                  // K9: a second cancel
  EXPECT_EQ(r.st().ack_code, code::INVALID_STATE);
  EXPECT_EQ(r.st().state, State::STOPPING);
  r.end(c, JobEnd::Tree::HALTED, "", false);
  EXPECT_EQ(r.st().result_code, code::CANCELLED);
  r.check(false);                          // K10: UNAVAILABLE
  ASSERT_EQ(r.st().state, State::UNAVAILABLE);
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().state, State::UNAVAILABLE);
  r.send(Cmd::CANCEL, 1);
  EXPECT_EQ(r.st().ack_code, code::NOT_CURRENT_JOB);
}

TEST(RtiStateMachine, K8_K11_ACancelMasksAFailureButNotACompletion)
{
  {
    Rig r;   // K8: the stopped motion's failure ends the tree FAILURE with the gate's TERMINAL_FAULT
    r.operational();
    r.end(r.start(1), JobEnd::Tree::SUCCESS, "", false);
    const auto c = r.start(2);
    r.send(Cmd::CANCEL, 2);
    r.end(c, JobEnd::Tree::FAILURE, "TERMINAL_FAULT", false);
    EXPECT_EQ(r.st().result_code, code::CANCELLED);
    EXPECT_NE(r.st().detail.find("masked"), std::string::npos);
  }
  {
    Rig r;   // K11: the tree completed before the halt took effect
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::CANCEL, 1);
    r.end(c, JobEnd::Tree::SUCCESS, "", true);   // even holding: a pick task may end holding by design
    EXPECT_EQ(r.st().result, Result::SUCCEEDED);
    EXPECT_EQ(r.st().result_code, code::OK);
    EXPECT_EQ(r.st().state, State::IDLE);
  }
}

TEST(RtiStateMachine, K12_K13_TheHeldStateIsCarriedAcrossJobsAndUnknownAfterAStart)
{
  Rig r;
  r.operational();
  const Effects f1 = r.send(Cmd::START, 1, 12);
  EXPECT_FALSE(f1.start->holding.has_value());   // K13: unknown after the start
  r.end(f1.start->counter, JobEnd::Tree::SUCCESS, "", true);   // a pick: ends holding
  const Effects f2 = r.send(Cmd::START, 2, 12);
  ASSERT_TRUE(f2.start->holding.has_value());
  EXPECT_TRUE(*f2.start->holding);               // K12: seeded HELD
  r.send(Cmd::CANCEL, 2);
  r.end(f2.start->counter, JobEnd::Tree::HALTED, "", true);
  EXPECT_EQ(r.st().result_code, code::CANCELLED_WHILE_HOLDING);
}

// ---------------------------------------------------------------------------------------------------- faults
TEST(RtiStateMachine, F3_RuntimeLostWhileIdleAndBack)
{
  Rig r;
  r.operational();
  r.check(false);
  EXPECT_EQ(r.st().state, State::UNAVAILABLE);
  r.send(Cmd::START, 1, 12);
  EXPECT_EQ(r.st().ack_code, code::NOT_READY);
  r.check(true);
  EXPECT_EQ(r.st().state, State::IDLE);
}

TEST(RtiStateMachine, F4_F5_RuntimeLostOrTimeoutWhileRunning)
{
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    const Effects fx = r.check(false);
    ASSERT_TRUE(fx.stop);
    EXPECT_EQ(r.st().state, State::STOPPING);
    r.end(c, JobEnd::Tree::HALTED);
    EXPECT_EQ(r.st().result_code, code::RUNTIME_LOST);
    EXPECT_EQ(r.st().state, State::FAULTED);
  }
  {
    Rig r;
    r.operational();
    const auto c = r.start(1, 7);   // home: 5 s
    for (int k = 0; k < 49; ++k)
    {
      r.advance(100);
      r.beat();
      r.check();
    }
    EXPECT_EQ(r.st().state, State::RUNNING);   // 4.9 s
    r.advance(100);
    const Effects fx = r.beat();               // 5.0 s: the first input applied at the deadline stops it
    ASSERT_TRUE(fx.stop);
    EXPECT_EQ(r.sm.cause(), Cause::TASK_TIMEOUT);
    r.end(c, JobEnd::Tree::SUCCESS);           // however the tree ended: the first cause owns the result
    EXPECT_EQ(r.st().result_code, code::TASK_TIMEOUT);
    EXPECT_EQ(r.st().state, State::FAULTED);
  }
}

TEST(RtiStateMachine, F6_F7_RequesterLostWhileIdleOrRunning)
{
  {
    Rig r;   // F6
    r.operational();
    r.advance(1000);
    r.check();
    EXPECT_EQ(r.st().state, State::IDLE);      // exactly 1.0 s: not yet
    r.advance(1);
    r.check();
    EXPECT_EQ(r.st().state, State::UNAVAILABLE);
    r.beat();                                  // the same session resumes: no re-adoption, no RESET
    EXPECT_EQ(r.st().state, State::IDLE);
  }
  {
    Rig r;   // F7
    r.operational();
    const auto c = r.start(1);
    r.advance(1001);
    const Effects fx = r.check();
    ASSERT_TRUE(fx.stop);
    r.end(c, JobEnd::Tree::HALTED, "", false);
    EXPECT_EQ(r.st().result_code, code::REQUESTER_LOST);
    EXPECT_EQ(r.st().state, State::FAULTED);
    r.beat();
    r.send(Cmd::RESET);
    EXPECT_EQ(r.st().state, State::IDLE);
  }
}

TEST(RtiStateMachine, F11_LossesWhileFaultedChangeNothing)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  r.end(c, JobEnd::Tree::FAILURE, "TERMINAL_FAULT");
  ASSERT_EQ(r.st().fault_code, code::TERMINAL_FAULT);
  r.advance(2000);
  r.check(false);
  EXPECT_EQ(r.st().state, State::FAULTED);
  EXPECT_EQ(r.st().fault_code, code::TERMINAL_FAULT);
  r.beat();
  r.check(true);
  EXPECT_EQ(r.st().state, State::FAULTED);
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().state, State::IDLE);
}

TEST(RtiStateMachine, F12_TheFirstCauseWinsAndARequesterLossNeverTurnsACancelIntoAFault)
{
  Rig r;
  r.operational();
  r.end(r.start(1), JobEnd::Tree::SUCCESS, "", false);
  const auto c = r.start(2, 7);      // home: timeout_s 5
  for (int k = 0; k < 40; ++k)       // 4 s of a live requester
  {
    r.advance(100);
    r.beat();
  }
  r.send(Cmd::CANCEL, 2);            // t = 4 s: the stop is bounded at t = 9 s (Config::stop_timeout 5 s)
  r.advance(2000);                   // t = 6 s: the requester is lost AND the 5 s timeout passed, during the stop
  r.check();
  EXPECT_EQ(r.sm.cause(), Cause::CANCEL);
  r.end(c, JobEnd::Tree::HALTED, "", false);
  EXPECT_EQ(r.st().result, Result::CANCELLED);
  EXPECT_EQ(r.st().result_code, code::CANCELLED);
  EXPECT_EQ(r.st().state, State::UNAVAILABLE);   // CANCELLED -> IDLE -> UNAVAILABLE (the requester is absent)
  EXPECT_NE(r.st().detail.find("REQUESTER_LOST"), std::string::npos);
  EXPECT_NE(r.st().detail.find("TASK_TIMEOUT"), std::string::npos);
}

TEST(RtiStateMachine, F12_FailClosed_HeldOrUnknownStaysFaultedWhateverFollows)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);         // the held state unknown (first job)
  r.send(Cmd::CANCEL, 1);
  r.advance(2000);
  r.check();
  r.end(c, JobEnd::Tree::HALTED, "", std::nullopt);
  EXPECT_EQ(r.st().result_code, code::CANCELLED_HOLD_UNKNOWN);
  EXPECT_EQ(r.st().state, State::FAULTED);
}

TEST(RtiStateMachine, Precedence_ARuntimeLossDuringACancelIsAStrictlyMoreSevereUncertainty)
{
  Rig r;
  r.operational();
  r.end(r.start(1), JobEnd::Tree::SUCCESS, "", false);
  const auto c = r.start(2);
  r.send(Cmd::CANCEL, 2);
  r.check(false);                    // MoveIt / an end-effector device lost during the stop
  r.end(c, JobEnd::Tree::HALTED, "", false);
  EXPECT_EQ(r.st().result_code, code::RUNTIME_LOST);
  EXPECT_EQ(r.st().state, State::FAULTED);
}

TEST(RtiStateMachine, Precedence_OneEvaluationFindingSeveralCausesRecordsTheFixedOrder)
{
  Rig r;
  r.operational();
  const auto c = r.start(1, 7);
  r.advance(6000);                   // requester lost, runtime lost, timeout, two publishers: all at once
  r.check(false, 2);
  EXPECT_EQ(r.sm.cause(), Cause::REQUESTER_CONFLICT);
  r.end(c, JobEnd::Tree::HALTED);
  EXPECT_EQ(r.st().result_code, code::REQUESTER_CONFLICT);
}

// Race orderings (Decision 8a.5): a CANCEL and the tree's natural SUCCESS -- either order, the same result.
TEST(RtiStateMachine, Race_CancelVersusNaturalEnd_BothOrdersGiveTheJobsOwnResult)
{
  Rig a;   // the end first, then the cancel
  a.operational();
  const auto ca = a.start(1);
  a.end(ca, JobEnd::Tree::SUCCESS, "BIN_EMPTY", false);
  a.send(Cmd::CANCEL, 1);
  EXPECT_EQ(a.st().ack_code, code::NOT_CURRENT_JOB);
  Rig b;   // the cancel first, then the end (the tree had completed)
  b.operational();
  const auto cb = b.start(1);
  b.send(Cmd::CANCEL, 1);
  EXPECT_EQ(b.st().ack, Ack::ACCEPTED);
  b.end(cb, JobEnd::Tree::SUCCESS, "BIN_EMPTY", false);
  EXPECT_EQ(a.st().result, b.st().result);
  EXPECT_EQ(a.st().result_code, code::BIN_EMPTY);
  EXPECT_EQ(b.st().result_code, code::BIN_EMPTY);
  EXPECT_EQ(a.st().state, b.st().state);
}

TEST(RtiStateMachine, Race_ALateEndOfAnEarlierJobIsIgnored)
{
  Rig r;
  r.operational();
  const auto c1 = r.start(1);
  r.end(c1, JobEnd::Tree::SUCCESS, "", false);
  const auto c2 = r.start(2);
  EXPECT_FALSE(r.end(c1, JobEnd::Tree::FAILURE));   // a duplicate report of job 1 never ends job 2
  EXPECT_EQ(r.st().state, State::RUNNING);
  EXPECT_TRUE(r.end(c2, JobEnd::Tree::SUCCESS, "", false));
}

// ---------------------------------------------------------------------------------------------------- protocol
TEST(RtiStateMachine, P2_P3_P10_OncePerSeqSequenceErrorsAndTheWrap)
{
  Rig r;
  r.operational();
  r.start(1);
  for (int k = 0; k < 5; ++k) r.feed();          // P2: the same image again: no-op
  EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
  EXPECT_EQ(r.st().job_id, 1u);
  r.img.cmd_seq = nextSeq(nextSeq(r.img.cmd_seq));   // P3: one skipped
  r.img.cmd = static_cast<uint8_t>(Cmd::RESET);
  r.feed();
  EXPECT_EQ(r.st().ack_code, code::SEQUENCE_ERROR);
  r.send(Cmd::RESET);                            // the next in-order one
  EXPECT_EQ(r.st().ack_code, code::INVALID_STATE);   // processed (RUNNING): no longer a sequence error

  Rig w;                                         // P10: the wrap
  w.img.cmd_seq = UINT32_MAX - 1;
  w.operational();                               // the baseline is UINT32_MAX - 1; RESET used UINT32_MAX
  ASSERT_EQ(w.st().ack_seq, UINT32_MAX);
  w.send(Cmd::START, 1, 12);                     // seq 1 after UINT32_MAX
  EXPECT_EQ(w.st().ack_seq, 1u);
  EXPECT_EQ(w.st().ack, Ack::ACCEPTED);
}

TEST(RtiStateMachine, P5_AdoptionTakesTwoHeartbeatChangesAndNeverExecutesTheImage)
{
  Rig r;
  r.check();
  r.img.cmd_seq = 3;
  r.img.cmd = static_cast<uint8_t>(Cmd::START);   // (c) a START left level-held in the image
  r.img.job_id = 9;
  r.img.task = 12;
  r.feed(1);
  EXPECT_FALSE(r.sm.requesterAdopted());
  EXPECT_EQ(r.st().ack_code, code::NOT_SYNCHRONIZED);
  r.beat();                                       // (a) one change: not yet
  EXPECT_FALSE(r.sm.requesterAdopted());
  r.img.cmd_seq = 4;
  r.feed();
  EXPECT_EQ(r.st().ack_code, code::NOT_SYNCHRONIZED);
  EXPECT_EQ(r.st().state, State::FAULTED);
  const Effects fx = r.beat();                    // (b) two: adopted, the image's seq is the baseline
  EXPECT_TRUE(r.sm.requesterAdopted());
  EXPECT_FALSE(fx.start);                         // (c) never executed
  EXPECT_EQ(r.st().ack_seq, 4u);
  EXPECT_EQ(r.st().ack, Ack::NONE);
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().state, State::IDLE);
}

TEST(RtiStateMachine, P7_ARequesterRestartWithItsCountersResetIsReadopted)
{
  Rig r;
  r.operational();
  r.end(r.start(1), JobEnd::Tree::SUCCESS, "", false);
  r.img = Command{};                              // a restart: new session, cmd_seq and job ids from the start
  r.img.requester_session = 8;
  r.adopt();
  EXPECT_EQ(r.st().state, State::IDLE);
  const Effects fx = r.send(Cmd::START, 1, 12);   // job id 1 again, in a NEW session
  EXPECT_TRUE(fx.start);
}

TEST(RtiStateMachine, P12_ASessionChangeDuringAJobIsRequesterLost)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  r.img = Command{};
  r.img.requester_session = 8;
  const Effects fx = r.feed(1);                   // the first image of the new session
  ASSERT_TRUE(fx.stop);
  EXPECT_EQ(r.sm.cause(), Cause::REQUESTER_LOST);
  r.end(c, JobEnd::Tree::HALTED, "", false);
  EXPECT_EQ(r.st().result_code, code::REQUESTER_LOST);
  EXPECT_EQ(r.st().state, State::FAULTED);
  r.beat();
  r.beat();                                       // the new session adopted
  r.send(Cmd::RESET);
  EXPECT_EQ(r.st().state, State::IDLE);
}

TEST(RtiStateMachine, P6_TwoPublishers)
{
  {
    Rig r;   // (a) IDLE: UNAVAILABLE, every command NOT_SYNCHRONIZED; the adopted session resumes after
    r.operational();
    r.check(true, 2);
    EXPECT_EQ(r.st().state, State::UNAVAILABLE);
    EXPECT_EQ(r.st().detail, "TWO_REQUESTERS");
    EXPECT_FALSE(r.send(Cmd::START, 1, 12).start);
    EXPECT_EQ(r.st().ack_code, code::NOT_SYNCHRONIZED);
    r.check(true, 1);
    r.beat();
    EXPECT_EQ(r.st().state, State::IDLE);
    EXPECT_TRUE(r.send(Cmd::START, 2, 12).start);
  }
  {
    Rig r;   // (b) RUNNING, the second writes the adopted session: REQUESTER_CONFLICT
    r.operational();
    const auto c = r.start(1);
    ASSERT_TRUE(r.check(true, 2).stop);
    r.end(c, JobEnd::Tree::HALTED, "", false);
    EXPECT_EQ(r.st().result_code, code::REQUESTER_CONFLICT);
  }
  {
    Rig r;   // (c) a foreign session counted with its first image: the conflict outranks the session change
    r.operational();
    const auto c = r.start(1);
    r.img.requester_session = 99;
    ASSERT_TRUE(r.feed(2).stop);
    EXPECT_EQ(r.sm.cause(), Cause::REQUESTER_CONFLICT);
    EXPECT_TRUE(r.sm.requesterAdopted());          // not taken as a session change
    r.end(c, JobEnd::Tree::HALTED, "", false);
    EXPECT_EQ(r.st().result_code, code::REQUESTER_CONFLICT);
  }
  {
    Rig r;   // (c') the discovery race: the foreign image before the graph shows it -> REQUESTER_LOST, still fails closed
    r.operational();
    const auto c = r.start(1);
    r.img.requester_session = 99;
    ASSERT_TRUE(r.feed(1).stop);
    r.end(c, JobEnd::Tree::HALTED, "", false);
    EXPECT_EQ(r.st().result_code, code::REQUESTER_LOST);
    EXPECT_EQ(r.st().state, State::FAULTED);
  }
}

// ---------------------------------------------------------------------------------------------------- review fixes
// P0.a protocol review M1 (ADR-0032 Decisions 6 and 10): the adopted session's sequence reference is its OWN last
// cmd_seq, never the published ack_seq -- a foreign image that rewrites the acknowledgement can never re-arm a
// command the adopted session already had processed (or rejected).
TEST(RtiStateMachine, Review_M1_AForeignImageNeverReArmsTheAdoptedSessionsCommand)
{
  {
    Rig r;   // a START rejected NOT_READY; a one-shot foreign writer; the adopted image again: never executed
    r.operational();
    r.check(false);
    ASSERT_EQ(r.st().state, State::UNAVAILABLE);
    EXPECT_FALSE(r.send(Cmd::START, 7, 12).start);
    EXPECT_EQ(r.st().ack_code, code::NOT_READY);
    const Command adopted = r.img;
    Command foreign = adopted;
    foreign.requester_session = 4242;
    foreign.cmd_seq = adopted.cmd_seq - 1;
    r.sm.onCommand(foreign, r.now, 2);                    // counted with it: two publishers
    EXPECT_EQ(r.st().ack_code, code::NOT_SYNCHRONIZED);
    EXPECT_EQ(r.st().ack_seq, foreign.cmd_seq);           // the acknowledgement fields: rewritten
    r.check(true, 1);                                     // the writer gone, the runtime back
    EXPECT_FALSE(r.sm.onCommand(adopted, r.now, 1).start);   // the adopted level-held image (recounted)
    r.beat();
    r.check();
    EXPECT_EQ(r.st().state, State::IDLE);                 // nothing started
    EXPECT_TRUE(r.send(Cmd::START, 8, 12).start);         // the next in-order command
  }
  {
    Rig r;   // a RESET rejected INVALID_STATE during a job that then faults: the foreign image never replays it
    r.operational();
    const auto c = r.start(1);
    r.send(Cmd::RESET);
    EXPECT_EQ(r.st().ack_code, code::INVALID_STATE);
    r.end(c, JobEnd::Tree::FAILURE, "TERMINAL_FAULT");
    ASSERT_EQ(r.st().state, State::FAULTED);
    Command foreign = r.img;
    foreign.requester_session = 4242;
    foreign.cmd_seq = r.img.cmd_seq - 1;
    r.sm.onCommand(foreign, r.now, 2);
    r.check(true, 1);
    r.feed(1);
    r.beat();
    EXPECT_EQ(r.st().state, State::FAULTED);              // the fault was NOT acknowledged by a replay
    EXPECT_EQ(r.st().fault_code, code::TERMINAL_FAULT);
  }
  {
    Rig r;   // a new command of the adopted session DURING a conflict is consumed as rejected, never run afterwards
    r.operational();
    r.check(true, 2);
    EXPECT_FALSE(r.send(Cmd::START, 5, 12).start);
    EXPECT_EQ(r.st().ack_code, code::NOT_SYNCHRONIZED);
    r.check(true, 1);
    EXPECT_FALSE(r.beat().start);
    EXPECT_EQ(r.st().state, State::IDLE);
    EXPECT_TRUE(r.send(Cmd::START, 6, 12).start);
  }
}

// P0.a protocol review M2 (ADR-0032 Decisions 8, 9): STOPPING is bounded by Config::stop_timeout.
TEST(RtiStateMachine, Review_M2_StoppingIsBounded)
{
  {
    Rig r;   // a CANCEL whose tree never returns
    r.operational();
    const auto c = r.start(1);
    ASSERT_TRUE(r.send(Cmd::CANCEL, 1).stop);
    for (int k = 0; k < 49; ++k)
    {
      r.advance(100);
      r.beat();
      r.check();
    }
    EXPECT_EQ(r.st().state, State::STOPPING);             // 4.9 s
    r.advance(100);
    r.beat();
    r.check();                                            // 5.0 s: decided now
    EXPECT_EQ(r.st().state, State::FAULTED);
    EXPECT_EQ(r.st().result, Result::FAILED);
    EXPECT_EQ(r.st().result_code, code::CANCELLED_HOLD_UNKNOWN);   // the tree never reported: unknown
    EXPECT_EQ(r.st().fault_code, code::CANCELLED_HOLD_UNKNOWN);
    EXPECT_NE(r.st().detail.find("did not complete"), std::string::npos) << r.st().detail;
    EXPECT_FALSE(r.sm.heldState().has_value());
    EXPECT_TRUE(r.sm.jobStuck());
    EXPECT_EQ(r.sm.stoppingJob(), std::optional<uint64_t>(c));    // still stopped every period
    r.send(Cmd::RESET);                                   // acknowledged; the tick thread is still busy
    EXPECT_EQ(r.st().ack, Ack::ACCEPTED);
    EXPECT_EQ(r.st().state, State::UNAVAILABLE);
    EXPECT_NE(r.st().detail.find("has not returned"), std::string::npos) << r.st().detail;
    EXPECT_FALSE(r.send(Cmd::START, 2, 12).start);
    EXPECT_EQ(r.st().ack_code, code::NOT_READY);
    EXPECT_FALSE(r.end(c, JobEnd::Tree::HALTED, "", false).has_value());   // the late end: ignored (decided once)
    EXPECT_FALSE(r.sm.jobStuck());
    EXPECT_FALSE(r.sm.stoppingJob().has_value());
    EXPECT_EQ(r.st().result_code, code::CANCELLED_HOLD_UNKNOWN);
    EXPECT_FALSE(r.sm.heldState().has_value());           // not the late end's "false"
    EXPECT_EQ(r.st().state, State::IDLE);
    EXPECT_TRUE(r.send(Cmd::START, 3, 12).start);
  }
  {
    Rig r;   // a task timeout whose stop never completes: the recorded cause's code
    r.operational();
    r.start(1, 7);                                        // home: 5 s
    for (int k = 0; k < 100 && r.st().state != State::FAULTED; ++k)
    {
      r.advance(100);
      r.beat();
      r.check();
    }
    EXPECT_EQ(r.st().state, State::FAULTED);
    EXPECT_EQ(r.st().result_code, code::TASK_TIMEOUT);
    EXPECT_TRUE(r.sm.jobStuck());
  }
}

// P0.a protocol review minor 2 (ADR-0032 Decision 8): an end that leaves the tool physically unconfirmed hands the next
// job an UNKNOWN held state, whatever the key said.
TEST(RtiStateMachine, Review_TheHeldStateIsUnknownAfterAnUnconfirmedEnd)
{
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    r.check(false);                                       // the runtime lost
    r.end(c, JobEnd::Tree::HALTED, "", false);
    EXPECT_EQ(r.st().result_code, code::RUNTIME_LOST);
    EXPECT_FALSE(r.sm.heldState().has_value());
  }
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    r.end(c, JobEnd::Tree::SUCCESS, "", false, /*eef=*/true);
    EXPECT_EQ(r.st().result_code, code::EEF_FAULT);
    EXPECT_FALSE(r.sm.heldState().has_value());
  }
  {
    Rig r;   // a confirmed end hands its state on (K12)
    r.operational();
    const auto c = r.start(1);
    r.end(c, JobEnd::Tree::SUCCESS, "", true);
    EXPECT_EQ(r.sm.heldState(), std::optional<bool>(true));
  }
}

// P0.a protocol review minor 3 (ADR-0032 Decision 7): a failure outcome on a tree that SUCCEEDED is inconsistent (a
// hand-written tree): fail closed.
TEST(RtiStateMachine, Review_AFailureOutcomeOnASucceededTreeIsTaskFailed)
{
  for (const char* outcome : {"TERMINAL_FAULT", "PICKABLE_EXHAUSTED", "RECOVERY_EXHAUSTED"})
  {
    Rig r;
    r.operational();
    const auto c = r.start(1);
    r.end(c, JobEnd::Tree::SUCCESS, outcome);
    EXPECT_EQ(r.st().result, Result::FAILED) << outcome;
    EXPECT_EQ(r.st().result_code, code::TASK_FAILED) << outcome;
  }
}

// P0.a protocol review minor 4: a finished job's `detail` stays with its result (a late TRANSIENT_LOCAL reader sees
// it) until the next START.
TEST(RtiStateMachine, Review_AFinishedJobsDetailStaysUntilTheNextStart)
{
  Rig r;
  r.operational();
  const auto c = r.start(1);
  r.send(Cmd::CANCEL, 1);
  r.end(c, JobEnd::Tree::SUCCESS, "BIN_EMPTY");          // K11: too late
  ASSERT_EQ(r.st().state, State::IDLE);
  for (int k = 0; k < 5; ++k)
  {
    r.advance(100);
    r.beat();
    r.check();
  }
  EXPECT_NE(r.st().detail.find("too late"), std::string::npos) << r.st().detail;
  r.start(2);
  EXPECT_TRUE(r.st().detail.empty()) << r.st().detail;
}

// P0.a protocol review minor 1: the canonical text is in milliseconds -- a timeout below 1 ms, or not a whole number
// of them, is refused (TSA refuses it too), so a job never starts with a zero deadline.
TEST(RtiContract, Review_ATimeoutIsAWholeNumberOfMilliseconds)
{
  const std::string id = "export_table_id: " + std::to_string(table().id) + "\n";
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12, timeout_s: 0.0004}"))), TableRefused);
  EXPECT_THROW(loadExportTable(writeTasks(tasksYaml(id, "{number: 12, timeout_s: 30.0005}"))), TableRefused);
  ExportEntry e;
  e.number = 12;
  e.name = "place_part";
  e.timeout_s = 0.001;
  e.timeout_ms = 1;
  ExportEntry h = table().entries[0];
  const std::string ok = "export_table_id: " + std::to_string(exportTableId({h, e})) + "\n";
  const auto t = loadExportTable(writeTasks(tasksYaml(ok, "{number: 12, timeout_s: 0.001}")));
  EXPECT_EQ(t.byNumber(12)->timeout_ms, 1);
}
