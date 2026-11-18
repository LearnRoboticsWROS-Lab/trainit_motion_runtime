// test_p0a_rti_server (P0.a, ADR-0032): the frozen matrix's S cases, END TO END through the ROS binding -- the real
// RobotTaskServer (its callback group, its lock, its wall timer, its QoS) driven by the REFERENCE requester (the fake
// PLC's logic, Decision 10a), with a SCRIPTED job executor (scripted durations, outcomes, held state, stop latency)
// and a scripted runtime probe. Domain 75, localhost only. Each test is named after its case; its measured timings
// are recorded as test properties (the gtest XML is the evidence).
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/reference_requester.hpp"
#include "trainit_motion_runtime/robot_task_server.hpp"

using namespace trainit::rti;
using namespace std::chrono_literals;
using StatusMsg = trainit_interfaces::msg::RobotTaskStatus;
using CommandMsg = trainit_interfaces::msg::RobotTaskCommand;
using Clock = std::chrono::steady_clock;

namespace
{
ExportTable testTable()
{
  ExportTable t;
  auto add = [&](uint16_t n, const char* name, double s) {
    ExportEntry e;
    e.number = n;
    e.name = name;
    e.timeout_s = s;
    e.timeout_ms = static_cast<int64_t>(s * 1000);
    t.entries.push_back(e);
  };
  add(3, "quick", 1.0);
  add(7, "home", 5.0);
  add(12, "place_part", 30.0);
  t.id = exportTableId(t.entries);
  return t;
}

double msSince(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// --- the scripted job executor ------------------------------------------------------------------------------------
class ScriptedExecutor : public JobExecutor
{
public:
  struct Script
  {
    int run_ms{200};
    JobEnd::Tree tree{JobEnd::Tree::SUCCESS};
    std::string outcome;
    std::optional<std::optional<bool>> holding_end;   // unset: the job leaves the seeded held state unchanged
    bool eef_fault{false};
    int stop_ms{10};                                   // the halt's latency after a stop
    bool complete_on_stop{false};                      // the tree completes SUCCESS before the halt takes effect
  };

  std::function<std::optional<Decided>(uint64_t, const JobEnd&)> report;

  ScriptedExecutor() : worker_([this] { loop(); }) {}
  ~ScriptedExecutor() override { finish(); }
  // ends the worker (idempotent): after it no report reaches the server
  void finish()
  {
    {
      std::lock_guard<std::mutex> l(m_);
      quit_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  void push(const Script& s)
  {
    std::lock_guard<std::mutex> l(m_);
    scripts_.push_back(s);
  }
  void start(const StartJob& job) override
  {
    {
      std::lock_guard<std::mutex> l(m_);
      started_.push_back(job);
      pending_ = job;
    }
    cv_.notify_all();
  }
  void requestStop(uint64_t counter) override
  {
    {
      std::lock_guard<std::mutex> l(m_);
      ++stops_;
      if (current_ == counter || (pending_ && pending_->counter == counter)) stop_ = counter;
    }
    cv_.notify_all();
  }
  std::vector<StartJob> started() const
  {
    std::lock_guard<std::mutex> l(m_);
    return started_;
  }
  int stops() const
  {
    std::lock_guard<std::mutex> l(m_);
    return stops_;
  }

private:
  void loop()
  {
    std::unique_lock<std::mutex> l(m_);
    for (;;)
    {
      cv_.wait(l, [&] { return quit_ || pending_; });
      if (quit_) return;
      const StartJob job = *pending_;
      pending_.reset();
      current_ = job.counter;
      Script s = scripts_.empty() ? Script{} : scripts_.front();
      if (!scripts_.empty()) scripts_.pop_front();
      const auto deadline = Clock::now() + std::chrono::milliseconds(s.run_ms);
      cv_.wait_until(l, deadline, [&] { return quit_ || stop_ == job.counter; });
      if (quit_) return;
      JobEnd end;
      end.holding = s.holding_end ? *s.holding_end : job.holding;
      end.eef_fault = s.eef_fault;
      if (stop_ == job.counter)
      {
        cv_.wait_for(l, std::chrono::milliseconds(s.stop_ms), [&] { return quit_; });   // the halt's latency
        if (quit_) return;
        end.tree = s.complete_on_stop ? JobEnd::Tree::SUCCESS : JobEnd::Tree::HALTED;
        end.outcome = s.complete_on_stop ? s.outcome : "";
      }
      else
      {
        end.tree = s.tree;
        end.outcome = s.outcome;
      }
      current_ = 0;
      stop_ = 0;
      auto rep = report;
      l.unlock();
      if (rep) rep(job.counter, end);
      l.lock();
    }
  }

  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<Script> scripts_;
  std::optional<StartJob> pending_;
  std::vector<StartJob> started_;
  uint64_t current_{0};
  uint64_t stop_{0};
  int stops_{0};
  bool quit_{false};
  std::thread worker_;
};

class ScriptedProbe : public RuntimeProbe
{
public:
  std::atomic<bool> up{true};
  bool ready(std::string& why) override
  {
    if (!up) why = "scripted: MoveIt not answering";
    return up;
  }
};

// --- a status relay that DELAYS (FIFO, order kept) the statuses produced inside a window: a late acknowledgement --
class DelayRelay
{
public:
  DelayRelay(const rclcpp::Node::SharedPtr& node, std::chrono::milliseconds delay) : node_(node), delay_(delay)
  {
    pub_ = node_->create_publisher<StatusMsg>("p0a/delayed_status", RobotTaskServer::qos());
    sub_ = node_->create_subscription<StatusMsg>(kStatusTopic, RobotTaskServer::qos(), [this](StatusMsg::SharedPtr m) {
      std::lock_guard<std::mutex> l(m_);
      const auto now = Clock::now();
      auto release = now < window_end_ ? now + delay_ : now;
      if (!q_.empty() && release < q_.back().first) release = q_.back().first;   // never reorder
      q_.emplace_back(release, *m);
    });
    timer_ = node_->create_wall_timer(5ms, [this] {
      std::lock_guard<std::mutex> l(m_);
      while (!q_.empty() && q_.front().first <= Clock::now())
      {
        pub_->publish(q_.front().second);
        q_.pop_front();
      }
    });
  }
  void delayFor(std::chrono::milliseconds window)
  {
    std::lock_guard<std::mutex> l(m_);
    window_end_ = Clock::now() + window;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::milliseconds delay_;
  std::mutex m_;
  std::deque<std::pair<Clock::time_point, StatusMsg>> q_;
  Clock::time_point window_end_{};
  rclcpp::Publisher<StatusMsg>::SharedPtr pub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// --- the rig: one server, one requester, one recorder, spun by one executor ------------------------------------------
struct Rig
{
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 4};
  std::thread spinner;
  rclcpp::Node::SharedPtr server_node, plc_node, rec_node;
  ScriptedExecutor executor;
  ScriptedProbe probe;
  std::unique_ptr<RobotTaskServer> server;
  std::unique_ptr<ReferenceRequester> plc;
  std::unique_ptr<DelayRelay> relay;
  mutable std::mutex rec_m;
  std::vector<StatusMsg> recorded;
  rclcpp::Subscription<StatusMsg>::SharedPtr rec_sub;
  bool spinning{false};

  RobotTaskServer::Params params;

  explicit Rig(bool with_server = true, ReferenceRequester::Options opts = {}, bool with_relay = false,
               RobotTaskServer::Params server_params = {})
  : params(server_params)
  {
    static std::atomic<int> n{0};
    const std::string k = std::to_string(++n);
    rec_node = std::make_shared<rclcpp::Node>("p0a_rec_" + k);
    rec_sub = rec_node->create_subscription<StatusMsg>(kStatusTopic, RobotTaskServer::qos(), [this](StatusMsg::SharedPtr m) {
      std::lock_guard<std::mutex> l(rec_m);
      recorded.push_back(*m);
    });
    exec.add_node(rec_node);
    if (with_server)
    {
      newServer();
      executor.report = [this](uint64_t c, const JobEnd& e) {
        std::lock_guard<std::mutex> l(server_m);
        return server ? server->jobEnded(c, e) : std::nullopt;
      };
    }
    if (with_relay)
    {
      relay = std::make_unique<DelayRelay>(rec_node, 1500ms);
      opts.status_topic = "p0a/delayed_status";
    }
    newRequester(opts);
    spinner = std::thread([this] { exec.spin(); });
    spinning = true;
  }
  ~Rig()
  {
    executor.finish();   // first: no job report may reach a server being destroyed
    if (server) server->shutdown();
    exec.cancel();
    if (spinner.joinable()) spinner.join();
    executor.report = nullptr;
  }
  std::mutex server_m;
  void newServer()
  {
    std::lock_guard<std::mutex> l(server_m);
    server_node = std::make_shared<rclcpp::Node>("p0a_server_" + std::to_string(++server_count()));
    auto group = server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    exec.add_node(server_node);
    server = std::make_unique<RobotTaskServer>(server_node, group, params, testTable(), executor, probe);
  }
  static int& server_count()
  {
    static int n = 0;
    return n;
  }
  void newRequester(ReferenceRequester::Options opts = {})
  {
    static std::atomic<int> n{0};
    if (plc_node) dropRequester();
    plc_node = std::make_shared<rclcpp::Node>("p0a_plc_" + std::to_string(++n));
    plc = std::make_unique<ReferenceRequester>(plc_node, opts);
    exec.add_node(plc_node);
  }
  void dropRequester()   // a requester process that ends: its publisher leaves the graph
  {
    exec.remove_node(plc_node);
    plc.reset();
    plc_node.reset();
    if (server_node)
    {
      const auto deadline = Clock::now() + 5s;
      while (server_node->count_publishers("/" + std::string(kCommandTopic)) > 0 && Clock::now() < deadline)
        std::this_thread::sleep_for(20ms);
    }
  }
  bool waitState(uint8_t state, std::chrono::milliseconds t = 3000ms)
  {
    return plc->waitFor([state](const StatusMsg& s) { return s.state == state; }, t);
  }
  bool waitResult(uint32_t job, std::chrono::milliseconds t = 5000ms)
  {
    return plc->waitFor([job](const StatusMsg& s) { return s.job_id == job && s.result != StatusMsg::RESULT_NONE; }, t);
  }
  StatusMsg st() const { return *plc->lastStatus(); }
  // adopted, the start-up RESET accepted, IDLE
  void operational()
  {
    ASSERT_TRUE(plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, 3000ms)) << "no server status";
    std::this_thread::sleep_for(300ms);   // the requester's first heartbeats: adopted
    const auto r = plc->reset();
    ASSERT_EQ(r.outcome, ReferenceRequester::Outcome::ACCEPTED) << r.text;
    ASSERT_TRUE(waitState(StatusMsg::STATE_IDLE)) << st().detail;
  }
  // the first status (in arrival order) whose ack_seq is `seq`
  std::optional<StatusMsg> firstAck(uint32_t seq) const
  {
    std::lock_guard<std::mutex> l(rec_m);
    for (const auto& s : recorded)
      if (s.ack_seq == seq && s.ack != StatusMsg::ACK_NONE) return s;
    return std::nullopt;
  }
};

using O = ReferenceRequester::Outcome;
}  // namespace

// ---------------------------------------------------------------------------------------------------- start-up
TEST(P0aRtiServer, SU1_StartupIsFaultedServerStartedUntilAReset)
{
  Rig r;
  ASSERT_TRUE(r.plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, 3000ms));
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_SERVER_STARTED);
  EXPECT_EQ(r.st().job_id, 0u);
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_NONE);
  EXPECT_NE(r.st().server_session, 0u);
  EXPECT_EQ(r.st().interface_version, 1);
  EXPECT_EQ(r.st().export_table_id, testTable().id);
  std::this_thread::sleep_for(300ms);   // adopted
  auto a = r.plc->start(12, 1);
  EXPECT_EQ(a.outcome, O::REJECTED);
  EXPECT_EQ(a.code, StatusMsg::REJECT_FAULTED);
  const auto t0 = Clock::now();
  a = r.plc->reset();
  EXPECT_EQ(a.outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  ::testing::Test::RecordProperty("reset_to_idle_ms", std::to_string(msSince(t0)));
  EXPECT_EQ(r.st().fault_code, 0);
  EXPECT_TRUE(r.st().ready);
  EXPECT_TRUE(r.executor.started().empty());
}

// ---------------------------------------------------------------------------------------------------- normal
TEST(P0aRtiServer, N1_AValidStartIsAcceptedAtomicallyAndCompletes)
{
  Rig r;
  r.operational();
  const auto t0 = Clock::now();
  const auto a = r.plc->start(12, 41);
  ::testing::Test::RecordProperty("request_to_accepted_ms", std::to_string(msSince(t0)));
  ASSERT_EQ(a.outcome, O::ACCEPTED) << a.text;
  const auto first = r.firstAck(a.seq);
  ASSERT_TRUE(first);
  EXPECT_EQ(first->state, StatusMsg::STATE_RUNNING);   // ACCEPTED and RUNNING in the SAME image
  EXPECT_EQ(first->job_id, 41u);
  EXPECT_EQ(first->task, 12);
  EXPECT_EQ(first->task_name, "place_part");
  ASSERT_TRUE(r.waitResult(41));
  ::testing::Test::RecordProperty("request_to_result_ms", std::to_string(msSince(t0)));
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_SUCCEEDED);
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_OK);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  EXPECT_TRUE(r.st().ready);
  ASSERT_EQ(r.executor.started().size(), 1u);
  EXPECT_EQ(r.executor.started()[0].tree_id, "job_place_part");
  const auto v = r.plc->view(41);   // the simple PLC surface
  EXPECT_TRUE(v.done);
  EXPECT_FALSE(v.busy);
  EXPECT_TRUE(v.ready);
  EXPECT_FALSE(v.fault);
}

namespace
{
void expectOutcome(const char* outcome, JobEnd::Tree tree, bool eef, uint8_t result, uint16_t code, uint8_t state)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.tree = tree;
  s.outcome = outcome;
  s.eef_fault = eef;
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result, result) << outcome;
  EXPECT_EQ(r.st().result_code, code) << outcome;
  ASSERT_TRUE(r.waitState(state)) << outcome;
  EXPECT_EQ(r.st().fault_code, state == StatusMsg::STATE_FAULTED ? code : 0) << outcome;
}
}  // namespace

TEST(P0aRtiServer, N2_BinEmpty)
{
  expectOutcome("BIN_EMPTY", JobEnd::Tree::SUCCESS, false, StatusMsg::RESULT_SUCCEEDED, StatusMsg::CODE_BIN_EMPTY,
                StatusMsg::STATE_IDLE);
}
TEST(P0aRtiServer, N3_NothingPickable)
{
  expectOutcome("NOTHING_PICKABLE", JobEnd::Tree::SUCCESS, false, StatusMsg::RESULT_SUCCEEDED,
                StatusMsg::CODE_NOTHING_PICKABLE, StatusMsg::STATE_IDLE);
}
TEST(P0aRtiServer, F1a_TerminalFault)
{
  expectOutcome("TERMINAL_FAULT", JobEnd::Tree::FAILURE, false, StatusMsg::RESULT_FAILED,
                StatusMsg::CODE_TERMINAL_FAULT, StatusMsg::STATE_FAULTED);
}
TEST(P0aRtiServer, F1b_FailureWithoutACanonicalOutcome)
{
  expectOutcome("", JobEnd::Tree::FAILURE, false, StatusMsg::RESULT_FAILED, StatusMsg::CODE_TASK_FAILED,
                StatusMsg::STATE_FAULTED);
}
TEST(P0aRtiServer, F2a_PickableExhausted)
{
  expectOutcome("PICKABLE_EXHAUSTED", JobEnd::Tree::FAILURE, false, StatusMsg::RESULT_ENDED,
                StatusMsg::CODE_PICKABLE_EXHAUSTED, StatusMsg::STATE_IDLE);
}
TEST(P0aRtiServer, F2b_RecoveryExhausted)
{
  expectOutcome("RECOVERY_EXHAUSTED", JobEnd::Tree::FAILURE, false, StatusMsg::RESULT_ENDED,
                StatusMsg::CODE_RECOVERY_EXHAUSTED, StatusMsg::STATE_IDLE);
}
TEST(P0aRtiServer, F10_SuccessWithAnEndEffectorFaultLatched)
{
  expectOutcome("", JobEnd::Tree::SUCCESS, true, StatusMsg::RESULT_FAILED, StatusMsg::CODE_EEF_FAULT,
                StatusMsg::STATE_FAULTED);
}

// ---------------------------------------------------------------------------------------------------- identity
TEST(P0aRtiServer, I1_DuplicateJobIdWhileRunning)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 1500;
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 5).outcome, O::ACCEPTED);
  const auto a = r.plc->start(12, 5);
  EXPECT_EQ(a.outcome, O::REJECTED);
  EXPECT_EQ(a.code, StatusMsg::REJECT_DUPLICATE_JOB);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_RUNNING);   // unaffected
  ASSERT_TRUE(r.waitResult(5));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_OK);
  EXPECT_EQ(r.executor.started().size(), 1u);
}

TEST(P0aRtiServer, I2_I4_DuplicateOfAFinishedJobThenANewOne)
{
  Rig r;
  r.operational();
  ASSERT_EQ(r.plc->start(12, 5).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(5));
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  const auto a = r.plc->start(12, 5);
  EXPECT_EQ(a.code, StatusMsg::REJECT_DUPLICATE_JOB);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_IDLE);
  EXPECT_EQ(r.plc->start(12, 6).outcome, O::ACCEPTED);   // I4
}

TEST(P0aRtiServer, I3_StaleCancelWhileAnotherJobRuns)
{
  Rig r;
  r.executor.push({});
  ScriptedExecutor::Script s;
  s.run_ms = 1500;
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(r.plc->start(12, 2).outcome, O::ACCEPTED);
  const auto a = r.plc->cancel(1);
  EXPECT_EQ(a.code, StatusMsg::REJECT_NOT_CURRENT_JOB);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_RUNNING);
  ASSERT_TRUE(r.waitResult(2));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_OK);
}

TEST(P0aRtiServer, I5_JobIdZero)
{
  Rig r;
  r.operational();
  EXPECT_EQ(r.plc->start(12, 0).code, StatusMsg::REJECT_INVALID_REQUEST);
  EXPECT_EQ(r.plc->cancel(0).code, StatusMsg::REJECT_INVALID_REQUEST);
  EXPECT_TRUE(r.executor.started().empty());
}

TEST(P0aRtiServer, I6_TheSixtyFourJobHistoryBound)
{
  Rig r;
  for (int k = 0; k < 70; ++k)
  {
    ScriptedExecutor::Script s;
    s.run_ms = 1;
    r.executor.push(s);
  }
  r.operational();
  auto run = [&](uint32_t id) {
    ASSERT_EQ(r.plc->start(12, id).outcome, O::ACCEPTED) << id;
    ASSERT_TRUE(r.waitResult(id)) << id;
    ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE)) << id;
  };
  run(1);
  for (uint32_t id = 1001; id < 1064; ++id) run(id);   // 63 more: 64 jobs
  EXPECT_EQ(r.plc->start(12, 1).code, StatusMsg::REJECT_DUPLICATE_JOB);   // (a) 64 jobs earlier
  run(1064);                                                               // the 65th
  EXPECT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);                     // (b) 65 jobs earlier
}

// ---------------------------------------------------------------------------------------------------- validation
TEST(P0aRtiServer, V1_UnknownTask)
{
  Rig r;
  r.operational();
  const auto a = r.plc->start(99, 1);
  EXPECT_EQ(a.code, StatusMsg::REJECT_UNKNOWN_TASK);
  EXPECT_TRUE(r.executor.started().empty());
}

TEST(P0aRtiServer, V2_UnknownCommandCode)
{
  Rig r;
  r.operational();
  const auto a = r.plc->command(7, 1, 12, false);
  EXPECT_EQ(a.outcome, O::REJECTED);
  EXPECT_EQ(a.code, StatusMsg::REJECT_INVALID_REQUEST);
}

// ---------------------------------------------------------------------------------------------------- concurrency
TEST(P0aRtiServer, C1_StartWhileRunningAndWhileStopping)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 5000;
  s.stop_ms = 1500;
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  EXPECT_EQ(r.plc->start(12, 2).code, StatusMsg::REJECT_BUSY);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_RUNNING);   // the job unaffected
  EXPECT_EQ(r.st().job_id, 1u);
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_STOPPING));
  EXPECT_EQ(r.plc->start(12, 3).code, StatusMsg::REJECT_BUSY);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_STOPPING);  // the stop unaffected
  EXPECT_EQ(r.st().job_id, 1u);
  ASSERT_TRUE(r.waitResult(1));                        // job 1 completes its own way: the cancel's result
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  EXPECT_EQ(r.executor.started().size(), 1u);
}

TEST(P0aRtiServer, C2_StartWhileFaultedAndWhileUnavailable)
{
  Rig r;
  ASSERT_TRUE(r.plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, 3000ms));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(r.plc->start(12, 1).code, StatusMsg::REJECT_FAULTED);   // FAULTED (start-up)
  r.probe.up = false;
  std::this_thread::sleep_for(250ms);
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));   // a RESET with the runtime not ready
  EXPECT_EQ(r.plc->start(12, 2).code, StatusMsg::REJECT_NOT_READY);
}

// ---------------------------------------------------------------------------------------------------- control
TEST(P0aRtiServer, K1_CancelOfAJobNeverStarted)
{
  Rig r;
  r.operational();
  EXPECT_EQ(r.plc->cancel(77).code, StatusMsg::REJECT_NOT_CURRENT_JOB);
}

TEST(P0aRtiServer, K2_CancelInANonBlockingStepWithTheHeldStateKnownFalse)
{
  Rig r;
  ScriptedExecutor::Script first;
  first.holding_end = std::optional<bool>(false);   // the job begins with tool: open -> known false
  first.run_ms = 50;
  r.executor.push(first);
  ScriptedExecutor::Script dwell;
  dwell.run_ms = 10000;
  dwell.stop_ms = 10;   // a non-blocking step halts within one tick
  r.executor.push(dwell);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(r.plc->start(12, 2).outcome, O::ACCEPTED);
  std::this_thread::sleep_for(300ms);
  const auto t0 = Clock::now();
  const auto c = r.plc->cancel(2);
  ASSERT_EQ(c.outcome, O::ACCEPTED);
  EXPECT_EQ(r.firstAck(c.seq)->state, StatusMsg::STATE_STOPPING);
  ASSERT_TRUE(r.waitResult(2));
  const double latency = msSince(t0);
  ::testing::Test::RecordProperty("cancel_latency_ms", std::to_string(latency));
  EXPECT_LE(latency, 110.0) << "1 status period + 1 tick";
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_CANCELLED);
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
}

TEST(P0aRtiServer, K4a_CancelWhileHolding)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.holding_end = std::optional<bool>(true);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_FAILED);
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_WHILE_HOLDING);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_CANCELLED_WHILE_HOLDING);
}

TEST(P0aRtiServer, K4b_CancelInAProgramThatDoesNotTrackTheHeldState)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;   // the key is never set: unknown
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);
}

TEST(P0aRtiServer, K5_ResetFromFaulted)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.tree = JobEnd::Tree::FAILURE;
  s.outcome = "TERMINAL_FAULT";
  r.executor.push(s);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);   // (a)
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  EXPECT_EQ(r.st().fault_code, 0);
  ASSERT_EQ(r.plc->start(12, 2).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  r.probe.up = false;
  std::this_thread::sleep_for(250ms);
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);   // (b) the runtime not ready
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));
  EXPECT_EQ(r.st().fault_code, 0);
}

TEST(P0aRtiServer, K6_K7_ResetWhileRunningStoppingAndIdle)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.stop_ms = 1500;
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  const StatusMsg before = r.st();
  EXPECT_EQ(r.plc->reset().outcome, O::ACCEPTED);   // K7: IDLE, no change
  std::this_thread::sleep_for(250ms);
  const StatusMsg after = r.st();
  EXPECT_EQ(after.state, StatusMsg::STATE_IDLE);
  EXPECT_EQ(after.fault_code, before.fault_code);
  EXPECT_EQ(after.job_id, before.job_id);
  EXPECT_EQ(after.result, before.result);
  EXPECT_EQ(after.result_code, before.result_code);
  EXPECT_EQ(after.ready, before.ready);
  EXPECT_EQ(after.detail, before.detail);
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  EXPECT_EQ(r.plc->reset().code, StatusMsg::REJECT_INVALID_STATE);   // K6: RUNNING
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  EXPECT_EQ(r.plc->reset().code, StatusMsg::REJECT_INVALID_STATE);   // K6: STOPPING
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED);           // the stop unaffected
}

TEST(P0aRtiServer, K9_ASecondCancelWhileStopping)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.stop_ms = 1500;
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  EXPECT_EQ(r.plc->command(CommandMsg::CMD_CANCEL, 1, 0, false).code, StatusMsg::REJECT_INVALID_STATE);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED);
  EXPECT_GE(r.executor.stops(), 1);   // re-asserted every period while STOPPING (Decision 8): one job, one result
}

TEST(P0aRtiServer, K10_ResetAndCancelWhileUnavailable)
{
  Rig r;
  r.operational();
  r.probe.up = false;
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));
  EXPECT_EQ(r.plc->reset().outcome, O::ACCEPTED);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_UNAVAILABLE);
  EXPECT_EQ(r.plc->cancel(1).code, StatusMsg::REJECT_NOT_CURRENT_JOB);
}

TEST(P0aRtiServer, K11_ACancelThatComesTooLateKeepsTheJobsOwnResult)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.stop_ms = 50;
  s.complete_on_stop = true;
  s.outcome = "BIN_EMPTY";
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_SUCCEEDED);
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_BIN_EMPTY);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
}

TEST(P0aRtiServer, K12_TheHeldStateIsCarriedToTheNextJob)
{
  Rig r;
  ScriptedExecutor::Script pick;
  pick.run_ms = 50;
  pick.holding_end = std::optional<bool>(true);   // a pick task ends holding by design
  r.executor.push(pick);
  ScriptedExecutor::Script place;
  place.run_ms = 10000;                            // leaves the seeded state unchanged until the cancel
  r.executor.push(place);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_OK);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(r.plc->start(7, 2).outcome, O::ACCEPTED);
  ASSERT_EQ(r.executor.started().size(), 2u);
  ASSERT_TRUE(r.executor.started()[1].holding.has_value());
  EXPECT_TRUE(*r.executor.started()[1].holding);   // seeded HELD
  ASSERT_EQ(r.plc->cancel(2).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(2));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_WHILE_HOLDING);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
}

TEST(P0aRtiServer, K13_AfterAStartTheHeldStateIsUnknown)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  EXPECT_FALSE(r.executor.started()[0].holding.has_value());   // the RESET did not make it known
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
}

// ---------------------------------------------------------------------------------------------------- faults
TEST(P0aRtiServer, F3_RuntimeLostWhileIdleThenBack)
{
  Rig r;
  r.operational();
  r.probe.up = false;
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));
  EXPECT_EQ(r.plc->start(12, 1).code, StatusMsg::REJECT_NOT_READY);
  r.probe.up = true;
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));   // no RESET
}

TEST(P0aRtiServer, F5_TaskTimeoutOnTheSteadyClock)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  r.executor.push(s);
  r.operational();
  const auto t0 = Clock::now();
  ASSERT_EQ(r.plc->start(3, 1).outcome, O::ACCEPTED);   // quick: 1.0 s
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_STOPPING, 3000ms));
  const double fired = msSince(t0);
  ::testing::Test::RecordProperty("timeout_fired_ms", std::to_string(fired));
  EXPECT_GE(fired, 1000.0);
  EXPECT_LE(fired, 1250.0);
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_TASK_TIMEOUT);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
}

TEST(P0aRtiServer, F6_RequesterLostWhileIdleThenTheSameSessionResumes)
{
  Rig r;
  r.operational();
  const uint32_t session = r.plc->session();
  const auto t0 = Clock::now();
  r.plc->setHeartbeatPaused(true);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE, 3000ms));
  const double lost = msSince(t0);
  ::testing::Test::RecordProperty("requester_lost_ms", std::to_string(lost));
  EXPECT_LE(lost, 1000.0 + 100.0 + 150.0) << "1.0 s + 1 period (+ scheduling)";
  r.plc->setHeartbeatPaused(false);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));   // no re-adoption, no RESET
  EXPECT_EQ(r.plc->session(), session);
}

TEST(P0aRtiServer, F7_RequesterLostWhileRunning)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  r.plc->setHeartbeatPaused(true);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_STOPPING, 3000ms));
  ASSERT_TRUE(r.waitResult(1));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_REQUESTER_LOST);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  r.plc->setHeartbeatPaused(false);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
}

TEST(P0aRtiServer, F11_LossesWhileFaultedChangeNothing)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.tree = JobEnd::Tree::FAILURE;
  s.outcome = "TERMINAL_FAULT";
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED));
  // episode 1: the requester heartbeat lost, then resumed
  r.plc->setHeartbeatPaused(true);
  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_TERMINAL_FAULT);
  r.plc->setHeartbeatPaused(false);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_TERMINAL_FAULT);
  // episode 2: the runtime lost, then back
  r.probe.up = false;
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_TERMINAL_FAULT);
  r.probe.up = true;
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_TERMINAL_FAULT);
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
}

TEST(P0aRtiServer, F12_FirstCauseWins_CancelThenRequesterLossAndTimeout)
{
  Rig r;
  ScriptedExecutor::Script first;
  first.run_ms = 50;
  first.holding_end = std::optional<bool>(false);
  r.executor.push(first);
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.stop_ms = 2500;   // a long stop: the requester loss AND the 1 s timeout both happen during it
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(r.plc->start(3, 2).outcome, O::ACCEPTED);   // quick: 1.0 s timeout
  ASSERT_EQ(r.plc->cancel(2).outcome, O::ACCEPTED);
  r.plc->setHeartbeatPaused(true);
  ASSERT_TRUE(r.waitResult(2, 6000ms));
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_CANCELLED);   // not REQUESTER_LOST, not TASK_TIMEOUT
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED);
  EXPECT_NE(r.st().detail.find("REQUESTER_LOST"), std::string::npos) << r.st().detail;
  EXPECT_NE(r.st().detail.find("TASK_TIMEOUT"), std::string::npos) << r.st().detail;
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));  // CANCELLED -> IDLE -> UNAVAILABLE
}

// ---------------------------------------------------------------------------------------------------- protocol
TEST(P0aRtiServer, P1_ALateAcknowledgementIsNeverTakenForANewerCommand)
{
  Rig r(true, {}, /*with_relay=*/true);
  r.operational();
  ScriptedExecutor::Script s;
  s.run_ms = 6000;
  r.executor.push(s);
  r.relay->delayFor(300ms);    // the statuses of the next 0.3 s (the acknowledgement) arrive 1.5 s late, in order
  const uint32_t before = r.plc->lastSeq();
  const auto a = r.plc->start(12, 9);
  EXPECT_TRUE(a.retried);
  EXPECT_EQ(a.seq, nextSeq(nextSeq(before)));            // the retry: a NEW cmd_seq (the first + 1), the same job
  EXPECT_EQ(a.outcome, O::ALREADY_ACCEPTED) << a.text;   // the retry got DUPLICATE_JOB for its own job
  EXPECT_EQ(r.executor.started().size(), 1u);           // exactly one job
  EXPECT_EQ(r.plc->view(9).busy, true);
}

TEST(P0aRtiServer, P2_ALevelHeldImageIsProcessedOnce)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 100;
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  std::this_thread::sleep_for(1500ms);   // ~15 re-deliveries of the same image
  EXPECT_EQ(r.executor.started().size(), 1u);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_IDLE);
}

TEST(P0aRtiServer, P3_P10_SequenceErrorsAndTheWrap)
{
  Rig r;
  r.operational();
  const uint32_t skip = nextSeq(nextSeq(r.plc->lastSeq()));
  const auto a = r.plc->commandWithSeq(skip, CommandMsg::CMD_RESET, 0, 0);
  EXPECT_EQ(a.code, StatusMsg::REJECT_SEQUENCE_ERROR);
  EXPECT_EQ(r.st().ack_seq, skip);                       // P3: ack_seq resynchronised to the received seq
  const auto next = r.plc->reset();                      // the next in-order one
  EXPECT_EQ(next.seq, nextSeq(skip));
  EXPECT_EQ(next.outcome, O::ACCEPTED);
  // P10, IN ORDER across the wrap: resynchronised to 2^32-2, then 2^32-1 and 1 both accepted (0 skipped)
  const auto w = r.plc->commandWithSeq(UINT32_MAX - 1, CommandMsg::CMD_RESET, 0, 0);
  EXPECT_EQ(w.code, StatusMsg::REJECT_SEQUENCE_ERROR);
  EXPECT_EQ(r.st().ack_seq, UINT32_MAX - 1);
  const auto top = r.plc->reset();
  EXPECT_EQ(top.seq, UINT32_MAX);
  EXPECT_EQ(top.outcome, O::ACCEPTED);
  const auto after = r.plc->reset();
  EXPECT_EQ(after.seq, 1u);
  EXPECT_EQ(after.outcome, O::ACCEPTED);
}

TEST(P0aRtiServer, P4_NoServerStatusAtAll)
{
  Rig r(/*with_server=*/false);
  const auto t0 = Clock::now();
  const auto a = r.plc->start(12, 1);
  EXPECT_EQ(a.outcome, O::REFUSED_NO_SERVER);
  EXPECT_LE(msSince(t0), 1000.0);
  EXPECT_FALSE(r.plc->view(1).server_alive);
}

TEST(P0aRtiServer, P4b_TheServerGoesSilent)
{
  Rig r;
  r.operational();
  const auto t0 = Clock::now();
  r.server->shutdown();   // no more status
  while (r.plc->serverAlive() && msSince(t0) < 3000) std::this_thread::sleep_for(10ms);
  const double lost = msSince(t0);
  ::testing::Test::RecordProperty("server_lost_ms", std::to_string(lost));
  EXPECT_LE(lost, 1000.0 + 100.0 + 150.0);
  EXPECT_EQ(r.plc->start(12, 1).outcome, O::REFUSED_NO_SERVER);
}

TEST(P0aRtiServer, P5_AdoptionTakesTwoHeartbeatsAndNeverExecutesTheLevelHeldImage)
{
  Rig r(true, [] {
    ReferenceRequester::Options o;
    o.command_topic = "p0a/unused_command";   // this requester only listens; the test writes raw images
    return o;
  }());
  auto raw_node = std::make_shared<rclcpp::Node>("p0a_raw_plc");
  auto raw = raw_node->create_publisher<CommandMsg>(kCommandTopic, RobotTaskServer::qos());
  ASSERT_TRUE(r.plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, 3000ms));
  const auto deadline = Clock::now() + 3s;
  while (r.server_node->count_publishers("/" + std::string(kCommandTopic)) < 1 && Clock::now() < deadline)
    std::this_thread::sleep_for(10ms);
  CommandMsg img;
  img.requester_session = 4242;
  img.heartbeat = 1;
  img.cmd_seq = 3;
  img.cmd = CommandMsg::CMD_RESET;   // (c) a command left level-held in the image
  auto put = [&](const CommandMsg& m) {
    raw->publish(m);
    std::this_thread::sleep_for(150ms);
  };
  put(img);
  EXPECT_EQ(r.st().ack_code, StatusMsg::REJECT_NOT_SYNCHRONIZED);   // not adopted
  img.heartbeat = 2;
  put(img);                                                          // (a) one change
  img.cmd_seq = 4;
  put(img);
  EXPECT_EQ(r.st().ack_code, StatusMsg::REJECT_NOT_SYNCHRONIZED);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
  img.heartbeat = 3;
  put(img);                                                          // (b) two: adopted
  EXPECT_EQ(r.st().ack_seq, 4u);
  EXPECT_EQ(r.st().ack, StatusMsg::ACK_NONE);                        // the baseline
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);                 // (c) the RESET in the image never executed
  img.heartbeat = 4;
  img.cmd_seq = 5;
  put(img);                                                          // the next in-order command
  EXPECT_EQ(r.st().ack, StatusMsg::ACK_ACCEPTED);
  EXPECT_EQ(r.st().state, StatusMsg::STATE_IDLE);
}

TEST(P0aRtiServer, P7_ARestartedRequesterWithItsCountersResetIsReadopted)
{
  Rig r;
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitResult(1));
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  const uint32_t old_session = r.plc->session();
  r.newRequester();   // the old one ends first (its publisher leaves the graph); counters from the start
  ASSERT_NE(r.plc->session(), old_session);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE, 3000ms));
  const auto a = r.plc->start(12, 1);   // job id 1 again: a new session
  EXPECT_EQ(a.outcome, O::ACCEPTED) << a.text;
}

TEST(P0aRtiServer, P8_AVersionOrTableMismatchIsRefusedBeforeSending)
{
  ReferenceRequester::Options o;
  o.expected_table_id = testTable().id ^ 0x1u;
  Rig r(true, o);
  ASSERT_TRUE(r.plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, 3000ms));
  std::this_thread::sleep_for(300ms);
  const uint32_t seq = r.plc->lastSeq();
  const auto a = r.plc->reset();
  EXPECT_EQ(a.outcome, O::REFUSED_MISMATCH) << a.text;
  EXPECT_EQ(r.plc->lastSeq(), seq);   // nothing sent
  EXPECT_EQ(r.st().state, StatusMsg::STATE_FAULTED);
}

TEST(P0aRtiServer, P11_NoneWithANewSequenceNumber)
{
  Rig r;
  r.operational();
  const auto a = r.plc->command(CommandMsg::CMD_NONE, 0, 0, false);
  EXPECT_EQ(a.code, StatusMsg::REJECT_INVALID_REQUEST);
  EXPECT_EQ(r.st().ack_seq, a.seq);   // advanced
  EXPECT_TRUE(r.executor.started().empty());
}

TEST(P0aRtiServer, P12_ARequesterRestartDuringAJobIsRequesterLost)
{
  Rig r;
  ScriptedExecutor::Script s;
  s.run_ms = 10000;
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  const auto t0 = Clock::now();
  r.newRequester();   // a restart within 1.0 s: a new session
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED, 3000ms));
  ::testing::Test::RecordProperty("session_change_to_faulted_ms", std::to_string(msSince(t0)));
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_REQUESTER_LOST);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);   // the new session, adopted
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
}

// ---------------------------------------------------------------------------------------------------- events
TEST(P0aRtiServer, E3_ARejectedStartStartsNoRun)
{
  Rig r;
  r.operational();
  EXPECT_EQ(r.plc->start(99, 1).outcome, O::REJECTED);
  EXPECT_EQ(r.plc->start(12, 0).outcome, O::REJECTED);
  EXPECT_TRUE(r.executor.started().empty());   // no job, so no run and no event (R: test_p0a_task_server E3)
}

TEST(P0aRtiServer, P8b_AnInterfaceVersionMismatchIsRefusedBeforeSending)
{
  ReferenceRequester::Options o;   // a requester reading a server that speaks another RTI version
  o.status_topic = "p0a/v2_status";
  o.command_topic = "p0a/v2_command";
  o.expected_table_id = testTable().id;
  Rig r(/*with_server=*/false, o);
  auto pub = r.rec_node->create_publisher<StatusMsg>("p0a/v2_status", RobotTaskServer::qos());
  std::atomic<bool> go{true};
  std::thread server([&] {
    uint16_t hb = 0;
    while (go)
    {
      StatusMsg m;
      m.interface_version = 2;   // not 1
      m.export_table_id = testTable().id;
      m.server_session = 77;
      m.heartbeat = ++hb;
      m.state = StatusMsg::STATE_IDLE;
      m.ready = true;
      pub->publish(m);
      std::this_thread::sleep_for(50ms);
    }
  });
  ASSERT_TRUE(r.plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 3; }, 3000ms));
  const uint32_t seq = r.plc->lastSeq();
  const auto a = r.plc->reset();
  go = false;
  server.join();
  EXPECT_EQ(a.outcome, O::REFUSED_MISMATCH) << a.text;
  EXPECT_NE(a.text.find("RTI v2"), std::string::npos) << a.text;   // the mismatch is reported
  EXPECT_EQ(r.plc->lastSeq(), seq);                               // nothing sent
}

// ---------------------------------------------------------------------------------------------------- review fixes
// P0.a protocol review M2 (ADR-0032 Decision 8): STOPPING is bounded -- a stop whose tree does not return within
// rti_stop_timeout_ms is decided FAILED with the held state unknown, FAULTED; the server stays not ready until the tree
// has returned, then a RESET (already given) is enough.
TEST(P0aRtiServer, Review_M2_AStopThatNeverCompletesIsBounded)
{
  RobotTaskServer::Params p;
  p.stop_timeout = 1000ms;
  Rig r(true, {}, false, p);
  ScriptedExecutor::Script s;
  s.run_ms = 20000;
  s.stop_ms = 3000;   // the tree returns 3 s after the stop: 2 s after the bound
  s.holding_end = std::optional<bool>(false);
  r.executor.push(s);
  r.operational();
  ASSERT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  const auto t0 = Clock::now();
  ASSERT_EQ(r.plc->cancel(1).outcome, O::ACCEPTED);
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_FAULTED, 3000ms));
  const double bounded = msSince(t0);
  ::testing::Test::RecordProperty("stop_timeout_to_faulted_ms", std::to_string(bounded));
  EXPECT_GE(bounded, 1000.0);
  EXPECT_LE(bounded, 1000.0 + 100.0 + 150.0);
  EXPECT_EQ(r.st().result, StatusMsg::RESULT_FAILED);
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);   // the tree never said: unknown
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);
  const int stops = r.executor.stops();
  ASSERT_EQ(r.plc->reset().outcome, O::ACCEPTED);   // acknowledged; the tick thread is still busy
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_UNAVAILABLE));
  EXPECT_NE(r.st().detail.find("has not returned"), std::string::npos) << r.st().detail;
  EXPECT_EQ(r.plc->start(12, 2).code, StatusMsg::REJECT_NOT_READY);
  std::this_thread::sleep_for(300ms);
  EXPECT_GT(r.executor.stops(), stops);   // the stuck job is still stopped every period
  ASSERT_TRUE(r.waitState(StatusMsg::STATE_IDLE, 4000ms)) << r.st().detail;   // the tree returned: ready
  EXPECT_EQ(r.st().result_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN);       // its late end changed nothing
  ASSERT_EQ(r.plc->start(12, 3).outcome, O::ACCEPTED);
  ASSERT_EQ(r.executor.started().size(), 2u);
  EXPECT_FALSE(r.executor.started()[1].holding.has_value());   // the held state stayed unknown
}

// P0.a protocol review M3 (ADR-0032 Decision 10a rule 2): after a server RESTART the requester's very first command
// -- the RESET of "Robot needs reset" -- is accepted without any sleep: it waits to be adopted by the new session.
TEST(P0aRtiServer, Review_M3_AfterAServerRestartTheFirstResetIsAccepted)
{
  Rig r;
  r.operational();
  const uint32_t old_session = r.st().server_session;
  r.server->shutdown();   // the server process "dies": no status any more
  // ... and a NEW server process starts: its own node, executor, job executor and probe (a new server_session)
  auto node = std::make_shared<rclcpp::Node>("p0a_server_restarted");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  ScriptedExecutor jobs;
  ScriptedProbe probe;
  auto server = std::make_unique<RobotTaskServer>(node, group, RobotTaskServer::Params{}, testTable(), jobs, probe);
  jobs.report = [&server](uint64_t c, const JobEnd& e) { return server->jobEnded(c, e); };
  std::thread spin([&exec] { exec.spin(); });
  ASSERT_TRUE(r.plc->waitFor([old_session](const StatusMsg& s) { return s.server_session != old_session; }, 3000ms));
  EXPECT_EQ(r.st().fault_code, StatusMsg::CODE_SERVER_STARTED);
  const auto a = r.plc->reset();   // at once: the requester waits to be adopted by the new session (10a rule 2)
  EXPECT_EQ(a.outcome, O::ACCEPTED) << a.text;
  EXPECT_TRUE(r.waitState(StatusMsg::STATE_IDLE));
  EXPECT_EQ(r.plc->start(12, 1).outcome, O::ACCEPTED);
  EXPECT_TRUE(r.waitResult(1));
  jobs.finish();
  server->shutdown();
  exec.cancel();
  spin.join();
  jobs.report = nullptr;
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
