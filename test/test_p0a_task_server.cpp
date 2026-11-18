// test_p0a_task_server (P0.a, ADR-0032): the frozen matrix's R cases -- the REAL trainit_task_server (and, for B2,
// the real trainit_run_bt) as a CHILD PROCESS, the way a launch starts it, on the bundle TSA generates for
// test/data/p0a/served.yaml (BYTE COPIES in test/data/p0a/served/, sha256-pinned here and in TSA's
// test_p0a_task_export.py), against the M2.5 fake move_group and joint states and the M2.6 fake end-effector adapter,
// with the REFERENCE requester (the fake PLC's logic) in this process -- or the fake PLC executable as a child (F8).
// Domain 77, localhost only, graph-guarded. Each test is named after its case; measured timings are test properties.
//
// The fixture: place_part #12 (30 s: open the tool, PTP ready, dwell 2 s, PTP home), home #7 (5 s: open the tool,
// PTP home), pick_part #3 (10 s: PTP ready, close the tool -- it ENDS HOLDING), carry #4 (10 s: PTP ready, PTP home --
// no tool step: its motions read the SEEDED held state), inspect internal; faults: stop; a mock suction cup.
// place_part and home open the tool first: their held state is KNOWN and false before anything moves (the frozen
// preamble for the CANCELLED 200 cases); pick_part -> carry is the real-tree K12 / K13 pair.
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <trainit_interfaces/msg/execution_event.hpp>

#include "m2_5/capture.hpp"
#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "trainit_motion_runtime/reference_requester.hpp"
#include "trainit_motion_runtime/rti_contract.hpp"

namespace
{
using trainit::rti::ReferenceRequester;
using StatusMsg = trainit_interfaces::msg::RobotTaskStatus;
using CommandMsg = trainit_interfaces::msg::RobotTaskCommand;
using Event = trainit_interfaces::msg::ExecutionEvent;
using Clock = std::chrono::steady_clock;
using O = ReferenceRequester::Outcome;
using std::chrono::milliseconds;

const std::string kData = TRAINIT_P0A_DATA_DIR;
const std::string kServed = kData + "/served";
constexpr uint32_t kTableId = 3767158916u;   // TSA: 3:pick_part:10000 4:carry:10000 7:home:5000 12:place_part:30000
const std::map<std::string, std::string> kPinned = {   // the digests TSA's test_p0a_task_export.py pins too
  {"task_program.xml", "f864d0e3b046dec1c4be8f700b7d3c80e8e8db0d977b8e0623ee1c2b57247c1b"},
  {"tasks.yaml", "6f02aa4347736cfc0be8ed8a3de16d09bdfd004d3e6df63696e98fd7274e4e5e"},
  {"bt_params.yaml", "ee5f12cc8fe8988836d9e3dd909f1687a928907694bf2c059257d13b6b3d9716"},
  {"motion_profiles.yaml", "8b0b793ffc1d549259474fcbfa120cea3b19b53ec1abc9a237330f074c803750"},
};

std::string actualDir()
{
  return TRAINIT_P0A_ACTUAL_DIR;
}

double msSince(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

std::string readAll(const std::string& path)
{
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

// --- the harness: the fakes, a recorder, an optional in-process requester -------------------------------------------
class Harness
{
public:
  Harness()
  : mg(mlog, "/"),
    js("/"),
    contract(trainit::eef::parseContract(m26::after::loadParams(kServed + "/bt_params.yaml")))
  {
    ::mkdir(actualDir().c_str(), 0755);
    adapter = std::make_unique<m26::after::FakeEefAdapter>(elog, "/", *contract.device("main"),
                                                           m26::after::AdapterOptions{});
    node = std::make_shared<rclcpp::Node>("p0a_recorder", "/", rclcpp::NodeOptions().use_global_arguments(false));
    status_sub = node->create_subscription<StatusMsg>(
      trainit::rti::kStatusTopic, rclcpp::QoS(1).reliable().transient_local(), [this](StatusMsg::SharedPtr m) {
        std::lock_guard<std::mutex> l(m_);
        statuses.emplace_back(Clock::now(), *m);
      });
    event_sub = node->create_subscription<Event>("trainit/execution_events", rclcpp::QoS(100).reliable(),
                                                 [this](Event::SharedPtr e) {
                                                   std::lock_guard<std::mutex> l(m_);
                                                   events.push_back(*e);
                                                 });
    exec.add_node(node);
    spinner = std::thread([this] { exec.spin(); });
  }
  ~Harness()
  {
    plc.reset();
    if (plc_node) exec.remove_node(plc_node);
    exec.cancel();
    spinner.join();
    adapter.reset();
  }

  void newPlc()
  {
    dropPlc();
    static int n = 0;
    plc_node = std::make_shared<rclcpp::Node>("p0a_plc_" + std::to_string(++n), "/",
                                              rclcpp::NodeOptions().use_global_arguments(false));
    ReferenceRequester::Options o;
    o.expected_table_id = kTableId;
    plc = std::make_unique<ReferenceRequester>(plc_node, o);
    exec.add_node(plc_node);
  }
  void dropPlc()
  {
    if (!plc_node) return;
    exec.remove_node(plc_node);
    plc.reset();
    plc_node.reset();
  }
  int commandPublishers() { return static_cast<int>(node->count_publishers("/trainit/robot_task/command")); }

  // a fresh scenario: the fakes' scripts, the recordings, the adapter present
  void fresh()
  {
    mg.resetScenario();
    mg.stop_releases_held = false;
    mg.releaseHeld();
    mlog.reset();
    if (!adapter)
      adapter = std::make_unique<m26::after::FakeEefAdapter>(elog, "/", *contract.device("main"),
                                                             m26::after::AdapterOptions{});
    std::lock_guard<std::mutex> l(m_);
    statuses.clear();
    events.clear();
  }
  std::optional<StatusMsg> last()
  {
    std::lock_guard<std::mutex> l(m_);
    if (statuses.empty()) return std::nullopt;
    return statuses.back().second;
  }
  bool waitStatus(const std::function<bool(const StatusMsg&)>& pred, milliseconds t)
  {
    return m26::waitUntil([&] {
      const auto s = last();
      return s && pred(*s);
    }, t);
  }
  bool waitState(uint8_t state, milliseconds t = milliseconds(5000))
  {
    return waitStatus([state](const StatusMsg& s) { return s.state == state; }, t);
  }
  bool waitResult(uint32_t job, milliseconds t = milliseconds(10000))
  {
    return waitStatus([job](const StatusMsg& s) { return s.job_id == job && s.result != StatusMsg::RESULT_NONE; }, t);
  }
  std::vector<Event> eventsCopy()
  {
    std::lock_guard<std::mutex> l(m_);
    return events;
  }
  std::vector<std::pair<Clock::time_point, StatusMsg>> statusCopy()
  {
    std::lock_guard<std::mutex> l(m_);
    return statuses;
  }
  bool waitEvent(const std::function<bool(const Event&)>& pred, milliseconds t)
  {
    return m26::waitUntil([&] {
      for (const auto& e : eventsCopy())
        if (pred(e)) return true;
      return false;
    }, t);
  }
  std::size_t executes() { return mlog.count("/execute_trajectory"); }
  // the ARRIVAL time of the first recorded status satisfying `pred` (a short-lived state -- STOPPING ends as soon as
  // the stopped motion returns -- is never missed: the recorder keeps every status), waiting up to `t`
  std::optional<Clock::time_point> firstSeen(const std::function<bool(const StatusMsg&)>& pred, milliseconds t)
  {
    std::optional<Clock::time_point> at;
    m26::waitUntil([&] {
      for (const auto& [when, m] : statusCopy())
        if (pred(m))
        {
          at = when;
          return true;
        }
      return false;
    }, t);
    return at;
  }
  std::optional<Clock::time_point> sawState(uint8_t state, milliseconds t = milliseconds(5000))
  {
    return firstSeen([state](const StatusMsg& s) { return s.state == state; }, t);
  }

  m25::Log mlog;
  m25::FakeMoveGroup mg;
  m25::FakeJointStates js;
  m26::Log elog;
  trainit::eef::Contract contract;
  std::unique_ptr<m26::after::FakeEefAdapter> adapter;
  rclcpp::Node::SharedPtr node, plc_node;
  std::unique_ptr<ReferenceRequester> plc;

private:
  std::mutex m_;
  std::vector<std::pair<Clock::time_point, StatusMsg>> statuses;
  std::vector<Event> events;
  rclcpp::Subscription<StatusMsg>::SharedPtr status_sub;
  rclcpp::Subscription<Event>::SharedPtr event_sub;
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 3};
  std::thread spinner;
};

Harness& H()
{
  static std::unique_ptr<Harness> h;   // test harness: one per process
  if (!h)
  {
    h = std::make_unique<Harness>();
    m26::teardowns().push_back([] { h.reset(); });
  }
  return *h;
}

// --- the children -------------------------------------------------------------------------------------------------
std::string harnessParams(const std::string& id, const std::string& tree, const std::string& tasks,
                          const std::string& extra)
{
  const std::string path = actualDir() + "/" + id + ".yaml";
  std::ostringstream y;
  y << "# P0.a " << id << ": the harness keys over the bundle's bt_params (test_p0a_task_server)\n"
    << "/**:\n  ros__parameters:\n"
    << "    bt_tree_file: \"" << tree << "\"\n"
    << "    tasks_file: \"" << tasks << "\"\n"
    << "    motion_profiles_file: \"" << kServed << "/motion_profiles.yaml\"\n"
    << "    groot_monitor: false\n"
    << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
    << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6) << extra;
  m26::writeFile(path, y.str());
  return path;
}

std::unique_ptr<m26::ChildProcess> startChild(const char* exe, const std::string& id, const std::string& tree,
                                              const std::string& tasks, const std::string& extra = "",
                                              const std::vector<std::string>& ros_args = {})
{
  std::vector<std::string> args{"--ros-args", "--params-file", kServed + "/bt_params.yaml", "--params-file",
                                harnessParams(id, tree, tasks, extra)};
  args.insert(args.end(), ros_args.begin(), ros_args.end());
  auto child = std::make_unique<m26::ChildProcess>(exe, args, m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()),
                                                   actualDir() + "/" + id + ".log");
  EXPECT_TRUE(child->started()) << id;
  return child;
}

std::unique_ptr<m26::ChildProcess> startServer(const std::string& id, const std::string& extra = "",
                                               const std::string& tree = kServed + "/task_program.xml",
                                               const std::string& tasks = kServed + "/tasks.yaml",
                                               const std::vector<std::string>& ros_args = {})
{
  return startChild(TRAINIT_TASK_SERVER_EXE, id, tree, tasks, extra, ros_args);
}

// served: the first status with a heartbeat
bool serving(milliseconds t = milliseconds(40000))
{
  return H().waitStatus([](const StatusMsg& s) { return s.heartbeat > 0 && s.server_session != 0; }, t);
}

// the in-process requester adopted, the start-up RESET accepted, IDLE
void operational()
{
  ASSERT_TRUE(serving()) << "no server status";
  if (!H().plc) H().newPlc();
  ASSERT_TRUE(H().plc->waitFor([](const StatusMsg& s) { return s.heartbeat > 0; }, milliseconds(5000)));
  const auto r = H().plc->reset();
  ASSERT_EQ(r.outcome, O::ACCEPTED) << r.text;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE)) << H().last()->detail;
}

void stopChild(std::unique_ptr<m26::ChildProcess>& child)
{
  if (child) child->stop(milliseconds(15000));
  child.reset();
}

struct Scope   // every test: a fresh scenario, the requester dropped at the end
{
  explicit Scope(const char* id, int budget_s = 240)
  {
    H().fresh();
    H().dropPlc();
    m26::watchdog().arm(id, std::chrono::seconds(budget_s));
  }
  ~Scope()
  {
    m26::watchdog().disarm();
    H().dropPlc();
    H().mg.releaseHeld();
  }
};

void record(const char* key, double v)
{
  ::testing::Test::RecordProperty(key, std::to_string(v));
}

std::vector<std::string> projection(const std::vector<Event>& events, const std::string& run)
{
  std::vector<std::string> out;
  for (const auto& e : events)
  {
    if (e.run_id != run || e.kind == Event::RUN_STARTED || e.kind == Event::RUN_ENDED) continue;
    std::ostringstream os;
    os << int(e.kind) << "|" << e.task_id << "|" << e.step_id << "|" << e.step_kind << "|" << int(e.status);
    out.push_back(os.str());
  }
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------------------------------- the fixture
TEST(P0aTaskServer, TheServedFixtureIsThePinnedTsaOutput)
{
  for (const auto& [name, digest] : kPinned) EXPECT_EQ(m25::sha256(readAll(kServed + "/" + name)), digest) << name;
}

// ---------------------------------------------------------------------------------------------------- start-up, normal
TEST(P0aTaskServer, SU1_N1_X5_StartupThenAJobOnTheRealServer)
{
  Scope s("SU1_N1");
  auto server = startServer("su1_n1");
  ASSERT_TRUE(serving());
  auto st = *H().last();
  EXPECT_EQ(st.state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(st.fault_code, StatusMsg::CODE_SERVER_STARTED);
  EXPECT_EQ(st.job_id, 0u);
  EXPECT_EQ(st.export_table_id, kTableId);   // X5: the server's own computation = TSA's tasks.yaml value
  H().newPlc();
  std::this_thread::sleep_for(milliseconds(400));
  EXPECT_EQ(H().plc->start(12, 1).code, StatusMsg::REJECT_FAULTED);
  ASSERT_EQ(H().plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  EXPECT_EQ(H().last()->fault_code, 0);   // SU1: the RESET cleared SERVER_STARTED
  EXPECT_TRUE(H().last()->ready);
  // N1
  const auto t0 = Clock::now();
  const auto a = H().plc->start(12, 41);
  record("request_to_accepted_ms", msSince(t0));
  ASSERT_EQ(a.outcome, O::ACCEPTED) << a.text;
  bool atomic = false;
  for (const auto& [t, m] : H().statusCopy())
    if (m.ack_seq == a.seq && m.ack == StatusMsg::ACK_ACCEPTED)
    {
      atomic = m.state == StatusMsg::STATE_RUNNING && m.job_id == 41u;
      break;
    }
  EXPECT_TRUE(atomic) << "ACCEPTED and RUNNING in the same image";
  ASSERT_TRUE(H().waitResult(41, milliseconds(15000)));
  record("request_to_result_ms", msSince(t0));
  st = *H().last();
  EXPECT_EQ(st.result, StatusMsg::RESULT_SUCCEEDED);
  EXPECT_EQ(st.result_code, StatusMsg::CODE_OK);
  EXPECT_EQ(st.task_name, "place_part");
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  EXPECT_TRUE(H().last()->ready);   // N1: ready again
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- control
TEST(P0aTaskServer, K2_E2_CancelInTheDwellAndItsEvents)
{
  Scope s("K2_E2");
  auto server = startServer("k2");
  operational();
  ASSERT_EQ(H().plc->start(12, 2).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitEvent([](const Event& e) { return e.kind == Event::STEP_ENTERED && e.step_id == "settle"; },
                            milliseconds(10000)));
  std::this_thread::sleep_for(milliseconds(200));
  const auto t0 = Clock::now();
  ASSERT_EQ(H().plc->cancel(2).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitResult(2));
  const double latency = msSince(t0);
  record("cancel_latency_ms", latency);
  EXPECT_LE(latency, 110.0) << "1 status period + 1 tick";
  auto st = *H().last();
  EXPECT_EQ(st.result, StatusMsg::RESULT_CANCELLED);
  EXPECT_EQ(st.result_code, StatusMsg::CODE_CANCELLED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  // E2: HALTED exits innermost first, then RUN_ENDED HALTED carrying 200
  ASSERT_TRUE(H().waitEvent([](const Event& e) { return e.kind == Event::RUN_ENDED && e.job_id == 2u; },
                            milliseconds(3000)));
  const auto ev = H().eventsCopy();
  std::vector<Event> run;
  for (const auto& e : ev)
    if (e.job_id == 2u) run.push_back(e);
  ASSERT_GE(run.size(), 3u);
  const auto& end = run.back();
  const auto& task = run[run.size() - 2];
  const auto& step = run[run.size() - 3];
  EXPECT_EQ(step.kind, Event::STEP_EXITED);
  EXPECT_EQ(step.step_id, "settle");
  EXPECT_EQ(step.status, Event::HALTED);
  EXPECT_EQ(task.kind, Event::TASK_EXITED);
  EXPECT_EQ(task.task_id, "place_part");
  EXPECT_EQ(task.status, Event::HALTED);
  EXPECT_EQ(task.result_code, StatusMsg::CODE_CANCELLED);
  EXPECT_EQ(end.kind, Event::RUN_ENDED);
  EXPECT_EQ(end.status, Event::HALTED);
  EXPECT_EQ(end.result_code, StatusMsg::CODE_CANCELLED);
  stopChild(server);
}

TEST(P0aTaskServer, K3_K8_CancelInABlockingMotionTwentyTimes)
{
  Scope s("K3_K8", 600);
  auto server = startServer("k3");
  operational();
  H().mg.stop_releases_held = true;   // a real move_group ends an execution its "stop" event names
  std::vector<double> latencies;
  for (uint32_t k = 0; k < 20; ++k)
  {
    const uint32_t job = 100 + k;
    H().mg.resetScenario();
    H().mg.ex_script.push_back(m25::Reply::hold());   // to_ready's execution never ends by itself
    const std::size_t before = H().executes();
    ASSERT_EQ(H().plc->start(12, job).outcome, O::ACCEPTED) << job;
    ASSERT_TRUE(m26::waitUntil([&] { return H().executes() > before && H().mg.heldCount() > 0; },
                               milliseconds(15000))) << "job " << job << ": the motion is not in flight";
    const auto t0 = Clock::now();
    ASSERT_EQ(H().plc->cancel(job).outcome, O::ACCEPTED) << job;
    ASSERT_TRUE(H().waitResult(job, milliseconds(10000))) << job;
    latencies.push_back(msSince(t0));
    EXPECT_TRUE(H().firstSeen([job](const StatusMsg& m) { return m.state == StatusMsg::STATE_STOPPING && m.job_id == job; },
                              milliseconds(200)))
      << job << ": STOPPING never published";
    const auto st = *H().last();
    EXPECT_EQ(st.result, StatusMsg::RESULT_CANCELLED) << job << " " << st.detail;
    EXPECT_EQ(st.result_code, StatusMsg::CODE_CANCELLED) << job << " " << st.detail;   // K8: never TERMINAL_FAULT
    // K8: the stopped motion's FAILURE (the fault gate's reason) is masked by the cancel and goes to `detail`
    EXPECT_NE(st.detail.find("MOTION_FAULT"), std::string::npos) << job << " " << st.detail;
    if (k == 0) ::testing::Test::RecordProperty("k8_detail", st.detail);
    ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE)) << job << " " << H().last()->detail;
  }
  EXPECT_TRUE(server->running()) << "K3: the server survived 20 cancels"; 
  std::sort(latencies.begin(), latencies.end());
  record("k3_cancel_latency_min_ms", latencies.front());
  record("k3_cancel_latency_median_ms", latencies[latencies.size() / 2]);
  record("k3_cancel_latency_max_ms", latencies.back());
  EXPECT_GE(H().mg.stops_heard(), 20);   // MotionRuntime::stop reached move_group every time
  // K3b (a regression found by P6(c), 2026-11-18): the CANCEL arrives BEFORE the motion is in flight (the tick is in
  // the gripper step or planning): the stop is re-asserted every period while STOPPING, so the motion that starts
  // afterwards is stopped within one period -- never run to its end (here it would never end: the fake holds it)
  H().mg.resetScenario();
  H().mg.ex_script.push_back(m25::Reply::hold());
  ASSERT_EQ(H().plc->start(12, 200).outcome, O::ACCEPTED);
  const auto t0 = Clock::now();
  ASSERT_EQ(H().plc->cancel(200).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitResult(200, milliseconds(5000))) << "the motion started after the stop was never stopped";
  record("k3b_cancel_before_motion_ms", msSince(t0));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_CANCELLED) << H().last()->detail;
  stopChild(server);
}

// K12 / K13 on the REAL tree (A7 end to end): pick_part (#3) ends holding the part by design -- its CloseGripper's
// post-condition writes trainit__holding := true -- and the server hands that held state to the next job, carry (#4),
// which never touches the tool: cancelled during its motion it is CANCELLED_WHILE_HOLDING 307. After a server start
// the held state is unknown: the first carry cancelled is CANCELLED_HOLD_UNKNOWN 309.
TEST(P0aTaskServer, K12_K13_TheHeldStateOnTheRealTree)
{
  Scope s("K12_K13", 300);
  auto server = startServer("k12");
  operational();
  H().mg.stop_releases_held = true;
  auto cancelCarry = [](uint32_t job) {
    H().mg.resetScenario();
    H().mg.ex_script.push_back(m25::Reply::hold());   // carry's to_ready never ends by itself
    const std::size_t before = H().executes();
    ASSERT_EQ(H().plc->start(4, job).outcome, O::ACCEPTED) << job;
    ASSERT_TRUE(m26::waitUntil([&] { return H().executes() > before && H().mg.heldCount() > 0; },
                               milliseconds(15000))) << job << ": the motion is not in flight";
    ASSERT_EQ(H().plc->cancel(job).outcome, O::ACCEPTED) << job;
    ASSERT_TRUE(H().waitResult(job, milliseconds(10000))) << job;
  };
  // K13: the first job after the start-up RESET -- the held state unknown
  cancelCarry(1);
  EXPECT_EQ(H().last()->result, StatusMsg::RESULT_FAILED) << H().last()->detail;
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_CANCELLED_HOLD_UNKNOWN) << H().last()->detail;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  ASSERT_EQ(H().plc->reset().outcome, O::ACCEPTED);   // a RESET never makes the held state known
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  // K12: pick_part ends holding (SUCCEEDED), carry inherits it
  H().mg.resetScenario();
  ASSERT_EQ(H().plc->start(3, 2).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitResult(2, milliseconds(15000)));
  EXPECT_EQ(H().last()->result, StatusMsg::RESULT_SUCCEEDED) << H().last()->detail;
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_OK) << H().last()->detail;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  cancelCarry(3);
  EXPECT_EQ(H().last()->result, StatusMsg::RESULT_FAILED) << H().last()->detail;
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_CANCELLED_WHILE_HOLDING) << H().last()->detail;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  EXPECT_EQ(H().last()->fault_code, StatusMsg::CODE_CANCELLED_WHILE_HOLDING);
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- faults
TEST(P0aTaskServer, F1b_AMotionFailureIsTaskFailed)
{
  Scope s("F1b");
  auto server = startServer("f1b");
  operational();
  H().mg.ex_script.push_back(m25::Reply::abort(m25::ErrorCodes::CONTROL_FAILED));   // to_ready fails: a fault
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitResult(1));
  const auto st = *H().last();
  EXPECT_EQ(st.result, StatusMsg::RESULT_FAILED);
  EXPECT_EQ(st.result_code, StatusMsg::CODE_TASK_FAILED) << st.detail;   // no canonical outcome: fail closed
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  EXPECT_EQ(H().last()->fault_code, StatusMsg::CODE_TASK_FAILED);
  stopChild(server);
}

TEST(P0aTaskServer, F1a_N2_F2_TheRealExecutorReadsTheCanonicalOutcome)
{
  Scope s("F1a");
  // hand-written job trees, in TSA's job_<task> shape, that end on a canonical outcome
  const std::string dir = actualDir() + "/outcomes";
  ::mkdir(dir.c_str(), 0755);
  m26::writeFile(dir + "/tree.xml",
                 "<?xml version=\"1.0\"?>\n<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
                 "  <BehaviorTree ID=\"MainTree\"><AlwaysSuccess/></BehaviorTree>\n"
                 "  <BehaviorTree ID=\"job_terminal\"><Sequence><Script code=\"trainit__outcome := 'TERMINAL_FAULT'\"/>"
                 "<AlwaysFailure/></Sequence></BehaviorTree>\n"
                 "  <BehaviorTree ID=\"job_empty\"><Sequence><Script code=\"trainit__outcome := 'BIN_EMPTY'\"/>"
                 "<AlwaysSuccess/></Sequence></BehaviorTree>\n"
                 "  <BehaviorTree ID=\"job_exhausted\"><Sequence><Script code=\"trainit__outcome := 'PICKABLE_EXHAUSTED'\"/>"
                 "<AlwaysFailure/></Sequence></BehaviorTree>\n</root>\n");
  std::vector<trainit::rti::ExportEntry> table;
  for (auto [n, name] : std::vector<std::pair<uint16_t, std::string>>{{1, "terminal"}, {2, "empty"}, {3, "exhausted"}})
  {
    trainit::rti::ExportEntry e;
    e.number = n;
    e.name = name;
    e.timeout_s = 10;
    e.timeout_ms = 10000;
    table.push_back(e);
  }
  std::ostringstream t;
  t << "schema_version: 1\nmain_task: terminal\nexport_table_id: " << trainit::rti::exportTableId(table) << "\ntasks:\n";
  for (const auto& e : table)
    t << "  " << e.name << ":\n    tree: task_" << e.name << "\n    inputs: []\n    outputs: []\n    export: {number: "
      << e.number << ", timeout_s: 10}\n";
  m26::writeFile(dir + "/tasks.yaml", t.str());
  auto server = startServer("f1a", "", dir + "/tree.xml", dir + "/tasks.yaml");
  ASSERT_TRUE(serving());
  ReferenceRequester::Options o;   // a requester generated for THIS table, not the served one
  o.expected_table_id = trainit::rti::exportTableId(table);
  auto plc_node = std::make_shared<rclcpp::Node>("p0a_plc_outcomes", "/", rclcpp::NodeOptions().use_global_arguments(false));
  auto plc = std::make_unique<ReferenceRequester>(plc_node, o);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(plc_node);
  std::thread spin([&ex] { ex.spin(); });
  ASSERT_EQ(plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(plc->start(2, 1).outcome, O::ACCEPTED);   // N2
  ASSERT_TRUE(H().waitResult(1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_BIN_EMPTY);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(plc->start(3, 2).outcome, O::ACCEPTED);   // F2
  ASSERT_TRUE(H().waitResult(2));
  EXPECT_EQ(H().last()->result, StatusMsg::RESULT_ENDED);
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_PICKABLE_EXHAUSTED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  ASSERT_EQ(plc->start(1, 3).outcome, O::ACCEPTED);   // F1(a)
  ASSERT_TRUE(H().waitResult(3));
  EXPECT_EQ(H().last()->result, StatusMsg::RESULT_FAILED);
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_TERMINAL_FAULT);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  ex.cancel();
  spin.join();
  plc.reset();
  plc_node.reset();
  stopChild(server);
}

TEST(P0aTaskServer, F4_TheRuntimeLostWhileRunning)
{
  Scope s("F4");
  auto server = startServer("f4");
  operational();
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(m26::waitUntil([&] { return H().mg.heldCount() > 0; }, milliseconds(15000)));
  H().adapter.reset();   // the end-effector device goes silent: its state stale after 1 s
  ASSERT_TRUE(H().sawState(StatusMsg::STATE_STOPPING, milliseconds(5000)));
  ASSERT_TRUE(H().waitResult(1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_RUNTIME_LOST) << H().last()->detail;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  EXPECT_EQ(H().last()->fault_code, StatusMsg::CODE_RUNTIME_LOST);
  stopChild(server);
}

TEST(P0aTaskServer, F5_TheTaskTimeoutOnTheSteadyClock)
{
  Scope s("F5");
  auto server = startServer("f5");
  operational();
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());   // home's motion never ends by itself
  const auto t0 = Clock::now();
  ASSERT_EQ(H().plc->start(7, 1).outcome, O::ACCEPTED);
  const auto stopping = H().sawState(StatusMsg::STATE_STOPPING, milliseconds(8000));
  ASSERT_TRUE(stopping);
  const double fired = std::chrono::duration<double, std::milli>(*stopping - t0).count();
  record("timeout_fired_ms", fired);
  EXPECT_GE(fired, 5000.0);
  EXPECT_LE(fired, 5250.0);
  ASSERT_TRUE(H().waitResult(1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_TASK_TIMEOUT);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  stopChild(server);
}

TEST(P0aTaskServer, F8_TheRequesterProcessIsKilledWhileRunning)
{
  Scope s("F8", 300);
  auto server = startServer("f8");
  ASSERT_TRUE(serving());
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());
  auto plc = std::make_unique<m26::ChildProcess>(
    TRAINIT_FAKE_PLC_EXE,
    std::vector<std::string>{"--ros-args", "-p", "tasks_file:=" + kServed + "/tasks.yaml", "-p",
                             "script:=sleep 300; reset; sleep 300; start place_part 1", "-p", "stay_on_eof:=true"},
    m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), actualDir() + "/f8_plc.log");
  ASSERT_TRUE(H().waitStatus([](const StatusMsg& m) { return m.state == StatusMsg::STATE_RUNNING && m.job_id == 1u; },
                             milliseconds(20000))) << readAll(actualDir() + "/f8_plc.log");
  const auto t0 = Clock::now();
  plc->signal(SIGKILL);
  plc->wait(milliseconds(5000));
  const auto stopping = H().sawState(StatusMsg::STATE_STOPPING, milliseconds(5000));
  ASSERT_TRUE(stopping);
  record("requester_killed_to_stopping_ms", std::chrono::duration<double, std::milli>(*stopping - t0).count());
  ASSERT_TRUE(H().waitResult(1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_REQUESTER_LOST);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  // the killed participant leaves the graph after its lease: a restarted PLC is then the ONE requester
  const auto g0 = Clock::now();
  ASSERT_TRUE(m26::waitUntil([] { return H().commandPublishers() == 0; }, milliseconds(60000)));
  record("killed_publisher_left_the_graph_ms", msSince(g0));
  auto again = std::make_unique<m26::ChildProcess>(
    TRAINIT_FAKE_PLC_EXE,
    std::vector<std::string>{"--ros-args", "-p", "tasks_file:=" + kServed + "/tasks.yaml", "-p",
                             "script:=sleep 300; reset", "-p", "stay_on_eof:=true"},
    m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), actualDir() + "/f8_plc2.log");
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE, milliseconds(15000))) << H().last()->detail;
  again->stop(milliseconds(5000));
  stopChild(server);
}

TEST(P0aTaskServer, F9_TheServerIsKilledDuringAJobThenRestarted)
{
  Scope s("F9", 300);
  auto server = startServer("f9");
  operational();
  const uint32_t old_session = H().last()->server_session;
  H().mg.ex_script.push_back(m25::Reply::hold());   // the trajectory in flight at the controller
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(m26::waitUntil([&] { return H().mg.heldCount() > 0; }, milliseconds(15000)));
  const auto t0 = Clock::now();
  server->signal(SIGKILL);
  server->wait(milliseconds(5000));
  ASSERT_TRUE(m26::waitUntil([] { return !H().plc->serverAlive(); }, milliseconds(3000)));
  const double lost = msSince(t0);
  record("server_killed_to_requester_lost_ms", lost);
  EXPECT_LE(lost, 1000.0 + 100.0 + 150.0);
  // A2, measured: the dead client's goal is NOT cancelled -- the controller keeps executing it
  std::this_thread::sleep_for(milliseconds(1500));
  record("held_goals_after_server_death", static_cast<double>(H().mg.heldCount()));
  EXPECT_EQ(H().mg.heldCount(), 1u) << "the trajectory handed to the controller kept running";
  const std::size_t executes = H().executes();
  server = startServer("f9_restart");
  ASSERT_TRUE(H().waitStatus([](const StatusMsg& m) { return m.fault_code == StatusMsg::CODE_SERVER_STARTED; },
                             milliseconds(40000)));
  std::this_thread::sleep_for(milliseconds(1500));   // the requester re-adopted; its image still holds START job 1
  const auto st = *H().last();
  EXPECT_NE(st.server_session, old_session);         // a new server_session
  EXPECT_EQ(st.state, StatusMsg::STATE_FAULTED);
  EXPECT_EQ(st.job_id, 0u);                          // no job: never resumed
  EXPECT_EQ(st.result, StatusMsg::RESULT_NONE);
  EXPECT_EQ(H().executes(), executes);               // the command left in the image was NOT executed
  EXPECT_TRUE(H().plc->view(1).job_lost);            // JOB LOST, reported to the requester
  H().mg.releaseHeld();
  ASSERT_EQ(H().plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- validation
TEST(P0aTaskServer, V3_V5_V6_RefusedBeforeServing)
{
  Scope s("V3_V5_V6", 300);
  const std::string dir = actualDir() + "/refused";
  ::mkdir(dir.c_str(), 0755);
  // V3: an exported task whose job tree is not in the tree file
  trainit::rti::ExportEntry e;
  e.number = 9;
  e.name = "missing";
  e.timeout_s = 5;
  e.timeout_ms = 5000;
  m26::writeFile(dir + "/v3_tasks.yaml",
                 "schema_version: 1\nmain_task: missing\nexport_table_id: " + std::to_string(trainit::rti::exportTableId({e})) +
                   "\ntasks:\n  missing:\n    tree: task_missing\n    inputs: []\n    outputs: []\n    export: {number: 9, timeout_s: 5}\n");
  // V3 (review): a tree file that is not well-formed XML
  m26::writeFile(dir + "/v3b_tree.xml", "<root BTCPP_format=\"4\">\n  <BehaviorTree ID=\"job_home\">\n");
  // V6: the served table with another id
  std::string tasks = readAll(kServed + "/tasks.yaml");
  tasks.replace(tasks.find(std::to_string(kTableId)), std::to_string(kTableId).size(), "12345");
  m26::writeFile(dir + "/v6_tasks.yaml", tasks);
  struct Case
  {
    const char* id;
    std::string extra, tree, tasks;
    std::vector<std::string> ros_args;
    std::string says;   // the refusal names its reason (the log)
  };
  const std::string tree = kServed + "/task_program.xml", served = kServed + "/tasks.yaml";
  const std::vector<Case> cases{
    {"v3", "", tree, dir + "/v3_tasks.yaml", {}, "its job tree job_missing does not load"},
    {"v3b", "", dir + "/v3b_tree.xml", served, {}, "failed to load BT file"},
    {"v5", "    rti_status_period_ms: 20\n", tree, served, {}, "invalid rti_status_period_ms 20 (bounds 50..10000 ms)"},
    {"v5b", "    rti_status_period_ms: 10001\n", tree, served, {}, "invalid rti_status_period_ms 10001"},
    {"v5c", "    rti_requester_timeout_ms: 150\n", tree, served, {},
     "invalid rti_requester_timeout_ms 150 (bounds 50..10000 ms, and at least two status periods)"},
    {"v5d", "    rti_runtime_state_max_age_ms: 10001\n", tree, served, {}, "invalid rti_runtime_state_max_age_ms 10001"},
    {"v6", "", tree, dir + "/v6_tasks.yaml", {}, "export_table_id 12345 differs from the table's own"},
    // ADR-0023 Decision 5 (review: architecture, finding 1): the RTI's command and status stay in the robot's slot
    {"ns_escape", "", tree, served,
     {"-r", "__ns:=/r1", "-r", "trainit/robot_task/command:=/r2/trainit/robot_task/command"},
     "topic /r1/trainit/robot_task/command -> /r2/trainit/robot_task/command"},
    {"ns_escape_status", "", tree, served,
     {"-r", "__ns:=/r1", "-r", "trainit/robot_task/status:=/trainit/robot_task/status"},
     "topic /r1/trainit/robot_task/status -> /trainit/robot_task/status"}};
  for (const auto& c : cases)
  {
    H().fresh();
    auto child = startServer(c.id, c.extra, c.tree, c.tasks, c.ros_args);
    ASSERT_TRUE(child->wait(milliseconds(60000))) << c.id;
    const std::string log = readAll(child->output());
    EXPECT_EQ(child->exitText(), "exit code 2") << c.id << "\n" << log;
    EXPECT_NE(log.find(c.says), std::string::npos) << c.id << ": the refusal does not say '" << c.says << "'\n" << log;
    EXPECT_FALSE(H().last().has_value()) << c.id << ": a status was published";
  }
}

// ---------------------------------------------------------------------------------------------------- protocol
TEST(P0aTaskServer, P6_TwoRequesters)
{
  Scope s("P6", 300);
  auto server = startServer("p6");
  operational();
  // (a) IDLE: a second requester appears
  auto second_node = std::make_shared<rclcpp::Node>("p0a_second_plc", "/", rclcpp::NodeOptions().use_global_arguments(false));
  auto second = std::make_unique<ReferenceRequester>(second_node, ReferenceRequester::Options{});
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(second_node);
  std::thread spin([&ex] { ex.spin(); });
  const auto t0 = Clock::now();
  ASSERT_TRUE(H().waitStatus([](const StatusMsg& m) { return m.detail == "TWO_REQUESTERS"; }, milliseconds(3000)));
  record("p6a_detected_ms", msSince(t0));
  EXPECT_EQ(H().last()->state, StatusMsg::STATE_UNAVAILABLE);
  // every command NOT_SYNCHRONIZED: read on the recorder (each status is published; with two writers the requester's
  // own acknowledgement may be overwritten by the other's image before it reads it)
  // (processed ONCE, Decision 6: the rejection is published once -- the recorder, another subscription, may read it a
  // few ms after the requester did: a bounded wait on the recorded history)
  const auto rejected = H().plc->command(CommandMsg::CMD_RESET, 0, 0, false);
  const auto seen = H().firstSeen([seq = rejected.seq](const StatusMsg& m) {
    return m.ack_seq == seq && m.ack == StatusMsg::ACK_REJECTED && m.ack_code == StatusMsg::REJECT_NOT_SYNCHRONIZED;
  }, milliseconds(1000));
  EXPECT_TRUE(seen.has_value()) << "the RESET of seq " << rejected.seq << " was not rejected NOT_SYNCHRONIZED";
  EXPECT_EQ(H().last()->state, StatusMsg::STATE_UNAVAILABLE);
  ex.cancel();
  spin.join();
  second.reset();
  second_node.reset();
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE, milliseconds(5000))) << H().last()->detail;
  // (b) RUNNING: a second publisher writes the ADOPTED session with a non-advancing heartbeat
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  auto raw_node = std::make_shared<rclcpp::Node>("p0a_raw_plc", "/", rclcpp::NodeOptions().use_global_arguments(false));
  auto raw = raw_node->create_publisher<CommandMsg>(trainit::rti::kCommandTopic, rclcpp::QoS(1).reliable().transient_local());
  CommandMsg img;
  img.requester_session = H().plc->session();
  img.heartbeat = 7;
  const auto t1 = Clock::now();
  bool conflict = false;
  while (!conflict && msSince(t1) < 3000)
  {
    raw->publish(img);
    conflict = H().last()->state == StatusMsg::STATE_STOPPING || H().last()->state == StatusMsg::STATE_FAULTED;
    std::this_thread::sleep_for(milliseconds(100));
  }
  ASSERT_TRUE(H().waitResult(1));
  record("p6b_detected_ms", msSince(t1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_REQUESTER_CONFLICT) << H().last()->detail;
  raw.reset();
  raw_node.reset();
  ASSERT_TRUE(m26::waitUntil([] { return H().commandPublishers() == 1; }, milliseconds(5000)));
  std::this_thread::sleep_for(milliseconds(300));
  ASSERT_EQ(H().plc->reset().outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE));
  // (c) RUNNING: a second publisher writes a FOREIGN session
  H().mg.resetScenario();
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());
  ASSERT_EQ(H().plc->start(12, 2).outcome, O::ACCEPTED);
  raw_node = std::make_shared<rclcpp::Node>("p0a_raw_plc2", "/", rclcpp::NodeOptions().use_global_arguments(false));
  raw = raw_node->create_publisher<CommandMsg>(trainit::rti::kCommandTopic, rclcpp::QoS(1).reliable().transient_local());
  img.requester_session = 424242;
  const auto t2 = Clock::now();
  while (msSince(t2) < 3000 && H().last()->state == StatusMsg::STATE_RUNNING)
  {
    ++img.heartbeat;
    raw->publish(img);
    std::this_thread::sleep_for(milliseconds(100));
  }
  ASSERT_TRUE(H().waitResult(2));
  const auto code = H().last()->result_code;
  ::testing::Test::RecordProperty("p6c_code", std::to_string(code));
  EXPECT_TRUE(code == StatusMsg::CODE_REQUESTER_CONFLICT || code == StatusMsg::CODE_REQUESTER_LOST) << code;
  ASSERT_TRUE(H().waitState(StatusMsg::STATE_FAULTED));
  raw.reset();
  raw_node.reset();
  stopChild(server);
}

TEST(P0aTaskServer, P9_TheHeartbeatGoesOnWhileAMotionBlocksTheTick)
{
  Scope s("P9");
  auto server = startServer("p9");
  operational();
  H().mg.ex_script.push_back(m25::Reply::hold());
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(m26::waitUntil([&] { return H().mg.heldCount() > 0; }, milliseconds(15000)));
  const auto t0 = Clock::now();
  std::this_thread::sleep_for(milliseconds(3000));   // the tick thread is blocked inside execute()
  double gap = 0.0;
  Clock::time_point prev{};
  uint16_t prev_hb = 0;
  for (const auto& [t, m] : H().statusCopy())
  {
    if (t < t0) continue;
    if (prev != Clock::time_point{} && m.heartbeat != prev_hb)
      gap = std::max(gap, std::chrono::duration<double, std::milli>(t - prev).count());
    if (m.heartbeat != prev_hb)
    {
      prev = t;
      prev_hb = m.heartbeat;
    }
  }
  record("max_heartbeat_gap_ms", gap);
  EXPECT_GT(gap, 0.0);
  EXPECT_LE(gap, 200.0) << "no gap > 2 periods";
  EXPECT_TRUE(H().plc->serverAlive());
  EXPECT_EQ(H().last()->state, StatusMsg::STATE_RUNNING);
  H().mg.releaseHeld();
  ASSERT_TRUE(H().waitResult(1, milliseconds(15000)));
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- clocks
TEST(P0aTaskServer, CL1_SimTimeAtHalfSpeedLeavesTheRtiOnTheWallClock)
{
  Scope s("CL1");
  auto clock_node = std::make_shared<rclcpp::Node>("p0a_clock", "/", rclcpp::NodeOptions().use_global_arguments(false));
  auto clock_pub = clock_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
  std::atomic<bool> ticking{true};
  const auto c0 = Clock::now();
  std::thread sim([&] {
    while (ticking)
    {
      rosgraph_msgs::msg::Clock c;
      const double sim_s = 100.0 + 0.5 * msSince(c0) / 1000.0;   // 0.5x real time
      c.clock.sec = static_cast<int32_t>(sim_s);
      c.clock.nanosec = static_cast<uint32_t>((sim_s - c.clock.sec) * 1e9);
      clock_pub->publish(c);
      std::this_thread::sleep_for(milliseconds(10));
    }
  });
  auto server = startServer("cl1", "    use_sim_time: true\n");
  operational();
  // the heartbeat period on the WALL clock
  const auto t0 = Clock::now();
  std::this_thread::sleep_for(milliseconds(2000));
  std::vector<double> periods;
  Clock::time_point prev{};
  uint16_t prev_hb = 0;
  for (const auto& [t, m] : H().statusCopy())
  {
    if (t < t0) continue;
    if (m.heartbeat == prev_hb) continue;
    if (prev != Clock::time_point{}) periods.push_back(std::chrono::duration<double, std::milli>(t - prev).count());
    prev = t;
    prev_hb = m.heartbeat;
  }
  ASSERT_GE(periods.size(), 10u);
  const double mean = std::accumulate(periods.begin(), periods.end(), 0.0) / periods.size();
  record("cl1_heartbeat_period_wall_ms", mean);
  EXPECT_NEAR(mean, 100.0, 20.0);
  // timeout_s on the WALL clock: home (5 s) with a held motion
  H().mg.stop_releases_held = true;
  H().mg.ex_script.push_back(m25::Reply::hold());
  const auto t1 = Clock::now();
  ASSERT_EQ(H().plc->start(7, 1).outcome, O::ACCEPTED);
  const auto stopping = H().sawState(StatusMsg::STATE_STOPPING, milliseconds(12000));
  ASSERT_TRUE(stopping);
  const double fired = std::chrono::duration<double, std::milli>(*stopping - t1).count();
  record("cl1_timeout_fired_wall_ms", fired);
  EXPECT_GE(fired, 5000.0);
  EXPECT_LE(fired, 5300.0);   // not 10 s: sim time runs at 0.5x
  ASSERT_TRUE(H().waitResult(1));
  EXPECT_EQ(H().last()->result_code, StatusMsg::CODE_TASK_TIMEOUT);   // not REQUESTER_LOST
  EXPECT_TRUE(H().plc->serverAlive());
  ticking = false;
  sim.join();
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- events
TEST(P0aTaskServer, E1_E3_ThreeSequentialJobsAreThreeRuns)
{
  Scope s("E1_E3");
  auto server = startServer("e1");
  operational();
  EXPECT_EQ(H().plc->start(99, 50).outcome, O::REJECTED);   // E3: a rejected START
  for (uint32_t job = 1; job <= 3; ++job)
  {
    ASSERT_EQ(H().plc->start(7, job).outcome, O::ACCEPTED) << job;
    ASSERT_TRUE(H().waitResult(job)) << job;
    ASSERT_TRUE(H().waitState(StatusMsg::STATE_IDLE)) << job;
  }
  ASSERT_TRUE(H().waitEvent([](const Event& e) { return e.kind == Event::RUN_ENDED && e.job_id == 3u; },
                            milliseconds(3000)));
  const auto ev = H().eventsCopy();
  const auto session = H().last()->server_session;
  char prefix[16];
  std::snprintf(prefix, sizeof(prefix), "%08x-", session);
  std::map<std::string, std::vector<Event>> runs;
  for (const auto& e : ev) runs[e.run_id].push_back(e);
  EXPECT_EQ(runs.size(), 3u);   // E3: the rejected START made no run
  std::set<uint32_t> jobs;
  for (const auto& [id, list] : runs)
  {
    EXPECT_EQ(id.rfind(prefix, 0), 0u) << id;
    ASSERT_FALSE(list.empty());
    EXPECT_EQ(list.front().kind, Event::RUN_STARTED) << id;
    EXPECT_EQ(list.back().kind, Event::RUN_ENDED) << id;
    for (std::size_t k = 0; k < list.size(); ++k)
    {
      EXPECT_EQ(list[k].seq, k + 1) << id;
      EXPECT_EQ(list[k].job_id, list.front().job_id) << id;
    }
    jobs.insert(list.front().job_id);
    EXPECT_EQ(list.back().result_code, StatusMsg::CODE_OK) << id;
    bool task_exit = false;
    for (const auto& e : list)
      if (e.kind == Event::TASK_EXITED && e.task_id == "home")
      {
        task_exit = true;
        EXPECT_EQ(e.result_code, StatusMsg::CODE_OK) << id;
      }
      else if (e.kind != Event::RUN_ENDED)
      {
        EXPECT_EQ(e.result_code, 0) << id << " kind " << int(e.kind);
      }
    EXPECT_TRUE(task_exit) << id;
  }
  EXPECT_EQ(jobs, (std::set<uint32_t>{1, 2, 3}));
  // the run ids are EXACTLY <server_session>-1..3 (the server's job counter; a rejected START does not count)
  for (uint32_t job = 1; job <= 3; ++job)
  {
    const std::string id = std::string(prefix) + std::to_string(job);
    ASSERT_TRUE(runs.count(id)) << id;
    EXPECT_EQ(runs[id].front().job_id, job) << id;
  }
  stopChild(server);
}

// ---------------------------------------------------------------------------------------------------- product (B2)
TEST(P0aTaskServer, B2_OneBundleTwoRunModelsTheSameTaskEvents)
{
  Scope s("B2", 300);
  // one-shot: trainit_run_bt runs MainTree (main_task place_part), exactly as the bundle's trainit_bt launch does
  auto runner = startChild(TRAINIT_RUN_BT_EXE, "b2_run_bt", kServed + "/task_program.xml", kServed + "/tasks.yaml");
  ASSERT_TRUE(runner->wait(milliseconds(90000)));
  EXPECT_EQ(runner->exitText(), "exit code 0") << readAll(runner->output());
  const auto one_shot = H().eventsCopy();
  std::string run_bt_run;
  for (const auto& e : one_shot)
    if (e.kind == Event::RUN_STARTED)
    {
      run_bt_run = e.run_id;
      EXPECT_EQ(e.job_id, 0u);   // not a job
    }
  ASSERT_FALSE(run_bt_run.empty());
  H().fresh();
  // served: the same files, a job of place_part
  auto server = startServer("b2_server");
  operational();
  ASSERT_EQ(H().plc->start(12, 1).outcome, O::ACCEPTED);
  ASSERT_TRUE(H().waitResult(1, milliseconds(15000)));
  ASSERT_TRUE(H().waitEvent([](const Event& e) { return e.kind == Event::RUN_ENDED; }, milliseconds(3000)));
  const auto served = H().eventsCopy();
  std::string job_run;
  for (const auto& e : served)
    if (e.kind == Event::RUN_STARTED) job_run = e.run_id;
  const auto a = projection(one_shot, run_bt_run);
  const auto b = projection(served, job_run);
  EXPECT_FALSE(a.empty());
  EXPECT_EQ(a, b) << "the same TASK / STEP sequence: no file edited between the two run models";
  stopChild(server);
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
