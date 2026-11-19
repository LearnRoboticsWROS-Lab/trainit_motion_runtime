// test_p0b_machine_nodes (P0.b, P0B_DESIGN.md §3.3-§3.5; the frozen matrix's S cases, P0_CHECKPOINT.md "P0.b frozen
// acceptance matrix"): RequestMachine / WaitForMachine TICKED in process, as the runners tick them (every 10 ms), with
// the REAL machine-channel registry (its callback group, subscriptions, request publisher, publisher counts) against
// a SCRIPTED status source implementing §3.3 -- a request consumed once (accepted or rejected by its identity), BUSY /
// DONE / FAULT / HOMING, the heartbeat every period, knobs: a timed cycle, a stuck actuator, a station fault, a
// cycle that never completes, a frozen image, silence (killed), a restart (a NEW session on the same publisher), a
// request consumed late, a request injected without a second publisher, a displaced rejection, a second status /
// request publisher. Domain 95, localhost only; CL2 / CL3 publish /clock at 0.5x (as P0.a's CL1). Each test is named
// after its case; its measured timings are test properties (the gtest XML is the evidence). The job scope is begun
// and ended as BtJobExecutor does; the RTI's activity path (D5) is run end to end with the real RobotTaskServer and
// the reference requester (the fake PLC's lines).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <regex>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <trainit_interfaces/msg/channel_request.hpp>
#include <trainit_interfaces/msg/channel_status.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/machine_channels.hpp"
#include "trainit_motion_runtime/reference_requester.hpp"
#include "trainit_motion_runtime/robot_task_server.hpp"
#include "p0b/scripted_source.hpp"

using namespace std::chrono_literals;
using p0b_test::Clock;
using p0b_test::RequestMsg;
using p0b_test::ScriptedSource;
using p0b_test::StatusMsg;
using trainit::machine::Identity;

namespace
{
const char* kNs = "/p0b";

double msSince(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double msBetween(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

void record(const std::string& key, double ms)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", ms);
  ::testing::Test::RecordProperty(key, buf);
}

template <class Pred>
bool eventually(Pred pred, int ms = 5000)
{
  const auto end = Clock::now() + std::chrono::milliseconds(ms);
  while (Clock::now() < end)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

rclcpp::QoS channelQos()
{
  return p0b_test::sourceQos();
}

// =====================================================================================================================
// The robot: a runner's node, its machine-channel registry (created BEFORE the executor: its callback group), the
// BtContext the nodes read, the job scope's activity sink and the PROCESS_EVENTs the registry reports.
// =====================================================================================================================
struct Job
{
  BT::Tree tree;
  BT::Blackboard::Ptr bb;
};

class Robot
{
public:
  explicit Robot(const std::string& name, trainit::MachineChannels::Options o = {}, bool sim_time = false)
  {
    rclcpp::NodeOptions no;
    no.use_global_arguments(false);
    if (sim_time) no.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node = std::make_shared<rclcpp::Node>(name, kNs, no);
    machines = std::make_unique<trainit::MachineChannels>(node, o);
    observer = std::make_shared<trainit::ObservedProcessController::Observer>([this](const trainit::ProcessEvent& e) {
      std::lock_guard<std::mutex> l(m_);
      events_.push_back(e);
    });
    machines->setProcessObserver(observer);
    exec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    exec->add_node(node);
    spinner = std::thread([this] { exec->spin(); });
    ctx.node = node;
    ctx.machines = machines.get();
    trainit::registerAllNodes(factory);
  }
  ~Robot()
  {
    exec->cancel();
    if (spinner.joinable()) spinner.join();
    exec.reset();
    machines.reset();
    node.reset();
  }

  void prepare(const std::string& service) { machines->prepare(service); }
  // the runner's settle: the service live, synchronised, one status and at most one request publisher
  bool settle(const std::string& service, int ms = 5000)
  {
    return eventually(
      [&] {
        const auto f = machines->feed(service);
        const auto now = Clock::now();
        return f && f->live(now) && f->synchronised() && f->statusPublishers() == 1 && f->requestPublishers() <= 1;
      },
      ms);
  }
  void beginJob()
  {
    {
      std::lock_guard<std::mutex> l(m_);
      activity_.clear();
    }
    machines->beginJob([this](const std::string& t) {
      std::lock_guard<std::mutex> l(m_);
      activity_.emplace_back(Clock::now(), t);
    });
  }
  void endJob() { machines->endJob(); }

  Job tree(const std::string& body)
  {
    const std::string id = "T" + std::to_string(++trees_);
    return fromXml("<root BTCPP_format=\"4\"><BehaviorTree ID=\"" + id + "\">" + body + "</BehaviorTree></root>", id);
  }
  Job fromXml(const std::string& xml, const std::string& id)
  {
    factory.registerBehaviorTreeFromText(xml);
    Job j;
    j.bb = BT::Blackboard::create();
    j.bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
    j.tree = factory.createTree(id, j.bb);
    return j;
  }

  std::vector<std::string> activity() const
  {
    std::lock_guard<std::mutex> l(m_);
    std::vector<std::string> out;
    for (const auto& [t, s] : activity_) out.push_back(s);
    return out;
  }
  std::optional<Clock::time_point> firstActivity(const std::string& part) const
  {
    std::lock_guard<std::mutex> l(m_);
    for (const auto& [t, s] : activity_)
      if (s.find(part) != std::string::npos) return t;
    return std::nullopt;
  }
  bool hasActivity(const std::string& part) const { return firstActivity(part).has_value(); }
  std::vector<trainit::ProcessEvent> events() const
  {
    std::lock_guard<std::mutex> l(m_);
    return events_;
  }

  rclcpp::Node::SharedPtr node;
  std::unique_ptr<trainit::MachineChannels> machines;
  std::shared_ptr<trainit::ObservedProcessController::Observer> observer;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
  std::thread spinner;
  trainit::BtContext ctx;
  BT::BehaviorTreeFactory factory;

private:
  mutable std::mutex m_;
  std::vector<std::pair<Clock::time_point, std::string>> activity_;
  std::vector<trainit::ProcessEvent> events_;
  int trees_{0};
};

// One run of a tree (TickRun: gtest's Test::Run would shadow a `Run`), ticked as the runners tick: tickOnce, then 10 ms while RUNNING. `each(ms)` runs before every tick;
// it returns false to HALT the tree there (a cancel: BtJobExecutor halts between ticks).
struct TickRun
{
  BT::NodeStatus status{BT::NodeStatus::RUNNING};
  Clock::time_point t0{};
  Clock::time_point end{};
  double ms{0};
  int ticks{0};
  bool halted{false};
  double halt_ms{0};
};

TickRun run(Job& job, int limit_ms, const std::function<bool(double)>& each = nullptr)
{
  TickRun r;
  r.t0 = Clock::now();
  for (;;)
  {
    const double ms = msSince(r.t0);
    if (each && !each(ms))
    {
      const auto h0 = Clock::now();
      job.tree.haltTree();
      r.halt_ms = msSince(h0);
      r.halted = true;
      break;
    }
    r.status = job.tree.tickOnce();
    ++r.ticks;
    if (r.status != BT::NodeStatus::RUNNING || ms > limit_ms) break;
    std::this_thread::sleep_for(10ms);
  }
  r.end = Clock::now();
  r.ms = msBetween(r.t0, r.end);
  return r;
}

std::string faultOf(const Job& j)
{
  try
  {
    if (!j.bb->getEntry("trainit__fault")) return "";
    return j.bb->get<std::string>("trainit__fault");
  }
  catch (const std::exception&)
  {
    return "";
  }
}

// --- the contract's XML (the TSA <-> TMR contract of P0B_IMPLEMENTATION_PLAN.md, under faults: stop) -------------------
std::string request(const std::string& s, const std::string& step = "ask")
{
  return "<Sequence name=\"trainit_step:" + step + "_" + s + ":process\"><RequestMachine channel=\"" + s +
         "\" fault=\"{trainit__fault}\"/></Sequence>";
}
std::string waitFor(const std::string& s, const char* what, int timeout_ms, const std::string& step = "wait")
{
  return "<Sequence name=\"trainit_step:" + step + "_" + s + ":wait\"><WaitForMachine channel=\"" + s + "\" for=\"" +
         what + "\" timeout_ms=\"" + std::to_string(timeout_ms) + "\" fault=\"{trainit__fault}\"/></Sequence>";
}
std::string sleepMs(int ms)   // the robot's motion meanwhile (BT.CPP's Sleep: what a TSA dwell runs)
{
  return "<Sleep msec=\"" + std::to_string(ms) + "\"/>";
}
std::string seq(const std::vector<std::string>& parts)
{
  std::string out = "<Sequence>";
  for (const auto& p : parts) out += p;
  return out + "</Sequence>";
}

bool isState(const StatusMsg& m, uint8_t s) { return m.state == s; }
}  // namespace

// =====================================================================================================================
// The contract (no ROS graph): the XML of the TSA <-> TMR contract loads, the ports are exactly the contract's, the
// Groot palette lists them, a bad port or a missing registry fails the node (never an exception out of a tick).
// =====================================================================================================================
TEST(P0bContract, TheContractXmlLoadsAndThePortsAreTheContracts)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  for (const char* id : {"RequestMachine", "WaitForMachine"})
    ASSERT_EQ(factory.manifests().count(id), 1u) << id;
  auto ports = [&](const char* id) {
    std::map<std::string, std::string> out;
    for (const auto& [name, info] : factory.manifests().at(id).ports)
      out[name] = info.direction() == BT::PortDirection::OUTPUT ? "output" : "input";
    return out;
  };
  EXPECT_EQ(ports("RequestMachine"),
            (std::map<std::string, std::string>{{"channel", "input"}, {"fault", "output"}}));
  EXPECT_EQ(ports("WaitForMachine"), (std::map<std::string, std::string>{
                                       {"channel", "input"}, {"for", "input"}, {"timeout_ms", "input"},
                                       {"fault", "output"}}));
  for (const char* id : {"RequestMachine", "WaitForMachine"})
    for (const auto& [name, info] : factory.manifests().at(id).ports)
      EXPECT_TRUE(info.defaultValueString().empty()) << id << "." << name << ": no default (timeout_ms has none)";
  // the plan's pinned XML, both forms (with and without the fault gate)
  factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="C">
    <Sequence>
      <Sequence name="trainit_step:ask_turn:process"><RequestMachine channel="turner"/></Sequence>
      <Sequence name="trainit_step:turned:wait"><WaitForMachine channel="turner" for="done" timeout_ms="8000"/></Sequence>
      <Sequence name="trainit_step:ask_turn2:process"><RequestMachine channel="turner" fault="{trainit__fault}"/></Sequence>
      <Sequence name="trainit_step:turned2:wait"><WaitForMachine channel="turner" for="ready" timeout_ms="2000" fault="{trainit__fault}"/></Sequence>
    </Sequence></BehaviorTree></root>)");
  EXPECT_NO_THROW(factory.createTree("C"));
}

TEST(P0bContract, TheGrootPaletteListsTheMachineNodes)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  std::ifstream in(std::string(TRAINIT_SOURCE_DIR) + "/config/bt/trainit_nodes.groot.xml");
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string xml = ss.str();
  for (const char* id : {"RequestMachine", "WaitForMachine"})
  {
    const auto a = xml.find(std::string("<Action ID=\"") + id + "\">");
    ASSERT_NE(a, std::string::npos) << id << " is not in the Groot model";
    const std::string block = xml.substr(a, xml.find("</Action>", a) - a);
    std::map<std::string, std::string> model;
    const std::regex port("<(input|output)_port name=\"([A-Za-z_]+)\"");
    for (auto it = std::sregex_iterator(block.begin(), block.end(), port); it != std::sregex_iterator(); ++it)
      model[(*it)[2]] = (*it)[1];
    std::map<std::string, std::string> manifest;
    for (const auto& [name, info] : factory.manifests().at(id).ports)
      manifest[name] = info.direction() == BT::PortDirection::OUTPUT ? "output" : "input";
    EXPECT_EQ(model, manifest) << id;
    EXPECT_EQ(block.find("default="), std::string::npos) << id << ": no default in the palette";
  }
}

TEST(P0bContract, ABadPortOrNoRegistryFailsTheNodeWithoutACode)
{
  auto robot = std::make_unique<Robot>("p0b_robot_ports");
  robot->beginJob();
  for (const std::string& body :
       {std::string("<WaitForMachine channel=\"ports_svc\" for=\"later\" timeout_ms=\"100\"/>"),
        std::string("<WaitForMachine channel=\"ports_svc\" for=\"done\" timeout_ms=\"0\"/>"),
        std::string("<WaitForMachine channel=\"ports_svc\" for=\"done\" timeout_ms=\"600001\"/>"),
        std::string("<WaitForMachine channel=\"ports_svc\" for=\"done\"/>"),
        std::string("<WaitForMachine channel=\"ports/svc\" for=\"done\" timeout_ms=\"100\"/>"),
        std::string("<RequestMachine channel=\"/plc/output/12\"/>"),
        std::string("<RequestMachine channel=\"not_prepared\"/>")})
  {
    Job j = robot->tree(body);
    const TickRun r = run(j, 1000);
    EXPECT_EQ(r.status, BT::NodeStatus::FAILURE) << body;
    EXPECT_EQ(r.ticks, 1) << body;
    EXPECT_EQ(robot->machines->jobStationCode(), 0) << body << ": a program error, never a machine condition";
  }
  // no registry (a hand-built context): FAILURE, never a throw out of the tick
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  trainit::BtContext bare;
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &bare);
  factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="B">
    <RequestMachine channel="turner"/></BehaviorTree></root>)");
  auto tree = factory.createTree("B", bb);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  auto empty = BT::Blackboard::create();   // no BtContext at all
  auto tree2 = factory.createTree("B", empty);
  EXPECT_EQ(tree2.tickOnce(), BT::NodeStatus::FAILURE);
}

// =====================================================================================================================
// NOMINAL
// =====================================================================================================================
// N1 (S): request turner -> BUSY -> DONE; the wait for done succeeds <= 1 status period + 1 tick after DONE; the robot
// continues; one station cycle; the PROCESS_EVENT `request` once.
TEST(P0bMachineNodes, N1_ARequestThenItsDoneTheRobotContinues)
{
  const std::string s = "n1_turner";
  ScriptedSource src(s, {100, 600});
  Robot r("p0b_robot_n1");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000), "<AlwaysSuccess/>"}));
  const TickRun run1 = run(j, 10000);
  r.endJob();
  ASSERT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  const auto requested = r.firstActivity("REQUESTED");
  const auto busy = src.firstPublished([](const StatusMsg& m) { return isState(m, StatusMsg::STATE_BUSY); });
  const auto done = src.firstPublished([](const StatusMsg& m) { return m.cycle_done; });
  ASSERT_TRUE(requested && busy && done);
  const double done_to_success = msBetween(*done, run1.end);
  record("n1_request_to_busy_ms", msBetween(*requested, *busy));
  record("n1_request_to_done_ms", msBetween(*requested, *done));
  record("n1_done_to_success_ms", done_to_success);
  EXPECT_LE(done_to_success, 110.0) << "<= 1 status period + 1 tick after DONE";
  EXPECT_EQ(src.accepted(), 1) << "one station cycle";
  EXPECT_EQ(src.completed(), 1);
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  EXPECT_TRUE(faultOf(j).empty());
  const auto words = r.activity();
  ASSERT_GE(words.size(), 3u);
  EXPECT_EQ(words.front(), "MACHINE n1_turner REQUESTED");
  EXPECT_TRUE(r.hasActivity("MACHINE n1_turner BUSY: "));
  EXPECT_EQ(words.back().rfind("MACHINE n1_turner DONE in ", 0), 0u) << words.back();
  EXPECT_EQ(words.back().find("(wall)"), std::string::npos)
    << "the request recorded the runner's clock: DONE in is on the timeout's clock (acceptance finding 1): "
    << words.back();
  const auto ev = r.events();
  ASSERT_EQ(ev.size(), 1u) << "one PROCESS_EVENT per request written";
  EXPECT_EQ(ev[0].type, trainit::ProcessEventType::REQUEST);
  EXPECT_EQ(ev[0].channel, s);
  EXPECT_EQ(trainit::processKindAndValue(ev[0]).first, "request");
  const auto o = r.machines->outstanding(s);
  ASSERT_TRUE(o);
  EXPECT_TRUE(o->completed);
  EXPECT_EQ(o->written.id.requester, r.machines->requesterSession());
  ASSERT_TRUE(o->runner_ns.has_value()) << "the request recorded the runner's clock (acceptance finding 1)";
  EXPECT_GT(*o->runner_ns, 0);
}

// C++ review M1 (protocol N4): the PROCESS_EVENT `request` (ADR-0030) is reported AFTER the publish -- an observer that
// throws is OBSERVATION failing, never the request step: the request is on the wire and recorded, the step SUCCEEDS
// and its wait matches it (before the fix the node reported "could not be written" for a request the source executed).
TEST(P0bMachineNodes, AThrowingEventObserverNeverFailsAWrittenRequest)
{
  const std::string s = "obs_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_obs");
  std::atomic<int> calls{0};
  r.machines->setProcessObserver(
    std::make_shared<trainit::ObservedProcessController::Observer>([&calls](const trainit::ProcessEvent&) {
      ++calls;
      throw std::runtime_error("the execution-event sink is gone");
    }));
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun run1 = run(j, 5000);
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j) << r.machines->jobStationText();
  EXPECT_EQ(calls.load(), 1) << "the observer was called once (and threw)";
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  EXPECT_TRUE(r.machines->jobStationText().empty()) << r.machines->jobStationText();
  EXPECT_TRUE(faultOf(j).empty()) << faultOf(j);
  EXPECT_EQ(r.activity().front(), "MACHINE obs_turner REQUESTED");
  ASSERT_TRUE(r.machines->lastWritten(s)) << "recorded as this process's last written request";
  EXPECT_EQ(src.accepted(), 1) << "on the wire: the source ran its cycle";
  ASSERT_EQ(src.consumed().size(), 1u);
  EXPECT_EQ(src.consumed()[0], r.machines->lastWritten(s)->id);
  EXPECT_TRUE(r.machines->outstanding(s)->completed) << "the wait matched the request it wrote";
}

// Protocol review fix 3: the status SUBSCRIPTION keeps a deeper reader history than the depth-1 writers (KEEP_LAST 20,
// RELIABLE, TRANSIENT_LOCAL; the wire unchanged, the request publisher stays depth 1, level-held): two images that
// arrive between two deliveries of the registry's callback group are BOTH applied, in order -- the one naming this
// process's request is never dropped before the consumed latch sees it. A depth-1 reader beside it shows the mechanism:
// there only the second image survives (with it, the registry's consumed stayed false: a 313 lock-out).
TEST(P0bMachineNodes, TwoImagesBetweenTwoDeliveriesAreBothApplied)
{
  const std::string s = "qos_turner";
  const auto w = trainit::MachineChannels::qos().get_rmw_qos_profile();   // the request WRITER: the wire's profile
  EXPECT_EQ(w.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(w.depth, 1u);
  EXPECT_EQ(w.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(w.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  const auto rd = trainit::MachineChannels::statusQos().get_rmw_qos_profile();   // the status READER
  EXPECT_EQ(rd.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(rd.depth, 20u);
  EXPECT_EQ(rd.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(rd.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // the robot's node and its registry, NOT spun yet: what arrives meanwhile waits in the readers' histories
  const auto opts = rclcpp::NodeOptions().use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>("p0b_robot_qos", kNs, opts);
  trainit::MachineChannels machines(node, {});
  machines.prepare(s);
  auto probe = std::make_shared<rclcpp::Node>("p0b_probe_qos", kNs, opts);
  std::mutex m;
  std::vector<uint16_t> depth1;   // the heartbeats a depth-1 reader delivers
  auto sub1 = probe->create_subscription<StatusMsg>(s + "/status", channelQos(), [&](StatusMsg::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> l(m);
    depth1.push_back(msg->heartbeat);
  });
  auto src = std::make_shared<rclcpp::Node>("p0b_src_qos", kNs, opts);
  auto pub =
    src->create_publisher<StatusMsg>(s + "/status", rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local());
  ASSERT_TRUE(eventually([&] { return pub->get_subscription_count() == 2u; })) << "both readers matched";
  const uint32_t session = 0x5E550001u;
  const auto x = machines.write(s, session, Clock::now());   // this process's request X: the feed watches for it
  ASSERT_TRUE(x);
  StatusMsg a;
  a.source_session = session;
  a.heartbeat = 1;
  a.state = StatusMsg::STATE_BUSY;
  a.cycle_requester = x->requester;   // the source took X ...
  a.cycle_seq = x->seq;
  pub->publish(a);
  std::this_thread::sleep_for(50ms);
  StatusMsg b = a;                     // ... then another requester's cycle displaced it, before any delivery
  b.heartbeat = 2;
  b.cycle_requester = 0x2222;
  b.cycle_seq = 1;
  pub->publish(b);
  std::this_thread::sleep_for(200ms);
  ASSERT_FALSE(machines.feed(s)->hasImage()) << "nothing is delivered before the executor runs";
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std::thread spin([&] { exec.spin(); });
  const bool delivered = eventually([&] {
    std::lock_guard<std::mutex> l(m);
    return machines.feed(s)->last().heartbeat == 2 && !depth1.empty();
  }, 3000);
  std::this_thread::sleep_for(100ms);
  exec.cancel();
  spin.join();
  ASSERT_TRUE(delivered);
  {
    std::lock_guard<std::mutex> l(m);
    EXPECT_EQ(depth1, (std::vector<uint16_t>{2})) << "a depth-1 reader drops the intermediate image";
  }
  const auto f = machines.feed(s);
  EXPECT_EQ(f->last().cycle.requester, 0x2222u) << "in order: the second image is the last applied";
  EXPECT_TRUE(f->consumed(trainit::machine::Written{session, *x})) << "the first image, naming X, was applied too";
}

// O1 (S, M run 4): the fake PLC prints EVERY state the robot published, even a RUNNING detail the job leaves within a
// millisecond -- a ready-wait that is ready at once (N3) publishes "MACHINE turner DONE: ready ..." and then the
// result. The reference requester's status READER keeps a KEEP_LAST 20 history (QoS-compatible with the depth-1
// writer): both statuses, published while its node is not spun, are handled in order; a depth-1 reader beside it
// delivers only the second (the mechanism, shown in the same run).
TEST(P0bMachineNodes, O1_TheFakePlcPrintsARunningDetailTheJobLeavesAtOnce)
{
  using trainit::rti::ReferenceRequester;
  const auto rd = ReferenceRequester::statusReaderQos().get_rmw_qos_profile();
  EXPECT_EQ(rd.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(rd.depth, 20u);
  EXPECT_EQ(rd.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(rd.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  const auto no = rclcpp::NodeOptions().use_global_arguments(false);
  const std::string topic = "o1_rti/status";
  auto plc_node = std::make_shared<rclcpp::Node>("p0b_o1_plc", kNs, no);
  ReferenceRequester::Options opts;
  opts.status_topic = topic;
  opts.command_topic = "o1_rti/command";
  ReferenceRequester plc(plc_node, opts);
  std::mutex m;
  std::vector<std::string> said;
  plc.setObserver([&](const std::string& line) {
    std::lock_guard<std::mutex> l(m);
    said.push_back(line);
  });
  auto probe = std::make_shared<rclcpp::Node>("p0b_o1_probe", kNs, no);
  std::vector<std::string> depth1;   // the details a depth-1 reader delivers
  auto sub1 = probe->create_subscription<ReferenceRequester::StatusMsg>(
    topic, rclcpp::QoS(1).reliable().transient_local(), [&](ReferenceRequester::StatusMsg::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> l(m);
      depth1.push_back(msg->detail);
    });
  auto server = std::make_shared<rclcpp::Node>("p0b_o1_server", kNs, no);
  auto pub =
    server->create_publisher<ReferenceRequester::StatusMsg>(topic, rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(eventually([&] { return pub->get_subscription_count() == 2u; })) << "both readers matched";
  ReferenceRequester::StatusMsg a;
  a.interface_version = 1;
  a.server_session = 0x0D1E0001u;
  a.heartbeat = 7;
  a.state = ReferenceRequester::StatusMsg::STATE_RUNNING;
  a.job_id = 6;
  a.task = 6;
  a.task_name = "ready_check";
  a.detail = "MACHINE turner DONE: ready after 0.0 s";
  pub->publish(a);
  ReferenceRequester::StatusMsg b = a;   // the job ends at once: the result, before any delivery
  b.state = ReferenceRequester::StatusMsg::STATE_IDLE;
  b.result = ReferenceRequester::StatusMsg::RESULT_SUCCEEDED;
  b.result_code = ReferenceRequester::StatusMsg::CODE_OK;
  b.detail = "";
  pub->publish(b);
  std::this_thread::sleep_for(200ms);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(plc_node);
  exec.add_node(probe);
  std::thread spin([&] { exec.spin(); });
  const bool handled = eventually([&] {
    std::lock_guard<std::mutex> l(m);
    return !depth1.empty() && std::any_of(said.begin(), said.end(), [](const std::string& x) {
             return x.find("Job 6 (ready_check) finished: SUCCEEDED") != std::string::npos;
           });
  }, 3000);
  std::this_thread::sleep_for(100ms);
  exec.cancel();
  spin.join();
  ASSERT_TRUE(handled);
  std::lock_guard<std::mutex> l(m);
  EXPECT_EQ(depth1, (std::vector<std::string>{""})) << "a depth-1 reader drops the RUNNING detail";
  const auto at = [&](const std::string& part) {
    for (std::size_t k = 0; k < said.size(); ++k)
      if (said[k].find(part) != std::string::npos) return static_cast<long>(k);
    return -1L;
  };
  const long done = at("Robot busy: job 6: MACHINE turner DONE: ready after 0.0 s");
  const long finished = at("Job 6 (ready_check) finished: SUCCEEDED");
  ASSERT_GE(done, 0) << "the fake PLC printed the RUNNING detail";
  EXPECT_LT(done, finished) << "in order: the detail, then the result";
}

// N2 (S): the line indexer, as N1.
TEST(P0bMachineNodes, N2_TheIndexerAsN1)
{
  const std::string s = "n2_indexer";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_n2");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s, "ask_index"), waitFor(s, "done", 8000, "indexed"), "<AlwaysSuccess/>"}));
  const TickRun run1 = run(j, 10000);
  ASSERT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  const auto done = src.firstPublished([](const StatusMsg& m) { return m.cycle_done; });
  ASSERT_TRUE(done);
  record("n2_done_to_success_ms", msBetween(*done, run1.end));
  EXPECT_LE(msBetween(*done, run1.end), 110.0);
  EXPECT_EQ(src.accepted(), 1);
}

// N3 (S): wait for ready with no request -- READY, then DONE as idle -- succeeds at once; no request written.
TEST(P0bMachineNodes, N3_WaitForReadyWithNoRequestSucceedsAtOnce)
{
  const std::string s = "n3_turner";
  ScriptedSource src(s, {100, 200});
  Robot r("p0b_robot_n3");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(waitFor(s, "ready", 2000));
  const TickRun ready = run(a, 3000);
  EXPECT_EQ(ready.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ready.ticks, 1) << "at once: its first tick";
  // the machine DONE as idle (a cycle of the machine master's, not ours)
  src.injectRequest(Identity{0x5EED, 1});
  ASSERT_TRUE(eventually([&] {
    const auto f = r.machines->feed(s);
    return f && f->last().state == StatusMsg::STATE_DONE;
  }));
  r.beginJob();
  Job b = r.tree(waitFor(s, "ready", 2000));
  const TickRun done = run(b, 3000);
  EXPECT_EQ(done.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(done.ticks, 1);
  EXPECT_EQ(src.requestImages(), 0) << "no request written";
  EXPECT_TRUE(r.events().empty());
  record("n3_ready_ms", ready.ms);
  record("n3_done_as_idle_ms", done.ms);
}

// N4 (S): the DONE of THIS request arrives before the wait begins (the robot moves meanwhile): the wait succeeds at its
// first tick (no WAITING / BUSY of the wait).
TEST(P0bMachineNodes, N4_ADoneBeforeTheWaitMatchesAtItsFirstTick)
{
  const std::string s = "n4_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_n4");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), sleepMs(1200), waitFor(s, "done", 8000)}));
  // the WAIT's own ticks (coverage review): counted on the node itself, after each of its ticks
  int wait_ticks = 0;
  BT::NodeStatus wait_first = BT::NodeStatus::IDLE;
  j.tree.applyVisitor([&](BT::TreeNode* n) {
    if (n->registrationName() != "WaitForMachine") return;
    n->setTickMonitorCallback([&](BT::TreeNode&, BT::NodeStatus st, std::chrono::microseconds) {
      if (++wait_ticks == 1) wait_first = st;
    });
  });
  const TickRun run1 = run(j, 5000);
  ASSERT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  EXPECT_EQ(wait_ticks, 1) << "the wait decided at its FIRST tick";
  EXPECT_EQ(wait_first, BT::NodeStatus::SUCCESS) << "never RUNNING";
  const auto words = r.activity();
  ASSERT_EQ(words.size(), 2u) << ::testing::PrintToString(words);
  EXPECT_EQ(words[0], "MACHINE n4_turner REQUESTED");
  EXPECT_EQ(words[1].rfind("MACHINE n4_turner DONE in ", 0), 0u) << words[1];
  record("n4_total_ms", run1.ms);
}

// =====================================================================================================================
// FAULTS
// =====================================================================================================================
// F1 (S): a stuck actuator -> the source FAULT ACTUATOR_TIMEOUT -> the wait FAILED STATION_FAULT 306, MACHINE FAULT shown;
// in the job shape TSA generates under faults: stop (the gate logs the reason, the job tree FAILS).
TEST(P0bMachineNodes, F1_AStuckActuatorIsStationFault306)
{
  const std::string s = "f1_turner";
  ScriptedSource src(s, {100, 2000});
  src.faultNextCycleAfter(300, StatusMsg::FAULT_ACTUATOR_TIMEOUT);
  Robot r("p0b_robot_f1");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  const std::string xml = R"(<root BTCPP_format="4">
    <BehaviorTree ID="job_f1"><Sequence>
      <Script code="trainit__fault := ''"/>
      <Fallback>
        <SubTree ID="task_f1" _autoremap="true"/>
        <Sequence _failureIf="trainit__fault == ''">
          <Log message="FAULT: the run stops (application.faults: stop), never retried or recovered; the reason follows"/>
          <Log message="{trainit__fault}"/>
          <AlwaysFailure/>
        </Sequence>
      </Fallback>
    </Sequence></BehaviorTree>
    <BehaviorTree ID="task_f1"><Sequence>)" + request(s) + waitFor(s, "done", 8000) + R"(</Sequence></BehaviorTree>
  </root>)";
  Job j = r.fromXml(xml, "job_f1");
  const TickRun run1 = run(j, 6000);
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 306);
  const std::string text = r.machines->jobStationText();
  EXPECT_NE(text.find("MACHINE f1_turner FAULT -- STATION_FAULT (306)"), std::string::npos) << text;
  EXPECT_NE(text.find("ACTUATOR_TIMEOUT"), std::string::npos) << text;
  EXPECT_EQ(faultOf(j), text) << "the gate holds the node's fault";
  EXPECT_TRUE(r.hasActivity("MACHINE f1_turner FAULT"));
  const auto fault = src.firstPublished([](const StatusMsg& m) { return isState(m, StatusMsg::STATE_FAULT); });
  ASSERT_TRUE(fault);
  record("f1_fault_to_failure_ms", msBetween(*fault, run1.end));
  EXPECT_LE(msBetween(*fault, run1.end), 110.0);
}

// F2 (S): no DONE (BUSY for ever) -> STATION_TIMEOUT 305 at timeout_ms (+ <= 1 period + 1 tick), the last state named.
TEST(P0bMachineNodes, F2_NoDoneIsStationTimeout305AtTheTimeout)
{
  const std::string s = "f2_turner";
  ScriptedSource src(s, {100, 400});
  src.neverComplete(true);
  Robot r("p0b_robot_f2");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 1000)}));
  const TickRun run1 = run(j, 4000);
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 305);
  EXPECT_NE(r.machines->jobStationText().find("MACHINE f2_turner TIMEOUT -- STATION_TIMEOUT (305): still BUSY after 1.0 s"),
            std::string::npos)
    << r.machines->jobStationText();
  const auto started = r.firstActivity("REQUESTED");   // the wait starts in the same tick
  ASSERT_TRUE(started);
  const double fired = msBetween(*started, run1.end);
  record("f2_timeout_fired_ms", fired);
  EXPECT_GE(fired, 1000.0);
  EXPECT_LE(fired, 1110.0);
}

// F3 (S): the station reports FAULT (STATION_FAULT) for this request -> 306 within 1 period + 1 tick.
TEST(P0bMachineNodes, F3_AStationFaultIs306WithinAPeriodAndATick)
{
  const std::string s = "f3_turner";
  ScriptedSource src(s, {100, 3000});
  Robot r("p0b_robot_f3");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  bool faulted = false;
  const TickRun run1 = run(j, 6000, [&](double) {
    if (!faulted && r.hasActivity("BUSY"))
    {
      src.faultNow(StatusMsg::FAULT_STATION_FAULT);
      faulted = true;
    }
    return true;
  });
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 306);
  EXPECT_NE(r.machines->jobStationText().find("the machine reports FAULT (STATION_FAULT)"), std::string::npos);
  const auto fault = src.firstPublished([](const StatusMsg& m) { return isState(m, StatusMsg::STATE_FAULT); });
  ASSERT_TRUE(fault);
  record("f3_fault_to_failure_ms", msBetween(*fault, run1.end));
  EXPECT_LE(msBetween(*fault, run1.end), 110.0);
}

// F4 (S): the DONE late but inside the timeout -> success, no fault.
TEST(P0bMachineNodes, F4_ALateDoneInsideTheTimeoutSucceeds)
{
  const std::string s = "f4_turner";
  ScriptedSource src(s, {100, 1500});
  Robot r("p0b_robot_f4");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 3000)}));
  const TickRun run1 = run(j, 5000);
  EXPECT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  EXPECT_TRUE(faultOf(j).empty());
  record("f4_total_ms", run1.ms);
}

// F5 (S): the DONE late, OUTSIDE the timeout -> 305; the late DONE never turns the FAILED result around.
TEST(P0bMachineNodes, F5_ALateDoneOutsideTheTimeoutStays305)
{
  const std::string s = "f5_turner";
  ScriptedSource src(s, {100, 2000});
  Robot r("p0b_robot_f5");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 1000)}));
  const TickRun run1 = run(j, 4000);
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 305);
  ASSERT_TRUE(eventually([&] { return src.completed() == 1; }, 4000)) << "the late DONE";
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(r.machines->jobStationCode(), 305) << "the late DONE never turns the result around";
  EXPECT_FALSE(r.machines->outstanding(s)->completed);
  EXPECT_EQ(faultOf(j).find("MACHINE f5_turner TIMEOUT"), 0u) << faultOf(j);
  r.endJob();
}

// F7 (S): the status source stops publishing during a wait (killed, its publisher lingering) -> STATION_UNAVAILABLE 310
// within channel_status_max_age_ms + 1 period; MACHINE UNAVAILABLE (no status).
TEST(P0bMachineNodes, F7_ASilentSourceIs310WithinTheMaxAge)
{
  const std::string s = "f7_turner";
  ScriptedSource src(s, {100, 5000});
  Robot r("p0b_robot_f7");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  bool killed = false;
  Clock::time_point silenced{};
  const TickRun run1 = run(j, 6000, [&](double) {
    if (!killed && r.hasActivity("BUSY"))
    {
      src.silence(true);
      silenced = Clock::now();
      killed = true;
    }
    return true;
  });
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  const std::string text = r.machines->jobStationText();
  EXPECT_NE(text.find("MACHINE f7_turner UNAVAILABLE -- STATION_UNAVAILABLE (310)"), std::string::npos) << text;
  EXPECT_NE(text.find("no status"), std::string::npos) << text;
  ASSERT_TRUE(killed);
  // the matrix's bound: 310 within the max age + 1 period of the source going silent
  const double detect = msBetween(silenced, run1.end);
  record("f7_silent_to_310_ms", detect);
  EXPECT_LE(detect, 1000.0 + 100.0) << "max age + 1 period";
  // never early: the max age runs from the last heartbeat CHANGE the robot received (the last periodic image; an
  // on-change image of the same heartbeat may follow it)
  const auto last_image = src.lastPublishedImage();
  ASSERT_TRUE(last_image);
  const auto last_beat = src.firstPublished(
    [hb = last_image->heartbeat](const StatusMsg& m) { return m.heartbeat == hb; });
  ASSERT_TRUE(last_beat);
  record("f7_last_heartbeat_change_to_310_ms", msBetween(*last_beat, run1.end));
  EXPECT_GE(msBetween(*last_beat, run1.end), 1000.0);
  EXPECT_LE(msBetween(*last_beat, run1.end), 1000.0 + 100.0 + 10.0) << "max age + 1 period + 1 tick";
}

// F8 (S): the job CANCELLED while the station is BUSY: the wait halted <= 1 tick; the station finishes its own cycle;
// the NEXT job's request while still BUSY -> STATION_BUSY 313 (nothing written); after DONE -> it succeeds.
TEST(P0bMachineNodes, F8_ACancelHaltsTheWaitTheNextRequestIs313ThenSucceeds)
{
  const std::string s = "f8_turner";
  ScriptedSource src(s, {100, 1500});
  Robot r("p0b_robot_f8");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun cancelled = run(a, 5000, [&](double) { return !r.hasActivity("BUSY"); });
  r.endJob();
  ASSERT_TRUE(cancelled.halted);
  record("f8_halt_ms", cancelled.halt_ms);
  EXPECT_LE(cancelled.halt_ms, 10.0) << "halted within one tick";
  EXPECT_EQ(r.machines->jobStationCode(), 0) << "a halt is no machine failure";
  EXPECT_EQ(src.image().state, StatusMsg::STATE_BUSY) << "the robot never cancels the machine's cycle";
  // the next job, at once, while still BUSY
  const int images = src.requestImages();
  r.beginJob();
  Job b = r.tree(request(s));
  const TickRun busy = run(b, 3000);
  r.endJob();
  EXPECT_EQ(busy.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  EXPECT_NE(r.machines->jobStationText().find("MACHINE f8_turner FAULT -- STATION_BUSY (313)"), std::string::npos);
  EXPECT_EQ(src.requestImages(), images) << "nothing written";
  // the station finishes its own cycle; then the next request succeeds
  ASSERT_TRUE(eventually([&] { return src.completed() == 1; }, 4000));
  r.beginJob();
  Job c = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun after = run(c, 6000);
  r.endJob();
  EXPECT_EQ(after.status, BT::NodeStatus::SUCCESS) << faultOf(c);
  EXPECT_EQ(src.accepted(), 2);
  EXPECT_EQ(src.completed(), 2);
}

// F9 (S): a request with no live status source -> 310 at once (its first tick), nothing written.
TEST(P0bMachineNodes, F9_NoSourceIs310AtOnceNothingWritten)
{
  const std::string s = "f9_turner";
  Robot r("p0b_robot_f9");
  auto probe = std::make_shared<rclcpp::Node>("p0b_probe_f9", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  std::atomic<int> written{0};
  auto sub = probe->create_subscription<RequestMsg>(s + "/request", channelQos(),
                                                    [&](RequestMsg::ConstSharedPtr) { ++written; });
  rclcpp::executors::SingleThreadedExecutor pexec;
  pexec.add_node(probe);
  std::thread pspin([&] { pexec.spin(); });
  r.prepare(s);
  std::this_thread::sleep_for(1200ms);   // the runner's settle (> the max age)
  r.beginJob();
  Job j = r.tree(request(s));
  const TickRun run1 = run(j, 3000);
  std::this_thread::sleep_for(300ms);
  pexec.cancel();
  pspin.join();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(run1.ticks, 1) << "at once";
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  EXPECT_NE(r.machines->jobStationText().find("no status for service 'f9_turner'"), std::string::npos)
    << r.machines->jobStationText();
  EXPECT_EQ(written.load(), 0) << "nothing written";
  EXPECT_TRUE(r.events().empty()) << "a request not written is no PROCESS_EVENT";
  record("f9_decided_ms", run1.ms);
}

// =====================================================================================================================
// PROTOCOL
// =====================================================================================================================
// PB-P1 (S): the COMPLETED cycle of the previous request is still visible (DONE) when the next request is written: the
// wait never succeeds on it, only on the cycle of its own request identity.
TEST(P0bMachineNodes, PB_P1_TheDoneOfThePreviousRequestNeverMatches)
{
  const std::string s = "pbp1_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_pbp1");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  ASSERT_EQ(run(a, 5000).status, BT::NodeStatus::SUCCESS);
  r.endJob();
  ASSERT_EQ(src.image().state, StatusMsg::STATE_DONE);
  src.setConsumeDelay(600);   // the next request is taken late: the DONE of the previous one stays visible
  r.beginJob();
  Job b = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun run2 = run(b, 5000);
  r.endJob();
  ASSERT_EQ(run2.status, BT::NodeStatus::SUCCESS) << faultOf(b);
  const auto requested = r.firstActivity("REQUESTED");
  const auto waiting = r.firstActivity("WAITING FOR MACHINE pbp1_turner");
  ASSERT_TRUE(requested && waiting) << ::testing::PrintToString(r.activity());
  const double took = msBetween(*requested, run2.end);
  record("pbp1_second_request_to_success_ms", took);
  EXPECT_GE(took, 600.0 + 300.0 - 20.0) << "it waited for ITS cycle (consumed after 600 ms, 300 ms long)";
  EXPECT_EQ(src.accepted(), 2);
}

// PB-P2 (S): (a) a request while the observed state is BUSY -> 313, nothing written; (b) a request that reaches a BUSY
// source (scripted: it became busy between the robot's view and the request's consumption) -> REJECTED BUSY: no second
// cycle, the running cycle continues to its DONE, the rejected request's waiter 313 at once; cycles = accepted ones.
TEST(P0bMachineNodes, PB_P2a_ARequestWhileBusyIs313NothingWritten)
{
  const std::string s = "pbp2a_turner";
  ScriptedSource src(s, {100, 3000});
  Robot r("p0b_robot_pbp2a");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  src.injectRequest(Identity{0x0BAD, 1});   // the machine is busy with another cycle
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().state == StatusMsg::STATE_BUSY; }));
  r.beginJob();
  Job j = r.tree(request(s));
  const TickRun run1 = run(j, 3000);
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(run1.ticks, 1);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  EXPECT_NE(r.machines->jobStationText().find("the machine was still busy (BUSY)"), std::string::npos);
  EXPECT_EQ(src.requestImages(), 0) << "nothing written";
  EXPECT_TRUE(r.events().empty());
}

TEST(P0bMachineNodes, PB_P2b_ARequestReachingABusySourceIsRejectedTheCycleContinues)
{
  const std::string s = "pbp2b_turner";
  ScriptedSource src(s, {100, 1000, 300});
  Robot r("p0b_robot_pbp2b");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  bool injected = false;
  const Identity other{0x0BAD, 7};
  const TickRun run1 = run(j, 5000, [&](double) {
    if (!injected && r.hasActivity("REQUESTED"))
    {
      src.injectRequest(other);   // accepted at once: BUSY before the robot's request is consumed (300 ms)
      injected = true;
    }
    return true;
  });
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  EXPECT_NE(r.machines->jobStationText().find("the machine was still busy and did not take the request"),
            std::string::npos)
    << r.machines->jobStationText();
  const auto rejected = src.firstPublished([&](const StatusMsg& m) { return m.reject_requester == r.machines->requesterSession(); });
  ASSERT_TRUE(rejected);
  record("pbp2b_rejection_to_313_ms", msBetween(*rejected, run1.end));
  EXPECT_LE(msBetween(*rejected, run1.end), 110.0) << "at once";
  // the RUNNING cycle continues untouched, its own waiter (the scripted requester of `other`) sees its DONE
  ASSERT_TRUE(eventually([&] {
    const auto m = src.image();
    return m.cycle_requester == other.requester && m.cycle_seq == other.seq && m.cycle_done;
  }, 3000));
  EXPECT_EQ(src.accepted(), 1) << "no second cycle: the cycle counter = the accepted requests";
  EXPECT_EQ(src.completed(), 1);
}

// PB-P5 (S): two publishers on the service's status -> UNAVAILABLE ("two sources"), nothing taken as success -- not
// even a DONE of the request's own identity published by the second one.
TEST(P0bMachineNodes, PB_P5_TwoStatusPublishersAreUnavailable)
{
  const std::string s = "pbp5_turner";
  ScriptedSource src(s, {100, 4000});
  Robot r("p0b_robot_pbp5");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  auto second = std::make_shared<rclcpp::Node>("p0b_second_source", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  rclcpp::Publisher<StatusMsg>::SharedPtr pub2;
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  bool counted = false;
  const TickRun run1 = run(j, 6000, [&](double) {
    if (!pub2 && r.hasActivity("BUSY")) pub2 = second->create_publisher<StatusMsg>(s + "/status", channelQos());
    if (pub2 && !counted && r.machines->feed(s)->statusPublishers() == 2) counted = true;
    if (counted)
    {
      // the second source claims the request DONE
      StatusMsg fake = src.image();
      const auto w = r.machines->lastWritten(s);
      fake.cycle_requester = w->id.requester;
      fake.cycle_seq = w->id.seq;
      fake.cycle_done = true;
      fake.state = StatusMsg::STATE_DONE;
      ++fake.heartbeat;
      pub2->publish(fake);
    }
    return true;
  });
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  EXPECT_NE(r.machines->jobStationText().find("two sources publish its status (a crashed source's endpoint leaves the "
                                               "ROS graph within about 20 s)"),
            std::string::npos)
    << r.machines->jobStationText();
  // and a request while two sources publish
  r.beginJob();
  Job k = r.tree(request(s));
  EXPECT_EQ(run(k, 3000).status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  r.endJob();
  pub2.reset();
}

// PB-P5, a variant (protocol review MINOR-1): the second status source publishes with its OWN session -- its image
// reaches the waiting robot with two status publishers counted. The wait's rule 2 (the session is not the request's)
// is checked BEFORE liveness and keeps its code 310, but its words say "two sources", never "the machine restarted";
// nothing is taken as success (the second source even claims the request's DONE). The tick loop is held while the
// second publisher is counted and its image arrives (the runners tick between those moments as well).
TEST(P0bMachineNodes, PB_P5_ASecondSourceWithItsOwnSessionIsTwoSourcesNotARestart)
{
  const std::string s = "pbp5s_turner";
  ScriptedSource src(s, {100, 4000});
  Robot r("p0b_robot_pbp5s");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  auto second =
    std::make_shared<rclcpp::Node>("p0b_second_source_s", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  rclcpp::Publisher<StatusMsg>::SharedPtr pub2;
  StatusMsg own;   // the second source: its own session, its own heartbeat, a forged DONE of the robot's request
  own.source_session = src.session() == 0xFFFFFFFFu ? 1u : src.session() + 1u;
  own.state = StatusMsg::STATE_DONE;
  own.cycle_done = true;
  bool counted = false;
  bool arrived = false;
  Clock::time_point forged{};
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun run1 = run(j, 6000, [&](double) {
    if (!pub2 && r.hasActivity("BUSY"))
    {
      pub2 = second->create_publisher<StatusMsg>(s + "/status", channelQos());
      counted = eventually([&] { return r.machines->feed(s)->statusPublishers() == 2; }, 3000);
      const auto w = r.machines->lastWritten(s);
      own.cycle_requester = w->id.requester;
      own.cycle_seq = w->id.seq;
      arrived = eventually(
        [&] {
          ++own.heartbeat;
          pub2->publish(own);
          return r.machines->feed(s)->last().source_session == own.source_session;
        },
        3000);
      forged = Clock::now();
    }
    return true;
  });
  r.endJob();
  ASSERT_TRUE(counted) << "the second status publisher was counted";
  ASSERT_TRUE(arrived) << "its own-session image reached the robot";
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE) << "nothing taken as success";
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  const std::string text = r.machines->jobStationText();
  EXPECT_NE(text.find("MACHINE pbp5s_turner UNAVAILABLE -- STATION_UNAVAILABLE (310): two sources publish its status "
                      "(a crashed source's endpoint leaves the ROS graph within about 20 s)"),
            std::string::npos)
    << text;
  EXPECT_EQ(text.find("restarted"), std::string::npos) << text;
  record("pbp5s_forged_to_310_ms", msBetween(forged, run1.end));
  EXPECT_LE(msBetween(forged, run1.end), 50.0) << "decided at the next tick";
  EXPECT_EQ(src.accepted(), 1);
  pub2.reset();
}

// PB-P6 (S): the seq from an INJECTED counter in a real run: 2^32-1, then 1 (0 skipped), in order across the wrap; a
// second request before the first is consumed -> 313, nothing written (never two requests with one seq, never a
// depth-1 overwrite).
TEST(P0bMachineNodes, PB_P6_TheSeqWrapsInARealRunAndASecondUnconsumedRequestIs313)
{
  const std::string s = "pbp6_turner";
  ScriptedSource src(s, {100, 200});
  trainit::MachineChannels::Options o;
  o.initial_seq = UINT32_MAX;
  Robot r("p0b_robot_pbp6", o);
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  for (int k = 0; k < 2; ++k)
  {
    r.beginJob();
    Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
    ASSERT_EQ(run(j, 5000).status, BT::NodeStatus::SUCCESS) << k << ": " << faultOf(j);
    r.endJob();
  }
  src.setConsumeDelay(400);
  const int images = src.requestImages();
  r.beginJob();
  Job twice = r.tree(seq({request(s, "first"), request(s, "second")}));
  const TickRun run2 = run(twice, 3000);
  r.endJob();
  EXPECT_EQ(run2.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  EXPECT_NE(r.machines->jobStationText().find("it has not yet taken the previous request"), std::string::npos)
    << r.machines->jobStationText();
  // protocol review MINOR-2: §3.5's words, the parenthetical and how long ago the previous request was written (the
  // same tick here: 0.0 s)
  EXPECT_TRUE(std::regex_search(r.machines->jobStationText(),
                                std::regex("MACHINE pbp6_turner FAULT -- STATION_BUSY \\(313\\): the machine was still "
                                           "busy \\(it has not yet taken the previous request, written "
                                           "[0-9]+\\.[0-9] s ago\\); nothing was requested$")))
    << r.machines->jobStationText();
  ASSERT_TRUE(eventually([&] { return src.consumed().size() == 3u; }, 3000));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(src.requestImages(), images + 1) << "the second request was never written";
  const auto ids = src.consumed();
  ASSERT_EQ(ids.size(), 3u);
  EXPECT_EQ(ids[0].seq, UINT32_MAX);
  EXPECT_EQ(ids[1].seq, 1u) << "0 skipped across the wrap";
  EXPECT_EQ(ids[2].seq, 2u);
  for (const auto& id : ids) EXPECT_EQ(id.requester, r.machines->requesterSession());
}

// PB-P7 (S): a hand-written wait for done with no earlier request on that service fails at run time, without a station
// code (D4: TASK_FAILED 301 at the RTI), the detail saying why.
TEST(P0bMachineNodes, PB_P7_AWaitWithoutARequestFailsAtRunTime)
{
  const std::string s = "pbp7_turner";
  ScriptedSource src(s, {100, 200});
  Robot r("p0b_robot_pbp7");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(waitFor(s, "done", 2000));
  const TickRun run1 = run(j, 3000);
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(run1.ticks, 1);
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  EXPECT_NE(r.machines->jobStationText().find("a wait for done with no request on it in this job"), std::string::npos);
}

// PB-P10 (S): the requesting process restarts (a new requester session) while the station is BUSY with the dead
// process's request: its first request synchronises (>= 2 heartbeat changes), sees BUSY -> 313; after DONE its request
// runs a NEW cycle; its wait never succeeds on the dead process's cycle.
TEST(P0bMachineNodes, PB_P10_ARestartedRequesterNeverSucceedsOnTheDeadProcesssCycle)
{
  const std::string s = "pbp10_turner";
  ScriptedSource src(s, {100, 2000});
  uint32_t dead_session = 0;
  {
    Robot dead("p0b_robot_pbp10_a");
    dead.prepare(s);
    ASSERT_TRUE(dead.settle(s));
    dead.beginJob();
    Job j = dead.tree(request(s));
    ASSERT_EQ(run(j, 3000).status, BT::NodeStatus::SUCCESS);
    dead_session = dead.machines->requesterSession();
    ASSERT_TRUE(eventually([&] { return src.image().state == StatusMsg::STATE_BUSY; }));
  }   // the process ends (orderly, D11): its request publisher leaves the graph
  ASSERT_TRUE(eventually([&] { return src.node()->count_publishers("/p0b/" + s + "/request") == 0; }, 5000));
  Robot r("p0b_robot_pbp10_b");
  EXPECT_NE(r.machines->requesterSession(), dead_session);
  r.prepare(s);
  r.beginJob();
  Job first = r.tree(request(s));   // at once: no settle
  const TickRun synced = run(first, 3000);
  r.endJob();
  EXPECT_EQ(synced.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313) << r.machines->jobStationText();
  EXPECT_GT(synced.ticks, 1) << "it synchronised first";
  record("pbp10_sync_ms", synced.ms);
  EXPECT_LE(synced.ms, 1000.0 + 20.0);
  ASSERT_TRUE(eventually([&] { return src.completed() == 1; }, 4000));
  src.setConsumeDelay(500);   // the dead process's DONE stays visible after the new request is written
  r.beginJob();
  Job second = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun run2 = run(second, 6000);
  r.endJob();
  ASSERT_EQ(run2.status, BT::NodeStatus::SUCCESS) << faultOf(second);
  const auto requested = r.firstActivity("REQUESTED");
  ASSERT_TRUE(requested);
  EXPECT_GE(msBetween(*requested, run2.end), 500.0 + 2000.0 - 20.0) << "its own cycle, never the dead one's";
  EXPECT_EQ(src.accepted(), 2);
  EXPECT_EQ(src.consumed().back().requester, r.machines->requesterSession());
}

// PB-P11 (S): a second publisher on the service's request -> the robot's request 310; the source rejects every new
// request TWO_REQUESTERS; no cycle runs.
TEST(P0bMachineNodes, PB_P11_ASecondRequesterIsUnavailableNoCycleRuns)
{
  const std::string s = "pbp11_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_pbp11");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  auto other = std::make_shared<rclcpp::Node>("p0b_second_requester", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  auto pub2 = other->create_publisher<RequestMsg>(s + "/request", channelQos());
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->requestPublishers() == 2; }));
  r.beginJob();
  Job j = r.tree(request(s));
  const TickRun run1 = run(j, 3000);
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  EXPECT_NE(r.machines->jobStationText().find("another requester also writes its requests (one robot per machine "
                                               "service; a crashed requester's endpoint leaves the ROS graph within "
                                               "about 20 s)"),
            std::string::npos)
    << r.machines->jobStationText();
  // the second requester's request reaches the source: rejected TWO_REQUESTERS, never executed
  std::this_thread::sleep_for(200ms);   // the source counted both publishers
  RequestMsg req;
  req.source_session = src.session();
  req.requester_session = 0x2222;
  req.seq = 1;
  pub2->publish(req);
  ASSERT_TRUE(eventually([&] { return src.image().reject_code == StatusMsg::REJECT_TWO_REQUESTERS; }, 3000));
  EXPECT_EQ(src.accepted(), 0) << "no cycle runs";
}

// PB-P11's aftermath (S; the M-tier finding of P0.b WP6, M run 1): the robot's request was REJECTED (it saw the
// rejection), then ANOTHER requester's request was rejected too -- `reject_*` now names the other one. CONSUMED is an
// event (rule 3, latched in the feed): the robot's next request is written and its cycle runs, never a 313 lock-out
// ("still busy, not yet taken") that only a source restart would end.
TEST(P0bMachineNodes, PB_P11_AnotherRequestersLaterRejectionNeverLocksTheRobotOut)
{
  const std::string s = "pbp11b_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_pbp11b");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  src.forceTwoRequesters(true);                      // the source counts two requesters: every new request rejected
  r.beginJob();
  Job j1 = r.tree(seq({request(s), waitFor(s, "done", 2000)}));
  const TickRun run1 = run(j1, 4000);
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310) << r.machines->jobStationText();
  EXPECT_NE(r.machines->jobStationText().find("saw two requesters"), std::string::npos) << r.machines->jobStationText();
  src.injectRequest(Identity{0x2222, 1});           // another requester's request, rejected as well
  ASSERT_TRUE(eventually([&] { return src.image().reject_requester == 0x2222; }, 3000));
  src.forceTwoRequesters(false);                     // the second requester is gone
  std::this_thread::sleep_for(300ms);                // the robot has seen the displacing image
  r.beginJob();
  Job j2 = r.tree(seq({request(s), waitFor(s, "done", 2000)}));
  const TickRun run2 = run(j2, 4000);
  r.endJob();
  EXPECT_EQ(run2.status, BT::NodeStatus::SUCCESS) << r.machines->jobStationText();
  EXPECT_EQ(r.machines->jobStationCode(), 0) << r.machines->jobStationText();
  EXPECT_EQ(src.accepted(), 1) << "the robot's NEXT request ran its cycle";
}

// PB-P12 (S): request then wait for READY: it succeeds only after THAT request's cycle completed, never on the status
// of before it (B1).
TEST(P0bMachineNodes, PB_P12_AReadyWaitAfterARequestNeedsItsCompletion)
{
  const std::string s = "pbp12_turner";
  ScriptedSource src(s, {100, 600, 400});
  Robot r("p0b_robot_pbp12");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "ready", 8000)}));
  const TickRun run1 = run(j, 5000);
  ASSERT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  const auto requested = r.firstActivity("REQUESTED");
  const auto done = src.firstPublished([](const StatusMsg& m) { return m.cycle_done; });
  ASSERT_TRUE(requested && done);
  EXPECT_GE(msBetween(*requested, run1.end), 400.0 + 600.0 - 20.0) << "never on the READY of before the request";
  EXPECT_GE(run1.end, *done);
  EXPECT_LE(msBetween(*done, run1.end), 110.0);
  EXPECT_TRUE(r.hasActivity("WAITING FOR MACHINE pbp12_turner"));
  EXPECT_TRUE(r.hasActivity("MACHINE pbp12_turner BUSY"));
  EXPECT_TRUE(r.machines->outstanding(s)->completed);
  EXPECT_EQ(src.accepted(), 1);
}

// PB-P14 (S): a job cancelled right after its request, the next job requests at once -> 313 (the previous request not
// yet consumed, or BUSY); the first request's cycle is not killed; no cycle runs twice.
TEST(P0bMachineNodes, PB_P14_ACancelledRequestStillBlocksTheNextJobsRequest)
{
  const std::string s = "pbp14_turner";
  ScriptedSource src(s, {100, 800, 300});
  Robot r("p0b_robot_pbp14");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun cancelled = run(a, 3000, [&](double) { return !r.hasActivity("REQUESTED"); });
  r.endJob();
  ASSERT_TRUE(cancelled.halted);
  r.beginJob();
  Job b = r.tree(request(s));
  const TickRun next = run(b, 3000);
  r.endJob();
  EXPECT_EQ(next.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  record("pbp14_next_request_ms", next.ms);
  ASSERT_TRUE(eventually([&] { return src.completed() == 1; }, 4000)) << "the first request's cycle runs to its DONE";
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(src.requestImages(), 1) << "the next job wrote nothing";
  EXPECT_EQ(src.accepted(), 1) << "no cycle runs twice";
}

// PB-P24 (S), D19 -- the owner's amendment of §3.4 (2026-11-19; protocol review MAJOR-2), reachable with ONE robot:
// job A writes its request and is cancelled at once; job B's FIRST step is a ready-wait; the source takes A's request
// late (consume_delay). B never succeeds on the READY of before A's request is taken: it waits ("it has not yet taken
// the previous request"); A's cycle's BUSY keeps it waiting; A's DONE is ready. Before D19, B succeeded at its first
// tick and A's cycle then started under the robot.
TEST(P0bMachineNodes, PB_P24_AReadyWaitAfterACancelledRequestWaitsForItToBeTaken)
{
  const std::string s = "pbp24_turner";
  ScriptedSource src(s, {100, 600, 300});
  Robot r("p0b_robot_pbp24");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun cancelled = run(a, 3000, [&](double) { return !r.hasActivity("REQUESTED"); });
  r.endJob();
  ASSERT_TRUE(cancelled.halted);
  const auto mine = r.machines->lastWritten(s);
  ASSERT_TRUE(mine) << "A's request was written";
  r.beginJob();
  Job b = r.tree(waitFor(s, "ready", 8000));
  const auto b0 = Clock::now();
  ASSERT_EQ(src.accepted(), 0) << "A's request is not yet taken when B starts (consume_delay 300 ms)";
  const TickRun ready = run(b, 5000);
  r.endJob();
  ASSERT_EQ(ready.status, BT::NodeStatus::SUCCESS) << faultOf(b);
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  const auto isA = [&](const StatusMsg& m) {
    return m.source_session == mine->source_session && m.cycle_requester == mine->id.requester &&
           m.cycle_seq == mine->id.seq;
  };
  const auto taken = src.firstPublished(isA);
  const auto done = src.firstPublished([&](const StatusMsg& m) { return isA(m) && m.cycle_done; });
  ASSERT_TRUE(taken && done) << "the source took A's request and completed its cycle";
  EXPECT_GE(*taken, b0) << "A's request was taken AFTER B's ready-wait started (the case's condition)";
  EXPECT_GE(ready.end, *done) << "never ready before A's cycle completed (not taken, then its BUSY, kept B waiting)";
  EXPECT_LE(msBetween(*done, ready.end), 110.0) << "ready within a period and a tick of A's DONE";
  EXPECT_TRUE(r.hasActivity("WAITING FOR MACHINE pbp24_turner (it has not yet taken the previous request)"));
  EXPECT_TRUE(r.hasActivity("WAITING FOR MACHINE pbp24_turner (it is BUSY)"));
  EXPECT_EQ(src.accepted(), 1) << "A's cycle ran once";
  record("pbp24_ready_after_done_ms", msBetween(*done, ready.end));
  record("pbp24_wait_ms", ready.ms);
}

// PB-P15 (S): the request's cycle COMPLETES, then an idle FAULT, before the wait reads the status -> the wait SUCCEEDS
// (DONE of the request wins); the next request -> 306.
TEST(P0bMachineNodes, PB_P15_ACompletedCycleWinsOverALaterFault)
{
  const std::string s = "pbp15_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_pbp15");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), sleepMs(1200), waitFor(s, "done", 8000)}));
  bool faulted = false;
  const TickRun run1 = run(j, 5000, [&](double) {
    if (!faulted && src.completed() == 1)
    {
      src.faultNow(StatusMsg::FAULT_STATION_FAULT);   // idle: cycle_done stays true
      faulted = true;
    }
    return true;
  });
  r.endJob();
  ASSERT_TRUE(faulted);
  EXPECT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  r.beginJob();
  Job k = r.tree(request(s));
  EXPECT_EQ(run(k, 3000).status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 306);
  r.endJob();
}

// PB-P16 (S): job A requests and ends; job B's hand-written wait for done with no request of its own fails (no
// outstanding request in this job), never matching job A's (its DONE visible).
TEST(P0bMachineNodes, PB_P16_AJobNeverWaitsOnAnEarlierJobsRequest)
{
  const std::string s = "pbp16_turner";
  ScriptedSource src(s, {100, 200});
  Robot r("p0b_robot_pbp16");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(request(s));
  ASSERT_EQ(run(a, 3000).status, BT::NodeStatus::SUCCESS);
  r.endJob();
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().cycle_done; }, 3000)) << "job A's DONE visible";
  r.beginJob();
  Job b = r.tree(waitFor(s, "done", 2000));
  const TickRun run2 = run(b, 3000);
  r.endJob();
  EXPECT_EQ(run2.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(run2.ticks, 1);
  EXPECT_EQ(r.machines->jobStationCode(), 0);
  EXPECT_NE(r.machines->jobStationText().find("no request on it in this job"), std::string::npos);
}

// PB-P17 (S): a source restart during a wait for READY: (a) with no outstanding request -> succeeds once the restarted
// source is READY; (b) with one -> 310.
TEST(P0bMachineNodes, PB_P17a_ARestartWithoutARequestEndsTheReadyWait)
{
  const std::string s = "pbp17a_turner";
  ScriptedSource src(s, {100, 5000});
  Robot r("p0b_robot_pbp17a");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  src.injectRequest(Identity{0x0BAD, 3});   // busy with the machine master's cycle
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().state == StatusMsg::STATE_BUSY; }));
  r.beginJob();
  Job j = r.tree(waitFor(s, "ready", 5000));
  bool restarted = false;
  Clock::time_point restart_at{};
  const TickRun run1 = run(j, 6000, [&](double ms) {
    if (!restarted && ms > 500)
    {
      src.restart();
      restart_at = Clock::now();
      restarted = true;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::SUCCESS) << faultOf(j);
  ASSERT_TRUE(restarted);
  record("pbp17a_restart_to_success_ms", msBetween(restart_at, run1.end));
  EXPECT_LE(msBetween(restart_at, run1.end), 110.0);
}

TEST(P0bMachineNodes, PB_P17b_ARestartWithARequestPendingIs310)
{
  const std::string s = "pbp17b_turner";
  ScriptedSource src(s, {100, 5000});
  Robot r("p0b_robot_pbp17b");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "ready", 8000)}));
  bool restarted = false;
  Clock::time_point restart_at{};
  const TickRun run1 = run(j, 6000, [&](double) {
    if (!restarted && r.hasActivity("BUSY"))
    {
      src.restart();
      restart_at = Clock::now();
      restarted = true;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  EXPECT_NE(r.machines->jobStationText().find("the machine restarted"), std::string::npos);
  ASSERT_TRUE(restarted);
  EXPECT_LE(msBetween(restart_at, run1.end), 110.0) << "at once";
}

// PB-P3 (S): the status source restarts after a request, before its DONE: a new session; the waiter 310 at once
// ("restarted"), never a stale success; the restarted source never executes the level-held request of the old
// session; the next request (new session) succeeds.
TEST(P0bMachineNodes, PB_P3_ASourceRestartDuringAnOutstandingRequest)
{
  const std::string s = "pbp3_turner";
  ScriptedSource src(s, {100, 3000});
  Robot r("p0b_robot_pbp3");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  bool restarted = false;
  Clock::time_point restart_at{};
  const TickRun run1 = run(a, 6000, [&](double) {
    if (!restarted && r.hasActivity("BUSY"))
    {
      src.restart();
      restart_at = Clock::now();
      restarted = true;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 310);
  EXPECT_NE(r.machines->jobStationText().find("the machine restarted (a new session)"), std::string::npos);
  ASSERT_TRUE(restarted);
  record("pbp3_restart_to_310_ms", msBetween(restart_at, run1.end));
  EXPECT_LE(msBetween(restart_at, run1.end), 110.0);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(src.accepted(), 1) << "the old session's level-held request is never executed again";
  r.beginJob();
  Job b = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  src.setCycleMs(300);
  EXPECT_EQ(run(b, 5000).status, BT::NodeStatus::SUCCESS) << faultOf(b);
  r.endJob();
  EXPECT_EQ(src.accepted(), 2);
}

// PB-P18 (S): a run whose FIRST step is a request, the registry just subscribed: the request synchronises (>= 2
// heartbeat changes, within the max age) and succeeds -- never a start-up 310.
TEST(P0bMachineNodes, PB_P18_AFirstRequestRightAfterTheSubscriptionSynchronises)
{
  const std::string s = "pbp18_turner";
  ScriptedSource src(s, {100, 300});
  std::this_thread::sleep_for(300ms);   // the source is up
  Robot r("p0b_robot_pbp18");
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const auto subscribed = Clock::now();
  r.prepare(s);   // just subscribed: no settle, the request is the first tick
  const TickRun run1 = run(j, 5000);
  r.endJob();
  ASSERT_EQ(run1.status, BT::NodeStatus::SUCCESS) << r.machines->jobStationText();
  EXPECT_GT(run1.ticks, 2);
  const auto requested = r.firstActivity("REQUESTED");
  ASSERT_TRUE(requested);
  const double sync_ms = msBetween(subscribed, *requested);
  record("pbp18_sync_ms", sync_ms);
  EXPECT_GE(sync_ms, 100.0) << ">= 2 heartbeat changes of a 100 ms period after the baseline image";
  EXPECT_LE(sync_ms, 1000.0) << "within the max age";
}

// PB-P19 (S): the source restarts healthily between two jobs; the next job's request synchronises on the NEW session and
// succeeds (no RTI RESET needed).
TEST(P0bMachineNodes, PB_P19_ARestartBetweenJobsTheNextRequestSucceeds)
{
  const std::string s = "pbp19_turner";
  ScriptedSource src(s, {100, 300});
  Robot r("p0b_robot_pbp19");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job a = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  ASSERT_EQ(run(a, 5000).status, BT::NodeStatus::SUCCESS);
  r.endJob();
  const uint32_t old_session = src.session();
  src.restart();   // healthily, BETWEEN two jobs: the next job starts once the restarted source has published
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().source_session == src.session(); }));
  r.beginJob();
  Job b = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun run2 = run(b, 5000);
  r.endJob();
  ASSERT_EQ(run2.status, BT::NodeStatus::SUCCESS) << r.machines->jobStationText();
  const auto requested = r.firstActivity("REQUESTED");
  ASSERT_TRUE(requested);
  const double synced = msBetween(run2.t0, *requested);
  record("pbp19_sync_on_the_new_session_ms", synced);
  EXPECT_GT(run2.ticks, 2) << "it synchronised on the NEW session (two heartbeat changes of it) before writing";
  EXPECT_GE(synced, 50.0);
  EXPECT_NE(src.session(), old_session);
  EXPECT_EQ(r.machines->lastWritten(s)->source_session, src.session()) << "written FOR the new session";
  EXPECT_EQ(src.accepted(), 2);
}

// PB-P20 (S): a request's rejection displaced by another rejection before its waiter read it (two requesters,
// scripted) -> the waiter ends STATION_TIMEOUT 305 at its timeout (bounded: the stated outcome).
TEST(P0bMachineNodes, PB_P20_ADisplacedRejectionEndsAtTheTimeout)
{
  const std::string s = "pbp20_turner";
  ScriptedSource src(s, {100, 5000, 300});
  src.displaceNextRejection(Identity{0x0C0C, 9});
  Robot r("p0b_robot_pbp20");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 1500)}));
  bool injected = false;
  const TickRun run1 = run(j, 4000, [&](double) {
    if (!injected && r.hasActivity("REQUESTED"))
    {
      src.injectRequest(Identity{0x0BAD, 1});   // BUSY before the robot's request is consumed
      injected = true;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 305);
  EXPECT_NE(r.machines->jobStationText().find("the request not yet taken after 1.5 s (the machine is BUSY)"),
            std::string::npos)
    << r.machines->jobStationText();
  const auto requested = r.firstActivity("REQUESTED");
  ASSERT_TRUE(requested);
  const double fired = msBetween(*requested, run1.end);
  record("pbp20_timeout_fired_ms", fired);
  EXPECT_GE(fired, 1500.0);
  EXPECT_LE(fired, 1610.0);
}

// PB-P20 FOLLOWED by the robot's next request (protocol review MAJOR-1): no image ever named the robot's displaced
// request, so for the robot it stays "not yet taken" -- its next request on the SAME source session is refused 313 and
// nothing is written: today's bounded, fail-closed outcome (never a second request while one may still be unconsumed).
// The scripted source mirrors the cell's rule-2 refinement: the robot's latched image delivered again after another
// requester's identity was consumed is a no-op (consumed once, never executed later -- without it, the re-delivery
// would be consumed a second time and could run a cycle nobody waits for). A source restart (a new session) lets the
// next request through.
TEST(P0bMachineNodes, PB_P20_ADisplacedRejectionThenTheNextRequestIs313UntilTheSourceRestarts)
{
  const std::string s = "pbp20n_turner";
  ScriptedSource src(s, {100, 2000, 300});
  src.displaceNextRejection(Identity{0x0C0C, 9});
  Robot r("p0b_robot_pbp20n");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 1500)}));
  bool injected = false;
  const TickRun run1 = run(j, 4000, [&](double) {
    if (!injected && r.hasActivity("REQUESTED"))
    {
      src.injectRequest(Identity{0x0BAD, 1});   // BUSY before the robot's request is consumed
      injected = true;
    }
    return true;
  });
  r.endJob();
  ASSERT_EQ(run1.status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(r.machines->jobStationCode(), 305) << "PB-P20 itself: " << r.machines->jobStationText();
  const auto mine = r.machines->lastWritten(s);
  ASSERT_TRUE(mine);
  ASSERT_TRUE(eventually([&] { return src.consumed().size() == 3u; }, 2000));
  EXPECT_EQ(src.consumed()[1], mine->id) << "the robot's request was consumed (rejected BUSY) ...";
  EXPECT_EQ(src.consumed()[2], (Identity{0x0C0C, 9})) << "... and its rejection displaced in the same scan";
  EXPECT_FALSE(r.machines->feed(s)->consumed(*mine)) << "no image ever named it";
  // the robot's level-held image delivered again (BUSY, then DONE): a no-op both times -- rule 2's per-requester memory
  src.redeliverLastRequest();
  ASSERT_TRUE(eventually([&] { return src.completed() == 1; }, 3000)) << "the machine master's cycle completes";
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().state == StatusMsg::STATE_DONE; }, 1000));
  src.redeliverLastRequest();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(src.consumed().size(), 3u) << "never consumed twice";
  EXPECT_EQ(src.accepted(), 1) << "only the machine master's cycle ran";
  // the robot's next request on the SAME source session: 313 at once, nothing written
  const int images = src.requestImages();
  r.beginJob();
  Job k = r.tree(request(s));
  const TickRun next = run(k, 3000);
  r.endJob();
  EXPECT_EQ(next.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(next.ticks, 1) << "decided at once";
  EXPECT_EQ(r.machines->jobStationCode(), 313) << r.machines->jobStationText();
  EXPECT_NE(r.machines->jobStationText().find("the machine was still busy (it has not yet taken the previous request, "
                                               "written "),
            std::string::npos)
    << r.machines->jobStationText();
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(src.requestImages(), images) << "nothing written";
  // a source restart (a NEW session): the next request is written for it and runs its cycle
  src.restart();
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().source_session == src.session(); }));
  src.setCycleMs(300);
  r.beginJob();
  Job m = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  const TickRun after = run(m, 5000);
  r.endJob();
  EXPECT_EQ(after.status, BT::NodeStatus::SUCCESS) << r.machines->jobStationText();
  EXPECT_EQ(r.machines->lastWritten(s)->source_session, src.session()) << "written FOR the new session";
  EXPECT_EQ(src.accepted(), 2);
  EXPECT_EQ(src.requestImages(), images + 1);
}

// PB-P22 (S): (a) the request's cycle FAULTs and the source is RESET before the wait for done starts -> 306 at once ("the
// cycle was aborted"), never a timeout; (b) a wait for ready on a REJECTED request -> 313 / 306 / 310 per the
// rejection, at once.
TEST(P0bMachineNodes, PB_P22a_AnAbortedCycleIs306AtOnce)
{
  const std::string s = "pbp22a_turner";
  ScriptedSource src(s, {100, 3000});
  Robot r("p0b_robot_pbp22a");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), sleepMs(1500), waitFor(s, "done", 8000)}));
  int step = 0;
  const TickRun run1 = run(j, 5000, [&](double ms) {
    if (step == 0 && ms > 300 && src.image().state == StatusMsg::STATE_BUSY)
    {
      src.faultNow(StatusMsg::FAULT_ACTUATOR_TIMEOUT);
      step = 1;
    }
    else if (step == 1 && ms > 600)
    {
      src.reset();   // HOMING, then READY: the aborted cycle's identity stays, not done
      step = 2;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 306);
  EXPECT_NE(r.machines->jobStationText().find("its cycle was aborted (the machine was reset before completing it)"),
            std::string::npos)
    << r.machines->jobStationText();
  EXPECT_FALSE(r.hasActivity("WAITING")) << "decided at the wait's first tick, never a timeout";
  EXPECT_LE(run1.ms, 1500.0 + 150.0);
}

TEST(P0bMachineNodes, PB_P22b_AReadyWaitOnARejectedRequestAnswersAtOnce)
{
  // the request is rejected while the robot moves (BUSY with another cycle / in FAULT, then RESET and homed / two
  // requesters); the ready-wait that follows answers by the REJECTION at its first tick -- never a wait for a
  // completion that can never come (N3), never a success on the READY that follows a reset
  struct Case
  {
    const char* name;
    uint16_t code;
  };
  for (const Case c : {Case{"busy", 313}, Case{"faulted", 306}, Case{"two", 310}})
  {
    const std::string s = std::string("pbp22b_") + c.name;
    ScriptedSource src(s, {100, 3000, 300, 200});
    Robot r(std::string("p0b_robot_pbp22b_") + c.name);
    r.prepare(s);
    ASSERT_TRUE(r.settle(s));
    r.beginJob();
    Job j = r.tree(seq({request(s), sleepMs(1200), waitFor(s, "ready", 8000)}));
    int step = 0;
    Clock::time_point stimulus{};
    const TickRun run1 = run(j, 5000, [&](double) {
      if (step == 0 && r.hasActivity("REQUESTED"))
      {
        if (c.code == 313) src.injectRequest(Identity{0x0BAD, 1});
        else if (c.code == 306) src.faultNow(StatusMsg::FAULT_SENSOR_FAULT);
        else src.forceTwoRequesters(true);
        stimulus = Clock::now();
        step = 1;
      }
      else if (step == 1 && c.code == 306 && msSince(stimulus) > 500)
      {
        src.reset();   // HOMING, then READY: only the rejection still names the request
        step = 2;
      }
      return true;
    });
    r.endJob();
    EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE) << c.name;
    EXPECT_EQ(r.machines->jobStationCode(), c.code) << c.name << ": " << r.machines->jobStationText();
    const uint32_t me = r.machines->requesterSession();
    const auto rejected = src.firstPublished([me](const StatusMsg& m) { return m.reject_requester == me; });
    ASSERT_TRUE(rejected) << c.name;
    EXPECT_LT(*rejected, run1.end) << c.name;
    EXPECT_FALSE(r.hasActivity("WAITING")) << c.name << ": decided at the ready-wait's first tick";
    record(std::string("pbp22b_") + c.name + "_total_ms", run1.ms);
    EXPECT_LE(run1.ms, 1200.0 + 150.0) << c.name << ": at once";
    if (c.code == 306)
    {
      EXPECT_EQ(src.image().state, StatusMsg::STATE_READY) << "the source was READY again";
    }
    EXPECT_EQ(src.accepted(), c.code == 313 ? 1 : 0) << c.name;
  }
}

// PB-P23 (S): a request while the source is HOMING after a RESET -> 313 (busy, not a fault), nothing written; after the
// homing a request runs one cycle.
TEST(P0bMachineNodes, PB_P23_ARequestWhileHomingIs313)
{
  const std::string s = "pbp23_turner";
  ScriptedSource src(s, {100, 300, 0, 1500});
  Robot r("p0b_robot_pbp23");
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  src.faultNow(StatusMsg::FAULT_STATION_FAULT);
  src.reset();
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().state == StatusMsg::STATE_HOMING; }));
  r.beginJob();
  Job j = r.tree(request(s));
  const TickRun run1 = run(j, 3000);
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 313);
  EXPECT_NE(r.machines->jobStationText().find("(HOMING after a reset)"), std::string::npos);
  EXPECT_EQ(src.requestImages(), 0) << "nothing written";
  ASSERT_TRUE(eventually([&] { return r.machines->feed(s)->last().state == StatusMsg::STATE_READY; }, 4000));
  r.beginJob();
  Job k = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  EXPECT_EQ(run(k, 5000).status, BT::NodeStatus::SUCCESS) << faultOf(k);
  r.endJob();
  EXPECT_EQ(src.accepted(), 1) << "one cycle: the request is never executed after the homing by itself";
}

// =====================================================================================================================
// CLOCKS
// =====================================================================================================================
class SimClock
{
public:
  explicit SimClock(double factor) : factor_(factor), t0_(Clock::now())
  {
    node_ = std::make_shared<rclcpp::Node>("p0b_clock", "/", rclcpp::NodeOptions().use_global_arguments(false));
    pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    thread_ = std::thread([this] {
      while (ticking_)
      {
        rosgraph_msgs::msg::Clock c;
        const double sim_s = 100.0 + factor_ * msSince(t0_) / 1000.0;
        c.clock.sec = static_cast<int32_t>(sim_s);
        c.clock.nanosec = static_cast<uint32_t>((sim_s - c.clock.sec) * 1e9);
        pub_->publish(c);
        std::this_thread::sleep_for(10ms);
      }
    });
  }
  ~SimClock()
  {
    ticking_ = false;
    thread_.join();
  }

private:
  double factor_;
  Clock::time_point t0_;
  std::atomic<bool> ticking_{true};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  std::thread thread_;
};

// CL2 (S): a wait's timeout under sim time at 0.5x -> 305 after timeout_ms of SIM time (about 2x wall), measured on both
// clocks.
TEST(P0bMachineNodes, CL2_AWaitTimeoutRunsOnTheRunnersSimClock)
{
  const std::string s = "cl2_turner";
  SimClock sim(0.5);
  ScriptedSource src(s, {100, 400});
  src.neverComplete(true);
  Robot r("p0b_robot_cl2", {}, true);
  ASSERT_TRUE(eventually([&] { return r.node->get_clock()->started(); }, 3000)) << "the robot's clock follows /clock";
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 1000)}));
  const int64_t sim0 = r.node->now().nanoseconds();
  const TickRun run1 = run(j, 6000);
  const int64_t sim1 = r.node->now().nanoseconds();
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(r.machines->jobStationCode(), 305) << r.machines->jobStationText();
  const double sim_ms = static_cast<double>(sim1 - sim0) / 1e6;
  record("cl2_timeout_sim_ms", sim_ms);
  record("cl2_timeout_wall_ms", run1.ms);
  EXPECT_GE(sim_ms, 1000.0);
  EXPECT_LE(sim_ms, 1000.0 + 60.0) << "1 period + 1 tick of wall at 0.5x";
  EXPECT_GE(run1.ms, 1900.0) << "about 2x on the wall clock";
  EXPECT_LE(run1.ms, 2300.0);
}

// CL3 (S): the liveness watchdog (STEADY clock) while the source's heartbeat runs on sim time at 0.5x (a heartbeat every
// 200 ms of wall time): no false UNAVAILABLE; the request synchronises and its DONE arrives on sim time.
TEST(P0bMachineNodes, CL3_ASlowSimHeartbeatIsNoFalseUnavailable)
{
  const std::string s = "cl3_turner";
  SimClock sim(0.5);
  ScriptedSource::Knobs k;
  k.cycle_ms = 1500;   // sim: 3 s of wall
  k.sim_time = true;
  ScriptedSource src(s, k);
  Robot r("p0b_robot_cl3", {}, true);
  ASSERT_TRUE(eventually([&] { return r.node->get_clock()->started(); }, 3000));
  r.prepare(s);
  ASSERT_TRUE(r.settle(s));
  r.beginJob();
  Job j = r.tree(seq({request(s), waitFor(s, "done", 8000)}));
  uint16_t last_hb = r.machines->feed(s)->last().heartbeat;
  Clock::time_point last_change = Clock::now();
  std::vector<double> gaps;
  const TickRun run1 = run(j, 8000, [&](double) {
    const auto f = r.machines->feed(s);
    if (f->last().heartbeat != last_hb)
    {
      gaps.push_back(msSince(last_change));
      last_change = Clock::now();
      last_hb = f->last().heartbeat;
    }
    return true;
  });
  r.endJob();
  EXPECT_EQ(run1.status, BT::NodeStatus::SUCCESS) << r.machines->jobStationText();
  ASSERT_GE(gaps.size(), 5u);
  const double max_gap = *std::max_element(gaps.begin(), gaps.end());
  double mean = 0;
  for (double g : gaps) mean += g;
  mean /= static_cast<double>(gaps.size());
  record("cl3_heartbeat_mean_wall_ms", mean);
  record("cl3_heartbeat_max_gap_wall_ms", max_gap);
  record("cl3_cycle_wall_ms", run1.ms);
  EXPECT_NEAR(mean, 200.0, 40.0) << "a heartbeat every 200 ms of wall time";
  EXPECT_LT(max_gap, 1000.0) << "never near the steady max age";
  EXPECT_GE(run1.ms, 2800.0) << "the 1.5 s sim cycle took about 3 s of wall";
}

// =====================================================================================================================
// D5 (S): the activity path -- a machine step's text reaches the RTI detail while RUNNING at once (one `RTI: job N:`
// line) and the fake PLC prints it ("Robot busy: job N: ..."); the job's machine failure ends it FAILED with its STATION
// code, the result's detail on the fake PLC's "finished" line. The real RobotTaskServer and reference requester.
// =====================================================================================================================
namespace
{
class ActivityExecutor : public trainit::rti::JobExecutor
{
public:
  std::function<void(uint64_t)> on_start;
  void start(const trainit::rti::StartJob& job) override
  {
    if (worker_.joinable()) worker_.join();
    const uint64_t counter = job.counter;
    worker_ = std::thread([this, counter] {
      if (on_start) on_start(counter);
    });
  }
  void requestStop(uint64_t) override {}
  ~ActivityExecutor() override
  {
    if (worker_.joinable()) worker_.join();
  }

private:
  std::thread worker_;
};

class ReadyProbe : public trainit::rti::RuntimeProbe
{
public:
  bool ready(std::string&) override { return true; }
};
}  // namespace

TEST(P0bMachineNodes, D5_TheActivityReachesTheRtiDetailAndTheFakePlcAtOnce)
{
  using namespace trainit::rti;
  ExportTable table;
  ExportEntry e;
  e.number = 5;
  e.name = "turn_part";
  e.timeout_s = 30.0;
  e.timeout_ms = 30000;
  table.entries = {e};
  table.id = exportTableId(table.entries);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  auto server_node = std::make_shared<rclcpp::Node>("p0b_d5_server", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  auto plc_node = std::make_shared<rclcpp::Node>("p0b_d5_plc", kNs, rclcpp::NodeOptions().use_global_arguments(false));
  auto group = server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec.add_node(server_node);
  exec.add_node(plc_node);
  ActivityExecutor jobs;
  ReadyProbe probe;
  RobotTaskServer server(server_node, group, RobotTaskServer::Params{}, table, jobs, probe);
  ReferenceRequester::Options opts;
  opts.expected_table_id = table.id;
  ReferenceRequester plc(plc_node, opts);
  std::mutex lines_m;
  std::vector<std::pair<Clock::time_point, std::string>> lines;
  plc.setObserver([&](const std::string& l) {
    std::lock_guard<std::mutex> g(lines_m);
    lines.emplace_back(Clock::now(), l);
  });
  std::thread spin([&] { exec.spin(); });

  const std::vector<std::string> texts = {"MACHINE turner REQUESTED", "WAITING FOR MACHINE turner: 0 s of 8.0 s",
                                          "MACHINE turner BUSY: 0 s of 8.0 s", "MACHINE turner BUSY: 1 s of 8.0 s"};
  const std::string failure = "MACHINE turner TIMEOUT -- STATION_TIMEOUT (305): still BUSY after 8.0 s";
  std::mutex sent_m;
  std::vector<Clock::time_point> sent;
  jobs.on_start = [&](uint64_t counter) {
    std::this_thread::sleep_for(200ms);
    for (const auto& t : texts)
    {
      {
        std::lock_guard<std::mutex> g(sent_m);
        sent.push_back(Clock::now());
      }
      server.jobActivity(counter, t);
      server.jobActivity(counter, t);   // the same text again: no change, no new line
      std::this_thread::sleep_for(300ms);
    }
    JobEnd end;
    end.tree = JobEnd::Tree::FAILURE;
    end.holding = false;
    end.station_code = code::STATION_TIMEOUT;
    end.station_text = failure;
    end.reason = failure;   // the fault gate's reason: the same text (shown once)
    server.jobEnded(counter, end);
  };

  ASSERT_TRUE(plc.waitFor([](const ReferenceRequester::StatusMsg& m) { return m.heartbeat > 0; }, 3000ms));
  std::this_thread::sleep_for(400ms);
  ASSERT_EQ(plc.reset().outcome, ReferenceRequester::Outcome::ACCEPTED);
  ASSERT_TRUE(plc.waitFor([](const ReferenceRequester::StatusMsg& m) { return m.state == m.STATE_IDLE; }, 3000ms));
  ASSERT_EQ(plc.start(5, 1).outcome, ReferenceRequester::Outcome::ACCEPTED);
  ASSERT_TRUE(plc.waitFor([](const ReferenceRequester::StatusMsg& m) { return m.job_id == 1 && m.result != 0; }, 5000ms));
  std::this_thread::sleep_for(300ms);
  exec.cancel();
  spin.join();

  std::lock_guard<std::mutex> g(lines_m);
  std::lock_guard<std::mutex> gs(sent_m);
  ASSERT_EQ(sent.size(), texts.size());
  for (std::size_t k = 0; k < texts.size(); ++k)
  {
    const std::string want = "Robot busy: job 1: " + texts[k];
    int seen = 0;
    for (const auto& [t, l] : lines)
      if (l == want)
      {
        ++seen;
        record("d5_activity_" + std::to_string(k) + "_to_fake_plc_ms", msBetween(sent[k], t));
        EXPECT_LE(msBetween(sent[k], t), 150.0) << want << " (O2: <= 150 ms)";
      }
    EXPECT_EQ(seen, 1) << want;
  }
  bool finished = false;
  for (const auto& [t, l] : lines)
    if (l == "Job 1 (turn_part) finished: FAILED STATION_TIMEOUT (305) -- " + failure) finished = true;
  EXPECT_TRUE(finished) << "the result's detail on the finished line";
  const auto st = server.status();
  EXPECT_EQ(st.state, State::FAULTED);
  EXPECT_EQ(st.fault_code, code::STATION_TIMEOUT);
  EXPECT_EQ(st.detail, failure) << "the machine's words once";
  for (const auto& [t, l] : lines)
    for (const char* never : {"REFUSED", "JOB LOST", "Robot LOST", "exit 2", "internal error"})
      EXPECT_EQ(l.find(never), std::string::npos) << l;
  server.shutdown();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
