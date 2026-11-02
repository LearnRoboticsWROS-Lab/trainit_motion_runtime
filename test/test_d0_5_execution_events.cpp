// D0.5 (ADR-0030): the execution-event observer, ticking trees SHAPED AS TSA GENERATES THEM (task_<name> trees
// called with SubTree, identified steps inside <Sequence name="trainit_step:<id>:<kind>">, a grasp's
// SetWaypointFromCandidate, ProcessEvent through the process controller), with scripted test leaves. Pinned:
//   - the exact event sequence of a run: tasks on every entry, identified steps only, a retry's attempts, a
//     repeat's re-entries, statuses, the process events with their innermost task / step, seq 1..n;
//   - a grasp step's attempt index (per run, across the retry and the repeat) and its candidate id / provider;
//   - a halted step ends HALTED; a refused process dispatch is no event;
//   - two runs of the same tree give the same events (determinism);
//   - the runner's publisher: relative topic in the robot's namespace, RELIABLE + VOLATILE, depth 100, and the
//     cross-namespace remap guard knows the topic;
//   - a step re-ticked by a reactive parent is entered again; an observation error never fails the run; a stopped
//     run closes every open scope, innermost first, and emits nothing after RUN_ENDED;
//   - the TSA -> TMR carrier (ADR-0030 Decision 6, grammar v1) on its ONE shared artefact: a byte copy of TSA's
//     golden test/data/d0_5/task_program.xml (TSA's test_d0_5_step_ids checks the copy), run here.
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/execution_events.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"

namespace
{
using trainit::ExecutionEventPublisher;
using Msg = trainit_interfaces::msg::ExecutionEvent;

// --- scripted leaves --------------------------------------------------------------------------------
class Ok : public BT::SyncActionNode
{
public:
  Ok(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

class FailOnce : public BT::SyncActionNode   // FAILURE on its first tick ever, then SUCCESS
{
public:
  FailOnce(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return ticks_++ == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS; }

private:
  int ticks_ = 0;
};

class Flip : public BT::ConditionNode        // SUCCESS on its first tick, FAILURE afterwards
{
public:
  Flip(const std::string& n, const BT::NodeConfig& c) : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return ticks_++ == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }

private:
  int ticks_ = 0;
};

class Hold : public BT::StatefulActionNode   // RUNNING until halted
{
public:
  Hold(const std::string& n, const BT::NodeConfig& c) : BT::StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

class StubController : public trainit::ProcessController
{
public:
  bool executeEvent(const trainit::ProcessEvent& e) override { return e.channel != "refused"; }
};

trainit::ProcessController* g_process = nullptr;

class Proc : public BT::SyncActionNode        // TMR's ProcessEvent, reduced to its dispatch
{
public:
  Proc(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("kind"), BT::InputPort<std::string>("channel"),
            BT::InputPort<std::string>("value")};
  }
  BT::NodeStatus tick() override
  {
    trainit::ProcessEvent e;
    const std::string kind = getInput<std::string>("kind").value();
    e.type = kind == "process_on" ? trainit::ProcessEventType::ENABLE_PROCESS
           : kind == "process_off" ? trainit::ProcessEventType::DISABLE_PROCESS
           : kind == "set_value" ? trainit::ProcessEventType::SET_ANALOG_VALUE : trainit::ProcessEventType::TRIGGER;
    e.channel = getInput<std::string>("channel").value();
    const auto v = getInput<std::string>("value");
    e.value = (v && !v->empty()) ? std::stod(*v) : 0.0;
    return g_process->executeEvent(e) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class SetWaypointFromCandidate : public BT::SyncActionNode   // the name and the two ports the observer reads
{
public:
  SetWaypointFromCandidate(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("from"), BT::InputPort<int>("index", 0, "")};
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

const char* kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="task_cycle" _autoremap="true"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="task_cycle">
    <Sequence>
      <Sequence name="trainit_step:start:move">
        <Ok/>
      </Sequence>
      <Sequence name="trainit_step:glue_on:process">
        <Proc kind="process_on" channel="glue"/>
      </Sequence>
      <Proc kind="set_value" channel="speed" value="2.5"/>
      <Sequence name="trainit_step:cycles:repeat">
        <Repeat num_cycles="2">
          <Sequence>
            <Sequence name="trainit_step:one:task">
              <SubTree ID="task_pick" _autoremap="true"/>
            </Sequence>
          </Sequence>
        </Repeat>
      </Sequence>
      <Ok name="not_a_step"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="task_pick">
    <Sequence>
      <RetryUntilSuccessful num_attempts="3">
        <Sequence>
          <Sequence name="trainit_step:pick:grasp">
            <SetWaypointFromCandidate from="cands" index="0"/>
            <FailOnce/>
          </Sequence>
        </Sequence>
      </RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>
</root>)";

const char* kHaltTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SubTree ID="task_watch" _autoremap="true"/>
  </BehaviorTree>
  <BehaviorTree ID="task_watch">
    <ReactiveSequence>
      <Flip/>
      <Sequence name="trainit_step:wait:dwell">
        <Hold/>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>
</root>)";

void registerLeaves(BT::BehaviorTreeFactory& f)
{
  f.registerNodeType<Ok>("Ok");
  f.registerNodeType<FailOnce>("FailOnce");
  f.registerNodeType<Flip>("Flip");
  f.registerNodeType<Hold>("Hold");
  f.registerNodeType<Proc>("Proc");
  f.registerNodeType<SetWaypointFromCandidate>("SetWaypointFromCandidate");
}

class Leaf : public BT::SyncActionNode       // any TSA leaf (its literal ports accepted, never read)
{
public:
  Leaf(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("id"), BT::InputPort<std::string>("frame"),
            BT::InputPort<std::string>("dims"), BT::InputPort<std::string>("position"),
            BT::InputPort<std::string>("waypoint"), BT::InputPort<std::string>("eef"),
            BT::InputPort<std::string>("site"), BT::InputPort<std::string>("expect")};
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

struct RunResult
{
  std::vector<Msg> events;
  BT::NodeStatus status = BT::NodeStatus::IDLE;
};

RunResult runTree(const std::string& xml, int max_ticks = 50, bool tsa_leaves = false, bool throwing_sink = false)
{
  BT::BehaviorTreeFactory factory;
  registerLeaves(factory);
  if (tsa_leaves)
  {
    for (const char* name : {"AddCollisionObject", "MoveWaypoint", "CloseGripper", "OpenGripper", "VerifyGrasp"})
      factory.registerNodeType<Leaf>(name);
    factory.registerNodeType<Proc>("ProcessEvent");
  }
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("cands.0.id", "c7");
  bb->set<std::string>("cands.0.provider", "synthetic");
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("MainTree", bb);
  RunResult run;
  int32_t t = 0;
  auto observer = std::make_shared<trainit::ObservedProcessController::Observer>();
  trainit::ObservedProcessController controller(std::make_unique<StubController>(), observer);
  g_process = &controller;
  {
    ExecutionEventPublisher events(
      tree,
      [&run, throwing_sink](const Msg& m) {
        if (throwing_sink) throw std::runtime_error("the middleware failed");
        run.events.push_back(m);
      },
      [&t]() { builtin_interfaces::msg::Time s; s.sec = 100; s.nanosec = static_cast<uint32_t>(t++); return s; },
      "run-test");
    *observer = [&events](const trainit::ProcessEvent& e) { events.processDispatched(e); };
    events.runStarted();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && status == BT::NodeStatus::RUNNING; ++i) status = tree.tickOnce();
    run.status = status;
    events.runEnded(status, status == BT::NodeStatus::RUNNING);
    *observer = nullptr;
  }
  g_process = nullptr;
  return run;
}

// "<KIND> task/step(kind) [status] [#attempt candidate] [channel kind value]" per event
std::string text(const Msg& m)
{
  static const char* kinds[] = {"?", "RUN_STARTED", "RUN_ENDED", "TASK_ENTERED", "TASK_EXITED", "STEP_ENTERED",
                                "STEP_EXITED", "PROCESS_EVENT"};
  static const char* statuses[] = {"", " SUCCEEDED", " FAILED", " HALTED"};
  std::string s = kinds[m.kind];
  if (!m.task_id.empty()) s += " " + m.task_id;
  if (!m.step_id.empty()) s += "/" + m.step_id + "(" + m.step_kind + ")";
  s += statuses[m.status];
  if (m.attempt_index) s += " #" + std::to_string(m.attempt_index) + " " + m.candidate_id + "@" + m.candidate_provider;
  if (m.kind == Msg::PROCESS_EVENT)
  {
    char v[32];
    std::snprintf(v, sizeof(v), "%g", m.process_value);
    s += " " + m.process_channel + " " + m.process_kind + " " + v;
  }
  return s;
}

std::vector<std::string> texts(const RunResult& run)
{
  std::vector<std::string> out;
  for (const auto& m : run.events) out.push_back(text(m));
  return out;
}
}  // namespace

TEST(ExecutionEvents, TheStepNameAndTheTaskTreeIdAreParsedStrictly)
{
  const auto tag = trainit::parseStepName("trainit_step:pick:grasp");
  ASSERT_TRUE(tag.has_value());
  EXPECT_EQ(tag->id, "pick");
  EXPECT_EQ(tag->kind, "grasp");
  for (const char* other : {"pick", "trainit_step:", "trainit_step:pick", "trainit_step::grasp", "trainit_step:pick:",
                            "trainit_step:a:b:c", "Trainit_step:a:b"})
    EXPECT_FALSE(trainit::parseStepName(other).has_value()) << other;
  EXPECT_EQ(trainit::taskOfTreeId("task_pick").value_or(""), "pick");
  EXPECT_FALSE(trainit::taskOfTreeId("MainTree").has_value());
  EXPECT_FALSE(trainit::taskOfTreeId("task_").has_value());
  const auto run_id = trainit::makeRunId();
  EXPECT_EQ(run_id.rfind("run-", 0), 0u);
  EXPECT_EQ(run_id.size(), std::string("run-20261030T120000Z-0123abcd").size()) << run_id;
}

TEST(ExecutionEvents, ARunIsItsTasksItsIdentifiedStepsItsProcessEventsInOrder)
{
  const RunResult run = runTree(kTree);
  ASSERT_EQ(run.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(texts(run), (std::vector<std::string>{
    "RUN_STARTED",
    "TASK_ENTERED cycle",
    "STEP_ENTERED cycle/start(move)", "STEP_EXITED cycle/start(move) SUCCEEDED",
    "STEP_ENTERED cycle/glue_on(process)",
    "PROCESS_EVENT cycle/glue_on(process) glue process_on 1",
    "STEP_EXITED cycle/glue_on(process) SUCCEEDED",
    "PROCESS_EVENT cycle speed set_value 2.5",                  // no identified step active: the task only
    "STEP_ENTERED cycle/cycles(repeat)",
    "STEP_ENTERED cycle/one(task)", "TASK_ENTERED pick",
    "STEP_ENTERED pick/pick(grasp) #1 c7@synthetic", "STEP_EXITED pick/pick(grasp) FAILED #1 c7@synthetic",
    "STEP_ENTERED pick/pick(grasp) #2 c7@synthetic", "STEP_EXITED pick/pick(grasp) SUCCEEDED #2 c7@synthetic",
    "TASK_EXITED pick SUCCEEDED", "STEP_EXITED cycle/one(task) SUCCEEDED",
    "STEP_ENTERED cycle/one(task)", "TASK_ENTERED pick",       // the repeat's second cycle: entered again
    "STEP_ENTERED pick/pick(grasp) #3 c7@synthetic", "STEP_EXITED pick/pick(grasp) SUCCEEDED #3 c7@synthetic",
    "TASK_EXITED pick SUCCEEDED", "STEP_EXITED cycle/one(task) SUCCEEDED",
    "STEP_EXITED cycle/cycles(repeat) SUCCEEDED",
    "TASK_EXITED cycle SUCCEEDED",
    "RUN_ENDED SUCCEEDED"}));
  for (size_t i = 0; i < run.events.size(); ++i)
  {
    EXPECT_EQ(run.events[i].seq, i + 1);                      // the deterministic order of equal stamps
    EXPECT_EQ(run.events[i].run_id, "run-test");
    if (run.events[i].kind != Msg::PROCESS_EVENT) EXPECT_TRUE(run.events[i].process_channel.empty());
  }
}

TEST(ExecutionEvents, AHaltedStepEndsHaltedAndItsTaskFails)
{
  const RunResult run = runTree(kHaltTree);
  ASSERT_EQ(run.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(texts(run), (std::vector<std::string>{
    "RUN_STARTED", "TASK_ENTERED watch", "STEP_ENTERED watch/wait(dwell)", "STEP_EXITED watch/wait(dwell) HALTED",
    "TASK_EXITED watch FAILED", "RUN_ENDED FAILED"}));
}

TEST(ExecutionEvents, AStoppedRunEndsHalted)
{
  const char* hold = R"(<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree"><Sequence name="trainit_step:x:dwell"><Hold/></Sequence></BehaviorTree></root>)";
  const RunResult run = runTree(hold, 3);
  ASSERT_EQ(run.status, BT::NodeStatus::RUNNING);
  ASSERT_GE(run.events.size(), 3u);
  EXPECT_EQ(text(run.events[1]), "STEP_ENTERED/x(dwell)") << "a step outside any task tree has no task";
  EXPECT_EQ(text(run.events.back()), "RUN_ENDED HALTED");
}

TEST(ExecutionEvents, ARefusedProcessDispatchIsNoEvent)
{
  const char* refused = R"(<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree"><Fallback><Proc kind="process_on" channel="refused"/><Ok/></Fallback>
    </BehaviorTree></root>)";
  const RunResult run = runTree(refused);
  for (const auto& m : run.events) EXPECT_NE(m.kind, Msg::PROCESS_EVENT) << text(m);
}

TEST(ExecutionEvents, TwoRunsOfOneTreeGiveTheSameEvents)
{
  EXPECT_EQ(texts(runTree(kTree)), texts(runTree(kTree)));
}

TEST(ExecutionEvents, TheProcessKindsAreTheCanonicalModels)
{
  trainit::ProcessEvent e;
  e.type = trainit::ProcessEventType::ENABLE_PROCESS;
  EXPECT_EQ(trainit::processKindAndValue(e), (std::pair<std::string, double>{"process_on", 1.0}));
  e.type = trainit::ProcessEventType::DISABLE_PROCESS;
  EXPECT_EQ(trainit::processKindAndValue(e), (std::pair<std::string, double>{"process_off", 0.0}));
  e.type = trainit::ProcessEventType::SET_ANALOG_VALUE;
  e.value = 3.25;
  EXPECT_EQ(trainit::processKindAndValue(e), (std::pair<std::string, double>{"set_value", 3.25}));
  e.type = trainit::ProcessEventType::TRIGGER;
  EXPECT_EQ(trainit::processKindAndValue(e), (std::pair<std::string, double>{"external", 0.0}));
}

TEST(ExecutionEvents, TheRunnersPublisherIsRelativeReliableVolatileAndGuarded)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("trainit_run_bt", "/r1");
    BT::BehaviorTreeFactory factory;
    registerLeaves(factory);
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree"><Ok/>
      </BehaviorTree></root>)");
    auto tree = factory.createTree("MainTree");
    auto events = ExecutionEventPublisher::forNode(node, tree);
    EXPECT_EQ(std::string(trainit::kExecutionEventsTopic), "trainit/execution_events");
    const auto infos = node->get_publishers_info_by_topic("/r1/trainit/execution_events");
    ASSERT_EQ(infos.size(), 1u);
    const auto qos = infos[0].qos_profile().get_rmw_qos_profile();
    EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);   // never a late-joiner's history
    const auto mine = ExecutionEventPublisher::qos().get_rmw_qos_profile();   // the graph omits history / depth
    EXPECT_EQ(mine.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(mine.depth, 100u);
    EXPECT_EQ(mine.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(mine.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::NodeOptions remapped;
    remapped.arguments({"--ros-args", "-r", "trainit/execution_events:=/r2/trainit/execution_events"});
    auto escaping = std::make_shared<rclcpp::Node>("trainit_run_bt", "/r1", remapped);
    const auto escaped = trainit::MoveItMotionRuntime::crossNamespaceRemaps(escaping, trainit::eef::Contract{});
    bool named = false;
    for (const auto& e : escaped) named = named || e.find("/r1/trainit/execution_events") != std::string::npos;
    EXPECT_TRUE(named) << "the cross-namespace remap guard does not know the execution-event topic";
  }
  rclcpp::shutdown();
}


TEST(ExecutionEvents, AStepReTickedByAReactiveParentIsEnteredAgain)
{
  const char* reactive = R"(<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree"><SubTree ID="task_watch" _autoremap="true"/></BehaviorTree>
    <BehaviorTree ID="task_watch"><ReactiveSequence>
      <Sequence name="trainit_step:light:process"><Proc kind="process_on" channel="light"/></Sequence>
      <Sequence name="trainit_step:wait:dwell"><Hold/></Sequence>
    </ReactiveSequence></BehaviorTree></root>)";
  const RunResult run = runTree(reactive, 2);
  ASSERT_EQ(run.status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(texts(run), (std::vector<std::string>{
    "RUN_STARTED", "TASK_ENTERED watch",
    "STEP_ENTERED watch/light(process)", "PROCESS_EVENT watch/light(process) light process_on 1",
    "STEP_EXITED watch/light(process) SUCCEEDED", "STEP_ENTERED watch/wait(dwell)",
    "STEP_ENTERED watch/light(process)", "PROCESS_EVENT watch/light(process) light process_on 1",   // tick 2
    "STEP_EXITED watch/light(process) SUCCEEDED",
    "STEP_EXITED watch/wait(dwell) HALTED", "TASK_EXITED watch HALTED", "RUN_ENDED HALTED"}));
}

TEST(ExecutionEvents, AnObservationErrorNeverFailsTheRun)
{
  const RunResult run = runTree(kTree, 50, false, true);
  EXPECT_EQ(run.status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(run.events.empty());
}

TEST(ExecutionEvents, AStoppedRunClosesEveryOpenScopeInnermostFirst)
{
  const RunResult run = runTree(kHaltTree, 1);           // stopped after one tick: wait is RUNNING
  ASSERT_EQ(run.status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(texts(run), (std::vector<std::string>{
    "RUN_STARTED", "TASK_ENTERED watch", "STEP_ENTERED watch/wait(dwell)", "STEP_EXITED watch/wait(dwell) HALTED",
    "TASK_EXITED watch HALTED", "RUN_ENDED HALTED"}));
}

TEST(ExecutionEvents, TheTsaGoldenIsReadAsItsStepsAndTasks)
{
  std::ifstream in(TRAINIT_D0_5_TSA_GOLDEN);
  ASSERT_TRUE(in.is_open()) << TRAINIT_D0_5_TSA_GOLDEN;
  std::stringstream xml;
  xml << in.rdbuf();
  const RunResult run = runTree(xml.str(), 50, true);
  ASSERT_EQ(run.status, BT::NodeStatus::SUCCESS);
  std::multiset<std::string> entered;
  for (const auto& m : run.events)
    if (m.kind == Msg::STEP_ENTERED || m.kind == Msg::TASK_ENTERED) entered.insert(text(m));
  EXPECT_EQ(entered, (std::multiset<std::string>{
    "TASK_ENTERED cell_cycle", "STEP_ENTERED cell_cycle/start(move)", "STEP_ENTERED cell_cycle/conveyor_on(process)",
    "STEP_ENTERED cell_cycle/cycles(repeat)", "STEP_ENTERED cell_cycle/one_part(task)",
    "STEP_ENTERED cell_cycle/one_part(task)", "TASK_ENTERED transfer", "TASK_ENTERED transfer",
    "STEP_ENTERED transfer/pick(grasp) #1 @", "STEP_ENTERED transfer/pick(grasp) #2 @",
    "STEP_ENTERED transfer/place(release)", "STEP_ENTERED transfer/place(release)",
    "STEP_ENTERED cell_cycle/reach(move)"}));                  // a taught-target grasp states no candidate
  bool process_in_step = false;
  for (const auto& m : run.events)
    process_in_step = process_in_step || (m.kind == Msg::PROCESS_EVENT && m.step_id == "conveyor_on");
  EXPECT_TRUE(process_in_step);
}
