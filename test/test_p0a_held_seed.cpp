// test_p0a_held_seed (P0.a, assumption A7, ADR-0032 Decision 8): the held state the task server SEEDS into a job's
// fresh blackboard (`trainit__holding := true|false`, the same script assignment TSA's post-conditions use) is read by
// a node's `holding` port inside the exported task's SubTree (_autoremap, as TSA generates it) exactly as if the job
// had set it, a post-condition updates the ROOT entry the server reads back at the end, and an UNSEEDED key stays
// absent (UNKNOWN: ADR-0024 Amendment M4.5d reads it as HELD). Pure BehaviorTree.CPP: no ROS graph.
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>

#include "trainit_motion_runtime/bt_job_executor.hpp"

namespace
{
std::vector<std::string> g_reads;   // what each ReadHolding saw, in order

class ReadHolding : public BT::SyncActionNode
{
public:
  ReadHolding(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<bool>("holding")}; }
  BT::NodeStatus tick() override
  {
    const auto v = getInput<bool>("holding");
    g_reads.push_back(name() + "=" + (v ? (*v ? "true" : "false") : "UNREADABLE"));
    return BT::NodeStatus::SUCCESS;
  }
};

const char* kTrees = R"(<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="job_pick">
    <Sequence><Script code="trainit__fault := ''"/><SubTree ID="task_pick" _autoremap="true"/></Sequence>
  </BehaviorTree>
  <BehaviorTree ID="task_pick">
    <Sequence>
      <ReadHolding name="before" holding="{trainit__holding}"/>
      <AlwaysSuccess _onSuccess="trainit__holding := true"/>
      <ReadHolding name="after" holding="{trainit__holding}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="job_place">
    <Sequence><Script code="trainit__fault := ''"/><SubTree ID="task_place" _autoremap="true"/></Sequence>
  </BehaviorTree>
  <BehaviorTree ID="task_place">
    <Sequence>
      <ReadHolding name="before" holding="{trainit__holding}"/>
      <AlwaysSuccess _onSuccess="trainit__holding := false"/>
      <ReadHolding name="after" holding="{trainit__holding}"/>
    </Sequence>
  </BehaviorTree>
</root>)";

std::optional<bool> runJob(const char* tree, std::optional<bool> seed)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ReadHolding>("ReadHolding");
  factory.registerBehaviorTreeFromText(kTrees);
  auto bb = BT::Blackboard::create();
  if (seed) trainit::rti::seedHeld(bb, *seed);
  auto t = factory.createTree(tree, bb);
  EXPECT_EQ(t.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  return trainit::rti::readHeld(bb);   // what the server reads back at the job's end
}
}  // namespace

TEST(P0aHeldSeed, ASeededFreeStateIsReadByTheTaskAndUpdatedByItsPostCondition)
{
  g_reads.clear();
  const auto end = runJob("job_pick", false);
  EXPECT_EQ(g_reads, (std::vector<std::string>{"before=false", "after=true"}));
  ASSERT_TRUE(end.has_value());
  EXPECT_TRUE(*end);   // the root entry: what the next job is seeded with
}

TEST(P0aHeldSeed, ASeededHeldStateIsCarriedIntoTheNextJob)
{
  g_reads.clear();
  const auto end = runJob("job_place", true);   // a place job after a pick job
  EXPECT_EQ(g_reads, (std::vector<std::string>{"before=true", "after=false"}));
  ASSERT_TRUE(end.has_value());
  EXPECT_FALSE(*end);
}

TEST(P0aHeldSeed, AnUnseededStateStaysUnknownUntilAGrippingNodeSetsIt)
{
  g_reads.clear();
  auto bb = BT::Blackboard::create();
  EXPECT_FALSE(trainit::rti::readHeld(bb).has_value());   // after a start: unknown
  const auto end = runJob("job_pick", std::nullopt);
  EXPECT_EQ(g_reads, (std::vector<std::string>{"before=UNREADABLE", "after=true"}));   // the motions read HELD
  ASSERT_TRUE(end.has_value());
  EXPECT_TRUE(*end);
}

TEST(P0aHeldSeed, ReadHeldAcceptsOnlyABooleanMeaning)
{
  auto bb = BT::Blackboard::create();
  bb->set<double>("trainit__holding", 1.0);
  EXPECT_EQ(trainit::rti::readHeld(bb), std::optional<bool>(true));
  bb = BT::Blackboard::create();
  bb->set<std::string>("trainit__holding", "false");
  EXPECT_EQ(trainit::rti::readHeld(bb), std::optional<bool>(false));
  bb = BT::Blackboard::create();
  bb->set<double>("trainit__holding", 0.5);
  EXPECT_FALSE(trainit::rti::readHeld(bb).has_value());   // anything else: UNKNOWN (fail closed)
  bb = BT::Blackboard::create();
  bb->set<std::string>("trainit__holding", "maybe");
  EXPECT_FALSE(trainit::rti::readHeld(bb).has_value());
}
