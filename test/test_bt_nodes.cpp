// Registration + tree-load test for the BT node set, focused on the new
// dynamic-object runtime-flag nodes (SetAttachedCollisionCheck, SetReleasePolicy).
#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include "trainit_motion_runtime/bt_nodes.hpp"

TEST(BtNodes, RegistersDynamicObjectFlagNodes)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const auto& m = factory.manifests();
  EXPECT_EQ(m.count("SetAttachedCollisionCheck"), 1u);
  EXPECT_EQ(m.count("SetReleasePolicy"), 1u);
  // sanity: the existing vocabulary is still registered
  EXPECT_EQ(m.count("MoveWaypoint"), 1u);
  EXPECT_EQ(m.count("CloseGripper"), 1u);
  EXPECT_EQ(m.count("AttachObject"), 1u);
}

TEST(BtNodes, LoadsTreeUsingNewNodes)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const char* xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <MoveWaypoint waypoint="pre_pick"/>
          <CloseGripper/>
          <SetAttachedCollisionCheck value="true"/>
          <MoveWaypoint waypoint="approach_prewash"/>
          <SetReleasePolicy policy="freeze"/>
          <OpenGripper/>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto bb = BT::Blackboard::create();
  BT::Tree tree;
  // registration + instantiation validates node IDs + ports against the factory
  // (no tick: ticking needs the live BtContext runtime stack).
  EXPECT_NO_THROW({
    factory.registerBehaviorTreeFromText(xml);
    tree = factory.createTree("MainTree", bb);
  });
}


// ─── perception nodes ─────────────────────────────────────────────────────────
TEST(BtNodes, RegistersPerceptionNodes)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  EXPECT_EQ(factory.manifests().count("DetectObject"), 1u);
  EXPECT_EQ(factory.manifests().count("SetWaypointFromDetection"), 1u);
}

TEST(BtNodes, LoadsVisionGuidedTree)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const char* xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <DetectObject detector="cube" class_id="cube" target_frame="base_link" out_key="detected.cube"/>
          <SetWaypointFromDetection waypoint="pre_pick"  from="detected.cube" dz="0.10" orientation="from:pick"/>
          <SetWaypointFromDetection waypoint="pick"      from="detected.cube" dz="0.00"/>
          <SetWaypointFromDetection waypoint="post_pick" from="detected.cube" dz="0.10" orientation="from:pick"/>
          <MoveWaypoint waypoint="pre_pick"/>
          <MoveWaypoint waypoint="pick"/>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto bb = BT::Blackboard::create();
  EXPECT_NO_THROW({ factory.registerBehaviorTreeFromText(xml); factory.createTree("MainTree", bb); });
}

// SetWaypointFromDetection is pure blackboard, so it can be TICKED without a runtime.
static BT::NodeStatus tickSetWaypoint(const std::string& attrs, BT::Blackboard::Ptr bb)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">"
    "<SetWaypointFromDetection " + attrs + "/></BehaviorTree></root>";
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("T", bb);
  return tree.tickOnce();
}

TEST(SetWaypointFromDetection, WritesPositionWithOffsetAndCopiesOrientation)
{
  auto bb = BT::Blackboard::create();
  // what DetectObject would have written, and what bt_params defined for 'pick'
  bb->set<std::string>("detected.cube.position", "0.5617;-0.0252;0.0246");
  bb->set<std::string>("pick.orientation", "-0.707;0.7071;-0.0108;0.0103");
  bb->set<std::string>("pre_pick.type", "joint");           // captured as joints in the 3.2 bundle
  bb->set<std::string>("pre_pick.named", "pre_pick");

  auto st = tickSetWaypoint(R"(waypoint="pre_pick" from="detected.cube" dz="0.10" orientation="from:pick")", bb);
  ASSERT_EQ(st, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("pre_pick.position"), "0.5617;-0.0252;0.1246");
  EXPECT_EQ(bb->get<std::string>("pre_pick.orientation"), "-0.707;0.7071;-0.0108;0.0103");
  EXPECT_EQ(bb->get<std::string>("pre_pick.type"), "tcp");   // joint -> tcp, so MoveWaypoint builds a pose
}

TEST(SetWaypointFromDetection, KeepIsTheDefaultAndPreservesBtParamsOrientation)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("detected.cube.position", "0.5;-0.02;0.03");
  bb->set<std::string>("pick.orientation", "0;1;0;0");
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="pick" from="detected.cube")", bb), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("pick.position"), "0.5;-0.02;0.03");
  EXPECT_EQ(bb->get<std::string>("pick.orientation"), "0;1;0;0");
}

TEST(SetWaypointFromDetection, DetectedAndLiteralOrientations)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("d.position", "1;2;3");
  bb->set<std::string>("d.orientation", "0;0;0.7071;0.7071");
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="a" from="d" orientation="detected")", bb), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("a.orientation"), "0;0;0.7071;0.7071");
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="b" from="d" orientation="0;0;0;1")", bb), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("b.orientation"), "0;0;0;1");
}

TEST(SetWaypointFromDetection, FailsLoudlyWhenNothingWasDetectedOrNoOrientation)
{
  auto bb = BT::Blackboard::create();
  // no detection on the blackboard at all
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="pick" from="detected.cube")", bb), BT::NodeStatus::FAILURE);
  // detection present, but the waypoint has no orientation anywhere -> MoveWaypoint
  // would refuse it later; fail here, where the cause is visible
  bb->set<std::string>("detected.cube.position", "0.5;0;0");
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="fresh" from="detected.cube")", bb), BT::NodeStatus::FAILURE);
  // a bad literal
  bb->set<std::string>("x.orientation", "0;0;0;1");
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="x" from="detected.cube" orientation="1;2;3")", bb), BT::NodeStatus::FAILURE);
}
