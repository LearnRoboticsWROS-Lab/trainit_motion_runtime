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
