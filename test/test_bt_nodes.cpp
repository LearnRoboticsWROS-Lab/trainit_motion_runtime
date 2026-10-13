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


// SetWaypointRelative is pure blackboard too.
static BT::NodeStatus tickSetRelative(const std::string& attrs, BT::Blackboard::Ptr bb)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">"
    "<SetWaypointRelative " + attrs + "/></BehaviorTree></root>";
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("T", bb);
  return tree.tickOnce();
}

TEST(SetWaypointRelative, FollowsTheReferenceWithAnOffsetAndCopiesOrientation)
{
  auto bb = BT::Blackboard::create();
  // pick as vision would have left it this cycle
  bb->set<std::string>("pick.position", "0.6396;0.0297;0.0306");
  bb->set<std::string>("pick.orientation", "-0.707;0.7071;-0.0072;0.0066");
  auto st = tickSetRelative(R"(waypoint="post_pick" from="pick" dz="0.08")", bb);
  ASSERT_EQ(st, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("post_pick.position"), "0.6396;0.0297;0.1106");
  EXPECT_EQ(bb->get<std::string>("post_pick.orientation"), "-0.707;0.7071;-0.0072;0.0066");
  EXPECT_EQ(bb->get<std::string>("post_pick.type"), "tcp");
}

TEST(SetWaypointRelative, RpyDeltaComposesInBaseFrame)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("a.position", "1;0;0");
  bb->set<std::string>("a.orientation", "0;0;0;1");     // identity reference
  ASSERT_EQ(tickSetRelative(R"(waypoint="b" from="a" dyaw="90")", bb), BT::NodeStatus::SUCCESS);
  // Rz(90 deg) on identity -> qz = sin(45 deg), qw = cos(45 deg)
  const auto q = bb->get<std::string>("b.orientation");
  std::vector<double> v;
  { std::stringstream ss(q); std::string t;
    while (std::getline(ss, t, ';')) v.push_back(std::stod(t)); }
  ASSERT_EQ(v.size(), 4u);
  EXPECT_NEAR(v[0], 0.0, 1e-6);
  EXPECT_NEAR(v[1], 0.0, 1e-6);
  EXPECT_NEAR(v[2], 0.7071067, 1e-5);
  EXPECT_NEAR(v[3], 0.7071067, 1e-5);
}

TEST(SetWaypointRelative, FailsLoudlyWithoutAReferencePose)
{
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tickSetRelative(R"(waypoint="b" from="ghost" dz="0.1")", bb), BT::NodeStatus::FAILURE);
  bb->set<std::string>("j.position", "0;0;0");           // position but no orientation
  EXPECT_EQ(tickSetRelative(R"(waypoint="b" from="j")", bb), BT::NodeStatus::FAILURE);
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

// ─── ADR-0016: the candidate LIST ─────────────────────────────────────────────
// DetectObject with keep>1 writes "<from>.<i>.*"; with keep=1 — and in every bundle
// generated before ADR-0016 — it writes the FLAT "<from>.*". The resolution between
// the two is what makes "try the next candidate" writable without touching a single
// existing tree.
namespace
{
// three candidates as DetectObject keep=3 would have left them, in the order the
// detector PUBLISHED them (which is the rank, ADR-0016 Decision 1)
const char* kCandidatePos[3] = {"0.61;0.1;0.03", "0.62;0.2;0.03", "0.63;0.3;0.03"};

BT::Blackboard::Ptr threeCandidates()
{
  auto bb = BT::Blackboard::create();
  for (int i = 0; i < 3; ++i)
  {
    const std::string pre = "detected.capsule." + std::to_string(i);
    bb->set<std::string>(pre + ".position", kCandidatePos[i]);
    bb->set<std::string>(pre + ".orientation", "0;1;0;" + std::to_string(i));
  }
  bb->set<std::string>("detected.capsule.count", "3");
  return bb;
}
}  // namespace

TEST(SetWaypointFromDetection, IndexTakesTheNthCandidateOfTheList)
{
  auto bb = threeCandidates();
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="pick" from="detected.capsule" index="2" orientation="detected")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("pick.position"), kCandidatePos[2]);
  EXPECT_EQ(bb->get<std::string>("pick.orientation"), "0;1;0;2");
  // and the default is candidate 0, so an unindexed node means the rank's first
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="grasp" from="detected.capsule" orientation="detected")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("grasp.position"), kCandidatePos[0]);
}

// The compatibility hinge: index=0 against a keep=1 (flat) detection.
TEST(SetWaypointFromDetection, IndexZeroFallsBackToTheFlatSingleCandidate)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("detected.cube.position", "0.5;-0.02;0.03");
  bb->set<std::string>("detected.cube.orientation", "0;1;0;0");
  // implicit (every pre-ADR-0016 tree) and explicit index=0 must agree
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="a" from="detected.cube" orientation="detected")", bb), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="b" from="detected.cube" index="0" orientation="detected")", bb), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("a.position"), bb->get<std::string>("b.position"));
  EXPECT_EQ(bb->get<std::string>("b.position"), "0.5;-0.02;0.03");
}

// A non-zero index must NOT fall back: a retry loop that silently re-reads the one
// flat candidate would re-try the SAME object for ever — the failure the list exists
// to remove.
TEST(SetWaypointFromDetection, ANonZeroIndexRefusesToFallBackToTheFlatCandidate)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("detected.cube.position", "0.5;0;0.03");
  bb->set<std::string>("detected.cube.orientation", "0;1;0;0");
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="a" from="detected.cube" index="1" orientation="detected")", bb),
            BT::NodeStatus::FAILURE);
  // ... and asking past the end of a real list fails too (count=3, index 3)
  auto list = threeCandidates();
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="a" from="detected.capsule" index="3" orientation="detected")", list),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickSetWaypoint(R"(waypoint="a" from="detected.capsule" index="-1" orientation="detected")", list),
            BT::NodeStatus::FAILURE);
}

// The index is driven by a counter, so it arrives as a blackboard INT, not a string.
// This is why the port is declared int: with a std::string port this very test fails
// at TREE CREATION — "the port [i] was initially created with type [int] and, later,
// type [std::string] was used somewhere else" — and so would any generated tree whose
// counter is an int before the tree is built. An int port binds both.
TEST(SetWaypointFromDetection, IndexCanComeFromTheBlackboardAsAnIntOrAString)
{
  auto bb = threeCandidates();
  bb->set<int>("i", 1);
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="pick" from="detected.capsule" index="{i}" orientation="detected")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("pick.position"), kCandidatePos[1]);

  auto bb2 = threeCandidates();
  bb2->set<std::string>("i", "2");
  ASSERT_EQ(tickSetWaypoint(R"(waypoint="pick" from="detected.capsule" index="{i}" orientation="detected")", bb2),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb2->get<std::string>("pick.position"), kCandidatePos[2]);
}

// The shape ADR-0016 Decision 4 generates: the list, and a retry over an advancing
// index. No new control node — RetryUntilSuccessful and Script are BT.CPP v4's own.
TEST(BtNodes, LoadsTheCandidateListRetryTree)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const char* xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <Script code="i := -1"/>
          <DetectObject detector="capsule" keep="3" out_key="detected.capsule"/>
          <RetryUntilSuccessful num_attempts="3">
            <Sequence>
              <Script code="i := i + 1"/>   <!-- advances on EVERY attempt, not only on success -->
              <SetWaypointFromDetection waypoint="pick" from="detected.capsule" index="{i}" orientation="detected"/>
              <MoveWaypoint waypoint="pick"/>
              <CloseGripper/>
            </Sequence>
          </RetryUntilSuccessful>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto bb = BT::Blackboard::create();
  BT::Tree tree;   // createTree() is [[nodiscard]]: hold it, like the tests above
  EXPECT_NO_THROW({
    factory.registerBehaviorTreeFromText(xml);
    tree = factory.createTree("MainTree", bb);
  });
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

// ─── ADR-0015: the motion/execution guard, and the ServoTrack node ────────────
// MoveWaypoint validates its DATA before it touches the runtime, so the guard is
// tickable with a bare tree (no BtContext, no MoveIt): an unrecognised value is a
// FAILURE, while an accepted one gets as far as the (absent) runtime and throws.
static BT::NodeStatus tickNode(const std::string& xml_node, BT::Blackboard::Ptr bb)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
    xml_node + "</BehaviorTree></root>";
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("T", bb);
  return tree.tickOnce();
}

static BT::Blackboard::Ptr tcpWaypoint(const std::string& wp, const std::string& motion)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>(wp + ".type", "tcp");
  bb->set<std::string>(wp + ".position", "0.6396;0.0297;0.0306");
  bb->set<std::string>(wp + ".orientation", "-0.707;0.7071;-0.0072;0.0066");
  if (!motion.empty()) bb->set<std::string>(wp + ".motion", motion);
  return bb;
}

TEST(MoveWaypointGuard, AnUnknownMotionFailsInsteadOfPlanningAFreeMove)
{
  // the real defect: "srvo" used to fall through the if/else into moveCollisionFree
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", tcpWaypoint("pick", "srvo")),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", tcpWaypoint("pick", "linaer")),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", tcpWaypoint("pick", "PTP")),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", tcpWaypoint("pick", "servo")),
            BT::NodeStatus::FAILURE);
}

TEST(MoveWaypointGuard, EveryAcceptedMotionReachesTheRuntime)
{
  // The guard must reject NOTHING else: these get past validation and then hit the
  // absent BtContext, which throws -- a different outcome from the FAILURE above.
  for (const char* m : {"ptp", "free", "lin", "linear", "circ", "circular", ""})
  {
    auto bb = tcpWaypoint("pick", m);
    if (std::string(m).empty()) bb->set<std::string>("pick.motion", "");   // explicit empty = free
    EXPECT_ANY_THROW(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", bb)) << "motion=" << m;
  }
}

TEST(MoveWaypointGuard, ExecutionServoIsRefusedBecauseServoTrackOwnsIt)
{
  auto bb = tcpWaypoint("pick", "free");
  bb->set<std::string>("pick.execution", "servo");
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", bb), BT::NodeStatus::FAILURE);

  auto bad = tcpWaypoint("pick", "free");
  bad->set<std::string>("pick.execution", "stream");
  EXPECT_EQ(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", bad), BT::NodeStatus::FAILURE);

  // the default axis value changes nothing: it still reaches the runtime
  auto plan = tcpWaypoint("pick", "free");
  plan->set<std::string>("pick.execution", "plan");
  EXPECT_ANY_THROW(tickNode(R"(<MoveWaypoint waypoint="pick"/>)", plan));
}

TEST(ServoTrack, IsRegisteredInTheMotionFamilyAndLoadsInATree)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  ASSERT_EQ(factory.manifests().count("ServoTrack"), 1u);
  // the single port is the waypoint name, like MoveWaypoint
  const auto& ports = factory.manifests().at("ServoTrack").ports;
  EXPECT_EQ(ports.count("waypoint"), 1u);

  const char* xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <DetectObject detector="capsule" out_key="detected.capsule"/>
          <SetWaypointFromDetection waypoint="pick" from="detected.capsule"/>
          <MoveWaypoint waypoint="pre_pick"/>
          <ServoTrack waypoint="pick" tolerance_m="0.002" timeout_ms="5000"/>
          <CloseGripper/>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto bb = BT::Blackboard::create();
  BT::Tree tree;
  EXPECT_NO_THROW({
    factory.registerBehaviorTreeFromText(xml);
    tree = factory.createTree("MainTree", bb);
  });
}

TEST(ServoTrack, RefusesABadTargetBeforeTouchingTheServoNode)
{
  // no position at all
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tickNode(R"(<ServoTrack waypoint="pick"/>)", bb), BT::NodeStatus::FAILURE);

  // a joint/named waypoint: streaming has no cartesian target to converge on
  auto joint = BT::Blackboard::create();
  joint->set<std::string>("pick.type", "joint");
  joint->set<std::string>("pick.named", "home");
  EXPECT_EQ(tickNode(R"(<ServoTrack waypoint="pick"/>)", joint), BT::NodeStatus::FAILURE);

  // a malformed orientation must fail here, not stream a wrong pose
  auto bad_q = tcpWaypoint("pick", "");
  bad_q->set<std::string>("pick.orientation", "1;2;3");
  EXPECT_EQ(tickNode(R"(<ServoTrack waypoint="pick"/>)", bad_q), BT::NodeStatus::FAILURE);

  // and an empty waypoint name
  EXPECT_EQ(tickNode(R"(<ServoTrack waypoint=""/>)", BT::Blackboard::create()),
            BT::NodeStatus::FAILURE);
}
