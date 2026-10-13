// DetectObject's WRITE path, executed: a real node, a real subscription, a real TF
// buffer, a real vision_msgs/Detection3DArray — and the blackboard read back.
//
// The pure selection rule is unit-tested in test_detection_candidates.cpp. What only
// a tick can show is the CONTRACT the rest of the system consumes: which keys exist,
// under which names, with which values. That contract is what a generated bundle and
// SetWaypointFromDetection both depend on, so it is worth a live node:
//
//   keep = 1  exactly the historical flat keys, and NOTHING else — no .count, no .0.* —
//             holding the FIRST published match (vNext M0C #4; it was the top score)
//   keep = N  <out_key>.<i>.* + <out_key>.count, in the order the array was PUBLISHED
//
// The array below is deliberately NOT in score order, so a test that passed by
// accident would have to sort to do it.
//
// Everything is per-SUITE, not per-test, and that is forced by the runtime: DetectObject
// keeps ONE subscription per topic in a static map, created on the node of the FIRST
// tick (so the subscription outlives a tree). A fresh node per test would leave that
// feed subscribed on a destroyed node and every later tick would time out — which is
// exactly what happened when this test was first written.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"

namespace
{
vision_msgs::msg::Detection3D one(const std::string& class_id, double score, double x)
{
  vision_msgs::msg::Detection3D d;
  d.header.frame_id = "cam_optical";
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = class_id;
  h.hypothesis.score = score;
  h.pose.pose.position.x = x;
  h.pose.pose.orientation.w = 1.0;
  d.results.push_back(h);
  d.bbox.size.x = 0.01; d.bbox.size.y = 0.02; d.bbox.size.z = 0.03;
  return d;
}

// "" when the key is absent, which is how the keep=1 guarantee is asserted
std::string bbStr(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  try { return bb->get<std::string>(key); } catch (const std::exception&) { return {}; }
}
}  // namespace

// One detector publishing one ranked array; each test ticks DetectObject against it.
class DetectObjectTick : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_detect_object_tick");

    // cam_optical one metre above base_link: the detector reports in the sensor's
    // frame and DetectObject is what brings it into the planning frame.
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = "cam_optical";
    T.transform.translation.z = 1.0;
    T.transform.rotation.w = 1.0;
    tf_->setTransform(T, "test", true);
    // DetectObject looks the transform up with a timeout; tf2 logs an ERROR for that
    // unless the buffer is told another thread fills it. Here the fill happened above,
    // off the ticking thread, so the statement is true and the noise is not useful.
    tf_->setUsingDedicatedThread(true);

    ctx_.node = node_;
    ctx_.tf = tf_;

    // RANKED best first, and the best is NOT the highest score (ADR-0014: the rank
    // is not the score). The bottle is there to be filtered out by class.
    msg_.header.frame_id = "cam_optical";
    msg_.detections.push_back(one("capsule", 0.40, 0.10));
    msg_.detections.push_back(one("capsule", 0.90, 0.20));
    msg_.detections.push_back(one("capsule", 0.70, 0.30));
    msg_.detections.push_back(one("bottle", 0.99, 0.40));

    pub_ = node_->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/perception/capsule/detections", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    // DetectObject waits for an arrival NEWER than its tick, so the message has to
    // keep coming while the node waits; and the node needs spinning from elsewhere,
    // because the tick itself blocks.
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spinner_ = std::thread([] {
      while (!stop_ && rclcpp::ok()) exec_->spin_some(std::chrono::milliseconds(10));
    });
    talker_ = std::thread([] {
      while (!stop_ && rclcpp::ok())
      {
        pub_->publish(msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });
  }

  static void TearDownTestSuite()
  {
    stop_ = true;
    if (talker_.joinable()) talker_.join();
    if (spinner_.joinable()) spinner_.join();
    exec_->remove_node(node_);
    exec_.reset();
    pub_.reset();
    // EVERY reference to the node goes before rclcpp::shutdown(), the BtContext's
    // included: a node shared_ptr left in a static outlives shutdown, and destroying
    // it then SEGFAULTS inside fastrtps while it unregisters the parameter services.
    ctx_ = trainit::BtContext{};
    node_.reset();
    tf_.reset();
    rclcpp::shutdown();
  }

  static BT::NodeStatus tick(const std::string& attrs, const BT::Blackboard::Ptr& bb)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    const std::string xml =
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">"
      "<DetectObject " + attrs + "/></BehaviorTree></root>";
    factory.registerBehaviorTreeFromText(xml);
    auto tree = factory.createTree("T", bb);
    return tree.tickOnce();
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static trainit::BtContext ctx_;
  static vision_msgs::msg::Detection3DArray msg_;
  static rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  static std::thread spinner_, talker_;
  static std::atomic<bool> stop_;
};

rclcpp::Node::SharedPtr DetectObjectTick::node_;
std::shared_ptr<tf2_ros::Buffer> DetectObjectTick::tf_;
trainit::BtContext DetectObjectTick::ctx_{};
vision_msgs::msg::Detection3DArray DetectObjectTick::msg_;
rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr DetectObjectTick::pub_;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> DetectObjectTick::exec_;
std::thread DetectObjectTick::spinner_;
std::thread DetectObjectTick::talker_;
std::atomic<bool> DetectObjectTick::stop_{false};

// The guarantee every bundle in the field rests on.
TEST_F(DetectObjectTick, KeepOneWritesTheFirstPublishedOnTheHistoricalFlatKeys)
{
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(R"(detector="capsule" class_id="capsule" target_frame="base_link" timeout_ms="5000" out_key="d")", bb),
            BT::NodeStatus::SUCCESS);
  // the detector's BEST (published first, score 0.40) -- not the 0.90 it ranked lower --
  // transformed into base_link: x stays, z gains the metre
  EXPECT_EQ(bbStr(bb, "d.position"), "0.1;0;1");
  EXPECT_EQ(bbStr(bb, "d.score"), "0.4");
  EXPECT_EQ(bbStr(bb, "d.class_id"), "capsule");
  EXPECT_EQ(bbStr(bb, "d.frame"), "base_link");
  EXPECT_EQ(bbStr(bb, "d.size"), "0.01;0.02;0.03");
  EXPECT_EQ(bbStr(bb, "d.orientation"), "0;0;0;1");
  // ... and the list keys do NOT appear: at keep=1 the blackboard is what it was
  EXPECT_EQ(bbStr(bb, "d.count"), "");
  EXPECT_EQ(bbStr(bb, "d.0.position"), "");
}

TEST_F(DetectObjectTick, KeepThreeWritesTheListInThePublishedOrder)
{
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(R"(detector="capsule" class_id="capsule" target_frame="base_link" timeout_ms="5000" keep="3" out_key="d")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bbStr(bb, "d.count"), "3");
  // candidate 0 is the DETECTOR's first, not the best score — the whole claim
  EXPECT_EQ(bbStr(bb, "d.0.position"), "0.1;0;1");
  EXPECT_EQ(bbStr(bb, "d.0.score"), "0.4");
  EXPECT_EQ(bbStr(bb, "d.1.position"), "0.2;0;1");
  EXPECT_EQ(bbStr(bb, "d.2.position"), "0.3;0;1");
  EXPECT_EQ(bbStr(bb, "d.2.class_id"), "capsule");    // the bottle was filtered out
  EXPECT_EQ(bbStr(bb, "d.3.position"), "");           // only three matched
  EXPECT_EQ(bbStr(bb, "d.position"), "");             // keep>1 writes no flat keys
}

TEST_F(DetectObjectTick, KeepBeyondWhatArrivedPublishesOnlyWhatArrived)
{
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(R"(detector="capsule" class_id="capsule" timeout_ms="5000" keep="9" out_key="d")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bbStr(bb, "d.count"), "3");
  EXPECT_EQ(bbStr(bb, "d.2.position"), "0.3;0;1");
  EXPECT_EQ(bbStr(bb, "d.3.position"), "");
}

// A fresh message with no match is a FAILURE, list or no list — the bin is empty, and
// that is the stall the tree's Fallback handles.
TEST_F(DetectObjectTick, NoMatchFailsWithAndWithoutTheList)
{
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(R"(detector="capsule" class_id="ghost" timeout_ms="5000" keep="3" out_key="d")", bb),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(bbStr(bb, "d.count"), "");
  EXPECT_EQ(tick(R"(detector="capsule" class_id="ghost" timeout_ms="5000" out_key="d")", bb),
            BT::NodeStatus::FAILURE);
}

// min_score applies to the list the same way it applied to the single best.
TEST_F(DetectObjectTick, MinScoreFiltersTheListWithoutReordering)
{
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(R"(detector="capsule" class_id="capsule" timeout_ms="5000" min_score="0.5" keep="3" out_key="d")", bb),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bbStr(bb, "d.count"), "2");
  EXPECT_EQ(bbStr(bb, "d.0.position"), "0.2;0;1");   // 0.40 dropped, order preserved
  EXPECT_EQ(bbStr(bb, "d.1.position"), "0.3;0;1");
}
