// SetAttachedCollisionCheck and ResetScene, TICKED against a live graph (vNext M0C #5).
//
// Both nodes call a service of scene_manager_node. Their defaults were ABSOLUTE
// ("/scene_manager_node/..."), so a tree running in its robot's namespace (ADR-0016
// Decision 3) asked the ROOT, found nothing, logged a warning and returned SUCCESS: in a
// namespaced cell neither node ever did anything, and nothing said so. What this pins:
//
//   1. the defaults are RELATIVE: a tree in /r3 reaches /r3/scene_manager_node/<service>,
//      and the request really arrives (the SetBool value, the reset);
//   2. a service that does not answer while a scene manager DOES run is a FAILURE -- the
//      change the tree asked for was not applied -- not a skip;
//   3. with no scene manager anywhere in the graph it stays the documented no-op.
//
// At the root the defaults resolve to the old absolute names byte for byte; that is
// unit-tested in test_scene_service_names.cpp.
//
// Case 3 must see a graph with NO scene manager, so its suite runs FIRST (gtest runs suites
// in the order they are defined) and skips itself if a foreign one is already running.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"

namespace
{
const char* kTreeNs = "/r3";      // the tree runs where ADR-0016 puts it: in its robot's slot

bool anySceneManagerRuns(const rclcpp::Node::SharedPtr& node)
{
  for (const auto& n : node->get_node_names())
    if (n.size() >= 19 && n.compare(n.size() - 19, 19, "/scene_manager_node") == 0) return true;
  return false;
}

BT::NodeStatus tickOne(trainit::BtContext* ctx, const std::string& element)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, ctx);
  factory.registerBehaviorTreeFromText(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + element +
    "</BehaviorTree></root>");
  auto tree = factory.createTree("T", bb);
  return tree.tickOnce();
}
}  // namespace

// --- 3. no scene manager in the graph: the documented no-op ----------------------------
class SceneNodesWithoutASceneManager : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("trainit_run_bt_no_manager", kTreeNs);
    ctx_.node = node_;
  }
  static void TearDownTestSuite()
  {
    ctx_ = trainit::BtContext{};
    node_.reset();
  }
  static rclcpp::Node::SharedPtr node_;
  static trainit::BtContext ctx_;
};
rclcpp::Node::SharedPtr SceneNodesWithoutASceneManager::node_;
trainit::BtContext SceneNodesWithoutASceneManager::ctx_{};

TEST_F(SceneNodesWithoutASceneManager, BothNodesAreTheDocumentedNoOp)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500));      // let discovery settle
  if (anySceneManagerRuns(node_)) GTEST_SKIP() << "a foreign scene_manager_node is running";
  EXPECT_EQ(tickOne(&ctx_, "<SetAttachedCollisionCheck value=\"true\"/>"),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickOne(&ctx_, "<ResetScene/>"), BT::NodeStatus::SUCCESS);
}

// --- 1 and 2. a scene manager in the tree's own namespace ------------------------------
class SceneNodesWithASceneManager : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    tree_node_ = std::make_shared<rclcpp::Node>("trainit_run_bt", kTreeNs);
    manager_ = std::make_shared<rclcpp::Node>("scene_manager_node", kTreeNs);
    acc_ = manager_->create_service<std_srvs::srv::SetBool>(
      "~/attached_collision_check",
      [](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        last_value_ = req->data ? 1 : 0;
        res->success = true;
      });
    reset_ = manager_->create_service<std_srvs::srv::Trigger>(
      "~/reset_scene",
      [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ++resets_;
        res->success = true;
        res->message = "reset";
      });
    ctx_.node = tree_node_;
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    exec_->add_node(tree_node_);
    exec_->add_node(manager_);
    spinner_ = std::thread([] {
      while (!stop_ && rclcpp::ok()) exec_->spin_some(std::chrono::milliseconds(10));
    });
  }
  static void TearDownTestSuite()
  {
    stop_ = true;
    if (spinner_.joinable()) spinner_.join();
    exec_->remove_node(tree_node_);
    exec_->remove_node(manager_);
    exec_.reset();
    acc_.reset();
    reset_.reset();
    // every node reference goes before rclcpp::shutdown() (see test_detect_object_tick.cpp)
    ctx_ = trainit::BtContext{};
    tree_node_.reset();
    manager_.reset();
    rclcpp::shutdown();
  }
  static rclcpp::Node::SharedPtr tree_node_, manager_;
  static rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr acc_;
  static rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  static trainit::BtContext ctx_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spinner_;
  static std::atomic<bool> stop_;
  static std::atomic<int> last_value_, resets_;
};
rclcpp::Node::SharedPtr SceneNodesWithASceneManager::tree_node_;
rclcpp::Node::SharedPtr SceneNodesWithASceneManager::manager_;
rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr SceneNodesWithASceneManager::acc_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr SceneNodesWithASceneManager::reset_;
trainit::BtContext SceneNodesWithASceneManager::ctx_{};
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> SceneNodesWithASceneManager::exec_;
std::thread SceneNodesWithASceneManager::spinner_;
std::atomic<bool> SceneNodesWithASceneManager::stop_{false};
std::atomic<int> SceneNodesWithASceneManager::last_value_{-1};
std::atomic<int> SceneNodesWithASceneManager::resets_{0};

TEST_F(SceneNodesWithASceneManager, ANamespacedTreeReachesItsOwnSceneManager)
{
  ASSERT_EQ(tickOne(&ctx_, "<SetAttachedCollisionCheck value=\"true\"/>"),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(last_value_.load(), 1);                 // the value ARRIVED, not just "success"
  ASSERT_EQ(tickOne(&ctx_, "<SetAttachedCollisionCheck value=\"false\"/>"),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(last_value_.load(), 0);
  ASSERT_EQ(tickOne(&ctx_, "<ResetScene/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(resets_.load(), 1);
}

TEST_F(SceneNodesWithASceneManager, AServiceThatDoesNotAnswerWhileAManagerRunsFails)
{
  // the old default, absolute: from /r3 it asks the root, where nothing answers
  EXPECT_EQ(tickOne(&ctx_, "<SetAttachedCollisionCheck value=\"true\" "
                           "service=\"/scene_manager_node/attached_collision_check\"/>"),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickOne(&ctx_, "<ResetScene service=\"/scene_manager_node/reset_scene\"/>"),
            BT::NodeStatus::FAILURE);
}
