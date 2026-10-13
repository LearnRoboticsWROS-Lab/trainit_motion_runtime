// The planning-scene service names scene_manager_node drives (ADR-0015 Decision 3).
//
// Two things must hold at once:
//   * a node in the GLOBAL namespace must resolve exactly the absolute names that
//     were hardcoded before (/apply_planning_scene, /get_planning_scene), because
//     generated bundles are already running against them;
//   * a node in a namespace (a multi-robot cell's /r3) must resolve to that namespace instead of
//     writing to the one global service four robots would share.
// Both are properties of rclcpp name resolution applied to a RELATIVE default, so
// the test creates real clients and reads the names back.
#include <gtest/gtest.h>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include "trainit_motion_runtime/scene_service_names.hpp"

using trainit_scene::kApplyPlanningSceneServiceDefault;
using trainit_scene::kGetPlanningSceneServiceDefault;
using trainit_scene::resolveSceneServiceName;
using trainit_scene::kAttachedCollisionCheckServiceDefault;
using trainit_scene::kResetSceneServiceDefault;

TEST(SceneServiceNames, AnEmptyOverrideMeansTheRelativeDefault)
{
  EXPECT_EQ(resolveSceneServiceName("", kApplyPlanningSceneServiceDefault), "apply_planning_scene");
  EXPECT_EQ(resolveSceneServiceName("   ", kGetPlanningSceneServiceDefault), "get_planning_scene");
  // the default is RELATIVE: that is the whole mechanism
  EXPECT_NE(std::string(kApplyPlanningSceneServiceDefault)[0], '/');
  EXPECT_NE(std::string(kGetPlanningSceneServiceDefault)[0], '/');
}

TEST(SceneServiceNames, AnExplicitOverrideWins)
{
  EXPECT_EQ(resolveSceneServiceName("/apply_planning_scene", kApplyPlanningSceneServiceDefault),
            "/apply_planning_scene");
  EXPECT_EQ(resolveSceneServiceName(" /r1/get_planning_scene\n", kGetPlanningSceneServiceDefault),
            "/r1/get_planning_scene");
}

class SceneServiceResolution : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static std::string applyName(const std::string& ns, const std::string& override_name)
  {
    auto node = std::make_shared<rclcpp::Node>("scene_manager_node", ns);
    auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      resolveSceneServiceName(override_name, kApplyPlanningSceneServiceDefault));
    return client->get_service_name();
  }
  static std::string getName(const std::string& ns, const std::string& override_name)
  {
    auto node = std::make_shared<rclcpp::Node>("scene_manager_node", ns);
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
      resolveSceneServiceName(override_name, kGetPlanningSceneServiceDefault));
    return client->get_service_name();
  }
};

// The byte-identical guarantee for every bundle already in the field.
TEST_F(SceneServiceResolution, InTheGlobalNamespaceTheNamesAreUnchanged)
{
  EXPECT_EQ(applyName("", ""), "/apply_planning_scene");
  EXPECT_EQ(getName("", ""), "/get_planning_scene");
}

// What a multi-robot cell needs: /r3's move_group, reached with no configuration at all.
TEST_F(SceneServiceResolution, ANamespacedNodeFollowsItsOwnNamespace)
{
  EXPECT_EQ(applyName("/r3", ""), "/r3/apply_planning_scene");
  EXPECT_EQ(getName("/r3", ""), "/r3/get_planning_scene");
  // four robots, four distinct services -- the point of the change
  EXPECT_NE(applyName("/r1", ""), applyName("/r2", ""));
}

// The escape hatch: a namespaced scene manager driving a global move_group.
TEST_F(SceneServiceResolution, AnAbsoluteOverrideEscapesTheNamespace)
{
  EXPECT_EQ(applyName("/r3", "/apply_planning_scene"), "/apply_planning_scene");
  EXPECT_EQ(getName("/r3", "/r1/get_planning_scene"), "/r1/get_planning_scene");
}

// The scene manager's own services, as a BT node in the same slot asks for them
// (vNext M0C #5): the old absolute defaults at the root, byte for byte; the slot's own
// scene manager from /r3. Resolved by a client on a node named like the tree runner.
TEST_F(SceneServiceResolution, TheSceneManagersOwnServicesFollowTheTreesNamespace)
{
  auto name = [](const std::string& ns, const char* relative) {
    auto node = std::make_shared<rclcpp::Node>("trainit_run_bt", ns);
    return std::string(node->create_client<std_srvs::srv::Trigger>(relative)->get_service_name());
  };
  EXPECT_EQ(name("", kAttachedCollisionCheckServiceDefault),
            "/scene_manager_node/attached_collision_check");
  EXPECT_EQ(name("", kResetSceneServiceDefault), "/scene_manager_node/reset_scene");
  EXPECT_EQ(name("/r3", kAttachedCollisionCheckServiceDefault),
            "/r3/scene_manager_node/attached_collision_check");
  EXPECT_EQ(name("/r3", kResetSceneServiceDefault), "/r3/scene_manager_node/reset_scene");
}
