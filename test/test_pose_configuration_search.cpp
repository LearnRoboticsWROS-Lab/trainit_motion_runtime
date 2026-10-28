// TMR 2.6.2 (D7): the IK solutions a pose goal's configuration is chosen from depend on the start
// state, the pose and the scene alone -- never on the clock or the load. The 2.6.0 search gave each
// seed a 50 ms budget inside which KDL re-seeds at random, and the ROS2ML course cell moved to 'place'
// in another arm posture in 2 runs of 8 (2026-10-28). An inline UR5-like arm with MoveIt's KDL plugin,
// the solver the cells use. A taught configuration and any other solver keep the 2.6.0 search.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "trainit_motion_runtime/pose_configuration.hpp"
#include "trainit_motion_runtime/pose_configuration_search.hpp"

namespace
{
// Another solver class (KDL inside): the runtime does not know its single attempt, so it gets the 2.6.0
// timed search and the choice is flagged as one that may differ between runs.
class OtherSolver : public kinematics::KinematicsBase
{
public:
  OtherSolver(kinematics::KinematicsBasePtr kdl, const moveit::core::RobotModel& model) : kdl_(std::move(kdl))
  {
    storeValues(model, kdl_->getGroupName(), kdl_->getBaseFrame(), kdl_->getTipFrames(),
                kdl_->getSearchDiscretization());
  }
  bool getPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& seed, std::vector<double>& sol,
                     moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions& o) const override
  {
    return kdl_->getPositionIK(p, seed, sol, e, o);
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& seed, double t,
                        std::vector<double>& sol, moveit_msgs::msg::MoveItErrorCodes& e,
                        const kinematics::KinematicsQueryOptions& o) const override
  {
    return kdl_->searchPositionIK(p, seed, t, sol, e, o);
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& seed, double t,
                        const std::vector<double>& lim, std::vector<double>& sol, moveit_msgs::msg::MoveItErrorCodes& e,
                        const kinematics::KinematicsQueryOptions& o) const override
  {
    return kdl_->searchPositionIK(p, seed, t, lim, sol, e, o);
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& seed, double t,
                        std::vector<double>& sol, const IKCallbackFn& cb, moveit_msgs::msg::MoveItErrorCodes& e,
                        const kinematics::KinematicsQueryOptions& o) const override
  {
    return kdl_->searchPositionIK(p, seed, t, sol, cb, e, o);
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose& p, const std::vector<double>& seed, double t,
                        const std::vector<double>& lim, std::vector<double>& sol, const IKCallbackFn& cb,
                        moveit_msgs::msg::MoveItErrorCodes& e, const kinematics::KinematicsQueryOptions& o) const override
  {
    return kdl_->searchPositionIK(p, seed, t, lim, sol, cb, e, o);
  }
  bool getPositionFK(const std::vector<std::string>& links, const std::vector<double>& q,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    return kdl_->getPositionFK(links, q, poses);
  }
  const std::vector<std::string>& getJointNames() const override { return kdl_->getJointNames(); }
  const std::vector<std::string>& getLinkNames() const override { return kdl_->getLinkNames(); }

private:
  kinematics::KinematicsBasePtr kdl_;
};

// CPU burners for a loaded machine, always stopped and joined (a failed ASSERT returns early)
class Load
{
public:
  Load()
  {
    for (unsigned i = 0; i < std::max(2u, std::thread::hardware_concurrency()); ++i)
      threads_.emplace_back([this] {
        volatile double x = 0.0;
        while (!stop_) x = x + std::sqrt(x + 1.0);
      });
  }
  ~Load()
  {
    stop_ = true;
    for (auto& t : threads_) t.join();
  }
  Load(const Load&) = delete;
  Load& operator=(const Load&) = delete;

private:
  std::atomic<bool> stop_{ false };
  std::vector<std::thread> threads_;
};

// the UR5's kinematics (ur_description, before calibration files), joints +-2*pi
const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="ur5_like">
  <link name="base_link"/><link name="shoulder_link"/><link name="upper_arm_link"/><link name="forearm_link"/>
  <link name="wrist_1_link"/><link name="wrist_2_link"/><link name="wrist_3_link"/><link name="tool0"/>
  <joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.089159" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="150" velocity="3.15"/></joint>
  <joint name="shoulder_lift_joint" type="revolute"><parent link="shoulder_link"/><child link="upper_arm_link"/>
    <origin xyz="0 0.13585 0" rpy="0 1.5707963 0"/><axis xyz="0 1 0"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="150" velocity="3.15"/></joint>
  <joint name="elbow_joint" type="revolute"><parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin xyz="0 -0.1197 0.425" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.1415926" upper="3.1415926" effort="150" velocity="3.15"/></joint>
  <joint name="wrist_1_joint" type="revolute"><parent link="forearm_link"/><child link="wrist_1_link"/>
    <origin xyz="0 0 0.39225" rpy="0 1.5707963 0"/><axis xyz="0 1 0"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="28" velocity="3.2"/></joint>
  <joint name="wrist_2_joint" type="revolute"><parent link="wrist_1_link"/><child link="wrist_2_link"/>
    <origin xyz="0 0.093 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="28" velocity="3.2"/></joint>
  <joint name="wrist_3_joint" type="revolute"><parent link="wrist_2_link"/><child link="wrist_3_link"/>
    <origin xyz="0 0 0.09465" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="28" velocity="3.2"/></joint>
  <joint name="tool0_fixed" type="fixed"><parent link="wrist_3_link"/><child link="tool0"/>
    <origin xyz="0 0.0823 0" rpy="0 0 1.5707963"/></joint>
</robot>)";
const char* kSrdf = R"(<?xml version="1.0"?>
<robot name="ur5_like"><group name="arm"><chain base_link="base_link" tip_link="tool0"/></group></robot>)";

class PoseConfigurationSearch : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pose_configuration_search");
    auto urdf = urdf::parseURDF(kUrdf);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, kSrdf));
    model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    loader_ = std::make_unique<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
      "moveit_core", "kinematics::KinematicsBase");
    solver_ = loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    ASSERT_TRUE(solver_->initialize(node_, *model_, "arm", "base_link", { "tool0" }, 0.005));
    jmg_ = model_->getJointModelGroup("arm");
    use(solver_);
  }
  void SetUp() override
  {
    ASSERT_NE(jmg_, nullptr) << "the KDL plugin could not be set up for the inline arm";
    use(solver_);
  }
  static void use(const kinematics::KinematicsBasePtr& solver)
  {
    jmg_->setSolverAllocators([solver](const moveit::core::JointModelGroup*) { return solver; });
  }
  static void TearDownTestSuite()
  {
    solver_.reset();
    jmg_ = nullptr;
    model_.reset();
    loader_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  static std::vector<trainit::JointBounds> bounds()
  {
    std::vector<trainit::JointBounds> out;
    for (const std::string& name : jmg_->getVariableNames())
    {
      const moveit::core::VariableBounds& b = model_->getVariableBounds(name);
      const auto* rev = dynamic_cast<const moveit::core::RevoluteJointModel*>(model_->getJointOfVariable(name));
      out.push_back({ b.min_position_, b.max_position_, rev != nullptr && rev->isContinuous(), rev != nullptr });
    }
    return out;
  }
  static moveit::core::RobotState state(const std::vector<double>& q)
  {
    moveit::core::RobotState s(model_);
    s.setToDefaultValues();
    s.setJointGroupPositions(jmg_, q);
    s.update();
    return s;
  }
  static Eigen::Isometry3d poseOf(const std::vector<double>& q)
  {
    return state(q).getGlobalLinkTransform("tool0");
  }
  // head: {start} (no taught configuration) or {taught, start}
  static trainit::ConfigurationSearch find(const std::vector<double>& start_q, const Eigen::Isometry3d& goal,
                                           const std::vector<std::vector<double>>& head,
                                           const moveit::core::GroupStateValidityCallbackFn& valid = {})
  {
    const moveit::core::RobotState start = state(start_q);
    return trainit::findConfigurations(start, jmg_, goal, "tool0", head, head.size() == 2, valid, head.front(),
                                       bounds());
  }

  static rclcpp::Node::SharedPtr node_;
  static moveit::core::RobotModelPtr model_;
  static moveit::core::JointModelGroup* jmg_;
  static std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
  static kinematics::KinematicsBasePtr solver_;
};

rclcpp::Node::SharedPtr PoseConfigurationSearch::node_;
moveit::core::RobotModelPtr PoseConfigurationSearch::model_;
moveit::core::JointModelGroup* PoseConfigurationSearch::jmg_ = nullptr;
std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> PoseConfigurationSearch::loader_;
kinematics::KinematicsBasePtr PoseConfigurationSearch::solver_;

// A two-joint arm with no solver (the facing seed is geometry only): its first joint's type, axis and range.
moveit::core::RobotModelPtr smallArm(const std::string& type, const std::string& axis, double lower, double upper)
{
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="small"><link name="base_link"/><link name="link1"/><link name="link2"/><link name="tip"/>
  <joint name="j1" type=")" + type + R"("><parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz=")" + axis + R"("/>
    <limit lower=")" + std::to_string(lower) + R"(" upper=")" + std::to_string(upper) + R"(" effort="1" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="tip_fixed" type="fixed"><parent link="link2"/><child link="tip"/><origin xyz="0.4 0 0" rpy="0 0 0"/></joint>
</robot>)";
  auto model = urdf::parseURDF(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*model, R"(<?xml version="1.0"?>
<robot name="small"><group name="arm"><chain base_link="base_link" tip_link="tip"/></group></robot>)");
  return std::make_shared<moveit::core::RobotModel>(model, srdf);
}

// the facing seed on a small arm: q at the start, the goal where q_goal puts the tip
std::optional<std::vector<double>> facingOnSmallArm(const moveit::core::RobotModelPtr& model, const std::vector<double>& q,
                                                    const std::vector<double>& q_goal)
{
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup("arm");
  std::vector<trainit::JointBounds> bounds;
  for (const std::string& name : jmg->getVariableNames())
  {
    const moveit::core::VariableBounds& b = model->getVariableBounds(name);
    const auto* rev = dynamic_cast<const moveit::core::RevoluteJointModel*>(model->getJointOfVariable(name));
    bounds.push_back({ b.min_position_, b.max_position_, rev != nullptr && rev->isContinuous(), rev != nullptr });
  }
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setJointGroupPositions(jmg, q);
  start.update();
  moveit::core::RobotState at(start);
  at.setJointGroupPositions(jmg, q_goal);
  at.update();
  return trainit::facingConfiguration(start, jmg, at.getGlobalLinkTransform("tip"), "tip", bounds);
}

// a start like the course cell's post-pick, a goal a quarter turn away on the other side of the arm
const std::vector<double> kStart = { 0.0, -1.2, 1.4, -1.8, -1.57, 0.0 };
const std::vector<double> kElsewhere = { 1.9, -1.0, 1.1, -1.6, -1.57, 0.4 };
}  // namespace

TEST_F(PoseConfigurationSearch, TheSameStartAndPoseFindTheSameSolutionsEvenUnderLoad)
{
  const Eigen::Isometry3d goal = poseOf(kElsewhere);
  const trainit::ConfigurationSearch first = find(kStart, goal, { kStart });
  ASSERT_FALSE(first.solutions.empty());
  EXPECT_FALSE(first.timed);
  const trainit::ConfigurationChoice choice = trainit::chooseConfiguration(first.solutions, kStart, bounds(),
                                                                           std::nullopt);
  ASSERT_TRUE(choice.joints.has_value());
  EXPECT_EQ(trainit::solverClassName(jmg_), "kdl_kinematics_plugin::KDLKinematicsPlugin");

  const Load load;   // the 2.6.0 search's 50 ms per seed bought fewer random re-seeds on a loaded machine
  for (int run = 0; run < 20; ++run)
  {
    const trainit::ConfigurationSearch again = find(kStart, goal, { kStart });
    EXPECT_EQ(again.solutions, first.solutions) << "run " << run;
    EXPECT_FALSE(again.timed);
    const trainit::ConfigurationChoice c = trainit::chooseConfiguration(again.solutions, kStart, bounds(),
                                                                        std::nullopt);
    EXPECT_TRUE(c.joints.has_value() && *c.joints == *choice.joints) << "run " << run;
  }
}

TEST_F(PoseConfigurationSearch, ARejectedBranchIsNeverFoundAndTheRestStaysTheSame)
{
  // the collision check's place: a validity callback (here: the elbow on one side only)
  const moveit::core::GroupStateValidityCallbackFn elbow_down = [](moveit::core::RobotState*,
                                                                   const moveit::core::JointModelGroup*,
                                                                   const double* q) { return q[2] < 0.0; };
  const Eigen::Isometry3d goal = poseOf(kElsewhere);
  const trainit::ConfigurationSearch first = find(kStart, goal, { kStart }, elbow_down);
  ASSERT_FALSE(first.solutions.empty());
  for (const auto& q : first.solutions) EXPECT_LT(q[2], 0.0);
  for (int run = 0; run < 5; ++run) EXPECT_EQ(find(kStart, goal, { kStart }, elbow_down).solutions, first.solutions);
}

TEST_F(PoseConfigurationSearch, AnotherSolverKeepsTheTimedSearchAndSaysSo)
{
  use(std::make_shared<OtherSolver>(solver_, *model_));
  const trainit::ConfigurationSearch found = find(kStart, poseOf(kElsewhere), { kStart });
  EXPECT_FALSE(found.solutions.empty());
  EXPECT_TRUE(found.timed);   // logged: the choice may differ between runs
  const trainit::ConfigurationSearch taught = find(kStart, poseOf(kElsewhere), { kElsewhere, kStart });
  EXPECT_FALSE(taught.timed);   // a taught configuration is never flagged: its own seed is the answer
}

TEST_F(PoseConfigurationSearch, ATaughtConfigurationIsFoundFromItselfAndEndsTheSearch)
{
  const trainit::ConfigurationSearch found = find(kStart, poseOf(kElsewhere), { kElsewhere, kStart });
  ASSERT_EQ(found.solutions.size(), 1u);   // the first seed is the answer: no other is tried
  EXPECT_FALSE(found.timed);
  for (std::size_t i = 0; i < kElsewhere.size(); ++i)
    EXPECT_NEAR(found.solutions[0][i], kElsewhere[i], 1e-3) << "joint " << i;
}

TEST_F(PoseConfigurationSearch, AFarMoveInTheArmsOwnBranchIsFoundAndChosen)
{
  // the course cell's 'place': the base turns ~1.97 rad to reach it in the branch the arm is in; one
  // attempt from each spread seed found only another branch (the elbow, 2.81 rad) there. (This arm's
  // spread seeds find it too, with no scene: the cell's case is the Gazebo check's, 8 runs.)
  std::vector<double> turned = kStart;
  turned[0] += 1.9;
  const trainit::ConfigurationSearch found = find(kStart, poseOf(turned), { kStart });
  EXPECT_FALSE(found.timed);
  const trainit::ConfigurationChoice c = trainit::chooseConfiguration(found.solutions, kStart, bounds(), std::nullopt);
  ASSERT_TRUE(c.joints.has_value());
  EXPECT_NEAR(c.max_difference, 1.9, 1e-3);
  EXPECT_EQ(c.worst_joint, 0u);   // shoulder_pan_joint
  for (std::size_t i = 0; i < turned.size(); ++i) EXPECT_NEAR((*c.joints)[i], turned[i], 1e-3) << "joint " << i;
}

TEST_F(PoseConfigurationSearch, FacingTurnsOnlyTheFirstJointToTheGoal)
{
  // what found the course cell's 'place' in the arm's own branch: the base turned to face it
  std::vector<double> turned = kStart;
  turned[0] += 1.9;
  const std::optional<std::vector<double>> facing =
    trainit::facingConfiguration(state(kStart), jmg_, poseOf(turned), "tool0", bounds());
  ASSERT_TRUE(facing.has_value());
  for (std::size_t i = 0; i < turned.size(); ++i) EXPECT_NEAR((*facing)[i], turned[i], 1e-9) << "joint " << i;
}

TEST_F(PoseConfigurationSearch, FacingStaysInsideTheFirstJointsRange)
{
  std::vector<double> start = kStart;
  start[0] = 5.8;                                        // near +2*pi: the turn wraps the other way
  std::vector<double> turned = start;
  turned[0] += 1.0;                                      // 6.8 > 2*pi
  const std::optional<std::vector<double>> facing =
    trainit::facingConfiguration(state(start), jmg_, poseOf(turned), "tool0", bounds());
  ASSERT_TRUE(facing.has_value());
  EXPECT_NEAR((*facing)[0], 6.8 - 6.283185307179586, 1e-9);
}

TEST_F(PoseConfigurationSearch, FacingHasNoAnswerForAGoalOnTheFirstJointsAxis)
{
  Eigen::Isometry3d above = Eigen::Isometry3d::Identity();
  above.translation() = Eigen::Vector3d(0.0, 0.0, 0.8);
  EXPECT_FALSE(trainit::facingConfiguration(state(kStart), jmg_, above, "tool0", bounds()).has_value());
}

TEST_F(PoseConfigurationSearch, TheSingleAttemptsStartFromTheStartThenTheFacingSeed)
{
  // the wiring: without the facing seed in the list, the course cell found only the elbow branch
  std::vector<double> turned = kStart;
  turned[0] += 1.9;
  const std::vector<std::vector<double>> seeds =
    trainit::singleAttemptSeeds(state(kStart), jmg_, poseOf(turned), "tool0", { kStart }, bounds());
  ASSERT_GE(seeds.size(), 66u);   // the start, the facing seed (+ its carries), the 64 of the series
  EXPECT_EQ(seeds[0], kStart);
  for (std::size_t i = 0; i < turned.size(); ++i) EXPECT_NEAR(seeds[1][i], turned[i], 1e-9) << "joint " << i;
  EXPECT_EQ(trainit::singleAttemptSeeds(state(kStart), jmg_, poseOf(turned), "tool0", { kStart }, bounds()), seeds);
}

TEST_F(PoseConfigurationSearch, FacingFollowsTheJointNotTheAxisSign)
{
  for (const char* axis : { "0 0 1", "0 0 -1" })
  {
    const std::optional<std::vector<double>> facing =
      facingOnSmallArm(smallArm("revolute", axis, -3.0, 3.0), { 0.2, 0.3 }, { 1.2, 0.3 });
    ASSERT_TRUE(facing.has_value()) << axis;
    EXPECT_NEAR((*facing)[0], 1.2, 1e-9) << axis;
    EXPECT_NEAR((*facing)[1], 0.3, 1e-12) << axis;   // the other joints as they are
  }
}

TEST_F(PoseConfigurationSearch, FacingHasNoAnswerOutsideTheRangeOrForALinearFirstJoint)
{
  EXPECT_FALSE(facingOnSmallArm(smallArm("revolute", "0 0 1", -0.5, 0.5), { 0.2, 0.3 }, { 1.2, 0.3 }).has_value());
  EXPECT_FALSE(facingOnSmallArm(smallArm("prismatic", "0 0 1", -0.5, 0.5), { 0.2, 0.3 }, { 0.3, 0.3 }).has_value());
}

TEST_F(PoseConfigurationSearch, FacingNeedsATipTheStateKnows)
{
  EXPECT_FALSE(trainit::facingConfiguration(state(kStart), jmg_, poseOf(kElsewhere), "no_such_frame", bounds())
                 .has_value());
}

TEST_F(PoseConfigurationSearch, ASmallMoveStaysInTheStartsConfiguration)
{
  std::vector<double> near = kStart;
  near[0] += 0.05;
  near[1] -= 0.04;
  const trainit::ConfigurationSearch found = find(kStart, poseOf(near), { kStart });
  const trainit::ConfigurationChoice c = trainit::chooseConfiguration(found.solutions, kStart, bounds(), std::nullopt);
  ASSERT_TRUE(c.joints.has_value());
  EXPECT_LT(c.max_difference, 0.2);
}

TEST_F(PoseConfigurationSearch, APoseOutOfReachFindsNothingAndIsNotCalledTimed)
{
  Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
  far.translation() = Eigen::Vector3d(3.0, 0.0, 0.5);   // the arm reaches under 1 m
  const trainit::ConfigurationSearch found = find(kStart, far, { kStart });
  EXPECT_TRUE(found.solutions.empty());
  EXPECT_FALSE(found.timed);
}
