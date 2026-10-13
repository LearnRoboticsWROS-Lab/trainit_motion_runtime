// vNext M2.5: the door names no MoveIt C++ API, and can be implemented without MoveIt.
//
// This translation unit includes ONLY motion_runtime.hpp. MoveIt's C++ headers define
// MOVEIT_CLASS_FORWARD (test_m2_5_state_reads.cpp holds the canary that proves they do), so if
// the interface ever pulls one in, this file stops compiling. MoveIt MESSAGE types (a
// RobotTrajectory, the error codes in MotionResult) are allowed: they are the ROS contract, not
// MoveIt's C++ API. FakeMotionRuntime implements every pure virtual, so a second implementation
// (a test double, a future backend) needs nothing from MoveIt.
#include <type_traits>

#include <gtest/gtest.h>

#include "trainit_motion_runtime/motion_runtime.hpp"

#ifdef MOVEIT_CLASS_FORWARD
#error "motion_runtime.hpp includes MoveIt's C++ API: the MotionRuntime door must name only messages"
#endif

namespace
{
class FakeMotionRuntime : public trainit::MotionRuntime
{
public:
  trainit::MotionResult movePtp(const trainit::JointTarget&, const trainit::MotionOptions&) override
  { return trainit::MotionResult::ok("ptp"); }
  trainit::MotionResult moveLinear(const trainit::CartesianTarget&, const trainit::MotionOptions&) override
  { return trainit::MotionResult::ok("lin"); }
  trainit::MotionResult moveCircular(const trainit::CircularTarget&, const trainit::MotionOptions&) override
  { return trainit::MotionResult::ok("circ"); }
  trainit::MotionResult moveCollisionFree(const trainit::MotionTarget&, const trainit::PlanningOptions&) override
  { return trainit::MotionResult::ok("free"); }
  trainit::MotionResult executeProcessSequence(const std::vector<trainit::ProcessSegment>&,
                                               const trainit::MotionOptions&) override
  { return trainit::MotionResult::ok("sequence"); }
  trainit::MotionResult executeCartesianProcess(const trainit::CartesianProcessPath&,
                                                const trainit::MotionOptions&) override
  { return trainit::MotionResult::ok("process"); }
  trainit::MotionResult executeTrajectory(const moveit_msgs::msg::RobotTrajectory&) override
  { return trainit::MotionResult::ok("execute"); }
  trainit::MotionResult stop() override { return trainit::MotionResult::ok("stop"); }
  void setPlannerMode(trainit::PlannerMode mode) override { mode_ = mode; }
  trainit::PlannerMode plannerMode() const override { return mode_; }
  geometry_msgs::msg::PoseStamped currentTipPose() const override
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = 0.5;
    p.pose.orientation.w = 1.0;
    return p;
  }
  // vNext M2.7 (owner decision 11.2): the frames and the named-state FK, std/geometry_msgs only
  const std::string& tipLink() const override { return tip_; }
  const std::string& baseFrame() const override { return base_; }
  std::optional<geometry_msgs::msg::Pose> namedPose(const std::string& named) const override
  {
    if (named != "home") return std::nullopt;
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    return p;
  }

private:
  trainit::PlannerMode mode_{trainit::PlannerMode::OMPL};
  std::string tip_{"tcp"};
  std::string base_{"base_link"};
};
}  // namespace

// The proof is at compile time: FakeMotionRuntime is concrete (every pure virtual implemented,
// nothing from MoveIt), and this unit compiled without MOVEIT_CLASS_FORWARD. Its include closure
// is checked separately (test_m2_5_interface_closure). The call below only shows it links.
static_assert(!std::is_abstract_v<FakeMotionRuntime>, "the fake must implement the whole door");

TEST(M25InterfacePurity, the_door_is_implementable_without_moveit)
{
  FakeMotionRuntime fake;
  const trainit::MotionRuntime& door = fake;
  EXPECT_EQ(door.currentTipPose().header.frame_id, "world");
  EXPECT_EQ(door.tipLink(), "tcp");
  EXPECT_EQ(door.baseFrame(), "base_link");
  EXPECT_TRUE(door.namedPose("home").has_value());
  EXPECT_FALSE(door.namedPose("nowhere").has_value());
}
