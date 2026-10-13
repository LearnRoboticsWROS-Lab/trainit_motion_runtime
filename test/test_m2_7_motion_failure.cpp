// vNext M2.7 (owner Q8.2): whose failure a failed motion is -- the pure classification table of
// motion_failure.hpp, the fault text, the TrajectoryValidator's cause tokens (on literal reports and on
// a real MoveIt model and scene), and MoveWaypoint's ports and fault write. No ROS graph.
//
//   TARGET  this target cannot be reached by this motion: the existing strategy applies (the next
//           candidate, the stall branch);
//   FAULT   anything that would recur for every target: never "try the next candidate", never
//           "bin empty". Default-deny: a failure is a FAULT unless the table names it TARGET.
//
// The expected classes below are LITERALS (WP_MF section 2.6), never computed from the code under
// test; every MoveIt error code is taken from the installed moveit_msgs message definition, so a code
// moveit_msgs adds later fails here until it is classified.
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <memory>

#include <behaviortree_cpp/bt_factory.h>
#include <geometric_shapes/shapes.h>
#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/robot_model_test_utils.h>

#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/motion_failure.hpp"
#include "trainit_motion_runtime/trajectory_validator.hpp"

using trainit::MotionFailureClass;
using trainit::MotionPhase;
using trainit::MotionResult;

namespace
{
const std::vector<MotionPhase> kPhases = {MotionPhase::NONE, MotionPhase::REQUEST, MotionPhase::PLAN,
                                          MotionPhase::VALIDATE, MotionPhase::EXECUTE,
                                          MotionPhase::PLAN_AND_EXECUTE};

// One MoveIt error code: its name in moveit_msgs 2.2.3 ("" = a value MoveIt does not name), and its
// class in a failed PLAN answer and in a failed PLAN_AND_EXECUTE answer ('T' TARGET, 'F' FAULT).
struct Row
{
  int code;
  const char* name;
  char plan;
  char plan_and_execute;
};
const std::vector<Row> kTable = {
  {1, "SUCCESS", 'F', 'F'},          // a failed result that says SUCCESS: INCONSISTENT_RESULT
  {99999, "FAILURE", 'F', 'F'},      // MoveGroupInterface's own "server not ready", a pipeline error
  {-1, "PLANNING_FAILED", 'T', 'T'},
  {-2, "INVALID_MOTION_PLAN", 'T', 'T'},
  {-3, "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE", 'F', 'F'},
  {-4, "CONTROL_FAILED", 'F', 'F'},
  {-5, "UNABLE_TO_AQUIRE_SENSOR_DATA", 'F', 'F'},
  {-6, "TIMED_OUT", 'T', 'F'},       // the planner's own budget for THIS goal; Pilz never returns it
  {-7, "PREEMPTED", 'F', 'F'},
  {-10, "START_STATE_IN_COLLISION", 'F', 'F'},   // the start is the robot's current state
  {-11, "START_STATE_VIOLATES_PATH_CONSTRAINTS", 'F', 'F'},
  {-26, "START_STATE_INVALID", 'F', 'F'},
  {-12, "GOAL_IN_COLLISION", 'T', 'T'},
  {-13, "GOAL_VIOLATES_PATH_CONSTRAINTS", 'T', 'T'},
  {-14, "GOAL_CONSTRAINTS_VIOLATED", 'T', 'T'},
  {-27, "GOAL_STATE_INVALID", 'T', 'T'},
  {-28, "UNRECOGNIZED_GOAL_TYPE", 'F', 'F'},
  {-15, "INVALID_GROUP_NAME", 'F', 'F'},
  {-16, "INVALID_GOAL_CONSTRAINTS", 'F', 'F'},
  {-17, "INVALID_ROBOT_STATE", 'F', 'F'},
  {-18, "INVALID_LINK_NAME", 'F', 'F'},
  {-19, "INVALID_OBJECT_NAME", 'F', 'F'},
  {-21, "FRAME_TRANSFORM_FAILURE", 'F', 'F'},
  {-22, "COLLISION_CHECKING_UNAVAILABLE", 'F', 'F'},
  {-23, "ROBOT_STATE_STALE", 'F', 'F'},
  {-24, "SENSOR_INFO_STALE", 'F', 'F'},
  {-25, "COMMUNICATION_FAILURE", 'F', 'F'},
  {-29, "CRASH", 'F', 'F'},
  {-30, "ABORT", 'F', 'F'},
  {-31, "NO_IK_SOLUTION", 'T', 'T'},
  // values MoveIt does not name: default-deny
  {0, "", 'F', 'F'},
  {2, "", 'F', 'F'},
  {-8, "", 'F', 'F'},
};
const std::set<int> kPlanTargets = {-1, -2, -6, -12, -13, -14, -27, -31};

MotionResult failed(MotionPhase phase, int code, const std::string& cause = "")
{
  MotionResult r = MotionResult::failure("a stage", code);
  r.phase = phase;
  r.cause = cause;
  return r;
}

MotionFailureClass cls(char c) { return c == 'T' ? MotionFailureClass::TARGET : MotionFailureClass::FAULT; }

std::string name(MotionFailureClass c)
{
  switch (c)
  {
    case MotionFailureClass::NONE: return "NONE";
    case MotionFailureClass::TARGET: return "TARGET";
    case MotionFailureClass::FAULT: return "FAULT";
  }
  return "?";
}

std::string read(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}  // namespace

TEST(M27MotionFailure, success_is_never_classified)
{
  for (const MotionPhase p : kPhases)
    for (const char* cause : {"", trainit::kCauseServerUnavailable, trainit::kCauseJointJump})
    {
      MotionResult r = MotionResult::ok("done");
      r.phase = p;
      r.cause = cause;
      EXPECT_EQ(trainit::classifyMotionFailure(r), MotionFailureClass::NONE) << trainit::toString(p) << " " << cause;
    }
  // the runtime records nothing on success
  const MotionResult ok = MotionResult::ok();
  EXPECT_EQ(ok.phase, MotionPhase::NONE);
  EXPECT_TRUE(ok.cause.empty());
  const MotionResult plain;
  EXPECT_EQ(plain.phase, MotionPhase::NONE);
  EXPECT_TRUE(plain.cause.empty());
}

TEST(M27MotionFailure, the_plan_table)
{
  std::set<int> targets;
  for (const Row& row : kTable)
  {
    const auto got = trainit::classifyMotionFailure(failed(MotionPhase::PLAN, row.code));
    EXPECT_EQ(name(got), name(cls(row.plan))) << "PLAN code " << row.code << " " << row.name;
    if (got == MotionFailureClass::TARGET) targets.insert(row.code);
  }
  EXPECT_EQ(targets, kPlanTargets);
}

TEST(M27MotionFailure, the_plan_and_execute_table)
{
  std::set<int> targets;
  for (const Row& row : kTable)
  {
    const auto got = trainit::classifyMotionFailure(failed(MotionPhase::PLAN_AND_EXECUTE, row.code));
    EXPECT_EQ(name(got), name(cls(row.plan_and_execute))) << "PLAN_AND_EXECUTE code " << row.code << " " << row.name;
    if (got == MotionFailureClass::TARGET) targets.insert(row.code);
  }
  std::set<int> want = kPlanTargets;
  want.erase(-6);   // TIMED_OUT in one plan-and-execute request is the execution's
  EXPECT_EQ(targets, want);
}

TEST(M27MotionFailure, execute_none_request_are_always_faults)
{
  for (const MotionPhase p : {MotionPhase::EXECUTE, MotionPhase::NONE, MotionPhase::REQUEST, MotionPhase::VALIDATE})
    for (const Row& row : kTable)
      EXPECT_EQ(trainit::classifyMotionFailure(failed(p, row.code)), MotionFailureClass::FAULT)
        << trainit::toString(p) << " code " << row.code << " " << row.name;
}

TEST(M27MotionFailure, validation_causes)
{
  using namespace trainit;
  for (const char* c : {kCauseJointJump, kCauseCollision})
    EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::VALIDATE, 1, c)), MotionFailureClass::TARGET) << c;
  for (const char* c : {kCauseEmptyTrajectory, kCauseNonFinite, kCauseJointLimits, kCauseNonMonotonicTime,
                        kCauseUnknownGroup, kCauseValidationFailed, kCauseStartStateCollision})
    EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::VALIDATE, 1, c)), MotionFailureClass::FAULT) << c;
  // a collision at waypoint 0 (the robot's start) has its own token, never the path's
  EXPECT_STREQ(kCauseStartStateCollision, "START_STATE_COLLISION");
  EXPECT_STREQ(kCauseCollision, "COLLISION");
  EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::VALIDATE, 1)), MotionFailureClass::FAULT) << "no cause";
  // a validator's TARGET cause names this path only in the VALIDATE phase
  for (const MotionPhase p : kPhases)
  {
    if (p == MotionPhase::VALIDATE) continue;
    for (const char* c : {kCauseJointJump, kCauseCollision})
      EXPECT_EQ(classifyMotionFailure(failed(p, -31, c)), MotionFailureClass::FAULT) << toString(p) << " " << c;
  }
}

TEST(M27MotionFailure, transport_causes_win_over_a_target_code)
{
  using namespace trainit;
  for (const char* c : {kCauseServerUnavailable, kCauseGoalSendTimeout, kCauseGoalRejected, kCauseResultTimeout})
    for (const int code : {-31, -1, 0})
      EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::PLAN_AND_EXECUTE, code, c)), MotionFailureClass::FAULT)
        << c << " with code " << code;
  for (const char* c : {kCauseEmptyWaypoint, kCauseUnknownMotion, kCauseServoExecution, kCauseUnknownExecution})
    EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::REQUEST, -31, c)), MotionFailureClass::FAULT) << c;
  EXPECT_EQ(classifyMotionFailure(failed(MotionPhase::NONE, -31, kCauseException)), MotionFailureClass::FAULT);
}

TEST(M27MotionFailure, the_fault_text)
{
  using namespace trainit;
  const std::vector<std::pair<MotionResult, std::string>> cases = {
    {failed(MotionPhase::EXECUTE, -4), "MOTION_FAULT: CONTROL_FAILED during execute"},
    {failed(MotionPhase::EXECUTE, -6), "MOTION_FAULT: TIMED_OUT during execute"},
    {failed(MotionPhase::EXECUTE, 99999), "MOTION_FAULT: FAILURE during execute"},
    {failed(MotionPhase::PLAN, 99999), "MOTION_FAULT: FAILURE during plan"},
    {failed(MotionPhase::PLAN, -10), "MOTION_FAULT: START_STATE_IN_COLLISION during plan"},
    {failed(MotionPhase::PLAN, -21), "MOTION_FAULT: FRAME_TRANSFORM_FAILURE during plan"},
    {failed(MotionPhase::PLAN, -7), "MOTION_FAULT: PREEMPTED during plan"},
    {failed(MotionPhase::PLAN, -8), "MOTION_FAULT: CODE_-8 during plan"},
    {failed(MotionPhase::PLAN, 2), "MOTION_FAULT: CODE_2 during plan"},
    {failed(MotionPhase::PLAN, 1), "MOTION_FAULT: INCONSISTENT_RESULT during plan"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 1), "MOTION_FAULT: INCONSISTENT_RESULT during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 0), "MOTION_FAULT: NO_ERROR_CODE during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, -4), "MOTION_FAULT: CONTROL_FAILED during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 0, kCauseServerUnavailable),
     "MOTION_FAULT: SERVER_UNAVAILABLE during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 0, kCauseGoalSendTimeout),
     "MOTION_FAULT: GOAL_SEND_TIMEOUT during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 0, kCauseGoalRejected), "MOTION_FAULT: GOAL_REJECTED during plan_and_execute"},
    {failed(MotionPhase::PLAN_AND_EXECUTE, 0, kCauseResultTimeout),
     "MOTION_FAULT: RESULT_TIMEOUT during plan_and_execute"},
    {failed(MotionPhase::VALIDATE, 1, kCauseEmptyTrajectory), "MOTION_FAULT: EMPTY_TRAJECTORY during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseNonFinite), "MOTION_FAULT: NON_FINITE during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseJointLimits), "MOTION_FAULT: JOINT_LIMITS during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseNonMonotonicTime), "MOTION_FAULT: NON_MONOTONIC_TIME during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseUnknownGroup), "MOTION_FAULT: UNKNOWN_GROUP during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseValidationFailed), "MOTION_FAULT: VALIDATION_FAILED during validate"},
    {failed(MotionPhase::VALIDATE, 1, kCauseStartStateCollision),
     "MOTION_FAULT: START_STATE_COLLISION during validate"},
    {failed(MotionPhase::REQUEST, 0, kCauseEmptyWaypoint), "MOTION_FAULT: EMPTY_WAYPOINT during request"},
    {failed(MotionPhase::REQUEST, 0, kCauseUnknownMotion), "MOTION_FAULT: UNKNOWN_MOTION during request"},
    {failed(MotionPhase::REQUEST, 0, kCauseServoExecution), "MOTION_FAULT: SERVO_EXECUTION during request"},
    {failed(MotionPhase::REQUEST, 0, kCauseUnknownExecution), "MOTION_FAULT: UNKNOWN_EXECUTION during request"},
    {failed(MotionPhase::NONE, 0, kCauseException), "MOTION_FAULT: EXCEPTION"},
    {failed(MotionPhase::NONE, -4), "MOTION_FAULT: CONTROL_FAILED"},
  };
  for (const auto& [r, want] : cases) EXPECT_EQ(motionFaultText(r), want);
  // every FAULT of the PLAN table: the code's name (or the token) during plan
  for (const Row& row : kTable)
  {
    if (row.plan != 'F') continue;
    std::string token = row.name;
    if (row.code == 1) token = "INCONSISTENT_RESULT";
    else if (row.code == 0) token = "NO_ERROR_CODE";
    else if (token.empty()) token = "CODE_" + std::to_string(row.code);
    EXPECT_EQ(motionFaultText(failed(MotionPhase::PLAN, row.code)), "MOTION_FAULT: " + token + " during plan");
  }
}

TEST(M27MotionFailure, the_phase_tokens)
{
  const std::map<MotionPhase, std::string> want = {
    {MotionPhase::NONE, ""}, {MotionPhase::REQUEST, "request"}, {MotionPhase::PLAN, "plan"},
    {MotionPhase::VALIDATE, "validate"}, {MotionPhase::EXECUTE, "execute"},
    {MotionPhase::PLAN_AND_EXECUTE, "plan_and_execute"}};
  ASSERT_EQ(want.size(), kPhases.size());
  for (const MotionPhase p : kPhases) EXPECT_EQ(trainit::toString(p), want.at(p));
}

// Every constant of the INSTALLED moveit_msgs/msg/MoveItErrorCodes.msg is named by its own name, and
// the literal table above holds exactly those constants (plus the three values MoveIt does not name).
TEST(M27MotionFailure, every_moveit_code_is_named)
{
  const std::string msg = read(TRAINIT_MOVEIT_ERROR_CODES_MSG);
  ASSERT_FALSE(msg.empty()) << TRAINIT_MOVEIT_ERROR_CODES_MSG;
  const std::regex constant("^int32 ([A-Z_]+)=(-?[0-9]+)\\s*$");
  std::map<int, std::string> defined;
  std::istringstream in(msg);
  std::string line;
  while (std::getline(in, line))
  {
    std::smatch m;
    if (std::regex_match(line, m, constant)) defined[std::stoi(m[2])] = m[1];
  }
  EXPECT_EQ(defined.size(), 30u) << "moveit_msgs 2.2.3 defines 30 error codes";
  std::map<int, std::string> table;
  for (const Row& row : kTable)
    if (*row.name != '\0') table[row.code] = row.name;
  EXPECT_EQ(table, defined) << "the literal table must hold exactly the installed constants";
  for (const auto& [code, n] : defined) EXPECT_EQ(trainit::moveItErrorCodeName(code), n) << code;
  for (const int unnamed : {0, 2, -8, -9, -20, 100000})
    EXPECT_STREQ(trainit::moveItErrorCodeName(unnamed), "") << unnamed;
}

// The TrajectoryValidator's verdict -> its cause token, in validate()'s own order
// (trainit::validationCause, trajectory_validator.hpp), and the class of each. Each report is the one
// validate() returns at that check: the flags it has set so far. A collision at waypoint 0 is the
// trajectory's START, the robot's current state, and MoveIt plans from a colliding start without
// refusing it (move_group accepts a path whose only invalid waypoint is 0; Pilz never checks the start
// for collision; FixStartStateCollision puts the colliding start back as waypoint 0): it would recur for
// every candidate, a FAULT. A collision first met at a later waypoint is this path's: TARGET.
TEST(M27MotionFailure, the_validation_cause_of_every_check)
{
  using trainit::ValidationReport;
  // the flags validate() has set when it returns
  auto report = [](bool non_empty, bool finite, bool within_limits, bool monotonic) {
    ValidationReport r;
    r.non_empty = non_empty;
    r.finite = finite;
    r.within_joint_limits = within_limits;
    r.timestamps_monotonic = monotonic;
    return r;
  };
  ValidationReport group;
  group.reason = "unknown planning group 'arm'";
  const ValidationReport empty = report(false, false, false, false);
  const ValidationReport non_finite = report(true, false, true, false);
  const ValidationReport limits = report(true, true, false, false);
  ValidationReport jump = report(true, true, true, false);
  jump.joint_jump = true;
  const ValidationReport time = report(true, true, true, false);
  auto collision = [&report](std::optional<std::size_t> first) {
    ValidationReport r = report(true, true, true, true);
    r.collision_checked = true;
    r.collision_free = false;
    r.first_colliding_waypoint = first;
    return r;
  };
  ValidationReport passed = report(true, true, true, true);
  passed.collision_checked = true;
  passed.valid = true;
  ValidationReport unchecked_but_colliding = report(true, true, true, true);   // never returned by validate()
  unchecked_but_colliding.collision_free = false;
  unchecked_but_colliding.first_colliding_waypoint = 0;

  struct Case
  {
    const char* what;
    ValidationReport report;
    std::string cause;
    char cls;   // 'T' TARGET, 'F' FAULT
  };
  const std::vector<Case> cases = {
    {"unknown planning group", group, "UNKNOWN_GROUP", 'F'},
    {"empty trajectory", empty, "EMPTY_TRAJECTORY", 'F'},
    {"non-finite joint value", non_finite, "NON_FINITE", 'F'},
    {"joint limits violated", limits, "JOINT_LIMITS", 'F'},
    {"joint jump", jump, "JOINT_JUMP", 'T'},
    {"non-monotonic time", time, "NON_MONOTONIC_TIME", 'F'},
    {"collision first at waypoint 0: the start", collision(0), "START_STATE_COLLISION", 'F'},
    {"collision first at waypoint 1", collision(1), "COLLISION", 'T'},
    {"collision first at waypoint 6", collision(6), "COLLISION", 'T'},
    {"collision with no waypoint named", collision(std::nullopt), "VALIDATION_FAILED", 'F'},
    {"collision not checked", unchecked_but_colliding, "VALIDATION_FAILED", 'F'},
    {"every check passed", passed, "VALIDATION_FAILED", 'F'},
  };
  for (const Case& c : cases)
  {
    EXPECT_EQ(std::string(trainit::validationCause(c.report)), c.cause) << c.what;
    const MotionResult r = failed(MotionPhase::VALIDATE, 1, c.cause);   // what planAndExecute records
    EXPECT_EQ(name(trainit::classifyMotionFailure(r)), name(cls(c.cls))) << c.what;
    if (c.cls == 'F')
    {
      EXPECT_EQ(trainit::motionFaultText(r), "MOTION_FAULT: " + c.cause + " during validate") << c.what;
    }
  }
}

namespace
{
// One continuous joint about z sweeps a 0.4 m box (its centre 0.5 m out) through a 0.1 m world box at
// (0.5, 0, 0): the link collides at q = 0 and is free from q = 0.25 on. Steps of 0.25 rad (no joint
// jump at the default 0.5), 0.1 s apart.
moveit_msgs::msg::RobotTrajectory sweep(const std::vector<double>& qs)
{
  moveit_msgs::msg::RobotTrajectory t;
  t.joint_trajectory.joint_names = {"base_link-link_a-joint"};
  for (std::size_t k = 0; k < qs.size(); ++k)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {qs[k]};
    p.time_from_start.sec = static_cast<int32_t>(k / 10);
    p.time_from_start.nanosec = static_cast<uint32_t>((k % 10) * 100000000);
    t.joint_trajectory.points.push_back(p);
  }
  return t;
}

std::vector<double> steps(double from, double to)
{
  std::vector<double> q;
  const int n = static_cast<int>(std::lround(std::abs(to - from) / 0.25));
  for (int k = 0; k <= n; ++k) q.push_back(from + (to - from) * k / n);
  return q;
}
}  // namespace

// The fact validationCause reads, from the REAL TrajectoryValidator on a real MoveIt model and planning
// scene (no ROS graph): PlanningScene::isPathValid names every invalid waypoint, and the report keeps
// the first. A trajectory that STARTS in collision -- what MoveIt hands back for a robot whose current
// state collides -- names waypoint 0 even when it collides later too; one that collides only further on
// names that waypoint; a free one, or one checked without a scene, names none. Every other field and
// the reason text are those validate() wrote before vNext M2.7.
TEST(M27MotionFailure, the_validator_names_the_first_colliding_waypoint)
{
  moveit::core::RobotModelBuilder builder("m27_sweep_arm", "base_link");
  builder.addChain("base_link->link_a", "continuous", {geometry_msgs::msg::Pose()}, urdf::Vector3(0, 0, 1));
  geometry_msgs::msg::Pose box;
  box.position.x = 0.5;
  builder.addCollisionBox("link_a", {0.4, 0.05, 0.05}, box);
  builder.addGroupChain("base_link", "link_a", "arm");
  ASSERT_TRUE(builder.isValid());
  const moveit::core::RobotModelPtr model = builder.build();
  ASSERT_TRUE(model);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  Eigen::Isometry3d at = Eigen::Isometry3d::Identity();
  at.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  scene->getWorldNonConst()->addToObject("obstacle", std::make_shared<shapes::Box>(0.1, 0.1, 0.1), at);
  const trainit::TrajectoryValidator validator(model, "arm");

  struct Case
  {
    const char* what;
    std::vector<double> q;
    bool with_scene;
    bool valid;
    std::optional<std::size_t> first;
    std::string reason;
    std::string cause;   // "" for a valid trajectory
  };
  std::vector<double> there_and_back = steps(0.0, 1.5);
  for (double q : steps(1.5, 0.0)) if (q != 1.5) there_and_back.push_back(q);
  const std::vector<Case> cases = {
    {"starts in collision (q 0 -> 1.5)", steps(0.0, 1.5), true, false, 0,
     "collision at waypoint(s) starting 0", "START_STATE_COLLISION"},
    {"starts and ends in collision (q 0 -> 1.5 -> 0)", there_and_back, true, false, 0,
     "collision at waypoint(s) starting 0", "START_STATE_COLLISION"},
    {"ends in collision (q 1.5 -> 0)", steps(1.5, 0.0), true, false, 6, "collision at waypoint(s) starting 6",
     "COLLISION"},
    {"collision-free (q 1.5 -> 2.5)", steps(1.5, 2.5), true, true, std::nullopt, "ok", ""},
    {"no scene: the collision check is skipped (q 0 -> 1.5)", steps(0.0, 1.5), false, true, std::nullopt, "ok", ""},
  };
  ASSERT_EQ(there_and_back.size(), 13u);
  for (const Case& c : cases)
  {
    const trainit::ValidationReport r =
      validator.validate(sweep(c.q), c.with_scene ? scene : planning_scene::PlanningScenePtr(), 0.5);
    EXPECT_EQ(r.valid, c.valid) << c.what;
    EXPECT_EQ(r.reason, c.reason) << c.what;
    EXPECT_EQ(r.first_colliding_waypoint, c.first) << c.what;
    EXPECT_EQ(r.collision_checked, c.with_scene) << c.what;
    EXPECT_EQ(r.collision_free, c.valid) << c.what;
    EXPECT_TRUE(r.non_empty && r.finite && r.within_joint_limits && r.timestamps_monotonic && !r.joint_jump) << c.what;
    if (!c.cause.empty())
    {
      EXPECT_EQ(std::string(trainit::validationCause(r)), c.cause) << c.what;
      const MotionResult m = failed(MotionPhase::VALIDATE, 1, trainit::validationCause(r));
      EXPECT_EQ(name(trainit::classifyMotionFailure(m)), c.cause == "COLLISION" ? "TARGET" : "FAULT") << c.what;
    }
  }
}

// MoveWaypoint's `fault` write never throws out of tick() (vNext M2.7, TMR-7 review). BT.CPP's setOutput
// THROWS when the remapped key already holds a value of another type (Blackboard::set): the write is
// dropped with an ERROR line and the node returns FAILURE, as for any refusal. Two ways the key gets
// another type: a Script writes a number into it, or a later write of a number retypes a key preset as
// a string (an untyped bt_params value). Ticked with no BtContext: an unknown motion is refused before
// the runtime is asked. Every write of the port in MoveWaypoint::tick goes through the one guarded
// helper (its refusals, the catch block, a runtime FAULT): checked in the source.
TEST(M27MotionFailure, a_fault_write_that_throws_never_escapes_tick)
{
  auto load = [](BT::BehaviorTreeFactory& factory, const std::string& nodes) {
    trainit::registerAllNodes(factory);
    factory.registerBehaviorTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + nodes +
      "</BehaviorTree></root>");
  };
  {
    BT::BehaviorTreeFactory factory;
    load(factory, "<Sequence><Script code=\"f:=5\"/><MoveWaypoint waypoint=\"a\" fault=\"{f}\"/></Sequence>");
    auto bb = BT::Blackboard::create();
    bb->set<std::string>("a.type", "tcp");
    bb->set<std::string>("a.motion", "srvo");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    EXPECT_NO_THROW(status = tree.tickOnce()) << "a Script wrote a number into the fault key";
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    ASSERT_NE(bb->getEntry("f"), nullptr);
    EXPECT_FALSE(bb->getEntry("f")->value.isString()) << "the fault text was not written over the number";
  }
  {
    BT::BehaviorTreeFactory factory;
    load(factory, "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>");
    auto bb = BT::Blackboard::create();
    bb->set<std::string>("a.type", "tcp");
    bb->set<std::string>("a.motion", "srvo");
    bb->set<std::string>("f", "");
    auto tree = factory.createTree("T", bb);
    bb->set<int>("f", 7);   // the untyped string entry is retyped by its first typed write
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    EXPECT_NO_THROW(status = tree.tickOnce()) << "the fault key was retyped to int";
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(bb->get<int>("f"), 7);
  }
  // the source: one setOutput("fault", ...) in MoveWaypoint::tick, inside the helper, and three calls of it
  const std::string src = read(std::string(TRAINIT_SOURCE_DIR) + "/src/bt_nodes.cpp");
  const auto a = src.find("BT::NodeStatus MoveWaypoint::tick()");
  ASSERT_NE(a, std::string::npos);
  const auto b = src.find("\nBT::", a + 1);
  ASSERT_NE(b, std::string::npos);
  const std::string tick = src.substr(a, b - a);
  auto count = [&tick](const std::string& needle) {
    int n = 0;
    for (auto p = tick.find(needle); p != std::string::npos; p = tick.find(needle, p + 1)) ++n;
    return n;
  };
  EXPECT_EQ(count("setOutput(\"fault\""), 1);
  EXPECT_EQ(count("try { return setOutput(\"fault\", text).has_value(); }"), 1);
  EXPECT_EQ(count("writeFault("), 3) << "the refusals, the catch block and a runtime FAULT";
}

// MoveWaypoint declares exactly {waypoint: input, fault: output}; `fault` has NO default (a tree that
// does not remap it writes nothing), and the Groot palette lists the same ports.
TEST(M27MotionFailure, move_waypoint_ports_are_waypoint_and_an_optional_fault)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  ASSERT_EQ(factory.manifests().count("MoveWaypoint"), 1u);
  std::map<std::string, std::string> manifest;
  for (const auto& [port, info] : factory.manifests().at("MoveWaypoint").ports)
    manifest[port] = info.direction() == BT::PortDirection::OUTPUT ? "output" : "input";
  EXPECT_EQ(manifest, (std::map<std::string, std::string>{{"waypoint", "input"}, {"fault", "output"}}));
  const auto& fault = factory.manifests().at("MoveWaypoint").ports.at("fault");
  EXPECT_TRUE(fault.defaultValueString().empty()) << "'" << fault.defaultValueString() << "'";
  EXPECT_EQ(fault.description(), "MOTION_FAULT: CAUSE during PHASE");

  const std::string xml = read(std::string(TRAINIT_SOURCE_DIR) + "/config/bt/trainit_nodes.groot.xml");
  const auto a = xml.find("<Action ID=\"MoveWaypoint\">");
  ASSERT_NE(a, std::string::npos) << "MoveWaypoint is not in the Groot model";
  const std::string block = xml.substr(a, xml.find("</Action>", a) - a);
  std::map<std::string, std::string> model;
  const std::regex port("<(input|output)_port name=\"([A-Za-z_]+)\"");
  for (auto it = std::sregex_iterator(block.begin(), block.end(), port); it != std::sregex_iterator(); ++it)
    model[(*it)[2]] = (*it)[1];
  EXPECT_EQ(model, manifest);
  EXPECT_EQ(block.find("default="), std::string::npos) << "the Groot model gives the fault port no default";
}

// A tree that does not remap `fault` loads, and its blackboard never gains the key; one that remaps it
// gets it. Ticked with no BtContext: an unknown motion is refused before the runtime is asked.
TEST(M27MotionFailure, an_unremapped_fault_port_writes_nothing)
{
  auto tick = [](const std::string& node, BT::Blackboard::Ptr bb) {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    factory.registerBehaviorTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + node +
      "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    return tree.tickOnce();
  };
  auto plain = BT::Blackboard::create();
  plain->set<std::string>("a.type", "tcp");
  plain->set<std::string>("a.motion", "srvo");
  EXPECT_EQ(tick("<MoveWaypoint waypoint=\"a\"/>", plain), BT::NodeStatus::FAILURE);
  EXPECT_EQ(plain->getEntry("fault"), nullptr);
  std::vector<std::string> keys;
  for (const auto& k : plain->getKeys()) keys.emplace_back(k);
  EXPECT_EQ(keys.size(), 2u) << "no key was added";

  auto remapped = BT::Blackboard::create();
  remapped->set<std::string>("a.type", "tcp");
  remapped->set<std::string>("a.motion", "srvo");
  remapped->set<std::string>("f", "");
  EXPECT_EQ(tick("<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>", remapped), BT::NodeStatus::FAILURE);
  EXPECT_EQ(remapped->get<std::string>("f"), "MOTION_FAULT: UNKNOWN_MOTION during request");
}
