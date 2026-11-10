// TMR 2.8.1 (the M4.5 independent review): AttachObject / DetachObject report what move_group answered.
//
// Until 2.8.0 PlanningSceneManager ignored the answer of apply_planning_scene and both nodes always returned
// SUCCESS: an attach move_group refused (an unknown link) left the planner blind to the held part while the
// tree went on as if it held it. Pinned here, ticked for real on a MoveItMotionRuntime against the M2.5 fake
// move_group (its apply_planning_scene scripted to answer failure), on a private domain:
//
//   SF-01  a refused attach: FAILURE, `fault` PLANNING_SCENE_UPDATE_FAILED (the request WAS sent)
//   SF-02  a refused detach: FAILURE, `fault` PLANNING_SCENE_UPDATE_FAILED
//   SF-03  a detach of an object that is not attached: SUCCESS, no fault (idempotent -- move_group answers a
//          REMOVE with nothing to remove with success, which the fake does)
//   SF-04  unreadable data: FAILURE, `fault` INVALID_CONFIGURATION, nothing sent
//   SF-05  touch_links: the AttachedCollisionObject carries them; absent, the message every TMR before 2.8.1
//          sent (touch_links: [])
//   SF-06  an unremapped `fault`: the same FAILURE, nothing thrown
//
// The committed M2.7 captures (test_m2_7_scene_nodes) keep pinning the success path byte for byte.
#include <memory>
#include <string>

#include "m2_5/motion_harness.hpp"

namespace
{
struct Result
{
  std::string bt;
  std::string fault;      // "<unset>" when nothing was written
  std::string events;
  int applies{0};
};

Result tickScene(const std::string& xml, bool apply_succeeds)
{
  m25::World& w = m25::world();
  w.mg.resetScenario();
  w.mg.apply_succeeds = apply_succeeds;
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.dog.arm("build", std::chrono::seconds(60));
  w.rig.build(m25::defaultConfig());
  w.dog.disarm();
  auto bb = BT::Blackboard::create();
  w.log.reset();
  w.dog.arm(xml, std::chrono::seconds(60));
  Result r;
  r.bt = w.rig.tick(xml, bb);
  m25::settleTopics(w.log);
  w.dog.disarm();
  try { r.fault = bb->get<std::string>("f"); } catch (const std::exception&) { r.fault = "<unset>"; }
  r.events = w.log.events();
  r.applies = w.log.count("/apply_planning_scene");
  w.mg.resetScenario();
  return r;
}

const char* kAttach = "<AttachObject id=\"held_main\" link=\"tcp\" dims=\"0.04;0.04;0.03\" position=\"0;0;0.015\" "
                      "fault=\"{f}\"/>";
}  // namespace

TEST(M45SceneFaults, SF_01_a_refused_attach_is_a_fault)
{
  const Result r = tickScene(kAttach, false);
  EXPECT_EQ(r.bt, "FAILURE");
  EXPECT_EQ(r.fault, "PLANNING_SCENE_UPDATE_FAILED");
  EXPECT_EQ(r.applies, 1) << r.events;
}

TEST(M45SceneFaults, SF_02_a_refused_detach_is_a_fault)
{
  const Result r = tickScene("<DetachObject id=\"held_main\" link=\"tcp\" fault=\"{f}\"/>", false);
  EXPECT_EQ(r.bt, "FAILURE");
  EXPECT_EQ(r.fault, "PLANNING_SCENE_UPDATE_FAILED");
  EXPECT_EQ(r.applies, 1) << r.events;
}

TEST(M45SceneFaults, SF_03_a_detach_of_nothing_attached_is_success)
{
  const Result r = tickScene("<DetachObject id=\"never_attached\" link=\"tcp\" fault=\"{f}\"/>", true);
  EXPECT_EQ(r.bt, "SUCCESS");
  EXPECT_EQ(r.fault, "<unset>");
  // an attach that move_group applies is SUCCESS with no fault, too
  const Result a = tickScene(kAttach, true);
  EXPECT_EQ(a.bt, "SUCCESS");
  EXPECT_EQ(a.fault, "<unset>");
}

TEST(M45SceneFaults, SF_04_unreadable_data_is_a_configuration_fault_and_sends_nothing)
{
  for (const std::string xml :
       {"<AttachObject id=\"p\" dims=\"0.04\" fault=\"{f}\"/>",
        "<AttachObject id=\"p\" dims=\"0.04;0.04;0.03\" position=\"0;0\" fault=\"{f}\"/>",
        "<AttachObject id=\"p\" dims=\"0.04;0.04;0.03\" touch_links=\"tcp;suction link\" fault=\"{f}\"/>"})
  {
    const Result r = tickScene(xml, true);
    EXPECT_EQ(r.bt, "FAILURE") << xml;
    EXPECT_EQ(r.fault, "INVALID_CONFIGURATION") << xml;
    EXPECT_EQ(r.applies, 0) << xml << "\n" << r.events;
  }
}

TEST(M45SceneFaults, SF_05_touch_links_reach_the_attached_object)
{
  const Result with = tickScene("<AttachObject id=\"held_main\" link=\"tcp\" dims=\"0.04;0.04;0.03\" "
                                "touch_links=\"wrist3_link;suction_link;;tcp\" fault=\"{f}\"/>", true);
  ASSERT_EQ(with.bt, "SUCCESS");
  const auto at = with.events.find("touch_links:");
  ASSERT_NE(at, std::string::npos) << with.events;
  const std::string tail = with.events.substr(at, with.events.find("detach_posture:", at) - at);
  EXPECT_NE(tail.find("\"wrist3_link\""), std::string::npos) << tail;
  EXPECT_NE(tail.find("\"suction_link\""), std::string::npos) << tail;
  EXPECT_NE(tail.find("\"tcp\""), std::string::npos) << tail;
  EXPECT_LT(tail.find("\"wrist3_link\""), tail.find("\"suction_link\"")) << "in the order given";
  const Result without = tickScene(kAttach, true);
  ASSERT_EQ(without.bt, "SUCCESS");
  EXPECT_NE(without.events.find("touch_links: []"), std::string::npos) << without.events;
}

TEST(M45SceneFaults, SF_06_an_unremapped_fault_port_fails_the_same_and_throws_nothing)
{
  const Result r = tickScene("<AttachObject id=\"held_main\" link=\"tcp\" dims=\"0.04;0.04;0.03\"/>", false);
  EXPECT_EQ(r.bt, "FAILURE");
  EXPECT_EQ(r.fault, "<unset>");
  const Result d = tickScene("<DetachObject id=\"held_main\"/>", false);
  EXPECT_EQ(d.bt, "FAILURE");
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M27_DOMAIN);
}
