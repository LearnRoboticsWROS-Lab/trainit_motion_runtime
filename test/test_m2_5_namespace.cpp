// vNext M2.5 (tests first): where each MoveIt request lands when the tree runs in /r3.
//
// Two binaries are built from this file:
//   test_m2_5_namespace        the runtime node is CONSTRUCTED in /r3 (namespace.txt)
//   test_m2_5_namespace_remap  the whole process runs with --ros-args -r __ns:=/r3, the way a
//                              generated namespaced launch starts trainit_run_bt
//                              (namespace_remap.txt)
// Two fake sets serve the same endpoints, one at the ROOT and one in /r3, so every capture says
// which one each request reached.
//
// Pinned since vNext M2.7 (owner decision D2; the namespace fix, ADR-0023 Decision 3): the
// runtime's MoveGroupInterface and PlanningSceneInterface resolve in the node's namespace, and both
// binaries compare with the AFTER series in test/data/m2_7_ns/. The BEFORE record (TMR 1.8.0,
// test/data/m2_5/namespace*.txt, kept byte for byte) has /move_action, /execute_trajectory and
// /compute_cartesian_path at the ROOT; test_m2_7_namespace_diff proves the two series differ only
// by that resolution. NS-03 shows, stage by stage, that the constructor no longer waits for a root
// endpoint. The scenario titles ("pins:") keep the M2.5 wording verbatim, "at the root" included.
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "m2_5/motion_harness.hpp"

using m25::Config;
using m25::FakeMoveGroup;
using m25::Scenario;
using trainit::PlannerMode;

namespace
{
#ifdef TRAINIT_M25_REMAP
const char* kFamily = "namespace_remap";
const char* kRigNs = "/";              // the process-wide remap moves it to /r3
#else
const char* kFamily = "namespace";
const char* kRigNs = "/r3";
#endif

const geometry_msgs::msg::Pose kP1 = m25::pose(0.40, 0.10, 0.30, 1.0, 0.0, 0.0, 0.0);
const geometry_msgs::msg::Pose kP2 = m25::pose(0.45, 0.15, 0.30, 1.0, 0.0, 0.0, 0.0);

// One per process (ServoTrack's static caches bind to the first node): the root fakes start
// with NOTHING served, so NS-03 can open them one by one.
struct NsWorld
{
  m25::Log log;
  FakeMoveGroup root{log, "/", 0u};
  FakeMoveGroup r3{log, "/r3"};
  m25::FakeJointStates js_r3{"/r3", {0.0, 0.0, 0.0}};
  m25::FakeJointStates js_root{"/", {0.3, -0.4, 0.6}};
  m25::Rig rig{kRigNs};
  m25::Watchdog dog;
};

NsWorld& nsWorld()
{
  static std::unique_ptr<NsWorld> w;
  if (!w)
  {
    w = std::make_unique<NsWorld>();
    m25::teardowns().push_back([] { w.reset(); });
  }
  return *w;
}

std::string storedPose(BT::Blackboard& bb)
{
  try { return "stored: cur=Pose" + m25::describe(bb.get<geometry_msgs::msg::Pose>("cur")) + "\n"; }
  catch (const std::exception&) { return "stored: (none)\n"; }
}
}  // namespace

// NS-03: the runtime constructor, with the root endpoints opened one at a time.
TEST(M25Namespace, ns03_where_the_constructor_waits)
{
  NsWorld& w = nsWorld();
  m25::Family f(std::string(kFamily) + "_constructor", TRAINIT_M27_NS_DATA_DIR);
  std::ostringstream os;
  os << "=== scenario NS-03\n"
     << "pins: where MoveItMotionRuntime's constructor waits when only the /r3 fakes serve\n"
     << "node: " << w.rig.node()->get_fully_qualified_name() << "\n";
  std::atomic<bool> built{false};
  w.log.reset();
  w.dog.arm("NS-03", std::chrono::seconds(120));
  std::thread builder([&] { w.rig.build(m25::defaultConfig()); built = true; });
  // A blocked constructor stays blocked whatever the wait; an unblocked one finishes in well
  // under a second. Each stage waits up to 10 s for it (early exit), so a loaded machine cannot
  // move the completion into the next stage.
  auto stage = [&](const std::string& what) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!built && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    os << "after " << what << ", within 10 s: constructed=" << (built ? "yes" : "no") << "\n";
  };
  stage("only the /r3 fakes");
  w.root.start(FakeMoveGroup::MOVE_ACTION);
  stage("root /move_action");
  w.root.start(FakeMoveGroup::EXECUTE);
  stage("root /execute_trajectory");
  w.root.start(FakeMoveGroup::SCENE_SERVICES);
  stage("root get_planning_scene + apply_planning_scene");
  builder.join();
  w.dog.disarm();
  os << "events:\n" << w.log.events() << "counts: " << w.log.counts() << "\n\n";
  // the rest of the root set, for the scenarios below
  w.root.start(FakeMoveGroup::ALL);
  f.add(os.str());
  f.check();
}

// NS-01 / NS-02: every motion family, a state read and ServoTrack, from /r3.
TEST(M25Namespace, ns01_where_each_request_lands)
{
  NsWorld& w = nsWorld();
  const std::string ns = w.rig.node()->get_namespace();
  m25::Family f(kFamily, TRAINIT_M27_NS_DATA_DIR);
  auto run = [&](const Scenario& s) {
    w.r3.resetScenario();
    return m25::run(w.log, w.root, w.js_r3, w.rig, w.dog, ns, s);
  };
  std::unique_ptr<m25::FakeServo> servo;
  f.add(run({"NS-01a", "planned moves from /r3: MGI goals at the root, the Pilz sequence and the PSM in /r3",
             [](Config& c) { c.mode = PlannerMode::OMPL; }, {}, {{"p1", kP1}, {"p2", kP2}},
             {"<MoveToNamed target=\"ready\"/>", "<MoveLinear pose_key=\"p1\"/>",
              "<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"}}));
  f.add(run({"NS-01b", "a cartesian process path from /r3: compute_cartesian_path at the root",
             [](Config& c) { c.process_backend = trainit::ProcessPathBackend::CARTESIAN_WAYPOINTS; },
             {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"}}));
  Scenario reads{"NS-01c",
                 "state reads use <ns>/joint_states: /r3 publishes home, the root publishes ready; "
                 "tick 2 stops the root publisher, tick 3 stops /r3 and restarts the root",
                 nullptr, {}, {},
                 {"<StoreCurrentPose pose_key=\"cur\"/>", "<StoreCurrentPose pose_key=\"cur\"/>",
                  "<StoreCurrentPose pose_key=\"cur\"/>"}};
  int tick = 0;
  reads.report = [&](BT::Blackboard& bb) {
    const std::string out = storedPose(bb);
    ++tick;
    if (tick == 1) w.js_root.on(false);
    if (tick == 2) { w.js_r3.on(false); w.js_root.on(true); }
    return out;
  };
  reads.after = [&] { w.js_r3.on(true); w.js_root.on(true); };
  f.add(run(reads));
  Scenario servo_s{"NS-01d", "ServoTrack's servo_node is relative: the Trigger calls reach <ns>/servo_n1",
                   nullptr,
                   {{"t.type", "tcp"}, {"t.position", "0.55;0.0;0.4"}},
                   {}, {"<ServoTrack waypoint=\"t\" servo_node=\"servo_n1\" timeout_ms=\"300\"/>"}};
  servo_s.before_tick = [&] { servo = std::make_unique<m25::FakeServo>(w.log, "/r3", "servo_n1"); };
  servo_s.report = [&](BT::Blackboard&) { return servo->twists(); };
  servo_s.after = [&] { servo.reset(); };
  f.add(run(servo_s));
  f.check();
}

int main(int argc, char** argv)
{
#ifdef TRAINIT_M25_REMAP
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN, {"--ros-args", "-r", "__ns:=/r3"});
#else
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
#endif
}
