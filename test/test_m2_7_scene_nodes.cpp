// vNext M2.7 (tests BEFORE, owner decision 11.2): what the four planning-scene BT nodes send today.
//
// AddCollisionObject, RemoveCollisionObject, AttachObject and DetachObject write the planning
// scene through the runner's PlanningSceneManager (src/bt_nodes.cpp:661-699; at the freeze they
// reach it as BtContext::runtime->scene()). No earlier capture ticks them: TaskRunner's payload
// steps are in task_runner.txt, these nodes nowhere. Frozen on production adf94b0 (TMR 2.0.0)
// BEFORE the BtContext/TaskRunner narrowing touches a production line; every later commit must
// reproduce it byte for byte (test/data/m2_7/README.md).
//
// Two binaries are built from this file:
//   test_m2_7_scene_nodes        the root namespace (scene_nodes.txt)
//   test_m2_7_scene_nodes_remap  the whole process runs with --ros-args -r __ns:=/r3, the way a
//                                generated namespaced launch starts trainit_run_bt
//                                (scene_nodes_remap.txt); a root and an /r3 fake set both serve,
//                                so the capture says which one each scene write reached.
// Never a runtime node CONSTRUCTED in /r3: the M2.7 namespace fix moves those writes on purpose.
//
// The M2.5 fakes record what the nodes send: an apply_planning_scene request is an ordered EVENT
// (PlanningSceneInterface calls the service and waits), a removal is a planning_scene message
// (PlanningSceneInterface PUBLISHES it: an asynchronous TOPIC, printed sorted after the tick).
// No tick publishes more than one topic message: the interface's publisher keeps one sample.
// The trees use the ports, the defaults and the blackboard forms TSA generates
// (applications/pick_and_place.py: dims="{scene_<id>_dims}", position="{scene_<id>_pos}",
// dims="{payload_<id>_dims}"). Built and run unchanged on adf94b0 (it uses no API M2.7 adds).
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "m2_5/motion_harness.hpp"
#include "m2_7/capture27.hpp"

using m25::Scenario;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

// The blackboard a generated bundle's bt_params gives these trees (arrays joined by ';').
const Strings kGenerated = {
  {"scene_bin_dims", "0.3;0.2;0.1"},
  {"scene_bin_pos", "0.4;-0.2;0.05"},
  {"payload_part_dims", "0.04;0.04;0.03"},
};

std::vector<Scenario> scenarios()
{
  return {
    {"SCENE-01", "AddCollisionObject: every port literal, then the defaults (frame base_link, position 0;0;0), "
     "then the generated form; each tick is ONE apply_planning_scene with one world box ADD",
     nullptr, kGenerated, {},
     {"<AddCollisionObject id=\"table\" frame=\"world\" dims=\"0.8;0.6;0.05\" position=\"0.5;0;-0.025\"/>",
      "<AddCollisionObject id=\"box\" dims=\"0.1;0.1;0.1\"/>",
      "<AddCollisionObject id=\"bin\" frame=\"base_link\" dims=\"{scene_bin_dims}\" position=\"{scene_bin_pos}\"/>"}},
    {"SCENE-02", "AddCollisionObject data errors (two dims values, two position values, no dims): FAILURE "
     "before anything is sent",
     nullptr, {}, {},
     {"<AddCollisionObject id=\"bad\" dims=\"0.1;0.1\"/>",
      "<AddCollisionObject id=\"bad\" dims=\"0.1;0.1;0.1\" position=\"1;2\"/>",
      "<AddCollisionObject id=\"bad\"/>"}},
    {"SCENE-03", "RemoveCollisionObject: a planning_scene diff (world REMOVE), no service call; an absent id is "
     "sent as the empty id",
     nullptr, {}, {},
     {"<RemoveCollisionObject id=\"table\"/>", "<RemoveCollisionObject/>"}},
    {"SCENE-04", "AttachObject: every port literal, then the defaults (link tcp, position 0;0;0), then the "
     "generated form; each tick is ONE apply_planning_scene with one attached box ADD on the link",
     nullptr, kGenerated, {},
     {"<AttachObject id=\"part\" link=\"suction_link\" dims=\"0.05;0.05;0.02\" position=\"0;0;0.01\"/>",
      "<AttachObject id=\"part\" dims=\"0.05;0.05;0.02\"/>",
      "<AttachObject id=\"part\" link=\"tcp\" dims=\"{payload_part_dims}\" position=\"0;0;0.02\"/>"}},
    {"SCENE-05", "AttachObject data errors (one dims value, two position values): FAILURE before anything is sent",
     nullptr, {}, {},
     {"<AttachObject id=\"part\" dims=\"0.05\"/>",
      "<AttachObject id=\"part\" dims=\"0.05;0.05;0.02\" position=\"0;0\"/>"}},
    {"SCENE-06", "DetachObject: one apply_planning_scene (attached REMOVE on the link) THEN a planning_scene "
     "diff (world REMOVE); the link literal, then the default (tcp)",
     nullptr, {}, {},
     {"<DetachObject id=\"part\" link=\"suction_link\"/>", "<DetachObject id=\"part\"/>"}},
    {"SCENE-07", "the generated pick-and-place order on ONE runtime, one node per tick: the fixture, the "
     "payload attached at the pick, detached at the place, the fixture removed",
     nullptr, kGenerated, {},
     {"<AddCollisionObject id=\"bin\" frame=\"base_link\" dims=\"{scene_bin_dims}\" position=\"{scene_bin_pos}\"/>",
      "<AttachObject id=\"part\" link=\"tcp\" dims=\"{payload_part_dims}\" position=\"0;0;0.02\"/>",
      "<DetachObject id=\"part\" link=\"tcp\"/>",
      "<RemoveCollisionObject id=\"bin\"/>"}},
  };
}

#ifdef TRAINIT_M27_REMAP
// One per process (the M2.5 namespace world): a root and an /r3 fake set, both serving everything,
// the /r3 joint states, and the rig, whose node the process-wide remap moves to /r3.
struct RemapWorld
{
  m25::Log log;
  m25::FakeMoveGroup root{log, "/"};
  m25::FakeMoveGroup r3{log, "/r3"};
  m25::FakeJointStates js{"/r3"};
  m25::Rig rig{"/"};
  m25::Watchdog dog;
};

RemapWorld& remapWorld()
{
  static std::unique_ptr<RemapWorld> w;
  if (!w)
  {
    w = std::make_unique<RemapWorld>();
    m25::teardowns().push_back([] { w.reset(); });
  }
  return *w;
}
#endif
}  // namespace

TEST(M27SceneNodes, scene_nodes)
{
#ifdef TRAINIT_M27_REMAP
  RemapWorld& w = remapWorld();
  const std::string ns = w.rig.node()->get_namespace();
  m27::Family f("scene_nodes_remap");
  for (const Scenario& s : scenarios())
  {
    w.r3.resetScenario();
    f.add(m25::run(w.log, w.root, w.js, w.rig, w.dog, ns, s));
  }
#else
  m25::World& w = m25::world();
  m27::Family f("scene_nodes");
  for (const Scenario& s : scenarios()) f.add(m25::run(w, s));
#endif
  f.check();
}

int main(int argc, char** argv)
{
#ifdef TRAINIT_M27_REMAP
  return m25::runAll(argc, argv, TRAINIT_M27_DOMAIN, {"--ros-args", "-r", "__ns:=/r3"});
#else
  return m25::runAll(argc, argv, TRAINIT_M27_DOMAIN);
#endif
}
