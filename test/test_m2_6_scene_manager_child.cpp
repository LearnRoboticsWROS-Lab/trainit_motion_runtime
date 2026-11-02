// vNext M2.6 (tests BEFORE, ADR-0022 families F3/F5, test family E8): eef_scene_manager_child --
// the REAL scene_manager_node, started as a child process (a params file, a `-r __ns:=` remap for a
// namespaced cell), driven by the grasp Bool the runner publishes. Frozen on production 1dda503
// before the canonical EEF contract touches a production line (test/data/m2_6/README.md).
//
// The parent serves the planning-scene services (FakeSceneServices) and a static base_link -> tcp
// transform, publishes the grasp Bool the way the runner does (QoS(1).transient_local()), calls
// ~/reset_scene, and records the reset pulse. Captured: every GetPlanningScene and every
// ApplyPlanningScene request (the whole message, precision 17, stamps removed) in the order the
// scene manager sent them, step by step. What is pinned: the scene follows the COMMAND edge
// (no state is read), it is edge-triggered on its own last value, a latched close that arrives
// before the scene is loaded is consumed and lost, a refused attach diff leaves nothing to release,
// and the default topics (/isaac_gripper_cmd, /isaac_scene_reset) are absolute: two managers in two
// namespaces both follow one robot's grasp Bool. Harness choices in every params file: boxes only,
// force_republish_hz 0.0 (no periodic traffic), scene_update_wait_ms 100.
#include <sys/stat.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "m2_6/child_process.hpp"
#include "m2_6/eef_fakes.hpp"

using m26::BoolRecorder;

namespace
{
std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/scene_manager_child"; }

struct Manager
{
  std::string ns;          // "/" | "/r1" | "/r3"
};

// One step of a scenario: publish a grasp Bool, call reset, or nothing; then wait for `applies`
// apply requests in total (per namespace) and for quiet.
struct Step
{
  enum class Kind { START, BOOL, RESET } kind;
  bool value{false};
  std::string ns{};        // RESET: whose ~/reset_scene
  std::map<std::string, int> applies{};   // the total apply requests expected per namespace after it
};

struct SmScenario
{
  std::string id;
  std::string pins;
  std::vector<Manager> managers;
  std::string mode;                   // grasp_attach_mode
  bool refuse_attach{false};          // EMULATE move_group refusing an attach_box diff
  bool default_topics{false};         // leave gripper_cmd_topic / scene_reset_topic at their defaults
  bool latched_before_start{false};   // publish (latched) true BEFORE the managers start
  std::vector<Step> steps;
};

std::string gripperTopic(const SmScenario& s) { return s.default_topics ? "/isaac_gripper_cmd" : "/m26_" + m26::tagOf(s.id) + "/grasp"; }
std::string resetTopic(const SmScenario& s) { return s.default_topics ? "/isaac_scene_reset" : "/m26_" + m26::tagOf(s.id) + "/scene_reset"; }

std::string paramsYaml(const SmScenario& s)
{
  std::ostringstream y;
  y << "# vNext M2.6 " << s.id << ": scene_manager_node params, written by test_m2_6_scene_manager_child\n"
    << "/**:\n  ros__parameters:\n"
    << "    frame_id: base_link\n"
    << "    force_republish_hz: 0.0\n"
    << "    scene_update_wait_ms: 100\n";
  if (!s.default_topics)
    y << "    gripper_cmd_topic: \"" << gripperTopic(s) << "\"\n"
      << "    scene_reset_topic: \"" << resetTopic(s) << "\"\n";
  y << "    attach_link: tcp\n"
    << "    attached_collision_check: false\n"
    << "    grasp_attach_mode: " << s.mode << "\n"
    << "    touch_links: [tcp]\n"
    << "    attach_object_ids: [part]\n"
    << "    object_ids: [table, part]\n"
    << "    objects:\n"
    << "      table:\n"
    << "        type: box\n"
    << "        size: [0.6, 0.6, 0.02]\n"
    << "        position: [0.4, 0.0, -0.01]\n"
    << "        orientation: [0.0, 0.0, 0.0, 1.0]\n"
    << "        dynamic: false\n"
    << "      part:\n"
    << "        type: box\n"
    << "        size: [0.04, 0.04, 0.04]\n"
    << "        position: [0.4, 0.0, 0.02]\n"
    << "        orientation: [0.0, 0.0, 0.0, 1.0]\n"
    << "        dynamic: true\n"
    << "        grasp_box: [0.04, 0.04, 0.04]\n"
    << "        grasp_box_center: [0.0, 0.0, 0.0]\n";
  return y.str();
}

std::string managerName(const Manager& m) { return m26::fqn(m.ns, "scene_manager_node"); }

std::string run(const SmScenario& s)
{
  ::mkdir(actualDir().c_str(), 0755);
  auto probe = std::make_shared<rclcpp::Node>("m26_e8_probe", "/", rclcpp::NodeOptions().use_global_arguments(false));
  // spun: the reset_scene client's answer arrives through it (stopped before the probe dies)
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> probe_spin;
  probe_spin.executor().add_node(probe);
  probe_spin.start();
  // the tool frame the scene manager reads at grasp and release: tcp 0.3 m above the part, pointing down
  tf2_ros::StaticTransformBroadcaster tf(probe);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "base_link";
  t.child_frame_id = "tcp";
  t.transform.translation.x = 0.4;
  t.transform.translation.z = 0.3;
  t.transform.rotation.x = 1.0;
  t.transform.rotation.w = 0.0;
  t.header.stamp = probe->now();
  tf.sendTransform(t);

  std::map<std::string, std::unique_ptr<m26::Log>> logs;
  std::map<std::string, std::unique_ptr<m26::FakeSceneServices>> scenes;
  for (const auto& m : s.managers)
  {
    logs[m.ns] = std::make_unique<m26::Log>();
    scenes[m.ns] = std::make_unique<m26::FakeSceneServices>(*logs[m.ns], m.ns, s.refuse_attach);
  }
  // the grasp Bool, published the way the runner publishes it; in a namespaced scenario from /r1
  const std::string grasp_ns = s.managers.size() > 1 ? "/r1" : "/";
  m26::BoolProvider grasp("m26_grasp_cmd", grasp_ns, gripperTopic(s), true);
  BoolRecorder reset_rec("m26_rec_" + m26::tagOf(s.id) + "_reset", resetTopic(s), BoolRecorder::Qos::VOLATILE);
  const std::string params = actualDir() + "/" + m26::tagOf(s.id) + ".yaml";
  m26::writeFile(params, paramsYaml(s));

  std::ostringstream os;
  os << "=== scenario " << s.id << "\n" << "pins: " << s.pins << "\n"
     << "params (every manager):\n" << m26::indentBlock(paramsYaml(s).substr(paramsYaml(s).find('\n') + 1), 2)
     << "tf: base_link -> tcp static, translation (0.4, 0, 0.3), rotation (x=1, y=0, z=0, w=0)\n"
     << "grasp Bool: " << grasp.name() << " (" << grasp.qosText() << ") published by " << m26::fqn(grasp_ns, "m26_grasp_cmd") << "\n"
     << "reset recorder: " << resetTopic(s) << " (" << reset_rec.qosText() << ")\n";
  for (const auto& m : s.managers)
    os << "manager " << managerName(m) << ": scene_manager_node --ros-args --params-file <" << s.id << " params>"
       << (m.ns == "/" ? std::string() : " -r __ns:=" + m.ns) << "; scene services served at " << m.ns
       << (s.refuse_attach ? " (EMULATING a move_group that refuses an attach_box diff)" : "") << "\n";

  m26::watchdog().arm(s.id, std::chrono::seconds(150));
  m26::Semantic sem;
  std::vector<std::string> published;
  if (s.latched_before_start)
  {
    grasp.publish(true);
    published.push_back("true (latched, before the managers started)");
    os << "before the managers start: the harness publishes true (latched)\n";
  }
  std::vector<std::unique_ptr<m26::ChildProcess>> kids;
  std::map<std::string, std::size_t> seen;     // records already printed, per namespace
  std::vector<std::string> release;
  for (std::size_t i = 0; i < s.steps.size(); ++i)
  {
    const Step& st = s.steps[i];
    std::string what;
    switch (st.kind)
    {
      case Step::Kind::START:
        for (const auto& m : s.managers)
        {
          std::vector<std::string> args{"--ros-args", "--params-file", params};
          if (m.ns != "/") { args.push_back("-r"); args.push_back("__ns:=" + m.ns); }
          kids.push_back(std::make_unique<m26::ChildProcess>(
            TRAINIT_SCENE_MANAGER_EXE, args, m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()),
            actualDir() + "/" + m26::tagOf(s.id) + m26::tagOf(m.ns == "/" ? "_root" : m.ns) + ".log"));
        }
        what = "start the manager(s); wait for the initial scene load";
        break;
      case Step::Kind::BOOL:
        // the grasp publisher is matched by every manager before a value is sent (the runner sleeps 3 s)
        m26::waitUntil([&] { return grasp.matched() >= s.managers.size(); }, m26::milliseconds(5000));
        grasp.publish(st.value);
        published.push_back(m26::boolText(st.value));
        what = "the harness publishes " + m26::boolText(st.value) + " on " + gripperTopic(s);
        break;
      case Step::Kind::RESET:
      {
        auto client = probe->create_client<std_srvs::srv::Trigger>(m26::fqn(st.ns, "scene_manager_node/reset_scene"));
        std::string answer = "no service";
        if (client->wait_for_service(std::chrono::seconds(5)))
        {
          auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
          if (fut.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
          {
            const auto resp = fut.get();
            answer = "success=" + m26::boolText(resp->success) + " message='" + resp->message + "'";
          }
          else
            answer = "no answer within 20 s";
        }
        what = "the harness calls " + std::string(client->get_service_name()) + " -> " + answer;
        sem.ack.push_back("reset_scene: " + answer);
        reset_rec.settle(2, m26::milliseconds(3000));
        break;
      }
    }
    for (const auto& m : s.managers)
    {
      const int n = st.applies.count(m.ns) ? st.applies.at(m.ns) : scenes[m.ns]->applies();
      scenes[m.ns]->settle(n, m26::milliseconds(20000));
    }
    os << "--- step " << (i + 1) << ": " << what << "\n";
    for (const auto& m : s.managers)
    {
      const auto recs = scenes[m.ns]->records();
      std::vector<std::string> fresh(recs.begin() + static_cast<long>(seen[m.ns]), recs.end());
      seen[m.ns] = recs.size();
      os << "requests from " << managerName(m) << ":" << (fresh.empty() ? std::string(" (none)\n") : "\n");
      for (const auto& r : fresh) os << "  " << r << "\n";
      const std::string tag = "step " + std::to_string(i + 1) + " " + managerName(m) + ": ";
      sem.state.push_back(tag + (fresh.empty() ? std::string("no request") : std::to_string(fresh.size()) + " request(s)"));
      if (st.kind == Step::Kind::BOOL && !st.value)
        release.push_back(tag + (fresh.empty() ? std::string("no request") : m26::join(fresh, " + ")));
    }
  }
  // the Bool subscriptions of each manager (the only input it attaches on), while they still run
  for (const auto& m : s.managers)
  {
    const auto subs = m26::nodeTopics(probe, "scene_manager_node", m.ns, "std_msgs/msg/Bool", true);
    const bool only_command = subs == std::vector<std::string>{gripperTopic(s)};
    sem.grasp.push_back("Bool subscriptions of " + managerName(m) + ": " + m26::listOrNone(subs) +
                        (only_command ? " -- its only Bool input is the command topic the harness drives"
                                      : " -- it has a Bool input other than the command topic"));
  }
  for (auto& k : kids) k->stop();
  probe_spin.stop();
  probe_spin.executor().remove_node(probe);
  m26::watchdog().disarm();

  os << "whole requests, per manager:\n";
  for (const auto& m : s.managers)
    os << "events " << managerName(m) << ":\n" << (logs[m.ns]->events().empty() ? std::string("(none)\n") : logs[m.ns]->events());
  os << "reset recorder received: " << reset_rec.byPublisher() << "\n";
  for (std::size_t i = 0; i < kids.size(); ++i)
    os << "manager " << managerName(s.managers[i]) << " after SIGINT: " << kids[i]->exitText() << "\n";

  sem.command.push_back("grasp Bool on " + gripperTopic(s) + ": " + (published.empty() ? std::string("none") : m26::join(published)));
  if (sem.ack.empty()) sem.ack.push_back("none (a topic has no ack)");
  sem.bt.push_back("none (no tree: the scene manager is driven by the topic)");
  sem.release = release;
  if (reset_rec.count() > 0) sem.state.push_back("reset pulse on " + resetTopic(s) + ": " + reset_rec.byPublisher());
  for (const auto& m : s.managers)
  {
    int ok = 0, refused = 0;
    for (const auto& r : scenes[m.ns]->records())
    {
      if (r.find("-> success=true") != std::string::npos) ++ok;
      if (r.find("-> success=false") != std::string::npos) ++refused;
    }
    sem.code.push_back(managerName(m) + ": ApplyPlanningScene success=true x" + std::to_string(ok) +
                       ", success=false x" + std::to_string(refused));
  }
  os << sem.line() << "\n";
  return os.str();
}

using K = Step::Kind;
}  // namespace

TEST(M26SceneManagerChild, eef_scene_manager_child)
{
  const std::vector<SmScenario> scenarios{
    {"SM-01", "grasp_attach_mode remove: true REMOVES the part, a second true is not an edge, false re-ADDS it at the "
              "tool pose and re-allows it, reset puts it back and pulses the reset topic",
     {{"/"}}, "remove", false, false, false,
     {{K::START, false, "", {{"/", 2}}}, {K::BOOL, true, "", {{"/", 3}}}, {K::BOOL, true, "", {{"/", 3}}},
      {K::BOOL, false, "", {{"/", 5}}}, {K::RESET, false, "/", {{"/", 7}}}}},
    {"SM-02", "grasp_attach_mode attach_box with move_group REFUSING the attach diff (EMULATED by the fake): nothing is "
              "committed, so the following false finds nothing to release and sends nothing",
     {{"/"}}, "attach_box", true, false, false,
     {{K::START, false, "", {{"/", 2}}}, {K::BOOL, true, "", {{"/", 3}}}, {K::BOOL, false, "", {{"/", 3}}}}},
    {"SM-03", "grasp_attach_mode attach_box accepted: true ATTACHES the part's box with NO world REMOVE (move_group "
              "moves an attached id off the world itself; the fake refuses the REMOVE + ATTACH diff as move_group does), "
              "then the ACM; false DETACHES and re-ADDS it, then the ACM",
     {{"/"}}, "attach_box", false, false, false,
     {{K::START, false, "", {{"/", 2}}}, {K::BOOL, true, "", {{"/", 4}}}, {K::BOOL, false, "", {{"/", 6}}}}},
    {"SM-04", "a LATCHED true published before the manager starts arrives before the scene is loaded: it is consumed "
              "(no grasp); the next false is an edge with nothing to release; only the following true grasps",
     {{"/"}}, "remove", false, false, true,
     {{K::START, false, "", {{"/", 2}}}, {K::BOOL, false, "", {{"/", 2}}}, {K::BOOL, true, "", {{"/", 3}}}}},
    {"SM-05", "two managers in /r1 and /r3 with the DEFAULT topics (absolute): one true from /r1 makes BOTH scenes "
              "grasp; /r1's reset pulses the one global reset topic",
     {{"/r1"}, {"/r3"}}, "remove", false, true, false,
     {{K::START, false, "", {{"/r1", 2}, {"/r3", 2}}}, {K::BOOL, true, "", {{"/r1", 3}, {"/r3", 3}}},
      {K::RESET, false, "/r1", {{"/r1", 5}, {"/r3", 3}}}}},
  };
  m26::Family f("eef_scene_manager_child");
  for (const auto& s : scenarios) f.add(run(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
