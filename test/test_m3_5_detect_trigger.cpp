// test_m3_5_detect_trigger (vNext M3.5, M3_DESIGN §9): DetectObject trigger="true" -- a CAPTURE --
// ticked for real against a fake on-demand detector that speaks the perception contract
// (std_srvs/Trigger /perception/<d>/detect + vision_msgs/Detection3DArray /perception/<d>/detections,
// RELIABLE depth 1), on a private domain.
//
// What it pins:
//   H  the trigger is called, once per capture when the first answer is fresh;
//   I  a detection that exists BEFORE the capture is never accepted -- neither one that arrived before
//      the trigger nor one that arrives after it but was TAKEN before it (its stamp): the detector is
//      triggered again;
//   J  a fresh result lands on the blackboard: the same keys as before plus <out_key>.stamp;
//   K  no service -> TRIGGER_UNAVAILABLE; only stale or no results -> DETECTION_TIMEOUT; both FAILURE;
//   L  three captures in a task's subtree land on captures.0, captures.1, captures.2 of the ROOT
//      blackboard, in order, each the result of its own trigger;
//   and trigger="false" (every bundle before M3.5) still takes the first arrival, whatever its stamp.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"

namespace
{
using Array = vision_msgs::msg::Detection3DArray;

enum class Mode { FRESH, STALE_THEN_FRESH, STALE_ONLY, SILENT, FUTURE, HANG, REFUSE };

// The fake detector: on each trigger it publishes according to `mode`. A FRESH result's x is
// 0.1 * the trigger count, so a capture's value says WHICH trigger produced it; a stale one is 9.9.
class FakeDetector
{
public:
  explicit FakeDetector(const std::string& name)
  {
    node = std::make_shared<rclcpp::Node>("m35_fake_detector_" + name, "/");
    pub = node->create_publisher<Array>("/perception/" + name + "/detections",
                                        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    srv = node->create_service<std_srvs::srv::Trigger>(
      "/perception/" + name + "/detect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const int k = ++triggers;
        const auto now = node->now();
        switch (mode.load())
        {
          case Mode::FRESH: publish(now, 0.1 * k); break;
          case Mode::STALE_ONLY: publish(now - rclcpp::Duration::from_seconds(5.0), 9.9); break;
          case Mode::SILENT: break;
          case Mode::FUTURE: publish(now + rclcpp::Duration::from_seconds(3600.0), 5.5); break;   // another clock
          case Mode::HANG: std::this_thread::sleep_for(std::chrono::milliseconds(1500)); break;
          case Mode::REFUSE: publish(now, 4.4); break;   // a result, but the answer says it did not sample
          case Mode::STALE_THEN_FRESH:
            publish(now - rclcpp::Duration::from_seconds(5.0), 9.9);   // an image taken before the trigger
            later(100, k);
            break;
        }
        res->success = mode.load() != Mode::SILENT && mode.load() != Mode::REFUSE;
        res->message = "trigger " + std::to_string(k);
      });
  }
  ~FakeDetector()
  {
    std::lock_guard<std::mutex> g(m);
    for (auto& t : threads) if (t.joinable()) t.join();
  }
  void publish(const rclcpp::Time& stamp, double x)
  {
    Array a;
    a.header.stamp = stamp;
    a.header.frame_id = "cam_optical";
    vision_msgs::msg::Detection3D d;
    d.header = a.header;
    vision_msgs::msg::ObjectHypothesisWithPose h;
    h.hypothesis.class_id = "part";
    h.hypothesis.score = 0.9;
    h.pose.pose.position.x = x;
    h.pose.pose.orientation.w = 1.0;
    d.results.push_back(h);
    a.detections.push_back(d);
    pub->publish(a);
  }
  void later(int ms, int k)
  {
    std::lock_guard<std::mutex> g(m);
    threads.emplace_back([this, ms, k] {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      publish(node->now(), 0.1 * k);
    });
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Array>::SharedPtr pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv;
  std::atomic<Mode> mode{Mode::FRESH};
  std::atomic<int> triggers{0};
  std::mutex m;
  std::vector<std::thread> threads;
};

std::string bbStr(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  try { return bb->get<std::string>(key); } catch (const std::exception&) { return "<unset>"; }
}

double xOf(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  const std::string p = bbStr(bb, key + ".position");
  return p == "<unset>" ? -1.0 : std::stod(p.substr(0, p.find(';')));
}

class DetectTrigger : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("m35_detect_trigger", "/");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = "cam_optical";
    T.transform.translation.z = 1.0;
    T.transform.rotation.w = 1.0;
    tf_->setTransform(T, "test", true);
    tf_->setUsingDedicatedThread(true);
    ctx_.node = node_;
    ctx_.tf = tf_;
    cam_ = std::make_unique<FakeDetector>("m35cam");
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
    exec_->add_node(node_);
    exec_->add_node(cam_->node);
    spinner_ = std::thread([] { exec_->spin(); });
  }
  static void TearDownTestSuite()
  {
    exec_->cancel();
    spinner_.join();
    exec_.reset();
    cam_.reset();
    ctx_ = trainit::BtContext{};
    node_.reset();
    tf_.reset();
    rclcpp::shutdown();
  }

  static std::string tickXml(const std::string& trees, const std::string& main, const BT::Blackboard::Ptr& bb)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"" + main + "\">" + trees +
                                         "</root>");
    auto tree = factory.createTree(main, bb);
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    while (s == BT::NodeStatus::RUNNING) s = tree.tickOnce();
    return BT::toStr(s);
  }
  static std::string tick(const std::string& node_xml, const BT::Blackboard::Ptr& bb)
  {
    return tickXml("<BehaviorTree ID=\"T\">" + node_xml + "</BehaviorTree>", "T", bb);
  }
  static std::string capture(const std::string& out, int timeout_ms = 2000)
  {
    return "<DetectObject detector=\"m35cam\" trigger=\"true\" timeout_ms=\"" + std::to_string(timeout_ms) +
           "\" out_key=\"" + out + "\" fault=\"{f}\"/>";
  }
  // a detection that EXISTS before the capture: it arrives, and it is the feed's latest message
  static void oldDetectionExists()
  {
    cam_->publish(node_->now() - rclcpp::Duration::from_seconds(1.0), 7.7);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static trainit::BtContext ctx_;
  static std::unique_ptr<FakeDetector> cam_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spinner_;
};
rclcpp::Node::SharedPtr DetectTrigger::node_;
std::shared_ptr<tf2_ros::Buffer> DetectTrigger::tf_;
trainit::BtContext DetectTrigger::ctx_;
std::unique_ptr<FakeDetector> DetectTrigger::cam_;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> DetectTrigger::exec_;
std::thread DetectTrigger::spinner_;
}  // namespace

// (the gtest order is the file order: the first test also creates DetectObject's per-process feed)

TEST_F(DetectTrigger, H_J_a_capture_triggers_once_and_writes_a_fresh_result)
{
  cam_->mode = Mode::FRESH;
  const int before = cam_->triggers;
  auto bb = BT::Blackboard::create();
  const int64_t start_ns = node_->now().nanoseconds();
  EXPECT_EQ(tick(capture("cap"), bb), "SUCCESS");
  EXPECT_EQ(cam_->triggers - before, 1);
  EXPECT_NEAR(xOf(bb, "cap"), 0.1 * cam_->triggers, 1e-9);
  // the same keys as an untriggered DetectObject (keep=1: flat), the result in base_link, plus .stamp
  EXPECT_EQ(bbStr(bb, "cap.frame"), "base_link");
  EXPECT_EQ(bbStr(bb, "cap.class_id"), "part");
  EXPECT_NE(bbStr(bb, "cap.orientation"), "<unset>");
  EXPECT_NE(bbStr(bb, "cap.score"), "<unset>");
  EXPECT_NE(bbStr(bb, "cap.size"), "<unset>");
  EXPECT_EQ(bbStr(bb, "cap.count"), "<unset>");
  const std::string stamp = bbStr(bb, "cap.stamp");
  ASSERT_NE(stamp, "<unset>");
  const auto dot = stamp.find('.');
  ASSERT_NE(dot, std::string::npos);
  EXPECT_EQ(stamp.size() - dot - 1, 9u);   // nanoseconds, zero-padded
  const int64_t stamp_ns = std::stoll(stamp.substr(0, dot)) * 1000000000LL + std::stoll(stamp.substr(dot + 1));
  EXPECT_GE(stamp_ns, start_ns);
  EXPECT_EQ(bbStr(bb, "f"), "<unset>");
}

TEST_F(DetectTrigger, I_an_old_detection_and_a_stale_answer_are_never_accepted)
{
  cam_->mode = Mode::STALE_THEN_FRESH;
  oldDetectionExists();                         // arrived before the capture: x 7.7
  const int before = cam_->triggers;
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap"), bb), "SUCCESS");
  // the trigger's first answer (x 9.9, taken 5 s before) arrived AFTER the trigger and was rejected by
  // its stamp; the fresh one 100 ms later is the capture
  EXPECT_EQ(cam_->triggers - before, 1);
  EXPECT_NEAR(xOf(bb, "cap"), 0.1 * cam_->triggers, 1e-9);
}

TEST_F(DetectTrigger, I_an_untriggered_detect_object_still_takes_the_first_arrival)
{
  // trigger="false" (every bundle before M3.5): freshness by arrival only, whatever the stamp
  cam_->mode = Mode::FRESH;
  auto bb = BT::Blackboard::create();
  std::thread late([] {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    cam_->publish(node_->now() - rclcpp::Duration::from_seconds(5.0), 9.9);
  });
  EXPECT_EQ(tick("<DetectObject detector=\"m35cam\" timeout_ms=\"2000\" out_key=\"legacy\"/>", bb), "SUCCESS");
  late.join();
  EXPECT_NEAR(xOf(bb, "legacy"), 9.9, 1e-9);
  EXPECT_EQ(bbStr(bb, "legacy.stamp"), "<unset>");
}

TEST_F(DetectTrigger, K_only_stale_results_time_out_after_retriggering)
{
  cam_->mode = Mode::STALE_ONLY;
  const int before = cam_->triggers;
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap", 1600), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
  EXPECT_GE(cam_->triggers - before, 2);       // re-triggered after each stale answer
  EXPECT_EQ(bbStr(bb, "cap.position"), "<unset>");
}

TEST_F(DetectTrigger, K_a_detector_that_answers_but_never_publishes_times_out)
{
  cam_->mode = Mode::SILENT;
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap", 1200), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
}

TEST_F(DetectTrigger, K_no_detect_service_is_trigger_unavailable)
{
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick("<DetectObject detector=\"m35_nobody\" trigger=\"true\" timeout_ms=\"800\" out_key=\"cap\" "
                 "fault=\"{f}\"/>", bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "TRIGGER_UNAVAILABLE");
}

TEST_F(DetectTrigger, K_a_trigger_needs_a_detector_and_a_boolean)
{
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick("<DetectObject topic=\"/perception/m35cam/detections\" trigger=\"true\" timeout_ms=\"500\"/>", bb),
            "FAILURE");
  EXPECT_EQ(tick("<DetectObject detector=\"m35cam\" trigger=\"yes\" timeout_ms=\"500\"/>", bb), "FAILURE");
}

TEST_F(DetectTrigger, K_a_result_on_another_clock_is_never_accepted)
{
  cam_->mode = Mode::FUTURE;          // stamped an hour AHEAD: a wall-clock detector and a sim-time robot
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap", 1200), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "CLOCK_MISMATCH");
  EXPECT_EQ(bbStr(bb, "cap.position"), "<unset>");
}

TEST_F(DetectTrigger, K_a_clock_that_is_not_running_refuses_the_capture)
{
  // a node on sim time with no /clock on this domain: its time is 0, and every stamp would pass
  auto sim = std::make_shared<rclcpp::Node>("m35_sim_clock", "/",
                                            rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  ASSERT_EQ(sim->now().nanoseconds(), 0);
  trainit::BtContext ctx = ctx_;
  ctx.node = sim;
  cam_->mode = Mode::FRESH;
  const int before = cam_->triggers;
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                       capture("cap", 1000) + "</BehaviorTree></root>");
  auto tree = factory.createTree("T", bb);
  EXPECT_EQ(BT::toStr(tree.tickOnce()), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "CLOCK_UNAVAILABLE");
  EXPECT_EQ(cam_->triggers, before);  // refused before the trigger
}

TEST_F(DetectTrigger, K_a_service_that_never_answers_is_trigger_unavailable)
{
  cam_->mode = Mode::HANG;            // the request goes out; the answer comes after the capture's time
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap", 700), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "TRIGGER_UNAVAILABLE");
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));   // the fake finishes its answer
}

TEST_F(DetectTrigger, K_nothing_is_accepted_after_an_answer_that_did_not_sample)
{
  cam_->mode = Mode::REFUSE;          // success=false, yet a fresh-stamped result is published
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(capture("cap", 1200), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
  EXPECT_EQ(bbStr(bb, "cap.position"), "<unset>");
}

TEST_F(DetectTrigger, L_three_captures_land_in_order_on_the_root_blackboard)
{
  cam_->mode = Mode::STALE_THEN_FRESH;
  oldDetectionExists();
  const int first = cam_->triggers + 1;
  auto bb = BT::Blackboard::create();
  // the generated shape: MainTree runs the task's subtree with _autoremap (vNext M3.3)
  const std::string trees =
    "<BehaviorTree ID=\"MainTree\"><SubTree ID=\"task_inspect\" _autoremap=\"true\"/></BehaviorTree>"
    "<BehaviorTree ID=\"task_inspect\"><Sequence>" + capture("captures.0") + capture("captures.1") +
    capture("captures.2") + "</Sequence></BehaviorTree>";
  EXPECT_EQ(tickXml(trees, "MainTree", bb), "SUCCESS");
  EXPECT_EQ(cam_->triggers - first + 1, 3);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(xOf(bb, "captures." + std::to_string(i)), 0.1 * (first + i), 1e-9) << i;
  const auto stamp = [&](int i) { return std::stod(bbStr(bb, "captures." + std::to_string(i) + ".stamp")); };
  EXPECT_LT(stamp(0), stamp(1));
  EXPECT_LT(stamp(1), stamp(2));
}
