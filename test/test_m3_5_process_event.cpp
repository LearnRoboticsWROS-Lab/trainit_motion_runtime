// test_m3_5_process_event (vNext M3.5, ADR-0024, M3_DESIGN §9): the generic ProcessEvent node and the
// ROS process controller's publisher lifecycle, ticked for real on a private domain.
//
// What it pins:
//   - ProcessEvent maps its four kinds onto the ONE controller abstraction (process_on -> ENABLE,
//     process_off -> DISABLE, set_value -> SET_ANALOG_VALUE, external -> TRIGGER) and refuses, with
//     nothing dispatched: an unknown kind (capture and dwell are other nodes), a channel that is not a
//     LOGICAL id (a ROS topic path, a PLC address, a controller symbol), a value where none belongs, a
//     missing or non-finite value, no controller; a controller that does not dispatch is a FAILURE;
//   - EnableProcess / DisableProcess, the legacy aliases, are unchanged (any channel, ENABLE 1.0 /
//     DISABLE 0.0);
//   - RosProcessController: ONE publisher per channel and signal, reused by every event (2.3.0 made
//     one per event and destroyed it), separate per channel, ready before the first event when
//     prepared, and a late transient_local subscriber receives the latched last command; the topics
//     are "<channel>/process_enabled | process_value" in the node's namespace, as before.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace
{
using trainit::ProcessEventType;

const char* typeName(ProcessEventType t)
{
  switch (t)
  {
    case ProcessEventType::ENABLE_PROCESS: return "ENABLE";
    case ProcessEventType::DISABLE_PROCESS: return "DISABLE";
    case ProcessEventType::TRIGGER: return "TRIGGER";
    case ProcessEventType::SET_ANALOG_VALUE: return "SET_ANALOG";
  }
  return "?";
}

// Records what reaches the controller, and answers `dispatch`.
class RecordingProcess : public trainit::ProcessController
{
public:
  bool executeEvent(const trainit::ProcessEvent& e) override
  {
    std::lock_guard<std::mutex> g(m);
    events.push_back(std::string(typeName(e.type)) + " " + e.channel + " " + std::to_string(e.value));
    return dispatch;
  }
  void prepare(const std::string& channel, bool analog) override
  {
    std::lock_guard<std::mutex> g(m);
    prepared.push_back(channel + (analog ? " value" : " enabled"));
  }
  std::mutex m;
  std::vector<std::string> events, prepared;
  bool dispatch{true};
};

class ProcessEventTick : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("m35_process", "/");
    r3_ = std::make_shared<rclcpp::Node>("m35_process_r3", "/r3");
    probe_ = std::make_shared<rclcpp::Node>("m35_probe", "/");
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    exec_->add_node(node_);
    exec_->add_node(r3_);
    exec_->add_node(probe_);
    spinner_ = std::thread([] { exec_->spin(); });
  }
  static void TearDownTestSuite()
  {
    exec_->cancel();
    spinner_.join();
    exec_.reset();
    probe_.reset();
    r3_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  // ticks one node; `process` may be null
  static std::string tick(const std::string& node_xml, trainit::ProcessController* process)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    trainit::BtContext ctx;
    ctx.node = node_;
    ctx.process = process;
    auto bb = BT::Blackboard::create();
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         node_xml + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    return BT::toStr(tree.tickOnce());
  }

  // waits until `pred` holds (bounded)
  template <typename Pred>
  static bool eventually(Pred pred, int ms = 3000)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end)
    {
      if (pred()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return pred();
  }

  static rclcpp::Node::SharedPtr node_, r3_, probe_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spinner_;
};
rclcpp::Node::SharedPtr ProcessEventTick::node_, ProcessEventTick::r3_, ProcessEventTick::probe_;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ProcessEventTick::exec_;
std::thread ProcessEventTick::spinner_;
}  // namespace

// --- A: the kinds onto the controller ------------------------------------------------------------

TEST_F(ProcessEventTick, A_the_four_kinds_reach_the_one_controller)
{
  RecordingProcess rec;
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"dispenser\"/>", &rec), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"set_value\" channel=\"dispenser\" value=\"0.35\"/>", &rec), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_off\" channel=\"dispenser\"/>", &rec), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"external\" channel=\"washer_start\"/>", &rec), "SUCCESS");
  EXPECT_EQ(rec.events, (std::vector<std::string>{"ENABLE dispenser 1.000000", "SET_ANALOG dispenser 0.350000",
                                                  "DISABLE dispenser 0.000000", "TRIGGER washer_start 0.000000"}));
}

TEST_F(ProcessEventTick, A_the_mock_controller_dispatches_every_kind)
{
  trainit::MockProcessController mock(node_->get_logger());
  for (const char* xml : {"<ProcessEvent kind=\"process_on\" channel=\"inspection_light\"/>",
                          "<ProcessEvent kind=\"set_value\" channel=\"inspection_light\" value=\"-2\"/>",
                          "<ProcessEvent kind=\"process_off\" channel=\"inspection_light\"/>",
                          "<ProcessEvent kind=\"external\" channel=\"camera_trigger\"/>"})
    EXPECT_EQ(tick(xml, &mock), "SUCCESS") << xml;
}

// --- E: refusals, loud, nothing dispatched ---------------------------------------------------------

TEST_F(ProcessEventTick, E_what_is_not_a_process_event_is_refused_and_nothing_is_dispatched)
{
  RecordingProcess rec;
  for (const char* xml : {
         "<ProcessEvent kind=\"glue_on\" channel=\"dispenser\"/>",            // no application kinds
         "<ProcessEvent kind=\"capture\" channel=\"dispenser\"/>",            // DetectObject trigger
         "<ProcessEvent kind=\"dwell\" channel=\"dispenser\"/>",              // Sleep
         "<ProcessEvent kind=\"\" channel=\"dispenser\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"/trainit/process/dispenser\"/>",   // a ROS topic path
         "<ProcessEvent kind=\"process_on\" channel=\"dispenser/process_enabled\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"/plc/output/12\"/>",               // a PLC address
         "<ProcessEvent kind=\"process_on\" channel=\"MAIN.fbWasher.Cmd_Start\"/>",      // a TwinCAT symbol
         "<ProcessEvent kind=\"process_on\" channel=\"192.168.1.4/ADS/851\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"%QX0.3\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"1st_dispenser\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"washer-start\"/>",
         "<ProcessEvent kind=\"process_on\" channel=\"dispenser\" value=\"1\"/>",       // no value on on/off
         "<ProcessEvent kind=\"external\" channel=\"washer_start\" value=\"1\"/>",
         "<ProcessEvent kind=\"set_value\" channel=\"dispenser\"/>",                    // a value is needed
         "<ProcessEvent kind=\"set_value\" channel=\"dispenser\" value=\"nan\"/>",
         "<ProcessEvent kind=\"set_value\" channel=\"dispenser\" value=\"inf\"/>",
         "<ProcessEvent kind=\"set_value\" channel=\"dispenser\" value=\"0.5 bar\"/>"})
    EXPECT_EQ(tick(xml, &rec), "FAILURE") << xml;
  EXPECT_TRUE(rec.events.empty());
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"dispenser\"/>", nullptr), "FAILURE");
}

TEST_F(ProcessEventTick, E_a_controller_that_does_not_dispatch_fails_the_event)
{
  RecordingProcess rec;
  rec.dispatch = false;
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"dispenser\"/>", &rec), "FAILURE");
  EXPECT_EQ(rec.events.size(), 1u);
}

TEST_F(ProcessEventTick, E_a_failed_event_stops_the_sequence_after_it)
{
  RecordingProcess rec;
  EXPECT_EQ(tick("<Sequence><ProcessEvent kind=\"process_on\" channel=\"a\"/>"
                 "<ProcessEvent kind=\"process_on\" channel=\"/b\"/>"
                 "<ProcessEvent kind=\"process_off\" channel=\"a\"/></Sequence>", &rec), "FAILURE");
  EXPECT_EQ(rec.events, (std::vector<std::string>{"ENABLE a 1.000000"}));
}

// --- M: the legacy aliases, unchanged ---------------------------------------------------------------

TEST_F(ProcessEventTick, M_enable_and_disable_process_are_unchanged)
{
  RecordingProcess rec;
  // they accept what they accepted (a channel is not checked by the legacy nodes)
  EXPECT_EQ(tick("<EnableProcess channel=\"bin_agitator\"/>", &rec), "SUCCESS");
  EXPECT_EQ(tick("<DisableProcess channel=\"bin_agitator\"/>", &rec), "SUCCESS");
  EXPECT_EQ(tick("<EnableProcess channel=\"/cell/agitator\"/>", &rec), "SUCCESS");
  EXPECT_EQ(rec.events, (std::vector<std::string>{"ENABLE bin_agitator 1.000000", "DISABLE bin_agitator 0.000000",
                                                  "ENABLE /cell/agitator 1.000000"}));
  EXPECT_EQ(tick("<EnableProcess channel=\"x\"/>", nullptr), "FAILURE");
}

// --- B, C, D: the ROS controller --------------------------------------------------------------------

TEST_F(ProcessEventTick, B_C_one_retained_publisher_per_channel_and_signal)
{
  trainit::RosProcessController ros(node_);
  const std::string on = "/m35_dispenser/process_enabled", value = "/m35_dispenser/process_value";
  EXPECT_EQ(node_->count_publishers(on), 0u);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"m35_dispenser\"/>", &ros), "SUCCESS");
    EXPECT_EQ(tick("<ProcessEvent kind=\"process_off\" channel=\"m35_dispenser\"/>", &ros), "SUCCESS");
  }
  EXPECT_EQ(tick("<ProcessEvent kind=\"set_value\" channel=\"m35_dispenser\" value=\"0.5\"/>", &ros), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"set_value\" channel=\"m35_dispenser\" value=\"0.7\"/>", &ros), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"m35_lamp\"/>", &ros), "SUCCESS");
  // ten on/off events: ONE publisher on the channel's signal, still there; one for its value; the
  // other channel has its own
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers(on) == 1u; })) << probe_->count_publishers(on);
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers(value) == 1u; }));
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers("/m35_lamp/process_enabled") == 1u; }));
  EXPECT_EQ(probe_->count_publishers("/m35_lamp/process_value"), 0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(probe_->count_publishers(on), 1u);
}

TEST_F(ProcessEventTick, B_D_a_late_subscriber_receives_the_latched_last_command)
{
  trainit::RosProcessController ros(node_);
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"m35_late\"/>", &ros), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_off\" channel=\"m35_late\"/>", &ros), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"external\" channel=\"m35_late\"/>", &ros), "SUCCESS");
  EXPECT_EQ(tick("<ProcessEvent kind=\"set_value\" channel=\"m35_late\" value=\"0.25\"/>", &ros), "SUCCESS");
  // the driver subscribes AFTER every event (transient_local, depth 1: the contract's latched QoS)
  std::mutex m;
  std::vector<bool> enabled;
  std::vector<double> values;
  const auto latched = rclcpp::QoS(1).transient_local();
  auto s1 = probe_->create_subscription<std_msgs::msg::Bool>(
    "/m35_late/process_enabled", latched, [&](std_msgs::msg::Bool::SharedPtr b) {
      std::lock_guard<std::mutex> g(m); enabled.push_back(b->data); });
  auto s2 = probe_->create_subscription<std_msgs::msg::Float64>(
    "/m35_late/process_value", latched, [&](std_msgs::msg::Float64::SharedPtr v) {
      std::lock_guard<std::mutex> g(m); values.push_back(v->data); });
  EXPECT_TRUE(eventually([&] { std::lock_guard<std::mutex> g(m); return !enabled.empty() && !values.empty(); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::lock_guard<std::mutex> g(m);
  EXPECT_EQ(enabled, (std::vector<bool>{true}));    // the LAST on/off command only: external = true
  EXPECT_EQ(values, (std::vector<double>{0.25}));
}

TEST_F(ProcessEventTick, C_a_prepared_channel_has_its_publisher_before_the_first_event)
{
  trainit::RosProcessController ros(node_);
  ros.prepare("m35_ready", false);
  ros.prepare("m35_ready_v", true);
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers("/m35_ready/process_enabled") == 1u; }));
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers("/m35_ready_v/process_value") == 1u; }));
  EXPECT_EQ(probe_->count_publishers("/m35_ready/process_value"), 0u);
  // an early VOLATILE subscriber is matched before the event, so it receives it
  std::atomic<int> got{0};
  auto sub = probe_->create_subscription<std_msgs::msg::Bool>(
    "/m35_ready/process_enabled", rclcpp::QoS(1), [&](std_msgs::msg::Bool::SharedPtr) { ++got; });
  EXPECT_TRUE(eventually([&] { return node_->count_subscribers("/m35_ready/process_enabled") >= 1u; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"m35_ready\"/>", &ros), "SUCCESS");
  EXPECT_TRUE(eventually([&] { return got.load() == 1; }));
  // and the event reuses the prepared publisher
  EXPECT_EQ(probe_->count_publishers("/m35_ready/process_enabled"), 1u);
}

TEST_F(ProcessEventTick, B_a_logical_channel_resolves_in_the_controllers_namespace)
{
  trainit::RosProcessController ros(r3_);
  EXPECT_EQ(tick("<ProcessEvent kind=\"process_on\" channel=\"m35_ns\"/>", &ros), "SUCCESS");
  EXPECT_TRUE(eventually([&] { return probe_->count_publishers("/r3/m35_ns/process_enabled") == 1u; }));
  EXPECT_EQ(probe_->count_publishers("/m35_ns/process_enabled"), 0u);
}

TEST(ProcessChannel, a_logical_id_is_a_name_never_a_transport_address)
{
  for (const char* good : {"dispenser", "washer_start", "camera_trigger", "inspection_light", "A1", "x"})
    EXPECT_TRUE(trainit::isLogicalChannel(good)) << good;
  for (const char* bad : {"", "/dispenser", "dispenser/", "a/b", "MAIN.fbWasher.Cmd_Start", "%IX0.3", "1abc",
                          "_x", "a-b", "a b", "192.168.1.4", "plc:out12", "~/x"})
    EXPECT_FALSE(trainit::isLogicalChannel(bad)) << bad;
}
