// vNext M2.6 AFTER (ADR-0022), M7: static scans. No ROS graph; the SOURCE files are read.
//
//   - the new end-effector sources hold no function-static (cache, map or publisher) and no
//     absolute ROS name in a string literal (ADR-0022 D7: every name is relative to the runner);
//   - the legacy units gained none: bt_nodes.cpp keeps exactly its legacy statics and absolute
//     literals (pinned here from 1dda503/a0fbace), the runner and GripperController have none;
//   - the legacy tick bodies of Open/CloseGripper are textually unchanged, and GripperController
//     changed only by the EEF-D3 WARN;
//   - the new sources are classified in the M2.5 seam scan (MoveIt-free, scanned);
//   - the Groot node model lists exactly the ports the end-effector nodes provide;
//   - VerifyGrasp has exactly its D4 ports, and NO actuator port (owner section 24.11).
#include <algorithm>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include "trainit_motion_runtime/bt_nodes.hpp"

namespace
{
const std::vector<std::string> kNewSources = {
  "src/end_effector_contract.cpp", "src/actuator_outcome.cpp", "src/verify_grasp.cpp",
  "src/end_effector_registry.cpp", "include/trainit_motion_runtime/end_effector_contract.hpp",
  "include/trainit_motion_runtime/actuator_outcome.hpp", "include/trainit_motion_runtime/verify_grasp.hpp",
  "include/trainit_motion_runtime/end_effector_registry.hpp",
  "include/trainit_motion_runtime/end_effector_reason.hpp",
  // vNext M2.7 (owner Q8.2): the motion-failure classification held to the same rules
  "src/motion_failure.cpp", "include/trainit_motion_runtime/motion_failure.hpp"};

std::string read(const std::string& rel)
{
  std::ifstream in(std::string(TRAINIT_SOURCE_DIR) + "/" + rel, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Code (comments and literals removed) and the contents of every string literal -- the same
// splitter as test_m2_5_seam_scan.cpp.
struct Split
{
  std::string code;
  std::vector<std::string> literals;
};

Split split(const std::string& src)
{
  Split out;
  std::size_t i = 0;
  const std::size_t n = src.size();
  while (i < n)
  {
    const char c = src[i];
    if (c == '/' && i + 1 < n && src[i + 1] == '/')
    {
      while (i < n && src[i] != '\n') ++i;
      continue;
    }
    if (c == '/' && i + 1 < n && src[i + 1] == '*')
    {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) ++i;
      i += 2;
      out.code += ' ';
      continue;
    }
    if (c == 'R' && i + 1 < n && src[i + 1] == '"')
    {
      const std::size_t open = src.find('(', i + 2);
      const std::string delim = ")" + src.substr(i + 2, open - (i + 2)) + "\"";
      const std::size_t close = src.find(delim, open);
      out.literals.push_back(src.substr(open + 1, close - open - 1));
      i = close + delim.size();
      out.code += "\"\"";
      continue;
    }
    if (c == '"' || c == '\'')
    {
      std::string lit;
      ++i;
      while (i < n && src[i] != c)
      {
        if (src[i] == '\\' && i + 1 < n) { lit += src[i + 1]; i += 2; continue; }
        lit += src[i++];
      }
      ++i;
      if (c == '"') out.literals.push_back(lit);
      out.code += "\"\"";
      continue;
    }
    out.code += c;
    ++i;
  }
  return out;
}

int statics(const Split& s)
{
  const std::regex re("\\bstatic\\b");   // static_cast is one word: not counted
  return static_cast<int>(std::distance(std::sregex_iterator(s.code.begin(), s.code.end(), re), std::sregex_iterator()));
}

// A literal that IS an absolute ROS name, or starts one ("/isaac_release_policy", "/perception/").
std::vector<std::string> absoluteNames(const Split& s)
{
  std::vector<std::string> out;
  const std::regex re("^/[A-Za-z_~]");
  for (const auto& l : s.literals)
    if (std::regex_search(l, re)) out.push_back(l);
  std::sort(out.begin(), out.end());
  return out;
}
}  // namespace

TEST(M26EefStaticScan, the_new_sources_have_no_function_static_and_no_absolute_name)
{
  for (const auto& f : kNewSources)
  {
    const std::string src = read(f);
    ASSERT_FALSE(src.empty()) << f;
    const Split s = split(src);
    EXPECT_EQ(statics(s), 0) << f << ": no function-static cache, map or publisher (ADR-0022 D7)";
    EXPECT_EQ(absoluteNames(s), std::vector<std::string>{}) << f << ": every endpoint name is relative";
    EXPECT_EQ(s.code.find("include <moveit"), std::string::npos) << f;
  }
}

TEST(M26EefStaticScan, the_legacy_units_gained_no_static_and_no_absolute_name)
{
  // bt_nodes.cpp's legacy statics (SetReleasePolicy's publisher, DetectionFeed, latestFeedFor,
  // triggerClientFor, publisherFor, the TF fallback, ServoStartRegistry, runningSceneManagers)
  // and its legacy absolute defaults, as they are at the M2.6 BEFORE freeze.
  const Split bt = split(read("src/bt_nodes.cpp"));
  EXPECT_EQ(statics(bt), 19);
  EXPECT_EQ(absoluteNames(bt),
            (std::vector<std::string>{"/delta_twist_cmds", "/detections", "/isaac_release_policy",
                                      "/isaac_release_policy", "/pause_servo", "/perception/",
                                      "/scene_manager_node", "/start_servo", "/status",
                                      "/trainit_policy_runtime/run", "/trainit_policy_runtime/run",
                                      "/trainit_policy_runtime/status", "/trainit_policy_runtime/status",
                                      "/unpause_servo"}));
  for (const char* f : {"src/trainit_run_bt_node.cpp", "src/gripper_controller.cpp"})
  {
    const Split s = split(read(f));
    EXPECT_EQ(statics(s), 0) << f;
    EXPECT_EQ(absoluteNames(s), std::vector<std::string>{}) << f;
  }
}

// ADR-0022 D9: the legacy tick bodies (bt_nodes.cpp:494-499 and 505-510 at the freeze) stay
// textually unchanged -- the canonical branch is one line BEFORE them.
TEST(M26EefStaticScan, the_legacy_gripper_tick_bodies_are_textually_unchanged)
{
  const std::string src = read("src/bt_nodes.cpp");
  const char* open_body =
    "  auto* ctx = ctxOf(config());\n"
    "  if (!ctx->gripper && !ctx->gripper_cmd_pub)\n"
    "  { RCLCPP_ERROR(nlog(config()), \"OpenGripper: no gripper configured\"); return BT::NodeStatus::FAILURE; }\n"
    "  const bool ok = ctx->gripper ? ctx->gripper->command(ctx->gripper_open_pos) : true;\n"
    "  if (ok) fireGraspSignal(ctx, false);\n"
    "  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;\n"
    "}\n";
  const char* close_body =
    "  auto* ctx = ctxOf(config());\n"
    "  if (!ctx->gripper && !ctx->gripper_cmd_pub)\n"
    "  { RCLCPP_ERROR(nlog(config()), \"CloseGripper: no gripper configured\"); return BT::NodeStatus::FAILURE; }\n"
    "  const bool ok = ctx->gripper ? ctx->gripper->command(ctx->gripper_close_pos) : true;\n"
    "  if (ok) fireGraspSignal(ctx, true);\n"
    "  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;\n"
    "}\n";
  EXPECT_NE(src.find(std::string("BT::NodeStatus OpenGripper::tick()\n{\n"
                                 "  if (ctxOf(config())->eefs) return eefGrip(*this, config(), false);   "
                                 "// the canonical contract (ADR-0022)\n") + open_body), std::string::npos);
  EXPECT_NE(src.find(std::string("BT::NodeStatus CloseGripper::tick()\n{\n"
                                 "  if (ctxOf(config())->eefs) return eefGrip(*this, config(), true);   "
                                 "// the canonical contract (ADR-0022)\n") + close_body), std::string::npos);
  // SetGripper's body is untouched too (a canonical tree refuses it at start-up instead)
  EXPECT_NE(src.find("  if (!ctx->gripper) { RCLCPP_ERROR(nlog(config()), \"SetGripper: no gripper configured\"); "
                     "return BT::NodeStatus::FAILURE; }\n"
                     "  return ctx->gripper->command(std::stod(in(*this, \"position\", \"1.0\"))) ? "
                     "BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;\n}\n"), std::string::npos);
}

// EEF-D3: GripperController reads the result that is already there, warns, and still returns true.
TEST(M26EefStaticScan, the_gripper_controller_changed_only_by_the_warn)
{
  const std::string src = read("src/gripper_controller.cpp");
  const char* legacy_head =
    "  auto result_future = client_->async_get_result(handle);\n"
    "  if (result_future.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready)\n"
    "  {\n"
    "    RCLCPP_ERROR(node_->get_logger(), \"gripper result timed out\");\n"
    "    return false;\n"
    "  }\n";
  const char* legacy_tail =
    "  RCLCPP_INFO(node_->get_logger(), \"gripper command position=%.2f done\", position);\n"
    "  return true;\n"
    "}\n";
  const auto head = src.find(legacy_head);
  const auto tail = src.find(legacy_tail);
  ASSERT_NE(head, std::string::npos);
  ASSERT_NE(tail, std::string::npos);
  const std::string between = src.substr(head + std::string(legacy_head).size(), tail - head - std::string(legacy_head).size());
  EXPECT_NE(between.find("is counted as done (LEGACY contract, ADR-0022)"), std::string::npos);
  EXPECT_EQ(between.find("return"), std::string::npos) << "the WARN never changes what command() returns";
  // everything before the WARN is the frozen client, byte for byte
  EXPECT_NE(src.find("  GripperCommand::Goal goal;\n  goal.command.position = position;\n"
                     "  goal.command.max_effort = 0.0;\n"), std::string::npos);
}

TEST(M26EefStaticScan, the_new_sources_are_classified_in_the_seam_scan)
{
  const std::string seam = read("test/test_m2_5_seam_scan.cpp");
  const auto plain = seam.find("const std::vector<std::string> kPlain");
  const auto end = seam.find("};", plain);
  ASSERT_NE(plain, std::string::npos);
  const std::string list = seam.substr(plain, end - plain);
  for (const auto& f : kNewSources)
  {
    if (f.rfind("src/", 0) != 0) continue;
    EXPECT_NE(list.find("\"" + f + "\""), std::string::npos) << f << " is not in kPlain";
  }
}

TEST(M26EefStaticScan, the_groot_model_lists_the_end_effector_ports)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  const std::string xml = read("config/bt/trainit_nodes.groot.xml");
  for (const char* id : {"CloseGripper", "OpenGripper", "CommandActuator", "VerifyGrasp", "DetectObject"})
  {
    const std::string open = std::string("<Action ID=\"") + id + "\">";
    const auto a = xml.find(open);
    ASSERT_NE(a, std::string::npos) << id << " is not in the Groot model";
    const auto b = xml.find("</Action>", a);
    const std::string block = xml.substr(a, b - a);
    std::map<std::string, std::string> model;
    const std::regex port("<(input|output)_port name=\"([A-Za-z_]+)\"");
    for (auto it = std::sregex_iterator(block.begin(), block.end(), port); it != std::sregex_iterator(); ++it)
      model[(*it)[2]] = (*it)[1];
    std::map<std::string, std::string> manifest;
    for (const auto& [name, info] : factory.manifests().at(id).ports)
      manifest[name] = info.direction() == BT::PortDirection::OUTPUT ? "output" : "input";
    EXPECT_EQ(model, manifest) << id;
  }
}

// ADR-0022 D4: "There is NO actuator port. Command outcomes never enter a verdict." VerifyGrasp asks
// the TOOL; its ports are exactly these (the Groot test above only checks the model against the
// manifest, so a port added to both would pass there).
TEST(M26EefStaticScan, verify_grasp_has_exactly_its_d4_ports_and_no_actuator_port)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  std::map<std::string, std::string> ports;
  for (const auto& [name, info] : factory.manifests().at("VerifyGrasp").ports)
    ports[name] = info.direction() == BT::PortDirection::OUTPUT ? "output" : "input";
  const std::map<std::string, std::string> d4{
    {"eef", "input"},       {"site", "input"},       {"expect", "input"},  {"timeout_ms", "input"},
    {"max_age_ms", "input"}, {"accept", "input"},     {"reason", "output"}, {"fault", "output"},
    {"subject", "output"}};
  EXPECT_EQ(ports, d4);
  for (const auto& [name, direction] : ports)
  {
    EXPECT_EQ(name.find("actuator"), std::string::npos) << name << ": VerifyGrasp names no actuator";
    EXPECT_EQ(name.find("target"), std::string::npos) << name << ": VerifyGrasp names no command target";
  }
}
