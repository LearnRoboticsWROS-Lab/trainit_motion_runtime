// ROS2ML-SYNC D3 (TMR 2.6.1): a static scan of the BT runner. Its tf2 Buffer is filled by the executor's
// spinner thread while the tree ticks on the main one (the listener is built with spin_thread=false), so
// the buffer must be told so: without setUsingDedicatedThread(true) tf2 refuses every lookup timeout --
// it logs an error and answers at once instead of waiting (seen on the ROS2ML course cell, whose current
// pose comes in 'world', at a FollowPath start and a DetectCandidates transform).
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace
{
std::string code(const std::string& rel)
{
  std::ifstream in(std::string(TRAINIT_SOURCE_DIR) + "/" + rel, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  // comments out: a commented-out call must not count
  return std::regex_replace(std::regex_replace(ss.str(), std::regex(R"(//[^\n]*)"), ""),
                            std::regex(R"(/\*[\s\S]*?\*/)"), "");
}
}  // namespace

TEST(TfDedicatedThread, TheRunnersBufferIsToldTheSpinnerFillsIt)
{
  const std::string src = code("src/trainit_run_bt_node.cpp");
  ASSERT_FALSE(src.empty());
  const std::regex listener(R"(TransformListener>\(\s*\*\s*(\w+)\s*,\s*\w+\s*,\s*false\s*\))");
  int listeners = 0;
  for (std::sregex_iterator it(src.begin(), src.end(), listener), end; it != end; ++it)
  {
    ++listeners;
    const std::string buffer = (*it)[1];
    const std::string before = src.substr(0, static_cast<std::size_t>(it->position()));
    EXPECT_NE(before.find(buffer + "->setUsingDedicatedThread(true)"), std::string::npos)
      << "the tf2 Buffer '" << buffer << "' is spun by the executor's thread: call "
      << buffer << "->setUsingDedicatedThread(true) before its TransformListener";
  }
  EXPECT_EQ(listeners, 1) << "the runner builds one listener without its own thread";
  // the premise of the flag: the executor holds the node and its spinner thread runs BEFORE the buffer
  const auto buffer_at = src.find("std::make_shared<tf2_ros::Buffer>");
  ASSERT_NE(buffer_at, std::string::npos);
  for (const char* before_buffer : {"exec.add_node(node)", "std::thread spinner"})
  {
    const auto at = src.find(before_buffer);
    EXPECT_NE(at, std::string::npos) << before_buffer;
    EXPECT_LT(at, buffer_at) << "'" << before_buffer << "' must come before the tf2 Buffer it fills";
  }
}
