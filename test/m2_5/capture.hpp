// vNext M2.5 test harness -- the capture: what a scenario sent to MoveIt, as deterministic text.
//
// Every request a fake MoveIt endpoint receives is recorded as an EVENT: a global sequence
// number, the RESOLVED endpoint name, the type, the scripted answer, a one-line summary and the
// whole message. One BT thread causes every request (each call waits for its answer), so the
// ORDER of events is part of what is compared. Asynchronous traffic (topics) is recorded apart
// and printed sorted. A message is written as rosidl block-style YAML at precision 17 (rosidl's
// value_to_yaml honours the stream precision; the std::string to_yaml overload prints 6
// significant digits) with every `stamp:` mapping removed; a trajectory as a summary plus the
// sha256 of that block. Excluded on purpose: stamps, goal ids, result planning times, wall-clock
// durations, log text, joint_states traffic, the number of streamed servo twists.
//
// A FAMILY file (test/data/m2_5/<family>.txt) is a header (family, the MoveIt and BT.CPP versions
// this binary was built against, the rmw) and one block per scenario. Each run writes its capture
// to the build tree (TRAINIT_M25_ACTUAL_DIR) and compares it byte for byte with the committed one,
// which was recorded on TMR 1.8.0 (9cfb601) before the M2.5 seam touched any production line.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_5_CAPTURE_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_5_CAPTURE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <memory>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rmw/rmw.h>

namespace m25
{

// --- sha256 (FIPS 180-4): a digest of a captured trajectory -- identity, not security -----------
inline std::string sha256(const std::string& data)
{
  static const uint32_t k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  std::string msg = data;
  const uint64_t bits = static_cast<uint64_t>(data.size()) * 8u;
  msg.push_back(static_cast<char>(0x80));
  while (msg.size() % 64 != 56) msg.push_back('\0');
  for (int i = 7; i >= 0; --i) msg.push_back(static_cast<char>((bits >> (i * 8)) & 0xff));
  for (std::size_t off = 0; off < msg.size(); off += 64)
  {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = (static_cast<uint32_t>(static_cast<unsigned char>(msg[off + 4 * i])) << 24) |
             (static_cast<uint32_t>(static_cast<unsigned char>(msg[off + 4 * i + 1])) << 16) |
             (static_cast<uint32_t>(static_cast<unsigned char>(msg[off + 4 * i + 2])) << 8) |
             static_cast<uint32_t>(static_cast<unsigned char>(msg[off + 4 * i + 3]));
    for (int i = 16; i < 64; ++i)
    {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i)
    {
      const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = hh + S1 + ch + k[i] + w[i];
      const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = S0 + mj;
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  char out[65];
  for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", h[i]);
  return std::string(out, 64);
}

// --- serialization ---------------------------------------------------------------------------------
// Delete every `stamp:` mapping (the line and everything indented deeper) and indent by `indent`.
inline std::string stripStamps(const std::string& yaml, int indent = 2)
{
  std::istringstream in(yaml);
  std::ostringstream out;
  std::string line;
  int skip_deeper_than = -1;
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  while (std::getline(in, line))
  {
    const auto first = line.find_first_not_of(' ');
    const int depth = first == std::string::npos ? 0 : static_cast<int>(first);
    if (skip_deeper_than >= 0)
    {
      if (depth > skip_deeper_than) continue;
      skip_deeper_than = -1;
    }
    const std::string text = first == std::string::npos ? "" : line.substr(first);
    if (text.rfind("stamp:", 0) == 0)
    {
      skip_deeper_than = depth;
      continue;
    }
    out << pad << line << "\n";
  }
  std::string s = out.str();
  if (!s.empty() && s.back() == '\n') s.pop_back();
  return s;
}

// The whole message, block style, precision 17, stamps removed (found by ADL in the message's
// own namespace, e.g. moveit_msgs::action::to_block_style_yaml).
template <typename MsgT>
std::string yaml17(const MsgT& msg, int indent = 2)
{
  std::ostringstream os;
  os.precision(17);
  to_block_style_yaml(msg, os, 0);
  return stripStamps(os.str(), indent);
}

inline std::string num17(double v)
{
  std::ostringstream os;
  os.precision(17);
  os << v;
  return os.str();
}

inline std::string vec17(const std::vector<double>& v)
{
  std::string s = "[";
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + num17(v[i]);
  return s + "]";
}

inline std::string point17(const trajectory_msgs::msg::JointTrajectoryPoint& p)
{
  return "positions=" + vec17(p.positions) + " velocities=" + vec17(p.velocities) +
         " accelerations=" + vec17(p.accelerations) + " t=" + std::to_string(p.time_from_start.sec) +
         "s+" + std::to_string(p.time_from_start.nanosec) + "ns";
}

// A trajectory as a reviewable summary plus the digest of its full precision-17 block.
inline std::string trajectorySummary(const moveit_msgs::msg::RobotTrajectory& t, int indent = 2)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  const auto& jt = t.joint_trajectory;
  std::ostringstream os;
  os << pad << "joint_trajectory: frame=" << jt.header.frame_id << " joints=[";
  for (std::size_t i = 0; i < jt.joint_names.size(); ++i) os << (i ? "," : "") << jt.joint_names[i];
  os << "] points=" << jt.points.size() << "\n";
  if (!jt.points.empty())
  {
    os << pad << "  first: " << point17(jt.points.front()) << "\n";
    os << pad << "  last:  " << point17(jt.points.back()) << "\n";
  }
  os << pad << "multi_dof_points=" << t.multi_dof_joint_trajectory.points.size() << "\n";
  os << pad << "sha256(precision-17 block)=" << sha256(yaml17(t, 0));
  return os.str();
}

// --- the log ---------------------------------------------------------------------------------------
class Log
{
public:
  // `summary` is always kept; `body` (the whole message) is dropped in a COMPACT family, where
  // only what the summary says is compared (e.g. the planner-identity matrix).
  void event(const std::string& kind, const std::string& endpoint, const std::string& what,
             const std::string& summary = "", const std::string& body = "")
  {
    std::lock_guard<std::mutex> g(m_);
    std::ostringstream os;
    os << "[" << ++seq_ << "] " << kind << " " << endpoint << " " << what;
    if (!summary.empty()) os << "\n  | " << summary;
    if (!body.empty() && bodies_) os << "\n" << body;
    events_.push_back(os.str());
    ++counts_[endpoint];
  }
  void topic(const std::string& endpoint, const std::string& what, const std::string& body)
  {
    std::lock_guard<std::mutex> g(m_);
    topics_.push_back(endpoint + " " + what + (body.empty() ? "" : "\n" + body));
    ++counts_[endpoint];
  }
  void setBodies(bool on)
  {
    std::lock_guard<std::mutex> g(m_);
    bodies_ = on;
  }
  void reset()
  {
    std::lock_guard<std::mutex> g(m_);
    events_.clear();
    topics_.clear();
    counts_.clear();
    seq_ = 0;
  }
  std::string events() const
  {
    std::lock_guard<std::mutex> g(m_);
    std::string s;
    for (const auto& e : events_) s += e + "\n";
    return s;
  }
  std::string topics() const
  {
    std::lock_guard<std::mutex> g(m_);
    std::vector<std::string> t = topics_;
    std::sort(t.begin(), t.end());
    std::string s;
    for (const auto& e : t) s += e + "\n";
    return s;
  }
  std::string counts() const
  {
    std::lock_guard<std::mutex> g(m_);
    std::string s;
    for (const auto& [k, v] : counts_) s += (s.empty() ? "" : " ") + k + "=" + std::to_string(v);
    return s.empty() ? "none" : s;
  }
  std::size_t topicCount() const
  {
    std::lock_guard<std::mutex> g(m_);
    return topics_.size();
  }
  int count(const std::string& endpoint) const
  {
    std::lock_guard<std::mutex> g(m_);
    const auto it = counts_.find(endpoint);
    return it == counts_.end() ? 0 : it->second;
  }

private:
  mutable std::mutex m_;
  std::vector<std::string> events_, topics_;
  std::map<std::string, int> counts_;
  int seq_{0};
  bool bodies_{true};
};

// --- a spinner without the cancel-before-spin race ---------------------------------------------------
// Executor::cancel() only stops a spin() that has already started: a cancel() that lands between
// std::thread's start and spin()'s first line is lost, and join() then waits for ever. stop()
// therefore cancels until the spin has really returned.
template <typename ExecutorT>
class Spinner
{
public:
  template <typename... Args>
  explicit Spinner(Args&&... args) : exec_(std::make_unique<ExecutorT>(std::forward<Args>(args)...)) {}
  ~Spinner() { stop(); }
  ExecutorT& executor() { return *exec_; }
  void start()
  {
    done_ = std::async(std::launch::async, [this] { exec_->spin(); });
  }
  void stop()
  {
    if (!done_.valid()) return;
    while (done_.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) exec_->cancel();
    done_.get();
  }

private:
  std::unique_ptr<ExecutorT> exec_;
  std::future<void> done_;
};

// --- a watchdog: a hang is a NAMED failure, never a silent ctest timeout -----------------------------
class Watchdog
{
public:
  Watchdog() : thread_([this] { loop(); }) {}
  ~Watchdog()
  {
    { std::lock_guard<std::mutex> g(m_); quit_ = true; }
    cv_.notify_all();
    thread_.join();
  }
  void arm(const std::string& what, std::chrono::seconds budget)
  {
    std::lock_guard<std::mutex> g(m_);
    what_ = what;
    deadline_ = std::chrono::steady_clock::now() + budget;
    armed_ = true;
    cv_.notify_all();
  }
  void disarm()
  {
    std::lock_guard<std::mutex> g(m_);
    armed_ = false;
    cv_.notify_all();
  }

private:
  void loop()
  {
    std::unique_lock<std::mutex> lk(m_);
    while (!quit_)
    {
      if (!armed_) { cv_.wait(lk); continue; }
      if (cv_.wait_until(lk, deadline_) == std::cv_status::timeout && armed_ && !quit_ &&
          std::chrono::steady_clock::now() >= deadline_)
      {
        std::fprintf(stderr, "\nM2.5 WATCHDOG: '%s' exceeded its budget -- a hang is a FAILURE\n",
                     what_.c_str());
        std::fflush(stderr);
        std::_Exit(3);
      }
    }
  }
  std::mutex m_;
  std::condition_variable cv_;
  std::string what_;
  std::chrono::steady_clock::time_point deadline_{};
  bool armed_{false}, quit_{false};
  std::thread thread_;
};

// --- a family file ---------------------------------------------------------------------------------
inline std::string environmentLine()
{
  return std::string("moveit_ros_planning_interface ") + TRAINIT_MOVEIT_VERSION +
         " | behaviortree_cpp " + TRAINIT_BTCPP_VERSION + " | rmw " +
         rmw_get_implementation_identifier();
}

// The legacy families vNext M3.4 moved (their AFTER: test/data/m3_4/legacy_after/).
inline constexpr const char* kM34LegacyAfter[] = {
  "planned_waypoint", "planned_lin_circ", "planner_identity", "sequences", "task_runner", "task_runner_2",
  "state_reads", "servo_track_2", "namespace", "namespace_remap"};

class Family
{
public:
  // `frozen_dir`: where the committed capture lives -- test/data/m2_5 (the 1.8.0 BEFORE record) by
  // default; test/data/m2_7_ns for the namespace families' AFTER series (owner decision D2, M2.7).
  explicit Family(std::string name, std::string frozen_dir = TRAINIT_M25_DATA_DIR)
  : name_(std::move(name)), frozen_dir_(std::move(frozen_dir))
  {
    // vNext M3.4 (owner D3): a family whose approved M3.4 scenarios moved compares with its AFTER
    // record, test/data/m3_4/legacy_after/; its BEFORE stays where it was, byte for byte, and
    // test_m3_4_capture_diff proves the difference is exactly the approved scenarios.
    for (const char* moved : kM34LegacyAfter)
      if (name_ == moved) frozen_dir_ = std::string(TRAINIT_M25_DATA_DIR) + "/../m3_4/legacy_after";
    text_ = "m2_5 capture " + name_ + "\n" + environmentLine() + "\n\n";
  }

  void add(const std::string& block) { text_ += block; }

  // Writes the capture to the build tree, then compares it with the committed BEFORE capture.
  void check() const
  {
    const std::string actual = std::string(TRAINIT_M25_ACTUAL_DIR) + "/" + name_ + ".txt";
    {
      std::ofstream out(actual, std::ios::binary | std::ios::trunc);
      out << text_;
    }
    const std::string frozen = frozen_dir_ + "/" + name_ + ".txt";
    std::ifstream in(frozen, std::ios::binary);
    ASSERT_TRUE(in.is_open()) << "no committed capture " << frozen << " -- this run's capture is "
                              << actual;
    std::stringstream committed;
    committed << in.rdbuf();
    const std::string want = committed.str();
    const auto header_end = [](const std::string& s) {
      const auto a = s.find('\n');
      return a == std::string::npos ? s.size() : s.find('\n', a + 1);
    };
    ASSERT_EQ(want.substr(0, header_end(want)), text_.substr(0, header_end(text_)))
      << "the environment differs from the one the captures were recorded in (MoveIt, BT.CPP or "
         "the rmw): this is NOT a pass, and NOT a reason to re-capture from the current code. A "
         "capture is re-frozen only from the production sources the README beside it names ("
      << frozen_dir_ << "/README.md: 9cfb601 for test/data/m2_5). This run's capture: " << actual;
    EXPECT_EQ(want, text_) << "the " << name_ << " family's MoveIt traffic changed; compare "
                           << frozen << " with " << actual;
  }

  const std::string& text() const { return text_; }

private:
  std::string name_;
  std::string frozen_dir_;
  std::string text_;
};

}  // namespace m25

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_5_CAPTURE_HPP_
