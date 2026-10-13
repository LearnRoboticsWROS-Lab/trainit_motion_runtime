// vNext M2.7 test harness -- the M2.7 captures, as deterministic text.
//
// Same format as test/data/m2_5 (test/m2_5/capture.hpp): a header (the family, then the MoveIt,
// BT.CPP and rmw versions this binary was built against) and one block per scenario, written by
// the M2.5 scenario runner (m25::run). Each run writes its capture to the build tree
// (TRAINIT_M27_ACTUAL_DIR) and compares it byte for byte with the committed one, which was
// recorded on TMR production adf94b0 (2.0.0) before vNext M2.7 touched a production line
// (test/data/m2_7/README.md).
//
// Reused from M2.5 (test/m2_5/capture.hpp): environmentLine, Log, Spinner, Watchdog, yaml17.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_7_CAPTURE27_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_7_CAPTURE27_HPP_

#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "m2_5/capture.hpp"

namespace m27
{

// The production sources every M2.7 BEFORE capture was frozen on (test/data/m2_7/README.md).
inline constexpr const char* kFrozenOn = "adf94b0";

class Family
{
public:
  explicit Family(std::string name) : name_(std::move(name))
  {
    text_ = "m2_7 capture " + name_ + "\n" + m25::environmentLine() + "\n\n";
  }

  void add(const std::string& block) { text_ += block; }

  // Writes the capture to the build tree, then compares it with the committed BEFORE capture.
  void check() const
  {
    const std::string actual = std::string(TRAINIT_M27_ACTUAL_DIR) + "/" + name_ + ".txt";
    {
      std::ofstream out(actual, std::ios::binary | std::ios::trunc);
      out << text_;
    }
    const std::string frozen = std::string(TRAINIT_M27_DATA_DIR) + "/" + name_ + ".txt";
    std::ifstream in(frozen, std::ios::binary);
    ASSERT_TRUE(in.is_open()) << "no committed capture " << frozen << " -- this run's capture is "
                              << actual << ". A capture is frozen only from the " << kFrozenOn
                              << " production sources, after the determinism gate "
                                 "(test/data/m2_7/README.md).";
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
         "capture is re-frozen only from the " << kFrozenOn << " production sources "
         "(test/data/m2_7/README.md). This run's capture: " << actual;
    EXPECT_EQ(want, text_) << "the " << name_ << " family's MoveIt traffic changed; compare "
                           << frozen << " with " << actual << ". A behaviour change is an owner "
                           << "decision, never a re-freeze (test/data/m2_7/README.md; frozen on "
                           << "production " << kFrozenOn << ").";
  }

  const std::string& text() const { return text_; }

private:
  std::string name_;
  std::string text_;
};

}  // namespace m27

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_7_CAPTURE27_HPP_
