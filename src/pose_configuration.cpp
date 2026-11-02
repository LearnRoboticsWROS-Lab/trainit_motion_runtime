// =============================================================================
// pose_configuration.cpp -- see pose_configuration.hpp (TMR 2.6.0, ADR-0028).
// =============================================================================
#include "trainit_motion_runtime/pose_configuration.hpp"

#include <algorithm>
#include <cmath>

namespace trainit
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;
constexpr double kSameSolution = 1e-4;   // [rad] two solutions this close on every joint count once

struct Distance
{
  double max{0.0};
  double sum{0.0};
  std::size_t worst{0};
};

Distance distance(const std::vector<double>& a, const std::vector<double>& b)
{
  Distance d;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i)
  {
    const double v = std::fabs(a[i] - b[i]);
    d.sum += v;
    if (v > d.max)
    {
      d.max = v;
      d.worst = i;
    }
  }
  return d;
}
}  // namespace

std::optional<PoseGoalConfiguration> poseGoalConfigurationFromString(const std::string& text)
{
  if (text == "nearest") return PoseGoalConfiguration::NEAREST;
  if (text == "planner") return PoseGoalConfiguration::PLANNER;
  return std::nullopt;
}

const char* toString(PoseGoalConfiguration mode)
{
  return mode == PoseGoalConfiguration::NEAREST ? "nearest" : "planner";
}

std::vector<double> nearestEquivalent(const std::vector<double>& joints, const std::vector<double>& reference,
                                      const std::vector<JointBounds>& bounds)
{
  std::vector<double> out = joints;
  for (std::size_t i = 0; i < out.size() && i < reference.size() && i < bounds.size(); ++i)
  {
    const JointBounds& b = bounds[i];
    if (!b.angular) continue;                // a prismatic variable has no other equivalent
    // the multiple of 2*pi that brings the value nearest the reference, then the nearest one the
    // bounds allow (a bounded joint keeps a value it cannot move)
    const double k = std::round((reference[i] - out[i]) / kTwoPi);
    double best = out[i];
    for (const double kk : {k, k - 1.0, k + 1.0})
    {
      const double v = out[i] + kk * kTwoPi;
      if (!b.continuous && (v < b.lower - 1e-9 || v > b.upper + 1e-9)) continue;
      if (std::fabs(v - reference[i]) < std::fabs(best - reference[i])) best = v;
    }
    out[i] = best;
  }
  return out;
}

ConfigurationChoice chooseConfiguration(const std::vector<std::vector<double>>& solutions,
                                        const std::vector<double>& reference,
                                        const std::vector<JointBounds>& bounds,
                                        std::optional<double> tolerance)
{
  ConfigurationChoice choice;
  std::vector<std::vector<double>> distinct;
  for (const auto& s : solutions)
  {
    if (s.size() != reference.size() || !allFinite(s)) continue;
    std::vector<double> q = nearestEquivalent(s, reference, bounds);
    const bool seen = std::any_of(distinct.begin(), distinct.end(), [&](const std::vector<double>& d) {
      return distance(d, q).max < kSameSolution;
    });
    if (!seen) distinct.push_back(std::move(q));
  }
  choice.distinct = distinct.size();
  if (distinct.empty()) return choice;
  const std::vector<double>* best = nullptr;
  Distance best_d;
  for (const auto& q : distinct)
  {
    const Distance d = distance(q, reference);
    if (!best || d.max < best_d.max - 1e-12 || (std::fabs(d.max - best_d.max) <= 1e-12 && d.sum < best_d.sum))
    {
      best = &q;
      best_d = d;
    }
  }
  choice.max_difference = best_d.max;
  choice.worst_joint = best_d.worst;
  if (tolerance && best_d.max > *tolerance)
  {
    choice.refused = true;
    return choice;
  }
  choice.joints = *best;
  return choice;
}

bool allFinite(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

}  // namespace trainit
