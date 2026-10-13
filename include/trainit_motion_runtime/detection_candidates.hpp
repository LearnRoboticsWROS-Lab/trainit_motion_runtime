// =============================================================================
// detection_candidates.hpp — the candidate LIST a detection array offers, and the
// blackboard names it is published under (ADR-0016 Decision 2).
//
// ADR-0016 Decision 1: the detector publishes `vision_msgs/Detection3DArray`
// **best first**, and that published ORDER IS THE RANKING. So the selection below
// filters and truncates — it never sorts. The ranking terms a bin-picking cell ranks by
// (`score x pose_quality x visible_fraction`) are the detector's vocabulary, and
// the perception contract exists precisely so the runtime never learns it: a
// detector reports in the camera's optical frame and knows nothing about robots,
// a runtime consumes poses and knows nothing about masks.
//
// One rule for every `keep`: the first `keep` matches IN PUBLISHED ORDER. No sort, and
// no comparison of scores. What differs is only where they are written, and that is the
// compatibility hinge:
//
//   keep = 1   the first match, on the FLAT keys `<out_key>.*`. Every bundle generated
//              before ADR-0016 emits `<DetectObject .../>` with no `keep` attribute, so
//              this LAYOUT must not move.
//   keep > 1   `<out_key>.<i>.*` with `<out_key>.count` beside them.
//
// keep = 1 used to pick the single HIGHEST-SCORING match instead: a re-rank by score
// inside the runtime, which overrode the detector's published ranking for every
// single-candidate application. Removed in vNext M0C #4. A detector that publishes in the
// order the contract asks for (best first) is unaffected when it has one object; a
// pre-ranking colour mask publishes largest-first, which is what keep = 1 now takes.
//
// Both nodes of the pair share these naming functions, so the layout is defined
// once: DetectObject writes it, SetWaypointFromDetection reads it.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__DETECTION_CANDIDATES_HPP_
#define TRAINIT_MOTION_RUNTIME__DETECTION_CANDIDATES_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <vision_msgs/msg/detection3_d_array.hpp>

namespace trainit
{

// `keep` is a COUNT: anything below 1 is a configuration mistake, and the only
// safe reading of it is "the single best" — the first published.
inline int clampDetectionKeep(int keep)
{
  return keep < 1 ? 1 : keep;
}

// The blackboard prefix candidate `i` is written under. keep = 1 keeps the flat
// prefix the pre-ADR-0016 vocabulary uses; keep > 1 indexes.
inline std::string detectionCandidatePrefix(const std::string& out_key, int keep, std::size_t i)
{
  if (clampDetectionKeep(keep) == 1) return out_key;
  return out_key + "." + std::to_string(i);
}

// Where the list's length lives. Written only when there IS a list (keep > 1):
// at keep = 1 the blackboard must look exactly as it always has.
inline std::string detectionCountKey(const std::string& out_key)
{
  return out_key + ".count";
}

// The prefix a CONSUMER tries for candidate `index` — the same names
// detectionCandidatePrefix() produces for keep > 1. When it is absent the consumer
// falls back to the flat `from` (see SetWaypointFromDetection: the fallback is what
// keeps `index` defaulted to 0 working against a keep = 1 detection).
inline std::string detectionIndexedPrefix(const std::string& from, int index)
{
  return from + "." + std::to_string(index);
}

// Indices into `msg.detections` of the candidates to publish.
//
//   - a detection with no hypothesis is skipped (nothing to read a class or a
//     score from);
//   - `class_id` empty means any class;
//   - a hypothesis score below `min_score` is skipped;
//   - the first `keep` matches, IN PUBLISHED ORDER. Not sorted, for keep = 1 too.
//
// Returns empty when nothing matches, which is the caller's "fresh message but no
// detection" failure.
inline std::vector<std::size_t> selectDetectionCandidates(
  const vision_msgs::msg::Detection3DArray& msg, const std::string& class_id,
  double min_score, int keep)
{
  const int k = clampDetectionKeep(keep);
  std::vector<std::size_t> out;

  for (std::size_t i = 0; i < msg.detections.size() && out.size() < static_cast<std::size_t>(k); ++i)
  {
    const auto& d = msg.detections[i];
    if (d.results.empty()) continue;
    const auto& h = d.results.front();
    if (!class_id.empty() && h.hypothesis.class_id != class_id) continue;
    if (h.hypothesis.score < min_score) continue;
    out.push_back(i);   // published order == rank order; no comparison, by design
  }
  return out;
}

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__DETECTION_CANDIDATES_HPP_
