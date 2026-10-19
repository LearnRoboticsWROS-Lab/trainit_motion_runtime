// =============================================================================
// grasp_candidates.hpp -- the GraspCandidate contract on the runtime side (vNext M3.6, ADR-0026).
//
// ANY grasp producer publishes trainit_interfaces/GraspCandidateArray; the runtime executes every
// candidate through ONE contract, whatever made it. This header is that contract's pure half (no ROS
// graph, no MoveIt, unit-tested by calling it):
//
//   candidateProblem     what makes a candidate unusable ("" when usable): a frame, an id and a
//                        provider; a finite pose with a unit quaternion; a finite UNIT approach
//                        (within 1e-3, never repaired); a finite standoff >= 0; finite numbers
//   arrayProblem         the same over an array, and the ids unique within it
//   projectDetection     the MINIMAL M3 bridge Detection3D -> GraspCandidate (NOT a grasp method):
//                        the perception contract's fitted orientation takes +Z onto the surface
//                        normal, which faces the camera, so approach = -normal (INTO the surface);
//                        an identity orientation means "no normal" -> the configured default
//                        approach, else refused; a zero, non-finite or non-unit quaternion is refused
//                        (never read as "no normal"). Position, frame and stamp are kept; no object
//                        pose and no rotation about the approach are invented (orientation identity)
//   minimalRotation      the smallest rotation taking unit z onto unit a; none when they are
//                        antiparallel (its axis is undefined)
//   alignApproach        `orientation: align_approach` (owner decision M3.6-1): R_tcp =
//                        MinRot(R_ref e_z -> a) * R_ref, the application's reference orientation
//                        tilted as little as possible; refused when R_ref's +Z opposes the approach
//
// The blackboard layout DetectCandidates writes and SetWaypointFromCandidate reads (always indexed,
// strings, 12 significant digits; geometry in the candidate set's frame):
//   <out>.count
//   <out>.<i>.position | orientation | approach | normal | standoff_m | eef | site | score |
//            confidence | id | provider | object_class | object_id | frame | stamp
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__GRASP_CANDIDATES_HPP_
#define TRAINIT_MOTION_RUNTIME__GRASP_CANDIDATES_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <std_msgs/msg/header.hpp>
#include <trainit_interfaces/msg/grasp_candidate.hpp>
#include <trainit_interfaces/msg/grasp_candidate_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

namespace trainit
{

using GraspCandidateMsg = trainit_interfaces::msg::GraspCandidate;
using GraspCandidateArrayMsg = trainit_interfaces::msg::GraspCandidateArray;

// how far a quaternion's or an approach's norm may be from 1 (the TSA / path-library tolerance)
inline constexpr double kUnitTolerance = 1e-3;
// a detection orientation closer than this to the identity [rad] carries no surface normal
inline constexpr double kIdentityAngle = 1e-6;
// how close to antiparallel two unit vectors may be before the minimal rotation's axis is undefined
inline constexpr double kAntiparallelSine = 1e-6;
// the provider id of the detection projection
inline constexpr const char* kProjectionProvider = "tmr.candidates_from_detections";

// --- the blackboard layout ---------------------------------------------------------------------------
inline std::string candidatePrefix(const std::string& out_key, std::size_t i)
{
  return out_key + "." + std::to_string(i);
}
inline std::string candidateCountKey(const std::string& out_key) { return out_key + ".count"; }

// "a;b;c" with 12 significant digits (a candidate's geometry survives the string round trip)
std::string formatNumbers(const std::vector<double>& v);

// --- validation ----------------------------------------------------------------------------------------
std::string candidateProblem(const GraspCandidateMsg& c);
// "" when every candidate is usable and the ids are unique; else "candidate <i> ('<id>'): <why>"
std::string arrayProblem(const GraspCandidateArrayMsg& a);

// --- the detection projection --------------------------------------------------------------------------
struct ProjectionConfig
{
  // the approach of a detection WITHOUT a fitted normal, in the DETECTION's frame (a unit vector);
  // none: such a detection is refused
  std::optional<Eigen::Vector3d> default_approach;
  double standoff_m{0.0};    // the candidate's suggested standoff (0 = none suggested)
  std::string eef;           // the device / site the candidates are for ("" = the defaults)
  std::string site;
  std::string detector;      // recorded in the metadata (the source of the observation)
};

struct Projection
{
  GraspCandidateMsg candidate;
  bool fitted_normal{false};
  std::string problem;       // "" when the candidate is usable
};

// Detection `d`, published at position `index` of an array with header `array_header`.
Projection projectDetection(const vision_msgs::msg::Detection3D& d, const std_msgs::msg::Header& array_header,
                            std::size_t index, const ProjectionConfig& cfg);

// --- orientation -------------------------------------------------------------------------------------
std::optional<Eigen::Quaterniond> minimalRotation(const Eigen::Vector3d& z, const Eigen::Vector3d& a);
std::optional<Eigen::Quaterniond> alignApproach(const Eigen::Quaterniond& reference, const Eigen::Vector3d& approach);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__GRASP_CANDIDATES_HPP_
