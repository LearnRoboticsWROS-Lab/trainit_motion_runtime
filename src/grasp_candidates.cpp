// =============================================================================
// grasp_candidates.cpp -- the GraspCandidate contract's pure half (vNext M3.6, ADR-0026). See the
// header: validation, the minimal Detection3D projection, align_approach.
// =============================================================================
#include "trainit_motion_runtime/grasp_candidates.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <set>
#include <sstream>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace trainit
{
namespace
{
bool finite(std::initializer_list<double> v)
{
  for (double x : v)
    if (!std::isfinite(x)) return false;
  return true;
}

double norm3(double x, double y, double z) { return std::sqrt(x * x + y * y + z * z); }

std::string num(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.12g", v);
  return buf;
}

diagnostic_msgs::msg::KeyValue kv(const std::string& k, const std::string& v)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = k;
  out.value = v;
  return out;
}
}  // namespace

std::string formatNumbers(const std::vector<double>& v)
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ";" : "") + num(v[i]);
  return s;
}

std::string candidateProblem(const GraspCandidateMsg& c)
{
  if (c.header.frame_id.empty()) return "no frame (header.frame_id is empty)";
  if (c.id.empty()) return "no id (a candidate's id is its identity: it is never empty)";
  if (c.provider.empty()) return "no provider (the producer's row id is never empty)";
  const auto& p = c.grasp_pose.position;
  const auto& q = c.grasp_pose.orientation;
  if (!finite({p.x, p.y, p.z})) return "the grasp position is not finite";
  if (!finite({q.x, q.y, q.z, q.w})) return "the grasp orientation is not finite";
  const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::abs(qn - 1.0) > kUnitTolerance)
    return "the grasp orientation is not a unit quaternion (norm " + num(qn) + "): it is never normalised silently";
  const auto& a = c.approach;
  if (!finite({a.x, a.y, a.z})) return "the approach is not finite";
  const double an = norm3(a.x, a.y, a.z);
  if (std::abs(an - 1.0) > kUnitTolerance)
    return "the approach is not a unit vector (norm " + num(an) + "): the direction the tool travels must be "
           "explicit, it is never normalised silently";
  if (!std::isfinite(c.standoff_m) || c.standoff_m < 0.0)
    return "the standoff " + num(c.standoff_m) + " m is not a finite distance >= 0";
  if (!finite({c.score, c.confidence})) return "the score or the confidence is not finite";
  if (c.confidence < 0.0 || c.confidence > 1.0)
    return "the confidence " + num(c.confidence) + " is not in 0..1 (0 = the producer states none)";
  const auto& n = c.surface_normal;
  if (!finite({n.x, n.y, n.z})) return "the surface normal is not finite";
  return "";
}

std::string arrayProblem(const GraspCandidateArrayMsg& a)
{
  std::set<std::string> ids;
  for (std::size_t i = 0; i < a.candidates.size(); ++i)
  {
    const auto& c = a.candidates[i];
    const std::string why = candidateProblem(c);
    if (!why.empty()) return "candidate " + std::to_string(i) + " ('" + c.id + "'): " + why;
    if (!ids.insert(c.id).second)
      return "candidate " + std::to_string(i) + " ('" + c.id + "'): its id is not unique within the array";
  }
  return "";
}

Projection projectDetection(const vision_msgs::msg::Detection3D& d, const std_msgs::msg::Header& array_header,
                            std::size_t index, const ProjectionConfig& cfg)
{
  Projection out;
  GraspCandidateMsg& c = out.candidate;
  c.header.frame_id = d.header.frame_id.empty() ? array_header.frame_id : d.header.frame_id;
  c.header.stamp = array_header.stamp;
  c.id = "detection_" + std::to_string(index);
  c.provider = kProjectionProvider;
  c.standoff_m = cfg.standoff_m;
  c.eef = cfg.eef;
  c.site = cfg.site;
  if (d.results.empty())
  {
    out.problem = "the detection carries no hypothesis (no pose)";
    return out;
  }
  const auto& h = d.results.front();
  c.score = h.hypothesis.score;   // the detector's rank value; the projection states no confidence (0)
  c.object_class = h.hypothesis.class_id;
  const auto& p = h.pose.pose.position;
  const auto& q = h.pose.pose.orientation;
  c.grasp_pose.position = p;
  // the identity: a projected detection proposes a point and an approach, never a rotation about it
  c.grasp_pose.orientation.x = c.grasp_pose.orientation.y = c.grasp_pose.orientation.z = 0.0;
  c.grasp_pose.orientation.w = 1.0;
  c.metadata.push_back(kv("source", "detection"));
  if (!cfg.detector.empty()) c.metadata.push_back(kv("detector", cfg.detector));
  if (!finite({p.x, p.y, p.z}))
  {
    out.problem = "the detection's position is not finite";
    return out;
  }
  // the orientation is read only as a NORMAL: a zero, non-finite or non-unit quaternion is a broken
  // detection, never "no normal"
  if (!finite({q.x, q.y, q.z, q.w}))
  {
    out.problem = "the detection's orientation is not finite: its surface normal cannot be read";
    return out;
  }
  const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::abs(qn - 1.0) > kUnitTolerance)
  {
    out.problem = "the detection's orientation is not a unit quaternion (norm " + num(qn) +
                  "): its surface normal cannot be read";
    return out;
  }
  const Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
  // the rotation angle, exact near 0 (acos loses precision there)
  const double angle = 2.0 * std::atan2(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z), std::abs(q.w));
  if (angle > kIdentityAngle)
  {
    // the perception contract: the orientation takes +Z onto the fitted surface normal, which faces
    // the camera -- the tool travels against it, INTO the surface
    const Eigen::Vector3d n = (Q.normalized() * Eigen::Vector3d::UnitZ()).normalized();
    c.surface_normal.x = n.x();
    c.surface_normal.y = n.y();
    c.surface_normal.z = n.z();
    c.approach.x = -n.x();
    c.approach.y = -n.y();
    c.approach.z = -n.z();
    out.fitted_normal = true;
    c.metadata.push_back(kv("approach", "-normal"));
  }
  else
  {
    if (!cfg.default_approach)
    {
      out.problem = "the detection has no fitted surface normal (identity orientation) and no default approach "
                    "is configured: the direction the tool travels is unknown";
      return out;
    }
    const Eigen::Vector3d a = *cfg.default_approach;
    const double an = a.norm();
    if (!std::isfinite(an) || std::abs(an - 1.0) > kUnitTolerance)
    {
      out.problem = "the configured default approach is not a unit vector (norm " + num(an) + ")";
      return out;
    }
    c.approach.x = a.x() / an;
    c.approach.y = a.y() / an;
    c.approach.z = a.z() / an;
    c.metadata.push_back(kv("approach", "default (no fitted normal)"));
  }
  out.problem = candidateProblem(c);
  return out;
}

std::optional<Eigen::Quaterniond> minimalRotation(const Eigen::Vector3d& z, const Eigen::Vector3d& a)
{
  const Eigen::Vector3d u = z.normalized(), v = a.normalized();
  const Eigen::Vector3d axis = u.cross(v);
  const double s = axis.norm(), c = u.dot(v);
  if (s < kAntiparallelSine)
  {
    if (c > 0.0) return Eigen::Quaterniond::Identity();
    return std::nullopt;   // antiparallel: every axis perpendicular to z is a "minimal" one
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(std::atan2(s, c), axis / s));
}

std::optional<Eigen::Quaterniond> alignApproach(const Eigen::Quaterniond& reference, const Eigen::Vector3d& approach)
{
  const Eigen::Quaterniond R = reference.normalized();
  const auto m = minimalRotation(R * Eigen::Vector3d::UnitZ(), approach);
  if (!m) return std::nullopt;
  return (*m * R).normalized();
}

}  // namespace trainit
