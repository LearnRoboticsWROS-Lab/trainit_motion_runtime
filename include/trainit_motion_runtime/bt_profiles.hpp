// =============================================================================
// bt_profiles.hpp — named motion profiles for BT nodes.
//
// A profile name (e.g. "transit", "approach", "process") maps to a MotionOptions
// (vel/acc scaling, planning time, tolerances). BT nodes take a `profile` port;
// the runner loads the registry from YAML once. Mirrors the reference's
// MotionProfileRegistry but produces OUR MotionOptions.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__BT_PROFILES_HPP_
#define TRAINIT_MOTION_RUNTIME__BT_PROFILES_HPP_

#include <string>
#include <unordered_map>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

using MotionProfileRegistry = std::unordered_map<std::string, MotionOptions>;

// Load profiles from a YAML file (root key `motion_profiles:`). Returns a registry
// that always contains a "default" entry. Missing file -> just the built-in default.
MotionProfileRegistry loadMotionProfiles(const std::string& yaml_path);

// Resolve a profile name -> MotionOptions, falling back to "default", then to
// MotionOptions{} (never throws).
MotionOptions resolveProfile(const MotionProfileRegistry& registry, const std::string& name);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__BT_PROFILES_HPP_
