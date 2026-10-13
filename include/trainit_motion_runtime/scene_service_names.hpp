// =============================================================================
// scene_service_names.hpp — the planning-scene service names scene_manager_node
// talks to, and the one rule that lets a namespaced instance work (ADR-0015
// Decision 3).
//
// The names are RELATIVE by default, so rclcpp resolves them against the NODE's
// own namespace:
//
//   node in /         -> /apply_planning_scene     byte-identical to the absolute
//                                                  name that was hardcoded here,
//                                                  so every generated bundle keeps
//                                                  working untouched;
//   node in /r3       -> /r3/apply_planning_scene  a namespaced cell's move_group — and four
//                                                  robots in four namespaces stop
//                                                  writing to one global service.
//
// A parameter overrides either name. An ABSOLUTE override ("/apply_planning_scene")
// reaches a move_group in a different namespace than the scene manager's own.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__SCENE_SERVICE_NAMES_HPP_
#define TRAINIT_MOTION_RUNTIME__SCENE_SERVICE_NAMES_HPP_

#include <string>

namespace trainit_scene
{

// Relative on purpose — see the header comment.
inline constexpr const char* kApplyPlanningSceneServiceDefault = "apply_planning_scene";
inline constexpr const char* kGetPlanningSceneServiceDefault = "get_planning_scene";

// The scene manager's OWN services, as a behaviour-tree node asks for them (the
// `service` port defaults of SetAttachedCollisionCheck and ResetScene). Relative for the
// same reason: a tree runs in its robot's namespace (ADR-0016 Decision 3), so from /r3
// these reach /r3/scene_manager_node/..., and from the root /scene_manager_node/... --
// byte for byte the absolute defaults every bundle used. Those absolute defaults asked
// the ROOT from /r3, found nothing, and the nodes reported success (vNext M0C #5).
inline constexpr const char* kAttachedCollisionCheckServiceDefault =
  "scene_manager_node/attached_collision_check";
inline constexpr const char* kResetSceneServiceDefault = "scene_manager_node/reset_scene";

// Parameter names of the explicit overrides.
inline constexpr const char* kApplyPlanningSceneServiceParam = "apply_planning_scene_service";
inline constexpr const char* kGetPlanningSceneServiceParam = "get_planning_scene_service";

// The name to create the client with: the override when the project set one, else
// the relative default. Whitespace is trimmed, so `apply_planning_scene_service: ""`
// (or "   ") means "use the default" rather than an empty, unresolvable name.
inline std::string resolveSceneServiceName(const std::string& override_name,
                                           const char* default_relative)
{
  const auto first = override_name.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) return default_relative;
  const auto last = override_name.find_last_not_of(" \t\n\r");
  return override_name.substr(first, last - first + 1);
}

}  // namespace trainit_scene

#endif  // TRAINIT_MOTION_RUNTIME__SCENE_SERVICE_NAMES_HPP_
