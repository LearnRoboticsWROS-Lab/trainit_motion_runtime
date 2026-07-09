// =============================================================================
// task_loader.hpp — build a MotionTask from a YAML file.
//
// The YAML is the application data a setup assistant would generate. The runtime
// stays application-agnostic: it loads and executes whatever the file describes.
//
// Schema (see config examples shipped with an application package):
//   name: <task name>
//   group / base_frame / tip_link: <optional overrides>
//   orientation: [x, y, z, w]            # default TCP orientation for pose steps
//   defaults: { velocity_scaling, acceleration_scaling, planning_time, planning_attempts }
//   scene:  [ { id, frame, dims:[x,y,z], position:[x,y,z] }, ... ]
//   steps:  [ { name, motion: free|ptp|linear|circular, planner: pilz|ompl|ompl_chomp,
//               position:[x,y,z], orientation:[x,y,z,w], named:<state>, joints:[...],
//               circ_interim:[x,y,z] | circ_center:[x,y,z],
//               eef: { gripper:<0..1> } | { process:{type,channel,value} },
//               attach:{id,link,dims:[x,y,z],position:[x,y,z]} | detach:{id,link} }, ... ]
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__TASK_LOADER_HPP_
#define TRAINIT_MOTION_RUNTIME__TASK_LOADER_HPP_

#include <string>

#include "trainit_motion_runtime/motion_task.hpp"

namespace trainit
{

class TaskLoader
{
public:
  // Throws std::runtime_error on parse error / missing required fields.
  static MotionTask fromYamlFile(const std::string& path);
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__TASK_LOADER_HPP_
