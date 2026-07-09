# TrainIt Motion Runtime — architecture

A robot- and **application-agnostic** motion engine on top of MoveIt 2 (Humble,
2.5.9). It is the "motor" that future application bundles plug into; a setup
assistant will generate those bundles, the engine executes them.

## Layered design
```
Application bundle (DATA: a MotionTask YAML)   <-- generated, no engine code
        |
trainit_run_task  (generic executable: load YAML -> run)
        |
TaskRunner  ->  MotionRuntime (semantic API: movePtp/Linear/Circular/
        |                      moveCollisionFree/executeProcessSequence/
        |                      executeCartesianProcess)
        |
MotionStrategySelector  (intent + planner_mode -> {pipeline, planner})
        |
+-----------------------+----------------------------+-------------------+
| Discrete PTP/LIN/CIRC | Free-space OMPL/CHOMP/STOMP| Process path       |
|       (Pilz)          |                            | Pilz seq / cart wp |
+-----------------------+----------------------------+-------------------+
        |
TrajectoryValidator -> execution (FollowJointTrajectory / ros2_control / Isaac)
```

## What lives WHERE (the key principle)
- **`trainit_motion_runtime` (this package)** — the engine. NO application type,
  NO poses, NO robot name. Provides: the motion API + MoveIt-backed
  implementation, the planner selector, the process-path backends, generators &
  loaders, validator, generic gripper / process controllers, a generic task
  model + runner, the generic planning pipelines (ompl/pilz/chomp), and the
  launch helper `trainit_moveit_config.py` that assembles `move_group` params.
- **An application bundle (e.g. `fr3wml_app/`)** — robot + application data:
  the robot description, the robot MoveIt config (which *pulls* the generic
  pipelines from this framework), and the application **task YAML** + launch.
  No bespoke C++.

So an "application" is a YAML `MotionTask`, not code. `trainit_run_task` runs any
of them. This is what a future `trainit_setup_assistant` will emit.

## The semantic API (no MoveIt leakage)
`MotionRuntime` (motion_runtime.hpp) is the only surface the application/BT sees.
It NEVER exposes `setPlannerId`, `computeCartesianPath`, etc. — those are confined
to `MoveItMotionRuntime` and the backends. Methods: `movePtp`, `moveLinear`,
`moveCircular`, `moveCollisionFree`, `executeProcessSequence`,
`executeCartesianProcess`, `executeTrajectory`, `stop`, `setPlannerMode`.

## Strategy selection (Sections 5-6, 14-15)
`ManualMotionStrategySelector` maps `(MotionIntent, planner_mode)` to a
`PlannerProfile{pipeline_id, planner_id}`:
- **LINEAR / CIRCULAR** → always Pilz `LIN` / `CIRC` (deterministic MoveL/MoveC),
  regardless of mode. Approach/retreat stay linear.
- **Free-space transfer (PTP / COLLISION_FREE)** follows the mode:
  - `pilz` → Pilz `PTP`
  - `ompl` → OMPL `RRTConnectkConfigDefault`
  - `ompl_chomp` → two-stage (see below)
  - `stomp` → guarded: degrades to OMPL with a warning (STOMP not installed)
A per-step `planner_override` lets a task choose the planner segment-by-segment.

## planner_mode = ompl_chomp (MoveIt 2.5.9 reality)
MoveIt 2.5.9 has **no CHOMP request-adapter** and `MotionPlanRequest` has **no
`reference_trajectories` field**, so OMPL cannot *seed* CHOMP. `OmplChompStrategy`
therefore: **plans with the `chomp` pipeline; on failure falls back to OMPL.**
CHOMP is genuinely loaded and executed (visible in the `move_group` log); the
executed pipeline is reported in the metrics. True OMPL-seeded CHOMP requires a
newer MoveIt or a custom PlanningPipeline.

## MotionTask data model (motion_task.hpp)
A task = optional scene (collision boxes) + ordered steps. A step has: a motion
type (`free|ptp|linear|circular|process_path`), a target (pose / named state /
joints), an optional per-step planner, an optional end-effector action (gripper
position or a process event), an optional payload attach/detach, and — for
`process_path` — a `ProcessPathSpec` (generator or file). See `task_loader.hpp`
for the YAML schema and `config/tasks/*.yaml` in an application package.

## Validation & metrics
`TrajectoryValidator` runs real checks before execution (non-empty, finite,
monotonic timestamps, joint limits, joint-jump, and collision via a
PlanningScene snapshot when available). Free-space metrics (Section 20) and
process-path metrics (Section 21, FK-based deviation) are collected and logged.

## Execution & robot adapter
Trajectories execute through `FollowJointTrajectory` (ros2_control), the same in
mock / Isaac / real. A future `RobotMotionAdapter` (robot_motion_adapter.hpp)
will allow a vendor-native backend without touching the engine.

## Not yet (by design)
- Online servoing: `OnlineServoBackend` is an interface + docs only (MoveIt Servo).
- STOMP: guarded until `moveit_planners_stomp` is installed.
- BehaviorTree.CPP: not integrated; see `behavior_tree_migration_plan.md`.
