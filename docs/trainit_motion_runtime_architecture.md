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
`MotionRuntime` (motion_runtime.hpp) is the TrainIt door to MoveIt for every planned move
(ADR-0015 and its vNext M2.5 amendment). It NEVER exposes `setPlannerId`,
`computeCartesianPath`, etc. — those are confined to `MoveItMotionRuntime` and the
backends — and its header names no MoveIt C++ API, only message types
(`test_m2_5_interface_purity` compiles it alone and implements it with a fake). Methods:
`movePtp`, `moveLinear`, `moveCircular`, `moveCollisionFree`, `executeProcessSequence`,
`executeCartesianProcess`, `executeTrajectory`, `stop`, `setPlannerMode`, `plannerMode`,
since TMR 1.9.0 `currentTipPose` — the robot-state read both the planned path and
ServoTrack use (the current TCP pose, stamped in the robot model frame) — and, since TMR 2.1.0
(vNext M2.7, owner decision 11.2), `tipLink`, `baseFrame` and `namedPose`: the frames and the
named-state FK the BT nodes and TaskRunner read. `BtContext::runtime` is a `MotionRuntime*` and
`TaskRunner` takes a `MotionRuntime&`. 2.1.0 is a minor number, but two of these changes break
source compatibility with 2.0.x: an implementer of `MotionRuntime` must add the three methods,
and code that reads `scene()`, `group()` or `node()` (members of `MoveItMotionRuntime` only)
through `BtContext::runtime` no longer compiles; the scene is `BtContext::scene`. `BtContext`,
`MotionResult` and `ValidationReport` also changed layout, so a package built against 2.0.x is
rebuilt. No implementer and no `BtContext` user exists outside TMR in the workspaces searched
`[static]`, and TaskRunner's 2.0.0 constructor stays (ADR-0015 M2.7-8).

What does NOT go through `MotionRuntime`, and why (M2.5, classes B/C/D):
- the planning-scene WRITES (a different contract from motion): the runner's
  `PlanningSceneManager` (`MoveItMotionRuntime::scene()`) is handed to the BT nodes as
  `BtContext::scene` and to `TaskRunner` as its `scene` argument, since 2.1.0. The four scene
  nodes (`test_m2_7_scene_nodes`, frozen on 2.0.0) and TaskRunner's fixtures and payloads
  (`task_runner.txt`) write it. The concrete runtime is named only by the composition roots (the
  two runners build it) and by TaskRunner's 2.0.0 constructor `TaskRunner(MoveItMotionRuntime&,
  …)`, kept as an overload that delegates (its removal is a 3.0.0 change);
- `ServoTrack`'s streaming: a ROS client of an out-of-process `moveit_servo` node
  (ADR-0015 Decision 5); only its state reads go through the door;
- the raw MoveGroupInterface: `moveGroup()` was deprecated in 1.9.0 and removed in vNext M2.6
  (no caller in any workspace). `bt_nodes.cpp`, `task_runner.cpp` and the runners are built with
  `-Werror=deprecated-declarations`, and `test_m2_5_seam_scan` counts a fixed list of
  MoveIt names, MoveIt includes and MoveIt endpoint names in those files and in the MoveIt-free
  sources (expected: none), checks that the compiler barrier covers exactly those consumers, and
  fails on any source file nobody classified (consumer, MoveIt-free, or a B/C exception).

What every motion consumer sends to MoveIt is frozen by the M2.5 tests (`test/m2_5/`, captures in
`test/data/m2_5/`, recorded on 1.8.0 before the seam moved): any change to a goal, a request,
their order, a BT result or the planner mode left behind fails a byte-for-byte comparison.

## Strategy selection (Sections 5-6, 14-15)
`ManualMotionStrategySelector` maps `(MotionIntent, planner_mode)` to a
`PlannerProfile{pipeline_id, planner_id}`:
- **LINEAR / CIRCULAR** → always Pilz `LIN` / `CIRC` (deterministic MoveL/MoveC),
  regardless of mode. Approach/retreat stay linear.
- **Free-space transfer (PTP / COLLISION_FREE)** follows the mode:
  - `pilz` → Pilz `PTP`
  - `ompl` → OMPL `RRTConnectkConfigDefault`
  - `ompl_chomp` → two-stage (see below)
  - `stomp` → declared, never loaded (MoveIt on Humble has no STOMP plugin): degrades
    to OMPL **silently** — the runner logs the requested mode (recorded debt)
A per-step `planner_override` lets a task choose the planner segment-by-segment.

The vocabulary — the modes, what each resolves to, the pipelines move_group loads,
the defaults — is the planner registry `config/planner_profiles.yaml` (ADR-0021):
the launch helper reads it, TSA keeps a byte copy of it, and `test_planner_registry`
checks this C++ against it; `test_planner_resolution_identity` pins the resolution.

## planner_mode = ompl_chomp (MoveIt 2.5 reality)
MoveIt 2.5 ships **no CHOMP optimizer request-adapter**, so the runtime does not
chain OMPL into CHOMP (whether CHOMP would read `MotionPlanRequest.reference_trajectories`
as a seed has not been tested here). `MoveItMotionRuntime::planAndExecute` therefore
**plans with the `chomp` pipeline first, standalone; on failure it falls back to OMPL.**
CHOMP is genuinely loaded and executed (visible in the `move_group` log); the
executed pipeline is reported in the metrics. OMPL-seeded CHOMP would need a CHOMP
request adapter (a newer MoveIt) or a custom planning pipeline.

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

## Online servoing (ADR-0015)
`execution: servo` is a PER-MOVE field on its own axis, orthogonal to `motion`
(`ptp|lin|free|circ`) and to `PlannerMode`; the default `plan` changes nothing. It is
executed by the `ServoTrack` BT node, which streams a `TwistStamped` to an
**out-of-process** `moveit_servo` node and owns the controller while it does (start
once, pause/unpause per motion, refuse to stream while a trajectory execution is in
flight). `OnlineServoBackend` is the rejected shape, kept only as a record:
`moveit_servo` 2.5.10 is a node, not a library, so TMR does not link it. Ports,
defaults and the three traps: `behavior_tree_migration_plan.md`.

## The end-effector contract (ADR-0022)
Since TMR 2.0.0 a project may carry an `eef` parameter block (TSA emits it into `bt_params`):
devices, each with actuators on ONE standard command interface apiece (`std_srvs/SetBool`,
`control_msgs` `GripperCommand` or `FollowJointTrajectory`), grasp sites (one grip actuator each)
and a verification policy. Without the block every legacy path runs exactly as before. What a
legacy project can see (ADR-0022 D9): `GripperController`'s WARN when a result is not SUCCEEDED
(EEF-D3); ONE INFO line per run, `end-effector contract: LEGACY (ADR-0022)`; the new optional ports
of `CloseGripper`/`OpenGripper` (`eef`, `site`, `reason`, `fault`, `subject`) and `DetectObject`'s
`fault`, never remapped or written on the legacy branch but listed in Groot and in the node
manifests; and an `eef.*` parameter-override probe that finds nothing.
- `end_effector_contract` parses the block from the node's parameter OVERRIDES, exactly as the
  reference normaliser does (`test/data/m2_6_contract/`); a malformed block exits the runner with
  code 2, naming the key.
- `EndEffectorRegistry` holds one `EndEffectorDevice` per device: its standard clients, its
  `eef/<id>/state` subscription (`trainit_interfaces/EndEffectorState`), the `eef/<id>/engaged`
  scene signal when the block says `scene_signal: tmr`, and a fault latch. Every name is relative,
  so a runner in `/r1` reaches `/r1`'s adapter and controllers only.
- `CloseGripper`/`OpenGripper` (ports `eef`, `site`), `CommandActuator` (`actuator`, a NAMED
  `target`) and `VerifyGrasp` (`expect`, optional `site`, `timeout_ms`, `max_age_ms`, `accept`)
  report `reason`, `subject` and, unless the reason is a VERDICT, `fault`, on FAILURE only.
  Command success is ACTUATION (`actuator_outcome`: the D2 table, a ros2_control GripperCommand
  goal is never cancelled); a grasp is only ever judged by `VerifyGrasp` (`verify_grasp`: the D4
  check order, the reference rule on the adapter's clock, the deadline precedence, the
  tool-level rules for a multi-site device). A command is DONE only once the device's own state
  shows it (every `set_bool` actuator; a GripperCommand or FollowJointTrajectory actuator while a
  state publisher is visible): no sample at all is `STATE_MISSING`, no new sample since the
  command `STATE_STALE`, new samples none of which shows it `CONTRACT_MISMATCH`.
- Every wait runs on the steady clock and ends `COMMAND_ABORTED` ("interrupted: the ROS context
  shut down") once the context shuts down; a canonical tree that throws is halted and the run
  fails (exit 1).
- The runner refuses a tree whose literal ports the contract does not know, `SetGripper`, and two
  writers on a device's state topic (exit 2), and exits with code 3 when the tree ends SUCCESS
  while an end-effector FAULT was latched, whoever wrote the tree (the exit codes below).
  `DetectObject` gained an optional `fault` output (a silent detector or a failed TF lookup) for
  canonical bin picking's gate.

## Whose failure a failed motion is: MoveWaypoint's `fault` (vNext M2.7, owner Q8.2)
A failed move is either the TARGET's (this target cannot be reached by this motion: no IK, no valid
plan, the goal in collision, a joint jump in the plan; another target may work) or a FAULT
(anything that would recur for every target: a missing server, a rejected goal, a transport or
execution timeout, any execution failure, the robot's start state, the configuration, an
exception, a code MoveIt does not name). The rule is default-deny: a failure is a FAULT unless the
table names it the target's. The owner ratified this classification at vNext G0 (2026-10-13) as
the current public behaviour and the baseline M4.5 validates; it is not redesigned before then.
- `MotionResult` records the FACTS on the failure paths MoveWaypoint reaches, additively (nothing
  on success, no request changed): `phase` (`REQUEST`, `PLAN` for every retry, `VALIDATE`,
  `EXECUTE`, `PLAN_AND_EXECUTE` for the one Pilz sequence goal) and, when MoveIt's error code is not
  the cause, a `cause` token: the Pilz transport outcomes (`SERVER_UNAVAILABLE`, `GOAL_SEND_TIMEOUT`,
  `GOAL_REJECTED`, `RESULT_TIMEOUT`), the validator check that refused the plan
  (`validationCause`, `trajectory_validator.hpp`; for a collision the validator's report keeps the
  first colliding waypoint), MoveWaypoint's own refusals, `EXCEPTION`.
- `motion_failure.hpp` holds the ONE table, pure (`classifyMotionFailure`): TARGET only for
  `PLANNING_FAILED`, `INVALID_MOTION_PLAN`, `GOAL_IN_COLLISION`, `GOAL_VIOLATES_PATH_CONSTRAINTS`,
  `GOAL_CONSTRAINTS_VIOLATED`, `GOAL_STATE_INVALID` and `NO_IK_SOLUTION` in a planning answer (`PLAN`
  or `PLAN_AND_EXECUTE`), `TIMED_OUT` in `PLAN` (the planner's own budget for this goal), and the
  validator's `JOINT_JUMP` and `COLLISION` (a collision first met at a waypoint after the start).
  Everything else is a FAULT, `START_STATE_*` included: the start is the robot's current state, the
  same for every candidate. That holds for the validator too: a collision at waypoint 0 is
  `START_STATE_COLLISION`, a FAULT, because MoveIt plans from a colliding start without refusing it
  (move_group accepts a path whose only invalid waypoint is the start, Pilz never checks the start
  for collision, and FixStartStateCollision puts the colliding start back as waypoint 0), so on the
  Pilz moves a robot that starts in collision reaches TMR as a plan whose waypoint 0 collides, not as
  `START_STATE_IN_COLLISION`. `motionFaultText` writes a FAULT as `MOTION_FAULT: <CAUSE>[ during
  <phase>]` (`test_m2_7_motion_failure`, which also runs the validator on a real model and scene).

  | Phase | TARGET (class A) | FAULT (class B) |
  |---|---|---|
  | `PLAN` | `PLANNING_FAILED`, `INVALID_MOTION_PLAN`, `GOAL_IN_COLLISION`, `GOAL_VIOLATES_PATH_CONSTRAINTS`, `GOAL_CONSTRAINTS_VIOLATED`, `GOAL_STATE_INVALID`, `NO_IK_SOLUTION`, `TIMED_OUT` | every other code (`START_STATE_*`, `FAILURE`, 0, a code MoveIt does not name, a failed result carrying `SUCCESS`) |
  | `PLAN_AND_EXECUTE` | the same but `TIMED_OUT` | every other code; the Pilz transport causes |
  | `VALIDATE` | `JOINT_JUMP`; `COLLISION` (first colliding waypoint k > 0) | `START_STATE_COLLISION` (waypoint 0), `EMPTY_TRAJECTORY`, `NON_FINITE`, `JOINT_LIMITS`, `NON_MONOTONIC_TIME`, `UNKNOWN_GROUP`, `VALIDATION_FAILED` |
  | `EXECUTE`, `REQUEST`, none | none | all |
- `MoveWaypoint` has an optional output `fault`, written ONLY for a FAULT (never on SUCCESS, never
  for the target's failure), with one ERROR line naming the waypoint. The write never throws out of
  the tick: if the remapped key already holds a value of another type (BT.CPP refuses the write by
  throwing), the fault is not written, one ERROR line says so, and the node returns FAILURE. It is
  declared without a default: a tree that does not remap it writes nothing and logs nothing new,
  and MoveIt receives exactly the same requests either way (`test_m2_7_motion_fault`, every M2.5
  capture). The canonical bin-picking cycle remaps it to its gate key `eef_fault` on every
  MoveWaypoint inside the absorbing Fallback (TSA 7.8.0 on): a FAULT ends the run (exit 1) and
  never reads as the next candidate or as "bin empty", while the target's failure keeps the
  existing strategy (`test_m2_7_bin_picking_motion`: the BEFORE and AFTER captures and their exact
  difference, `test/data/m2_7_mf/`). A motion FAULT is not an end-effector latch: exit code 3 stays
  the contract's.
- Not reported: ServoTrack (no `fault` port), and the paths MoveWaypoint does not take (the
  cartesian process path, `executeTrajectory`, the MovePtp/MoveToNamed/MoveToJoint nodes), whose
  unattributed failures would be FAULTs by default. A tree that remaps the port needs a runtime that
  declares it, TMR 2.1.0 or later (the floor a generator must write for such a package): an older
  one refuses the tree at load (exit 2).

## The runner start-up: this robot's MoveIt, named and bounded (vNext M2.7, ADR-0023)
Before `trainit_run_bt` and `trainit_run_task` build the runtime (whose MoveGroupInterface waits
for its actions for ever, in silence), they check this robot's MoveIt wiring (ADR-0023 Decision 5).
The wait's defaults below are the current public behaviour, ratified by the owner at vNext G0
(2026-10-13) as the M4.5 baseline.
- **`moveit_wait_timeout_s`** (an integer or a double; default 60) bounds the wait for the four
  endpoints the runtime's constructor needs in the node's namespace: the actions `move_action` and
  `execute_trajectory` and the services `get_planning_scene` and `apply_planning_scene`. A finite
  value > 0 is the bound in seconds, on the steady clock (a stopped sim clock does not stop it).
  Exactly `-1` waits without a bound, as before vNext M2.7, and still WARNs. Any other value (0,
  another negative number, `nan`, `inf`, a string, a bool, an array) is a FATAL naming the value
  and its type, and exit 2.
- With MoveIt up the wait returns at once and prints nothing. Otherwise a WARN every 5 s names each
  endpoint still missing by its resolved name (`/r3/move_action` under `/r3`, never a root name),
  and an INFO line closes a wait that warned once every endpoint is served; at the bound a FATAL
  names the missing ones and the runner exits with code 2. Ctrl-C during the wait prints one INFO
  line (`interrupted while waiting for MoveIt: exit 1`) and exits 1.
- **The cross-namespace remap guard.** A runner in a namespace N refuses, at start-up and before
  any client exists, a remap that sends one of its robot's names out of N: one FATAL lists each as
  `service|topic <the name in N> -> <the remapped name>`, and the exit code is 2. The guard
  (`MoveItMotionRuntime::crossNamespaceRemaps`) checks, in both runners:
  - MoveGroupInterface's services `compute_cartesian_path`, `query_planner_interface`,
    `get_planner_params` and `set_planner_params`, and its publishers `trajectory_execution_event`
    and `attached_collision_object`;
  - the planning scene of the PlanningSceneInterface and the PlanningSceneMonitor:
    `get_planning_scene`, `apply_planning_scene`, `planning_scene`, `collision_object`,
    `planning_scene_world`, `attached_collision_object`; the monitor's scene publisher
    `monitored_planning_scene` (what RViz and MoveIt Servo subscribe to), which MoveIt itself
    starts when the `publish_planning_scene` parameter is true, as `build_move_group_params()`
    sets it for every generated runner; and the monitor's octomap services `save_map` and
    `load_map`, which the runner itself serves;
  - `joint_states` (both state monitors);
  - the scene manager's `scene_manager_node/attached_collision_check` and
    `scene_manager_node/reset_scene` (the `SetAttachedCollisionCheck` and `ResetScene` defaults);
  - the actions `move_action`, `execute_trajectory` and `sequence_move_group` (Pilz), each as its
    five entities `<action>/_action/send_goal`, `cancel_goal`, `get_result` (services), `feedback`
    and `status` (topics). Humble applies no remap of an action name, but it does apply one of each
    entity.

  `trainit_run_bt` checks, after those and once it has read the `eef` block:
  - ServoTrack's default servo node: `servo_node/delta_twist_cmds` and `servo_node/status`
    (topics), `servo_node/start_servo`, `servo_node/pause_servo` and `servo_node/unpause_servo`
    (services), whether or not the tree runs a ServoTrack;
  - the end-effector contract's names (`eefEndpoints()`), per device `eef/<id>/state`,
    `eef/<id>/engaged` when TMR publishes the scene signal, and per actuator
    `eef/<id>/actuators/<a>/set_engaged` (set_bool) or its action's five entities (gripper_command,
    follow_joint_trajectory).

  A remap that stays in N is not refused; one onto a root name is. Not checked, by design: the
  root, which does not change; the cell-global `/tf` and `/tf_static` (ADR-0023 Decision 4) and
  ROS's own global `/parameter_events`, `/rosout` and `/clock` (ADR-0023 Decision 5); each node's
  own parameter services; the names a tree passes
  in its ports (a servo node other than the default, a detector, an I/O channel, a scene-manager
  service, an attached topic); the LEGACY absolute endpoints as the project writes them (the
  gripper action, the grasp Bool, `/isaac_release_policy`); and `robot_description`, a parameter of
  every runner. At run time an end-effector state cross-remap still ends `WRONG_EEF` and a command
  one `CONTRACT_MISMATCH` (ADR-0022 D4, D5), for what the start-up check cannot see.
- **Exit codes**, all together: 0 the tree or task succeeded; 1 it failed, or Ctrl-C during the
  MoveIt wait; 2 a configuration error before the first move (a missing or invalid parameter,
  `moveit_wait_timeout_s` included; a cross-namespace remap; MoveIt not served at the bound; a tree
  or task file that does not load; an end-effector contract refusal); 3 (`trainit_run_bt` only)
  the tree ended SUCCESS with an end-effector FAULT latched. Exit 3 is the canonical execution's
  semantic (owner, vNext G0; ADR-0022 D5), keyed on the `eef` block, not on who wrote the tree: a
  run without the block never exits 3, and a tree run with it, generated or hand-written, exits 3
  when it ends SUCCESS after the registry latched a FAULT. A motion or `DetectObject` FAULT is
  never latched, so a tree that absorbs one ends 0. A LEGACY tree that throws still ends the
  process as it always has (ADR-0022 D9).
- **Groot2.** `groot_monitor` (true by default) starts BT.CPP's live monitor once the tree is
  loaded, on TCP `groot_port` (1667 by default) and the port after it. The ports are the host's,
  not the robot's: two runners on one host with the monitor on need pairs at least two apart
  (1667 and 1669, say), set as each runner's `groot_port` parameter, which no generated launch
  sets (it exposes `groot_monitor` only); with one pair the second runner would end in
  `std::terminate` `[static: trainit_run_bt_node.cpp, BT.CPP's Groot2Publisher; inferred, not
  run]`. vNext M2.7's multi-runner tests run with `groot_monitor` false. A port per runner is the
  cell level's (ADR-0020; KNOWN_DEBT).

## Not yet (by design)
- STOMP: declared in the registry and never loaded — there is no STOMP plugin for MoveIt
  on Humble, so it needs a port, not an install.
- Joint planning across robots: each robot keeps its own `robot_description` and
  `move_group`; the other arms and the cell furniture enter its planning scene as
  world collision objects (ADR-0015 Decision 3). `scene_manager_node` can be launched
  per namespace for exactly that.
