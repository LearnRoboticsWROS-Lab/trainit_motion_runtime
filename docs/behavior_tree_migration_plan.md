# Behavior Tree migration plan

**Status: integrated.** BehaviorTree.CPP v4 is a dependency, the nodes live in
`src/bt_nodes.cpp` and `trainit_run_bt` executes a tree. This file stays as the map
from the BT **vocabulary** to the runtime call behind each node; the node model with
every port and default is `config/bt/trainit_nodes.groot.xml`.

## Operations → BT nodes
Each maps to a `MotionRuntime` / controller call already implemented:

| BT node | Backing call |
|---|---|
| `MovePtp` | `MotionRuntime::movePtp` / `moveCollisionFree` (the planner mode; Pilz PTP by default) |
| `MoveLinear` | `MotionRuntime::moveLinear` |
| `MoveCircular` | `MotionRuntime::moveCircular` |
| `MoveCollisionFree` | `MotionRuntime::moveCollisionFree` |
| `ExecuteProcessPath` | `executeCartesianProcess` / `executeProcessSequence` |
| `EnableProcess` / `DisableProcess` | `ProcessController::executeEvent` |
| `OpenGripper` / `CloseGripper` | `GripperController::command` |
| `AttachObject` / `DetachObject` | `PlanningSceneManager::attachBox/detachObject` |
| `ServoTrack` | an OUT-OF-PROCESS `moveit_servo` node — ADR-0015, see below |
| `DetectObject` / `SetWaypointFromDetection` | no runtime call at all: the `vision_msgs/Detection3DArray` contract → blackboard waypoints — ADR-0016, see below |

## `ServoTrack` — `execution: servo` (ADR-0015)

The one node that does not PLAN through `MotionRuntime`: it streams a cartesian
velocity to a separate `moveit_servo` node instead of planning a trajectory (it reads the
current TCP through `MotionRuntime::currentTipPose()`, since TMR 1.9.0). That is
ADR-0015 Decision 5 — `moveit_servo` 2.5.10 on Humble is a *node*, not a library, so
the node is a `std_srvs/Trigger` client plus a `TwistStamped` publisher and
`moveit_servo` stays off the Community library's link line. `execution: plan`, the
default, emits nothing and changes nothing.

It takes the SAME single `waypoint` port as `MoveWaypoint` and reads the same
`<wp>.type | .position | .orientation | .frame` blackboard keys, so a waypoint whose
pose is written at run time by vision (or by a relative binding) needs no second
representation. The orientation is tracked only when the waypoint carries one.

| port | default | meaning |
|---|---|---|
| `waypoint` | — | the waypoint name, as `MoveWaypoint` takes it |
| `servo_node` | `servo_node` | **relative**: a tree under `/r3` drives `/r3/servo_node/*`. An absolute value reaches another namespace |
| `command_frame` | *the current-pose frame* | the twist's `header.frame_id`; must be servo's `planning_frame`, and a LINK of the robot model |
| `command_in_type` | `speed_units` | must match the servo node. `speed_units` is what the TSA's generated `servo.yaml` sets, so a velocity means the same on every cell |
| `linear_scale` / `angular_scale` | `0.4` / `0.8` | used ONLY in `unitless` mode, where the command is divided by them so the caps below still mean m/s and rad/s |
| `tolerance_m` | `0.002` | one servo cycle at the speed cap travels `0.05 * 0.034 = 1.7 mm` (0.5 mm with the generated `publish_period: 0.01`), so a tighter tolerance cannot be hit reliably — the loop would chatter through it |
| `tolerance_deg` | `1.0` | same arithmetic: `0.30 * 0.034 = 0.58 deg` per cycle |
| `timeout_ms` | `5000` | also the excursion bound: `5 s * 0.05 m/s = 0.25 m` of streamed travel, then FAILURE |
| `rate_hz` | `50` | 20 ms between commands: a 10x margin on the generated `incoming_command_timeout` of 0.2 s (5x on the moveit_servo default 0.1 s) |
| `max_linear_vel` / `max_angular_vel` | `0.05` m/s / `0.30` rad/s | conservative caps for a streamed correction |
| `gain` | `2.0` 1/s | `speed = min(gain * error, cap)`: decelerates inside 25 mm |
| `execution_status_topic` | *none: the watch is OFF* | the arbitration watch, e.g. `execute_trajectory/_action/status`. The 2-argument port declaration carries that text as a DESCRIPTION, not a default (BT.CPP v4), so the watch runs only when the port is set (KNOWN_DEBT) |
| `start_servo` | `true` | call `start_servo` once per process |

Three traps are load-bearing, and all three are in the node:

1. **A fresh `header.stamp` on EVERY message.** `twistStampedCB` only records the
   stamp when it is non-zero, and the servo loop treats a command older than
   `incoming_command_timeout` (0.1 s) as stale and halts. A zero or reused stamp
   therefore never moves the robot **and produces no error at all**.
2. **One owner per controller.** `JointTrajectoryController` accepts the servo topic
   while an action goal is in flight and the two fight silently. With
   `execution_status_topic` set, `ServoTrack` refuses to stream while a trajectory
   execution is in flight; it always pauses servo on EVERY
   exit path (success, failure, timeout, bad status). `start_servo` is called once per
   process — it joins and respawns the calc thread and waits up to 10 s for a
   complete robot state; per-motion ownership is `pause`/`unpause`.
3. **A missing servo node is FATAL here**, unlike the planning-scene flag nodes: a
   missing flag service is a no-op, a missing servo node means the move did not
   happen, and reporting SUCCESS for a move that never ran is the silent wrong move
   this ADR exists to remove. Only the clean-up pause is non-fatal.

`servo_node/status` (`std_msgs/Int8`) is surfaced as a readable failure: **2** =
singularity halt, **4** = collision halt, **5** = joint bound. 1/3/6 (decelerating)
are warnings and the stream continues.

## The candidate LIST — `keep` and `index` (ADR-0016)

Bin picking is nothing but retrying: measured on a real bin-picking twin over a 25-capsule bin,
**3,33 candidates are tried per capsule delivered**. Until ADR-0016 that could not be
written in a tree at all, because `DetectObject` wrote exactly one pose. Two ports fix
it, and nothing else was added:

| port | node | default | meaning |
|---|---|---|---|
| `keep` | `DetectObject` | `1` | how many candidates to publish onto the blackboard |
| `index` | `SetWaypointFromDetection` | `0` | which candidate the waypoint is taken from |

**The published ORDER is the ranking, and the runtime does not re-rank.** ADR-0016
Decision 1: the detector sorts by its configured rule before publishing, so
`keep = N` writes the first N *matches in publication order*. It does not look at any
ranking term — `score`, `pose_quality`, `visible_fraction` are the detector's
vocabulary, and the perception contract exists so a runtime never learns it. What the
runtime cannot check is whether the detector *did* sort: `keep > 1` hands the tree the
order it was given, whatever that order is — and TP's side of Decision 1
(`seg_pose_node` sorting by its `candidate_rule` before publishing, instead of only
highlighting the pick in the overlay) is tracked in that ADR, not here. The same rule
holds at `keep = 1`; only the keys it writes are the compatibility path:

| `keep` | written | chosen by |
|---|---|---|
| `1` (default) | the FLAT `<out_key>.position \| .orientation \| .score \| .class_id \| .frame \| .size` — the layout every bundle generated before ADR-0016 relies on | the FIRST match, in PUBLISHED order (until vNext M0C #4 the single highest-scoring one: a re-rank that overrode the detector's ranking) |
| `N > 1` | `<out_key>.<i>.*` for `i = 0..count-1`, plus `<out_key>.count` | the first N matches, in PUBLISHED order |

`index = 0` reads the indexed keys when they exist and the flat ones when they do not,
which is why a tree written before this change means exactly what it meant before. A
**non-zero** `index` requires the indexed keys and FAILS without them: falling back to
the flat single candidate would make a retry loop re-try the *same* candidate, which is
the failure the list exists to remove. `.count` is written as a string like every other
key here, and an integer port binds it anyway (`num_attempts="{detected.count}"`):
BT.CPP converts a string entry to the port's number, and converts this node's string
into an entry a port already declared `int`.

At `keep = 1` there is deliberately **no** `.count` and no `.0.*`: the blackboard looks
exactly as it always has. A generated tree therefore relies on the `index = 0` fallback
rather than on `.count` when the retry budget is 1.

Nothing else is needed for the loop — BehaviorTree.CPP v4 already has
`RetryUntilSuccessful`, `Fallback` and `Repeat`, so "try the next candidate" is a
decorator over an index that advances:

```xml
<Script code="i := -1"/>
<DetectObject detector="capsule" keep="3" out_key="detected.capsule"/>
<RetryUntilSuccessful num_attempts="3">
  <Sequence>
    <Script code="i := i + 1"/>     <!-- advances on EVERY attempt, not only on success -->
    <SetWaypointFromDetection waypoint="pick" from="detected.capsule" index="{i}" orientation="detected"/>
    <MoveWaypoint waypoint="pick"/>
    <CloseGripper/>
    <MoveWaypoint waypoint="post_pick"/>
    <CheckRobotState .../>          <!-- did the capsule actually come up -->
  </Sequence>
</RetryUntilSuccessful>
```

The increment belongs at the TOP of the attempt: `RetryUntilSuccessful` re-ticks its
child unchanged, so an index advanced after the move would re-try the same candidate
for ever.

`keep` and `index` are **integer** ports, unlike every other port in this runtime, and
that is load-bearing: a counter reaches the tree as an int blackboard entry (`Script
code="i := 0"` writes one), and a *string* port refuses to bind an entry already typed
int — `createTree()` throws *"the port [i] was initially created with type [int]"*. An
int port binds an int entry, a string entry and a literal (`index="2"`) alike. The one
shape that fails either way is a counter pre-set as a **string**, which `Script` then
cannot assign an int to.

## Node contract (already satisfied)
Every operation: explicit typed inputs, a typed `MotionResult`, no global state,
no hidden side effects, does not block indefinitely (action timeouts). Adding
cancellation is the main BT-specific addition.

## Migration steps (when ready)
1. Add `behaviortree_cpp` dependency to the **application** package (not the engine).
2. Wrap each call above in a `BT::SyncActionNode` / `StatefulActionNode`
   (async + cancellable), reading ports for targets/options.
3. Provide blackboard entries for named poses / paths (load from the same task
   YAML, or from the BT XML).
4. Author the tree (XML). Example:
```xml
<Sequence name="DispensingTask">
  <MoveCollisionFree target="{safe_start}"/>
  <MoveLinear        target="{process_start}"/>
  <EnableProcess     channel="glue"/>
  <ExecuteProcessPath path="{glue_path}"/>
  <DisableProcess    channel="glue"/>
  <MoveLinear        target="{safe_start}"/>
  <MoveCollisionFree target="{home}"/>
</Sequence>
```
The engine stays unchanged; the BT lives in the application bundle, alongside (or
instead of) the linear task YAML.
