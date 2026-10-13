# vNext M2.6 captures: TODAY's end-effector behaviour, frozen BEFORE the canonical EEF contract

Each `<family>.txt` is the byte-exact record of what one family of legacy end-effector (EEF) paths
does against in-process fakes (`test/m2_6/`), per scenario: the trees ticked, every request the
fakes received (whole messages, block YAML, precision 17, stamps removed) in order, the BT results,
and ONE `semantic:` line. They were recorded on **TMR 1.9.0, production sources == 1dda503**
(policy runtime 85053b3 and TSA 351eb8c unchanged), before vNext M2.6 touched a production line
(ADR-0022, "Order of work and test first"). The `test_m2_6_*` binaries regenerate them into
`build/trainit_motion_runtime/m2_6_actual/` and compare byte for byte.

| Family (file) | Binary | Domain | What it freezes |
|---|---|---|---|
| E1 `eef_gripper_command` | `test_m2_6_gripper_command` | 97 | Close/Open/SetGripper through `GripperController`: every GripperCommand answer met today (SUCCEEDED, ABORTED with and without a result, stalled, CANCELED by a preemption, REJECTED, no server, late accept, withheld result), joint targets, a non-numeric `SetGripper` |
| E2 `eef_grasp_signal` | `test_m2_6_grasp_signal` | 97 | the latched grasp Bool on `gripper_cmd_topic`: when it is published, volatile/latched/late-joiner reception, two runners on one global name |
| E3 `eef_check_robot_state` | `test_m2_6_check_robot_state` | 97 | `CheckRobotState require_attached`: the exact-"true" switch, no topic, no publisher, true/false providers, a latched provider, the fresh-arrival rule |
| E4 `eef_release_policy` | `test_m2_6_release_policy` | 97 | `SetReleasePolicy`'s function-static publisher binding the first topic |
| E5 `eef_run_policy` | `test_m2_6_run_policy` | 97 | `RunPolicy` against a fake policy runtime: accepted (success, failure, timeout, still running), hybrid, refused, absent, late answer |
| E6 `eef_task_runner` | `test_m2_6_task_runner` | 97 | TaskRunner's GRIPPER step on the M2.5 rig: SUCCEEDED, ABORTED, REJECTED, no gripper |
| E7 `eef_runner_child` | `test_m2_6_runner_child` | 88 | the real `trainit_run_bt` as a child process: golden-style keys, ROS2ML-style keys, misspelt keys, `/r1` + `/r3`, a late joiner, no server |
| E8 `eef_scene_manager_child` | `test_m2_6_scene_manager_child` | 89 | the real `scene_manager_node` as a child process: the grasp Bool edges in `remove` and `attach_box` mode, an EMULATED attach refusal, a latched close before the scene loads, reset, two managers in two namespaces |

`eef_tree.xml` and `eef_tree_hold.xml` are E7's trees; `eef_task.yaml` is E6's task.

## The semantic line

`semantic: command sent=… | ack=… | state=… | grasp confirmation=… | BT result=… | timeout=… |
release result=…[ | result code=…]` (owner brief M2.6 section 13). Every field is COMPUTED from what
the fakes and recorders observed: the ScriptedGripper's goal records (which tick a goal arrived in,
whether its response and its result came before the client returned), the recorders' samples
(attributed to the publishing node through the graph), the scene requests, the graph (which Bool
topics a node subscribes to), and the BT results. Nothing in it is typed by hand. Stamps, goal ids,
wall-clock durations and PIDs are excluded; the only log line read is trainit_run_bt's
`BT finished:`.

## Harness choices and emulations (labelled in the captures)

- A preemption is a SECOND client's real cancel request (rcl_action forbids EXECUTING -> CANCELED
  from the server); the goal ends CANCELED with an empty result.
- The "shim" answer reproduces `fairino_gripper/gripper_suction_action_server.py`'s failing path:
  ABORTED, `reached_goal=false`, position 1.0/0.0 at its 0.5 threshold.
- E8's refused attach is an EMULATION: the fake answers `success=false` to a diff that ADDs an
  attached object.
- E7's params files add `groot_monitor: false` (no TCP port; two children would collide on 1667)
  and the inline M2.5 test arm. E8's add boxes only, `force_republish_hz: 0.0` and
  `scene_update_wait_ms: 100`.

## A capture differs: what it means

- **Line 2 differs** (BT.CPP, the rmw or MoveIt): another environment. Not a pass, and NOT a
  reason to re-capture.
- **A scenario differs**: a legacy end-effector path changed behaviour. Read the diff
  (`diff test/data/m2_6/X.txt build/trainit_motion_runtime/m2_6_actual/X.txt`): a behaviour change
  is an owner decision, never a re-freeze. The only differences ADR-0022 allows after a production
  commit are classified WARN lines, if the owner chose them (EEF-D3).

## Re-freezing (a harness fix, a new scenario, another environment)

Only from the 1dda503 production sources, never from modified production code, never because a
test failed:

1. `git archive 1dda503` into a scratch workspace outside every repository, and copy the current
   `test/` tree and the test-only `CMakeLists.txt` lines over it.
2. Build it there; run its binaries with its build directory FIRST on `LD_LIBRARY_PATH` (a sourced
   install otherwise wins over the binary's RUNPATH; the E7/E8 children inherit it), and check
   with `ldd`.
3. Determinism gate: two runs, in two processes, byte-identical. If not, fix the harness.
4. Copy the scratch run's `m2_6_actual/<family>.txt` here, review the diff line by line, commit
   it on its own with the procedure in the message, then check that HEAD reproduces it.
