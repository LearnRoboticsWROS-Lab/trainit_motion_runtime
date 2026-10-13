# vNext M2.7 captures: frozen BEFORE M2.7 touched a production line

Each `<family>.txt` has the format of `test/data/m2_5` (`test/m2_5/capture.hpp`): a header (the
family, then the MoveIt, BT.CPP and rmw versions the binary was built against) and one block per
scenario, written by the M2.5 scenario runner (`m25::run`): the trees ticked, every request the
in-process fakes received (whole messages, block YAML, precision 17, stamps removed) in order, the
asynchronous topics sorted, the BT result and the planner mode after each tick. They were recorded
on **TMR 2.0.0, production sources == adf94b0**, before vNext M2.7 changed a production line; the
two binaries of the table below regenerate them into `build/trainit_motion_runtime/m2_7_actual/`
and compare byte for byte. Not every `test_m2_7_*` binary writes there:
`test_m2_7_bin_picking_motion` writes its captures to `m2_7_mf_actual/` and compares them with
`test/data/m2_7_mf/`.

| Family (file) | Binary | Domain | What it freezes |
|---|---|---|---|
| `scene_nodes` | `test_m2_7_scene_nodes` | 82 | the four planning-scene BT nodes (owner decision 11.2) at the root: AddCollisionObject and AttachObject (one `apply_planning_scene` each; literal ports, defaults, the TSA-generated `{scene_<id>_dims}` / `{payload_<id>_dims}` forms, data errors that send nothing), RemoveCollisionObject (a `planning_scene` diff, no service call; an absent id), DetachObject (the attached REMOVE, then the world REMOVE diff), and the generated pick-and-place order on one runtime |
| `scene_nodes_remap` | `test_m2_7_scene_nodes_remap` | 82 | the same seven scenarios in a process remapped with `--ros-args -r __ns:=/r3` (a generated namespaced launch), a root and an `/r3` fake set both serving: the writes reach `/r3` (the process-wide remap moves the PlanningSceneInterface's own node). A runtime node CONSTRUCTED in `/r3` is not captured here: the M2.7 namespace fix moves those writes on purpose |

`test_m2_7_bt_interface` (AFTER, same domain) replays `test/data/m2_5/task_runner.txt` through
TaskRunner's interface constructor and writes its copy to `m2_7_actual/` too.

## A capture differs: what it means

- **Line 2 differs** (MoveIt, BT.CPP or the rmw): another environment. Not a pass, and NOT a
  reason to re-capture.
- **A scenario differs**: what a scene node sends, or where, changed. Read the diff
  (`diff test/data/m2_7/X.txt build/trainit_motion_runtime/m2_7_actual/X.txt`): a behaviour change
  is an owner decision, never a re-freeze. For the BtContext/TaskRunner narrowing (owner decision
  11.2) a difference means: roll the narrowing back and record it as debt.

## Re-freezing (a harness fix, a new scenario, another environment)

Only from the adf94b0 production sources, never from modified production code, never because a
test failed:

1. `git archive adf94b0` into a scratch workspace outside every repository, and copy the current
   `test/` tree and the test-only `CMakeLists.txt` lines over it. Leave out what needs a later API
   (from M2.7 on: `test_m2_7_bt_interface.cpp`, and `test_m2_5_interface_purity.cpp`, whose fake
   implements the grown door).
2. Build it there; run its binaries in a clean environment with its build directory FIRST on
   `LD_LIBRARY_PATH` (a sourced install otherwise wins over the binary's RUNPATH), and check with
   `ldd`.
3. Determinism gate: two runs, in two processes, byte-identical. If not, fix the harness.
4. Copy the scratch run's `m2_7_actual/<family>.txt` here, review the diff line by line, commit it
   on its own with the procedure in the message, then check that HEAD reproduces it.
