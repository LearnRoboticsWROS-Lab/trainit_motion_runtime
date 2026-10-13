# vNext M2.7: the AFTER series of the M2.5 namespace families (owner decision D2)

The four files here have the names and the format of their BEFORE record in `test/data/m2_5/`
(TMR 1.8.0, kept there byte for byte): the same family line and environment line, the same
scenarios, written by the same M2.5 binaries; only the directory of the committed file changed.
They were recorded on the production sources of the commit that added this directory: the vNext
M2.7 namespace fix (ADR-0023 Decision 3: MoveGroupInterface and PlanningSceneInterface built with
the node's namespace) on top of 5c59662, with `package.xml` still at 2.0.0 (first released in TMR
2.1.0). They were frozen from a scratch build of exactly those sources: two runs, in two
processes, byte-identical.

| Family (file) | Binary | Domain | What it pins |
|---|---|---|---|
| `namespace` | `test_m2_5_namespace` | 92 | the runtime node CONSTRUCTED in `/r3`, a root and an `/r3` fake set both serving: NS-01a planned moves, NS-01b a cartesian process path, NS-01c state reads, NS-01d ServoTrack |
| `namespace_remap` | `test_m2_5_namespace_remap` | 93 | the same scenarios in a process remapped with `--ros-args -r __ns:=/r3` (a generated namespaced launch) |
| `namespace_constructor` | `test_m2_5_namespace` | 92 | NS-03: where the constructor waits when only the `/r3` fakes serve, the root endpoints opened one at a time |
| `namespace_remap_constructor` | `test_m2_5_namespace_remap` | 93 | NS-03 under the remap |

## The difference from BEFORE, exactly

`test_m2_7_namespace_diff` (no ROS graph) pins the sha256 of the four BEFORE files and checks that
exactly these lines change, each by its rule, and nothing else:

| File | Lines | Rule: what changes |
|---|---|---|
| `namespace.txt` (652 lines) | 12, 147, 159, 320, 533, 600 | R1: `[k] ACTION /move_action`, `ACTION /execute_trajectory` and `SERVICE /compute_cartesian_path` -> the same line with `/r3/...` |
| | 156, 329, 609 | R2: the `counts:` keys of those names under `/r3`, the keys re-sorted |
| `namespace_remap.txt` (652 lines) | the same 9 lines | the same: the process-wide remap never moved MoveGroupInterface's names |
| `namespace_constructor.txt` (14 lines) | 7, 8, 9 | R3: `constructed=no` -> `yes`: the in-process `/r3` constructor no longer waits for a root endpoint |
| `namespace_remap_constructor.txt` (14 lines) | 7, 8 | R3: the same (its PlanningSceneInterface was in `/r3` through the remap already) |

Identical: the requests (whole messages), their order, the ordinals, the trajectory digests, every
name that was already in `/r3` (the Pilz sequence, the planning-scene monitor, the state reads,
ServoTrack), and the scenario titles. The titles (`pins:` lines) keep the M2.5 wording verbatim,
"MGI goals at the root" and "compute_cartesian_path at the root" included: read the events. Only
MoveGroupInterface's eight names may move; a PlanningSceneInterface root name never counts.

## A capture differs: what it means

- **Line 2 differs** (MoveIt, BT.CPP or the rmw): another environment. Not a pass, and NOT a
  reason to re-capture.
- **A scenario differs**: TMR's MoveIt traffic from `/r3` changed. Read the diff (`diff
  test/data/m2_7_ns/X.txt build/trainit_motion_runtime/m2_5_actual/X.txt`): a behaviour change is
  an owner decision, never a re-freeze.

## Re-freezing (a harness fix, a new scenario, another environment)

Only from the production sources of the commit that added this directory (`git log
--diff-filter=A -- test/data/m2_7_ns/namespace.txt`), never from later modified production code,
never because a test failed: the procedure of `test/data/m2_5/README.md` with that commit in place
of `4a02c86` (`git archive` into a scratch workspace, the current `test/` tree over it, its build
directory FIRST on `LD_LIBRARY_PATH` and checked with `ldd`, two runs in two processes
byte-identical, then `m2_5_actual/namespace*.txt` copied here). `test_m2_7_namespace_diff` must
then pass on the new files; a line that moves changes its line map only after the diff was
reviewed line by line.
