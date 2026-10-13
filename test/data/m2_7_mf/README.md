# vNext M2.7 (owner Q8.2): what a motion failure does to the canonical bin-picking loop

`test_m2_7_bin_picking_motion` ticks the canonical bin-picking tree as XML with every node REAL,
`MoveWaypoint` included, on a real `MoveItMotionRuntime` against the M2.5 fake move_group, and the
real end-effector registry against the M2.6 fake end-effector adapter
(`test/m2_7/mf_bin_picking_cell.hpp`). A move's outcome is what `MoveWaypoint` makes of MoveIt's
answer, scripted per goal ordinal on the fake; the capture records every request MoveIt received and
what the tree did with the outcome: the next candidate, the stall branch, HOLD, or the end of the
run. Domain 97 (the M2.6 in-process lock), localhost only, graph-guarded.

| File | What it is | Frozen on |
|---|---|---|
| `bin_picking_motion_before.txt` | BEFORE (C0): the M2.6 tree `../m2_6_contract/bin_picking_canonical.xml` (TSA's generation for its pinned canonical bin-picking project, sha256 `25fb4e25…56716b5b`, never edited), 15 scenarios | **production adf94b0, TMR 2.0.0**, before vNext M2.7 changed a production line; sha256 `8df96d5f…adf50308` |
| `bin_picking_canonical.xml` | the M2.7 tree: the same project's generation with MoveWaypoint's `fault` remapped to the gate (`fault="{eef_fault}"`) on the five moves inside the cycle's absorbing Fallback (`pick`, `post_pick` in the retry; `pre_place`, `place`, `post_place` after it), sha256 `93c37df9…71557b59a5`, 4880 bytes. The prologue's `ready` and `pre_pick` are not wired | TSA's generation with the vNext M2.7 wiring (TSA 7.8.0 on); produced for this directory by a scratch copy of TSA 765346e with that wiring applied, two runs identical. Pinned here by sha256: the generator's own copy must be byte-identical |
| `bin_picking_motion_after.txt` | AFTER (C2): the M2.7 tree, the same 15 scenarios | the production of the commit that added it (vNext M2.7 MF-P1: `MotionResult`'s phase and cause, `motion_failure.hpp`, MoveWaypoint's `fault`; `package.xml` still 2.0.0, first released in TMR 2.1.0); three runs in three processes byte-identical; sha256 `cc564d62…c56d7e62e` |

The test ticks both trees on whatever production it is built from. The M2.6 tree must reproduce
`bin_picking_motion_before.txt` byte for byte (C1 == C0): a tree that does not remap MoveWaypoint's
`fault` behaves exactly as before. The M2.7 tree must reproduce `bin_picking_motion_after.txt`.
`the_BEFORE_to_AFTER_diff_is_exactly_the_intended_lines` (no ROS graph) reads the two committed
captures and requires exactly the difference below.

## The scenarios

Scripting is by goal ordinal: one MoveGroup goal per planned move (four when a plan fails: TMR
retries every plan failure, `plan_retries + 1`), one ExecuteTrajectory goal per executed move. The
moves in order: #1 `ready`, #2 `pre_pick`, then per cycle `pick`, `post_pick`, `pre_place`, `place`,
`post_place`. The tree is bounded to two cycles; the detector publishes three candidates, best first.

The outcome columns below are copied from the capture (verified by execution, two runs in two
processes, byte-identical). `stall` is `recovery/bin_empty/bin_stalled`.

| Id | What it pins | MoveIt script (or input) | bt | stall | vacuum | scene_calls | eef_fault | runner_exit |
|---|---|---|---|---|---|---|---|---|
| S0 | nominal | default | SUCCESS | 0/0/0 | engage release engage release | 4 | `""` | 0 |
| S1 | every candidate unreachable (class A) | move_action: SUCCEED ×2, ABORT(−31) ×24 | SUCCESS | 2/0/2 | (none) | 0 | `""` | 0 |
| S2 | move_group's MoveGroup action gone mid-run (class B, plan) | hook: drop `move_action` at the first RUNNING after 7 executes (cycle 1's post_place dwell) | SUCCESS | 1/0/1 | engage release | 2 | `""` | 0 |
| S3 | CONTROL_FAILED executing candidate 0's approach, nothing held | execute: SUCCEED ×2, ABORT(−4) | SUCCESS | 0/0/0 | engage release engage release | 4 | `""` | 0 |
| S4 | CONTROL_FAILED executing pre_place while holding (Q8.1) | execute: SUCCEED ×4, ABORT(−4) | FAILURE | 0/0/0 | engage | 2 | `""` | 1 |
| S5 | PLANNING_FAILED planning place while holding | move_action: SUCCEED ×5, ABORT(−1) ×4 | FAILURE | 0/0/0 | engage | 2 | `""` | 1 |
| S6 | a silent detector (Q9) | detector silent | FAILURE | 0/0/0 | (none) | 0 | `"DETECTION_TIMEOUT"` | 1 |
| S6b | a fresh detection with no match | detector class `bottle` | SUCCESS | 2/0/2 | (none) | 0 | `""` | 0 |
| S6c | a detection TF cannot transform (Q9) | detector frame with no transform | FAILURE | 0/0/0 | (none) | 0 | `"TF_LOOKUP_FAILED"` | 1 |
| S7 | candidate 0 unreachable, candidate 1 reachable | move_action: SUCCEED ×2, ABORT(−31) ×4 | SUCCESS | 0/0/0 | engage release engage release | 4 | `""` | 0 |
| S8 | CONTROL_FAILED executing post_place after the release, nothing held | execute: SUCCEED ×6, ABORT(−4) | SUCCESS | 1/0/1 | engage release engage release | 4 | `""` | 0 |
| S9 | PLANNING_FAILED planning post_place after the release | move_action: SUCCEED ×6, ABORT(−1) ×4 | SUCCESS | 1/0/1 | engage release engage release | 4 | `""` | 0 |
| S10 | PLANNING_FAILED planning post_pick after candidate 0's grasp | move_action: SUCCEED ×3, ABORT(−1) ×4 | SUCCESS | 0/0/0 | engage engage release engage release | 5 | `""` | 0 |
| S11 | CONTROL_FAILED executing post_pick after candidate 0's grasp | execute: SUCCEED ×3, ABORT(−4) | SUCCESS | 0/0/0 | engage engage release engage release | 5 | `""` | 0 |
| S12 | START_STATE_IN_COLLISION on every candidate plan | move_action: SUCCEED ×2, ABORT(−10) ×24 | SUCCESS | 2/0/2 | (none) | 0 | `""` | 0 |

What the BEFORE shows: a failure that is not the target's (S2, S3, S8, S11, S12) reads today as "try
the next candidate" or as a stalled bin, exactly like an unreachable target (S1, S7, S9, S10); a
failure while an object is held is HOLD (S4, S5); a detector that cannot be read is a FAULT, never an
empty bin (S6, S6c), while a fresh detection with no match is the empty bin (S6b).

## AFTER - BEFORE, exactly

- **Header:** lines 1-2 equal; line 3 names the M2.7 tree instead of the M2.6 one.
- **Unchanged, byte for byte:** S0, S1, S5, S6, S6b, S6c, S7, S9, S10. An unreachable target (class
  A) keeps the existing strategy, a planning failure while holding is HOLD as before, a detector
  fault is a FAULT as before.
- **S4** (CONTROL_FAILED on `pre_place` while holding): identical but one line, `eef_fault: ""` ->
  `eef_fault: "MOTION_FAULT: CONTROL_FAILED during execute"`: HOLD as before, and the gate now says why.
- **S2, S3, S8, S11, S12** (class B): the move writes its FAULT into the gate and the run stops.
  AFTER's MoveIt events are BEFORE's first events up to and including the class-B failure (the
  same text and digests: no request BEFORE did not send; S2's are all 21, nothing reached MoveIt
  after the drop in either run), its moves a prefix of BEFORE's, and:

| Id | events | bt | stall | vacuum | scene_calls | eef_fault | runner_exit |
|---|---|---|---|---|---|---|---|
| S2 | 21 -> 21 | SUCCESS -> FAILURE | 1/0/1 -> 0/0/0 | engage release (both) | 2 (both) | `""` -> `"MOTION_FAULT: FAILURE during plan"` | 0 -> 1 |
| S3 | 39 -> 9 | SUCCESS -> FAILURE | 0/0/0 (both) | engage release engage release -> (none) | 4 -> 0 | `""` -> `"MOTION_FAULT: CONTROL_FAILED during execute"` | 0 -> 1 |
| S8 | 36 -> 21 | SUCCESS -> FAILURE | 1/0/1 -> 0/0/0 | engage release engage release -> engage release | 4 -> 2 | `""` -> `"MOTION_FAULT: CONTROL_FAILED during execute"` | 0 -> 1 |
| S11 | 42 -> 12 | SUCCESS -> FAILURE | 0/0/0 (both) | engage engage release engage release -> engage | 5 -> 1 | `""` -> `"MOTION_FAULT: CONTROL_FAILED during execute"` | 0 -> 1 |
| S12 | 30 -> 10 | SUCCESS -> FAILURE | 2/0/2 -> 0/0/0 | (none) (both) | 0 (both) | `""` -> `"MOTION_FAULT: START_STATE_IN_COLLISION during plan"` | 0 -> 1 |

S12's `START_STATE_IN_COLLISION` is a scripted answer of the fake: for a robot whose current state
collides, real Pilz moves do not return it. MoveIt (2.5.9 source) plans from such a start anyway:
Pilz checks the start's bounds and velocity, never its collision; FixStartStateCollision puts the
colliding start back as waypoint 0; move_group accepts a path whose only invalid waypoint is 0. TMR's
validator then refuses the plan as `START_STATE_COLLISION`, a FAULT too (`MOTION_FAULT:
START_STATE_COLLISION during validate`; `test_m2_7_motion_failure`, on a real model and scene). The
scenario stays: it pins the class of the code. No scenario here reaches a validator collision (the
M2.5 test arm has no collision geometry).

`latched` stays `(none)` everywhere: a motion FAULT is not an end-effector latch, so a run it ends
is exit 1, never 3. S11 stops with the vacuum engaged and never released (HOLD semantics). The
residuals stay visible on purpose: S9 (a class-A failure of `post_place` after the release reads
as a stalled bin) and S10 (a class-A failure of `post_pick` after the grasp tries the next
candidate with the vacuum still engaged) are unchanged.

`bin_empty` counts the tree's `<Log message="bin empty"/>`: the stall Fallback reaches it only when
its recovery step FAILS, and no scenario here fails it (the recovery channel answers). The stall
branch shows as `recovery` (the `bin_agitator` pulses) and `bin_stalled` (its "bin stalled" Log).

## Format

Header: line 1 `m2_7 capture bin_picking_motion`; line 2 the BT.CPP, rmw and MoveIt versions the
binary was built against (`m26::environmentLine`); line 3 `tree: <file> sha256=<hex>`; a blank line.
One block per scenario, ending with a blank line:

```
=== scenario <id>
pins: <what it pins>
script: <FakeMoveGroup::scriptText()> | hook: <none | drop move_action after cycle 1>
events:
[n] <KIND> <resolved endpoint> <what>          the M2.5 Log's first line, verbatim
  | <its first summary line, verbatim>         when it has one
  sha256=<hex>                                 the rest of the event as the Log recorded it
topics (asynchronous, sorted): ...             only when present
moves: <every MoveWaypoint that called the runtime, in order>
bt: <the tree's result>
stall: recovery=<n> bin_empty=<n> bin_stalled=<n>
vacuum: <the adapter's SetBool calls: engage | release> | (none)
scene_calls: <SetAttachedCollisionCheck calls>
eef_fault: "<the blackboard's final eef_fault>"
latched: <the first token of each fault the registry latched> | (none)
runner_exit: <the exit code trainit_run_bt derives: 0, 1, 2 or 3>
```

- The event digest covers everything the M2.5 Log recorded after the event's first line: every
  summary line and the whole message as block YAML at precision 17, stamps removed. An event with
  nothing after its first line (GetPlanningScene) hashes the empty text (`e3b0c442…`). Two identical
  requests (one plan's four attempts) carry the same digest.
- `moves` comes from `MoveWaypoint`'s INFO line, logged after every runtime call
  (`MoveWaypoint::tick` in `src/bt_nodes.cpp`), whatever the result; its ERROR line for a FAULT is
  not a move.
- Excluded: stamps, goal ids, durations, joint_states traffic, log text other than the tree's
  `[BT]` lines and `MoveWaypoint`'s INFO line, end-effector traffic other than the vacuum and latch
  lines.

## A capture differs: what it means

- **Line 2 differs** (BT.CPP, the rmw or MoveIt): another environment. Not a pass, and NOT a
  reason to re-capture.
- **A scenario differs**: what a motion outcome does to the canonical cycle changed, or MoveIt's
  traffic did. Read the diff (`diff test/data/m2_7_mf/<file>.txt
  build/trainit_motion_runtime/m2_7_mf_actual/<file>.txt`): a behaviour change is an owner decision,
  never a re-freeze. The diff test writes its per-scenario summary to
  `m2_7_mf_actual/bin_picking_motion_before_to_after.diff` (not committed).

## Re-freezing (a harness fix, a new scenario, another environment)

`bin_picking_motion_before.txt` only from the adf94b0 production sources, and
`bin_picking_motion_after.txt` only from the production sources of the commit that added it (`git
log --diff-filter=A -- test/data/m2_7_mf/bin_picking_motion_after.txt`); never from modified
production code, never because a test failed:

1. `git archive <that commit>` into a scratch workspace outside every repository, and copy the
   current `test/` tree and the test-only `CMakeLists.txt` lines over it. Leave out any test that
   needs a later API (on adf94b0: `test_m2_7_motion_failure`, `test_m2_7_motion_fault`, and this
   binary's AFTER and diff tests).
2. Build it there; run `test_m2_7_bin_picking_motion` with that build directory FIRST on
   `LD_LIBRARY_PATH` (a sourced install otherwise wins over the binary's RUNPATH), and check with
   `ldd`.
3. Determinism gate: two runs, in two processes, byte-identical. If not, fix the harness.
4. Copy the scratch run's `m2_7_mf_actual/<file>.txt` here, review the diff line by line (for the
   AFTER, `the_BEFORE_to_AFTER_diff_is_exactly_the_intended_lines` must pass on it), commit it on its
   own with the procedure in the message, then check that HEAD reproduces it.
