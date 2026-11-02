# vNext M3.4 test data (ADR-0015 amendment M3.4, owner decisions D3 and D8)

M3.4 makes TMR execute the motion contract a canonical TSA bundle requests -- or fail loudly. Every
behaviour change follows the M2.5/M2.7 method: the SAME scenarios recorded BEFORE, from the
unmodified production, and AFTER, then an exact, classified diff.

| File | What it is |
|---|---|
| `motion_profiles.yaml` | the profiles `test_m3_4_execution` loads: TSA 7.10.0's task-program shape (`plain`, `tol`, `pos_only`, `bad_tol`: planning budget and goal tolerance only) plus the legacy `approach` / `transit` rows a waypoint without a `profile` key falls back to |
| `runner_stop_tree.xml` | `test_m3_4_runner_stop`'s tree: one planned move whose execution the fake holds |
| `before/m3_4_execution.txt` | BEFORE: `test_m3_4_execution` (domain 84) on **production 9418694, TMR 2.2.0**, two runs in two processes byte-identical. Frozen by the tests-first commit; re-frozen after the M3.4 review added ST-07, CIRC-08 and CIRC-09, and after M3-B added ST-08, each time from the same production (`git archive 9418694` + the current `test/` tree and 312b566's CMakeLists.txt in a scratch workspace, its library first on `LD_LIBRARY_PATH`, two runs identical): every scenario frozen before is byte-identical in it. sha256 in `sha256.inc` |
| `before/m3_4_runner_stop.txt` | BEFORE: `test_m3_4_runner_stop` (domain 85, the real `trainit_run_bt` as a child) on the same production, two runs byte-identical; sha256 `e0af0088…3a740` |

The BEFORE files were frozen by the tests-first commit, which changed no production line: the two
binaries, this directory, and one harness addition in `test/m2_5/fake_move_group.hpp` (the fake
logs a `MoveGroupSequence.cancel` request and a held goal its client asked to cancel ends
CANCELED). That addition is inert for every older family: no 2.2.0 production path cancels a goal
of the fake move_group `[static: grep]`. Never re-freeze a BEFORE file: it records what 2.2.0 did.

## AFTER (the M3.4 production)

| File | What it is |
|---|---|
| `after/m3_4_execution.txt` | AFTER of `test_m3_4_execution`, two runs in two processes byte-identical |
| `after/m3_4_runner_stop.txt` | AFTER of `test_m3_4_runner_stop`, likewise |
| `legacy_after/<family>.txt` | the AFTER of the ten legacy families whose approved scenarios M3.4 moved; the M2.5 binaries compare with these (`kM34LegacyAfter`, `test/m2_5/capture.hpp`); their BEFORE stays in `test/data/m2_5/` (TMR 1.8.0) or `test/data/m2_7_ns/` (the namespace pair, M2.7), byte for byte |
| `sha256.inc` | the sha256 of every BEFORE and AFTER above, pinned by `test_m3_4_capture_diff` |

`test_m3_4_capture_diff` (pure, no ROS graph) splits each family into its scenarios and requires the
header and every scenario not listed to be byte-identical; each listed scenario must differ, show
the facts its classification names in the AFTER and, where it applies, no longer show what it
replaced. The pins lines of the legacy scenarios still describe the BEFORE behaviour on purpose
(the test sources are unchanged): the diff is behaviour only.

| Scenario(s) | Change | Classification |
|---|---|---|
| new TOL-01..04 | a named profile's goal tolerance reaches the MoveGroup pose goal; no leak into the next move | DATA-GATED BEHAVIOUR CHANGE |
| new TOL-07, TOL-08 | a Cartesian tolerance on a joint target, or an invalid one: `UNSUPPORTED_TOLERANCE`, nothing sent | FAILURE-PATH BEHAVIOUR CHANGE |
| new PLN-01..03; legacy planned_waypoint WP-FREE-08, planner_identity T3-02 `bogus` x2 | an unknown planner: refused (`UNKNOWN_PLANNER`), never the runtime mode | FAILURE-PATH BEHAVIOUR CHANGE |
| new CIRC-02, CIRC-03; legacy planned_waypoint WP-CIRC-04, planned_lin_circ CIRC-03, task_runner TASK-02 | the CIRC centre form runs (it threw `bad_optional_access`) | BUG FIX |
| new CIRC-04..07 | a non-finite aux point (`INVALID_POSE`) or one on the goal (`CIRCLE_GEOMETRY`): refused | FAILURE-PATH BEHAVIOUR CHANGE |
| new TIP-01; legacy planned_lin_circ CIRC-02 | `CircularTarget::tip_link` is the sequence item's link | DATA-GATED BEHAVIOUR CHANGE (only a target that names a link) |
| new RST-01, RST-02; legacy sequences T5-06, task_runner_2 TASK-03a/b | the Cartesian backend restores the pose reference frame and EE link: the next LIN goal is in base_link, not world | BUG FIX |
| new ST-08 | FollowPath's start check waits (2 s, bounded) for an arm still settling into states[0] (found by M3-B in Isaac: 27 mm right after the PTP, 1 mm at rest) | FAILURE-PATH BEHAVIOUR CHANGE (a robot already at the start passes at the first read; one that never arrives is refused as before) |
| new ST-07 | no fresh state before a Cartesian path: refused before anything is sent (2.2.0 timed the path from the model's default state and executed it) | FAILURE-PATH BEHAVIOUR CHANGE |
| new CIRC-08, CIRC-09 | a non-finite CIRC goal; a sequence CIRC item with neither via point nor centre: refused | FAILURE-PATH BEHAVIOUR CHANGE |
| new ST-03..06; legacy state_reads READ-04, servo_track_2 SERVO-14, namespace / namespace_remap NS-01c | no fresh robot state: FAILURE with the reason (`STATE_UNAVAILABLE` for FollowPath), never the identity pose; ServoTrack sends nothing | FAILURE-PATH BEHAVIOUR CHANGE |
| new TO-01 | a timed-out Pilz sequence goal is cancelled | FAILURE-PATH BEHAVIOUR CHANGE |
| runner STOP-RUNNER-01 | SIGINT during a motion sends move_group's stop event | FAILURE-PATH BEHAVIOUR CHANGE (shutdown path) |
| new ALS-01, ALS-02; legacy planned_waypoint WP-LIN-04 | the aliases linear / circular take the approach profile | BUG FIX |

Unchanged by construction and verified byte-identical: every other scenario of these families and
every other family (M2.5, M2.6, M2.7, M3.1).
