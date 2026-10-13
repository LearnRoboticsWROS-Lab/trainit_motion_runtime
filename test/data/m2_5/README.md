# vNext M2.5 captures: what TMR sends to MoveIt, frozen BEFORE the MotionRuntime seam

Each `<family>.txt` is the byte-exact record of what one family of TMR motion consumers sent to
in-process fakes of MoveIt (`test/m2_5/`): every goal and request as a whole message (block YAML,
precision 17, stamps removed), in order, with the BT result and the planner mode after each move.
The `test_m2_5_*` binaries regenerate them into `build/.../m2_5_actual/` and compare byte for
byte. They were recorded on **TMR 1.8.0, production sources == 9cfb601**, before vNext M2.5
touched a production line (ADR-0015 amendment M2.5); every later commit must reproduce them.

## A capture differs: what it means

- **Line 2 differs** (MoveIt, BT.CPP or the rmw): another environment. Not a pass, and NOT a
  reason to re-capture. The numbers are bit-exact for this environment: TOTG/Ruckig and FK doubles
  are printed at 17 digits, so another architecture or libm may also differ in the last digit.
- **A scenario differs**: TMR's MoveIt traffic changed. Read the diff (`diff test/data/m2_5/X.txt
  build/trainit_motion_runtime/m2_5_actual/X.txt`): a behaviour change is an owner decision, never
  a re-freeze.

## Re-freezing (a harness fix, a new scenario, another environment)

Only from the 1.8.0 production sources, never from modified production code, never because a test
failed:

1. `git archive 4a02c86` (the last tests-first commit: production == 9cfb601) into a scratch
   workspace outside every repository, and copy the current `test/` tree (and any test-only
   `CMakeLists.txt`/`package.xml` lines) over it. A test that needs a later API (`currentTipPose`)
   is left out of that copy.
2. Build it there; run its binaries with its build directory FIRST on `LD_LIBRARY_PATH` (a sourced
   install otherwise wins over the binary's RUNPATH), and check with `ldd`.
3. Determinism gate: two runs, in two processes, byte-identical. If not, fix the harness.
4. Copy the scratch run's `m2_5_actual/` files here, review the diff line by line, commit them on
   their own with the procedure in the message, then check that HEAD reproduces them.

The vNext M2.5 forensics keep the scripts (`mutations/run_harness_fix_proof.sh`).

## The namespace families since vNext M2.7 (owner decision D2)

`namespace.txt`, `namespace_remap.txt`, `namespace_constructor.txt` and
`namespace_remap_constructor.txt` stay here, byte for byte, as the BEFORE record (TMR 1.8.0): a
runtime in /r3 resolved MoveGroupInterface's names at the ROOT. Since the vNext M2.7 namespace fix
(ADR-0023 Decision 3; first released in TMR 2.1.0) the runtime resolves them, and its
PlanningSceneInterface, in its node's namespace: `test_m2_5_namespace` and
`test_m2_5_namespace_remap` compare with the AFTER series in `test/data/m2_7_ns/` (they still
write their capture to `m2_5_actual/`), and `test_m2_7_namespace_diff` pins the sha256 of these
four files and proves that the two series differ only by namespace resolution. No binary compares
with these four files any more, and they are never re-frozen. Every other family here is
unchanged, still compared, and must still be reproduced byte for byte.
