# vNext M3.1 test data (ADR-0024)

- `bead.csv`, `bead.yaml`: dense path sources in the forms FollowPath honours (x,y,z lines; TMR's
  process-path YAML without per-sample speed or process keys).
- `bead_speeds.csv`, `bead_speeds.yaml`: the same sources WITH per-sample velocity/acceleration/
  process data, which the path model does not honour: the loader must refuse them.
- `paths.yaml`: a library that loads (test_path_library) and is ticked by test_m3_1_follow_path.
- `captures/`: the AFTER family of test_m3_1_follow_path, written by that test against the M2.5 fake
  move_group. M3.1 is new behaviour: there is no BEFORE. Re-freeze only from a reviewed change of
  FollowPath, MoveWaypoint's M3.1 keys or the sequence backend, never to make a failing test pass.
