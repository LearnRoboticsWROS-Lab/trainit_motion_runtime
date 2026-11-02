# vNext M3.5 test data -- process-path semantics (ADR-0024, M3_DESIGN §9)

- `paths.yaml`: paths for the M2.5 test arm whose states are poses the fake joint states can put the
  arm at (tcp = (0.5 cos q1, 0.5 sin q1, 0.3), yaw q1, with q1 = 0, 0.2, 0.4, 0.6): `inspect` (stop,
  an event after arriving at states 1, 2, 3), `glue` (blended into state 1, an event before departing
  state 2), `glue_whole` (the same geometry with no event: the reference of the split), `blended`
  (no events, a blend into state 1), `dense` (a generator path).
- `captures/m3_5_follow_path_range.txt`: the AFTER record of `test_m3_5_follow_path_range`
  (FollowPath from= to=). New behaviour, so there is no BEFORE: it was frozen from the first run of
  the M3.5 production on branch `feat/m3.5-process-path` after every scenario's outcome was asserted
  in the test and the goals read (RNG-07 + RNG-08 send exactly RNG-06's items, blend radii
  included). Re-freeze it only from production sources and only for an approved change; the legacy
  families (test/data/m2_5, m2_7*, m3_1, m3_4) are unaffected by this record.
