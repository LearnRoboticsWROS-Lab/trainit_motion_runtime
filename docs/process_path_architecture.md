# Process path architecture

For process applications (dispensing, sealing, welding, inspection, contouring),
the planner during the process is NOT a free-space sampler. The sequence is:
```
free-space transfer (OMPL/Pilz PTP)  ->  reach a safe start pose
process path backend                 ->  follow the cartesian path (process ON)
free-space transfer                  ->  leave the process zone
```

## Two backends

### pilz_sequence (`pilz_sequence_backend.*`)
Chains LIN/CIRC (and PTP) segments into ONE executed motion via the Pilz
`MoveGroupSequence` action (`sequence_move_group`), with optional blending.
- Each `ProcessSegment` becomes a `MotionSequenceItem{req, blend_radius}`.
- `req` uses `pipeline_id = pilz_industrial_motion_planner`, `planner_id =
  LIN|CIRC`, goal constraints via `kinematic_constraints::constructGoalConstraints`.
- CIRC carries its interim/center point as a named (`interim`/`center`) position
  constraint on the tip link.
- The LAST item's `blend_radius` is forced to 0 (Pilz requirement).
- Requires the Pilz sequence capability loaded by `move_group`
  (`build_move_group_params` sets `capabilities`).
This is the **preferred, deterministic** backend for production lines/arcs.

### cartesian_waypoints (`cartesian_waypoint_backend.*`)
Dense cartesian path via `MoveGroupInterface::computeCartesianPath` + time
parameterization.
- Accepts the path only if `fraction >= cartesian_fraction_threshold`
  (default 1.0) unless `allow_partial_cartesian` is set.
- `computeCartesianPath` returns zero timing → **TOTG**
  (`TimeOptimalTrajectoryGeneration`) re-times it; optional Ruckig smoothing.
- KDL cartesian interpolation is fragile on this robot — prefer `pilz_sequence`
  for straight production paths; use this for dense free-form curves.

## Generators & loaders (`process_path_generator.hpp`)
Reusable geometry → `CartesianProcessPath`:
- `PolylinePathGenerator`, `CirclePathGenerator`, `RectanglePathGenerator`,
  `RasterPathGenerator` (boustrophedon).
- `YamlProcessPathLoader`, `CsvProcessPathLoader` — load a path from file
  (CAD/CAM export, vision output, hand-authored). A "spline" is represented by
  DENSE sampling — it is NOT a native robot spline.

## In a task (the application view)
A `process_path` step carries a `ProcessPathSpec` (generator params or a file).
The runner builds the path, enables the process channel (if any), runs
`executeCartesianProcess`, then disables it. The backend is chosen by the launch
param `process_path_backend` (per-run).

## Process synchronization — LIMITATION
Process events (ON/OFF/trigger/analog) fire at **segment boundaries**, NOT at an
exact TCP position/time. Precise position/time sync is a future capability of the
robot adapter or the PLC. The `ProcessController` (mock or ROS) is the
abstraction; it does not bind to a specific digital output.

## Metrics (Section 21)
Collected per process path: input/output waypoint counts, cartesian fraction,
trajectory duration, path length, average/max TCP speed, max TCP acceleration,
and max TCP line deviation (computed by FK: each planned joint waypoint → TCP
transform → distance to the ideal polyline). Orientation deviation and obstacle
clearance are placeholders for future refinement.
