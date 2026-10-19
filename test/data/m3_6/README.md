# test/data/m3_6 -- vNext M3.6 / M3-D (ADR-0026)

`validation_c/` -- BYTE COPIES of what TSA generates for Validation C (TSA
`test/data/m3_6/validation_c.yaml` and `validation_c_detections.yaml`): the two trees
(`candidates.xml`, `detections.xml`: they differ only in their DetectCandidates line), the shared
`bt_params.yaml` (its `eef` block is the cell's contract, its `task_parameters` the blackboard) and
`motion_profiles.yaml`. Their sha256 digests are pinned on both sides: TMR
`test_m3_6_validation_c.cpp`, TSA `test_m3_6_validation_c.py` (which regenerates them in a temporary
directory and compares). A change to them is a TSA generation change: regenerate in a temp dir,
copy, re-pin both sides, and say why.

`captures/m3_6_validation_c.txt` -- the M3-D family: an AFTER record (new behaviour, no BEFORE),
frozen from the M3.6 production sources after two identical runs. Each scenario: the steps in the
order the tree ran them, every MoveGroup goal (pipeline / planner, scalings, link @ frame, the goal
position and orientation at 1e-9), the vacuum's commands and the tree's outcome. The designed values
are also checked numerically, independently of the record, by the test itself.
