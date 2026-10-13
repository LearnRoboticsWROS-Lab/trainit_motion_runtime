# vNext M2.6 AFTER: the canonical end-effector contract's shared data (ADR-0022)

| File | What it is | Owned by |
|---|---|---|
| `eef_conformance.yaml` | the conformance fixture: one `bt_params`-style `eef` block, four devices (the R1/R2-like `composite_tool`, `suction_simple`, `jaw_simple`, `jaw_binary`) | BYTE COPY, sha256 `873a1d58…a27a04f` |
| `eef_conformance.expected.txt` | its normalised dump | BYTE COPY, sha256 `1f6bd1d2…710d242c` |
| `eef_contract_tokens.txt` | the canonical tokens (interfaces, statuses, grasp, evidence, reasons and classes, …) | BYTE COPY, sha256 `55048ea4…2bd5de` |
| `normalize_eef.py` | the reference normaliser: the block's schema in executable form | BYTE COPY, sha256 `9fba00c3…49290a0b` |
| `eef_refusal_mutations.txt` | 125 mutations of the fixture and what the reference answers for each (every one of its 45 refusal sites is reached) | this repository |
| `bin_picking_canonical.xml` | the canonical bin-picking loop, as TSA generates it for its pinned canonical bin-picking project (TSA 7.7.0; since vNext M2.7, TSA 7.8.0, the generation is `../m2_7_mf/bin_picking_canonical.xml`, and this file is kept as its BEFORE) | BYTE COPY of TSA's `test/data/m2_6_eef/bin_picking_canonical.xml`, sha256 `25fb4e25…56716b5b` |

The four byte copies are never edited here: each repository that implements the contract keeps its
own copies and asserts their sha256 (`test_m2_6_eef_contract`, `test_m2_6_eef_reference.py`). No
test reads a path in another repository.

## How the parser is held to the reference

- `test_m2_6_eef_reference.py` runs the byte copy of `normalize_eef.py` (with PyYAML) on the
  fixture -- its dump must equal `eef_conformance.expected.txt` -- and on every row of
  `eef_refusal_mutations.txt`: the reference's answer must be the row's expected column. The column
  was COMPUTED by the reference when the table was written, never typed by hand, and is re-computed
  at every test run. Every `raise Refused(...)` site of the reference is reached by some row, checked
  by the SITE that raised it (its traceback line), not by its text: several sites raise the same
  words, so deleting the only row of a site fails the test.
- `test_m2_6_eef_contract.cpp` loads the same fixture through rcl's own YAML parser (exactly how a
  node receives its overrides), applies the same mutations (a `set` value is typed by rcl as a
  params file types it) and checks TMR's C++ parser gives the same column: the same refusal key
  and text, the same dump (by sha256), or LEGACY.

Rows the table cannot hold: an empty or mixed-type YAML sequence (rcl refuses it in a params
file).

History: the first reference copy (sha256 `57063387…bd384681`) raised `TypeError`, not `Refused`,
on a list where a site's `grip`, `engage` or `release` expects a name, and its token rule accepted
non-ASCII letters (Python's Unicode-aware `isalpha()`/`isalnum()`), which TMR's
`[A-Za-z][A-Za-z0-9_]*` refuses. Both were fixed at the source on 2026-10-10 (an
`isinstance(x, str)` guard; `str.isascii()`) and re-copied to every repository at once; rows
S14-S16 hold the list case. The second copy (sha256 `18acf5d0…af586f11`) gained two rules in its
third (`9fba00c3…49290a0b`), the same day, after the adversarial review: a `gripper_command`
actuator's target VALUES must be pairwise distinct (an adapter tells ENGAGE, RELEASE and TARGET
apart by the goal's position; rows A46-A55), and a `set_bool` grip's `engage` must be `engage`
(SetBool `data: true` engages, its polarity is the adapter's; S10, the swap, is now refused, and
rows S17-S21 hold the rest). The conformance fixture's dump did not change. There is no
deliberate difference left between TMR and the reference.

## The bin-picking tree

`bin_picking_canonical.xml` is what TSA generates (its `test_m2_6_eef_bin_picking` asserts the
bytes equal its generation) for the vision golden switched to bin picking and opted in to the
contract, made a bounded and quick run: `application.loop_cycles = 2`, 100 ms dwells, a 1.5 s
detection timeout, the recovery channel `bin_agitator`. So `test_m2_6_eef_bin_picking` proves
the GENERATED gate and can tell "bin empty" (the stall branch ran: the recovery pulsed, or its
`Log` was reached) from a FAILURE. The static rule the tree follows (TSA checks it on its side):
the reset Script is the first step of every cycle; EEF nodes inside the retry write `fault`, after
it `reason`; `holding` is set right after the retry and cleared right after the confirmed release;
the retried child and the stall Fallback carry the gate.

A first version of this file was written by hand from ADR-0022 D5's sketch; it was replaced by
TSA's generation before any commit, so the runtime is held to the tree a project really gets.

vNext M2.7 (owner Q8.2) wires MoveWaypoint's `fault` into the same gate on the five moves inside the
cycle's absorbing Fallback, so a motion failure that is not the target's ends the run instead of
reading as the next candidate or a stalled bin. That tree is `../m2_7_mf/bin_picking_canonical.xml`;
this one stays byte for byte (its sha256 is pinned by this repository's tests and by TSA's): it is
the BEFORE of that change, and `test_m2_6_eef_bin_picking` keeps ticking it with its stand-in
MoveWaypoint, which has no `fault` port (`../m2_7_mf/README.md`).
