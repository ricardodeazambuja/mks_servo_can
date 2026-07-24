# Roadmap — planned work, in priority order

Branch: `library-hardening`.
Baseline when written: commit `3bd37ce`, 508 tests passing / 30 skipped,
`ruff check .` clean, library coverage 58%.

Context: a Python library for MKS SERVO42D/57D stepper drivers over CAN, plus a
simulator that emulates the same protocol so the library can be developed with no
hardware attached. A review on 2026-07-24 found defects in both. **The simulator
findings have been fixed** (commit `3bd37ce`); the library findings are what this
roadmap mostly addresses.

**Where things are written down.** `REVIEW_NOTES.md` Part 0 is the source of
truth for *what is wrong and how it was proved* — symptoms, reproductions,
frame-level evidence. This file is the source of truth for *what to do about it
and in what order*. The two overlap deliberately so this file can be handed to
someone as a self-contained brief; where they disagree about a technical detail,
`REVIEW_NOTES.md` wins.

---

## Read this before touching tests

Two failure modes in this repo have repeatedly let broken code ship green, and
both are easy to reintroduce:

1. **`MagicMock(spec=SomeClass)` does not protect you.** `spec` restricts reads
   from a mock only until something writes to it. Tests here built
   `MagicMock(spec=SimulatedMotor)` and then *assigned* the attributes the code
   under test wanted, teaching the mock an API the real class had never had. The
   suite passed while `/status` returned HTTP 500 and `--json-output` crashed on
   startup. **Prefer a real object.** Where a stub is unavoidable, it must use
   the real schema of the thing it stands in for.
2. **Asserting that a call was made is not asserting that it worked.** The
   existing "coverage" for the stale-frame logic (item L1 below) mocks
   `expect_stale_notification` and checks it was *requested* — never that the
   credit lands on the right frame. The bug it was meant to guard is 100%
   reproducible and the test does not see it.

The simulator is now trustworthy enough to test against directly. Prefer an
integration test over a mock wherever the simulator can express the scenario.

Fixtures for driving the simulator from a test live in `tests/conftest.py`
(`basic_api`, `compliance_api`, `performance_api`).

---

## Item 1 — Fix the library defects — **done**

L1–L5 are fixed, each with a test that fails against the code as it was, and
each verified by mutation: the fix is undone, the test is confirmed to fail, and
the file is restored. `REVIEW_NOTES.md` Part 0 carries the detail; the tests are

| defect | fixed in | covered by |
|---|---|---|
| L1 re-targeting a move in flight always failed | `can_interface.py`, `axis.py` | `tests/integration/test_move_supersede.py` |
| L2 `save_or_clean_speed_mode_params` always timed out | `low_level_api.py` | `tests/integration/test_speed_mode_params.py` |
| L3 default speed read as user units | `axis.py` | `tests/integration/test_default_speed.py` |
| L4 one bad frame killed the receive path | `can_interface.py` | `tests/unit/test_can_interface_listener.py` |
| L5 stream start atomicity, feedback cost, small items | `realtime.py`, `axis.py`, `low_level_api.py`, `can_interface.py` | `tests/integration/test_stream_start_atomicity.py`, `tests/integration/test_stream_feedback_cost.py`, `tests/unit/test_regressions.py` |
| L8 absolute move skipped against a stale position cache | `axis.py` | `tests/integration/test_default_speed.py`, `tests/unit/test_regressions.py` |

L8 was not in the original review. It surfaced while building the test for L3:
`wait=True` returns as soon as the completion frame resolves the move future,
one step before the cached position is refreshed, so commanding the position the
axis started from hit the "already at target" shortcut against a pre-move cache
and returned success without sending anything.

Two of these are written to be correct under either answer to a question only
the hardware trace can settle — L1's credit expiry, and the removal of CanRSP
toggling from the feedback path. See Item 2, which is now the head of the list.

---

## Item 2 — Record a hardware trace (can run in parallel; needs the bench)

The only item nobody else can do, and the one `REVIEW_NOTES.md` Part 1 already
ranks first. The simulator is validated against the *manual*, and the library
was written from the same reading — so a shared misreading is invisible to every
test in the repo.

```
export MKS_HIL_CHANNEL=can0
pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json
```

The harness is `tests/hil/test_hardware_conformance.py`. One capture then
replays in CI forever with no hardware attached.

Three questions it settles, in descending value:

1. **Does 0xF5 actually emit an abort frame when re-targeted mid-move?** This
   confirms or refutes the model Item 1a is built on. Fix L1 defensively first,
   confirm here.
2. **Sign convention.** CCW-positive is currently an *inference* from the
   manual's worked examples; the manual's prose contradicts itself (recorded in
   the `errata` block of `tests/fixtures/manual_commands_v106.json`). Every
   conversion in `motor_profile.py` and every kinematics class rests on it.
3. **Does CanRSP (0x8C) suppress the reply to 0x8C itself, and to reads?** The
   simulator says no — only sections 6.4–6.8 of the manual, i.e. the run
   commands. If that is wrong, `ServoStream`'s feedback path deadlocks on
   hardware while passing every simulator test. Item 1e's change to
   `_run_feedback` depends on this answer.

---

## Item 3 — Burn down the documentation debt (gate is in place)

A mechanical check found **64 documentation problems**: references to methods and
parameters that do not exist, code blocks that are not valid Python, and index
links pointing at documents that were never written (27 of the 52 links in
`docs/README.md` were dead). Examples:

- `LinearKinematics(steps_per_mm=...)` — no such parameter
- `RotaryKinematics(units=...)` — no such parameter; the third argument is
  `degrees_per_output_revolution`, and `units` is hardcoded to `"deg"`
- `axis.move_absolute()`, `axis.get_current_position()`, `axis.set_kinematics()`,
  `axis.update_status()`, `axis.is_moving()`, `axis.get_error_description()` —
  none exist
- `CANInterface(enable_crc=...)`, `CANInterface(response_timeout_ms=...)`
- `BaseKinematics` — the exported name is `Kinematics`
- `docs/user_guides/library/reading_status.md` alone references seven
  non-existent `Axis` methods

**The gate now exists: `tests/test_docs_api.py`.** It extracts every
```` ```python ```` block from `docs/**/*.md` and `README.md`, parses each with
`ast`, and checks constructor keyword arguments, method names on known receiver
variables, `from mks_servo_can import ...` names, and that internal Markdown
links resolve.

It is a **ratchet**, not a clean gate: the 64 known problems are listed in
`tests/fixtures/docs_known_issues.json`, a new finding fails the build, and a
baseline entry that no longer occurs *also* fails the build with an instruction
to delete it. So documentation cannot get worse, and the debt can only shrink.

**The remaining work is to burn the baseline down.** Current contents:

| kind | count |
|---|---|
| `dead-link` — index entries pointing at documents never written | 31 |
| `bad-method` — methods that do not exist on `Axis`/`CANInterface`/… | 17 |
| `bad-argument` — constructor parameters that do not exist | 7 |
| `unparseable` — code blocks that are not valid Python | 7 |
| `bad-import` — names not exported from the module they are imported from | 2 |

Workflow: fix a file, run `pytest tests/test_docs_api.py`, delete the entries it
reports as stale. The `dead-link` group is the largest but the cheapest to
decide on — each is either a document worth writing or an index entry worth
deleting.

---

## Item 4 — Packaging: L7, then PyPI

**L7 first.** `mks_servo_simulator/mks_simulator/interface/llm_debug_interface.py`
loads the command specification from `tests/fixtures/manual_commands_v106.json`
(see `_MANUAL_SPEC_PATH`). An installed wheel has no `tests/` directory, so
`/commands` and `available_commands` are empty for anyone who did not clone the
repository. Move the specification into the library package alongside the
constants it describes, and have both the tests and the simulator read it from
there. Update `package_data`/`MANIFEST.in` accordingly.

Consumers to update: `llm_debug_interface.py`, `virtual_can_bus.py` (imports
`MANUAL_COMMANDS`), `tests/simulator_compliance/test_wire_format.py`,
`test_can_frame_format.py`, `test_protocol_compliance.py`.

**Then publish.** `pip install mks-servo-can` still fails; installation means
cloning and two editable installs from subdirectories. `REVIEW_NOTES.md` Part 1
ranks this as the largest remaining barrier to anyone else using the library.
L7 is a prerequisite: the simulator wheel currently depends on a directory that
is not shipped.

---

## Item 5 — Two loose ends from the simulator work

- **`textual_dashboard.py` is the only surface not on the snapshot.** It reads
  motor attributes directly. It happens to read the *right* ones today, but that
  is exactly the arrangement that produced the drift just removed from the other
  three surfaces. Either migrate it to `SimulatedMotor.status_snapshot()` or
  retire it in favour of the browser dashboard at `/dashboard`.
- **`--textual-dashboard` runs on the simulator's own event loop.** In `cli.py`,
  `loop.create_task(textual_app.run_async())` puts the TUI in direct competition
  with the 10 ms motor integration tick. If the TUI stays, move it off.

---

## Item 6 — Coverage where it is thinnest

Current library coverage, weakest first:

| module | coverage |
|---|---|
| `digitizer/base_digitizer.py` | 11% |
| `digitizer/surface_mapping.py` | 17% |
| `digitizer/precision_analyzer.py` | 35% |
| `multi_axis_controller.py` | 44% |
| `can_interface.py` | 47% (Item 1 lifts this) |
| `low_level_api.py` | 57% |
| `axis.py` | 61% |

The digitizer is by far the weakest area and the one most likely to harbour the
same class of defect the review found elsewhere.

---

## Item 7 — Deterministic simulated time (larger; consider after 1–4)

`tests/determinism/` has been an empty placeholder since May 2025 — the same
state `tests/hil/` was in before it was built out. Every simulator test currently
paces itself with `asyncio.sleep` against a wall clock, which is why the suite
takes ~75 s and why the timing-sensitive tests will eventually go flaky under CI
load.

A `--step` / `/step` control that advances simulated time explicitly would make
those tests deterministic and fast. It is also the single change that would most
improve the simulator for an agent driving it: right now an agent has to *guess*
how long to wait after issuing a command before reading back state.

This is a bigger piece of work than Items 1–4 and is not urgent. It is the
highest-leverage thing after the state model if the agent-facing direction
matters.

---

## Suggested sequencing

1. ~~**Items 1 and 2 in parallel.**~~ Item 1 is done; **Item 2 is now the head of
   the list** and needs only bench time.
2. **Items 3 and 4 together** as a release-readiness pass.
3. **Items 5, 6, 7** as capacity allows.

## Definition of done — the standard Item 1 was held to

Reuse this for the items below:

- Each defect fixed with a test that fails against the code as it was, driving
  the simulator or real objects rather than a mock.
- Each test verified by mutation: undo the fix, confirm the test fails, confirm
  the mutation actually landed on disk, restore. A test that passes under
  mutation is not a test.
- `pytest -q` green in random order, and `ruff check .` clean.
- `REVIEW_NOTES.md` Part 0 updated to say what was fixed and how it is covered.
- `CHANGELOG.md` updated under `[Unreleased]`.
