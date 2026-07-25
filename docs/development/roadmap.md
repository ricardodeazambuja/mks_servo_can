# Roadmap — remaining work, in priority order

Repository: `/home/ricardodeazambuja/backup/GitStuff/mks_servo_can`
Branch: `library-hardening`, **nothing pushed** (no upstream tracking branch).
Baseline: commit `d989efd`, 561 tests passing / 30 skipped, `ruff check .` clean,
library coverage 66%, documentation baseline at 26 known problems.

Context: a Python library for MKS SERVO42D/57D stepper drivers over CAN, plus a
simulator that emulates the same protocol so the library can be developed with no
hardware attached. Two reviews found defects in both. **All of them are now
fixed** — L1–L5 and L7 from the reviews, plus L8 and L9 found while testing the
fixes. `REVIEW_NOTES.md` Part 0 records each with its original reproduction;
`CHANGELOG.md` `[Unreleased]` says what changed and why.

**Where things are written down.** `REVIEW_NOTES.md` Part 0 is the source of
truth for what was wrong and how it was proved — symptoms, reproductions,
frame-level evidence. This file is the source of truth for what to do next and in
what order; where the two disagree on a technical detail, `REVIEW_NOTES.md` wins.
This file is deliberately self-contained, so it can be handed to someone (or to
an agent) as a complete brief with no other context.

---

## Read this before touching code or tests

Three things have repeatedly let broken code ship green here. All three have bitten
within the last session.

1. **The house defect is "reported success, did nothing".** Not crashes —
   silence. `/status` reported zeros for a moving motor; an absolute move was
   skipped against a stale cache and returned success without sending a frame;
   digitizer playback printed `PLAYBACK COMPLETE` after failing to command a
   motor; `ServoStream.start()` left motors mute with no path that would restore
   them. When reading unfamiliar code here, find the paths where a failure is
   caught or a shortcut is taken and ask what the caller sees.
2. **`MagicMock(spec=X)` does not protect you.** `spec` restricts a mock only
   until something writes to it. Tests built `MagicMock(spec=SimulatedMotor)` and
   then *assigned* the attributes the code wanted, teaching the mock an API the
   real class never had — the suite passed while `/status` returned HTTP 500.
   Prefer real objects; the simulator is cheap to start.
3. **Asserting a call was made is not asserting it worked.** Assert the *effect*:
   where the motor ended up, what frame went out, what the next command received.

**Verify every new guard test by mutation.** Undo the fix, confirm the test
fails, confirm the mutation actually landed on disk, restore. Restore from a
variable held in the mutation script — **never** `git checkout --`, which
destroys uncommitted work (it ate a whole transport fix last session). Run each
mutation against the *whole* test file and re-check anything that survived: a
test can be masked by state a previous test left behind.

**Facts that decide whether a test can see anything:**

- CanRSP (`0x8C`) suppresses only the run commands of manual sections 6.4–6.8
  (`0xF6`, `0xFD`, `0xFE`, `0xF4`, `0xF5`). An encoder read (`0x31`) is answered
  either way, so "is this motor still answering?" must be asked with a run
  command.
- Short moves are acceleration-limited: with the default accel parameter of 100,
  a 900° move takes 1.5 s at speed parameter 500 and 2.1 s at 83. A sixfold speed
  error shows up as 40% in wall clock, so compare a move against *the same move
  at the speed the default is supposed to mean*, not against a ratio threshold.
- The motor emits a superseded move's abort frame *after* the new command's
  acknowledgement, both carrying the same command byte.

Fixtures for driving the simulator live in `tests/conftest.py` (`basic_api`,
`compliance_api`, `performance_api`, and the `*_can_interface` fixtures under
them). The compliance simulator serves CAN IDs 1–3; ID 99 answers nothing, which
is how failures are provoked without stubbing.

---

## Item 1 — Record a hardware trace *(highest value; needs the bench)*

The only item nobody else can do, and now the head of the list. The simulator is
validated against the *manual*, and the library was written from the same
reading — so a shared misreading is invisible to every test in the repo.

```
export MKS_HIL_CHANNEL=can0          # or /dev/ttyACM0 for slcan
export MKS_HIL_ALLOW_MOTION=1        # only for the tests that rotate the shaft
pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json
```

The harness is `tests/hil/test_hardware_conformance.py`; the `--hil-record`
option is defined in `tests/conftest.py`. One capture then replays in CI forever
with no hardware attached.

Three questions it settles, in descending value:

1. **Does `0xF5` actually emit an abort frame when re-targeted mid-move?** The
   L1 fix is written to be correct either way — a credit that matches on the
   status byte *and* expires — but which case is real is still unknown.
2. **Sign convention.** CCW-positive is an inference from the manual's worked
   examples; the prose contradicts them. Recorded in the `errata` block of
   `mks_servo_can/data/manual_commands_v106.json`. Every conversion in
   `motor_profile.py` and every kinematics class rests on it.
3. **Does CanRSP suppress replies to reads as well as to run commands?** The
   simulator says no. If that is wrong, `ServoStream`'s feedback path stops
   working on hardware while passing every simulator test. The code now warns
   when feedback reads keep failing with responses disabled, naming this
   possibility, but only a trace settles it.

**Done when:** `tests/fixtures/hardware_trace.json` exists, the replay test runs
in CI, and the three answers are written into `REVIEW_NOTES.md` — including if an
answer contradicts what the simulator does, in which case the simulator changes.

---

## Item 2 — Finish the documentation: 26 pages that were never written

Every API mismatch is gone (63 problems down to 26). What remains is a backlog of
missing documents, all linked from `docs/README.md` and already marked
*(planned)* there, all tracked as `dead-link` in
`tests/fixtures/docs_known_issues.json`.

Each is a **decision**, not a repair: write the page, or drop the index entry.
Grouped so it can be decided in one pass:

| group | files | suggested call |
|---|---|---|
| API reference | `api_reference/library/{axis,can_interface,constants,exceptions,kinematics,low_level_api,multi_axis_controller}.md`, `api_reference/simulator/{cli,motor_model,virtual_can_bus}.md` | Generate from docstrings, or drop the entries and point at the modules — the docstrings already carry the design rationale and cannot drift |
| tutorials | `tutorials/{single_axis_sim,single_axis_hw,cartesian_robot_example,rrr_arm_example,two_link_planar_arm_example}.md` | Point at the runnable scripts in `examples/`, which are tested; a prose copy would drift |
| development | `development/{setup,running_tests,contributing,coding_standards}.md` | Worth writing — short, and the material is in `README.md`, `pyproject.toml` and the CI workflow already |
| simulator guides | `user_guides/simulator/{cli_options,logs,advanced_simulation}.md` | Worth writing — derivable from `--help` and the debug API |
| appendices & strays | `appendices/{glossary,mks_parameters}.md`, `user_guides/library/movement.md`, `user_guides/library/robot_control.md` | `movement.md` is a stale duplicate of `movements.md` — drop it. `mks_parameters.md` duplicates the packaged manual spec — point at it. `robot_control.md` is worth writing from `robot_kinematics.py` |

**Workflow:** fix or delete, run `pytest tests/test_docs_api.py`, then delete the
entries it reports as stale. The gate is a ratchet — a new finding fails the
build and a baseline entry that no longer occurs *also* fails, so the count can
only shrink. Do not add entries.

**Done when:** the baseline is empty and `docs/README.md` has no *(planned)*
markers left, or the remaining entries are ones a maintainer has explicitly
decided to keep as debt with a note saying why.

---

## Item 3 — Publish to PyPI *(needs maintainer decisions and credentials)*

`pip install mks-servo-can` still fails; installation means cloning and two
editable installs from subdirectories. `REVIEW_NOTES.md` Part 1 ranks this as the
largest remaining barrier to anyone else using the library. L7 — the packaged
manual spec — was its prerequisite and is done.

What has to happen first, in order:

1. **A `pyproject.toml` per distribution**, replacing the two `setup.py` files.
   Keep `package_data`/`MANIFEST.in` behaviour: `mks_servo_can/data/*.json` must
   stay in both the wheel and the sdist.
2. **One source of the version number.** It is currently parsed out of
   `__init__.py` by a regex in each `setup.py`, and the two distributions can
   drift apart.
3. **A decision:** does the simulator stay a separate distribution, or become an
   extra of the library (`mks-servo-can[simulator]`)? It already hard-depends on
   the library. *This is the maintainer's call and blocks the rest.*
4. **A CI job that builds both and runs the suite against the installed
   packages**, not the source tree. This is what would have caught L7, and it is
   the only guard that keeps packaging honest.
5. Then publish — needs credentials this work cannot supply.

**Done when:** `pip install mks-servo-can` works in a clean virtualenv on a
machine that has never seen the repository, and the simulator starts from that
install with a non-empty `/commands`.

---

## Item 4 — Coverage where it is thinnest

Library coverage is 66%, up from 58%. The digitizer's base class went from 11% to
61% and turned up four defects (recorded as L9) — expect the same from the rest.
These modules are untested because nobody has run them, not because they are
simple.

Weakest first:

| module | coverage | what is untested |
|---|---|---|
| `digitizer/surface_mapping.py` | 17% | probing patterns, the surface model, the file formats |
| `digitizer/precision_analyzer.py` | 35% | pure statistics over `PlaybackStats` |
| `multi_axis_controller.py` | 44% | `move_linearly_to`, `home_all_axes`, the group error paths |
| `can_interface.py` | 55% | the hardware branch, which no test touches |
| `low_level_api.py` | 58% | the commands with no coverage at all |
| `axis.py` | 64% | homing, calibration, work-mode changes |

**Start with `precision_analyzer.py`:** pure functions, no simulator needed, and
L9 showed its thresholds are load-bearing — it calls anything under 50 ms of
timing error `EXCELLENT`, which is precisely the margin the playback bug was
consuming. Check the boundaries and the empty/one-sample cases.

Then `multi_axis_controller.py`, where the group error paths matter: it gathers
per-axis failures into `MultiAxisError.individual_errors`, and nothing tests that
a partial failure is reported rather than swallowed — the house defect again.

**Done when:** each module worked on has tests that fail against the code as it
was, and any defect found is recorded in `REVIEW_NOTES.md` and `CHANGELOG.md`.
Coverage is the symptom, not the goal; a module at 90% whose tests assert only
that calls were made is worse than one at 40% with three tests that assert
effects.

---

## Item 5 — Deterministic simulated time *(largest; do last)*

`tests/determinism/` has been an empty placeholder since May 2025 — the same
state `tests/hil/` was in before it was built out. Every simulator test paces
itself with `asyncio.sleep` against a wall clock, which is why the suite takes
~2 minutes and why the timing-sensitive tests will eventually go flaky under CI
load. Several tests added recently — the default-speed comparisons, the playback
timing checks — are wall-clock measurements that a `--step` control would turn
into exact assertions.

A `--step` / `/step` control that advances simulated time explicitly would make
those tests deterministic and fast. It is also the single change that would most
improve the simulator for an agent driving it: right now an agent has to *guess*
how long to wait after issuing a command before reading back state.

Sketch: the motor model's integration tick takes its `dt` from a clock object
rather than `time.monotonic()`; the default clock is real time, and `--step` swaps
in one that only advances when `/step` is called. The debug API gains
`POST /step {"seconds": 0.1}` returning the resulting snapshots, so an agent can
command, step, and read with no sleeping at all.

**Done when:** at least the timing-sensitive tests run under stepped time, take
no wall-clock time, and give identical results across runs; and `/step` is
documented in the simulator guide.

---

## Suggested sequencing

1. **Item 1 whenever the bench is free** — it is asynchronous on everything else
   and is the only item that can invalidate work already done.
2. **Items 2 and 3 together** as a release-readiness pass. Item 3 step 3 needs a
   maintainer decision before the rest can proceed.
3. **Item 4** continuously, a module at a time.
4. **Item 5** when the rest is quiet; it touches the simulator's core loop.

## Definition of done — the standard the fixed defects were held to

- Each fix has a test that fails against the code as it was, driving the
  simulator or real objects rather than a mock.
- Each test verified by mutation: undo the fix, confirm the test fails, confirm
  the mutation landed, restore. A test that passes under mutation is not a test.
- `pytest -q` green in random order, and `ruff check .` clean.
- `REVIEW_NOTES.md` Part 0 updated with what was fixed and how it is covered,
  keeping the original reproduction — the evidence is what makes a fix checkable.
- `CHANGELOG.md` updated under `[Unreleased]`.
