# Roadmap — remaining work, in priority order

Repository: `/home/ricardodeazambuja/backup/GitStuff/mks_servo_can`
Branch: `library-hardening`, **nothing pushed** (no upstream tracking branch).
Baseline: 606 tests passing / 30 skipped, `ruff check .` clean, library coverage
68%, **documentation baseline empty** (63 → 26 → 0 known problems).

Context: a Python library for MKS SERVO42D/57D stepper drivers over CAN, plus a
simulator that emulates the same protocol so the library can be developed with no
hardware attached. Two reviews found defects in both. **All of them are now
fixed** — L1–L5 and L7 from the reviews, plus L8, L9, and L10–L13 found while
testing the fixes and while doing the work below. `REVIEW_NOTES.md` Part 0
records each with its original reproduction; `CHANGELOG.md` `[Unreleased]` says
what changed and why.

Note the pattern in those later numbers, because it is the most useful thing to
know about this codebase: **every one of them was found by a change that made
something previously invisible visible.** L10 by the first CI job to run against
an installed package rather than the source tree; L13 by the L11 fix, which
stopped a group operation discarding its errors. Nothing here was found by
reading the code.

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

## Item 2 — Publish to PyPI *(one step left; needs credentials)*

**The packaging is done.** One distribution declared in the root
`pyproject.toml`, with the simulator as its `[simulator]` extra — the
maintainer's decision, taken because the simulator hard-depends on the library
and a second distribution bought nothing but a second version number, which had
already drifted (0.1.0 against 0.3.0). Both `setup.py` files and the old
`MANIFEST.in` are gone.

Verified end to end: a wheel built, installed into an empty virtualenv, and the
simulator started from it serving 18 commands on `/commands`; the whole suite
run against the installed package rather than the source tree. That check is now
the `package` job in CI, and it found L10 the moment it existed — see
`REVIEW_NOTES.md`.

**What remains is the upload itself, which needs credentials this work cannot
supply.** When you have them:

```bash
python -m build
python -m twine check dist/*
python -m twine upload dist/*      # test.pypi.org first
```

Two things to decide at that point, neither blocking:

1. **The version to release under.** `mks_servo_can/__init__.py` says `0.3.0`
   and is now the single source; `CHANGELOG.md` `[Unreleased]` holds a
   substantial amount of change since it, including behavioural fixes, so this
   probably wants to go out as `0.4.0` with the `[Unreleased]` section renamed.
2. **Whether to automate it.** A `release` job on tag push, using PyPI trusted
   publishing, would avoid a long-lived token. The `package` job already builds
   and `twine check`s the artefacts, so this is a small addition.

**Done when:** `pip install mks-servo-can` works in a clean virtualenv on a
machine that has never seen the repository.

---

## Item 3 — Coverage where it is thinnest

Library coverage is 68%. The pattern holds without exception so far: every module
taken from "untested" to "tested" has produced a defect, and in two cases the
defect was the module's headline behaviour. `precision_analyzer.py` (35% → 98%)
gave L12, `multi_axis_controller.py` (44% → 61%) gave L11, and L11's fix
immediately exposed L13 in `axis.py`. These modules are untested because nobody
has run them, not because they are simple.

Weakest first:

| module | coverage | what is untested |
|---|---|---|
| `digitizer/surface_mapping.py` | 17% | probing patterns, the surface model, the file formats |
| `can_interface.py` | 56% | the hardware branch, which no test touches |
| `low_level_api.py` | 58% | the commands with no coverage at all |
| `digitizer/base_digitizer.py` | 61% | recording, the file formats, the error paths |
| `multi_axis_controller.py` | 61% | `move_linearly_to`, `home_all_axes`, sequential (`concurrent=False`) execution |
| `axis.py` | 66% | homing, calibration, work-mode changes |

**Start with `surface_mapping.py`**, now by far the weakest and completely
unexercised. It is also the one whose output a user acts on physically, so a
wrong height map is a crash into a workpiece rather than a wrong number.

**Then `low_level_api.py`**, where whole commands have no coverage at all and the
simulator can answer every one of them.

Note that `multi_axis_controller.py`'s remaining gap is mostly
`_execute_on_axes(concurrent=False)` and `move_all_relative_user`. The
concurrent path recovers axis names by splitting the asyncio task name on the
method name — worth a test with an axis whose name contains the method name,
which would truncate it.

**Done when:** each module worked on has tests that fail against the code as it
was, and any defect found is recorded in `REVIEW_NOTES.md` and `CHANGELOG.md`.
Coverage is the symptom, not the goal; a module at 90% whose tests assert only
that calls were made is worse than one at 40% with three tests that assert
effects. A test that still passes when you break the code it covers is not a
test — one written during this item did exactly that and was deleted rather than
kept (see L13).

---

## Item 4 — Deterministic simulated time *(largest; do last)*

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
2. **Item 2** as a release-readiness pass; the decision that blocked it is made.
3. **Item 3** continuously, a module at a time.
4. **Item 4** when the rest is quiet; it touches the simulator's core loop.

### Done and removed from this list

- **Documentation.** The `docs_known_issues.json` baseline is empty: every
  Python block in `docs/` and `README.md` names an API that exists and every
  internal link resolves. Nine pages were written (`development/*`,
  `user_guides/simulator/*`, `user_guides/library/robot_control.md`,
  `appendices/glossary.md`) and the rest of the index was repointed at the
  source, the docstrings and the scripts in `examples/` — things that cannot
  drift. See `CHANGELOG.md` `[Unreleased]` → Documentation.
- **Packaging** (item 2's steps 1–3). One distribution, one version number, one
  `pyproject.toml`, and a CI `package` job that builds it and runs the suite
  against the installed package. Only the upload is left, above.

## Definition of done — the standard the fixed defects were held to

- Each fix has a test that fails against the code as it was, driving the
  simulator or real objects rather than a mock.
- Each test verified by mutation: undo the fix, confirm the test fails, confirm
  the mutation landed, restore. A test that passes under mutation is not a test.
- `pytest -q` green in random order, and `ruff check .` clean.
- `REVIEW_NOTES.md` Part 0 updated with what was fixed and how it is covered,
  keeping the original reproduction — the evidence is what makes a fix checkable.
- `CHANGELOG.md` updated under `[Unreleased]`.
