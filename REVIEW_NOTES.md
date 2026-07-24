# mks_servo_can — Review Notes & Camera Gimbal Design

Review date: 2026-07-24. Based on commit `e4f11df`, Python 3.11.6, full test suite run,
and live benchmarking against the bundled simulator.

---

# Part 1 — Repository Review

## 1.0 Summary judgement

The **structure** is good: the layering (`crc` → `low_level_api` → `axis` → `multi_axis_controller`,
with kinematics injected) is the right decomposition, the CAN command coverage is genuinely
comprehensive, and having a simulator at all puts this ahead of most hobby motor libraries.

The **execution** has the signature of unsupervised AI generation: 223 tests pass but they mostly
assert that the code does what the code does; docstrings describe behaviour the code does not have;
and the two most load-bearing behaviours in the whole library — non-blocking moves and
request/response correlation — are broken in ways no test catches.

Test suite: **223 passed, 53% line coverage.** Coverage is concentrated in the pure functions
(crc 100%, constants 100%, robot_kinematics 84%) and absent where the risk is
(`base_digitizer.py` 11%, `surface_mapping.py` 17%, `multi_axis_controller.py` 44%,
`can_interface.py` 47%).

---

## 1.1 Critical bugs

### C1. `wait=False` does not return early — it blocks for the entire move

**Severity: critical.** This is the single worst defect.

`Axis._execute_move()` contains an unconditional `await asyncio.wait_for(completion_future, ...)`
(`axis.py:573`). The `wait` parameter is checked only *afterwards*, in
`_move_relative_handler` / `_move_absolute_handler`, to decide whether to additionally await
`self._active_move_future`. By that point the move has already physically finished.

Measured against the simulator:

```
move_to_position_abs_user(90 deg @ 60 deg/s, wait=False) returned after 1535.3 ms
                                    (should return in < 1 ms; move takes ~1500 ms)
```

Consequences:
- `wait=False` is a no-op across the entire public API.
- `MultiAxisController.move_all_to_positions_abs_user()` is built on a two-phase
  "dispatch all with `wait=False`, then gather completions" design (`multi_axis_controller.py:476`).
  Phase 1 already blocks, so Phase 2 is dead code and the `asyncio.gather` only accidentally
  preserves concurrency.
- Any streaming, jogging or servo-loop use is impossible.

**Fix:** `_execute_move` must split into *dispatch* (send command, create the completion future,
return) and *await* (consume the future). Only the latter is conditional on `wait`.

### C2. Responses are correlated only by `(can_id, command_code)` — asynchronous completion
messages are mistaken for command acks

**Severity: critical for any repeated-command use.**

`CANInterface._process_received_message()` (`can_interface.py:615`) matches an incoming frame to
the first pending future registered for `(arbitration_id, data[0])`. The MKS protocol reuses the
same command byte for the *synchronous ack* (`0x01 STARTING`) and the *asynchronous completion
notification* (`0x02 COMPLETE` / `0x00 FAIL`) of a move. There is nothing to tell them apart.

When a new `0xF5` supersedes a move in flight, the motor emits a `FAIL` for the old move. That
`FAIL` is delivered to the *new* command's future, and `_run_motor_command` raises `MotorError`.

Measured against the simulator, streaming absolute-position targets:

```
streaming 0xF5 at 500/200/100/50 Hz  :  0/60 spurious FAIL acks
streaming 0xF5 with NO pacing        : 29/60 spurious FAIL acks  (48%)
```

The bug is latent at fixed rates against the simulator (whose timing is regular), but a real motor
with real acceleration ramps and `CanRSP` active will hit this. It is a design flaw, not a race
that tuning can fix.

**Fix:** either (a) disable active responses (`0x8C`) on the streaming path and use fire-and-forget,
or (b) tag futures with a monotonically increasing sequence and match on arrival order per command
class, or (c) route `STARTING` and `COMPLETE/FAIL` through separate registries. (a) is what the
gimbal wants — see Part 2.

### C3. `_move_absolute_handler` inserts a hidden extra round-trip on every absolute move

`axis.py:729` calls `await self.get_current_position_steps()` before every absolute move, purely to
decide "is a move necessary". That is a full CAN request/response round trip added to the
latency of every commanded move, and it makes an absolute-position command depend on stale
feedback. On a real bus at 500 kbit/s this is ~0.5–1 ms plus adapter jitter, doubled.

**Fix:** use the cached `_current_position_steps` for the "already there" check, or drop the check
entirely — the motor already no-ops a zero-distance absolute move.

### C4. `multi_axis_controller.py:508` — statement at class-body scope

```python
    logger.info("Multi-axis absolute move command sequence finished.")
```

This is indented at class level, not inside `move_all_to_positions_abs_user`. It executes once at
**import time** and never at the end of the method. Harmless in effect, but it is proof that
nothing in the test suite exercises the end of that method, and it is exactly the kind of artefact
that indicates the file was edited by a model without being read.

### C5. `crc.py` uses PEP 585 annotations but the package claims Python 3.8 support

`calculate_crc(can_id: int, data_bytes: list[int])` and `verify_crc` (`crc.py:7`, `crc.py:41`).
Bare `list[int]` in a signature is evaluated at def time and raises `TypeError` on 3.8.
`setup.py` declares `python_requires=">=3.8"` and ships a `Programming Language :: Python :: 3.8`
classifier. The library cannot import on 3.8.

**Fix:** either `typing.List[int]` / `from __future__ import annotations`, or drop the 3.8 claim.
(Only these 2 sites are affected.)

---

## 1.2 Correctness and design issues (high, not critical)

### H1. Docstrings routinely describe behaviour the code does not have

These are worse than missing docstrings, because they are confidently wrong:

| Location | Docstring claims | Code actually does |
|---|---|---|
| `axis.py:750` `move_to_position_abs_pulses` | "emulates absolute positioning by leveraging the relative pulse command (0xFD) as a workaround" | routes to `_move_absolute_handler`, which uses `0xF5` absolute-axis |
| `axis.py:1147` `get_current_position_steps` | "inverts the value read from `read_encoder_value_addition`" | returns it unmodified |
| `robot_kinematics.py:615` `RRRArm.inverse_kinematics` | "This is a placeholder and requires specific implementation" | fully implemented |
| `axis.py:808` `move_relative_user` | "uses the MKS relative motion by pulses (0xFD)" | uses `0xF4` relative-axis |

Every one of these would mislead someone choosing a command for timing-sensitive work.

### H2. Speed conversion ignores microstepping, and the physics model lives in the wrong package

`RotaryKinematics.user_speed_to_motor_speed()` maps deg/s → RPM → speed parameter as an identity.
The manual (§6.1) states the speed parameter is *calibrated for 16/32/64 subdivisions only*:
at 8 subdivisions `speed=1200` yields 2400 RPM; at 128 it yields 150 RPM. The library never
consults `mstep_value` when converting speed, so a user who sets MSTEP=8 silently gets 2× the
commanded speed.

Meanwhile the **simulator** has the correct helpers — `mks_speed_param_to_rpm()` and
`mks_accel_param_to_rpm_per_sec_sq()` (`motor_model.py:178`, `:204`) — including the
`t = (256-acc) × 50 µs per RPM` acceleration law. The physical model of the motor lives in the test
double instead of in the library, so the library and simulator can drift apart silently.

**Fix:** move both conversions into `mks_servo_can` (e.g. `motor_profile.py`), have the simulator
import them, and make them mstep-aware. There is currently **no way at all** to convert an MKS
`acc` parameter into deg/s² from the library — which is required to plan any motion.

### H3. Per-frame logging at INFO with eager f-string formatting

`low_level_api.py:130`, `can_interface.py:494`, `can_interface.py:628`, `axis.py:501/551/557` all
emit `logger.info(f"...{msg.data.hex()}...")` for **every frame**. Two problems:

1. Semantically wrong: a library must not log every I/O operation at INFO. Any application that
   calls `logging.basicConfig(level=logging.INFO)` gets thousands of lines per second.
2. The f-strings are formatted eagerly, so the `.hex()` calls and string building happen even when
   the record is discarded. Measured cost on the position-read path: 0.257 ms → 0.293 ms median
   (~14%) with the output going to `/dev/null`; far worse with a real handler.

**Fix:** demote to `DEBUG` and use lazy `%s` interpolation, or guard with
`if logger.isEnabledFor(logging.DEBUG)`.

### H4. No fire-and-forget send path

Every `LowLevelAPI` method awaits a response. There is no way to send a command without a round
trip, even though `0x8C` (`set_slave_respond_active`) exists to turn responses off. This caps the
achievable command rate at half of what the bus can carry and forces the correlation problem of C2.

### H5. `asyncio.get_event_loop()` in constructors

`Axis.__init__` (`axis.py:136`) and `CANInterface.__init__` (`can_interface.py:234`) call
`asyncio.get_event_loop()`. Fine on 3.11, deprecated with a warning on 3.12/3.13, and removed in
3.14 when no loop is running. Constructing an `Axis` outside a coroutine will eventually be a hard
error. Capture the loop lazily at first use (`asyncio.get_running_loop()`), not in `__init__`.

### H6. `_execute_move`'s dynamic timeout hardcodes VFOC

`axis.py:511`: `max_rpm_reference = const.MAX_RPM_VFOC_MODE  # TODO: Make this mode-dependent`.
In `CR_OPEN`/`SR_OPEN` (400 RPM max) this underestimates move duration by 7.5×, so long moves in
open-loop mode will spuriously time out. The axis never reads back the work mode it is in.

---

## 1.3 What to remove

**Delete outright:**

- `tests/**/*.py.bak` (5 files, ~2900 lines) — near-duplicates of the live tests. `test_axis.py.bak`
  is 788 lines vs 813 live. These are checked into git and will rot.
- `simulator.log`, `manual_dashboard_debug.log`, `examples/simulator.log` — runtime artefacts,
  committed. `.gitignore` already has `*.log`; they were force-added or added before the rule.
- `docs/CODE_REVIEW.md`, `docs/CODE_REVIEW_DOCSTRINGS.md`, `docs/UNUSED_CODE_REPORT.md` — these are
  AI review transcripts, not documentation. They are stale (`CODE_REVIEW.md` claims
  `MOTOR_STATUS_MAP` is undefined; it is defined at `constants.py:168`) and they tell readers
  about the *process* rather than the *product*. Anything still true belongs in the issue tracker.
- `docs/manual_extracted.txt` (3994 lines) — raw `pdftotext` dump of a copyrighted manual. Keep the
  handful of tables you actually depend on as structured data (you already have
  `tests/fixtures/manual_commands_v106.json` — use that), and drop the dump. Note `.gitignore`
  already excludes `docs/*.pdf`, so the PDF itself is correctly untracked.
- `examples/motor_digitizer_compat.py` (746 bytes) — a `sys.path` shim.

**Consolidate:**

- `examples/` is 23 files / ~5000 lines and is now the largest surface in the repo. There are two
  height-map generators (`height_map_generator.py`, `..._v2.py`), two SVG plotters
  (`svg_plotter.py`, `enhanced_svg_plotter.py`), and two calligraphy plotters. Keep one of each
  and delete the superseded version — a new user cannot tell which one to read.
- The plotter/calligraphy/height-map/digitizer cluster is a *pen-plotter application*, not a motor
  library. It is ~60% of the example code and it is what a first-time visitor sees. Consider
  splitting it into a separate `mks-servo-plotter` repo (or an `applications/` subtree) so the
  core library reads as a motor library.

**Fix the packaging:**

- `pyproject.toml` contains only tool config (black/isort/pylint/flake8) — no `[project]` table.
  Packaging is done by two legacy `setup.py` files in subdirectories. Move to a real PEP 621
  `[project]` table.
- `.pylintrc`, `[tool.flake8]` in `pyproject.toml` (which flake8 does not read anyway), `black`,
  and `isort` are all configured and, judging by the code, none of them are run. Pick one —
  `ruff` replaces all four — and enforce it in CI.
- `check_docstrings.py` at the repo root is a one-off script; fold it into the linter config.

---

## 1.4 What is missing

Ordered by how much each blocks the library being adopted by someone other than you.

1. **CI.** There is no `.github/` directory at all. 223 tests exist and nothing runs them.
   A GitHub Actions matrix (3.9–3.13) running pytest + ruff + the simulator integration tests is
   perhaps two hours of work and is the highest-value single addition to the repo.

2. **Not on PyPI.** `pip install mks-servo-can` fails. Installation currently requires cloning and
   two editable installs from subdirectories. This is the largest single barrier to adoption.

3. **Hardware-in-the-loop tests that exist.** `tests/hil/` and `tests/determinism/` contain only
   `__init__.py`. The README advertises both. Everything is validated against a simulator that was
   written from the same manual reading as the library — so a shared misreading of the protocol is
   invisible. At minimum: one HIL smoke test you run manually before tagging a release, and a
   recorded `candump` trace from a real motor checked in as a fixture so the simulator can be
   validated against ground truth.

4. **A real-time / streaming API.** See Part 2. This is what the library needs to be good at
   something other than "run a job and wait".

5. **Motion profile primitives.** No jerk-limited or S-curve planning, no way to convert the
   `acc` parameter to engineering units (H2), no synchronised multi-axis interpolation beyond
   `move_linearly_to()`'s constant-velocity scaling — which computes per-axis speeds but cannot
   compensate for the axes' independent acceleration ramps, so the path bows at every corner.

6. **A changelog and versioning discipline.** `__version__ = "0.2.0"` with the comment
   "Minor version bump for new digitizer feature". No `CHANGELOG.md`, no tags, no deprecation
   policy. 115 commits, 38 of them from `google-labs-jules[bot]`.

7. **Docs that build.** `docs/` has 30 markdown files in a sensible tree, `setup.py` declares a
   `[docs]` extra with sphinx + myst-parser, and there is no `conf.py`, no `index.rst`, and nothing
   published. The README says documentation "is planned for the `docs/` directory" while 30 files
   sit there. Point Sphinx or MkDocs at it and publish to Read the Docs / GitHub Pages.

8. **README accuracy.** The project-structure block lists `rich_dashboard.py` and
   `interactive_controls.py`; the actual files are `textual_dashboard.py`, `sdk_client.py`,
   `config_manager.py`, `debug_tools.py`, `performance_monitor.py`, `llm_debug_interface.py`,
   `http_debug_server.py`. The "✅ NEW" markers are scattered through a document with no dates.
   The README is also very long and is mostly feature-listing; it needs a 10-line "here is a motor
   moving" opening.

9. **Type checking.** `mypy` is in the dev extras. The code is littered with `# type: ignore` on
   lines that would not need it if the `python-can` fallback shims were replaced by a proper
   `TYPE_CHECKING` guard + Protocol. Nothing appears to run mypy.

10. **Safety primitives.** For a library that drives motors: no soft position limits enforced in
    `Axis`, no watchdog/heartbeat (if the Python process dies mid-move, the motor keeps its last
    speed command in speed mode), no `async with` context manager on `CANInterface` to guarantee
    disconnect. `LimitError` exists but nothing raises it except a motor-reported end-limit.

---

## 1.5 Suggested order of work

**Now (correctness):** C1, C2, C3 → these three are what make the library unusable for real-time
work. C4, C5 are one-liners. Add a regression test per bug *first* — each of the four is easy to
pin against the simulator, and the fact that none of the 223 existing tests caught them is the
real finding.

**Next (credibility):** CI, PyPI, delete the `.bak`/log/review-report files, fix the README
structure block, publish the docs.

**Then (capability):** the streaming API (Part 2), motion profile primitives in engineering units,
HIL tests with a recorded real-motor trace.

**Ongoing:** demote the logging, adopt ruff, split the plotter application out of `examples/`.

---
---

# Part 2 — Three-Axis Camera Gimbal for Drone Tracking

## 2.0 The headline

Your instinct is that this is a speed problem. **It is a latency problem, and specifically a
*prediction* problem.** The MKS hardware has an order of magnitude more acceleration than you need.
The CAN bus has an order of magnitude more bandwidth than you need. What will actually determine
whether the drone stays in frame is how well you extrapolate its position forward through your
vision pipeline's delay.

Pointing error from pure transport delay is `ω × L`. For a drone at 30 m/s passing at 10 m
(ω = 172 deg/s):

| total latency | no prediction | constant-velocity predictor | constant-accel predictor |
|---|---|---|---|
| 10 ms | 1.72° | 0.011° | 0.0008° |
| 20 ms | 3.44° | 0.045° | 0.006° |
| 33 ms | 5.68° | 0.123° | 0.027° |
| 50 ms | 8.60° | 0.281° | 0.094° |
| 80 ms | 13.76° | 0.720° | 0.384° |

(Predictor residuals assume the target's unmodelled acceleration is 4 g lateral at 10 m = 225 deg/s².)

Read that table carefully. **Shaving 3 ms off the CAN command path buys you 0.5°. Adding an α–β
filter buys you 8°.** Do the filter first. Then optimise the transport.

## 2.1 Mechanical architecture

**Axis order (outermost → innermost):** base → **pan (yaw)** → **tilt (pitch)** → **roll** → camera.
Standard, and correct here: the pan axis carries the most inertia and moves the least in angle;
roll carries only the camera.

**Is roll the right third axis?** For *tracking*, pan + tilt is sufficient — roll only levels the
horizon. If your goal is a machine-vision tracker rather than cinematic footage, consider spending
the third motor on either (a) a **focus/zoom axis**, which matters far more for keeping a small
fast drone resolvable, or (b) a **coarse/fine dual-stage pan**, where a geared coarse stage handles
large slews and a direct-drive fine stage handles the tracking residual. Option (b) is how real
optical trackers are built and would be a genuinely novel demo for this library. If you do want
roll, keep it — just be aware it contributes nothing to acquisition.

**Gear ratio: use 1:1 direct drive on pan and tilt.** This is the key mechanical decision and it
goes against instinct.

| ratio | resolution | max output rate @1000 RPM (usable torque) |
|---|---|---|
| **1:1** | **21.97 mdeg (79 arcsec)** | **6000 deg/s** |
| 3:1 | 7.32 mdeg (26 arcsec) | 2000 deg/s |
| 5:1 | 4.40 mdeg (16 arcsec) | 1200 deg/s |
| 20:1 | 1.10 mdeg (4.0 arcsec) | 300 deg/s |

At 1:1 the 16384-count encoder gives 0.022° — that is 0.37% of a 6° FOV (roughly a 100 mm lens on
APS-C). Your pointing error budget is dominated by prediction residual (0.1–0.3°), not by encoder
quantisation (0.022°). Adding reduction buys you resolution you cannot use, costs you slew rate,
and — the real killer — **introduces backlash**, which is a nonlinearity your control loop cannot
compensate and which will show up as visible jitter every time the tracking error changes sign.
Only go to a reduction if you are running a genuinely long lens (< 2° FOV), and then use a
zero-backlash drive: harmonic drive, capstan, or a toothed belt under tension. Never a spur
gearbox.

**Motor selection** (balanced payload: 1.2 kg camera + short tele):

| axis | inertia | α available, SERVO42D | α available, SERVO57D | required |
|---|---|---|---|---|
| roll | 0.0019 kg·m² | 5968 deg/s² (27×) | 17905 deg/s² (80×) | 225 deg/s² |
| tilt | 0.0067 kg·m² | 1705 deg/s² (8×) | 5116 deg/s² (23×) | 225 deg/s² |
| pan | 0.0181 kg·m² | 633 deg/s² (3×) | 1899 deg/s² (8×) | 225 deg/s² |

(Using ~0.2 N·m for a SERVO42D and ~0.6 N·m for a SERVO57D, derated to ~50% for 600 RPM operation.)

→ **SERVO57D on pan, SERVO42D on tilt and roll.** A SERVO42D on pan gives only 3× margin, which
disappears the moment the gimbal is imperfectly balanced or you fit a heavier lens.

**Balance is not optional.** A stepper holding a static gravity torque burns holding current
continuously, heats up, and loses torque exactly when you need it. Balance each axis to within a
few grams·cm with adjustable counterweights, and check balance again after every lens change.

**Structural stiffness.** Direct-drive means the motor sees the structure's resonance directly.
Keep the first structural mode above ~3× your control bandwidth. For a 100 Hz loop that means the
first mode above 300 Hz, which means short stiff arms, no cantilevers, and metal — not printed
plastic — for the tilt yoke.

## 2.2 Motor configuration

- **Work mode `SR_vFOC` (mode 5, `const.MODE_SR_VFOC`)** — serial + field-oriented control,
  3000 RPM ceiling. The only sensible choice.
- **Microstepping 32 or 64** (`0x84`). The speed parameter is calibrated at 16/32/64 (manual §6.1),
  so staying in that band keeps `speed_param == RPM` true; 32/64 gives smoother low-speed motion
  than 16. Do **not** use 8 or 128 — the speed scaling changes and this library does not account
  for it (issue H2 above).
- **Enable subdivision interpolation (`0x89`)** for smoothness.
- **CAN bitrate 1 Mbit/s** (`0x8A`, code `0x03`) on all three motors and on the adapter. Short bus,
  twisted pair, 120 Ω at both physical ends only.
- **Acceleration parameter.** The law is `t = (256 − acc) × 50 µs` per 1 RPM step:

  | acc | time per RPM | 0→600 RPM | motor α |
  |---|---|---|---|
  | 236 | 1000 µs | 600 ms | 6000 deg/s² |
  | 250 | 300 µs | 180 ms | 20000 deg/s² |
  | 254 | 100 µs | 60 ms | 60000 deg/s² |
  | 255 | 50 µs | 30 ms | 120000 deg/s² |
  | 0 | — | instant | ∞ (will slip / jerk the camera) |

  **Use acc = 250–254.** You need ~225 deg/s²; acc=250 gives 20000 deg/s². Never use acc=0 —
  the step discontinuity will both jar the footage and risk losing sync. There is deliberate
  headroom here: the extra acceleration is what lets a streamed position command be tracked
  faithfully within one control period.

## 2.3 Control architecture

**Do not use `Axis.move_to_position_abs_user()` for tracking.** Because of bugs C1 and C3, each
call blocks for the whole move and inserts an extra round trip. You need a separate code path.

### The streaming loop

```
[camera] --frames-->  [detector]  --(az,el,t_capture)-->  [α-β / Kalman predictor]
                                                                    |
                                                          predicted (az,el) at t_now + L
                                                                    |
                                                          [gimbal servo task @ 200 Hz]
                                                                    |
                                            3 × 0xF5 absolute-axis frames, fire-and-forget
```

**Key protocol decision: disable slave responses on the streaming path.**
Send `0x8C` with `respond=0, active=0` before entering tracking mode. This:

- halves the frame count (no acks),
- **eliminates bug C2 entirely** — there are no async completion messages to be confused with acks,
- removes the head-of-line blocking of `send_and_wait_for_response`.

Frame budget with responses off, at 1 Mbit/s:

| traffic per cycle | theoretical | at 40% bus load |
|---|---|---|
| 3 × `0xF5` command only | 2572 Hz | **1029 Hz** |
| 3 × `0xF5` + 3 × `0x31` position poll | 1286 Hz | 514 Hz |
| current library (cmd + ack + poll + reply) | 643 Hz | 257 Hz |

A 200 Hz loop needs 8% of the bus. Bandwidth is a non-issue *once you stop waiting for acks.*

### How to use `0xF5` as a servo, not as a "go to and stop"

`0xF5` (absolute motion by axis) takes a target in raw encoder counts plus a speed and acceleration
parameter, and the manual notes it "supports real-time updates". If you re-issue a new target every
5 ms, the motor never completes any single move — it is continuously re-planning toward a moving
target. That is exactly what you want, but it means the *speed parameter is your primary control
input*, not an afterthought:

```
speed_param = clamp( |ω_predicted| * 60/360 * gear_ratio          # velocity feed-forward, RPM
                     + Kp * |θ_target − θ_measured| * some_gain,  # position correction
                     0, 3000 )
target_counts = θ_predicted * 16384/360 * gear_ratio
```

Set the speed from the *predicted angular rate* (feed-forward) plus a term proportional to the
tracking error. The motor's internal closed loop then handles the fine positioning. This is the
right division of labour: you do prediction and trajectory, the driver does the servo.

**Alternative:** `0xF6` speed mode with the position loop closed in Python. Cleaner control theory,
but it needs position feedback every cycle (double the frames) and you lose the driver's own
position loop. Start with `0xF5`.

### Feedback cadence

You do **not** need position feedback at the full loop rate. Poll `0x31` (encoder accumulated
value) on all three axes at 20–50 Hz, on a separate task, purely to (a) detect loss of sync,
(b) correct predictor drift, and (c) trigger a fault stop. The streaming loop runs open-loop on
predicted targets between polls.

**Do not differentiate the encoder to get velocity.** One-count quantisation at 1:1 becomes:

| loop rate | velocity noise from 1-count jitter |
|---|---|
| 50 Hz | 1.10 deg/s |
| 100 Hz | 2.20 deg/s |
| 200 Hz | 4.39 deg/s |
| 500 Hz | 10.99 deg/s |

At 200 Hz that is 2.6% of a 172 deg/s signal injected as noise straight into your derivative term.
Use the motor's own RPM register (`0x32`) or a filtered estimate from the predictor.

### The predictor

This is where the engineering effort belongs. An α–β filter is enough to start:

```
predict:  θ̂ = θ + v·Δt ;  v̂ = v
update:   r = θ_meas − θ̂ ;  θ = θ̂ + α·r ;  v = v̂ + (β/Δt)·r
output:   θ_cmd = θ + v·L        # L = measured total pipeline latency
```

Two things matter more than the filter's sophistication:

1. **Timestamp the measurement at capture, not at detection.** The camera driver should give you
   the frame's exposure-start timestamp. If you timestamp when the detector finishes, you have
   folded a variable delay into your measurement and no filter can undo it.
2. **Measure `L` empirically, don't guess it.** Put a blinking LED in frame, command a known
   step, and cross-correlate. Then feed the *measured* value into the extrapolation. Getting `L`
   wrong by 20 ms costs you 3.4° at 172 deg/s.

Upgrade to a constant-acceleration (or IMM) Kalman filter once the α–β version works — the table
in §2.0 shows it buys another 3× at high latency.

### Loop timing in Python

- asyncio gives ~1 ms of jitter typically, worse under GC. At a 5 ms period (200 Hz) that is 20%.
  Use `loop.call_at()` with an absolute schedule rather than `await asyncio.sleep(period)` so
  errors do not accumulate.
- Disable the library's INFO logging in the tracking loop (issue H3) — it is per-frame and eagerly
  formatted.
- Consider `SCHED_FIFO` for the servo task, and pin it to an isolated core.
- USB CAN adapters (CANable/gs_usb) add ~1 ms of latency and meaningful jitter. If jitter becomes
  the limiting term, move to a native SocketCAN peripheral (Pi + MCP2515 SPI HAT, or an SBC with
  on-die CAN). Measure before you buy: `cansniffer` timestamps will tell you.

## 2.4 What the library needs before it can drive this

Concretely, the gimbal needs these additions — all of which are also the right thing for the
library in general:

1. **Fix C1** — a true non-blocking dispatch. Without this nothing else matters.
2. **`LowLevelAPI` fire-and-forget send** (H4) — `send_no_wait()` alongside every command, used when
   `CanRSP` is off.
3. **A `ServoStream` / real-time controller class** — owns the fixed-rate loop, holds per-axis
   target state, packs and emits the three `0xF5` frames per cycle, and exposes
   `set_target(axis, position, feedforward_rate)`. Deliberately bypasses `Axis`.
4. **`motor_profile.py`** (H2) — `speed_param_to_rpm(param, mstep)`,
   `accel_param_to_deg_s2(acc, gear_ratio)` and their inverses, shared with the simulator.
   You cannot plan a trajectory without these and right now they only exist in the test double.
5. **Soft limits + watchdog** — a gimbal that keeps its last velocity command when the Python
   process dies is a gimbal that wraps its own cabling. Add a heartbeat that commands stop on
   timeout.
6. **Simulator fidelity for streaming** — `_handle_positional_move` currently cancels and re-plans
   on every superseded command and emits a `FAIL` for the old one. Verify against a real motor
   whether that matches hardware; if the hardware silently re-targets instead, the simulator is
   teaching you the wrong lesson about the exact behaviour the gimbal depends on.

## 2.5 Realistic expectations

With this design and a competent predictor:

- **Sustained tracking rate:** limited by the motors to well over 1000 deg/s; limited in practice by
  how fast your detector can keep the target in frame. Not the binding constraint.
- **Pointing error while tracking:** 0.1–0.3° dominated by prediction residual, assuming 30–50 ms
  total pipeline latency and a constant-velocity predictor. That keeps a target inside the central
  10% of a 6° FOV.
- **Static pointing repeatability:** ~0.02–0.05° (encoder quantisation plus stepper detent),
  provided the structure is stiff and balanced.
- **What will actually bite you:** balance drift after a lens change, structural resonance in a
  printed tilt yoke, cable routing torque on the pan axis, and a vision pipeline whose latency
  varies frame-to-frame. In that order.
