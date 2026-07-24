# mks_servo_can — Review Notes & Camera Gimbal Design

Review dated 2026-07-24, against commit `e4f11df`. **Most of Part 1 has since
been acted on** — see `CHANGELOG.md` for what changed and why. This file is kept
for the design reasoning in Part 2, and for the outstanding items below.

A second review on 2026-07-24, against `b28dded`, found the defects in Part 0.
All of them have since been fixed, along with one more (L8) found while building
a test for another. Each entry keeps its original reproduction, because the
evidence is what makes the fix checkable; `docs/development/roadmap.md` says what
remains.

---

# Part 0 — Second review: open library defects

Each of these was reproduced against the simulator, not inferred. None was
covered by a test when it was found; each entry says whether it has since been
fixed.

## L1. Re-targeting a move in flight always fails *(critical)* — **fixed**

Fixed in `can_interface.py` and `axis.py`, with
`tests/integration/test_move_supersede.py` covering both halves. A credit is now
a `_StaleCredit(statuses, expires_at)`: it only claims frames whose status byte
is one of `const.ASYNC_MOVE_NOTIFICATION_STATUSES`, so it can never consume the
acknowledgement, and it expires after
`const.STALE_NOTIFICATION_TTL_SECONDS`, so a motor that emits no abort at all
cannot poison a later reply. That form is correct under either answer to
question 1 of the hardware trace, which is still unrecorded.

The original finding, for reference:

`CANInterface.expect_stale_notification` discarded frames by **arrival order**,
but the motor emits the new command's acknowledgement *before* the superseded
move's abort frame. So the credit swallows the acknowledgement and the abort is
read as the reply:

```
rx ID=001 CMD=F5 Data=f501f7   → discarded as "stale"  (status 01 = STARTING: the real ack)
rx ID=001 CMD=F5 Data=f500f6   → resolves the ack future (status 00 = FAIL: the old move's abort)
MotorError: Motor run command F5 ... failed to start (status 0x00)
```

Reproduce: enable a motor, dispatch a long move with `wait=False`, then dispatch
a second move before it completes. Every retarget raises.

This is the behaviour the whole streaming design rests on. `ServoStream` is
unaffected because it never waits for acknowledgements, but every `Axis`-level
retarget fails.

The mirror-image failure is just as bad: if real hardware emits *no* abort frame,
the credit never expires and swallows the next legitimate response instead.
Nothing calls `clear_stale_notifications` automatically.

`tests/unit/test_regressions.py` asserted only that the credit was *requested*,
using a `MagicMock`, which is why this shipped: the bug was 100% reproducible and
the test could not see it.

The simulator reports the event explicitly (`move_superseded` in `/status`'s
`errors`), which is how the failure was made visible while it was being fixed.

## L2. `save_or_clean_speed_mode_params` (0xFF) always times out — **fixed**

The special case is gone; `tests/integration/test_speed_mode_params.py` covers
both sub-commands against the simulator. The unit test that existed had asserted
the wrong expectation outright, so it had to be corrected too.

The original finding, for reference:

`_send_command_and_get_response` special-cased 0xFF to expect the echoed
*sub-command* (0xC8/0xCA), but the motor echoes 0xFF — as the manual states and
as the comment above the status check in `save_or_clean_speed_mode_params`
already said, contradicting the code.

```
CommunicationError: Timeout waiting for response to command FF from CAN ID 001
(expected echoed cmd code C8). Sent: ffc8c8
```

The command has no test coverage at all.

## L3. Default speed is interpreted in the wrong units — **fixed**

Both handlers now convert only a speed the caller actually supplied, and use
`default_speed_param` as the MKS parameter it is otherwise.
`tests/integration/test_default_speed.py` times a default-speed move against the
same move at the speed that default is supposed to mean.

The original finding, for reference:

In `_move_absolute_handler` and `_move_relative_handler`, when `unit == 'user'`
and no speed was given, `default_speed_param` (500, an MKS parameter meaning
roughly 500 RPM) was passed through `kinematics.user_speed_to_motor_speed()`,
which read it as 500 deg/s and returned **83**. The guard `if sp is not None` was
dead — `sp` is never `None`, so the documented fallback never ran. Any
`move_to_position_abs_user()` without an explicit speed ran at a sixth of the
documented default.

## L8. An absolute move can be silently skipped against a stale position cache — **fixed**

Found while building the test for L3, and not in the original review.

`move_to_position_abs_*` short-circuits when the target is where the axis
already is, using the cached position so the check costs no CAN traffic. But
`wait=True` returns as soon as the completion frame resolves the move future,
while the cache is refreshed a moment later in the watcher's `finally`. Commanding
the starting position immediately after a move therefore compared the new target
against a *pre-move* cache, found them equal, and returned success without
sending anything. Reproduced with a 900° move followed by a move back to 0: the
second call returned in 0.000 s with the axis still at 900°.

The cache now carries a freshness flag. Dispatching a move, entering speed mode,
stopping and emergency-stopping all clear it; a read or a zeroing sets it. The
short-circuit consults the cache only while it is fresh, and otherwise simply
dispatches the move — which costs the same single round trip an encoder read
would, so the hidden pre-read that 0.3.0 removed does not come back.

## L4. A single bad frame kills the receive path — **fixed**

Both listeners now catch per message, log the frame, and carry on;
`tests/unit/test_can_interface_listener.py` drives the real listeners over a real
`StreamReader` and a real `Queue`.

The original finding, for reference:

In `_listen_for_messages_hw` and `_listen_for_messages_sim`, an exception raised
by `_process_received_message` escaped the per-message `try` and was caught by
the outer `except Exception`, which ends the listener task. One malformed frame
or one raising handler silently stopped all reception for the rest of the
session, and every later command then failed with a timeout, far from the cause.

It was reachable rather than theoretical: the homing predicate registered by
`Axis.home_axis` reads `data[1]` and raises `IndexError` on a one-byte frame.

## L5. Smaller items — **fixed**

- **`ServoStream.start()` was not atomic.** If it failed after disabling
  responses on some axes, `__aexit__` never ran and `stop()` early-returned on
  `_running == False`, leaving those motors mute for the rest of the session.
  `start()` now restores what it silenced before propagating.
  `tests/integration/test_stream_start_atomicity.py` provokes the failure with an
  axis on a CAN ID nothing answers, and checks recovery with a command CanRSP
  actually suppresses — an encoder read is answered either way and proves
  nothing.
- **`ServoStream._run_feedback` toggled `0x8C` around every read** — three round
  trips per axis per poll instead of one, and unnecessary: by the repo's own
  reading of the manual (`SUPPRESSIBLE_RESPONSE_COMMANDS`), `0x31` is not
  suppressible and always answers. The toggling is gone. Because that reading is
  from the manual rather than from hardware, repeated feedback failures while
  responses are disabled now log a warning naming the possibility instead of
  disappearing into debug records.
- **`can.interface.Bus(bustype=...)`** is deprecated in python-can 4 and removed
  in 5; the argument is now `interface`.
- **Stale docstrings** on `get_current_position_steps` (claimed to invert the
  value it reads) and `move_to_position_abs_pulses` (claimed to emulate absolute
  motion via 0xFD) corrected.
- **Per-frame `logger.info` with eager f-strings** converted to lazy `debug` in
  the eight remaining motion commands, and `HOT_PATH_FUNCTIONS` in
  `tests/unit/test_regressions.py` extended to cover all of them, so the next one
  cannot slip back in.

## L6. Documentation does not match the API *(now gated, debt outstanding)*

`tests/test_docs_api.py` now enforces this, as a ratchet over
`tests/fixtures/docs_known_issues.json`: new problems fail the build and the
baseline can only shrink. The findings below remain to be burned down — see
`docs/development/roadmap.md` item 3 for the workflow.

A mechanical check of the code blocks in `docs/` and `README.md` found **26
references to things that do not exist** across 12 files, plus 7 blocks that do
not parse as Python (so the real count is higher). Examples:
`LinearKinematics(steps_per_mm=)`, `RotaryKinematics(units=)`,
`axis.move_absolute()`, `axis.get_current_position()`, `axis.set_kinematics()`,
`axis.update_status()`, `CANInterface(enable_crc=)`, and `BaseKinematics` (the
exported name is `Kinematics`). `docs/user_guides/library/reading_status.md`
alone references seven non-existent `Axis` methods.

## L7. MANUAL_SPEC_NOT_PACKAGED - **fixed**

**Fixed.** The specification now ships as package data at
`mks_servo_can/data/manual_commands_v106.json`, read through
`mks_servo_can.manual_spec` with `importlib.resources` - so it works from a
wheel, a zip import or a checkout. The simulator, the debug tools and the
conformance tests all go through that one loader, and
`tests/unit/test_manual_spec_packaging.py` fails if any module starts naming the
file by path again. Verified by building a wheel, installing it into an empty
virtualenv and loading all 18 commands with no repository present.

`debug_tools.py` was found to have a second copy of the same bug on top of the
first: it looked along three relative paths under `tests/` *and* read `commands`
as a list when it is a mapping keyed by hex code, so it had always silently
fallen back to a hard-coded table that misnames several commands - 0x80 as
"Enable Motor" (it is calibrate) and 0xFD as absolute (it is relative).

The original finding, for reference:

`llm_debug_interface.py` loaded the command specification from
`tests/fixtures/manual_commands_v106.json`. An installed wheel has no `tests/`
directory, so `/commands` and `available_commands` were empty for anyone who did
not clone the repository.

---

# Part 1 — Repository Review: what remains

## Done

The five critical defects (non-blocking dispatch, response correlation, the
hidden pre-read round trip, the class-scope statement, and the Python 3.8
breakage), the per-frame INFO logging, the cached event loop, the mode-blind
move timeout, the duplicated motion model, the stale files, the missing CI, the
never-run linters, and the empty `tests/hil/` are all addressed. The suite went
from 223 tests at 53% coverage to 478 at 58%.

## Outstanding, in priority order

1. **Record a hardware trace.** This is the single highest-value thing left. The
   simulator is validated against the *manual*, and the library was written from
   the same reading, so a shared misreading is invisible to every test in the
   repo. One capture closes the loop and then runs in CI forever with no
   hardware attached:

   ```
   export MKS_HIL_CHANNEL=can0
   pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json
   ```

   Two specific questions only hardware can settle, both already written as
   tests in `tests/hil/test_hardware_conformance.py`:
   - **Sign convention.** Manual V1.0.6 contradicts itself; the fixture now
     follows the worked examples (CCW positive) but that is an inference.
   - **Does 0xF5 really accept a retarget mid-move**, and does the motor emit an
     abort frame for the superseded one? The whole streaming design and the
     gimbal example rest on this.

2. **Publish to PyPI.** `pip install mks-servo-can` still fails; installation
   means cloning and two editable installs from subdirectories. This is the
   largest remaining barrier to anyone else using the library.

3. **Test the digitizer.** `base_digitizer.py` is at 11% coverage and
   `surface_mapping.py` at 17% — by far the weakest area, and the one most
   likely to harbour the same class of defect that Part 1 found elsewhere.
   `can_interface.py` (47%) and `multi_axis_controller.py` (44%) are next.

4. **Split the plotter application out.** The SVG/calligraphy/height-map/
   digitizer cluster is a pen-plotter application, not a motor library. It is
   the majority of `examples/` and it is what a first-time visitor sees. A
   separate repo, or an `applications/` subtree, would let the core read as what
   it is.

5. **Publish the docs.** `docs/` has 27 markdown files in a sensible tree and
   `setup.py` declares a `[docs]` extra with Sphinx, but there is no `conf.py`
   and nothing is built. Point Sphinx or MkDocs at it and ship to Read the Docs.

6. **Motion profile primitives.** No jerk-limited or S-curve planning.
   `move_linearly_to()` scales per-axis speeds but cannot compensate for the
   axes' independent acceleration ramps, so the path bows at every corner.
   `motor_profile` now provides the units needed to do this properly.

7. **Enable the cosmetic lint rules** with a single `ruff format` pass, at a
   moment when nothing is in flight. The rules and the reasoning are recorded in
   the ignore block in `pyproject.toml`.

8. **Remaining safety gaps.** `Axis` still has no soft position limits (only
   `ServoStream` does), and `CANInterface` has no `async with` support to
   guarantee disconnect.

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
