# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [semantic](https://semver.org/).

## [Unreleased]

Simulator observability, and the library defects that observability exposed.

### Fixed

- **Re-targeting a move in flight always raised a spurious `MotorError`.**
  Dispatching a move with `wait=False` and then dispatching another before it
  completed failed every single time — the behaviour the whole streaming design
  rests on. The stale-frame filter discarded by *arrival order*, but the motor
  emits the new command's acknowledgement (`POS_RUN_STARTING`) **before** the
  superseded move's abort (`POS_RUN_FAIL`), so the credit ate the
  acknowledgement and the abort resolved its future. A credit is now scoped to
  the status bytes an asynchronous notification can carry, so it cannot claim an
  acknowledgement, and it expires after `STALE_NOTIFICATION_TTL_SECONDS`, so a
  motor that emits no abort frame at all cannot poison a later reply — correct
  whichever way the hardware behaves, which is still unrecorded.
  The pre-existing "coverage" mocked `expect_stale_notification` and asserted
  only that a credit was *requested*; `tests/integration/test_move_supersede.py`
  drives a real axis against the simulator instead.
- **`save_or_clean_speed_mode_params` (0xFF) always timed out.** The transport
  was told to wait for the *sub-command* (0xC8/0xCA) to come back, but the motor
  echoes 0xFF and reports the outcome in the status byte — as the comment
  directly above the status check already said. The command had no integration
  coverage, and the unit test that existed asserted the wrong expectation.
- **An absolute move could be silently skipped.** `wait=True` returns as soon as
  the move future resolves, a moment before the cached position is refreshed, so
  commanding the position the axis started from was compared against a pre-move
  cache, matched, and returned success without sending anything. The cache now
  tracks whether it is still fresh, and the "already at target" shortcut only
  consults it while it is. When it is not, the move is dispatched rather than
  preceded by an encoder read, so the hidden round trip removed in 0.3.0 stays
  removed.
- **One bad frame stopped all reception for the rest of the session.** An
  exception raised while processing a single message escaped the per-message
  `try` in both listeners and ended the listener task; every command afterwards
  failed with a timeout, far from the cause. Reachable rather than theoretical:
  the predicate `Axis.home_axis` registers reads `data[1]` and raises on a
  one-byte frame.
- **A user-unit move with no speed ran at a sixth of its documented default.**
  `default_speed_param` is an MKS parameter, but both move handlers converted it
  as though it were a speed in user units — reading 500 as 500 deg/s and
  producing 83. The `if sp is not None` guard meant to prevent this could never
  fire, because `sp` had already been defaulted.

- **`--json-output` crashed on startup** with
  `AttributeError: 'SimulatedMotor' object has no attribute 'name'`, and
  `/status` and `/health` both returned HTTP 500 for the same reason.
  `llm_debug_interface` read **sixteen** attributes the motor has never had —
  `enabled`, `encoder_position`, `current_speed`, `is_moving` and the rest. All
  but `name` were wrapped in `getattr(..., default)`, so they did not raise;
  they reported zeros for a moving motor. The tests passed throughout because
  they built `MagicMock(spec=SimulatedMotor)` and then *assigned* the
  fictitious attributes, teaching the mock the API the code wished for.
- **`--config-profile` crashed** with `TypeError: SimulatedMotor.__init__() got
  an unexpected keyword argument 'max_current'`. The profile path forwarded
  three arguments the motor model does not accept. `max_current` and
  `initial_position` now map onto the attributes that do exist; `max_speed` is
  not mapped, because a real motor's speed ceiling comes from its work mode.
- **`/health` raised `KeyError`** reading `total_messages_sent`, a key
  `get_system_status()` has never emitted. The health check was the one endpoint
  guaranteed to report the service as unhealthy.
- **Importing the package printed a banner to stdout**, which in
  `--json-output` mode is the machine-readable event stream — the first line a
  consumer read was not JSON.
- **The manual command specification never loaded.** Its path was off by one
  directory, so `/commands` returned nothing and `available_commands` reported
  0, silently. Both the load failure and its cause are now logged.
- **Command names never resolved** in the history: the lookup tested
  `hasattr(debug_interface, 'MANUAL_COMMANDS')`, but that is a module-level
  global, so every command was labelled with its own hex code.
- **The manual specification had the wrong opcodes for the motion commands.**
  `0x3B` was named `go_home` (it reads the power-on zero status; homing is
  `0x91`, which was absent). `0xF4`/`0xF5` were named as *pulse* commands with a
  `pulses(int32)|speed(uint16)` layout matching no command in the manual — they
  act on the axis, in raw encoder counts, with
  `speed(uint16)|acc(uint8)|value(int24)`. `0xFD` was missing and `0xFE` was
  named `move_absolute_pulses_enhanced`, which is not a command.
  `tests/simulator_compliance/` had inherited the same errors, so it reported
  compliance for `0x3B`, `0xF4` and `0xF5` while actually sending `0x91`, `0xFD`
  and `0xFE` — the two axis commands the streaming API depends on had no
  coverage at all.
- **`record_error` had no callers anywhere**, so the `errors` array was
  permanently empty and every error display was decorative.

### Added

- **`SimulatedMotor.status_snapshot()`** returning a frozen `MotorSnapshot` —
  now the only supported way to observe a motor. Every reporting surface renders
  this and nothing else, which is what makes the drift above impossible to
  repeat. Angles derive from the motor's own encoder resolution rather than an
  assumed 16384.
- **A browser dashboard** at `/dashboard` when `--debug-api` is running: motor
  table, rolling plot of measured position against commanded target, named
  command log, and anomaly panel. One self-contained file — no CDN, no fonts, no
  external assets — so it works on a bench with no internet. It renders
  `/status`, the same payload an agent polls, so the human and machine views
  cannot disagree.
- **Anomaly reporting.** `SimulatedMotor.report_anomaly()` surfaces protocol
  events a client cannot deduce from the wire. The first is `move_superseded`:
  re-targeting `0xF5` mid-move makes the motor abort the old move with a frame
  carrying the same command byte as the acknowledgement of the new one, so the
  client sees an inexplicable failure. The simulator now names it and attaches
  both targets.
- Tests that would have caught all of the above: `test_motor_snapshot.py`
  (drives real motors and asserts the snapshot tracks them), `test_simulator_cli.py`
  (runs the real console script in a subprocess), and `TestEndpointsAgainstRealMotors`
  (serves the API from real motors with nothing stubbed in between).
  `test_llm_debug_interface.py` was rewritten against real motors.

### Documentation

- **`tests/test_docs_api.py` gates the documentation against the code.** Every
  Python block in `docs/` and `README.md` is parsed and its method names,
  constructor arguments and imports checked against the real package; internal
  links must resolve. None of this had ever been verified, and it had drifted
  badly — 26 references to things that do not exist, 7 blocks that are not valid
  Python, and 27 of the 52 links in the documentation index pointing at
  documents that were never written.
  It is a ratchet rather than a clean gate: the 63 known problems live in
  `tests/fixtures/docs_known_issues.json`, a new finding fails the build, and a
  baseline entry that no longer occurs also fails, with an instruction to delete
  it. So the documentation cannot get worse and the debt can only shrink.
- **The documentation index no longer advertises documents that do not exist.**
  Entries with no file behind them are marked *(planned)*, and the index says so.
- **Added `docs/development/roadmap.md`** — the open defects, the order to
  address them in, and how to verify each. `docs/README.md` had reserved a link
  for this since the beginning; it was one of the 27 dead ones.

### Known issues

Four library defects found in the same review are documented with reproductions
in `REVIEW_NOTES.md` Part 0 and are **not yet fixed**. The most serious, L1, is
that re-targeting a move in flight always raises a spurious `MotorError`,
because the stale-frame filter discards by arrival order and the acknowledgement
arrives before the abort. `docs/development/roadmap.md` sequences the work.

## [0.3.0] - 2026-07-24

A correctness and real-time capability release. Several defects fixed here made
the library unusable for anything that tracks a moving reference, and none of
them were caught by the 223 tests that existed beforehand.

### Fixed

- **`wait=False` never returned early.** `Axis._execute_move` awaited the move
  completion unconditionally, so the `wait` flag only controlled whether the
  caller *additionally* awaited an already-resolved future. A 90° move at 60°/s
  with `wait=False` took 1535 ms to return; it now returns in 0.3 ms.
  This also revives `MultiAxisController.move_all_to_positions_abs_user`, whose
  dispatch-then-gather design was dead code because phase one already blocked.
- **Asynchronous completion frames were mistaken for command acknowledgements.**
  The protocol reuses one command byte for both, so a superseded move's abort
  frame resolved the *next* command's future and surfaced as a spurious
  `MotorError` — 48% of commands in an unpaced stream. `CANInterface` now tracks
  expected stale notifications and discards them.
- **Every absolute move issued a hidden extra encoder read**, adding a full CAN
  round trip to the latency of each commanded move.
- `multi_axis_controller.py` had a `logger.info()` at class-body scope, running
  at import instead of at the end of the method it belonged to.
- `crc.py` used PEP 585 builtin generics, so the package could not import on
  Python 3.8 despite `setup.py` claiming support. The floor is now 3.9 and is
  enforced by a test.
- `Axis` and `CANInterface` resolved `asyncio.get_event_loop()` in `__init__`,
  deprecated on 3.12+ and an error on 3.14+ when no loop is running.
- Move timeouts hardcoded the vFOC 3000 RPM ceiling, underestimating duration
  7.5× in open-loop modes.
- The digitizer imported from the package root, a circular import that only
  resolved by accident of ordering in `__init__.py`.
- The simulator answered frames with an invalid CRC. Real motors drop them, and
  a simulator that does not lets a CRC bug in the library go unnoticed.
- The simulator ignored `CanRSP` (0x8C), so "fire-and-forget" still paid for
  every reply. Suppression now applies to the run commands only, per manual
  sections 6.4–6.8.

### Added

- **`mks_servo_can.realtime`** — fixed-rate streaming control.
  - `ServoStream` drives several axes from a fixed-rate loop, streaming absolute
    targets fire-and-forget with motor responses disabled. Soft limits, a
    producer watchdog, and loop-jitter statistics included.
  - `AlphaBetaTracker` and `AlphaBetaGammaTracker` extrapolate a delayed, noisy
    measurement stream forward by the measured pipeline latency.
- **`mks_servo_can.motor_profile`** — converts the opaque MKS speed (0–3000) and
  acceleration (0–255) parameters to and from RPM, deg/s² and ramp times,
  including the microstep calibration and work-mode ceilings that manual
  section 6.1 describes only in prose. The simulator now shares this model
  rather than carrying its own copy.
- `LowLevelAPI.run_position_mode_absolute_axis_no_wait`,
  `run_speed_mode_no_wait` and `stop_position_mode_absolute_axis_no_wait` for
  fixed-rate control loops. Measured 0.065 ms per command versus 0.452 ms for
  the request/response path.
- `examples/camera_gimbal_tracker.py` — a three-axis camera gimbal tracking a
  fast target, runnable against the simulator. 0.66° median pointing error at
  the crossing point versus 4.15° with no prediction.
- `tests/simulator_compliance/test_wire_format.py` — conformance assertions on
  raw frames (DLC, CRC, integer widths, byte order) rather than on parsed return
  values.
- `tests/hil/` — a real hardware-in-the-loop harness, gated behind
  `MKS_HIL_CHANNEL`, with motion tests further gated behind
  `MKS_HIL_ALLOW_MOTION`. Includes a trace-recording mode so the simulator can
  be replayed against captured hardware responses in CI with no hardware.
- GitHub Actions CI across Python 3.9–3.13, with ruff linting.
- This changelog.

### Changed

- Per-frame logging demoted from INFO to lazy DEBUG. The hot path emitted
  several eagerly-formatted records (including `.hex()` calls) per CAN frame, so
  applications that enabled INFO logging drowned and paid the formatting cost
  even when records were discarded.
- The simulator now hard-depends on `mks-servo-can` (a dependency it always had
  but did not declare) instead of falling back to 160 lines of stubbed
  constants, CRC and exceptions.
- Tooling consolidated on ruff, replacing separately configured and
  never-enforced black, isort, flake8 and pylint.
- **Errata:** manual V1.0.6 contradicts itself on sign convention. The prose
  above the 0x31/0x35 worked examples says clockwise is positive; the examples
  themselves and the 0x32 note say counter-clockwise is. The examples are taken
  as authoritative, and `tests/fixtures/manual_commands_v106.json` — which had
  propagated the wrong version — is corrected and carries an errata block.
  A HIL test exists to settle it against real hardware.

### Removed

- Five `.bak` test files (~2900 lines), three committed `.log` artefacts, three
  automated review transcripts in `docs/`, and a 4000-line `pdftotext` dump of
  the vendor manual.
- Two superseded examples whose successors are strict supersets
  (`height_map_generator.py`, `svg_plotter.py`).

### Known gaps

- The simulator is validated against the *manual*, not against hardware. Both it
  and the library were derived from the same reading, so a shared misreading
  stays invisible. Recording a trace with `pytest tests/hil --hil-record=...`
  closes this and is the single highest-value contribution available.
- Neither distribution is published to PyPI yet.
- `ruff format` is not enforced; see the ignore block in `pyproject.toml`.

## [0.2.0]

- Motor digitizer system: recording, playback, precision analysis, surface
  mapping.

## [0.1.0]

- Initial release: low-level CAN API, `Axis`, `MultiAxisController`, kinematics,
  robot models, and the simulator.
