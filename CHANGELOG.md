# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [semantic](https://semver.org/).

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
