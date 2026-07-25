# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [semantic](https://semver.org/).

## [Unreleased]

Simulator observability, and the library defects that observability exposed.

### Fixed

- **Protocol compliance covered 18 of the 49 commands the library implements
  (L17).** `mks_servo_can/data/manual_commands_v106.json` is what the compliance
  suite checks the wire format against, what `/commands` reports and what the
  command injector builds its specifications from — and it described eighteen
  commands. The framing of the other thirty-one, `0xF3` (enable), `0xF6` (speed
  mode) and `0x8C` (CanRSP) among them, was verified against nothing, and the
  test meant to catch exactly this could not: it looks for library methods
  matching commands the specification lists, so a gap in the specification was a
  gap in the check.
  All 46 real commands are now transcribed from the V1.0.6 manual. `0xC8` and
  `0xCA` are recorded in a `deliberately_absent` block, being values of `0xFF`'s
  argument rather than commands of their own. Two entries were wrong: 0x35's
  response was recorded as DLC 4 carrying a uint16 where the manual gives DLC 8
  carrying an int48 — the entry's own notes, describing a value that moves by
  0x4000 a revolution, contradicted it — and 0x80's request was recorded as a
  two-byte frame where the manual shows three and the printed CRC confirms it.
  Four contradictions inside the manual are recorded in `errata` rather than
  resolved silently.
  A new `TestRequestFraming` drives every command through the library and checks
  both the outgoing and the returning frame against the manual; it is what caught
  0x35. `test_every_implemented_command_is_transcribed` fails if a command is
  ever implemented without being transcribed. The compliance suite went from 104
  passing with 28 skipped to 142 passing with none skipped.
- **The simulator's command injector had never injected a command (L16).**
  `/inject`, `/inject_template` and `/templates` only became reachable when L15
  was fixed, and every one of the eleven templates failed against a real motor.
  Three independent causes. `_load_command_specs` copied the manual's DLC into
  `CommandSpec.data_length`, but DLC counts the command code and the CRC as well
  as the arguments — so `validate_command` demanded two data bytes from every
  command that takes none, and all eighteen transcribed commands were rejected.
  `inject_command` then `await`ed `SimulatedMotor.process_command`, which is
  synchronous and returns a `(can_id, payload)` tuple, and handed it a
  one-argument completion callback where the motor calls a two-argument one; the
  immediate reply was discarded. And eight of the eleven templates named one
  command while sending another — `enable` sent `0x80`, which is encoder
  calibration; the two positioning templates were described as moving *to* a
  position and sent `0xFD`, relative pulses, with four bytes where the frame
  takes six; the speed templates put the direction bit in the wrong byte; and
  the current templates passed a percentage to a command that takes milliamps.
  The hard-coded fallback specification table repeated the misnamings, so the
  two ways of loading specifications agreed with each other and disagreed with
  the manual.
  Templates and the fallback table now take their codes from `constants`, DLC is
  converted to a payload count, the callback has the signature the motor calls
  and the previous one is restored afterwards (an injection used to divert a
  connected client's completion frames to itself permanently), and an injection
  that asked for a reply and got none reports failure rather than success.
  Commands the packaged manual transcription does not yet cover — `0xF3`,
  `0xF6`, `0x83` — still answer `Unknown command code`; completing that
  transcription is item 5 in `docs/development/roadmap.md`. Covered by
  `tests/unit/test_command_injector.py` against a real bus and a real motor.
- **A multi-axis move reported success when the last axis failed (L11).**
  `MultiAxisController.wait_for_all_moves_to_complete` built its task list from
  the axes that were still moving, awaited them, and then *rebuilt* that list to
  map results back to axis names by position. The rebuild happened after the
  wait, by which point every axis that finished had dropped out of it —
  including every axis that finished by **failing**, since a future resolved
  with an exception is `done()`. The two lists were different lengths, so the
  index was wrong; and the bounds check that looked defensive was doing the
  damage, discarding any error at an index past the end of the shorter list.
  `errors_found` stayed empty, no `MultiAxisError` was raised, and the caller
  was told every move completed. With the failure anywhere but first, the error
  was either dropped or attributed to the wrong axis.
  Axes and coroutines are now paired once, before the wait, and results mapped
  through `zip`; the check also widened to `BaseException` so a cancelled wait
  is reported rather than counted as success. The sibling methods
  `move_all_to_positions_abs_user` and `move_all_relative_user` were checked and
  are correct. Covered by `tests/integration/test_multi_axis_group_errors.py`
  against three real simulated motors.
- **A superseded move's watcher wiped the state of the move that replaced it
  (L13).** `_supersede_active_move` cancels the previous move's completion
  watcher, but `task.cancel()` only *schedules* cancellation: the watcher's
  `finally` ran later, after a new move had been dispatched, and
  unconditionally cleared `_pending_move_command` — which by then belonged to
  the new move. The next supersede then found nothing to register a
  stale-notification credit against, so the abort frame it should have swallowed
  resolved the new move's future as `MotorError: move (CMD=F5) failed with
  status 0x00`. L1's symptom by another route; it made a digitizer playback fail
  on its last point. The same `finally` also refreshed the position cache, which
  marks it *fresh* — doing that while a new move is in flight is exactly L8's
  precondition. The whole block is now guarded on the watcher still owning the
  axis. Found immediately by the L11 fix above, which stopped the resulting
  error being discarded.
- **The precision analyzer graded a playback that never ran (L12).**
  `PrecisionAnalyzer.assess_precision` read only the position and timing errors,
  which are computed over the points that were *executed* — so a playback that
  managed one point of five hundred, accurately, was reported as `EXCELLENT`.
  That is the headline verdict `examples/basic_digitizer_demo.py` prints, and it
  sat in `generate_performance_report` directly beside an
  `execution_success_rate` of 0.002 that it ignored. A `COMPLETENESS_CAPS` table
  now bounds the grade by how much of the sequence actually ran; the caps only
  demote, never promote.
  In the same function, `execution_success_rate` divided by `planned_points`
  with nothing rejecting an empty sequence, so a recording that captured nothing
  raised `ZeroDivisionError`. It now yields 0.0 and grades POOR.
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
- **Digitizer playback never reached its last point, and said otherwise.**
  Points are dispatched without waiting, and the cleanup stopped every axis as
  soon as the loop ended, cutting the final move short — a sweep ending at 15
  degrees settled at 9.6. Playback now waits for that move, stops the axes only
  when something went wrong, and raises instead of printing `PLAYBACK COMPLETE`
  when it could not command a motor. `speed_factor=0` is rejected up front
  rather than dividing by zero mid-run, and the precision report no longer
  counts its own 50 ms settle delay as lateness — which had put every
  measurement at the boundary of the `EXCELLENT` threshold that judges it.
- **An installed package had no command reference.** The manual's transcription
  was read out of `tests/fixtures/`, which a wheel does not contain, so
  `/commands` returned nothing and `available_commands` reported 0 for anyone
  who installed rather than cloned. It now ships as package data at
  `mks_servo_can/data/manual_commands_v106.json` and is read through
  `mks_servo_can.load_manual_spec()`, which uses `importlib.resources` and so
  works from a wheel, a zip import or a checkout.
  The simulator's `debug_tools` had the same bug twice over — it also parsed the
  command table as a list when it is a mapping keyed by hex code, so it had
  always fallen back to a hard-coded table that misnames several commands.
- **A `ServoStream.start()` that failed partway left motors mute.** Responses
  are disabled axis by axis; a failure after the first one meant `__aexit__`
  never ran and `stop()` early-returned, so those motors stayed silent for the
  rest of the session. `start()` now restores what it silenced before
  propagating.
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

### Added — deterministic simulated time

- **`--step` and `POST /step`.** The simulated motors' `dt` now comes from a
  clock object rather than `time.monotonic()`. The default clock is wall time
  and behaves exactly as before; `--step` swaps in one that moves only when
  `POST /step {"seconds": 0.1}` says so, and returns the resulting `/status`
  payload once every motor has finished the last sub-step.
  Time advances in 10 ms sub-steps however large a step is requested, so
  stepping ten seconds integrates the acceleration ramp exactly as a hundred
  100 ms steps would. Motors are held at a barrier between sub-steps, so they
  all see the same `dt` and none can run ahead — a multi-axis result no longer
  depends on the order the event loop happened to schedule the motor tasks in.
  Simulated time is counted in whole nanoseconds rather than accumulated as a
  float, so ten 10 ms steps come to exactly 0.1 s and two identical runs agree
  bit for bit.
- **`tests/determinism/` is no longer an empty placeholder.** 39 tests that run
  a full motor model — acceleration ramps, multi-motor lockstep, the HTTP
  endpoint — in 0.4 s of wall clock, with exact assertions rather than
  tolerances, including one that simply asserts two identical runs produce
  identical output. No wall-clock test in this repository could state that.
- For an agent or a script driving the simulator this removes the guesswork
  entirely: command, step, read, with no sleeping and no window in which a motor
  might be caught half-updated.

### Fixed (simulator)

- **`/status` returned HTTP 500 for a legal acceleration setting (L14).**
  Acceleration parameter 0 means "no ramp, jump straight to speed", so its
  converted value is `math.inf` — correct, and not JSON. Starlette serialises
  with `allow_nan=False`, so one motor set that way took out `/status` and the
  browser dashboard with it. `MotorSnapshot.accel_deg_per_s2` is now
  `Optional[float]` and reports `None`. Never caught because every test motor
  used the default acceleration of 100.
- **Nine debug API endpoints had never worked (L15).** `http_debug_server.py`
  read `debug_interface.virtual_can_bus` in thirteen places; the attribute is
  `can_bus`. `/inject`, `/inject_template`, `/templates`, `/injection_stats`
  and `/run_scenario` returned HTTP 500. Worse, the four `/performance*`
  endpoints guarded on `hasattr(..., 'virtual_can_bus')`, which was always
  false, and so returned `{"error": "Performance monitoring not enabled"}` —
  plausible, actionable-looking and untrue — while monitoring was running
  perfectly well.
- The simulator's homing and stop delays are measured on simulated time, so
  `--step` does not leave them running against the wall clock.

### Packaging

- **One distribution.** `mks-servo-can` and `mks-servo-simulator` were two
  distributions installed from two subdirectories with a `setup.py` each. They
  are now one: `pip install mks-servo-can[simulator]`. The simulator hard-depends
  on the library — it shares its constants, CRC and motion model rather than
  reimplementing them — so a second distribution bought nothing but a second
  version number, which had already drifted (the simulator said 0.1.0 while the
  library said 0.3.0). `mks_simulator.__version__` now re-exports the library's,
  and the packaging job asserts they agree.
- **Everything moved into the root `pyproject.toml`**, replacing both `setup.py`
  files and `mks_servo_can_library/MANIFEST.in`. The packages keep their
  historical directories, mapped through `package-dir`; moving them into a
  `src/` tree would have rewritten every import path in the tests and examples
  for no packaging benefit. `mks_servo_can/data/*.json` stays in both the wheel
  (via `package-data`) and the sdist (via a root `MANIFEST.in`), which is
  defect L7's guard at the artefact level.
- **`requires-python` is now `>=3.9`.** Both `setup.py` files claimed 3.8, which
  nothing had ever tested; 3.9 is the floor the CI matrix and
  `tests/unit/test_regressions.py` actually enforce.
- **The wheel no longer installs a top-level `examples` package.** The
  simulator's `setup.py` used `find_packages()`, which picked up
  `mks_servo_simulator/examples/` because it has an `__init__.py` — so
  installing the simulator put a generically-named `examples` package into
  site-packages, where it would shadow or be shadowed by anyone else's. The
  package list is now explicit.
- **Optional extras that are genuinely optional.** `simulator` (click, rich,
  fastapi, uvicorn) is everything the supported surfaces need; `dashboard`
  (textual) buys only the legacy `--textual-dashboard` TUI; `monitoring`
  (psutil) only the advanced performance panels; `dev` pulls in all of them plus
  pytest and ruff.
- **A CI job that builds the distribution and runs the suite against the
  installed package**, not the source tree — plus checks that the manual
  transcription is in both artefacts, that the simulator starts from the install
  and serves a non-empty `/commands`, and that the metadata passes
  `twine check`. This is the only job that can catch a packaging defect, and it
  caught one the moment it was written (L10, below). Both L7 and L10 were
  invisible to the existing job, because an editable install of a checkout has
  the whole repository on hand.

### Fixed (packaging)

- **`pip install mks-servo-can[simulator]` produced a simulator that could not
  start (L10).** `cli.py` imported the legacy Textual dashboard at module scope,
  making `textual` a hard requirement of the entire simulator — including
  `--debug-api` and `--json-output`, which have nothing to do with a TUI — while
  it was declared in no install requirement anywhere. `mks-servo-simulator` died
  with `ModuleNotFoundError: No module named 'textual'` before parsing a single
  argument. The import now happens inside the `--textual-dashboard` branch and
  fails there with a message naming the extra to install.
  Nothing had caught it because every environment that has ever run this suite
  had `textual` installed for `tests/test_textual_dashboard.py` — a dependency
  present for an unrelated reason in every environment that tested it.
  `tests/unit/test_optional_dependencies.py` blocks each optional module at
  import time in a subprocess and asserts the simulator still starts; it
  includes a test that the block itself works, so the others cannot pass
  vacuously.
- **`tests/simulator_compliance/test_simulator_cli.py` tested whichever
  simulator was first on `PATH`.** It resolved the console script with
  `shutil.which`, which need not belong to the interpreter running the tests, so
  on a machine with an older install elsewhere it exercised that one and
  reported on code nobody was looking at. It now prefers the script beside
  `sys.executable`. Found by running the suite against a wheel in a fresh
  virtualenv, where the failure first looked like a packaging defect.
- **`tests/unit/test_digitizer_integration.py` put the source tree on
  `sys.path`.** That made it import `mks_servo_can` from the checkout no matter
  what was installed, so the packaging job it now runs under would have silently
  tested the source tree and passed either way. Removed.

### Changed

- **Feedback polling costs one round trip per axis instead of three.**
  `ServoStream._run_feedback` no longer re-enables and re-disables motor
  responses around every encoder read: CanRSP suppresses only the run commands
  of manual sections 6.4–6.8, so 0x31 is answered either way. That reading comes
  from the manual and not yet from hardware, so persistent feedback failures
  while responses are disabled now warn and name the possibility rather than
  vanishing into debug records.
- `can.interface.Bus` is opened with `interface=` rather than the `bustype=`
  deprecated in python-can 4 and removed in 5.
- The remaining eight motion commands log per frame at lazy `debug` instead of
  eagerly-formatted `info`; only the 0xF5 path had been converted in 0.3.0. The
  hot-path gate in `tests/unit/test_regressions.py` now covers all of them.

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
  Every link in `docs/README.md` now resolves, and the *(planned)* markers are
  gone. The 26 remaining dead links were closed by deciding each one rather than
  by writing 26 pages: nine were worth writing and are new documents, and the
  rest were replaced by pointers to something executable.
  **Written:** `development/setup.md`, `development/running_tests.md`,
  `development/coding_standards.md`, `development/contributing.md`,
  `user_guides/simulator/cli_options.md`, `user_guides/simulator/logs.md`,
  `user_guides/simulator/advanced_simulation.md`,
  `user_guides/library/robot_control.md`, `appendices/glossary.md`.
  **Repointed:** the eight planned API-reference pages now link to the modules
  themselves — the docstrings already carry the signatures *and* the reasoning
  behind them, and cannot drift; the five planned tutorials now link to the
  scripts in `examples/`, which are linted and run; `mks_parameters.md` would
  have summarised `mks_servo_can/data/manual_commands_v106.json`, which is what
  the code is actually checked against, so it links there instead; and
  `user_guides/library/movement.md` was a stale duplicate of the `movements.md`
  that exists.
- **The documentation baseline is empty.**
  `tests/fixtures/docs_known_issues.json` went 63 → 26 → 0. Every Python block
  in `docs/` and `README.md` names an API that exists and every internal link
  resolves, so `test_no_new_problems` failing now means the text was written in
  that change. Verified by mutation: a dead link and a call to a non-existent
  `Axis` method were added and both were caught.
- **Documented the simulator's boundary of proof.** The new simulator guides say
  plainly what a green run against it does *not* establish — it was written from
  the same reading of the manual as the library, so it agrees with a shared
  misreading as readily as with a correct implementation, and it models no
  physics (no heating, lost steps, stall, supply sag or bus EMI).
- **Added `docs/development/roadmap.md`** — the open defects, the order to
  address them in, and how to verify each. `docs/README.md` had reserved a link
  for this since the beginning; it was one of the 27 dead ones.

### Known issues

The library defects found in the same review (L1–L5 in `REVIEW_NOTES.md`
Part 0) are all fixed above. Two things remain open:

- **No hardware trace has been recorded.** The simulator is validated against
  the *manual*, and the library was written from the same reading, so a shared
  misreading is invisible to every test here. Three questions turn on it: does
  0xF5 emit an abort frame when re-targeted mid-move, is the sign convention
  really CCW-positive, and does CanRSP suppress replies to reads as well as to
  the run commands. The fixes above are written to be correct under either
  answer to the first and third. See `docs/development/roadmap.md` item 1.

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
