# Hardware validation run

A record of validating the library against real hardware, so the next person does
not have to rediscover the setup, the firmware's gaps, or the measurement traps.

Rig: two MKS SERVO42D/57D_CAN boards on NEMA17 32 mm motors, both bare shaft,
driven from a gs_usb adapter (`1d50:606f`, candleLight/CANable) over SocketCAN at
500 kbps. CAN ID 3 is the pen axis and CAN ID 2 the Y axis from
`examples/calligraphy_plotter.py`.

## Bringing the bus up

The adapter enumerates as `can0` but comes up `DOWN`/`STOPPED`; it needs a bitrate
before it will carry traffic:

```bash
ip link set can0 up type can bitrate 500000
ip -details -statistics link show can0     # expect state ERROR-ACTIVE
```

`ERROR-ACTIVE` is the healthy state. Watch the counters rather than guessing at
wiring: a full session moving several revolutions produced `bus-errors 0`,
`error-warn 0`, `error-pass 0`, `bus-off 0`. Any nonzero value there points at
termination or bitrate before it points at the library.

## What this firmware does not implement

Two documented commands get no reply at all from this motor — verified at raw
frame level with `python-can`, not just through the library:

| Command | Manual | Result |
| --- | --- | --- |
| `0x00` read system parameter | §5.9 | no response, all 256 parameter codes |
| `0x35` read raw encoder addition | — | no response |

The library's implementation of `0x00` is correct: `_send_command_and_get_response`
already keys the reply on `data[0]` (the parameter code) rather than `0x00`, which
is what §5.9 specifies. The silence is the firmware's.

The manual's revision table adds both `0x00` and `0x35` in **V1.0.6**, so a board
that answers neither **predates V1.0.6**. It answers `0x34`, which V1.0.3 added,
so the floor is V1.0.3. There is no CAN command to read the firmware version — it
is shown on the OLED during boot, and that remains the only way to pin it down
exactly.

Commands that do work: `0x30`, `0x31`, `0x32`, `0x33`, `0x34`, `0x39`, `0x3A`,
`0x3B`, `0x3E`, `0xF1`, and the motion commands.

An earlier revision of this document said these boards predate V1.0.5 and
therefore carry the F4/F5 bug. Both claims were wrong, from reading §5.9's
presence in the V1.0.5 manual as the release that introduced it. The revision
table is the authority, and it puts §5.9 in V1.0.6. A board between V1.0.5 and
V1.0.6 has the F4/F5 fix and still fails these probes, so the bug can be neither
confirmed nor ruled out here.

## Never send a bare setter opcode

`0x84` (set subdivision) takes one data byte. Sending the opcode with **no** data
byte still returns a success status (`[84 01 88]`) and corrupts the stored
subdivision. The symptom is not an error — it is relative moves (`0xF4`) silently
becoming ~16x slower, completing 435 of 819 steps and then tripping the library's
5 s completion timeout. Absolute moves keep working, because they are expressed in
encoder counts.

Recovery is a normal `set_subdivision(can_id, 16)`; moves immediately returned to
0.13 s and 821 steps. Probing "does this opcode read?" by sending it bare is not
safe on this protocol — several setters accept a short frame and write garbage.

## Measured performance

Absolute positioning, ±5 revolutions, against a 16384 count/rev encoder:

| Target | Error |
| --- | --- |
| typical | 1–7 counts (0.02–0.15°) |
| worst observed | 26 counts (0.57°) |

Return-to-zero repeatability is excellent and shows clean directional hysteresis:
approaching 0° from the positive side always parks at −31 counts, from the
negative side always at −20 counts, over 5 cycles with a spread of ±1. That 11
count (0.24°) gap is backlash/deadband, and being this repeatable it is
correctable in software if a use case needs it.

## Speed: the trap worth knowing

`speed_param` equals RPM. `RotaryKinematics.user_speed_to_motor_speed` converts
deg/s to RPM and clamps to 3000; 360 deg/s → 60, 600 → 100, 1200 → 200.

**Do not infer top speed from a short move.** A 720° (2 rev) move plateaus near
415–428 deg/s regardless of whether the parameter says 100 or 200, which reads
like a ~70 RPM ceiling. It is not — the move is over before the motor finishes
accelerating. Repeating with a 20-revolution move and sampling the motor's own
RPM readback:

| cmd deg/s | param | peak RPM | sustained RPM |
| --- | --- | --- | --- |
| 360 | 60 | 60.0 | 60.0 |
| 600 | 100 | 100.0 | 99.8 |
| 1200 | 200 | 200.0 | 200.0 |
| 2400 | 400 | 338.0 | 304.0 |

The parameter is honoured exactly up to 200. The real ceiling is ~338 RPM, which
matches the manual's §6.1 maximum of 400 RPM for the open-loop control modes
(`CR_OPEN`/`SR_OPEN`) — so this motor is configured in an open-loop serial mode,
not `SR_vFOC` (which would allow 3000 RPM).

Two further notes on §6.1: the speed value is calibrated for 16/32/64
subdivisions, so any change to subdivision rescales it. And the RPM readback
(`0x32`) lags — the first few samples after a move starts can still report the
*previous* move's speed, including its sign. Sample past the ramp before trusting
it.

## Two motors on the bus

A read-only scan of IDs 1–16 with `0x3A` finds both boards and nothing else.
`MultiAxisController` behaves on real hardware: `initialize_all_axes` 5 ms,
`get_all_positions_user` 3 ms, `get_all_statuses` 9 ms, each axis reporting in its
own units (mm for Y via `LinearKinematics` at 40 mm/rev, degrees for the pen).
Five rounds of interleaved concurrent pings and position reads showed no
cross-talk — worth checking explicitly, since a group operation filing results
against the wrong axis was a real defect here (`ec15859`). Bus counters stayed at
zero errors with two nodes.

Y axis figures, for comparison with the pen axis above: worst absolute error 48
counts (0.117 mm), return-to-zero repeatable to ±1 count with a 72 count
(0.178 mm) directional hysteresis.

## Two timeout defects the hardware exposed

Both produced the same symptom — a move the motor completed normally reported as
a `CommunicationError` — and neither was reachable from the simulator, because
the simulator reaches its commanded speed instantly and its position cache is
refreshed by the same reads.

**The estimator misread the manual.** `_calculate_move_timeout` derived speed as
`speed_param / 3000 * max_rpm_for_work_mode`. Section 6.1 says the parameter *is*
RPM and the motor clamps it to the mode ceiling, so the estimate is
`min(param, ceiling)`. The old form happened to be right in vFOC and wrong
everywhere else. It also ignored acceleration entirely, even though section 6.1
gives the ramp explicitly — 1 RPM per `(256 - acc) * ACCEL_TICK_SECONDS` — which
for a small `acc` dominates a short move. Commanding Y at 400 mm/s got a 5.6 s
budget for a move that really takes 8.3 s.

**The distance was sized from a stale cache.** Nothing refreshes the position
cache when a move completes; only an explicit read does. So after travelling to
800 mm the cache still read −9 steps, and the return move to 0 was sized as a
9 step journey and given 5.93 s for 8 s of travel. The fix tracks the last
commanded target, which is free and stays accurate because absolute moves land
where they were told to. Reading the encoder instead would have worked but would
have reintroduced the per-move round trip that C3 deliberately removed.

Both are pinned by regression tests, each verified by mutation — reverting the
formula, the ramp, or the reference position individually makes the matching
test fail.

## Work mode drives current, and therefore heat

Manual §"Work Current": `OPEN` (400 RPM cap) and `CLOSE` (1500 RPM cap) draw a
**fixed** current equal to `Ma` at all times, including while idle and merely
enabled. Only `vFOC` (3000 RPM cap) self-adapts, with `Ma` as a ceiling. The
factory default is `CR_vFOC`.

Since the work mode cannot be read back on this firmware, infer it from the
measured speed plateau: the pen axis topped out near 338 RPM (an `OPEN` mode) and
Y reached 499 RPM (above the 400 cap, so a `CLOSE` mode). Both are therefore in
fixed-current modes and dissipate full `Ma` continuously — which is why they get
warm sitting still.

The relevant defaults: `Ma` = 1600 mA on a SERVO42D, and `HoldMa` = 50%, so an
idle enabled motor sits at ~800 mA. A 32 mm-body NEMA17 is typically rated
1.0–1.33 A/phase, so the default is 1.2–1.6x over rated. Heating goes as I², so
1600 mA runs 2.56x hotter than 1000 mA, and `HoldMa` 50% is 25x the idle heat of
10%. `HoldMa` is ignored in vFOC mode.

Nothing in the library or the examples ever sets working current — whatever is in
EEPROM is what runs. `Ma`, `HoldMa` and the work mode have to be read off the OLED
menu, since `0x00` is unavailable here.

## Firmware detection

`Axis.initialize()` now identifies the board and records it on `axis.firmware`.
Because there is no version command, it probes which commands answer and turns
that into a *floor* on the release, using the revision table shipped in
`data/manual_commands_v106.json`.

Two rules make this safe and honest, and both were learned the hard way:

- **Probes only read.** The probe set lives in `firmware_probes` and is a curated
  subset of the revision table, because probing with a setter is what corrupted
  a subdivision during bring-up. A test asserts no probe has a state-changing
  category.
- **A missing fix is reported as unknown, never absent.** Everything V1.0.4 and
  V1.0.5 added is a setter or an action, so no read-only probe can detect them.
  `has_fix()` returns `True` or `None` — never `False`. The F4/F5 fix is exactly
  this case, which is why the claim retracted above cannot be restated in either
  direction.

What detection changes in behaviour: when `0x00` answers, the work mode is read
back and move timeouts stop guessing. When it does not, the work mode stays
`None` and the timeout estimator assumes the **slowest** ceiling (400 RPM), which
errs long. The previous default assumed vFOC's 3000 RPM — the least safe choice,
and load-bearing once timeouts were derived from it.

On the lab boards detection costs 0.51 s per axis (two unanswered probes at
0.25 s each) and reports `v1.0.3+, work mode unreadable`. Pass
`initialize(detect_firmware=False)` to skip it; the work mode then stays unknown,
which is conservative rather than wrong.

## Diagnosing an axis that answers but will not move

A motor whose logic board is perfectly healthy can be mechanically dead, and it
looks nothing like a fault over CAN. On the assembled gimbal the pan axis
answered every command — `enable`, `disable`, `status`, every parameter write —
and acknowledged each move with `F5 01` and often `F5 02 complete`, while the
shaft went nowhere. The cause turned out to be inside the motor: **the bearing
preload spring had been displaced**, binding the bearing.

The symptoms, and why each one misleads:

| Symptom | Reads as | Actually |
| --- | --- | --- |
| Moves stop short, then lurch | driver or tuning fault | stick-slip against friction |
| Hot while merely holding | holding current too high | current spent fighting friction |
| More current makes it *worse* | wrong current setting | harder break-free, bigger lurch |
| Feels **smooth** turned by hand | motor demagnetised | friction swamping the detent ripple |
| Direction-dependent resistance | limit or config asymmetry | mechanical, and it is the giveaway |

What actually narrowed it, in order:

1. **`0x39` following error.** The decisive reading. A commanded 10° move
   advanced the internal commanded angle by the full 10° while the encoder moved
   2.22°, then 0.00°, and the error was never closed with the motor stationary
   and protection reporting `ok`. That is a stall stated in numbers, and it also
   explains the "flips to a new position when forced" report: the commanded
   angle runs away, and freeing the shaft lets the rotor snap to wherever it got
   to.
2. **Make both axes identical.** Nothing can be read back on this firmware
   (`0x00` is V1.0.6 and unanswered), so the only comparison available is to
   write the same configuration to both and see whether they still differ. They
   did — which ruled out configuration entirely, including work mode, both
   currents, subdivision and direction.
3. **Watch the encoder while turning the axis by hand.** Position tracked the
   hand exactly, which ruled out a slipping set screw on the D-flat — the
   otherwise obvious suspect for "motor turns, load does not".
4. **Decouple and turn the bare shaft, unpowered.** An unpowered stepper should
   turn easily with light notchy detent. This one was very hard, which puts the
   fault inside the motor.

One trap worth stating, because it nearly sent the diagnosis the wrong way:
**disconnecting the supply does not disconnect the windings.** A stepper is a
generator when turned, and its phases can still form a closed loop through the
output stage's body diodes with the board unpowered, which brakes the shaft
smoothly and heavily. So "hard to turn with the power off" does not by itself
mean a mechanical fault — unplug the motor's phase connector from the driver to
tell the two apart.

## Test suite

`python -m pytest tests/ -q` → **826 passed, 26 skipped**, against the simulator,
with no hardware attached and no stray simulator processes.
