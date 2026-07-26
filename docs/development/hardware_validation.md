# Hardware validation run

A record of validating the library against real hardware, so the next person does
not have to rediscover the setup, the firmware's gaps, or the measurement traps.

Rig: MKS SERVO42D/57D_CAN at CAN ID 3 (the pen axis from
`examples/calligraphy_plotter.py`), bare shaft, driven from a gs_usb adapter
(`1d50:606f`, candleLight/CANable) over SocketCAN at 500 kbps.

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

Because §5.9 is documented in the V1.0.5/V1.0.6 manual and this motor does not
answer it, **this board predates V1.0.5**. There is no CAN command to read the
firmware version — it is shown on the OLED during boot. That is the only way to
pin it down exactly.

Commands that do work: `0x30`, `0x31`, `0x32`, `0x33`, `0x34`, `0x39`, `0x3A`,
`0x3B`, `0x3E`, `0xF1`, and the motion commands.

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

## Test suite

`python -m pytest tests/ -q` → **807 passed, 26 skipped**, against the simulator,
with no hardware attached and no stray simulator processes.
