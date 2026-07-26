# Glossary of Terms

Terms as this project uses them. Where the MKS manual and common usage differ,
the manual's meaning is given and the difference noted — several of the sharper
bugs in this codebase came from assuming a word meant what it usually means.

## CAN and the bus

**CAN (Controller Area Network)** — the two-wire differential bus the motors
speak. Multi-drop: every motor shares one pair of wires and is distinguished by
its CAN ID.

**CAN ID** — the address of a motor on the bus, 1–2047 (`0x7FF`). Set on the
motor itself; the library addresses a motor by this and nothing else. The
default is `1`. Two motors with the same ID on one bus is a fault that presents
as garbled or duplicated replies.

**Frame** — one CAN message: an ID, a length, and up to 8 data bytes. Every MKS
command and reply fits in a single frame; the last data byte is always a
checksum.

**Bitrate** — bus speed, 500000 by default here. Every node must agree; a
mismatch looks like total silence, not like errors.

**Termination** — 120 Ω resistors at both physical ends of the bus. Missing
termination often works on a short bench cable and then fails intermittently at
length, which makes it an expensive thing to get wrong.

**slcan / SocketCAN / CANable** — ways of getting CAN into a computer. SocketCAN
is the Linux kernel interface (`can0`); slcan is a serial protocol used by USB
adapters such as the CANable, which appear as `/dev/ttyACM0`. Both are selected
through `CANInterface`. See
[the CAN on Linux survival guide](../tutorials/survival_guide_can_on_linux.md).

**Latency** — round-trip time for a command and its reply. The simulator's
`--latency-ms` is the round trip, applied as half before and half after.

## Motor and protocol

**MKS SERVO42D / SERVO57D** — the closed-loop stepper drivers this library
targets. 42 and 57 are NEMA frame sizes. `constants.py` also names the SERVO28D
and SERVO35D, which the manual mentions for current settings.

**Command code** — the byte identifying a command, e.g. `0x31` read encoder,
`0xF5` absolute-position move, `0xF6` speed mode. `constants.py` names all of
them; a bare hex literal in a call site is an anti-pattern here.

**Run command** — the manual's sections 6.4–6.8: `0xF4`, `0xF5`, `0xF6`, `0xFD`,
`0xFE`. The commands that make the shaft turn, as opposed to the reads.

**CanRSP (`0x8C`, "slave respond active")** — controls whether the motor replies
to commands. Important detail: it suppresses replies only to the **run
commands** above. A read such as `0x31` is answered either way. So "is this
motor still responding?" cannot be asked with an encoder read — it has to be
asked with a run command. (This is the simulator's behaviour, derived from the
manual, and is one of the questions a hardware trace is needed to settle.)

**Work mode** — how the motor accepts commands and how it closes the loop. Six
of them, in `constants.py`:

| mode | meaning |
|---|---|
| `MODE_CR_OPEN` / `MODE_SR_OPEN` | open loop, pulse / serial |
| `MODE_CR_CLOSE` / `MODE_SR_CLOSE` | closed loop, pulse / serial |
| `MODE_CR_VFOC` / `MODE_SR_VFOC` | field-oriented control, pulse / serial |

**CR / SR** — pulse (step/direction input) versus serial (commands over CAN).
CAN control needs an SR mode.

**FOC (field-oriented control)** — the smoothest and quietest of the three loop
types, and the one with the highest speed ceiling.

**Speed parameter** — the 0–3000 value carried in a run command. It is *not*
RPM. It maps onto a ceiling that depends on the work mode — 400 RPM open loop,
1500 closed loop, 3000 FOC — and it only equals RPM at microstep settings of 16,
32 or 64. Treating it as RPM is a mistake this codebase has made and fixed.

**Acceleration parameter** — 0–255, and **larger is slower**: each step changes
the speed by 1 RPM every `(256 - acc) × 50 µs`. `0` means no ramp at all, jump
straight to speed. The consequence that catches people is that short moves are
acceleration-limited, so a large speed error shows up as a small wall-clock
difference.

**Microstepping (subdivision, `mstep`)** — how finely one full motor step is
divided, 16 by default here. Affects resolution and the speed-parameter
calibration above.

**Encoder** — the built-in position sensor. 16384 counts per revolution
(`0x4000`). `ENCODER_PULSES_PER_REVOLUTION` in `constants.py`.

**Steps per revolution** — ambiguous unless qualified, and worth being careful
about: the *base motor* is 200 full steps/rev, the *driver* multiplies that by
the microstep setting, and the *encoder* reports 16384 counts/rev regardless.
Kinematics classes take the encoder figure.

**Homing** — driving to a known reference to establish absolute position.

**Zeroing** — declaring the current position to be zero, without moving.

## Library

**`CANInterface`** — owns the connection, hardware or simulator, and routes
frames to whichever axis is waiting for them. One per program.

**`Axis`** — one motor, with units. Wraps the protocol in user-facing terms:
`move_to_position_abs_user(50.0)` rather than a step count.

**`MultiAxisController`** — a named collection of axes, moved together. Group
operations gather per-axis failures into `MultiAxisError.individual_errors`
rather than failing on the first one.

**`LowLevelAPI`** — the raw command layer under `Axis`. Use it when you need a
command the higher layers do not expose.

**Kinematics** — the conversion between motor steps and the units you care
about. `RotaryKinematics` gives degrees, `LinearKinematics` millimetres from a
lead screw pitch, `EccentricKinematics` a non-linear arrangement. See
[Using Kinematics](../user_guides/library/kinematics.md).

**User units** — whatever the axis's kinematics produce: degrees, millimetres,
or your own. Methods with `_user` in the name take and return these; the ones
without take steps.

**Robot model** — a layer above `MultiAxisController` that converts a Cartesian
pose to joint targets. `CartesianRobot`, `TwoLinkArmPlanar`, `RRRArm`. See
[Controlling Robot Models](../user_guides/library/robot_control.md).

**Forward kinematics (FK)** — joint positions → end-effector pose.

**Inverse kinematics (IK)** — end-effector pose → joint positions. May have no
solution (out of reach) or several; `TwoLinkArmPlanar` always chooses elbow-up.

**DOF (degrees of freedom)** — independently controllable joints.

**Digitizer** — the subsystem that records a motion by hand and plays it back.
`examples/motor_digitizer.py`, and `README_Motor_Digitizer.md` beside it.

**`ServoStream` / real-time tracking** — continuous position streaming for
following a moving target rather than commanding discrete moves. See
[Asynchronous Control with asyncio](../advanced_topics/asynchronous_control_with_asyncio.md).

## Testing and the simulator

**Simulator** — `mks-servo-simulator`, which speaks the same protocol over TCP
so the whole library runs with nothing plugged in. It was written from the same
reading of the manual as the library, which bounds what its agreement proves.

**Virtual CAN bus** — the simulator's internal bus, carrying frames between the
socket server and the simulated motors, with configurable latency.

**HIL (hardware-in-the-loop)** — tests against a physical motor, in
`tests/hil/`. Skipped unless `MKS_HIL_CHANNEL` is set; the ones that turn the
shaft need `MKS_HIL_ALLOW_MOTION=1` as well.

**Hardware trace** — a recorded capture of what a real motor did, replayable in
CI forever afterwards. Recorded with `pytest tests/hil --hil-record=...`.

**Ratchet** — a test gate whose baseline of known problems may only shrink:
a new finding fails, and so does a baseline entry that no longer occurs.
`tests/test_docs_api.py` is one.

**Errata block** — the record, in
`mks_servo_can/data/manual_commands_v106.json`, of places where the MKS manual
is ambiguous or contradicts itself and this project has had to choose a reading.
Worth consulting before assuming a behaviour is a bug.
