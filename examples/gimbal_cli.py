#!/usr/bin/env python3
"""
Move the printed gimbal by hand from a command line.

`camera_gimbal_tracker.py` runs a tracking pass; this is for the other thing you
want on a bench, which is to poke one axis a few degrees and see what happens.
Every command is a single action that finishes and exits.

    python examples/gimbal_cli.py status              # reads only, moves nothing
    python examples/gimbal_cli.py set-zero            # define home, by hand
    python examples/gimbal_cli.py --zeroed point --yaw 30 --pitch -20
    python examples/gimbal_cli.py --zeroed jog pan 10          # relative
    python examples/gimbal_cli.py --zeroed goto tilt -20       # one axis
    python examples/gimbal_cli.py --zeroed home                # both to 0
    python examples/gimbal_cli.py --zeroed demo                # sweep the limits
    python examples/gimbal_cli.py release             # motors off, free to turn
    python examples/gimbal_cli.py hold                # motors on, holding

A whole session looks like: `set-zero`, then `point` as often as you like, then
`--zeroed home` and switch off at home so the next power-on is already zeroed.

Defaults are this machine: `can0` at 500 kbit/s, pan (yaw) on CAN 2 and tilt
(pitch) on CAN 3. Note that is *not* the tracker's default bitrate, which is
1 Mbit/s.

Safety, in the order it matters
===============================

**The limits come from `camera_gimbal_tracker.py`, imported rather than copied**,
so there is one place where pan's +/-90 lives. A target outside them is refused
before a frame is sent, rather than clamped: clamping is right for a streaming
loop that overwrites its own target, and wrong here, where you would be told a
move to 120 succeeded while the axis sat at 90.

**A limit is not an angle until zero is set.** These motors zero their encoder
wherever the shaft is standing at power-on, so `+/-90` means "90 either side of
wherever it was when you switched on" until `--set-zero` has been run at the
mechanical centre. This tool cannot detect that, so it refuses to move without
`--zeroed`, exactly as the tracker does. `status` and `release` do not need it -
neither commands motion.

**Ask for completion messages back before waiting for one.** `ServoStream` mutes
the motors (0x8C) and restores them with active initiation off, deliberately, so
any ordinary waited move after a tracking run waits for a frame that never
arrives. Every command here re-enables it first.

**Every move reports where the shaft actually ended up**, read back off the
encoder, and prints `OFF` and exits non-zero when that is more than 0.5 deg from
what was asked. This is not decoration: the first move made after a motor has
been released reliably falls short here - measured at 1.73 deg on pan - because
enabling snaps the rotor to the nearest detent and the move starts from a frame
that disagrees with the encoder. Issuing the same command again lands it.

It is deliberately **not** retried automatically. A move that stops short
because it has run into the cable loom looks identical from here, and retrying
that is pushing harder against the thing the limits exist to protect. Look at
the number, then decide.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from camera_gimbal_tracker import PAN_LIMITS, TILT_LIMITS

from mks_servo_can import Axis, CANInterface, RotaryKinematics
from mks_servo_can import constants as const
from mks_servo_can.exceptions import MKSServoError

# This machine. The tracker's --two-axis default agrees with these.
AXES = {
    "pan": {"can_id": 2, "limits": PAN_LIMITS, "what": "yaw"},
    "tilt": {"can_id": 3, "limits": TILT_LIMITS, "what": "pitch"},
}

DEFAULT_CHANNEL = "can0"
DEFAULT_BITRATE = 500000
DEFAULT_SPEED_DEG_S = 30.0

# Slowest speed that survives the conversion to an MKS speed parameter. The
# mapping rounds deg/s divided by six, so 3.0 still lands on 0 and 3.1 is the
# first value that reaches 1. Measured against the real conversion rather than
# derived, because "divided by six" is an observation about the current
# kinematics and not a promise.
MIN_SPEED_DEG_S = 3.1


def _axis(can_if: CANInterface, name: str) -> Axis:
    """Builds an `Axis` for one of the gimbal's motors."""
    return Axis(
        can_if,
        motor_can_id=AXES[name]["can_id"],
        name=name,
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        ),
    )


async def _unmute(axis: Axis) -> None:
    """Restores responses *and* active initiation on one motor.

    A motor left muted by a streaming run answers nothing, which is
    indistinguishable from a motor that is not on the bus - so this runs before
    anything else, including the read-only commands.
    """
    await axis._low_level_api.set_slave_respond_active(
        axis.can_id, respond_enabled=True, active_enabled=True
    )


def _targets(args) -> list:
    """Which axes a command acts on.

    Every command that can sensibly act on one motor takes `--axis`, defaulting
    to both. That matters more than tidiness when one axis is faulty: releasing
    a jammed yaw axis while leaving pitch holding, or zeroing only the axis you
    just moved, are both things you want during a diagnosis and neither is
    possible if the tool insists on operating in pairs.
    """
    chosen = getattr(args, "axis", None)
    return [chosen] if chosen else list(AXES)


async def _sync_enable_state(axis: Axis) -> None:
    """Makes the axis' cached enable flag match the motor before acting on it.

    `Axis.enable_motor()` and `disable_motor()` are deliberately idempotent
    against a *cached* flag - the unit tests assert they do not re-send when the
    cache already agrees. On a freshly constructed `Axis` that cache is False,
    so calling `disable_motor()` on a motor that is really holding takes the
    "already disabled" branch and sends nothing at all. Every command here
    builds its own `Axis`, so every one of them would hit that.

    This was not theoretical: `release` reported both axes STILL HOLDING while
    printing that there was no holding current, on a machine that was hot.
    `read_en_status()` queries 0x3A and writes the answer into the same cache,
    which makes the guard tell the truth.
    """
    await axis.read_en_status()


def check_speed(speed: float) -> int:
    """Refuses a speed the motor cannot express, and returns what it becomes.

    The speed sent in an 0xF5 frame is the MKS parameter, which is essentially
    shaft RPM as an integer. `user_speed_to_motor_speed` divides by six and
    truncates, so the resolution near the bottom is terrible and there is a dead
    zone below it:

        deg/s   2   4   5   8  10  15  20  30  60
        param   0   1   1   1   2   2   3   5  10

    Two consequences worth knowing. Anything from about 4 to 8 deg/s is the same
    command, so asking for 8 rather than 5 changes nothing. And below about
    3 deg/s the parameter is 0, which `Axis._move_absolute_handler` treats as
    "no move needed" - it returns success having sent no frame at all. Measured:
    a tilt move of +8 deg at 2 deg/s moved +0.00 deg and raised nothing, while
    the same move at 20 deg/s moved exactly +8.00.

    Args:
        speed: Requested speed in degrees per second.

    Returns:
        The MKS speed parameter the motor will actually receive.

    Raises:
        SystemExit: If the speed rounds to zero, rather than letting the move be
            silently discarded.
    """
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )
    param = kin.user_speed_to_motor_speed(speed)
    if param < 1:
        raise SystemExit(
            f"refusing: {speed:g} deg/s becomes MKS speed parameter 0, and a "
            f"move at speed 0 is\ndiscarded by the library without sending a "
            f"frame or raising - it would look\nlike a completed move that did "
            f"nothing. The slowest speed this motor can\nexpress is "
            f"{MIN_SPEED_DEG_S:g} deg/s (parameter 1)."
        )
    return param


def check_limits(name: str, target: float) -> None:
    """Refuses a target outside the axis' soft limits.

    Args:
        name: Axis name.
        target: Absolute target in degrees.

    Raises:
        SystemExit: If the target is outside the limits, with the numbers.
    """
    low, high = AXES[name]["limits"]
    if not (low <= target <= high):
        raise SystemExit(
            f"refusing: {name} to {target:+.2f} deg is outside its limits "
            f"({low:+.1f} .. {high:+.1f}).\n"
            f"These are the cable limits of the built machine, not a "
            f"preference. Nothing was sent."
        )


async def cmd_status(can_if: CANInterface, args) -> int:
    """Reports where each axis is and whether it is holding. Moves nothing."""
    print(f"{'axis':6}{'CAN':>5}{'position':>12}{'limits':>18}   state")
    print("-" * 60)
    for name in _targets(args):
        axis = _axis(can_if, name)
        low, high = AXES[name]["limits"]
        try:
            await _unmute(axis)
            pos = await axis.get_current_position_user()
            held = await axis.read_en_status()
            inside = "" if low <= pos <= high else "  OUTSIDE LIMITS"
            print(
                f"{name:6}{axis.can_id:>5}{pos:>+11.2f} deg"
                f"{f'{low:+.0f} .. {high:+.0f}':>18}   "
                f"{'holding' if held else 'released'}{inside}"
            )
        except MKSServoError as exc:
            print(f"{name:6}{axis.can_id:>5}   no answer: {type(exc).__name__}")
    print(
        "\nPositions are measured from wherever the motor last zeroed, which is "
        "power-on\nunless --set-zero has been run. They are only angles if it has."
    )
    return 0


async def _move(can_if: CANInterface, name: str, target: float,
                speed: float, relative_from: float | None) -> int:
    """Sends one absolute move and checks where the axis actually ended up.

    Returns non-zero when the shaft did not arrive. `point` and `demo` verified
    their moves from the start and these did not - they printed the landing
    position and exited 0 regardless, so `goto tilt -10` could report
    "now +34.67 (asked for -10.00)" and still look like a success.
    """
    param = check_speed(speed)
    axis = _axis(can_if, name)
    await _unmute(axis)
    await _sync_enable_state(axis)
    await axis.enable_motor()
    before = await axis.get_current_position_user()
    if relative_from is not None:
        target = before + relative_from
        check_limits(name, target)
    # The MKS parameter is printed beside the speed because the mapping is lossy
    # near the bottom: 4 through 8 deg/s all become 1.
    print(f"{name}: {before:+.2f} -> {target:+.2f} deg at {speed:.0f} deg/s "
          f"(MKS param {param})")
    await axis.move_to_position_abs_user(target, speed_user=speed)
    after = await axis.get_current_position_user()
    # Report the encoder, not the fact the command returned: where the shaft
    # ended up is the thing being tested.
    landed = abs(after - target) <= 1.0
    print(f"{name}: {'ok  ' if landed else 'SHORT'} now {after:+.2f} deg  "
          f"(asked for {target:+.2f})")
    return 0 if landed else 1


async def cmd_jog(can_if: CANInterface, args) -> int:
    """Moves one axis by a relative number of degrees."""
    return await _move(can_if, args.axis, 0.0, args.speed, relative_from=args.degrees)


async def cmd_goto(can_if: CANInterface, args) -> int:
    """Moves one axis to an absolute position."""
    check_limits(args.axis, args.degrees)
    return await _move(can_if, args.axis, args.degrees, args.speed, None)


async def cmd_home(can_if: CANInterface, args) -> int:
    """Drives the selected axes to zero."""
    for name in _targets(args):
        await _move(can_if, name, 0.0, args.speed, None)
    return 0


async def cmd_point(can_if: CANInterface, args) -> int:
    """Drives both axes to an absolute yaw/pitch pair, together.

    Yaw and pitch are named rather than positional on purpose. A transposed pair
    is not a slightly wrong pose, it is one axis being driven against the
    other's allowance. Both happen to be +/-90 today, and naming them means that
    stays a coincidence rather than something the tool relies on.

    Each axis can carry its own speed. `--yaw-speed` and `--pitch-speed` fall
    back to `--speed` when not given, so the simple case stays one number. They
    are worth having because the two axes are not interchangeable: yaw carries
    the whole upper assembly and drags the cable loom with it, while pitch moves
    a balanced cradle. Being able to creep one while the other moves normally is
    also how you probe an axis that is stalling without slowing the other down.
    """
    check_limits("pan", args.yaw)
    check_limits("tilt", args.pitch)

    yaw_speed = args.yaw_speed if args.yaw_speed is not None else args.speed
    pitch_speed = args.pitch_speed if args.pitch_speed is not None else args.speed
    yaw_param, pitch_param = check_speed(yaw_speed), check_speed(pitch_speed)

    # Both axes are checked before either moves. Sending yaw and then
    # discovering pitch is out of range would leave the machine somewhere
    # nobody asked for, half way through a pose.
    print(f"yaw (pan)   -> {args.yaw:+.2f} deg at {yaw_speed:.0f} deg/s (param {yaw_param})")
    print(f"pitch (tilt)-> {args.pitch:+.2f} deg at {pitch_speed:.0f} deg/s (param {pitch_param})")

    async def one(name: str, target: float, speed: float):
        axis = _axis(can_if, name)
        await _unmute(axis)
        await axis.enable_motor()
        await axis.move_to_position_abs_user(target, speed_user=speed)
        return name, await axis.get_current_position_user()

    # Together, not one after the other: a gimbal that yaws fully and then
    # pitches sweeps a different path through its own cabling than one that
    # does both at once, and the second is the pose you actually asked for.
    #
    # Different speeds mean they no longer arrive together, which is the point:
    # the pose is the destination, not the path. If you need them to arrive at
    # the same moment, leave the per-axis speeds alone.
    results = await asyncio.gather(
        one("pan", args.yaw, yaw_speed),
        one("tilt", args.pitch, pitch_speed),
        return_exceptions=True,
    )
    ok = True
    for item in results:
        if isinstance(item, BaseException):
            print(f"  FAILED: {type(item).__name__}: {item}")
            ok = False
            continue
        name, pos = item
        label = "yaw" if name == "pan" else "pitch"
        want = args.yaw if name == "pan" else args.pitch
        landed = abs(pos - want) <= 0.5
        print(f"  {'OK  ' if landed else 'OFF '} {label:5} now {pos:+.2f} deg "
              f"(asked {want:+.2f})")
        ok = ok and landed
    return 0 if ok else 1


async def cmd_demo(can_if: CANInterface, args) -> int:
    """Sweeps each axis limit to limit at a range of speeds, then both together.

    This is the most demanding thing the machine does: full travel is where the
    cable loom is tightest and where a stalling axis has the most room to build
    up following error. So it is built to stop rather than to finish.

    **It aborts on the first move that does not land.** A stepper that stalls
    does not report a failure - the commanded angle simply runs on without the
    shaft, and the further it runs the harder the eventual catch-up snap. So
    every leg is verified against the encoder and a miss ends the run, leaving
    the machine where it stopped and printing what it was doing. Finishing the
    script matters less than not driving a jammed axis to the end of its travel.

    `--fraction` scales the travel: 0.5 sweeps half of each limit. Use it the
    first time, on any machine whose full range has not been driven before.
    """
    speeds = [float(s) for s in args.speeds.split(",")]
    for s_ in speeds:
        check_speed(s_)   # fail before moving, not part way through
    frac = args.fraction
    names = _targets(args)

    print(f"Sweeping {', '.join(names)} at {frac*100:.0f}% of travel, "
          f"speeds {', '.join(f'{s:.0f}' for s in speeds)} deg/s.")
    print("Aborts on the first leg that does not land.\n")

    legs = []
    for name in names:
        low, high = AXES[name]["limits"]
        for speed in speeds:
            legs.append((name, low * frac, speed))
            legs.append((name, high * frac, speed))
        legs.append((name, 0.0, speeds[-1]))
    # Then both at once, each at a different speed, which is where the two axes
    # sharing one bus and one power supply actually gets tested.
    if len(names) > 1:
        for speed in speeds:
            legs.append(("both", frac, speed))

    print(f"{'axis':6}{'target':>10}{'speed':>9}{'landed':>10}   result")
    print("-" * 52)
    for name, target, speed in legs:
        if name == "both":
            hi_p = AXES["pan"]["limits"][1]
            lo_t = AXES["tilt"]["limits"][0]
            # Opposite corners, and deliberately at different speeds per axis.
            pairs = [("pan", hi_p * target, speed), ("tilt", lo_t * target, speed / 2)]
            res = await asyncio.gather(*[
                _demo_leg(can_if, n, t, s) for n, t, s in pairs
            ], return_exceptions=True)
            for (n, t, s), r in zip(pairs, res):
                landed, _ = _demo_report(n, t, s, r)
                if not landed:
                    return 1
            continue
        r = await _demo_leg(can_if, name, target, speed)
        landed, _ = _demo_report(name, target, speed, r)
        if not landed:
            return 1

    print("\nSwept both limits at every speed. Returning to home.")
    for name in names:
        await _demo_leg(can_if, name, 0.0, speeds[-1])
    return 0


async def _demo_leg(can_if: CANInterface, name: str, target: float,
                    speed: float):
    """Runs one leg of the demo, returning the landed position or the error."""
    try:
        check_limits(name, target)
        axis = _axis(can_if, name)
        await _unmute(axis)
        await _sync_enable_state(axis)
        await axis.enable_motor()
        await axis.move_to_position_abs_user(target, speed_user=speed)
        return await axis.get_current_position_user()
    except Exception as exc:
        return exc


def _demo_report(name: str, target: float, speed: float, result) -> tuple:
    """Prints one leg's outcome. Returns `(landed, position_or_None)`."""
    if isinstance(result, BaseException):
        print(f"{name:6}{target:>+9.1f}d{speed:>8.0f}{'-':>10}   "
              f"FAILED {type(result).__name__}: {str(result)[:60]}")
        return False, None
    landed = abs(result - target) <= 1.0
    print(f"{name:6}{target:>+9.1f}d{speed:>8.0f}{result:>+9.2f}d   "
          f"{'ok' if landed else 'SHORT - aborting'}")
    return landed, result


# The board's whole configurable state, minus anything that changes how it is
# addressed. set_can_id, set_can_bitrate and set_group_id are deliberately
# absent: a mistake in any of those takes the motor off the bus, and recovering
# it means the screen and the buttons, not this tool.
#
# Values are what this gimbal wants. They are applied in this order because
# work mode gates two of the others - holding current is ignored in vFOC, and
# the RPM ceiling depends on the mode - so the mode has to land first.
PARAM_PLAN = (
    ("work mode", "set_work_mode", const.MODE_SR_CLOSE,
     "serial closed loop: honours holding current, 1500 RPM ceiling"),
    ("working current", "set_working_current", 1200,
     "mA; torque for moves, and the base the holding percentage applies to"),
    ("holding current", "set_holding_current_percentage", 0x02,
     "code 0x02 = 30% of working current, to keep the printed parts cool"),
    ("subdivision", "set_subdivision", 32,
     "microsteps; only affects pulse commands, not the 0xF5 moves used here"),
    ("subdiv interpolation", "set_subdivision_interpolation", True,
     "smooths the microstep transitions"),
    ("stall protection", "set_stall_protection", True,
     "so a jammed axis reports it instead of silently accumulating error"),
    ("en pin level", "set_en_pin_active_level", const.EN_ACTIVE_LOW,
     "how the physical En input is read; irrelevant to CAN enable, but it "
     "should be known rather than whatever the board shipped with"),
    ("direction", "set_motor_direction", const.DIR_CW,
     "which way positive counts turn the shaft"),
    ("auto screen off", "set_auto_screen_off", False,
     "leave the display on; it is the only readout when CAN is not answering"),
    ("key lock", "set_key_lock", False,
     "leave the buttons usable for exactly the same reason"),
)


async def cmd_params(can_if: CANInterface, args) -> int:
    """Writes the board's whole configuration, except how it is addressed.

    THERE IS NO READ-BACK. Reading a system parameter is 0x00, added in firmware
    V1.0.6, and these boards do not answer it at all - so this cannot show a
    before-and-after, and the motor's acknowledgement attests that a command was
    accepted rather than that anything changed. That is also the reason to have
    this command: when two boards behave differently and neither will say how it
    is configured, the only way to compare them is to make them the same.

    CAN identity is deliberately not touched. `set_can_id`, `set_can_bitrate`
    and `set_group_id` would each take the motor off the bus if they went wrong,
    and getting it back means the screen and the buttons rather than this tool.

    THESE ARE ALMOST CERTAINLY PERSISTENT WRITES, AND THAT IS NOT VERIFIED HERE.
    The manual transcription says nothing about flash or EEPROM for any of these
    commands. What says they persist is indirect but strong: the same settings
    appear in the board's on-screen menu, and the firmware carries a separate
    `0x3F restore_default_parameters` - a command that would have nothing to
    restore if the settings were volatile. Against that, persistence cannot be
    demonstrated on these boards, because confirming it would mean reading a
    parameter back and 0x00 is unanswerable here.

    Treat it as writing non-volatile memory. Do not put this in a loop or a
    startup script: flash and EEPROM have finite write endurance, and a command
    that quietly consumes it every run is a bad thing to automate. Once per
    board, or when something is actually wrong, is the intended use.

    The way back from a bad write is `0x3F restore_default_parameters`
    (`LowLevelAPI.restore_default_parameters`), which is not exposed here on
    purpose: the motor reboots and has to be **recalibrated** afterwards, so it
    is a deliberate recovery step and not something to reach for casually.

    `--dry-run` prints the plan without sending anything, which is worth doing
    first: every one of these is a write to a board you cannot interrogate.
    """
    print(f"{'parameter':22}{'value':>10}   why")
    print("-" * 78)
    for label, _, value, why in PARAM_PLAN:
        shown = value if not isinstance(value, bool) else ("on" if value else "off")
        print(f"{label:22}{shown!s:>10}   {why}")

    if args.dry_run:
        print("\n--dry-run: nothing was sent.")
        return 0

    ok = True
    for name in _targets(args):
        axis = _axis(can_if, name)
        await _unmute(axis)
        api = axis._low_level_api
        print(f"\n{name} (CAN {axis.can_id}):")
        for label, method, value, _why in PARAM_PLAN:
            try:
                await getattr(api, method)(axis.can_id, value)
                print(f"  OK    {label}")
            except MKSServoError as exc:
                ok = False
                print(f"  FAIL  {label}: {type(exc).__name__}: {str(exc)[:60]}")
    print(
        "\nAccepted, not verified - 0x00 is unanswerable on this firmware, so "
        "nothing here\ncan read a parameter back to confirm it took.\n"
        "\nThese are almost certainly writes to non-volatile memory: the same "
        "settings are\nin the board's menu, and 0x3F restore_default_parameters "
        "exists, which would\nhave nothing to restore otherwise. So do not run "
        "this repeatedly - flash has a\nfinite number of writes. Zero is not "
        "among these; re-run `set-zero` after a\npower cycle."
    )
    return 0 if ok else 1


async def cmd_set_zero(can_if: CANInterface, args) -> int:
    """Defines the current position of both axes as zero.

    This is the thing every limit is measured from, so it is worth being
    deliberate about. By default the motors are released and you are asked to
    centre the axes by hand first; `--here` skips that and zeroes wherever the
    machine is standing, which is right when you have just positioned it and
    wrong the rest of the time.
    """
    axes = {name: _axis(can_if, name) for name in _targets(args)}
    for axis in axes.values():
        await _unmute(axis)

    if not args.here:
        for name, axis in axes.items():
            await axis.disable_motor()
            print(f"  {name:5} released")
        print(
            "\nCentre both axes by hand now:\n"
            "  pan  (yaw)   - camera facing straight forward, loom slack even\n"
            "  tilt (pitch) - camera level, balanced on its axis\n"
            "\nTake the loom to one side and back: the centre you want is the "
            "middle of the\ntravel, not the middle of what is convenient."
        )
        input("\nPress Enter when both axes are centred: ")
        print()

    ok = True
    for name, axis in axes.items():
        before = await axis.get_current_position_user()
        await axis.set_current_position_as_zero()
        # Read it back. The alternative is running a whole session on limits
        # anchored to a zero the motor never accepted.
        after = await axis.get_current_position_user()
        good = abs(after) < 0.1
        ok = ok and good
        print(f"  {'OK  ' if good else 'FAIL'} {name:5} was {before:+8.2f}, "
              f"now reads {after:+8.2f} deg")

    if ok:
        print("\nZero set. The limits now mean these angles about the centre:")
        for name in AXES:
            low, high = AXES[name]["limits"]
            label = "yaw" if name == "pan" else "pitch"
            print(f"  {label:5} {low:+7.1f} .. {high:+7.1f} deg")
        print(
            "\nThis holds until the motors lose power. Park at 0 before "
            "switching off\nand the next power-on comes up already zeroed."
        )
    return 0 if ok else 1


async def cmd_current(can_if: CANInterface, args) -> int:
    """Sets working and/or holding current on one or both motors.

    Holding current is what cooks the printed parts. A stepper asked to hold a
    position draws current continuously, the motor bolts straight to a PLA part,
    and a NEMA17 at 1.5 A settles at 60-70 C, which is PLA's glass transition.

    Two levers, and they are not equivalent:

    * ``--holding`` (0x9B) sets holding current as a *percentage of the working
      current*, 10% to 90%. The manual says plainly that it is "effective in
      OPEN and CLOSE modes only; vFOC ignores it". So on a board running vFOC
      this command succeeds and changes nothing.
    * ``--working`` (0x83) sets the working current in mA outright. It is not
      mode-dependent, so it bites regardless - at the cost of torque for
      everything, moves included.

    NEITHER CAN BE READ BACK ON THIS FIRMWARE. Reading a system parameter is
    0x00, which the manual adds in V1.0.6 and which these boards do not answer
    at all (see docs/development/hardware_validation.md). So the motor's
    acknowledgement is the only evidence there is: it says the command was
    accepted, not that it had an effect. If you need to know the heat actually
    went away, the honest instrument is a finger on the motor a few minutes
    later, or `release`.
    """
    targets = _targets(args)
    if args.holding is None and args.working is None:
        raise SystemExit("nothing to do: pass --holding and/or --working")

    for name in targets:
        axis = _axis(can_if, name)
        await _unmute(axis)
        api = axis._low_level_api
        if args.working is not None:
            await api.set_working_current(axis.can_id, args.working)
            print(f"  {name:5} working current -> {args.working} mA (accepted)")
        if args.holding is not None:
            # 0x00 is 10%, rising in 10% steps to 0x08 for 90%.
            code = args.holding // 10 - 1
            await api.set_holding_current_percentage(axis.can_id, code)
            print(f"  {name:5} holding current -> {args.holding}% of working "
                  f"(code 0x{code:02x}, accepted)")

    if args.holding is not None:
        print(
            "\n'accepted' is not 'applied': 0x9B is ignored outright in vFOC "
            "mode, and the\nwork-mode read needs firmware >= V1.0.6, which "
            "these boards do not have. If the\nmotors stay hot, use --working, "
            "which no mode ignores."
        )
    return 0


# 0x39 reports commanded-minus-actual shaft angle, where 0..51200 spans a turn.
SHAFT_ERROR_PER_DEG = 51200.0 / 360.0

# Only the serial modes accept the CAN command set at all; the manual is explicit
# that Part 6 requires SR_OPEN, SR_CLOSE or SR_VFOC.
MODES = {
    "open": const.MODE_SR_OPEN,
    "close": const.MODE_SR_CLOSE,
    "foc": const.MODE_SR_VFOC,
}


async def cmd_mode(can_if: CANInterface, args) -> int:
    """Sets the work mode on one or both motors.

    The choice is not cosmetic and it interacts with everything else here:

    * **close** (SR_CLOSE) closes the loop on the encoder and caps at 1500 RPM.
      Holding current (0x9B) is honoured.
    * **foc** (SR_VFOC) caps at 3000 RPM and **ignores 0x9B entirely**, so a
      board in this mode holds full current no matter what you set. That is why
      turning holding current down to 10% on a hot motor changed nothing.
    * **open** (SR_OPEN) does not correct anything, caps at 400 RPM, and a
      forced shaft loses its position permanently.

    THE MOTORS ARE RELEASED FIRST, ON PURPOSE. A motor carrying accumulated
    following error will drive to close it the moment the loop starts caring -
    pan was measured 13.23 degrees adrift - and that is a sudden unsupervised
    move on an axis limited by its cabling. Releasing first means the mode
    change lands on a motor that is not holding anything, and you re-zero after.
    """
    targets = _targets(args)
    code = MODES[args.mode]

    for name in targets:
        axis = _axis(can_if, name)
        await _unmute(axis)
        await _sync_enable_state(axis)
        err_before = await axis._low_level_api.read_shaft_angle_error(axis.can_id)
        await axis.disable_motor()
        await axis._low_level_api.set_work_mode(axis.can_id, code)
        print(f"  {name:5} -> {args.mode} (0x{code:02x}), released; "
              f"following error was {err_before / SHAFT_ERROR_PER_DEG:+.2f} deg")

    print(
        "\nBoth motors are released. Re-zero before moving again:\n"
        "    gimbal_cli.py set-zero\n"
        "The mode is stored on the board, so it survives a power cycle; the "
        "zero does not."
    )
    return 0


async def cmd_watch(can_if: CANInterface, args) -> int:
    """Streams position, following error and protection state while you push it.

    For diagnosing what a motor does when it is forced off its holding position.
    Four readings, chosen because between them they separate the causes that
    look identical from outside:

    * **position** (0x31) - where the encoder says the shaft is.
    * **error** (0x39) - commanded angle minus real angle. If the motor is
      holding a target and you push it away, this grows. If it *stops* growing
      while the shaft is somewhere else, the motor has stopped trying, which
      means it has either lost the pole or given up.
    * **protect** (0x3E) - whether locked-rotor protection has tripped. Once it
      trips, the motor stops driving until it is released with 0x3D, and it will
      not resist being turned at all.
    * **pulses** (0x33) - the commanded pulse count. This is the discriminator
      that matters: if the shaft moves 90 degrees and this does *not* change,
      nothing commanded that motion and the rotor slipped. If it does change,
      something is still driving.

    Run it, then force the axis by hand and watch which column moves first.
    """
    axis = _axis(can_if, args.axis)
    await _unmute(axis)
    api = axis._low_level_api
    print(f"Watching {args.axis} (CAN {axis.can_id}) at {args.hz:.0f} Hz. "
          f"Force it by hand. Ctrl-C to stop.\n")
    print(f"{'t':>6}{'position':>12}{'error':>12}{'speed':>8}"
          f"{'pulses':>12}  protect")
    print("-" * 66)

    period = 1.0 / args.hz
    started = asyncio.get_event_loop().time()
    last_pos = None
    try:
        while True:
            t = asyncio.get_event_loop().time() - started
            try:
                pos = await axis.get_current_position_user()
                err = await api.read_shaft_angle_error(axis.can_id)
                rpm = await api.read_motor_speed_rpm(axis.can_id)
                pulses = await api.read_pulses_received(axis.can_id)
                prot = await api.read_motor_protection_state(axis.can_id)
            except MKSServoError as exc:
                print(f"{t:6.1f}   read failed: {type(exc).__name__}: {exc}")
                await asyncio.sleep(period)
                continue
            # Flag the thing being investigated: a large sudden position change
            # with nothing commanding it.
            jump = ""
            if last_pos is not None and abs(pos - last_pos) > 5.0:
                jump = f"   <-- JUMPED {pos - last_pos:+.1f} deg"
            last_pos = pos
            print(
                f"{t:6.1f}{pos:>+11.2f}d{err/SHAFT_ERROR_PER_DEG:>+11.2f}d"
                f"{rpm:>8}{pulses:>12}  "
                f"{'TRIPPED' if prot else 'ok'}{jump}"
            )
            await asyncio.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


async def cmd_release(can_if: CANInterface, args) -> int:
    """Disables both motors so the axes can be turned by hand."""
    ok = True
    for name in _targets(args):
        axis = _axis(can_if, name)
        await _unmute(axis)
        await _sync_enable_state(axis)
        await axis.disable_motor()
        held = await axis.read_en_status()
        ok = ok and not held
        print(f"  {name:5} {'STILL HOLDING' if held else 'released'}")
    # Only claim the current is off if the motors agreed it is. Saying so
    # unconditionally is how a tool ends up reporting "no holding current"
    # over two lines that just said STILL HOLDING.
    print("\n" + (
        "No holding current. Whatever position the axes settle in is what the "
        "next\npower-on will call zero."
        if ok else
        "AT LEAST ONE MOTOR IS STILL ENERGISED. It is still heating, and these\n"
        "drivers pull full current regardless of load. Cut the supply."
    ))
    return 0 if ok else 1


async def cmd_hold(can_if: CANInterface, args) -> int:
    """Enables both motors so they hold position."""
    ok = True
    for name in _targets(args):
        axis = _axis(can_if, name)
        await _unmute(axis)
        await _sync_enable_state(axis)
        await axis.enable_motor()
        held = await axis.read_en_status()
        ok = ok and held
        print(f"  {name:5} {'holding' if held else 'NOT HOLDING'}")
    print("\nThese drivers pull full current regardless of load, and the motor "
          "reaches\nPLA's glass transition doing it. Do not leave it like this.")
    return 0 if ok else 1


COMMANDS = {
    "status": (cmd_status, False),
    "jog": (cmd_jog, True),
    "goto": (cmd_goto, True),
    "point": (cmd_point, True),
    "home": (cmd_home, True),
    # set-zero is what makes --zeroed true, so it cannot require it. It releases
    # the motors rather than driving them, so there is nothing to guard.
    "demo": (cmd_demo, True),
    "params": (cmd_params, False),
    "set-zero": (cmd_set_zero, False),
    "current": (cmd_current, False),
    "watch": (cmd_watch, False),
    "mode": (cmd_mode, False),
    "release": (cmd_release, False),
    "hold": (cmd_hold, False),
}

# Conservative ceiling. SERVO42D/28D/35D are rated 3000 mA and the 57D 5200, and
# nothing here can read back which board it is talking to, so this refuses to be
# the thing that overcurrents a motor on a guess.
MAX_WORKING_MA = 3000


def build_parser() -> argparse.ArgumentParser:
    """Builds the argument parser."""
    p = argparse.ArgumentParser(
        description="Move the printed gimbal from the command line.",
        epilog="Run 'set-zero' before any move, and again after every power "
               "cycle. Then pass --zeroed.",
    )
    p.add_argument("--channel", default=DEFAULT_CHANNEL)
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--speed", type=float, default=DEFAULT_SPEED_DEG_S,
                   help="degrees per second for moves (default 30)")
    p.add_argument("--zeroed", action="store_true",
                   help="confirm --set-zero has been run since the motors last "
                        "lost power; required by every command that moves")
    p.add_argument("--verbose", action="store_true")

    sub = p.add_subparsers(dest="command", required=True)
    st = sub.add_parser("status",
                        help="read positions and hold state; moves nothing")
    st.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    for verb, helptext in (("jog", "move by N degrees, relative"),
                           ("goto", "move to N degrees, absolute")):
        s = sub.add_parser(verb, help=helptext)
        s.add_argument("axis", choices=sorted(AXES))
        s.add_argument("degrees", type=float)
    pt = sub.add_parser("point", help="drive both axes to a yaw/pitch pose")
    pt.add_argument("--yaw", type=float, required=True,
                    help=f"pan, degrees ({PAN_LIMITS[0]:+.0f} .. {PAN_LIMITS[1]:+.0f})")
    pt.add_argument("--pitch", type=float, required=True,
                    help=f"tilt, degrees ({TILT_LIMITS[0]:+.0f} .. {TILT_LIMITS[1]:+.0f})")
    pt.add_argument("--yaw-speed", type=float, default=None, metavar="DEG_S",
                    help="deg/s for the yaw axis only (default: --speed)")
    pt.add_argument("--pitch-speed", type=float, default=None, metavar="DEG_S",
                    help="deg/s for the pitch axis only (default: --speed)")
    dm = sub.add_parser(
        "demo", help="sweep each axis limit to limit at several speeds"
    )
    dm.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    dm.add_argument("--speeds", default="10,30,60",
                    help="comma-separated deg/s to sweep at")
    dm.add_argument("--fraction", type=float, default=1.0,
                    metavar="F",
                    help="fraction of each limit to sweep, 0<F<=1; use "
                         "0.5 the first time on an untested machine")
    hm = sub.add_parser("home", help="drive the selected axes to 0")
    hm.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    sz = sub.add_parser("set-zero", help="define the current position as zero")
    sz.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    sz.add_argument("--here", action="store_true",
                    help="zero where the machine stands now, without releasing "
                         "the motors and asking you to centre it first")
    pr = sub.add_parser(
        "params", help="write the board's whole configuration "
                       "(not its CAN identity)"
    )
    pr.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    pr.add_argument("--dry-run", action="store_true",
                    help="print the plan without sending anything")
    cur = sub.add_parser(
        "current", help="set working and/or holding current (cannot be read back)"
    )
    cur.add_argument("--holding", type=int, choices=range(10, 100, 10),
                     metavar="PERCENT",
                     help="holding current as a percent of working current, "
                          "10-90 in steps of 10; ignored by vFOC mode")
    cur.add_argument("--working", type=int, metavar="MA",
                     help=f"working current in mA, 0-{MAX_WORKING_MA}; applies "
                          f"in every mode, and costs torque for moves too")
    cur.add_argument("--axis", choices=sorted(AXES), default=None,
                     help="just one axis (default: both)")
    md = sub.add_parser(
        "mode", help="set the work mode; releases the motors first"
    )
    md.add_argument("mode", choices=sorted(MODES),
                    help="close = SR_CLOSE (encoder loop, honours holding "
                         "current), foc = SR_VFOC (ignores holding current), "
                         "open = SR_OPEN (corrects nothing)")
    md.add_argument("--axis", choices=sorted(AXES), default=None)
    w = sub.add_parser(
        "watch", help="stream position, following error and protection state"
    )
    w.add_argument("--axis", choices=sorted(AXES), default="pan")
    w.add_argument("--hz", type=float, default=5.0, help="samples per second")
    rl = sub.add_parser("release",
                        help="motors off, axes free to turn by hand")
    rl.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    hd = sub.add_parser("hold", help="motors on, holding position")
    hd.add_argument("--axis", choices=sorted(AXES), default=None,
                    help="just one axis (default: both)")
    return p


async def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.ERROR,
        format="%(levelname)s %(name)s: %(message)s",
    )

    if args.command == "current" and args.working is not None:
        if not (0 <= args.working <= MAX_WORKING_MA):
            parser.error(
                f"--working {args.working} mA is outside 0..{MAX_WORKING_MA}. "
                f"That ceiling is the\nrating of the 28D/35D/42D; a 57D takes "
                f"5200, but nothing here can read back which\nboard it is "
                f"talking to, so it will not guess on your behalf."
            )

    handler, moves = COMMANDS[args.command]
    if moves and not args.zeroed:
        parser.error(
            f"'{args.command}' moves the machine and needs --zeroed.\n"
            "The motors zero their encoder wherever they are standing at "
            "power-on, so the\nsoft limits do not describe an angle until the "
            "centre has been set:\n\n"
            "    python examples/camera_gimbal_tracker.py --hardware "
            "--two-axis --set-zero\n\n"
            "Then add --zeroed here. 'status' and 'release' work without it."
        )

    can_if = CANInterface(
        interface_type=args.interface, channel=args.channel,
        bitrate=args.bitrate,
    )
    await can_if.connect()
    try:
        return await handler(can_if, args)
    finally:
        await can_if.disconnect()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
