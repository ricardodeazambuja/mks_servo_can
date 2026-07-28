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
    for name in AXES:
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
    """Sends one absolute move and reports where the axis actually ended up."""
    axis = _axis(can_if, name)
    await _unmute(axis)
    await axis.enable_motor()
    before = await axis.get_current_position_user()
    if relative_from is not None:
        target = before + relative_from
        check_limits(name, target)
    print(f"{name}: {before:+.2f} -> {target:+.2f} deg at {speed:.0f} deg/s")
    await axis.move_to_position_abs_user(target, speed_user=speed)
    after = await axis.get_current_position_user()
    # Report the encoder, not the fact the command returned: where the shaft
    # ended up is the thing being tested.
    print(f"{name}: now {after:+.2f} deg  (asked for {target:+.2f})")
    return 0


async def cmd_jog(can_if: CANInterface, args) -> int:
    """Moves one axis by a relative number of degrees."""
    return await _move(can_if, args.axis, 0.0, args.speed, relative_from=args.degrees)


async def cmd_goto(can_if: CANInterface, args) -> int:
    """Moves one axis to an absolute position."""
    check_limits(args.axis, args.degrees)
    return await _move(can_if, args.axis, args.degrees, args.speed, None)


async def cmd_home(can_if: CANInterface, args) -> int:
    """Drives every axis to zero."""
    for name in AXES:
        await _move(can_if, name, 0.0, args.speed, None)
    return 0


async def cmd_point(can_if: CANInterface, args) -> int:
    """Drives both axes to an absolute yaw/pitch pair, together.

    Yaw and pitch are named rather than positional on purpose. The two axes do
    not share limits - yaw is +/-90 and pitch is -45..+90 - so a transposed pair
    is not a slightly wrong pose, it is the yaw axis being driven against the
    pitch axis' allowance. Naming them makes that mistake impossible to make
    silently, at the cost of a few more characters.
    """
    check_limits("pan", args.yaw)
    check_limits("tilt", args.pitch)

    # Both axes are checked before either moves. Sending yaw and then
    # discovering pitch is out of range would leave the machine somewhere
    # nobody asked for, half way through a pose.
    print(f"yaw (pan)   -> {args.yaw:+.2f} deg")
    print(f"pitch (tilt)-> {args.pitch:+.2f} deg")

    async def one(name: str, target: float):
        axis = _axis(can_if, name)
        await _unmute(axis)
        await axis.enable_motor()
        await axis.move_to_position_abs_user(target, speed_user=args.speed)
        return name, await axis.get_current_position_user()

    # Together, not one after the other: a gimbal that yaws fully and then
    # pitches sweeps a different path through its own cabling than one that
    # does both at once, and the second is the pose you actually asked for.
    results = await asyncio.gather(
        one("pan", args.yaw), one("tilt", args.pitch),
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


async def cmd_set_zero(can_if: CANInterface, args) -> int:
    """Defines the current position of both axes as zero.

    This is the thing every limit is measured from, so it is worth being
    deliberate about. By default the motors are released and you are asked to
    centre the axes by hand first; `--here` skips that and zeroes wherever the
    machine is standing, which is right when you have just positioned it and
    wrong the rest of the time.
    """
    axes = {name: _axis(can_if, name) for name in AXES}
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


async def cmd_release(can_if: CANInterface, args) -> int:
    """Disables both motors so the axes can be turned by hand."""
    ok = True
    for name in AXES:
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
    for name in AXES:
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
    "set-zero": (cmd_set_zero, False),
    "release": (cmd_release, False),
    "hold": (cmd_hold, False),
}


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
    sub.add_parser("status", help="read positions and hold state; moves nothing")
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
    sub.add_parser("home", help="drive every axis to 0")
    sz = sub.add_parser("set-zero", help="define the current position as zero")
    sz.add_argument("--here", action="store_true",
                    help="zero where the machine stands now, without releasing "
                         "the motors and asking you to centre it first")
    sub.add_parser("release", help="motors off, axes free to turn by hand")
    sub.add_parser("hold", help="motors on, holding position")
    return p


async def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.ERROR,
        format="%(levelname)s %(name)s: %(message)s",
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
