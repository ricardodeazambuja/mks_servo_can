#!/usr/bin/env python3
"""
Three-axis camera gimbal that tracks a fast-moving target.

Runs against the simulator out of the box::

    mks-servo-simulator --num-motors 3 --start-can-id 1 --latency-ms 0
    python examples/camera_gimbal_tracker.py

Or against hardware::

    python examples/camera_gimbal_tracker.py --hardware --channel can0

The synthetic target is a drone flying a pass at configurable range and speed.
Replace `SyntheticTarget` with your detector to make this real; the interface is
one method returning a bearing and the time it was *captured*.


The one thing to understand before changing anything
====================================================

This is a latency problem, not a speed problem.

The motors have far more acceleration than the job needs. At ``acc=250`` the
shaft accelerates at 20000 deg/s^2; a drone pulling 4 g laterally at 10 m range
only demands 225 deg/s^2, so there is roughly a 90x margin. The CAN bus is
likewise not the constraint: three axes streaming fire-and-forget at 1 Mbit/s
have around 1000 Hz of headroom at 40% bus load, and this loop uses 200 Hz.

What actually determines whether the target stays in frame is the delay between
photons landing on the sensor and the motor moving, because pointing error from
pure transport delay is ``rate x latency``. For a target crossing at 172 deg/s:

    latency   no prediction   constant-velocity prediction
     10 ms        1.72 deg           0.011 deg
     33 ms        5.68 deg           0.123 deg
     50 ms        8.60 deg           0.281 deg
     80 ms       13.76 deg           0.720 deg

A telephoto lens has a field of view of a few degrees. Without extrapolation the
target leaves the frame; with it, the error is a fraction of a degree. Shaving
3 ms off the CAN path is worth about 0.5 deg. Adding the predictor is worth 8.

Hence the two things this example is really demonstrating:

1. `AlphaBetaGammaTracker` extrapolates the target forward by the *measured*
   pipeline latency. Measure that number, do not guess it: being wrong by 20 ms
   costs 3.4 deg at 172 deg/s. A constant-*acceleration* filter is used rather
   than constant-velocity because a target flying a straight line still has
   large angular acceleration near the crossing point - the geometry
   accelerates even though the target does not. See `GimbalTracker`.
2. `ServoStream` streams absolute position targets with a velocity feed-forward,
   fire-and-forget, so the motor's own closed loop does the fine positioning
   while this process does prediction and trajectory.


Mechanical notes that matter more than the code
===============================================

**Use 1:1 direct drive on pan and tilt.** The 16384-count encoder gives 0.022
deg (79 arcsec) at the output, already about 0.4% of a 6 deg field of view and
well below the prediction residual. Reduction buys resolution you cannot use,
costs you slew rate, and introduces backlash, which is a nonlinearity no
controller can compensate and which shows up as visible jitter every time the
tracking error changes sign. Only gear down for a genuinely long lens (< 2 deg
FOV), and then use a zero-backlash drive - harmonic, capstan, or a tensioned
belt. Never a spur gearbox.

**Put the SERVO57D on pan.** Carrying the tilt and roll stages, the pan axis has
roughly 0.018 kg m^2 of inertia with a 1.2 kg camera. A SERVO42D leaves about 3x
acceleration margin there, which disappears the moment the gimbal is slightly
out of balance or you fit a heavier lens. The SERVO57D gives about 8x. Tilt and
roll are comfortable on SERVO42Ds.

**Balance every axis.** A stepper holding a static gravity torque burns holding
current continuously, heats up, and loses torque exactly when you need it.
Re-balance after every lens change.

**Configure the motors** for SR_vFOC (mode 5), 32 or 64 microsteps - staying in
the 16/32/64 band keeps speed parameter equal to RPM, see
`mks_servo_can.motor_profile` - and 1 Mbit/s CAN with 120 ohm termination at
both physical ends of the bus only.

For roll: if you are tracking rather than filming, roll contributes nothing to
acquisition. Consider spending that third motor on a focus axis instead, which
matters far more for keeping a small distant target resolvable.
"""
import argparse
import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from mks_servo_can import (
    AlphaBetaGammaTracker,
    Axis,
    CANInterface,
    MotorError,
    RotaryKinematics,
    ServoStream,
    StreamAxis,
    motor_profile,
)
from mks_servo_can import (
    constants as const,
)

logger = logging.getLogger("gimbal")

# --------------------------------------------------------------------------
# Gimbal geometry
# --------------------------------------------------------------------------

# Soft limits. Pan is restricted rather than continuous because cabling to the
# camera has to come back down through the yoke; a slip ring would let you open
# this up. Tilt is restricted to keep the camera clear of the base.
#
# +/-90 on pan is measured off the built machine, not chosen: the loom to the
# tilt motor and the camera comes back down through the yoke, and it runs out of
# slack there. This said +/-170 until the first assembled gimbal showed that was
# most of a turn past what the cables tolerate. If you add a slip ring, this is
# the number to open up - and it is the only one.
PAN_LIMITS = (-90.0, 90.0)
TILT_LIMITS = (-45.0, 90.0)
ROLL_LIMITS = (-30.0, 30.0)

# acc=250 gives 20000 deg/s^2 at the shaft, about 90x what a 4 g target at 10 m
# demands. The headroom is deliberate: it is what lets a streamed position
# command be tracked faithfully inside one control period. Do not use acc=0,
# which removes the ramp entirely and will jar the footage and risk lost sync.
GIMBAL_ACCEL_PARAM = 250

CONTROL_RATE_HZ = 200.0
FEEDBACK_RATE_HZ = 25.0


@dataclass
class Bearing:
    """
    A sighting of the target.

    Attributes:
        azimuth: Bearing in degrees, in the gimbal's pan frame.
        elevation: Bearing in degrees, in the gimbal's tilt frame.
        captured_at: When the measurement was *captured*, on
            `time.monotonic()`. Not when it finished being processed: folding a
            variable processing delay into the timestamp corrupts the velocity
            estimate in a way no filter can undo. Real camera drivers expose an
            exposure timestamp; use it.
    """

    azimuth: float
    elevation: float
    captured_at: float


class SyntheticTarget:
    """
    A drone flying a straight pass, used so the example runs with no camera.

    The default is deliberately demanding: 30 m/s at 10 m closest approach gives
    a peak angular rate of about 172 deg/s at the crossing point, which is the
    worst case the design notes above are written around.

    Replace this class with your detector. The only contract is `observe()`.
    """

    def __init__(
        self,
        speed_mps: float = 30.0,
        closest_approach_m: float = 10.0,
        altitude_m: float = 15.0,
        detection_latency: float = 0.045,
        detection_noise_deg: float = 0.15,
    ):
        """
        Configures the synthetic pass.

        Args:
            speed_mps: Ground speed of the target.
            closest_approach_m: Perpendicular distance at the crossing point.
            altitude_m: Height above the gimbal.
            detection_latency: Simulated pipeline delay - exposure, readout and
                inference - between the target being at a position and that
                position becoming available here.
            detection_noise_deg: Peak bearing noise, applied deterministically
                so runs are reproducible.
        """
        self.speed_mps = speed_mps
        self.closest_approach_m = closest_approach_m
        self.altitude_m = altitude_m
        self.detection_latency = detection_latency
        self.detection_noise_deg = detection_noise_deg
        self._t0 = time.monotonic()
        self._sample = 0

    def true_bearing(self, t: float) -> Tuple[float, float]:
        """
        Returns the target's exact bearing at a given time.

        Args:
            t: Seconds since the pass began.

        Returns:
            (azimuth, elevation) in degrees.
        """
        # Fly along +x, offset by the closest approach in y.
        x = self.speed_mps * (t - 4.0)
        y = self.closest_approach_m
        z = self.altitude_m
        azimuth = math.degrees(math.atan2(x, y))
        elevation = math.degrees(math.atan2(z, math.hypot(x, y)))
        return azimuth, elevation

    def observe(self) -> Bearing:
        """
        Returns a delayed, noisy sighting - what a real detector would give you.

        Returns:
            A `Bearing` timestamped with its capture time, which is already in
            the past by `detection_latency`.
        """
        now = time.monotonic()
        captured_at = now - self.detection_latency
        azimuth, elevation = self.true_bearing(captured_at - self._t0)

        # Deterministic zero-mean jitter: an irrational stride avoids the
        # periodicity that would let the filter learn the "noise".
        self._sample += 1
        noise = self.detection_noise_deg * math.sin(self._sample * 2.399963)
        return Bearing(azimuth + noise, elevation + noise * 0.5, captured_at)


class GimbalTracker:
    """
    Closes the loop from sighting to motor command.

    Owns one `AlphaBetaTracker` per steered axis and one `ServoStream` for the
    whole gimbal, and does the only interesting arithmetic in the example:
    extrapolating each bearing forward to when the command will actually take
    effect.
    """

    def __init__(
        self,
        stream: ServoStream,
        command_latency: float,
        alpha: float = 0.5,
        beta: float = 0.3,
        gamma: float = 0.05,
    ):
        """
        Builds a tracker around a running stream.

        A constant-acceleration filter is used rather than constant-velocity,
        because a target flying a *straight line* still has large angular
        acceleration near the crossing point: the geometry accelerates even
        though the target does not. A 30 m/s pass at 10 m peaks at about
        335 deg/s^2. A constant-velocity filter's velocity estimate lags that by
        roughly `accel * dt / beta`, which at 60 fps and beta=0.08 is 70 deg/s -
        over a 50 ms horizon, 3.5 degrees of pointing error, making the filter a
        larger error source than everything else combined.

        Args:
            stream: The `ServoStream` driving the gimbal.
            command_latency: Measured delay in seconds between issuing a command
                and the motor acting on it. Only this part - the *detector*
                delay is derived per sighting from its capture timestamp, so it
                adapts automatically to a jittery pipeline. See
                `measure_command_latency`.
            alpha: Position gain. Higher tracks harder and admits more noise.
            beta: Velocity gain.
            gamma: Acceleration gain. The most noise-sensitive of the three.

        The default gains come from a sweep against this example's trajectory
        with 0.15 deg of bearing noise: they give about 0.24 deg of error at the
        crossing point and degrade gracefully as noise rises. Re-tune for your
        own detector's noise and frame rate.
        """
        self.stream = stream
        self.command_latency = command_latency
        self.azimuth = AlphaBetaGammaTracker(alpha=alpha, beta=beta, gamma=gamma)
        self.elevation = AlphaBetaGammaTracker(alpha=alpha, beta=beta, gamma=gamma)
        self.last_error: Optional[float] = None

    def on_sighting(self, bearing: Bearing) -> None:
        """
        Folds in a sighting and commands the gimbal.

        The prediction horizon is the age of this measurement plus the command
        latency: how far in the future the motor will act, measured from when
        the photons landed. Deriving the detector's share from the capture
        timestamp rather than assuming a fixed figure means a pipeline whose
        latency varies frame to frame is handled correctly and for free.

        Getting this wrong is expensive and silent. Adding the detector latency
        a second time - easy to do, since it is already inside `age` - doubles
        the horizon and over-predicts: in this example that turns 0.24 deg of
        error at the crossing point into 7.4 deg, worse than not predicting at
        all.

        Args:
            bearing: The sighting to process.
        """
        self.azimuth.update(bearing.azimuth, bearing.captured_at)
        self.elevation.update(bearing.elevation, bearing.captured_at)

        age = time.monotonic() - bearing.captured_at
        horizon = age + self.command_latency

        predicted_az = self.azimuth.predict(horizon)
        predicted_el = self.elevation.predict(horizon)

        # The feed-forward rate is what stops the motor decelerating into every
        # target. Without it the gimbal stutters: it arrives, stops, and waits
        # for the next command. With it, the trapezoidal planner inside the
        # driver is already moving at roughly the target's speed.
        #
        # Use the rate the target will have when the command *lands*, not the
        # one it had when last seen - the same extrapolation argument as for
        # position.
        self.stream.set_targets(
            {"pan": predicted_az, "tilt": predicted_el},
            {
                "pan": abs(self.azimuth.predict_velocity(horizon)),
                "tilt": abs(self.elevation.predict_velocity(horizon)),
            },
        )

    def pointing_error(self, truth: Tuple[float, float]) -> float:
        """
        Angular distance between where the gimbal is aimed and the truth.

        Only meaningful with a synthetic target, where truth is known. Kept
        because it is the number that tells you whether any of this works.

        Args:
            truth: The target's exact (azimuth, elevation) in degrees.

        Returns:
            Great-circle-ish angular error in degrees.
        """
        pan = self.stream.axes["pan"].target_position
        tilt = self.stream.axes["tilt"].target_position
        d_az = (truth[0] - pan) * math.cos(math.radians(tilt))
        d_el = truth[1] - tilt
        self.last_error = math.hypot(d_az, d_el)
        return self.last_error


def build_axes(
    include_roll: bool = True, can_ids: Optional[dict] = None
) -> list:
    """
    Describes the gimbal axes.

    Roll is optional because a two-motor build is a perfectly good tracking
    gimbal: roll does not change where the camera points, only how the frame is
    oriented about the optical axis. The printed pan/tilt design in
    `hardware/gimbal/` is exactly this two-axis case.

    Args:
        include_roll: If False, only pan and tilt are built.
        can_ids: Optional mapping of axis name to CAN ID, overriding the
            defaults. A two-axis build made from the boards on the bench uses
            `{"pan": 2, "tilt": 3}`.

    Returns:
        The `StreamAxis` objects for the requested axes.
    """
    # A two-axis build is the printed gimbal, and on that machine the boards are
    # 2 and 3 - the pan (yaw) motor is 2 and the tilt (pitch) motor is 3, which
    # is the one that used to carry the pen. Defaulting to that here means the
    # hardware case does not need --can-ids to be correct, and getting it wrong
    # drives the yaw axis with the tilt limits.
    # Roll keeps an id either way: every spec below is constructed before the
    # include_roll filter is applied at the end of this function.
    ids = {"pan": 1, "tilt": 2, "roll": 3}
    if not include_roll:
        ids.update({"pan": 2, "tilt": 3})
    ids.update(can_ids or {})
    built = [
        StreamAxis(
            "pan",
            can_id=ids["pan"],
            accel_param=GIMBAL_ACCEL_PARAM,
            min_position=PAN_LIMITS[0],
            max_position=PAN_LIMITS[1],
            # 3000 RPM at 1:1 is 18000 deg/s, but stepper torque collapses well
            # before that. 1000 RPM (6000 deg/s) is a defensible working ceiling
            # and still 35x the fastest target this is designed for.
            max_rate=6000.0,
            microsteps=32,
        ),
        StreamAxis(
            "tilt",
            can_id=ids["tilt"],
            accel_param=GIMBAL_ACCEL_PARAM,
            min_position=TILT_LIMITS[0],
            max_position=TILT_LIMITS[1],
            max_rate=6000.0,
            microsteps=32,
        ),
        StreamAxis(
            "roll",
            can_id=ids["roll"],
            accel_param=GIMBAL_ACCEL_PARAM,
            min_position=ROLL_LIMITS[0],
            max_position=ROLL_LIMITS[1],
            max_rate=3000.0,
            microsteps=32,
        ),
    ]
    return built if include_roll else [a for a in built if a.name != "roll"]


async def measure_command_latency(stream: ServoStream, axis: str = "pan") -> float:
    """
    Measures how long a command takes to reach the motor and start it moving.

    This is only the *command* half of the pipeline, and deliberately so: the
    camera half is derived per sighting from its capture timestamp, which keeps
    the horizon correct even when the detector's latency varies.

    If your camera does not give you a trustworthy capture timestamp you will
    have to measure its delay directly - put a blinking LED in frame, command a
    known step, and cross-correlate - and subtract it when you build `Bearing`.

    Args:
        stream: A running `ServoStream`.
        axis: Which axis to probe.

    Returns:
        Median command latency in seconds.
    """
    samples = []
    target = stream.axes[axis].target_position
    for i in range(40):
        started = time.perf_counter()
        stream.set_target(axis, target + (0.05 if i % 2 else -0.05))
        await asyncio.sleep(1.0 / CONTROL_RATE_HZ)
        samples.append(time.perf_counter() - started)
    samples.sort()
    return samples[len(samples) // 2]


def report_design_margins() -> None:
    """
    Prints the sizing calculations behind the constants at the top of the file.

    Worth reading once: it shows which constraints are comfortable (acceleration,
    bus bandwidth, encoder resolution) and which are not (latency).
    """
    resolution = motor_profile.encoder_resolution_degrees()
    accel = motor_profile.accel_param_to_deg_per_s2(GIMBAL_ACCEL_PARAM)
    max_rate = motor_profile.max_output_speed_deg_per_s(max_usable_rpm=1000)

    print("Gimbal design margins (1:1 direct drive)")
    print("-" * 62)
    print(f"  encoder resolution    {resolution:8.4f} deg  ({resolution*3600:.1f} arcsec)")
    print(f"  acceleration (acc={GIMBAL_ACCEL_PARAM})  {accel:8.0f} deg/s^2")
    print(f"  usable slew rate      {max_rate:8.0f} deg/s   (at 1000 RPM)")
    print()
    print("  worst-case target: 30 m/s drone at 10 m closest approach")
    print(f"    angular rate         {math.degrees(30/10):8.1f} deg/s")
    print(f"    angular accel (4 g)  {math.degrees(4*9.81/10):8.1f} deg/s^2")
    print(f"    acceleration margin  {accel/math.degrees(4*9.81/10):8.0f}x")
    print(f"    slew rate margin     {max_rate/math.degrees(30/10):8.0f}x")
    print()
    print("  => acceleration and slew rate are not the constraint. Latency is.")
    print()


async def set_zero(
    can_if: CANInterface, include_roll: bool = True,
    can_ids: Optional[dict] = None,
) -> None:
    """
    Walks the operator through defining the mechanical centre as zero.

    THE SOFT LIMITS ARE MEANINGLESS WITHOUT THIS. The motor sets its encoder to
    zero when it powers on, so position 0 is wherever the shaft happened to be
    sitting at switch-on, not the middle of the machine's travel. Power up with
    pan at +80 deg of its real range and `PAN_LIMITS` of +/-90 permits -10..+170
    - the limits report success the whole way into the cable loom. The limits
    constrain numbers; only this makes those numbers mean an angle.

    So: motors off, centre both axes by hand, and tell the motor that this is
    zero. Nothing is commanded to move at any point, because until zero is
    established there is no such thing as a safe target.

    Args:
        can_if: A connected `CANInterface`.
        include_roll: Whether a roll axis is present.
        can_ids: Optional mapping of axis name to CAN ID.

    Raises:
        MotorError, CommunicationError: If a motor will not disable or accept
            the new zero.
    """
    specs = build_axes(include_roll=include_roll, can_ids=can_ids)
    axes = {
        spec.name: Axis(
            can_if, motor_can_id=spec.can_id, name=spec.name,
            kinematics=RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION),
        )
        for spec in specs
    }

    print("Releasing the motors so the axes can be turned by hand.")
    for name, axis in axes.items():
        await axis.disable_motor()
        print(f"  {name:5s} (CAN {axis.can_id}) released")

    print(
        "\nCentre every axis by hand now:\n"
        "  pan  - camera facing straight forward, cable loom slack and even\n"
        "  tilt - camera level, its own weight balanced on the axis\n"
        + ("  roll - horizon level in frame\n" if include_roll else "")
        + "\nTake the slack in the loom to one side and back to check that the "
        "centre you\npick really is the middle of the travel, not the middle of "
        "what is convenient."
    )
    input("\nPress Enter when both axes are centred: ")

    print()
    for name, axis in axes.items():
        before = await axis.get_current_position_user()
        await axis.set_current_position_as_zero()
        # Read it back rather than trusting the write. This is one CAN round
        # trip against the risk of running a whole session on limits anchored to
        # a zero the motor never accepted.
        after = await axis.get_current_position_user()
        ok = abs(after) < 0.1
        print(
            f"  {'OK ' if ok else 'FAIL'} {name:5s} was {before:+8.2f} deg, "
            f"now reads {after:+8.2f} deg"
        )
        if not ok:
            raise MotorError(
                f"Axis '{name}' did not accept the new zero: still reads "
                f"{after:.2f} deg."
            )

    lim = {"pan": PAN_LIMITS, "tilt": TILT_LIMITS, "roll": ROLL_LIMITS}
    print("\nZero set. The limits now mean these angles about the centre:")
    for name in axes:
        low, high = lim[name]
        print(f"  {name:5s} {low:+7.1f} .. {high:+7.1f} deg")
    print(
        "\nThis holds until the motors lose power. After any power cycle the "
        "encoder\nzeroes itself wherever the shaft is standing, so run "
        "--set-zero again."
    )


# How fast to drive back to home when parking. Deliberately slow: this is the
# one move made with nobody watching a tracking loop, and a gimbal walking back
# to centre at 30 deg/s is easy to catch by hand if the zero turns out wrong.
PARK_SPEED_DEG_S = 30.0

# How close to zero a parked axis has to land for the park to count.
PARK_TOL_DEG = 0.5


async def park_at_home(
    can_if: CANInterface, include_roll: bool = True,
    can_ids: Optional[dict] = None,
) -> None:
    """
    Drives every axis back to zero and holds it there, ready for power-off.

    This is what makes home survive a power cycle without any firmware feature.
    The motor zeroes its encoder wherever the shaft is standing at switch-on, so
    if the machine is *always* switched off at home, then zero is home the next
    time it comes up - the same property that makes an unparked machine
    dangerous is what carries the reference across, for free.

    It degrades safely. Forget to park, and the next session is simply back to
    needing `--set-zero`; nothing silently drifts, because a machine powered off
    somewhere else comes up believing it is at zero and the operator is the one
    who knows it was not parked. That is why this prints what it did rather than
    exiting quietly.

    The motors are left **enabled and holding** at the end, because a released
    axis can be nudged between parking and switch-off, which would put the
    reference back where it started. Do not leave it in this state: these
    drivers hold full current regardless of load, and the motor reaches PLA's
    glass transition doing it.

    Args:
        can_if: A connected `CANInterface`.
        include_roll: Whether a roll axis is present.
        can_ids: Optional mapping of axis name to CAN ID.

    Raises:
        MotorError: If an axis does not reach home.
    """
    specs = build_axes(include_roll=include_roll, can_ids=can_ids)
    axes = {
        spec.name: Axis(
            can_if, motor_can_id=spec.can_id, name=spec.name,
            kinematics=RotaryKinematics(
                steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
            ),
        )
        for spec in specs
    }

    print(f"Parking at home, {PARK_SPEED_DEG_S:.0f} deg/s.\n")
    for name, axis in axes.items():
        # Ask for move-completion messages back before making a waited move.
        #
        # `ServoStream` mutes the motors on entry (0x8C) and restores them with
        # active initiation still OFF, deliberately: a streaming loop overwrites
        # its own target every few milliseconds and the abort frame for the
        # superseded move is indistinguishable from the acknowledgement of the
        # new one. But "active" is exactly what makes the motor volunteer the
        # completion message, so an ordinary `wait=True` move made after a
        # tracking run waits for a frame that will never come. Parking straight
        # after tracking - the whole point of this mode - timed out at 5.6 s
        # with the axis sitting right where it had been left.
        await axis._low_level_api.set_slave_respond_active(
            axis.can_id, respond_enabled=True, active_enabled=True
        )
        await axis.enable_motor()
        before = await axis.get_current_position_user()
        await axis.move_to_position_abs_user(0.0, speed_user=PARK_SPEED_DEG_S)
        # Read the encoder rather than trusting the move to have landed: the
        # whole point of parking is where the shaft physically ends up, which is
        # exactly the thing a completed-move notification does not tell you.
        after = await axis.get_current_position_user()
        ok = abs(after) <= PARK_TOL_DEG
        print(
            f"  {'OK ' if ok else 'FAIL'} {name:5s} {before:+8.2f} -> "
            f"{after:+8.2f} deg"
        )
        if not ok:
            raise MotorError(
                f"Axis '{name}' stopped {after:+.2f} deg from home, outside the "
                f"{PARK_TOL_DEG:.1f} deg the park allows. Do not power off yet: "
                f"this position would become the next session's zero."
            )

    print(
        "\nAt home and holding. **Power the motors off now**, and the next "
        "power-on\ncomes up already zeroed - no --set-zero needed, just "
        "--zeroed.\n\n"
        "Two things this does not survive: moving an axis by hand after the "
        "power is\noff, and powering down anywhere other than here. Either way "
        "the fix is the\nsame, --set-zero again.\n\n"
        "Do not leave it holding. These drivers pull full current regardless "
        "of load."
    )


async def run_tracking(
    can_if: CANInterface, duration: float, quiet: bool = False,
    include_roll: bool = True, can_ids: Optional[dict] = None,
) -> dict:
    """
    Runs a tracking pass and reports how well it went.

    Args:
        can_if: A connected `CANInterface`.
        duration: How long to track, in seconds.
        quiet: Suppress the per-second progress lines.
        include_roll: Build a three-axis gimbal. False gives pan/tilt only.
        can_ids: Optional per-axis CAN ID overrides.

    Returns:
        A summary dict with the error statistics and loop timing.
    """
    target = SyntheticTarget()
    axes = build_axes(include_roll=include_roll, can_ids=can_ids)

    async with ServoStream(
        can_if,
        axes,
        rate_hz=CONTROL_RATE_HZ,
        feedback_rate_hz=FEEDBACK_RATE_HZ,
        # A gimbal whose detector has crashed should hold, not coast into its
        # own cabling.
        watchdog_timeout=0.5,
    ) as stream:
        stream.set_targets({axis.name: 0.0 for axis in axes})
        await asyncio.sleep(0.2)

        command_latency = await measure_command_latency(stream)
        total_latency = command_latency + target.detection_latency
        if not quiet:
            print(
                f"  measured command latency {command_latency*1000:.2f} ms, "
                f"detector latency {target.detection_latency*1000:.0f} ms "
                f"-> prediction horizon {total_latency*1000:.0f} ms\n"
            )

        tracker = GimbalTracker(stream, command_latency=command_latency)

        errors = []
        naive_errors = []
        started = time.monotonic()
        next_report = started + 1.0
        detector_period = 1.0 / 60.0  # a 60 fps camera

        while time.monotonic() - started < duration:
            sighting = target.observe()
            tracker.on_sighting(sighting)

            truth = target.true_bearing(time.monotonic() - target._t0)
            rate = abs(tracker.azimuth.velocity)
            errors.append((rate, tracker.pointing_error(truth)))
            # What the error would have been pointing straight at the stale
            # measurement, with no extrapolation at all.
            naive_errors.append(
                (
                    rate,
                    math.hypot(
                        (truth[0] - sighting.azimuth)
                        * math.cos(math.radians(sighting.elevation)),
                        truth[1] - sighting.elevation,
                    ),
                )
            )

            if not quiet and time.monotonic() > next_report:
                next_report += 1.0
                print(
                    f"  t={time.monotonic()-started:4.1f}s  "
                    f"az={truth[0]:+7.1f} deg  el={truth[1]:5.1f} deg  "
                    f"rate={abs(tracker.azimuth.velocity):6.1f} deg/s  "
                    f"error={errors[-1][1]:5.3f} deg"
                )
            await asyncio.sleep(detector_period)

        # Segment by target angular rate. Averaging over a whole pass is
        # misleading: most of a pass is slow and far away, where any approach
        # works. The question is what happens at the crossing point, which is a
        # small fraction of the samples but the entire reason for the design.
        return {
            "samples": len(errors),
            "prediction_horizon_ms": total_latency * 1000.0,
            "bands": _summarise_by_rate(errors, naive_errors),
            "loop": stream.stats.as_dict(),
        }


RATE_BANDS = [
    ("slow    (< 20 deg/s)", 0.0, 20.0),
    ("moderate (20-60 deg/s)", 20.0, 60.0),
    ("fast     (60-120 deg/s)", 60.0, 120.0),
    ("crossing (> 120 deg/s)", 120.0, float("inf")),
]


def _summarise_by_rate(errors, naive_errors) -> list:
    """
    Groups pointing errors by how fast the target was moving.

    Args:
        errors: (angular_rate, error) pairs with prediction enabled.
        naive_errors: (angular_rate, error) pairs without prediction.

    Returns:
        One dict per rate band that had samples, each with median and worst
        error for both approaches.
    """
    summary = []
    for label, low, high in RATE_BANDS:
        tracked = sorted(e for r, e in errors if low <= r < high)
        naive = sorted(e for r, e in naive_errors if low <= r < high)
        if not tracked:
            continue
        summary.append(
            {
                "band": label,
                "samples": len(tracked),
                "median_deg": tracked[len(tracked) // 2],
                "worst_deg": tracked[-1],
                "median_naive_deg": naive[len(naive) // 2] if naive else float("nan"),
            }
        )
    return summary


async def main() -> None:
    """Parses arguments, connects, and runs one tracking pass."""
    parser = argparse.ArgumentParser(
        description="Three-axis camera gimbal tracking demonstration.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--hardware", action="store_true", help="use a real CAN bus (default: simulator)"
    )
    parser.add_argument("--channel", default="can0", help="CAN channel for --hardware")
    parser.add_argument(
        "--interface", default="socketcan", help="python-can interface type"
    )
    parser.add_argument(
        "--bitrate", type=int, default=1000000, help="CAN bitrate (default 1 Mbit/s)"
    )
    parser.add_argument(
        "--simulator-port", type=int, default=6789, help="simulator TCP port"
    )
    parser.add_argument(
        "--duration", type=float, default=8.0, help="seconds to track"
    )
    parser.add_argument(
        "--two-axis", action="store_true",
        help="pan and tilt only, for the printed hardware/gimbal/ design",
    )
    parser.add_argument(
        "--can-ids", default=None,
        help="per-axis CAN IDs, e.g. 'pan=2,tilt=3' (two-axis defaults to this)",
    )
    parser.add_argument(
        "--set-zero", action="store_true",
        help="release the motors, have you centre the axes by hand, and define "
             "that as zero; do this after every power cycle before tracking",
    )
    parser.add_argument(
        "--park", action="store_true",
        help="drive every axis back to home and hold, so the machine can be "
             "powered off there and comes up already zeroed; needs --zeroed",
    )
    parser.add_argument(
        "--zeroed", action="store_true",
        help="confirm the axes have been zeroed with --set-zero since the last "
             "power cycle; required by --hardware, because the soft limits are "
             "measured from that zero and the motor re-zeroes itself at power-on",
    )
    parser.add_argument("--quiet", action="store_true", help="summary only")
    parser.add_argument(
        "--verbose", action="store_true", help="enable library debug logging"
    )
    args = parser.parse_args()

    # The library logs every frame at DEBUG. Leave it off in a control loop:
    # even discarded records cost formatting time on the hot path.
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    # The first thing this loop does is slew every axis to 0.0, so on hardware a
    # wrong zero is not a degraded run - it is the run that drives the machine
    # into its own cabling before tracking starts. Refuse rather than warn.
    if args.hardware and not (args.zeroed or args.set_zero):
        parser.error(
            "refusing to drive hardware without --zeroed.\n"
            "The motors zero their encoders wherever they are standing at "
            "power-on, so the\nsoft limits do not describe an angle until the "
            "centre has been set. Run:\n\n"
            "    python examples/camera_gimbal_tracker.py --hardware "
            "--two-axis --set-zero\n\n"
            "then re-run with --zeroed. If you have already done that since "
            "the motors last\nlost power, just add --zeroed."
        )

    if not (args.set_zero or args.park):
        report_design_margins()

    if args.hardware:
        can_if = CANInterface(
            interface_type=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
        )
        print(f"Connecting to {args.interface}:{args.channel} @ {args.bitrate} bps")
    else:
        can_if = CANInterface(
            use_simulator=True, simulator_port=args.simulator_port
        )
        print(f"Connecting to simulator on port {args.simulator_port}")

    await can_if.connect()
    try:
        can_ids = None
        if args.can_ids:
            can_ids = {
                key.strip(): int(value)
                for key, value in (
                    pair.split("=") for pair in args.can_ids.split(",")
                )
            }

        if args.set_zero:
            await set_zero(
                can_if, include_roll=not args.two_axis, can_ids=can_ids
            )
            return

        if args.park:
            await park_at_home(
                can_if, include_roll=not args.two_axis, can_ids=can_ids
            )
            return

        print(f"Tracking for {args.duration:.0f}s at {CONTROL_RATE_HZ:.0f} Hz\n")
        result = await run_tracking(
            can_if, args.duration, quiet=args.quiet,
            include_roll=not args.two_axis, can_ids=can_ids,
        )

        print("\nPointing error by target angular rate")
        print("-" * 78)
        print(
            f"  {'band':26s} {'n':>5s} {'median':>9s} {'worst':>9s} "
            f"{'no predictor':>13s} {'gain':>7s}"
        )
        for band in result["bands"]:
            gain = (
                band["median_naive_deg"] / band["median_deg"]
                if band["median_deg"] > 0
                else float("inf")
            )
            print(
                f"  {band['band']:26s} {band['samples']:5d} "
                f"{band['median_deg']:8.3f}d {band['worst_deg']:8.3f}d "
                f"{band['median_naive_deg']:12.3f}d {gain:6.1f}x"
            )
        print()
        print(f"  sightings processed        {result['samples']}")
        print(f"  prediction horizon         {result['prediction_horizon_ms']:.0f} ms")
        print()
        loop = result["loop"]
        print(f"  control loop ticks         {loop['ticks']:.0f}")
        print(f"  frames sent                {loop['frames_sent']:.0f}")
        print(f"  send errors                {loop['send_errors']:.0f}")
        print(
            f"  loop lateness              mean {loop['mean_lateness_ms']:.2f} ms, "
            f"max {loop['max_lateness_ms']:.2f} ms"
        )
        if loop["late_ticks"]:
            print(
                f"  late ticks                 {loop['late_ticks']:.0f} "
                "(asyncio jitter; see the module docstring)"
            )
    finally:
        await can_if.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
