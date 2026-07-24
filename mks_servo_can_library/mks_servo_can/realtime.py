"""
Fixed-rate streaming control for applications that cannot wait for a move.

The `Axis` API is built around discrete moves: command a target, wait for the
motor to report completion. That is the right model for a plotter or a pick and
place, and the wrong one for anything tracking a moving reference, where the
target changes faster than any single move completes.

This module provides the other model. `ServoStream` runs a fixed-rate loop that
re-sends an absolute position target to every axis on every tick, without
waiting for acknowledgements. The MKS 0xF5 command accepts a new target while a
move is in progress, so streaming targets turns it from a point-to-point move
into a continuously re-planned servo input.

Why fire-and-forget
-------------------
Two reasons, and the second is the important one.

It halves the frames on the bus: three axes cost three frames per cycle instead
of six, which at 1 Mbit/s is the difference between roughly 1000 Hz and 500 Hz
of headroom at 40% bus load.

More importantly, it removes an ambiguity that cannot otherwise be resolved. The
MKS protocol reuses a single command byte for both the acknowledgement of a run
command and the asynchronous completion notification of the move it started.
When a new target supersedes a move in flight, the motor emits an abort frame
for the old move that is indistinguishable from the acknowledgement of the new
one. With responses disabled there are no such frames to confuse.

So `ServoStream` disables motor responses (0x8C) on entry and restores them on
exit. The consequence is that command failures are silent: nothing acknowledges
a dropped frame. That is an acceptable trade for a control loop that overwrites
its own command every few milliseconds, and a bad one for anything else.

Position feedback, if enabled, runs on a separate and much slower task. A
control loop does not need feedback at the loop rate; it needs it often enough
to detect loss of sync and to correct predictor drift.
"""
import asyncio
import logging
import time
from typing import Dict, Iterable, Optional

from . import constants as const
from . import motor_profile
from .can_interface import CANInterface
from .exceptions import ConfigurationError, MKSServoError, ParameterError
from .low_level_api import LowLevelAPI

logger = logging.getLogger(__name__)


class StreamAxis:
    """
    One axis participating in a streaming control loop.

    Holds the mechanical description needed to turn a user-unit target into an
    encoder count, plus the safety envelope the loop refuses to leave.

    Attributes:
        name: Identifier used when setting targets.
        can_id: The motor's CAN ID.
        gear_ratio: Reduction between motor and output. Above 1 means the output
            turns slower than the motor.
        accel_param: MKS acceleration parameter (0-255) sent with every command.
        invert: If True, positive user units command negative encoder counts.
        min_position / max_position: Soft limits in user units. Targets are
            clamped to this range; there is no way to command past them.
        max_rate: Ceiling on the commanded speed, in user units per second.
        microsteps: The motor's subdivision setting, needed to convert a rate
            into the correct speed parameter.
    """

    def __init__(
        self,
        name: str,
        can_id: int,
        gear_ratio: float = 1.0,
        accel_param: int = 250,
        invert: bool = False,
        min_position: float = -180.0,
        max_position: float = 180.0,
        max_rate: float = 720.0,
        microsteps: int = 16,
    ):
        """
        Describes one axis of a streaming loop.

        Args:
            name: Identifier used when setting targets.
            can_id: The motor's CAN ID (1-2047).
            gear_ratio: Reduction between motor and output shaft.
            accel_param: MKS acceleration parameter, 0-255. Higher is faster;
                see `motor_profile.accel_param_to_deg_per_s2`. 0 disables the
                ramp entirely, which will jerk the mechanism.
            invert: Reverse the sign of commanded motion.
            min_position: Lower soft limit in user units.
            max_position: Upper soft limit in user units.
            max_rate: Maximum commanded rate in user units per second.
            microsteps: The motor's subdivision setting.

        Raises:
            ParameterError: If the CAN ID, acceleration parameter, gear ratio,
                microsteps, or the soft-limit ordering is invalid.
        """
        if not (const.BROADCAST_ADDRESS < can_id <= 0x7FF):
            raise ParameterError(f"Invalid can_id {can_id}; must be 1-2047.")
        if not (0 <= accel_param <= const.MAX_ACCEL_PARAM):
            raise ParameterError(
                f"accel_param must be 0-{const.MAX_ACCEL_PARAM}, got {accel_param}."
            )
        if gear_ratio <= 0:
            raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")
        if microsteps <= 0:
            raise ParameterError(f"microsteps must be positive, got {microsteps}.")
        if min_position >= max_position:
            raise ParameterError(
                f"min_position ({min_position}) must be below max_position "
                f"({max_position})."
            )
        if max_rate <= 0:
            raise ParameterError(f"max_rate must be positive, got {max_rate}.")

        self.name = name
        self.can_id = can_id
        self.gear_ratio = gear_ratio
        self.accel_param = accel_param
        self.invert = invert
        self.min_position = min_position
        self.max_position = max_position
        self.max_rate = max_rate
        self.microsteps = microsteps

        # Live command state, written by set_target and read by the loop.
        self.target_position: float = 0.0
        self.feedforward_rate: float = 0.0
        # Last position read back, in user units. None until the first poll.
        self.measured_position: Optional[float] = None

    def clamp_position(self, position: float) -> float:
        """
        Clamps a target to the axis's soft limits.

        Args:
            position: Desired position in user units.

        Returns:
            The position, restricted to [min_position, max_position].
        """
        return min(max(position, self.min_position), self.max_position)

    def position_to_counts(self, position: float) -> int:
        """
        Converts a user-unit position to raw encoder counts.

        Args:
            position: Position in user units (degrees by convention).

        Returns:
            Signed encoder counts, honouring `invert` and `gear_ratio`.
        """
        signed = -position if self.invert else position
        return motor_profile.degrees_to_encoder_steps(signed, self.gear_ratio)

    def counts_to_position(self, counts: int) -> float:
        """
        Converts raw encoder counts to a user-unit position.

        Args:
            counts: Signed encoder counts.

        Returns:
            Position in user units, honouring `invert` and `gear_ratio`.
        """
        value = motor_profile.encoder_steps_to_degrees(counts, self.gear_ratio)
        return -value if self.invert else value

    def rate_to_speed_param(self, rate: float) -> int:
        """
        Converts a user-unit rate into an MKS speed parameter.

        Args:
            rate: Desired rate in user units per second. Sign is ignored;
                direction comes from the position target.

        Returns:
            A speed parameter in 0..`const.MAX_SPEED_PARAM`.
        """
        limited = min(abs(rate), self.max_rate)
        motor_rpm = (limited / 360.0) * self.gear_ratio * 60.0
        return motor_profile.rpm_to_speed_param(motor_rpm, self.microsteps)


class StreamStats:
    """
    Timing statistics for a streaming loop.

    Loop jitter is the thing that quietly ruins a control loop written in
    Python, and it is invisible unless measured. These counters make it visible.

    Attributes:
        ticks: Completed loop iterations.
        frames_sent: Command frames pushed onto the bus.
        send_errors: Iterations where a send raised.
        late_ticks: Iterations that started more than one period late.
        max_lateness: Worst observed start delay, in seconds.
        total_lateness: Accumulated start delay, for computing the mean.
    """

    def __init__(self):
        """Initialises all counters to zero."""
        self.ticks = 0
        self.frames_sent = 0
        self.send_errors = 0
        self.late_ticks = 0
        self.max_lateness = 0.0
        self.total_lateness = 0.0

    @property
    def mean_lateness(self) -> float:
        """
        Average delay between a tick's scheduled and actual start, in seconds.

        Returns:
            The mean, or 0.0 before any tick has run.
        """
        return self.total_lateness / self.ticks if self.ticks else 0.0

    def as_dict(self) -> Dict[str, float]:
        """
        Returns the statistics as a plain dictionary, for logging or display.

        Returns:
            A mapping of statistic name to value.
        """
        return {
            "ticks": self.ticks,
            "frames_sent": self.frames_sent,
            "send_errors": self.send_errors,
            "late_ticks": self.late_ticks,
            "max_lateness_ms": self.max_lateness * 1000.0,
            "mean_lateness_ms": self.mean_lateness * 1000.0,
        }

    def __repr__(self) -> str:
        """Returns a compact one-line summary."""
        return (
            f"StreamStats(ticks={self.ticks}, frames={self.frames_sent}, "
            f"late={self.late_ticks}, max_lateness="
            f"{self.max_lateness * 1000:.2f}ms)"
        )


class ServoStream:
    """
    Drives several axes from a fixed-rate loop, without waiting for replies.

    Use as an async context manager. Entering disables motor responses and
    starts the loop; leaving stops the axes and restores responses, including
    when the body raises.

        axes = [StreamAxis("pan", 1), StreamAxis("tilt", 2)]
        async with ServoStream(can_if, axes, rate_hz=200) as stream:
            while tracking:
                stream.set_target("pan", azimuth, feedforward_rate=az_rate)
                stream.set_target("tilt", elevation, feedforward_rate=el_rate)
                await asyncio.sleep(0.02)

    The loop never blocks on the caller: `set_target` only updates state, and
    whatever value is current at each tick is what gets sent. A producer slower
    than the loop simply results in the last target being re-sent, which is
    exactly the desired behaviour for a servo.

    Attributes:
        stats: Live `StreamStats` for the running loop.
    """

    def __init__(
        self,
        can_interface: CANInterface,
        axes: Iterable[StreamAxis],
        rate_hz: float = 200.0,
        position_gain: float = 0.0,
        feedback_rate_hz: float = 0.0,
        watchdog_timeout: Optional[float] = 0.5,
        manage_motor_responses: bool = True,
    ):
        """
        Configures a streaming controller.

        Args:
            can_interface: A connected `CANInterface`.
            axes: The `StreamAxis` objects to drive. Names must be unique.
            rate_hz: Loop frequency. Above roughly 500 Hz the bus, not the host,
                becomes the limit; see the module docstring.
            position_gain: Proportional gain folded into the commanded speed, in
                (user units/s) per user unit of error. Only has an effect when
                `feedback_rate_hz` is non-zero, since it needs measured
                position. Zero means pure feed-forward, which is usually right:
                the motor already closes its own position loop.
            feedback_rate_hz: Frequency of the separate position-polling task.
                Zero disables feedback entirely. This does not need to match
                `rate_hz`; 20-50 Hz is normally plenty.
            watchdog_timeout: If no target is set within this many seconds, the
                loop commands zero rate and holds position. None disables it.
                A control loop whose producer has died should stop, not coast.
            manage_motor_responses: Whether to disable motor responses on entry
                and restore them on exit. Set False if you have already
                configured the motors yourself.

        Raises:
            ConfigurationError: If no axes are given or names collide.
            ParameterError: If `rate_hz` is not positive.
        """
        axis_list = list(axes)
        if not axis_list:
            raise ConfigurationError("ServoStream requires at least one axis.")
        names = [a.name for a in axis_list]
        if len(set(names)) != len(names):
            raise ConfigurationError(f"Duplicate axis names: {names}")
        if rate_hz <= 0:
            raise ParameterError(f"rate_hz must be positive, got {rate_hz}.")

        self._can_if = can_interface
        self._api = LowLevelAPI(can_interface)
        self.axes: Dict[str, StreamAxis] = {a.name: a for a in axis_list}
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.position_gain = position_gain
        self.feedback_rate_hz = feedback_rate_hz
        self.watchdog_timeout = watchdog_timeout
        self.manage_motor_responses = manage_motor_responses

        self.stats = StreamStats()
        self._running = False
        self._loop_task: Optional[asyncio.Task] = None
        self._feedback_task: Optional[asyncio.Task] = None
        self._last_target_time: float = 0.0
        self._watchdog_tripped = False

    def set_target(
        self, axis_name: str, position: float, feedforward_rate: float = 0.0
    ) -> None:
        """
        Updates an axis's target. Cheap, synchronous, and safe to call at any rate.

        The value is latched, not queued: the loop sends whatever is current when
        it next ticks. Calling faster than the loop rate simply discards the
        intermediate values, which is what a servo wants.

        `feedforward_rate` is the important argument. Supplying the target's
        predicted velocity lets the motor's own trapezoidal planner move at
        roughly the right speed instead of decelerating into every target, which
        is the difference between tracking and stuttering.

        Args:
            axis_name: Name of a configured axis.
            position: Desired position in user units. Clamped to soft limits.
            feedforward_rate: Predicted rate of change of the target, in user
                units per second.

        Raises:
            KeyError: If `axis_name` is not a configured axis.
        """
        axis = self.axes[axis_name]
        axis.target_position = axis.clamp_position(position)
        axis.feedforward_rate = feedforward_rate
        self._last_target_time = time.monotonic()
        self._watchdog_tripped = False

    def set_targets(self, targets: Dict[str, float], rates: Optional[Dict[str, float]] = None) -> None:
        """
        Updates several axes at once.

        Args:
            targets: Mapping of axis name to position in user units.
            rates: Optional mapping of axis name to feed-forward rate.

        Raises:
            KeyError: If any name is not a configured axis.
        """
        for name, position in targets.items():
            self.set_target(name, position, (rates or {}).get(name, 0.0))

    def get_measured_positions(self) -> Dict[str, Optional[float]]:
        """
        Returns the most recent polled position of each axis.

        Values are None until the feedback task has run at least once, and stay
        None permanently if `feedback_rate_hz` is zero.

        Returns:
            Mapping of axis name to position in user units, or None.
        """
        return {name: axis.measured_position for name, axis in self.axes.items()}

    async def start(self) -> None:
        """
        Disables motor responses and starts the control loop.

        Raises:
            MKSServoError: If the interface is not connected, or if disabling
                motor responses fails.
        """
        if self._running:
            logger.warning("ServoStream.start() called while already running.")
            return
        if not self._can_if.is_connected:
            raise MKSServoError("ServoStream requires a connected CANInterface.")

        if self.manage_motor_responses:
            # Disabling responses is only undone by stop(), which returns early
            # unless the stream is running. A start that fails partway would
            # therefore leave the axes it had already reached mute for the rest
            # of the session, so it undoes its own work before propagating.
            silenced = []
            try:
                for axis in self.axes.values():
                    await self._api.set_slave_respond_active(
                        axis.can_id, respond_enabled=False, active_enabled=False
                    )
                    silenced.append(axis)
            except BaseException:
                for axis in reversed(silenced):
                    try:
                        await self._api.set_slave_respond_active(
                            axis.can_id, respond_enabled=True, active_enabled=False
                        )
                    except MKSServoError as exc:
                        logger.error(
                            "ServoStream: axis '%s' (CAN ID %d) is left with its "
                            "responses disabled after a failed start: %s",
                            axis.name,
                            axis.can_id,
                            exc,
                        )
                raise
            logger.info(
                "ServoStream: motor responses disabled on %d axes; commands are "
                "now fire-and-forget and failures will be silent.",
                len(self.axes),
            )

        self._running = True
        self._last_target_time = time.monotonic()
        self._loop_task = asyncio.create_task(self._run_loop(), name="servo_stream")
        if self.feedback_rate_hz > 0:
            self._feedback_task = asyncio.create_task(
                self._run_feedback(), name="servo_stream_feedback"
            )

    async def stop(self) -> None:
        """
        Stops the loop, commands zero rate, and restores motor responses.

        Safe to call more than once, and safe to call after an error.
        """
        if not self._running:
            return
        self._running = False

        for task in (self._loop_task, self._feedback_task):
            if task is not None and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._loop_task = None
        self._feedback_task = None

        # Halt in place. Re-commanding the current target at speed 0 does not
        # work: the firmware substitutes a minimum speed and creeps on to the
        # target. The protocol's stop encoding is 0xF5 with speed and target
        # both zero, which is a distinct command rather than a move to zero.
        for axis in self.axes.values():
            try:
                await self._api.stop_position_mode_absolute_axis_no_wait(
                    axis.can_id, axis.accel_param
                )
            except MKSServoError as exc:
                logger.warning(
                    "ServoStream: could not stop axis '%s': %s", axis.name, exc
                )

        if self.manage_motor_responses:
            for axis in self.axes.values():
                try:
                    await self._api.set_slave_respond_active(
                        axis.can_id, respond_enabled=True, active_enabled=False
                    )
                except MKSServoError as exc:
                    logger.warning(
                        "ServoStream: could not restore responses on axis '%s': %s",
                        axis.name,
                        exc,
                    )
        logger.info("ServoStream stopped. %s", self.stats)

    async def __aenter__(self) -> "ServoStream":
        """Starts the loop. See `start`."""
        await self.start()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        """Stops the loop, including on error. See `stop`."""
        await self.stop()

    async def _run_loop(self) -> None:
        """
        The control loop.

        Ticks are scheduled against an absolute clock rather than by sleeping for
        a fixed period, so a slow iteration does not push every subsequent one
        later. Lateness is recorded rather than hidden.
        """
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        logger.info(
            "ServoStream: loop started at %.1f Hz across %d axes.",
            self.rate_hz,
            len(self.axes),
        )
        try:
            while self._running:
                now = loop.time()
                lateness = max(0.0, now - next_tick)
                if lateness > self.period:
                    self.stats.late_ticks += 1
                self.stats.max_lateness = max(self.stats.max_lateness, lateness)
                self.stats.total_lateness += lateness

                await self._tick()
                self.stats.ticks += 1

                next_tick += self.period
                # If we have fallen far behind, resynchronise rather than trying
                # to catch up with a burst of back-to-back frames.
                if next_tick < now:
                    next_tick = now + self.period
                await asyncio.sleep(max(0.0, next_tick - loop.time()))
        except asyncio.CancelledError:
            raise
        except Exception:  # pylint: disable=broad-except
            logger.exception("ServoStream: control loop died")
            self._running = False
            raise

    async def _tick(self) -> None:
        """
        Sends one command frame per axis.

        A send failure is counted and skipped rather than raised: one dropped
        frame in a loop that re-sends every few milliseconds is not worth tearing
        the loop down for.
        """
        watchdog_expired = (
            self.watchdog_timeout is not None
            and time.monotonic() - self._last_target_time > self.watchdog_timeout
        )
        if watchdog_expired and not self._watchdog_tripped:
            self._watchdog_tripped = True
            logger.warning(
                "ServoStream: no target set for %.2fs; holding position.",
                self.watchdog_timeout,
            )

        for axis in self.axes.values():
            try:
                if watchdog_expired:
                    # Halt in place rather than continuing toward a target set
                    # by a producer that is no longer running.
                    await self._api.stop_position_mode_absolute_axis_no_wait(
                        axis.can_id, axis.accel_param
                    )
                else:
                    rate = abs(axis.feedforward_rate)
                    if self.position_gain and axis.measured_position is not None:
                        error = axis.target_position - axis.measured_position
                        rate += self.position_gain * abs(error)
                    await self._api.run_position_mode_absolute_axis_no_wait(
                        axis.can_id,
                        axis.rate_to_speed_param(rate),
                        axis.accel_param,
                        axis.position_to_counts(axis.target_position),
                    )
                self.stats.frames_sent += 1
            except MKSServoError as exc:
                self.stats.send_errors += 1
                logger.debug(
                    "ServoStream: send failed on axis '%s': %s", axis.name, exc
                )

    async def _run_feedback(self) -> None:
        """
        Polls position on a slow, separate cadence.

        Deliberately decoupled from the control loop: feedback exists to detect
        loss of sync and correct drift, not to close the inner loop, which the
        motor already does for itself.

        Reads are not bracketed by CanRSP toggling. By manual sections 6.4-6.8
        only the run commands are suppressible, so 0x31 answers whether or not
        responses are disabled - one round trip per axis per poll instead of
        three. That reading has not yet been confirmed against hardware (see
        `docs/development/roadmap.md`, item 2), so persistent read failures are
        reported at warning level naming that possibility, rather than being
        lost among debug records.
        """
        period = 1.0 / self.feedback_rate_hz
        loop = asyncio.get_running_loop()
        next_poll = loop.time()
        consecutive_failures = 0
        warned = False
        try:
            while self._running:
                for axis in self.axes.values():
                    try:
                        counts = await self._api.read_encoder_value_addition(
                            axis.can_id
                        )
                        axis.measured_position = axis.counts_to_position(counts)
                        consecutive_failures = 0
                    except MKSServoError as exc:
                        logger.debug(
                            "ServoStream: feedback read failed on '%s': %s",
                            axis.name,
                            exc,
                        )
                        consecutive_failures += 1
                        if (
                            not warned
                            and self.manage_motor_responses
                            and consecutive_failures >= 3 * len(self.axes)
                        ):
                            warned = True
                            logger.warning(
                                "ServoStream: %d consecutive feedback reads have "
                                "failed while motor responses are disabled. If "
                                "this motor suppresses replies to 0x31 as well as "
                                "to the run commands, feedback cannot work in "
                                "this mode; pass manage_motor_responses=False and "
                                "manage CanRSP yourself.",
                                consecutive_failures,
                            )
                next_poll += period
                if next_poll < loop.time():
                    next_poll = loop.time() + period
                await asyncio.sleep(max(0.0, next_poll - loop.time()))
        except asyncio.CancelledError:
            raise


class AlphaBetaTracker:
    """
    A constant-velocity predictor for a noisy, delayed measurement stream.

    Any closed loop driven by a camera is dominated by the delay between the
    photons landing and the command going out. Pointing error from pure
    transport delay is `rate x latency`: a target crossing at 172 deg/s with a
    50 ms pipeline is 8.6 degrees behind by the time the motor moves, which is
    more than a telephoto field of view. Extrapolating the target forward by the
    measured latency removes almost all of that. The residual is only what the
    constant-velocity assumption misses, roughly `0.5 x accel x latency^2`,
    which for the same case is about 0.3 degrees.

    Tuning: `alpha` is how much of each measurement's error is believed (0.1-0.5
    is typical), `beta` how much of it is attributed to a velocity error
    (0.005-0.1). Higher values track harder and admit more noise. Stability
    requires 0 < alpha < 1 and 0 < beta <= 2 and beta < 4 - 2*alpha.

    Attributes:
        position: Current filtered position estimate.
        velocity: Current filtered velocity estimate, per second.
        initialised: False until the first `update`.
    """

    def __init__(self, alpha: float = 0.3, beta: float = 0.05):
        """
        Creates a tracker.

        Args:
            alpha: Position correction gain, 0 < alpha < 1.
            beta: Velocity correction gain, 0 < beta <= 2.

        Raises:
            ParameterError: If the gains are outside their stable region.
        """
        if not 0.0 < alpha < 1.0:
            raise ParameterError(f"alpha must be in (0, 1), got {alpha}.")
        if not 0.0 < beta <= 2.0:
            raise ParameterError(f"beta must be in (0, 2], got {beta}.")
        if beta >= 4.0 - 2.0 * alpha:
            raise ParameterError(
                f"alpha={alpha}, beta={beta} is outside the stable region "
                "(requires beta < 4 - 2*alpha)."
            )
        self.alpha = alpha
        self.beta = beta
        self.position = 0.0
        self.velocity = 0.0
        self.initialised = False
        self._last_time: Optional[float] = None

    def reset(self) -> None:
        """Discards the current estimate, as after losing the target."""
        self.position = 0.0
        self.velocity = 0.0
        self.initialised = False
        self._last_time = None

    def update(self, measurement: float, timestamp: float) -> float:
        """
        Folds in a new measurement.

        Args:
            measurement: Observed position, in whatever unit you are tracking.
            timestamp: When the measurement was *captured*, not when it was
                processed. This matters: timestamping at detection time folds a
                variable delay into the measurement, and no filter can undo it.
                Use the camera's exposure timestamp if you have one.

        Returns:
            The updated position estimate.
        """
        if not self.initialised or self._last_time is None:
            self.position = measurement
            self.velocity = 0.0
            self.initialised = True
            self._last_time = timestamp
            return self.position

        dt = timestamp - self._last_time
        if dt <= 0:
            # Out-of-order or duplicate sample; fold it in without advancing.
            residual = measurement - self.position
            self.position += self.alpha * residual
            return self.position

        predicted_position = self.position + self.velocity * dt
        residual = measurement - predicted_position

        self.position = predicted_position + self.alpha * residual
        self.velocity = self.velocity + (self.beta / dt) * residual
        self._last_time = timestamp
        return self.position

    def predict(self, horizon: float) -> float:
        """
        Extrapolates the estimate forward.

        Args:
            horizon: How far ahead to predict, in seconds. This should be your
                *measured* total pipeline latency. Guessing it wrong by 20 ms
                costs 3.4 degrees at 172 deg/s, so measure it rather than
                assuming.

        Returns:
            The predicted position at `now + horizon`.
        """
        return self.position + self.velocity * horizon

    def predict_at(self, timestamp: float) -> float:
        """
        Extrapolates the estimate to an absolute time.

        Args:
            timestamp: The absolute time to predict for, on the same clock as
                the timestamps passed to `update`.

        Returns:
            The predicted position at that time.
        """
        if self._last_time is None:
            return self.position
        return self.predict(timestamp - self._last_time)


class AlphaBetaGammaTracker(AlphaBetaTracker):
    """
    A constant-acceleration predictor, for targets that manoeuvre.

    `AlphaBetaTracker` assumes constant velocity, so on an accelerating target
    its velocity estimate always lags. The lag is roughly
    `accel x dt / beta`, and it feeds straight into the prediction: at 60 Hz
    with beta=0.08, a target accelerating at 335 deg/s^2 leaves the velocity
    estimate about 70 deg/s behind, which over a 50 ms horizon is 3.5 degrees of
    pointing error. That is much larger than the residual the predictor was
    supposed to be left with, so the filter becomes the dominant error source
    rather than the motor.

    Note that this is not exotic: a target flying a *straight line* past the
    gimbal has large angular acceleration near the crossing point, because the
    geometry accelerates even though the target does not. A drone passing at
    30 m/s and 10 m peaks at about 335 deg/s^2 of angular acceleration while
    flying perfectly straight.

    Estimating acceleration as a third state removes that lag term instead of
    merely shrinking it. The cost is more noise sensitivity, since acceleration
    is a second difference of a noisy measurement.

    Attributes:
        acceleration: Current filtered acceleration estimate, per second squared.
    """

    def __init__(self, alpha: float = 0.5, beta: float = 0.4, gamma: float = 0.1):
        """
        Creates a constant-acceleration tracker.

        Args:
            alpha: Position correction gain, 0 < alpha < 1.
            beta: Velocity correction gain, 0 < beta <= 2.
            gamma: Acceleration correction gain, 0 < gamma <= 1. Start small:
                this term amplifies measurement noise the most.

        Raises:
            ParameterError: If the gains are outside their stable region.
        """
        super().__init__(alpha=alpha, beta=beta)
        if not 0.0 < gamma <= 1.0:
            raise ParameterError(f"gamma must be in (0, 1], got {gamma}.")
        self.gamma = gamma
        self.acceleration = 0.0

    def reset(self) -> None:
        """Discards the current estimate, including the acceleration state."""
        super().reset()
        self.acceleration = 0.0

    def update(self, measurement: float, timestamp: float) -> float:
        """
        Folds in a new measurement, updating position, velocity and acceleration.

        Args:
            measurement: Observed position.
            timestamp: When the measurement was captured. See
                `AlphaBetaTracker.update` on why capture time, not process time.

        Returns:
            The updated position estimate.
        """
        if not self.initialised or self._last_time is None:
            self.position = measurement
            self.velocity = 0.0
            self.acceleration = 0.0
            self.initialised = True
            self._last_time = timestamp
            return self.position

        dt = timestamp - self._last_time
        if dt <= 0:
            residual = measurement - self.position
            self.position += self.alpha * residual
            return self.position

        predicted_position = (
            self.position + self.velocity * dt + 0.5 * self.acceleration * dt * dt
        )
        predicted_velocity = self.velocity + self.acceleration * dt
        residual = measurement - predicted_position

        self.position = predicted_position + self.alpha * residual
        self.velocity = predicted_velocity + (self.beta / dt) * residual
        self.acceleration = self.acceleration + (
            2.0 * self.gamma / (dt * dt)
        ) * residual
        self._last_time = timestamp
        return self.position

    def predict(self, horizon: float) -> float:
        """
        Extrapolates forward using both velocity and acceleration.

        Args:
            horizon: How far ahead to predict, in seconds. Use your measured
                pipeline latency.

        Returns:
            The predicted position at `now + horizon`.
        """
        return (
            self.position
            + self.velocity * horizon
            + 0.5 * self.acceleration * horizon * horizon
        )

    def predict_velocity(self, horizon: float) -> float:
        """
        Extrapolates the velocity forward.

        Useful as the feed-forward rate for a streamed move: the rate the target
        will have when the command lands, rather than the rate it had when it
        was last seen.

        Args:
            horizon: How far ahead to predict, in seconds.

        Returns:
            The predicted velocity at `now + horizon`.
        """
        return self.velocity + self.acceleration * horizon
