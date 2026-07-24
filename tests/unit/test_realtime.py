"""Tests for the fixed-rate streaming controller and the alpha-beta tracker."""
import asyncio
import math
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from mks_servo_can import constants as const
from mks_servo_can import motor_profile
from mks_servo_can.can_interface import CANInterface
from mks_servo_can.exceptions import ConfigurationError, MKSServoError, ParameterError
from mks_servo_can.low_level_api import LowLevelAPI
from mks_servo_can.realtime import (
    AlphaBetaGammaTracker,
    AlphaBetaTracker,
    ServoStream,
    StreamAxis,
    StreamStats,
)


@pytest.fixture
def mock_api():
    """A LowLevelAPI mock that records every streamed command."""
    api = AsyncMock(spec=LowLevelAPI)
    api.run_position_mode_absolute_axis_no_wait = AsyncMock()
    api.stop_position_mode_absolute_axis_no_wait = AsyncMock()
    api.set_slave_respond_active = AsyncMock()
    api.read_encoder_value_addition = AsyncMock(return_value=0)
    return api


@pytest.fixture
def mock_can_if():
    """A connected CANInterface mock."""
    can_if = MagicMock(spec=CANInterface)
    can_if.is_connected = True
    return can_if


@pytest.fixture
def stream_factory(mock_can_if, mock_api):
    """Builds ServoStream instances wired to the mock API."""

    def build(axes=None, **kwargs):
        axes = axes or [StreamAxis("pan", 1), StreamAxis("tilt", 2)]
        with patch("mks_servo_can.realtime.LowLevelAPI", return_value=mock_api):
            return ServoStream(mock_can_if, axes, **kwargs)

    return build


class TestStreamAxis:
    """Unit conversions and the safety envelope."""

    def test_position_to_counts_direct_drive(self):
        axis = StreamAxis("pan", 1)
        assert axis.position_to_counts(360.0) == const.ENCODER_PULSES_PER_REVOLUTION
        assert axis.position_to_counts(0.0) == 0

    def test_gear_ratio_multiplies_counts(self):
        geared = StreamAxis("pan", 1, gear_ratio=5.0)
        assert geared.position_to_counts(360.0) == (
            const.ENCODER_PULSES_PER_REVOLUTION * 5
        )

    def test_invert_reverses_direction(self):
        axis = StreamAxis("pan", 1, invert=True)
        assert axis.position_to_counts(90.0) == -StreamAxis(
            "pan", 1
        ).position_to_counts(90.0)

    @pytest.mark.parametrize("position", [-720.0, -90.0, 0.0, 45.5, 720.0])
    def test_counts_round_trip(self, position):
        axis = StreamAxis("pan", 1, min_position=-1000, max_position=1000)
        back = axis.counts_to_position(axis.position_to_counts(position))
        assert back == pytest.approx(position, abs=motor_profile.encoder_resolution_degrees())

    def test_soft_limits_clamp(self):
        axis = StreamAxis("tilt", 2, min_position=-30.0, max_position=45.0)
        assert axis.clamp_position(1000.0) == 45.0
        assert axis.clamp_position(-1000.0) == -30.0
        assert axis.clamp_position(10.0) == 10.0

    def test_rate_is_capped_by_max_rate(self):
        axis = StreamAxis("pan", 1, max_rate=180.0)
        capped = axis.rate_to_speed_param(100000.0)
        expected = axis.rate_to_speed_param(180.0)
        assert capped == expected

    def test_rate_conversion_matches_motor_profile(self):
        """360 deg/s at 1:1 is 60 RPM, which at 16 microsteps is param 60."""
        axis = StreamAxis("pan", 1, max_rate=3600.0)
        assert axis.rate_to_speed_param(360.0) == 60

    def test_rate_sign_is_ignored(self):
        axis = StreamAxis("pan", 1)
        assert axis.rate_to_speed_param(-90.0) == axis.rate_to_speed_param(90.0)

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"can_id": 0},
            {"can_id": 5000},
            {"accel_param": 256},
            {"accel_param": -1},
            {"gear_ratio": 0},
            {"microsteps": 0},
            {"max_rate": 0},
            {"min_position": 10.0, "max_position": 10.0},
            {"min_position": 50.0, "max_position": -50.0},
        ],
    )
    def test_invalid_configuration_rejected(self, kwargs):
        base = {"name": "x", "can_id": 1}
        base.update(kwargs)
        with pytest.raises(ParameterError):
            StreamAxis(**base)


class TestServoStreamConfiguration:
    """Construction-time validation."""

    def test_requires_at_least_one_axis(self, mock_can_if):
        with pytest.raises(ConfigurationError):
            ServoStream(mock_can_if, [])

    def test_rejects_duplicate_axis_names(self, mock_can_if):
        with pytest.raises(ConfigurationError):
            ServoStream(mock_can_if, [StreamAxis("a", 1), StreamAxis("a", 2)])

    def test_rejects_non_positive_rate(self, mock_can_if):
        with pytest.raises(ParameterError):
            ServoStream(mock_can_if, [StreamAxis("a", 1)], rate_hz=0)

    @pytest.mark.asyncio
    async def test_refuses_to_start_when_disconnected(self, mock_can_if, mock_api):
        mock_can_if.is_connected = False
        with patch("mks_servo_can.realtime.LowLevelAPI", return_value=mock_api):
            stream = ServoStream(mock_can_if, [StreamAxis("a", 1)])
        with pytest.raises(MKSServoError):
            await stream.start()


class TestServoStreamLoop:
    """Behaviour of the running loop."""

    @pytest.mark.asyncio
    async def test_disables_responses_on_entry_and_restores_on_exit(
        self, stream_factory, mock_api
    ):
        """
        Fire-and-forget is only safe with responses off; leaving them off after
        the loop ends would break every subsequent request/response call.
        """
        stream = stream_factory(rate_hz=100)
        async with stream:
            await asyncio.sleep(0.05)
        calls = mock_api.set_slave_respond_active.await_args_list
        # Two axes disabled at entry...
        assert all(c.kwargs["respond_enabled"] is False for c in calls[:2])
        # ...and re-enabled at exit.
        assert all(c.kwargs["respond_enabled"] is True for c in calls[-2:])

    @pytest.mark.asyncio
    async def test_sends_one_frame_per_axis_per_tick(
        self, stream_factory, mock_api
    ):
        stream = stream_factory(rate_hz=200, watchdog_timeout=None)
        async with stream:
            stream.set_target("pan", 10.0, feedforward_rate=90.0)
            await asyncio.sleep(0.15)
        assert stream.stats.ticks > 5
        assert mock_api.run_position_mode_absolute_axis_no_wait.await_count >= (
            stream.stats.ticks * 2
        ) - 2

    @pytest.mark.asyncio
    async def test_streams_the_latest_target_not_a_queue(
        self, stream_factory, mock_api
    ):
        """
        Setting targets faster than the loop must discard intermediates. A queue
        would make the controller lag further behind the harder it was driven.
        """
        stream = stream_factory(
            axes=[StreamAxis("pan", 1, max_position=1000)],
            rate_hz=50,
            watchdog_timeout=None,
        )
        async with stream:
            for value in range(100):
                stream.set_target("pan", float(value))
            await asyncio.sleep(0.08)

        counts = [
            c.args[3]
            for c in mock_api.run_position_mode_absolute_axis_no_wait.await_args_list
        ]
        final = StreamAxis("pan", 1, max_position=1000).position_to_counts(99.0)
        assert counts[-1] == final, "loop did not send the most recent target"

    @pytest.mark.asyncio
    async def test_target_is_clamped_to_soft_limits(
        self, stream_factory, mock_api
    ):
        axis = StreamAxis("tilt", 2, min_position=-20.0, max_position=20.0)
        stream = stream_factory(axes=[axis], rate_hz=100, watchdog_timeout=None)
        async with stream:
            stream.set_target("tilt", 500.0)
            await asyncio.sleep(0.05)
        counts = [
            c.args[3]
            for c in mock_api.run_position_mode_absolute_axis_no_wait.await_args_list
        ]
        assert max(counts) <= axis.position_to_counts(20.0)

    @pytest.mark.asyncio
    async def test_feedforward_rate_sets_the_speed_parameter(
        self, stream_factory, mock_api
    ):
        stream = stream_factory(
            axes=[StreamAxis("pan", 1, max_rate=3600.0)],
            rate_hz=100,
            watchdog_timeout=None,
        )
        async with stream:
            stream.set_target("pan", 10.0, feedforward_rate=360.0)
            await asyncio.sleep(0.05)
        speeds = [
            c.args[1]
            for c in mock_api.run_position_mode_absolute_axis_no_wait.await_args_list
        ]
        assert 60 in speeds, f"expected speed param 60 for 360 deg/s, got {set(speeds)}"

    @pytest.mark.asyncio
    async def test_watchdog_halts_in_place_when_producer_stops(
        self, stream_factory, mock_api
    ):
        """
        A loop whose producer has died must halt, not coast to its last target.

        The obvious implementation - re-send the last target with speed 0 - does
        not work: the firmware substitutes a minimum speed and creeps all the
        way there anyway. Caught by an integration test where the axis reached
        60.01 deg after the watchdog was supposed to have stopped it. The
        protocol's stop encoding (0xF5 with speed and target both zero) is a
        distinct command and is what must be sent.
        """
        stream = stream_factory(
            axes=[StreamAxis("pan", 1)], rate_hz=100, watchdog_timeout=0.05
        )
        async with stream:
            stream.set_target("pan", 10.0, feedforward_rate=180.0)
            await asyncio.sleep(0.2)

        assert mock_api.stop_position_mode_absolute_axis_no_wait.await_count > 0, (
            "watchdog did not issue the protocol stop command"
        )
        # Once tripped, it must stop issuing move commands entirely.
        move_calls = mock_api.run_position_mode_absolute_axis_no_wait.await_count
        await asyncio.sleep(0)
        assert (
            mock_api.run_position_mode_absolute_axis_no_wait.await_count == move_calls
        )

    @pytest.mark.asyncio
    async def test_send_failures_are_counted_not_fatal(
        self, stream_factory, mock_api
    ):
        """One dropped frame must not tear down a loop that resends constantly."""
        mock_api.run_position_mode_absolute_axis_no_wait.side_effect = MKSServoError(
            "bus busy"
        )
        stream = stream_factory(
            axes=[StreamAxis("pan", 1)], rate_hz=100, watchdog_timeout=None
        )
        async with stream:
            stream.set_target("pan", 10.0)
            await asyncio.sleep(0.08)
            assert stream._running, "loop died on a send error"
        assert stream.stats.send_errors > 0

    @pytest.mark.asyncio
    async def test_stop_is_idempotent(self, stream_factory):
        stream = stream_factory(rate_hz=100)
        await stream.start()
        await stream.stop()
        await stream.stop()

    @pytest.mark.asyncio
    async def test_context_manager_stops_on_exception(
        self, stream_factory, mock_api
    ):
        """A crash in the body must still restore motor responses."""
        stream = stream_factory(rate_hz=100)
        with pytest.raises(ValueError):
            async with stream:
                raise ValueError("boom")
        assert not stream._running
        assert mock_api.set_slave_respond_active.await_args_list[-1].kwargs[
            "respond_enabled"
        ]

    @pytest.mark.asyncio
    async def test_records_loop_timing(self, stream_factory):
        stream = stream_factory(rate_hz=200, watchdog_timeout=None)
        async with stream:
            stream.set_target("pan", 1.0)
            await asyncio.sleep(0.15)
        stats = stream.stats.as_dict()
        assert stats["ticks"] > 5
        assert "max_lateness_ms" in stats and stats["max_lateness_ms"] >= 0

    @pytest.mark.asyncio
    async def test_set_targets_updates_several_axes(self, stream_factory):
        stream = stream_factory(rate_hz=100, watchdog_timeout=None)
        stream.set_targets({"pan": 5.0, "tilt": -5.0}, {"pan": 10.0})
        assert stream.axes["pan"].target_position == 5.0
        assert stream.axes["tilt"].target_position == -5.0
        assert stream.axes["pan"].feedforward_rate == 10.0
        assert stream.axes["tilt"].feedforward_rate == 0.0

    def test_unknown_axis_name_raises(self, stream_factory):
        stream = stream_factory()
        with pytest.raises(KeyError):
            stream.set_target("nonexistent", 1.0)


class TestAlphaBetaTracker:
    """The predictor that removes most of the pipeline-delay pointing error."""

    @pytest.mark.parametrize(
        "alpha,beta", [(0.0, 0.1), (1.0, 0.1), (0.5, 0.0), (0.5, 2.5), (0.9, 2.2)]
    )
    def test_unstable_gains_rejected(self, alpha, beta):
        with pytest.raises(ParameterError):
            AlphaBetaTracker(alpha=alpha, beta=beta)

    def test_first_update_seeds_the_estimate(self):
        tracker = AlphaBetaTracker()
        assert tracker.update(42.0, 0.0) == 42.0
        assert tracker.velocity == 0.0
        assert tracker.initialised

    def test_converges_on_a_constant_velocity_target(self):
        """The whole point: learn the rate so it can extrapolate."""
        tracker = AlphaBetaTracker(alpha=0.5, beta=0.2)
        rate = 100.0
        for i in range(200):
            t = i * 0.01
            tracker.update(rate * t, t)
        assert tracker.velocity == pytest.approx(rate, rel=0.05)

    def test_prediction_beats_no_prediction_under_latency(self):
        """
        Reproduces the case the gimbal exists for: a target crossing at
        172 deg/s seen through a 50 ms pipeline. Without extrapolation the
        pointing error is rate x latency; with it, only the unmodelled
        acceleration remains.
        """
        rate, latency, dt = 172.0, 0.05, 0.01
        tracker = AlphaBetaTracker(alpha=0.5, beta=0.2)
        for i in range(300):
            t = i * dt
            tracker.update(rate * t, t)

        now = 300 * dt
        truth = rate * (now + latency)
        naive_error = abs(truth - tracker.position)
        predicted_error = abs(truth - tracker.predict(latency + dt))

        assert naive_error > 8.0, "expected the un-predicted error to be large"
        assert predicted_error < 0.5
        assert predicted_error < naive_error / 10

    def test_tracks_through_noise(self):
        """Deterministic pseudo-noise; the estimate must stay near the truth."""
        tracker = AlphaBetaTracker(alpha=0.3, beta=0.05)
        rate, dt = 50.0, 0.02
        for i in range(400):
            t = i * dt
            noise = 0.4 * math.sin(i * 2.399963)  # irrational stride, zero-mean
            tracker.update(rate * t + noise, t)
        truth = rate * 399 * dt
        assert abs(tracker.position - truth) < 2.0
        assert tracker.velocity == pytest.approx(rate, rel=0.15)

    def test_reset_clears_state(self):
        tracker = AlphaBetaTracker()
        tracker.update(10.0, 0.0)
        tracker.update(20.0, 0.1)
        tracker.reset()
        assert not tracker.initialised
        assert tracker.position == 0.0
        assert tracker.velocity == 0.0

    def test_out_of_order_sample_does_not_explode(self):
        """
        A negative dt would divide by a negative number and invert the
        correction. Real pipelines do deliver frames out of order.
        """
        tracker = AlphaBetaTracker()
        tracker.update(0.0, 1.0)
        tracker.update(10.0, 1.1)
        before = tracker.velocity
        tracker.update(5.0, 1.05)  # earlier than the previous sample
        assert math.isfinite(tracker.position)
        assert math.isfinite(tracker.velocity)
        assert tracker.velocity == before

    def test_zero_dt_does_not_divide_by_zero(self):
        tracker = AlphaBetaTracker()
        tracker.update(0.0, 5.0)
        tracker.update(1.0, 5.0)
        assert math.isfinite(tracker.position)

    def test_predict_at_absolute_time(self):
        tracker = AlphaBetaTracker(alpha=0.5, beta=0.2)
        for i in range(100):
            tracker.update(10.0 * i * 0.01, i * 0.01)
        last_t = 99 * 0.01
        assert tracker.predict_at(last_t + 0.1) == pytest.approx(
            tracker.predict(0.1)
        )

    def test_predict_before_any_update_is_safe(self):
        assert AlphaBetaTracker().predict_at(123.0) == 0.0


class TestStreamStats:
    """The counters that make loop jitter visible."""

    def test_mean_lateness_before_any_tick(self):
        assert StreamStats().mean_lateness == 0.0

    def test_mean_lateness(self):
        stats = StreamStats()
        stats.ticks = 4
        stats.total_lateness = 0.008
        assert stats.mean_lateness == pytest.approx(0.002)

    def test_as_dict_reports_milliseconds(self):
        stats = StreamStats()
        stats.ticks = 2
        stats.total_lateness = 0.004
        stats.max_lateness = 0.003
        d = stats.as_dict()
        assert d["mean_lateness_ms"] == pytest.approx(2.0)
        assert d["max_lateness_ms"] == pytest.approx(3.0)


class TestAlphaBetaGammaTracker:
    """The constant-acceleration predictor, and why the gimbal needs it."""

    def test_gamma_must_be_in_range(self):
        for bad in (0.0, -0.1, 1.5):
            with pytest.raises(ParameterError):
                AlphaBetaGammaTracker(gamma=bad)

    def test_learns_a_constant_acceleration(self):
        tracker = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)
        accel, dt = 200.0, 1.0 / 60
        for i in range(600):
            t = i * dt
            tracker.update(0.5 * accel * t * t, t)
        assert tracker.acceleration == pytest.approx(accel, rel=0.1)

    def test_beats_constant_velocity_on_an_accelerating_target(self):
        """
        The reason the gimbal example uses this filter.

        A constant-velocity filter's velocity estimate lags by roughly
        accel * dt / beta, and that lag goes straight into the prediction. On a
        target with real angular acceleration the filter, not the motor, becomes
        the dominant error source.
        """
        accel, dt, horizon = 335.0, 1.0 / 60, 0.05

        def truth(t):
            return 0.5 * accel * t * t

        cv = AlphaBetaTracker(alpha=0.5, beta=0.3)
        ca = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)
        for i in range(600):
            t = i * dt
            cv.update(truth(t), t)
            ca.update(truth(t), t)

        now = 599 * dt
        target = truth(now + horizon)
        cv_error = abs(target - cv.predict(horizon))
        ca_error = abs(target - ca.predict(horizon))
        assert ca_error < cv_error / 3, (
            f"constant-acceleration error {ca_error:.4f} should be far below "
            f"constant-velocity error {cv_error:.4f}"
        )

    def test_predict_velocity_extrapolates_the_rate(self):
        tracker = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)
        accel, dt = 100.0, 1.0 / 60
        for i in range(600):
            t = i * dt
            tracker.update(0.5 * accel * t * t, t)
        assert tracker.predict_velocity(0.1) == pytest.approx(
            tracker.velocity + tracker.acceleration * 0.1
        )

    def test_reset_clears_acceleration(self):
        tracker = AlphaBetaGammaTracker()
        for i in range(20):
            tracker.update(float(i * i), i * 0.01)
        tracker.reset()
        assert tracker.acceleration == 0.0
        assert not tracker.initialised

    def test_out_of_order_and_zero_dt_are_safe(self):
        tracker = AlphaBetaGammaTracker()
        tracker.update(0.0, 1.0)
        tracker.update(10.0, 1.1)
        tracker.update(5.0, 1.05)
        tracker.update(6.0, 1.05)
        assert math.isfinite(tracker.position)
        assert math.isfinite(tracker.velocity)
        assert math.isfinite(tracker.acceleration)


class TestPredictionHorizon:
    """
    The horizon must be measured from capture time, exactly once.

    Double-counting the detector latency is easy - the measurement's age already
    contains it - and it is silent: the loop still runs, the motors still move,
    and the pointing error simply gets worse than not predicting at all. This
    was a real bug in the gimbal example.
    """

    @staticmethod
    def _crossing_error(horizon_builder):
        """
        Runs a straight-line pass and returns the error at the crossing point.

        Args:
            horizon_builder: Callable taking (age, command_latency) and
                returning the prediction horizon to use.

        Returns:
            Worst pointing error, in degrees, while the target was fastest.
        """
        speed, offset = 30.0, 10.0
        det_latency, cmd_latency, dt = 0.045, 0.006, 1.0 / 60

        def truth(t):
            return math.degrees(math.atan2(speed * (t - 4.0), offset))

        tracker = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)
        worst = 0.0
        t = 0.0
        while t < 8.0:
            captured = t - det_latency
            tracker.update(truth(captured), captured)
            age = t - captured
            predicted = tracker.predict(horizon_builder(age, cmd_latency))
            actual = truth(t + cmd_latency)
            if abs(tracker.velocity) > 120.0:
                worst = max(worst, abs(actual - predicted))
            t += dt
        return worst

    def test_correct_horizon_keeps_error_sub_degree(self):
        error = self._crossing_error(lambda age, cmd: age + cmd)
        assert error < 1.0, f"crossing error {error:.3f} deg with a correct horizon"

    def test_double_counting_the_detector_latency_is_much_worse(self):
        correct = self._crossing_error(lambda age, cmd: age + cmd)
        doubled = self._crossing_error(lambda age, cmd: age + cmd + age)
        assert doubled > correct * 5, (
            f"double-counted horizon gave {doubled:.3f} deg vs {correct:.3f} deg "
            "correct; the regression this guards against was a 30x degradation"
        )

    def test_prediction_beats_no_prediction_at_the_crossing(self):
        """Sanity: the whole exercise has to be worth doing."""
        speed, offset = 30.0, 10.0
        det_latency, dt = 0.045, 1.0 / 60

        def truth(t):
            return math.degrees(math.atan2(speed * (t - 4.0), offset))

        tracker = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)
        predicted_errors, naive_errors = [], []
        t = 0.0
        while t < 8.0:
            captured = t - det_latency
            measurement = truth(captured)
            tracker.update(measurement, captured)
            if abs(tracker.velocity) > 120.0:
                predicted_errors.append(abs(truth(t) - tracker.predict(det_latency)))
                naive_errors.append(abs(truth(t) - measurement))
            t += dt

        assert predicted_errors and naive_errors
        assert max(predicted_errors) < max(naive_errors) / 5
