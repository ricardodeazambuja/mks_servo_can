"""End-to-end streaming control against the simulator.

Exercises the path the camera gimbal example depends on: a fixed-rate loop
pushing fire-and-forget absolute-position targets to several axes at once, with
motor responses disabled. Unit tests cover this against mocks; these run it
against a real socket and a real motor model, which is where loop timing,
frame pacing and response-mode management actually get tested.
"""
import asyncio
import math
import time

import pytest

from mks_servo_can import (
    AlphaBetaGammaTracker,
    ServoStream,
    StreamAxis,
    constants as const,
    motor_profile,
)
from mks_servo_can.low_level_api import LowLevelAPI

pytestmark = pytest.mark.integration


def gimbal_axes():
    """
    Builds the three axes the gimbal example drives.

    Returns:
        StreamAxis objects for pan, tilt and roll on CAN IDs 1-3.
    """
    return [
        StreamAxis("pan", 1, accel_param=250, min_position=-170, max_position=170),
        StreamAxis("tilt", 2, accel_param=250, min_position=-45, max_position=90),
        StreamAxis("roll", 3, accel_param=250, min_position=-30, max_position=30),
    ]


@pytest.mark.asyncio
async def test_stream_runs_and_sends_frames(compliance_can_interface):
    """The loop must tick at roughly the requested rate and send every axis."""
    rate_hz = 100.0
    duration = 0.5
    async with ServoStream(
        compliance_can_interface, gimbal_axes(), rate_hz=rate_hz, watchdog_timeout=None
    ) as stream:
        stream.set_targets({"pan": 0.0, "tilt": 0.0, "roll": 0.0})
        await asyncio.sleep(duration)
        stats = stream.stats.as_dict()

    expected_ticks = rate_hz * duration
    assert stats["ticks"] > expected_ticks * 0.7, (
        f"loop managed only {stats['ticks']:.0f} of ~{expected_ticks:.0f} ticks"
    )
    assert stats["frames_sent"] >= stats["ticks"] * 3 - 3
    assert stats["send_errors"] == 0


@pytest.mark.asyncio
async def test_stream_moves_the_motor_to_the_streamed_target(
    compliance_can_interface,
):
    """
    A streamed target must actually move the shaft.

    This is the claim the whole streaming design rests on: 0xF5 accepts a fresh
    target while a move is in progress, so re-sending one every few
    milliseconds behaves as a continuous servo input rather than as a series of
    aborted point-to-point moves.
    """
    api = LowLevelAPI(compliance_can_interface)
    await api.enable_motor(1, True)
    await api.set_current_axis_to_zero(1)

    axis = StreamAxis("pan", 1, accel_param=250, max_position=180, max_rate=3600)
    async with ServoStream(
        compliance_can_interface, [axis], rate_hz=100, watchdog_timeout=None
    ) as stream:
        stream.set_target("pan", 30.0, feedforward_rate=180.0)
        await asyncio.sleep(1.5)

    await api.set_slave_respond_active(1, respond_enabled=True, active_enabled=False)
    counts = await api.read_encoder_value_addition(1)
    degrees = motor_profile.encoder_steps_to_degrees(counts)
    assert degrees > 5.0, (
        f"streamed target of 30 deg produced only {degrees:.2f} deg of motion"
    )


@pytest.mark.asyncio
async def test_stream_retargets_continuously_without_stalling(
    compliance_can_interface,
):
    """
    Sweeping the target must produce continuous motion, not a stutter.

    If each new target aborted the previous move rather than retargeting it,
    the shaft would barely move.
    """
    api = LowLevelAPI(compliance_can_interface)
    await api.enable_motor(1, True)
    await api.set_current_axis_to_zero(1)

    axis = StreamAxis("pan", 1, accel_param=250, max_position=180, max_rate=3600)
    async with ServoStream(
        compliance_can_interface, [axis], rate_hz=100, watchdog_timeout=None
    ) as stream:
        started = time.monotonic()
        while time.monotonic() - started < 2.0:
            phase = (time.monotonic() - started) / 2.0
            stream.set_target("pan", 40.0 * phase, feedforward_rate=20.0)
            await asyncio.sleep(0.01)
        assert stream.stats.send_errors == 0

    await api.set_slave_respond_active(1, respond_enabled=True, active_enabled=False)
    counts = await api.read_encoder_value_addition(1)
    degrees = motor_profile.encoder_steps_to_degrees(counts)
    assert degrees > 5.0, (
        f"continuously retargeted sweep produced only {degrees:.2f} deg; the "
        "motor may be aborting rather than retargeting"
    )


@pytest.mark.asyncio
async def test_soft_limits_are_enforced_on_the_wire(compliance_can_interface):
    """A target beyond the soft limit must never reach the motor."""
    api = LowLevelAPI(compliance_can_interface)
    await api.enable_motor(2, True)
    await api.set_current_axis_to_zero(2)

    axis = StreamAxis("tilt", 2, accel_param=250, min_position=-5.0, max_position=5.0)
    async with ServoStream(
        compliance_can_interface, [axis], rate_hz=100, watchdog_timeout=None
    ) as stream:
        stream.set_target("tilt", 500.0, feedforward_rate=360.0)
        await asyncio.sleep(1.5)

    await api.set_slave_respond_active(2, respond_enabled=True, active_enabled=False)
    counts = await api.read_encoder_value_addition(2)
    degrees = abs(motor_profile.encoder_steps_to_degrees(counts))
    assert degrees < 8.0, (
        f"axis reached {degrees:.2f} deg despite a 5 deg soft limit"
    )


@pytest.mark.asyncio
async def test_responses_are_restored_after_streaming(compliance_can_interface):
    """
    Request/response calls must work again once the stream stops.

    Leaving motor responses disabled would break every subsequent read in the
    process, in a way that looks like a communication fault rather than a
    configuration one.
    """
    api = LowLevelAPI(compliance_can_interface)
    async with ServoStream(
        compliance_can_interface, gimbal_axes(), rate_hz=100, watchdog_timeout=None
    ) as stream:
        stream.set_targets({"pan": 1.0, "tilt": 1.0, "roll": 1.0})
        await asyncio.sleep(0.2)

    # Should not raise or time out.
    for can_id in (1, 2, 3):
        assert isinstance(await api.read_en_pin_status(can_id), bool)


@pytest.mark.asyncio
async def test_watchdog_stops_a_stream_whose_producer_died(
    compliance_can_interface,
):
    """If nothing sets a target, the loop must command zero speed and hold."""
    api = LowLevelAPI(compliance_can_interface)
    await api.enable_motor(3, True)
    await api.set_current_axis_to_zero(3)

    axis = StreamAxis("roll", 3, accel_param=250, min_position=-90, max_position=90)
    async with ServoStream(
        compliance_can_interface, [axis], rate_hz=100, watchdog_timeout=0.15
    ) as stream:
        stream.set_target("roll", 60.0, feedforward_rate=180.0)
        await asyncio.sleep(0.2)  # let the watchdog expire
        await asyncio.sleep(1.0)  # ...and confirm it stays stopped

    await api.set_slave_respond_active(3, respond_enabled=True, active_enabled=False)
    counts = await api.read_encoder_value_addition(3)
    degrees = abs(motor_profile.encoder_steps_to_degrees(counts))
    assert degrees < 50.0, (
        f"axis coasted to {degrees:.2f} deg after the watchdog should have "
        "stopped it"
    )


@pytest.mark.asyncio
async def test_tracking_a_synthetic_pass_stays_on_target(
    compliance_can_interface,
):
    """
    The gimbal example's loop, end to end, against the simulator.

    Drives a straight-line pass through the predictor and the stream, and
    asserts the predicted bearing stays close to truth even at the crossing
    point, where the angular rate peaks near 172 deg/s.
    """
    api = LowLevelAPI(compliance_can_interface)
    for can_id in (1, 2):
        await api.enable_motor(can_id, True)
        await api.set_current_axis_to_zero(can_id)

    speed_mps, offset_m, detector_latency = 30.0, 10.0, 0.045

    def truth(t):
        return math.degrees(math.atan2(speed_mps * (t - 1.5), offset_m))

    axes = [
        StreamAxis("pan", 1, accel_param=250, min_position=-180, max_position=180,
                   max_rate=6000),
        StreamAxis("tilt", 2, accel_param=250, min_position=-90, max_position=90,
                   max_rate=6000),
    ]
    tracker = AlphaBetaGammaTracker(alpha=0.5, beta=0.3, gamma=0.05)

    errors = []
    async with ServoStream(
        compliance_can_interface, axes, rate_hz=100, watchdog_timeout=None
    ) as stream:
        started = time.monotonic()
        while True:
            now = time.monotonic() - started
            if now > 3.0:
                break
            captured = now - detector_latency
            tracker.update(truth(captured), captured)
            horizon = now - captured
            predicted = tracker.predict(horizon)
            stream.set_target(
                "pan", predicted, feedforward_rate=abs(tracker.predict_velocity(horizon))
            )
            if abs(tracker.velocity) > 100.0:
                errors.append(abs(truth(now) - predicted))
            await asyncio.sleep(1.0 / 60)

        assert stream.stats.send_errors == 0

    assert errors, "the pass never reached a high angular rate"
    worst = max(errors)
    assert worst < 2.0, (
        f"worst pointing error at the crossing was {worst:.3f} deg; the "
        "predictor should keep this well under a telephoto field of view"
    )
