"""
A simulated motor driven by simulated time.

This is what the clock was built for. Every other simulator test in this
repository paces itself with `asyncio.sleep` against a wall clock, so it
measures the machine as much as the motion model: the assertions have to be
tolerances, the suite takes minutes, and the timing-sensitive cases will go
flaky under CI load sooner or later.

Here the motor moves only when time is advanced, so the assertions are exact and
the whole file costs no measurable wall-clock time. `test_the_same_move_twice`
is the one that matters most - it is the property none of the wall-clock tests
can state.
"""
import asyncio

import pytest

from mks_servo_can import constants as const
from mks_simulator.clock import SteppedClock
from mks_simulator.motor_model import SimulatedMotor

STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION


async def _make_motor(clock, can_id=1):
    """
    Builds an enabled motor running on the given clock.

    Args:
        clock: The clock to drive it with.
        can_id: CAN ID for the motor.

    Returns:
        The started, enabled SimulatedMotor.
    """
    motor = SimulatedMotor(
        can_id=can_id,
        loop=asyncio.get_event_loop(),
        motor_type=const.MOTOR_TYPE_SERVO42D,
        steps_per_rev_encoder=STEPS_PER_REV,
    )
    motor._clock = clock
    motor.is_enabled = True
    await motor.start()
    # Let the update task reach its first tick before anyone advances time.
    await asyncio.sleep(0)
    return motor


@pytest.mark.asyncio
async def test_nothing_moves_until_time_is_advanced():
    """
    The guarantee everything else here rests on.

    Under a real clock a motor commanded to move starts moving immediately and
    a test has to guess how long to wait. Under a stepped clock it does not move
    at all, so "before" and "after" are exactly what the test says they are.
    """
    clock = SteppedClock()
    motor = await _make_motor(clock)
    try:
        motor.target_rpm = 60.0
        start = motor.position_steps

        for _ in range(200):
            await asyncio.sleep(0)

        assert motor.position_steps == start, (
            "the motor moved without simulated time being advanced"
        )
    finally:
        await motor.stop_simulation()


@pytest.mark.asyncio
async def test_a_motor_at_constant_speed_covers_the_expected_distance():
    """
    60 RPM for one simulated second is one revolution, to the step.

    Written against a wall clock this assertion has to carry a tolerance wide
    enough to absorb scheduling jitter - which is precisely the slack that hid a
    sixfold speed error until it was measured a different way.
    """
    clock = SteppedClock()
    motor = await _make_motor(clock)
    try:
        # Straight to speed: no acceleration ramp to account for.
        motor.target_accel_mks = 0
        motor.current_rpm = 60.0
        motor.target_rpm = 60.0
        start = motor.position_steps

        await clock.advance(1.0)

        travelled = motor.position_steps - start
        assert travelled == pytest.approx(STEPS_PER_REV, rel=1e-9), (
            f"one second at 60 RPM moved {travelled} steps, expected one "
            f"revolution ({STEPS_PER_REV})"
        )
    finally:
        await motor.stop_simulation()


@pytest.mark.asyncio
async def test_distance_scales_with_simulated_time():
    """Two seconds must be exactly twice one second."""
    results = []
    for seconds in (1.0, 2.0):
        clock = SteppedClock()
        motor = await _make_motor(clock)
        try:
            motor.target_accel_mks = 0
            motor.current_rpm = 60.0
            motor.target_rpm = 60.0
            start = motor.position_steps
            await clock.advance(seconds)
            results.append(motor.position_steps - start)
        finally:
            await motor.stop_simulation()

    assert results[1] == pytest.approx(2 * results[0], rel=1e-9)


@pytest.mark.asyncio
async def test_the_same_move_twice_gives_the_same_answer():
    """
    Determinism, stated as a test.

    Two identical runs must agree bit for bit. No wall-clock test in this
    repository can make this assertion, which is why the timing-sensitive ones
    are written as tolerances and will eventually go flaky under load.
    """
    async def run():
        clock = SteppedClock()
        motor = await _make_motor(clock)
        try:
            motor.target_accel_mks = 100
            motor.target_rpm = 120.0
            trace = []
            for _ in range(20):
                await clock.advance(0.05)
                trace.append((clock.now(), motor.position_steps, motor.current_rpm))
            return trace
        finally:
            await motor.stop_simulation()

    first = await run()
    second = await run()
    assert first == second, "two identical stepped runs diverged"


@pytest.mark.asyncio
async def test_acceleration_is_integrated_not_jumped():
    """
    A large advance must still ramp.

    `advance()` splits into sub-steps precisely so that stepping ten seconds is
    not the same as handing the motion model a single ten-second dt, which would
    apply the whole acceleration at once and overshoot.
    """
    clock = SteppedClock()
    motor = await _make_motor(clock)
    try:
        motor.target_accel_mks = 200  # A slow ramp: larger is slower.
        motor.target_rpm = 300.0

        await clock.advance(0.05)
        early = motor.current_rpm

        assert 0 < early < 300.0, (
            f"speed jumped to {early} instead of ramping towards 300 RPM"
        )
        assert motor.motor_status_code == const.MOTOR_STATUS_SPEED_UP
    finally:
        await motor.stop_simulation()


@pytest.mark.asyncio
async def test_two_motors_on_one_clock_stay_in_step():
    """
    Motors sharing a clock integrate the same intervals.

    Otherwise a multi-axis result would depend on the order the event loop
    happened to schedule the motor tasks in - non-determinism reintroduced at
    exactly the layer that coordinates axes.
    """
    clock = SteppedClock()
    a = await _make_motor(clock, can_id=1)
    b = await _make_motor(clock, can_id=2)
    try:
        for motor in (a, b):
            motor.target_accel_mks = 0
            motor.current_rpm = 60.0
            motor.target_rpm = 60.0

        await clock.advance(0.5)

        assert a.position_steps == b.position_steps, (
            f"motors diverged under one clock: {a.position_steps} vs "
            f"{b.position_steps}"
        )
    finally:
        await a.stop_simulation()
        await b.stop_simulation()


@pytest.mark.asyncio
async def test_a_stopped_motor_does_not_hold_up_the_clock():
    """
    Tearing a motor down must not hang the next advance.

    The barrier waits for every registered motor; one that has been stopped
    will never tick again. `stop_simulation` unregisters it, and this is the
    test that says so.
    """
    clock = SteppedClock()
    a = await _make_motor(clock, can_id=1)
    b = await _make_motor(clock, can_id=2)

    await clock.advance(0.05)
    await a.stop_simulation()

    advanced = await asyncio.wait_for(clock.advance(0.1), timeout=5.0)
    assert advanced == pytest.approx(0.1)

    await b.stop_simulation()
    # And with no motors left at all.
    await asyncio.wait_for(clock.advance(0.1), timeout=5.0)


@pytest.mark.asyncio
async def test_the_default_clock_is_still_real_time():
    """
    A motor built without a clock behaves exactly as it always has.

    `--step` is opt-in; every existing test and every existing user gets the
    wall-clock simulator unchanged.
    """
    motor = SimulatedMotor(can_id=9, loop=asyncio.get_event_loop())
    assert motor._clock.is_stepped is False
