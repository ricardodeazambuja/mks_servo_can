"""
The default speed of a user-unit move (defect L3).

`Axis.default_speed_param` is an MKS speed parameter, not a speed in user units.
Both move handlers pushed it through `kinematics.user_speed_to_motor_speed()`
anyway, which read 500 as 500 deg/s and returned 83 - so every
`move_to_position_abs_user()` or `move_relative_user()` with no explicit speed
ran at a sixth of its documented default. The `if sp is not None` guard that was
supposed to prevent this could never fire, because `sp` had already been
defaulted.

These tests time a default-speed move against the same move explicitly requested
at the speed the default is supposed to mean. Comparing two moves rather than
checking a wall-clock threshold keeps them independent of how fast the machine
running them is, and of the acceleration ramp, which dominates a short move.
"""
import asyncio
import time

import pytest
import pytest_asyncio

from mks_servo_can import Axis, CANInterface, RotaryKinematics
from mks_servo_can import constants as const

# What default_speed_param actually means, in user units: MKS parameter 500 is
# 500 RPM, which is 3000 deg/s. A move at this speed and a move with no speed
# given must take the same time.
EQUIVALENT_SPEED_DEG_PER_S = 3000.0

# Large enough that the ramp does not dominate, small enough to stay quick. The
# ramp is why these tests compare against the equivalent speed rather than
# against the 498 deg/s the defect produced: over this distance the motor spends
# much of the move accelerating, so a sixfold speed error only shows up as a
# 40% time difference.
MOVE_DEGREES = 900.0

# The two moves being compared are the same distance under the same ramp, so
# they should land within a few percent of each other.
TOLERANCE = 1.25


@pytest_asyncio.fixture
async def rotary_axis(basic_can_interface: CANInterface):
    """An enabled rotary Axis on the basic simulator, parked at zero."""
    axis = Axis(
        can_interface_manager=basic_can_interface,
        motor_can_id=1,
        name="DefaultSpeedAxis",
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        ),
    )
    await axis.initialize()
    await axis.enable_motor()
    await axis.stop_motor()
    await asyncio.sleep(0.2)
    await axis.set_current_position_as_zero()

    yield axis

    try:
        await axis.stop_motor()
        await asyncio.sleep(0.2)
    except Exception:  # pylint: disable=broad-except
        pass  # teardown must not mask the test's own failure


async def _timed_relative_move(axis: Axis, degrees: float, speed_user=None) -> float:
    """Runs a relative move to completion and returns how long it took."""
    started = time.monotonic()
    await axis.move_relative_user(degrees, speed_user=speed_user, wait=True)
    return time.monotonic() - started


@pytest.mark.integration
@pytest.mark.asyncio
async def test_relative_user_move_defaults_to_the_mks_parameter(rotary_axis: Axis):
    """
    Omitting the speed must use `default_speed_param` as an MKS parameter.

    Parameter 500 is 500 RPM, i.e. 3000 deg/s, so a move with no speed given
    must take as long as one explicitly asked for at 3000 deg/s - and clearly
    less than the 498 deg/s the defect produced.
    """
    axis = rotary_axis

    default_elapsed = await _timed_relative_move(axis, MOVE_DEGREES)
    equivalent_elapsed = await _timed_relative_move(
        axis, -MOVE_DEGREES, speed_user=EQUIVALENT_SPEED_DEG_PER_S
    )

    assert default_elapsed < equivalent_elapsed * TOLERANCE, (
        f"a default-speed move took {default_elapsed:.2f}s against "
        f"{equivalent_elapsed:.2f}s for the same move at "
        f"{EQUIVALENT_SPEED_DEG_PER_S} deg/s - the default is not being used as "
        "an MKS parameter"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_absolute_user_move_defaults_to_the_mks_parameter(rotary_axis: Axis):
    """The absolute handler had its own copy of the same mistake."""
    axis = rotary_axis

    started = time.monotonic()
    await axis.move_to_position_abs_user(MOVE_DEGREES, wait=True)
    default_elapsed = time.monotonic() - started

    started = time.monotonic()
    await axis.move_to_position_abs_user(
        0.0, speed_user=EQUIVALENT_SPEED_DEG_PER_S, wait=True
    )
    equivalent_elapsed = time.monotonic() - started

    assert default_elapsed < equivalent_elapsed * TOLERANCE, (
        f"a default-speed move took {default_elapsed:.2f}s against "
        f"{equivalent_elapsed:.2f}s for the return trip at "
        f"{EQUIVALENT_SPEED_DEG_PER_S} deg/s - the default is not being used as "
        "an MKS parameter"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_explicit_user_speed_is_still_converted(rotary_axis: Axis):
    """
    The fix must not swing the other way.

    A speed the caller *does* supply is in user units and still has to go
    through the kinematics: asking for 300 deg/s must be visibly slower than
    the 3000 deg/s default.
    """
    axis = rotary_axis

    slow_elapsed = await _timed_relative_move(axis, MOVE_DEGREES, speed_user=300.0)
    default_elapsed = await _timed_relative_move(axis, -MOVE_DEGREES)

    assert default_elapsed * 2 < slow_elapsed, (
        f"an explicit 300 deg/s move took {slow_elapsed:.2f}s against "
        f"{default_elapsed:.2f}s for the default - the caller's speed is being "
        "ignored"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_absolute_move_back_to_the_previous_position_is_not_skipped(
    rotary_axis: Axis,
):
    """
    Found while fixing L3: an absolute move can be dropped on the floor.

    `wait=True` returns as soon as the completion frame resolves the move
    future; the cached position is refreshed a moment later. A caller who
    immediately commands the position the axis started from therefore hits the
    "already at target" shortcut against a stale cache - the move is never sent
    and the call reports success.
    """
    axis = rotary_axis

    await axis.move_to_position_abs_user(MOVE_DEGREES, wait=True)
    await axis.move_to_position_abs_user(0.0, wait=True)

    final_degrees = await axis.get_current_position_user()
    assert abs(final_degrees) < 2.0, (
        f"axis is at {final_degrees:.2f} deg, not back at 0 - the return move "
        "was skipped against a stale position cache"
    )
