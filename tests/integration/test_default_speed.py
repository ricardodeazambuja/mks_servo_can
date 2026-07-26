"""
The default speed of a user-unit move (defect L3), under simulated time.

`Axis.default_speed_param` is an MKS speed parameter, not a speed in user units.
Both move handlers pushed it through `kinematics.user_speed_to_motor_speed()`
anyway, which read 500 as 500 deg/s and returned 83 - so every
`move_to_position_abs_user()` or `move_relative_user()` with no explicit speed
ran at a sixth of its documented default. The `if sp is not None` guard that was
supposed to prevent this could never fire, because `sp` had already been
defaulted.

**These tests used to run a stopwatch over a simulator subprocess.** A sixfold
speed error only shows up as a 40% difference in wall-clock time - the
acceleration ramp dominates a short move - so the assertion had to carry a 25%
tolerance, which left the defect and a correct implementation less than a factor
of two apart. On a loaded machine that margin is scheduling noise.

They now run the simulator in this test's own event loop on a stepped clock, so
"how long the move took" is simulated time, measured to the millisecond and
identical on every run and every machine. The two moves being compared come out
*equal*, not merely close, and the tolerance is gone.
"""
import asyncio

import pytest
import pytest_asyncio

from mks_servo_can import Axis, RotaryKinematics
from mks_servo_can import constants as const

from ..stepped_simulator import run_under_simulated_time, start_stepped_simulator

# What default_speed_param actually means, in user units: MKS parameter 500 is
# 500 RPM, which is 3000 deg/s. A move at this speed and a move with no speed
# given must take the same time.
EQUIVALENT_SPEED_DEG_PER_S = 3000.0

# Large enough that the ramp does not dominate. Under simulated time the size no
# longer costs anything to run, but it keeps the comparison meaningful.
MOVE_DEGREES = 900.0

# The clock advances a millisecond at a time, so a duration is observed at most
# one step after it happened. Anything within a couple of steps is the same
# duration; the two moves in fact agree exactly.
STEP_TOLERANCE_S = 0.002


@pytest_asyncio.fixture
async def stepped_axis():
    """An enabled rotary Axis on an in-process simulator whose clock the test drives."""
    simulator = await start_stepped_simulator(num_motors=1)
    axis = Axis(
        can_interface_manager=simulator.can_interface,
        motor_can_id=1,
        name="DefaultSpeedAxis",
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        ),
    )
    await run_under_simulated_time(simulator.clock, axis.initialize())
    await run_under_simulated_time(simulator.clock, axis.enable_motor())
    await run_under_simulated_time(simulator.clock, axis.set_current_position_as_zero())

    yield simulator, axis

    await simulator.close()


async def _timed_relative_move(simulator, axis: Axis, degrees: float, speed_user=None):
    """
    Runs a relative move to completion and returns the simulated time it took.

    Args:
        simulator: The `SteppedSimulator` whose clock to advance.
        axis: The axis to command.
        degrees: How far to move, in user units.
        speed_user: Optional explicit speed in user units.

    Returns:
        Simulated seconds the move took.
    """
    _, elapsed = await run_under_simulated_time(
        simulator.clock,
        axis.move_relative_user(degrees, speed_user=speed_user, wait=True),
    )
    return elapsed


@pytest.mark.integration
@pytest.mark.asyncio
async def test_relative_user_move_defaults_to_the_mks_parameter(stepped_axis):
    """
    Omitting the speed must use `default_speed_param` as an MKS parameter.

    Parameter 500 is 500 RPM, i.e. 3000 deg/s, so a move with no speed given
    must take exactly as long as one explicitly asked for at 3000 deg/s. The
    defect made it six times slower.
    """
    simulator, axis = stepped_axis

    default_elapsed = await _timed_relative_move(simulator, axis, MOVE_DEGREES)
    equivalent_elapsed = await _timed_relative_move(
        simulator, axis, -MOVE_DEGREES, speed_user=EQUIVALENT_SPEED_DEG_PER_S
    )

    assert default_elapsed == pytest.approx(equivalent_elapsed, abs=STEP_TOLERANCE_S), (
        f"a default-speed move took {default_elapsed:.3f}s of simulated time "
        f"against {equivalent_elapsed:.3f}s for the same move at "
        f"{EQUIVALENT_SPEED_DEG_PER_S} deg/s - the default is not being used as "
        "an MKS parameter"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_absolute_user_move_defaults_to_the_mks_parameter(stepped_axis):
    """The absolute handler had its own copy of the same mistake."""
    simulator, axis = stepped_axis

    _, default_elapsed = await run_under_simulated_time(
        simulator.clock, axis.move_to_position_abs_user(MOVE_DEGREES, wait=True)
    )
    _, equivalent_elapsed = await run_under_simulated_time(
        simulator.clock,
        axis.move_to_position_abs_user(
            0.0, speed_user=EQUIVALENT_SPEED_DEG_PER_S, wait=True
        ),
    )

    assert default_elapsed == pytest.approx(equivalent_elapsed, abs=STEP_TOLERANCE_S), (
        f"a default-speed move took {default_elapsed:.3f}s of simulated time "
        f"against {equivalent_elapsed:.3f}s for the return trip at "
        f"{EQUIVALENT_SPEED_DEG_PER_S} deg/s - the default is not being used as "
        "an MKS parameter"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_explicit_user_speed_is_still_converted(stepped_axis):
    """
    The fix must not swing the other way.

    A speed the caller *does* supply is in user units and still has to go
    through the kinematics: asking for 300 deg/s has to be markedly slower than
    the 3000 deg/s default.

    Not ten times slower, though, even at a tenth of the speed: over 900 degrees
    the acceleration ramp accounts for most of the fast move, so the ratio comes
    out near 2.1. That is the same reason the defect this file exists for showed
    up as 40% in wall-clock time rather than 600%.
    """
    simulator, axis = stepped_axis

    slow_elapsed = await _timed_relative_move(
        simulator, axis, MOVE_DEGREES, speed_user=300.0
    )
    default_elapsed = await _timed_relative_move(simulator, axis, -MOVE_DEGREES)

    assert slow_elapsed > default_elapsed * 2, (
        f"an explicit 300 deg/s move took {slow_elapsed:.3f}s of simulated time "
        f"against {default_elapsed:.3f}s for the default - the caller's speed is "
        "being ignored"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_absolute_move_back_to_the_previous_position_is_not_skipped(stepped_axis):
    """
    Found while fixing L3: an absolute move can be dropped on the floor.

    `wait=True` returns as soon as the completion frame resolves the move
    future; the cached position is refreshed a moment later. A caller who
    immediately commands the position the axis started from therefore hits the
    "already at target" shortcut against a stale cache - the move is never sent
    and the call reports success.
    """
    simulator, axis = stepped_axis

    await run_under_simulated_time(
        simulator.clock, axis.move_to_position_abs_user(MOVE_DEGREES, wait=True)
    )
    await run_under_simulated_time(
        simulator.clock, axis.move_to_position_abs_user(0.0, wait=True)
    )

    final_degrees, _ = await run_under_simulated_time(
        simulator.clock, axis.get_current_position_user()
    )
    assert abs(final_degrees) < 2.0, (
        f"axis is at {final_degrees:.2f} deg, not back at 0 - the return move "
        "was skipped against a stale position cache"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_the_same_move_takes_the_same_time_twice(stepped_axis):
    """
    The property the wall-clock version could not state at all.

    Two identical moves under simulated time agree to the step, which is what
    makes the assertions above equalities rather than tolerances.
    """
    simulator, axis = stepped_axis

    first = await _timed_relative_move(simulator, axis, MOVE_DEGREES)
    second = await _timed_relative_move(simulator, axis, -MOVE_DEGREES)

    # Simulated time is counted in whole nanoseconds; the float subtraction
    # that turns two readings into a duration is the only imprecision here.
    assert first == pytest.approx(second, abs=1e-9), (
        f"identical moves took {first:.9f}s and {second:.9f}s of simulated time"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_a_move_costs_no_wall_clock_time(stepped_axis):
    """
    Nine hundred degrees of motion, and the test does not wait for any of it.

    This is the other half of what stepped time buys: the suite stops paying
    real seconds to watch a simulated motor turn.
    """
    simulator, axis = stepped_axis
    loop = asyncio.get_running_loop()

    started = loop.time()
    simulated = await _timed_relative_move(simulator, axis, MOVE_DEGREES * 4)
    wall_clock = loop.time() - started

    assert simulated > 1.0, "the move should have taken over a second of simulated time"
    assert wall_clock < simulated / 2, (
        f"{simulated:.2f}s of simulated motion cost {wall_clock:.2f}s of real "
        "time; the clock is not actually stepped"
    )
