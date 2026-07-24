"""
The digitizer's record/playback path, against the simulator.

`base_digitizer.py` sat at 11% coverage - the weakest module in the library and
the one most likely to harbour the class of defect the review found elsewhere.
These tests drive it against a real motor and pin three things it got wrong:

* the precision report's timing error included the settle delay, so a perfect
  playback could never score better than 50 ms - which is exactly the
  `EXCELLENT`/`GOOD` boundary the analyser applies to it;
* a playback that failed part-way logged the error and then announced
  `PLAYBACK COMPLETE`, returning no indication that anything had gone wrong;
* `speed_factor=0` divided by zero from inside the loop, after the motors had
  already been commanded.
"""
import asyncio
import json

import pytest
import pytest_asyncio

from mks_servo_can import Axis, CANInterface, RotaryKinematics
from mks_servo_can import constants as const
from mks_servo_can import exceptions
from mks_servo_can.digitizer import MotorDigitizer
from mks_servo_can.digitizer.data_structures import (
    DigitizedPoint,
    DigitizedSequence,
)

# The compliance simulator serves CAN IDs 1-3; nothing answers on this one.
ABSENT_CAN_ID = 99


def _sequence(axis_name: str = "probe", points: int = 4) -> DigitizedSequence:
    """Builds a short synthetic sequence: a slow sweep on one axis."""
    return DigitizedSequence(
        points=[
            DigitizedPoint(
                timestamp=0.2 * i,
                positions={axis_name: 5.0 * i},
                velocities={axis_name: 25.0},
            )
            for i in range(points)
        ],
        axis_names=[axis_name],
        axis_configs={axis_name: {"motor_can_id": 1}},
        recording_date="2026-07-24T00:00:00",
        recording_duration=0.2 * (points - 1),
        sample_rate=5.0,
        metadata={"source": "test"},
    )


@pytest_asyncio.fixture
async def digitizer(compliance_can_interface: CANInterface):
    """A digitizer with one initialized, enabled axis on the simulator."""
    dig = MotorDigitizer(compliance_can_interface)
    axis = Axis(
        can_interface_manager=compliance_can_interface,
        motor_can_id=1,
        name="probe",
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        ),
    )
    await dig.add_axis(axis)
    await dig.initialize_axes(enable_motors=True)

    yield dig

    try:
        await dig.cleanup()
    except Exception:  # pylint: disable=broad-except
        pass


def test_sequence_round_trips_through_disk(tmp_path):
    """What is saved must load back identically, including the optional fields."""
    dig = MotorDigitizer(can_interface=None)
    original = _sequence()
    path = tmp_path / "nested" / "seq.json"

    dig.save_sequence(original, str(path))
    assert path.is_file(), "save_sequence did not create the parent directory"

    restored = dig.load_sequence(str(path))
    assert restored.axis_names == original.axis_names
    assert restored.sample_rate == original.sample_rate
    assert restored.metadata == original.metadata
    assert len(restored.points) == len(original.points)
    assert restored.points[2].positions == original.points[2].positions
    assert restored.points[2].velocities == original.points[2].velocities

    # And the file is plain JSON a human or another tool can read.
    payload = json.loads(path.read_text())
    assert payload["points"][0]["timestamp"] == 0.0


def test_sequence_without_velocities_round_trips(tmp_path):
    """`velocities` is optional; a sequence recorded without it must reload."""
    dig = MotorDigitizer(can_interface=None)
    sequence = _sequence()
    for point in sequence.points:
        point.velocities = None

    path = tmp_path / "no_velocities.json"
    dig.save_sequence(sequence, str(path))
    restored = dig.load_sequence(str(path))
    assert all(point.velocities is None for point in restored.points)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_playback_rejects_a_non_positive_speed_factor(digitizer):
    """A zero speed factor must be refused before anything moves."""
    with pytest.raises(exceptions.ParameterError):
        await digitizer.playback_sequence(_sequence(), speed_factor=0.0)

    with pytest.raises(exceptions.ParameterError):
        await digitizer.playback_sequence(_sequence(), speed_factor=-1.0)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_playback_timing_error_excludes_the_settle_delay(digitizer):
    """
    The reported timing error must measure lateness, not the settle sleep.

    Precision testing sleeps 50 ms after dispatching each point to let the
    motor settle, and then took its timestamp - so every measurement carried
    that 50 ms. `PrecisionAnalyzer` calls anything under 50 ms `EXCELLENT`, so
    a playback that was never late at all was reported at the boundary of
    merely `GOOD`.
    """
    stats = await digitizer.playback_sequence(
        _sequence(), speed_factor=1.0, precision_test=True
    )

    assert stats is not None
    assert stats.executed_points == stats.planned_points
    assert stats.average_timing_error < 0.05, (
        f"average timing error was {stats.average_timing_error * 1000:.0f} ms; "
        "the 50 ms settle delay is being counted as lateness"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_playback_reports_a_failure_instead_of_announcing_completion(
    compliance_can_interface,
):
    """
    A playback that cannot command its motors must not report success.

    The move failure used to be caught, logged and forgotten: the run printed
    PLAYBACK COMPLETE and returned None, which is also what a successful run
    without precision testing returns.
    """
    dig = MotorDigitizer(compliance_can_interface)
    absent = Axis(
        can_interface_manager=compliance_can_interface,
        motor_can_id=ABSENT_CAN_ID,
        name="probe",
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        ),
    )
    await dig.add_axis(absent)

    with pytest.raises(exceptions.MKSServoError):
        await dig.playback_sequence(_sequence(), speed_factor=1.0)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_playback_moves_the_motor_to_the_last_recorded_point(digitizer):
    """The point of playback: the axis ends up where the sequence said."""
    sequence = _sequence(points=4)
    await digitizer.playback_sequence(sequence, speed_factor=1.0)
    await asyncio.sleep(0.5)

    final = await digitizer.axes["probe"].get_current_position_user()
    target = sequence.points[-1].positions["probe"]
    assert abs(final - target) < 2.0, (
        f"axis settled at {final:.2f} deg, expected about {target:.2f} deg"
    )
