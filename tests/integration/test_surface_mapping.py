"""
The automated grid walk, against three real simulated motors.

`automated_grid_mapping` is the only part of `surface_mapping.py` that commands
hardware, and it had two defects a pure-function test cannot reach:

* the serpentine it documents alternated on `len(surface_points) % 2` - the
  number of points measured so far, not the row index - so a grid with an even
  number of columns went left-to-right on every row and the pen crossed the
  whole workpiece between them;
* any exception was logged and swallowed, after which the method printed
  "GRID MAPPING COMPLETE" and returned a partial map. The caller gets a map
  either way, so there was nothing to tell the two apart.

Heights here are not measured. `_probe_surface_height` has no contact detection;
it moves the pen down and invents a millimetre of variation. These tests assert
that everything it produces is labelled as simulated, because the map is saved
to JSON and read later by someone who was not here.
"""
import pytest
import pytest_asyncio

from mks_servo_can import CANInterface
from mks_servo_can.digitizer.data_structures import (
    DigitizedPoint,
    DigitizedSequence,
)
from mks_servo_can.digitizer.surface_mapping import EnhancedHeightMapGenerator
from mks_servo_can.digitizer.utils import create_linear_axis
from mks_servo_can.exceptions import MKSServoError

# The compliance simulator serves CAN IDs 1-3; nothing answers on this one.
ABSENT_CAN_ID = 99


@pytest_asyncio.fixture
async def plotter(compliance_can_interface: CANInterface):
    """An XYZ plotter on the simulator's three motors, initialized and enabled."""
    generator = EnhancedHeightMapGenerator(compliance_can_interface)
    await generator.setup_standard_plotter([1, 2, 3])

    yield generator

    try:
        await generator.cleanup()
    except Exception:  # pylint: disable=broad-except
        pass


@pytest.mark.asyncio
async def test_the_grid_walk_visits_every_point_in_a_serpentine(plotter):
    """
    Four columns by two rows: the row that matters is the second one.

    With an even number of columns the old alternation left both rows running
    left-to-right, which is the case this grid is chosen to catch.
    """
    surface_map = await plotter.automated_grid_mapping(
        x_range=(0.0, 3.0), y_range=(0.0, 1.0), grid_spacing=1.0, pen_probe_depth=1.0
    )

    visited = [(point.x, point.y) for point in surface_map.points]

    assert visited == [
        (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),
        (3.0, 1.0), (2.0, 1.0), (1.0, 1.0), (0.0, 1.0),
    ], "the pen did not walk a serpentine over the grid"


@pytest.mark.asyncio
async def test_a_completed_map_says_it_is_complete(plotter):
    """The counts in the metadata are what a later reader has to go on."""
    surface_map = await plotter.automated_grid_mapping(
        x_range=(0.0, 1.0), y_range=(0.0, 1.0), grid_spacing=1.0, pen_probe_depth=1.0
    )

    assert surface_map.metadata["planned_points"] == 4
    assert surface_map.metadata["measured_points"] == 4
    assert surface_map.metadata["complete"] is True
    assert surface_map.statistics["point_count"] == 4


@pytest.mark.asyncio
async def test_every_height_is_marked_as_simulated(plotter):
    """
    There is no contact detection in this build.

    A height nobody measured must not be readable as one, in the returned map or
    in the file it is saved to.
    """
    surface_map = await plotter.automated_grid_mapping(
        x_range=(0.0, 1.0), y_range=(0.0, 0.0), grid_spacing=1.0, pen_probe_depth=1.0
    )

    assert surface_map.metadata["simulated_probe"] is True
    for point in surface_map.points:
        assert point.metadata["simulated"] is True
        assert point.metadata["contact_detected"] is False


@pytest.mark.asyncio
async def test_a_recorded_sweep_plays_back_and_is_graded(plotter, tmp_path):
    """
    The whole path: a saved recording, replayed, then graded.

    `precision_test_from_recording` is what a user runs to find out whether the
    machine can retrace what it mapped, and nothing had ever called it.
    """
    sequence = DigitizedSequence(
        points=[
            DigitizedPoint(
                timestamp=0.3 * i,
                positions={"X": 1.0 * i, "Y": 0.5 * i, "Pen": -0.2 * i},
            )
            for i in range(3)
        ],
        axis_names=["X", "Y", "Pen"],
        axis_configs={"X": {}, "Y": {}, "Pen": {}},
        recording_date="2026-07-25T00:00:00",
        recording_duration=0.6,
        sample_rate=3.333,
    )
    path = tmp_path / "sweep.json"
    plotter.save_sequence(sequence, str(path))

    stats = await plotter.precision_test_from_recording(str(path))

    assert stats.planned_points == 3
    assert stats.executed_points == 3
    grades = plotter.grade_surface_mapping_precision(stats)
    assert set(grades) == {"height", "xy"}


@pytest.mark.asyncio
async def test_a_failed_grid_walk_does_not_report_completion(
    compliance_can_interface: CANInterface,
):
    """
    One axis points at a CAN ID nothing answers on.

    The failure used to be logged and swallowed: the method printed "GRID
    MAPPING COMPLETE" and handed back whatever it had, which the caller cannot
    distinguish from a finished map.
    """
    generator = EnhancedHeightMapGenerator(compliance_can_interface)
    for name, can_id in (("X", 1), ("Y", 2), ("Pen", ABSENT_CAN_ID)):
        await generator.add_axis(
            create_linear_axis(compliance_can_interface, can_id, name, 40.0)
        )
    # Initialization fails on the absent axis, which is the point; the axes are
    # registered either way, so the grid walk still has something to command.
    with pytest.raises(MKSServoError):
        await generator.initialize_axes(enable_motors=True)

    with pytest.raises(MKSServoError) as failure:
        await generator.automated_grid_mapping(
            x_range=(0.0, 1.0),
            y_range=(0.0, 0.0),
            grid_spacing=1.0,
            pen_probe_depth=1.0,
        )

    assert "GRID MAPPING COMPLETE" not in str(failure.value)
