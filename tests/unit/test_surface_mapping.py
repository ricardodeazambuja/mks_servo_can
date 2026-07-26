"""
The surface mapper's arithmetic and its empty cases.

`surface_mapping.py` sat at 17% - the weakest module in the library, and the one
whose output a user acts on physically: a wrong height map is a probe driven
into a workpiece, not a wrong number on a screen.

Three of the defects pinned here are the same shape as L12, which graded a
playback that never ran:

* a map of no points carried `statistics={}`, so anything that read its
  statistics - including this module's own display routine - raised `KeyError`
  rather than saying "nothing was measured";
* `_sequence_to_surface_map` guarded with `if not sequence`, which a dataclass
  never satisfies, so a recording of nothing produced an empty map that reads
  like a perfectly flat surface;
* the serpentine that `automated_grid_mapping` claims to walk alternated on the
  number of points collected rather than the row index, so a grid with an even
  number of columns went left-to-right on every row.

The grid walk itself is exercised against real motors in
`tests/integration/test_surface_mapping.py`.
"""
import json

import pytest

from mks_servo_can.digitizer.data_structures import (
    DigitizedPoint,
    DigitizedSequence,
    PlaybackStats,
)
from mks_servo_can.digitizer.surface_mapping import (
    EnhancedHeightMapGenerator,
    SurfaceMap,
    SurfacePoint,
)


def _generator() -> EnhancedHeightMapGenerator:
    """
    Builds a generator without touching a CAN interface.

    Everything tested here is arithmetic over points that have already been
    collected, so there is nothing for a bus to do.
    """
    generator = EnhancedHeightMapGenerator.__new__(EnhancedHeightMapGenerator)
    generator.plotter_config = {}
    generator.current_surface_map = None
    return generator


def _points(*coords) -> list:
    """Builds surface points from `(x, y, z)` triples."""
    return [
        SurfacePoint(x=x, y=y, z=z, timestamp=float(i))
        for i, (x, y, z) in enumerate(coords)
    ]


class TestEmptyInput:
    """What a map of nothing must look like, and must not do."""

    def test_a_map_of_no_points_still_has_statistics(self):
        """
        The statistics of an empty map used to be `{}`.

        Every consumer then had to know that one shape of `SurfaceMap` is
        missing the keys every other one has, and none of them did.
        """
        surface_map = _generator()._create_surface_map_from_points([])

        assert surface_map.points == []
        assert surface_map.statistics["point_count"] == 0
        assert surface_map.statistics["z_range"] == 0.0
        assert surface_map.statistics["z_mean"] == 0.0
        assert surface_map.statistics["z_std_dev"] == 0.0
        assert surface_map.statistics["surface_area"] == 0.0

    def test_displaying_an_empty_map_says_so_instead_of_raising(self, capsys):
        """
        This is reached straight after "GRID MAPPING COMPLETE" is printed.

        A run that measured nothing therefore announced success and then died
        with `KeyError: 'point_count'`.
        """
        generator = _generator()
        generator._display_surface_statistics(
            generator._create_surface_map_from_points([])
        )

        assert "No points were measured" in capsys.readouterr().out

    def test_a_recording_with_no_points_is_refused(self):
        """
        `if not sequence` never fires: a dataclass instance is always truthy.

        So an empty recording used to come back as a valid-looking flat surface
        rather than as an error.
        """
        empty = DigitizedSequence(
            points=[],
            axis_names=["X", "Y", "Pen"],
            axis_configs={},
            recording_date="2026-07-25T00:00:00",
            recording_duration=0.0,
            sample_rate=20.0,
        )

        with pytest.raises(ValueError, match="no points"):
            _generator()._sequence_to_surface_map(empty)

    def test_a_recording_without_x_and_y_is_refused(self):
        """
        A recording of the pen axis alone describes no surface.

        It used to produce an empty map, which then crashed the display.
        """
        sequence = DigitizedSequence(
            points=[DigitizedPoint(timestamp=0.1 * i, positions={"Pen": float(i)})
                    for i in range(5)],
            axis_names=["Pen"],
            axis_configs={},
            recording_date="2026-07-25T00:00:00",
            recording_duration=0.5,
            sample_rate=10.0,
        )

        with pytest.raises(ValueError, match="X and a Y"):
            _generator()._sequence_to_surface_map(sequence)

    def test_no_sequence_at_all_is_refused(self):
        """The original guard's intent, which still holds."""
        with pytest.raises(ValueError, match="No sequence"):
            _generator()._sequence_to_surface_map(None)


class TestArithmetic:
    """The numbers a user reads off the map."""

    def test_bounds_and_statistics_over_known_points(self):
        """Exact values, not ranges: this is arithmetic, not measurement."""
        surface_map = _generator()._create_surface_map_from_points(
            _points((0.0, 0.0, 1.0), (10.0, 0.0, 3.0), (0.0, 4.0, 2.0), (10.0, 4.0, 2.0))
        )

        assert surface_map.bounds == {"X": (0.0, 10.0), "Y": (0.0, 4.0), "Z": (1.0, 3.0)}
        assert surface_map.statistics["point_count"] == 4
        assert surface_map.statistics["z_range"] == 2.0
        assert surface_map.statistics["z_mean"] == 2.0
        assert surface_map.statistics["surface_area"] == 40.0

    def test_standard_deviation_is_the_sample_deviation(self):
        """Four values whose sample deviation is exactly 1."""
        assert _generator()._calculate_std_dev([1.0, 2.0, 3.0, 4.0]) == pytest.approx(
            1.2909944487358056
        )
        assert _generator()._calculate_std_dev([2.0, 4.0]) == pytest.approx(1.4142135623730951)

    def test_a_single_reading_has_no_spread(self):
        """One value has no deviation, and no exception either."""
        assert _generator()._calculate_std_dev([7.0]) == 0.0
        assert _generator()._calculate_std_dev([]) == 0.0

    def test_area_needs_three_points_to_mean_anything(self):
        """Two points describe a line; the estimator says zero rather than guessing."""
        generator = _generator()

        assert generator._estimate_surface_area([]) == 0.0
        assert generator._estimate_surface_area(_points((0.0, 0.0, 0.0))) == 0.0
        assert generator._estimate_surface_area(
            _points((0.0, 0.0, 0.0), (5.0, 5.0, 0.0))
        ) == 0.0

    def test_area_of_collinear_points_is_zero(self):
        """Three points on one line bound no area, however many there are."""
        area = _generator()._estimate_surface_area(
            _points((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
        )
        assert area == 0.0


class TestSequenceConversion:
    """Turning a recording into a surface."""

    def test_pen_position_becomes_the_height(self):
        """The Z of a mapped point is where the pen was."""
        sequence = DigitizedSequence(
            points=[
                DigitizedPoint(timestamp=0.0, positions={"X": 1.0, "Y": 2.0, "Pen": -0.5}),
                DigitizedPoint(timestamp=0.1, positions={"X": 3.0, "Y": 4.0, "Pen": -0.7}),
            ],
            axis_names=["X", "Y", "Pen"],
            axis_configs={},
            recording_date="2026-07-25T00:00:00",
            recording_duration=0.1,
            sample_rate=10.0,
        )

        surface_map = _generator()._sequence_to_surface_map(sequence)

        assert [(p.x, p.y, p.z) for p in surface_map.points] == [
            (1.0, 2.0, -0.5),
            (3.0, 4.0, -0.7),
        ]

    def test_points_missing_a_coordinate_are_dropped_not_defaulted(self):
        """
        A point without a Y is not a point at Y=0.

        Defaulting would put a measurement somewhere it was never taken.
        """
        sequence = DigitizedSequence(
            points=[
                DigitizedPoint(timestamp=0.0, positions={"X": 1.0, "Y": 2.0, "Pen": 0.0}),
                DigitizedPoint(timestamp=0.1, positions={"X": 3.0, "Pen": 0.0}),
            ],
            axis_names=["X", "Y", "Pen"],
            axis_configs={},
            recording_date="2026-07-25T00:00:00",
            recording_duration=0.1,
            sample_rate=10.0,
        )

        surface_map = _generator()._sequence_to_surface_map(sequence)

        assert len(surface_map.points) == 1
        assert (surface_map.points[0].x, surface_map.points[0].y) == (1.0, 2.0)

    def test_a_z_axis_stands_in_for_a_pen_axis(self):
        """Not every machine calls the vertical axis "Pen"."""
        sequence = DigitizedSequence(
            points=[DigitizedPoint(timestamp=0.0, positions={"X": 1.0, "Y": 2.0, "Z": 9.0})],
            axis_names=["X", "Y", "Z"],
            axis_configs={},
            recording_date="2026-07-25T00:00:00",
            recording_duration=0.0,
            sample_rate=10.0,
        )

        surface_map = _generator()._sequence_to_surface_map(sequence)

        assert surface_map.points[0].z == 9.0


class TestPrecisionGrading:
    """
    L12's defect, found again in a second grader.

    `_analyze_surface_mapping_precision` read the position errors and nothing
    else, so a playback that managed one point of five hundred - accurately -
    was reported EXCELLENT on both height and XY.
    """

    @staticmethod
    def _stats(executed: int, planned: int, error: float) -> PlaybackStats:
        """Builds playback statistics with a given accuracy and completeness."""
        return PlaybackStats(
            planned_points=planned,
            executed_points=executed,
            average_position_error={"X": error, "Y": error, "Pen": error},
            max_position_error={"X": error, "Y": error, "Pen": error},
            average_timing_error=0.001,
            max_timing_error=0.002,
            total_duration=1.0,
        )

    def test_an_accurate_complete_run_grades_well(self):
        """The caps must not demote a run that did everything it was asked to."""
        grades = EnhancedHeightMapGenerator.grade_surface_mapping_precision(
            self._stats(executed=500, planned=500, error=0.01)
        )

        assert grades == {"height": "EXCELLENT", "xy": "EXCELLENT"}

    def test_one_accurate_point_of_five_hundred_is_not_excellent(self):
        """The reason this grader exists at all."""
        grades = EnhancedHeightMapGenerator.grade_surface_mapping_precision(
            self._stats(executed=1, planned=500, error=0.01)
        )

        assert grades == {"height": "POOR", "xy": "POOR"}

    def test_a_run_that_planned_nothing_grades_poor(self):
        """Nothing was asked for, so nothing was demonstrated."""
        grades = EnhancedHeightMapGenerator.grade_surface_mapping_precision(
            self._stats(executed=0, planned=0, error=0.0)
        )

        assert grades == {"height": "POOR", "xy": "POOR"}

    def test_a_complete_but_inaccurate_run_stays_poor(self):
        """The caps only demote; they never promote."""
        grades = EnhancedHeightMapGenerator.grade_surface_mapping_precision(
            self._stats(executed=10, planned=10, error=5.0)
        )

        assert grades == {"height": "POOR", "xy": "POOR"}

    def test_an_axis_that_was_not_played_back_is_not_graded(self):
        """A machine without a pen gets no height grade, rather than a false one."""
        stats = PlaybackStats(
            planned_points=10,
            executed_points=10,
            average_position_error={"X": 0.01, "Y": 0.01},
            max_position_error={"X": 0.02, "Y": 0.02},
            average_timing_error=0.001,
            max_timing_error=0.002,
            total_duration=1.0,
        )

        grades = EnhancedHeightMapGenerator.grade_surface_mapping_precision(stats)

        assert "height" not in grades
        assert grades["xy"] == "EXCELLENT"


class TestPersistence:
    """The saved file, and reading it back."""

    def test_a_map_round_trips_through_disk(self, tmp_path):
        """
        Saving had no loader, so the format was never exercised.

        JSON has no tuples; the bounds have to come back as tuples or a reloaded
        map is not the map that was saved.
        """
        generator = _generator()
        original = generator._create_surface_map_from_points(
            _points((0.0, 0.0, 1.0), (2.0, 0.0, 1.5), (0.0, 2.0, 1.25))
        )
        path = tmp_path / "maps" / "surface.json"

        generator.save_surface_map(original, str(path))
        reloaded = generator.load_surface_map(str(path))

        assert reloaded == original

    def test_saving_creates_the_directory_it_was_given(self, tmp_path):
        """A caller should not have to make the folder first."""
        generator = _generator()
        path = tmp_path / "a" / "b" / "c" / "surface.json"

        generator.save_surface_map(
            generator._create_surface_map_from_points(_points((0.0, 0.0, 0.0))),
            str(path),
        )

        assert path.is_file()

    def test_the_saved_file_carries_the_metadata(self, tmp_path):
        """
        Whatever marks a map as simulated has to survive being written out.

        The heights `automated_grid_mapping` produces are not measurements, and
        the file is the only place a later reader can learn that.
        """
        generator = _generator()
        surface_map = SurfaceMap(
            points=_points((0.0, 0.0, 1.0)),
            bounds={"X": (0.0, 0.0), "Y": (0.0, 0.0), "Z": (1.0, 1.0)},
            statistics={"point_count": 1},
            measurement_date="2026-07-25T00:00:00",
            metadata={"simulated_probe": True, "complete": False},
        )
        path = tmp_path / "surface.json"

        generator.save_surface_map(surface_map, str(path))

        written = json.loads(path.read_text())
        assert written["metadata"]["simulated_probe"] is True
        assert written["metadata"]["complete"] is False
        assert written["points"][0]["z"] == 1.0
