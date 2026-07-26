"""
The precision analyzer is the layer that decides whether a playback went well.

It is pure arithmetic over `PlaybackStats`, so it needs no simulator - and it
had almost no coverage, which is how it came to contain the same defect the
playback code it grades did: a run that never finished was reported as a
success (L9, then L11 here).

The thresholds are load-bearing. `assess_precision` is what
`examples/basic_digitizer_demo.py` prints as its headline verdict, so a wrong
answer here is the answer a user sees.
"""
import math

import pytest

from mks_servo_can import (
    DigitizedPoint,
    DigitizedSequence,
    PlaybackStats,
    PrecisionAnalyzer,
)


def make_stats(
    planned=100,
    executed=100,
    avg_error=None,
    max_error=None,
    avg_timing=0.0,
    max_timing=0.0,
    duration=10.0,
):
    """
    Builds a PlaybackStats with sane defaults.

    Args:
        planned: Points the sequence asked for.
        executed: Points actually commanded.
        avg_error: Per-axis average position error; defaults to a perfect run.
        max_error: Per-axis worst position error; defaults to `avg_error`.
        avg_timing: Mean timing error in seconds.
        max_timing: Worst timing error in seconds.
        duration: Total playback duration in seconds.

    Returns:
        The PlaybackStats.
    """
    if avg_error is None:
        avg_error = {"X": 0.0, "Y": 0.0}
    if max_error is None:
        max_error = dict(avg_error)
    return PlaybackStats(
        planned_points=planned,
        executed_points=executed,
        average_position_error=avg_error,
        max_position_error=max_error,
        average_timing_error=avg_timing,
        max_timing_error=max_timing,
        total_duration=duration,
    )


def make_sequence(points, axes=("X", "Y")):
    """
    Builds a DigitizedSequence from a list of per-axis position dicts.

    Args:
        points: Sequence of `{axis: position}` mappings, one per point.
        axes: Axis names the sequence declares.

    Returns:
        The DigitizedSequence, timestamped at 10 Hz.
    """
    return DigitizedSequence(
        points=[
            DigitizedPoint(timestamp=i * 0.1, positions=dict(p))
            for i, p in enumerate(points)
        ],
        axis_names=list(axes),
        axis_configs={a: {} for a in axes},
        recording_date="2026-07-25T00:00:00",
        recording_duration=max(len(points) - 1, 0) * 0.1,
        sample_rate=10.0,
    )


class TestAssessPrecisionGrading:
    """The four grades, and the boundaries between them."""

    @pytest.mark.parametrize(
        ("avg", "mx", "timing", "expected"),
        [
            (0.05, 0.2, 0.02, "EXCELLENT"),
            # Each threshold is inclusive: exactly on the line still passes.
            (0.1, 0.5, 0.05, "EXCELLENT"),
            # A hair over any single one of them drops a grade.
            (0.1001, 0.5, 0.05, "GOOD"),
            (0.1, 0.5001, 0.05, "GOOD"),
            (0.1, 0.5, 0.0501, "GOOD"),
            (0.5, 2.0, 0.1, "GOOD"),
            (0.5001, 2.0, 0.1, "FAIR"),
            (1.0, 5.0, 0.2, "FAIR"),
            (1.0001, 5.0, 0.2, "POOR"),
            (100.0, 500.0, 10.0, "POOR"),
        ],
    )
    def test_grade_boundaries(self, avg, mx, timing, expected):
        """Every threshold is inclusive, and any one of the three can demote."""
        stats = make_stats(
            avg_error={"X": avg},
            max_error={"X": mx},
            avg_timing=timing,
        )
        assert PrecisionAnalyzer.assess_precision(stats) == expected

    def test_the_worst_axis_decides_the_max_error(self):
        """One bad axis must not be averaged away by good ones."""
        stats = make_stats(
            avg_error={"X": 0.05, "Y": 0.05},
            max_error={"X": 0.2, "Y": 4.0},  # FAIR territory
            avg_timing=0.0,
        )
        assert PrecisionAnalyzer.assess_precision(stats) == "FAIR"

    def test_no_position_data_is_not_a_pass(self):
        """An empty error table means nothing was measured, not that it was perfect."""
        stats = make_stats(avg_error={}, max_error={})
        assert PrecisionAnalyzer.assess_precision(stats) == "POOR"


class TestAssessPrecisionAccountsForCompleteness:
    """
    A playback that stopped early must not be graded on the part that ran.

    This is L9's shape one layer up. `playback_sequence` could cut itself short
    and report success; the analyzer would then compute error statistics over
    the handful of points that were executed, find them tiny, and print
    EXCELLENT. The error figures are perfectly true and completely misleading -
    they describe only the part of the job that happened.
    """

    def test_a_playback_that_barely_ran_is_not_excellent(self):
        """The headline case: 1 point of 500, executed perfectly."""
        stats = make_stats(planned=500, executed=1)
        assert PrecisionAnalyzer.assess_precision(stats) == "POOR"

    def test_a_playback_that_stopped_halfway_is_not_excellent(self):
        stats = make_stats(planned=100, executed=50)
        assert PrecisionAnalyzer.assess_precision(stats) == "POOR"

    def test_a_nearly_complete_playback_is_capped_not_condemned(self):
        """
        Missing one point of a thousand is a blemish, not a failure.

        The cap has to be graded, or the check is so harsh that the obvious
        response is to remove it.
        """
        stats = make_stats(planned=1000, executed=999)
        assert PrecisionAnalyzer.assess_precision(stats) == "GOOD"

    def test_a_mostly_complete_playback_is_fair(self):
        stats = make_stats(planned=100, executed=95)
        assert PrecisionAnalyzer.assess_precision(stats) == "FAIR"

    def test_completeness_only_caps_and_never_promotes(self):
        """A complete run with terrible errors is still POOR."""
        stats = make_stats(
            planned=10,
            executed=10,
            avg_error={"X": 50.0},
            max_error={"X": 100.0},
            avg_timing=5.0,
        )
        assert PrecisionAnalyzer.assess_precision(stats) == "POOR"

    def test_nothing_planned_is_not_a_perfect_score(self):
        """
        An empty sequence executed 'completely' has proved nothing.

        Note the error tables are populated with zeros rather than left empty:
        that is what `playback_sequence` actually produces for an empty
        sequence, because it seeds `position_errors` from `axis_names` and maps
        an empty list of errors to 0.0. Written with empty dicts instead, this
        test passes through the "nothing was measured" branch higher up and
        never reaches the completeness check at all - which is exactly how it
        was first written, and it survived the mutation that deletes the guard.
        """
        stats = make_stats(
            planned=0, executed=0, avg_error={"X": 0.0}, max_error={"X": 0.0}
        )
        assert PrecisionAnalyzer.assess_precision(stats) == "POOR"


class TestPathComplexity:
    """The 0-1 complexity score."""

    def test_too_few_points_is_zero(self):
        assert PrecisionAnalyzer.calculate_path_complexity(make_sequence([])) == 0.0
        assert (
            PrecisionAnalyzer.calculate_path_complexity(make_sequence([{"X": 0.0}]))
            == 0.0
        )

    def test_a_stationary_path_is_zero(self):
        sequence = make_sequence([{"X": 1.0, "Y": 1.0}] * 5)
        assert PrecisionAnalyzer.calculate_path_complexity(sequence) == 0.0

    def test_the_score_stays_within_bounds(self):
        """However far the axes move, the score is a fraction."""
        sequence = make_sequence(
            [{"X": i * 1000.0, "Y": i * 1000.0} for i in range(10)]
        )
        score = PrecisionAnalyzer.calculate_path_complexity(sequence)
        assert 0.0 <= score <= 1.0

    def test_more_movement_scores_higher(self):
        gentle = make_sequence([{"X": i * 0.01, "Y": 0.0} for i in range(10)])
        vigorous = make_sequence([{"X": i * 5.0, "Y": i * 5.0} for i in range(10)])
        assert PrecisionAnalyzer.calculate_path_complexity(
            vigorous
        ) > PrecisionAnalyzer.calculate_path_complexity(gentle)

    def test_an_axis_missing_from_a_point_does_not_crash(self):
        """Points need not carry every declared axis."""
        sequence = make_sequence([{"X": 0.0}, {"X": 1.0, "Y": 2.0}, {"Y": 3.0}])
        score = PrecisionAnalyzer.calculate_path_complexity(sequence)
        assert 0.0 <= score <= 1.0


class TestRepeatability:
    """Statistics across runs."""

    def test_no_runs_is_an_error_not_a_zero(self):
        with pytest.raises(ValueError, match="No statistics"):
            PrecisionAnalyzer.analyze_repeatability([])

    def test_a_single_run_has_no_spread(self):
        """One sample has no standard deviation; it must not be a crash either."""
        result = PrecisionAnalyzer.analyze_repeatability(
            [make_stats(avg_error={"X": 0.4}, avg_timing=0.02)]
        )
        assert result.run_count == 1
        assert result.mean_position_error["X"] == pytest.approx(0.4)
        assert result.std_position_error["X"] == 0.0
        assert result.std_timing_error == 0.0

    def test_the_spread_is_the_sample_standard_deviation(self):
        """n-1 in the denominator, not n."""
        runs = [
            make_stats(avg_error={"X": 1.0}, avg_timing=0.01),
            make_stats(avg_error={"X": 3.0}, avg_timing=0.03),
        ]
        result = PrecisionAnalyzer.analyze_repeatability(runs)

        assert result.mean_position_error["X"] == pytest.approx(2.0)
        # Sample sd of (1, 3) is sqrt(2); the population sd would be 1.0.
        assert result.std_position_error["X"] == pytest.approx(math.sqrt(2.0))
        assert result.mean_timing_error == pytest.approx(0.02)
        assert result.std_timing_error == pytest.approx(math.sqrt(2.0) * 0.01)

    def test_identical_runs_have_zero_spread(self):
        runs = [make_stats(avg_error={"X": 0.5}, avg_timing=0.01) for _ in range(4)]
        result = PrecisionAnalyzer.analyze_repeatability(runs)
        assert result.std_position_error["X"] == pytest.approx(0.0)
        assert result.std_timing_error == pytest.approx(0.0)

    def test_the_confidence_level_is_carried_through(self):
        result = PrecisionAnalyzer.analyze_repeatability(
            [make_stats()], confidence_level=0.99
        )
        assert result.confidence_interval == 0.99


class TestPerformanceReport:
    """The assembled report."""

    def test_the_report_describes_the_run(self):
        stats = make_stats(
            planned=10,
            executed=10,
            avg_error={"X": 0.05},
            max_error={"X": 0.2},
            avg_timing=0.02,
            max_timing=0.08,
        )
        sequence = make_sequence([{"X": float(i)} for i in range(10)], axes=("X",))

        report = PrecisionAnalyzer.generate_performance_report(stats, sequence)

        assert report["overall_assessment"] == "EXCELLENT"
        assert report["execution_success_rate"] == pytest.approx(1.0)
        # Seconds in, milliseconds out.
        assert report["timing_performance"]["average_error_ms"] == pytest.approx(20.0)
        assert report["timing_performance"]["max_error_ms"] == pytest.approx(80.0)
        assert report["position_performance"]["X"]["average_error"] == pytest.approx(0.05)
        assert report["sequence_info"]["total_points"] == 10
        assert report["sequence_info"]["axes"] == ["X"]
        assert "repeatability" not in report

    def test_an_empty_sequence_does_not_divide_by_zero(self):
        """
        A recording that captured nothing must not crash the report.

        `execution_success_rate` is executed/planned, and planned is
        `len(sequence.points)`. Nothing rejects an empty sequence on the way in,
        so this was a ZeroDivisionError from a legitimate input.
        """
        stats = make_stats(
            planned=0, executed=0, avg_error={"X": 0.0}, max_error={"X": 0.0}
        )
        report = PrecisionAnalyzer.generate_performance_report(
            stats, make_sequence([], axes=("X",))
        )

        assert report["execution_success_rate"] == 0.0
        assert report["overall_assessment"] == "POOR"

    def test_an_incomplete_run_reports_both_the_rate_and_a_capped_assessment(self):
        """The two must agree; an EXCELLENT verdict beside a 2% rate is the bug."""
        stats = make_stats(planned=500, executed=10)
        sequence = make_sequence([{"X": float(i)} for i in range(500)], axes=("X",))

        report = PrecisionAnalyzer.generate_performance_report(stats, sequence)

        assert report["execution_success_rate"] == pytest.approx(0.02)
        assert report["overall_assessment"] == "POOR"

    def test_repeatability_is_included_when_supplied(self):
        stats = make_stats(avg_error={"X": 0.05}, max_error={"X": 0.2})
        sequence = make_sequence([{"X": float(i)} for i in range(5)], axes=("X",))
        repeatability = PrecisionAnalyzer.analyze_repeatability([stats, stats])

        report = PrecisionAnalyzer.generate_performance_report(
            stats, sequence, repeatability
        )

        assert report["repeatability"]["run_count"] == 2
        assert report["repeatability"]["confidence_level"] == 0.95
