"""
Surface Mapping Module for Motor Digitizer

This module provides enhanced height mapping capabilities that extend
the base MotorDigitizer with surface profiling and analysis features.
"""

import logging
import math
import random
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from ..exceptions import MKSServoError
from .base_digitizer import MotorDigitizer
from .data_structures import DigitizedSequence, PlaybackStats

logger = logging.getLogger("SurfaceMapping")

# The statistics every surface map carries, so that a map of no points has the
# same shape as any other and nothing downstream has to guess which keys exist.
EMPTY_STATISTICS = {
    "point_count": 0,
    "z_range": 0.0,
    "z_mean": 0.0,
    "surface_area": 0.0,
    "z_std_dev": 0.0,
}


@dataclass
class SurfacePoint:
    """A point on a measured surface with position and height"""
    x: float
    y: float
    z: float
    timestamp: float
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class SurfaceMap:
    """Collection of surface points with analysis data"""
    points: List[SurfacePoint]
    bounds: Dict[str, Tuple[float, float]]  # axis -> (min, max)
    statistics: Dict[str, float]
    measurement_date: str
    metadata: Optional[Dict[str, Any]] = None


class EnhancedHeightMapGenerator(MotorDigitizer):
    """
    Enhanced height mapping generator that inherits from MotorDigitizer.
    
    Provides specialized functionality for surface mapping, height measurement,
    and precision testing of surface profiling operations.
    """

    def __init__(self, can_interface):
        """Initialize enhanced height map generator"""
        super().__init__(can_interface)
        self.plotter_config = {}
        self.current_surface_map: Optional[SurfaceMap] = None

    async def setup_standard_plotter(self, motor_can_ids: List[int],
                                   pen_motor_config: Optional[Dict[str, Any]] = None) -> None:
        """
        Setup standard XYZ plotter configuration.
        
        Args:
            motor_can_ids: List of [x_motor_id, y_motor_id, pen_motor_id]
            pen_motor_config: Optional pen motor configuration
        """
        if len(motor_can_ids) != 3:
            raise ValueError("Standard plotter requires exactly 3 motor CAN IDs: [X, Y, Pen]")

        # Import utility functions from this module
        from .utils import create_linear_axis

        # Default pen configuration
        if pen_motor_config is None:
            pen_motor_config = {"pitch_mm": 8.0, "gear_ratio": 1.0}

        # Create axes
        x_axis = create_linear_axis(self.can_interface, motor_can_ids[0], "X", 40.0)
        y_axis = create_linear_axis(self.can_interface, motor_can_ids[1], "Y", 40.0)
        pen_axis = create_linear_axis(
            self.can_interface,
            motor_can_ids[2],
            "Pen",
            pen_motor_config["pitch_mm"],
            pen_motor_config["gear_ratio"]
        )

        await self.add_axis(x_axis)
        await self.add_axis(y_axis)
        await self.add_axis(pen_axis)

        # Store plotter configuration
        self.plotter_config = {
            "x_motor_id": motor_can_ids[0],
            "y_motor_id": motor_can_ids[1],
            "pen_motor_id": motor_can_ids[2],
            "pen_config": pen_motor_config
        }

        await self.initialize_axes()
        logger.info("Standard XYZ plotter configured and initialized")

    async def interactive_height_mapping(self, save_recording: Optional[str] = None) -> SurfaceMap:
        """
        Perform interactive height mapping by recording manual movements.
        
        Args:
            save_recording: Optional filepath to save the recording sequence
            
        Returns:
            SurfaceMap with measured surface points
        """
        print("\n" + "="*70)
        print("🗺️  INTERACTIVE HEIGHT MAPPING")
        print("="*70)
        print("Instructions:")
        print("1. Manually move the pen to touch surface points")
        print("2. Recording will capture X, Y, and pen positions")
        print("3. Press Enter when finished mapping")
        print("4. System will analyze and create surface map")
        print("-"*70)

        # Start recording with higher sample rate for detailed mapping
        await self.start_recording(sample_rate=20.0, auto_velocity=True)

        # Save recording if requested
        if save_recording and self.current_sequence:
            self.save_sequence(self.current_sequence, save_recording)
            print(f"Recording saved to: {save_recording}")

        # Convert recording to surface map
        surface_map = self._sequence_to_surface_map(self.current_sequence)
        self.current_surface_map = surface_map

        self._display_surface_statistics(surface_map)
        return surface_map

    async def automated_grid_mapping(self,
                                   x_range: Tuple[float, float],
                                   y_range: Tuple[float, float],
                                   grid_spacing: float,
                                   pen_probe_depth: float = 5.0) -> SurfaceMap:
        """
        Perform automated grid-based surface mapping.
        
        Args:
            x_range: (min_x, max_x) range in user units
            y_range: (min_y, max_y) range in user units
            grid_spacing: Grid spacing in user units
            pen_probe_depth: How deep to probe with pen
            
        Returns:
            SurfaceMap with measured grid points
        """
        print("\n" + "="*70)
        print("🤖 AUTOMATED GRID MAPPING")
        print("="*70)

        # Calculate grid points
        x_points = []
        x = x_range[0]
        while x <= x_range[1]:
            x_points.append(x)
            x += grid_spacing

        y_points = []
        y = y_range[0]
        while y <= y_range[1]:
            y_points.append(y)
            y += grid_spacing

        total_points = len(x_points) * len(y_points)
        print(f"Grid: {len(x_points)} x {len(y_points)} = {total_points} points")
        print(f"Spacing: {grid_spacing} units")
        print(f"Probe depth: {pen_probe_depth} units")
        print("-"*70)

        print(
            "NOTE: heights are simulated. There is no contact detection in this "
            "build, so every point below is a plausible number rather than a "
            "measurement, and every point says so in its metadata."
        )

        surface_points = []
        point_count = 0
        interrupted = False

        try:
            for row, y in enumerate(y_points):
                # Alternate X direction for an efficient serpentine pattern.
                # This used to alternate on `len(surface_points) % 2`, the count
                # of points measured so far rather than the row index, so a grid
                # with an even number of columns went left-to-right every row
                # and the pen crossed the whole workpiece between rows.
                x_sequence = x_points if row % 2 == 0 else list(reversed(x_points))

                for x in x_sequence:
                    point_count += 1
                    print(f"Measuring point {point_count}/{total_points}: X={x:.1f}, Y={y:.1f}")

                    # Move to X,Y position
                    await self.controller.move_all_to_positions_abs_user({
                        "X": x,
                        "Y": y,
                        "Pen": 0.0  # Start at zero height
                    })
                    # `wait_for_all_axes` does not exist on MultiAxisController
                    # and never has, so the first point of every grid raised
                    # AttributeError - which the blanket `except Exception`
                    # below then swallowed.
                    await self.controller.wait_for_all_moves_to_complete()

                    # Probe down to find surface
                    surface_z = await self._probe_surface_height(pen_probe_depth)

                    # Record surface point
                    surface_point = SurfacePoint(
                        x=x,
                        y=y,
                        z=surface_z,
                        timestamp=point_count,  # Use point index as timestamp
                        metadata={
                            "probe_depth": pen_probe_depth,
                            # Travels with the point into the saved JSON: a
                            # height nobody measured must not be readable as one.
                            "simulated": True,
                            "contact_detected": False,
                        },
                    )
                    surface_points.append(surface_point)

                    # Lift pen slightly
                    await self.controller.move_all_to_positions_abs_user({"Pen": surface_z + 2.0})

        except KeyboardInterrupt:
            print("\n⚠️  Grid mapping interrupted by user")
            interrupted = True
        except (MKSServoError, OSError):
            # This used to log the error and carry on to print "GRID MAPPING
            # COMPLETE" over a partial map, which the caller could not tell from
            # a finished one - the map is returned either way.
            print("\n❌ GRID MAPPING FAILED")
            print(f"   Points measured before the failure: {len(surface_points)}")
            raise

        # Create surface map
        surface_map = self._create_surface_map_from_points(surface_points)
        surface_map.metadata = dict(surface_map.metadata or {})
        surface_map.metadata.update(
            {
                "simulated_probe": True,
                "planned_points": total_points,
                "measured_points": len(surface_points),
                "complete": not interrupted and len(surface_points) == total_points,
            }
        )
        self.current_surface_map = surface_map

        if surface_map.metadata["complete"]:
            print("\n✅ GRID MAPPING COMPLETE")
        else:
            print("\n⚠️  GRID MAPPING INCOMPLETE")
        print(f"   Points measured: {len(surface_points)}/{total_points}")
        self._display_surface_statistics(surface_map)

        return surface_map

    async def _probe_surface_height(self, max_depth: float) -> float:
        """
        Move the pen down by `max_depth` and report where it ended up.

        **This does not detect contact.** There is no force sensor and no
        endstop in the path, so there is nothing here that can tell a surface
        from empty air: the height it returns is `max_depth` below where the pen
        started, plus a millimetre of invented variation to make demo output
        look like a real workpiece. It is a stand-in for a probing routine, not
        a probing routine.

        Everything it feeds is marked `simulated` for that reason - see
        `automated_grid_mapping`. Nothing in this build should be used to decide
        where a tool may safely descend.

        Args:
            max_depth: How far below the current position to go, in user units.

        Returns:
            The height the pen was moved to.
        """
        current_pos = await self.axes["Pen"].get_current_position_user()

        surface_variation = random.uniform(-1.0, 1.0)  # +/-1mm of invented variation
        detected_height = current_pos - max_depth + surface_variation

        # Move to detected surface height
        await self.axes["Pen"].move_to_position_abs_user(detected_height)
        # `wait_for_move_complete` is not a method on Axis either; the name is
        # `wait_for_move_completion`. Nothing had ever run this far.
        await self.axes["Pen"].wait_for_move_completion()

        return detected_height

    def _sequence_to_surface_map(self, sequence: DigitizedSequence) -> SurfaceMap:
        """
        Convert a recorded sequence to a surface map.

        Args:
            sequence: The recording to convert.

        Returns:
            A `SurfaceMap` over every recorded point that carries both X and Y.

        Raises:
            ValueError: If there is no sequence, if it recorded no points, or if
                none of its points carry both an X and a Y position. The guard
                used to be `if not sequence`, which a dataclass never satisfies,
                so a recording of nothing produced an empty map that read like a
                flat surface.
        """
        if sequence is None:
            raise ValueError("No sequence provided for surface mapping")
        if not sequence.points:
            raise ValueError("The recorded sequence contains no points")

        surface_points = []

        for point in sequence.points:
            if "X" in point.positions and "Y" in point.positions:
                # Use pen position as Z, or default to 0 if no pen axis
                z_value = point.positions.get("Pen", point.positions.get("Z", 0.0))

                surface_point = SurfacePoint(
                    x=point.positions["X"],
                    y=point.positions["Y"],
                    z=z_value,
                    timestamp=point.timestamp
                )
                surface_points.append(surface_point)

        if not surface_points:
            raise ValueError(
                f"None of the {len(sequence.points)} recorded points carry both "
                "an X and a Y position, so there is no surface to map. Recorded "
                f"axes: {sorted(sequence.axis_names)}"
            )

        return self._create_surface_map_from_points(surface_points)

    def _create_surface_map_from_points(self, points: List[SurfacePoint]) -> SurfaceMap:
        """
        Create a SurfaceMap from a list of SurfacePoints.

        Args:
            points: The measured points; may be empty.

        Returns:
            A `SurfaceMap`. A map of no points still carries the full set of
            statistics, all zero. It used to carry `{}`, so anything that read
            the statistics of an empty map - including this module's own
            `_display_surface_statistics` - raised `KeyError` instead.
        """
        if not points:
            return SurfaceMap(
                points=[],
                bounds={},
                statistics=dict(EMPTY_STATISTICS),
                measurement_date="",
            )

        # Calculate bounds
        x_coords = [p.x for p in points]
        y_coords = [p.y for p in points]
        z_coords = [p.z for p in points]

        bounds = {
            "X": (min(x_coords), max(x_coords)),
            "Y": (min(y_coords), max(y_coords)),
            "Z": (min(z_coords), max(z_coords))
        }

        # Calculate statistics
        statistics = {
            "point_count": len(points),
            "z_range": max(z_coords) - min(z_coords),
            "z_mean": sum(z_coords) / len(z_coords),
            "surface_area": self._estimate_surface_area(points),
            "z_std_dev": self._calculate_std_dev(z_coords)
        }

        from datetime import datetime

        return SurfaceMap(
            points=points,
            bounds=bounds,
            statistics=statistics,
            measurement_date=datetime.now().isoformat(),
            metadata={"generator": "EnhancedHeightMapGenerator"}
        )

    def _estimate_surface_area(self, points: List[SurfacePoint]) -> float:
        """Estimate surface area from points (simplified calculation)"""
        if len(points) < 3:
            return 0.0

        # Simple rectangular area estimation
        x_coords = [p.x for p in points]
        y_coords = [p.y for p in points]

        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)

        return x_range * y_range

    def _calculate_std_dev(self, values: List[float]) -> float:
        """Calculate standard deviation"""
        if len(values) < 2:
            return 0.0

        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
        return math.sqrt(variance)

    def _display_surface_statistics(self, surface_map: SurfaceMap) -> None:
        """Display surface mapping statistics"""
        print("\n📊 SURFACE MAP STATISTICS")
        print("="*70)
        if not surface_map.points:
            print("No points were measured; there is nothing to summarise.")
            return
        print(f"Points measured: {surface_map.statistics['point_count']}")
        print("Surface bounds:")
        for axis, (min_val, max_val) in surface_map.bounds.items():
            print(f"  {axis}: {min_val:.2f} to {max_val:.2f} ({max_val-min_val:.2f} range)")
        print("Height statistics:")
        print(f"  Z range: {surface_map.statistics['z_range']:.3f}")
        print(f"  Z mean: {surface_map.statistics['z_mean']:.3f}")
        print(f"  Z std dev: {surface_map.statistics['z_std_dev']:.3f}")
        print(f"  Estimated area: {surface_map.statistics['surface_area']:.1f}")

    def save_surface_map(self, surface_map: SurfaceMap, filepath: str) -> None:
        """
        Save a surface map to a JSON file.

        Args:
            surface_map: The map to write.
            filepath: Destination path; parent directories are created.
        """
        import json
        from dataclasses import asdict
        from pathlib import Path

        data = {
            "points": [asdict(point) for point in surface_map.points],
            "bounds": surface_map.bounds,
            "statistics": surface_map.statistics,
            "measurement_date": surface_map.measurement_date,
            "metadata": surface_map.metadata or {}
        }

        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"Surface map saved to {filepath}")

    @staticmethod
    def load_surface_map(filepath: str) -> SurfaceMap:
        """
        Read back a surface map written by `save_surface_map`.

        Saving without a loader meant the file format had nothing that
        exercised it, so a map could only ever be checked by eye. JSON has no
        tuples, so the bounds come back as lists and are restored here.

        Args:
            filepath: The file to read.

        Returns:
            The `SurfaceMap` that was saved.
        """
        import json
        from pathlib import Path

        data = json.loads(Path(filepath).read_text())
        return SurfaceMap(
            points=[SurfacePoint(**point) for point in data["points"]],
            bounds={
                axis: (float(low), float(high))
                for axis, (low, high) in data["bounds"].items()
            },
            statistics=data["statistics"],
            measurement_date=data["measurement_date"],
            metadata=data.get("metadata") or None,
        )

    async def precision_test_from_recording(self, recording_filepath: str,
                                          speed_factor: float = 1.0) -> PlaybackStats:
        """
        Test precision by playing back a recorded surface mapping sequence.
        
        Args:
            recording_filepath: Path to saved recording file
            speed_factor: Playback speed factor
            
        Returns:
            PlaybackStats from precision testing
        """
        print("\n" + "="*70)
        print("🎯 PRECISION TEST FROM RECORDING")
        print("="*70)
        print(f"Loading recording: {recording_filepath}")

        # Load the recorded sequence
        sequence = self.load_sequence(recording_filepath)

        # Play back with precision testing enabled
        stats = await self.playback_sequence(
            sequence=sequence,
            speed_factor=speed_factor,
            precision_test=True
        )

        if stats:
            # Additional analysis specific to surface mapping
            self._analyze_surface_mapping_precision(stats, sequence)

        return stats

    @staticmethod
    def grade_surface_mapping_precision(stats: PlaybackStats) -> Dict[str, str]:
        """
        Grade the height and XY accuracy of a surface-mapping playback.

        Both grades are capped by how much of the sequence actually ran, the
        same way `PrecisionAnalyzer.assess_precision` is. This used to read the
        position errors alone, so a playback that managed one point of five
        hundred - accurately - graded EXCELLENT on both counts. That is L12's
        defect in a second place: precise figures about a job that did not
        happen. The caps only demote; a complete run with poor errors stays
        poor.

        Args:
            stats: The statistics from a playback run with `precision_test=True`.

        Returns:
            A mapping with a `"height"` grade when the Pen axis was played back
            and an `"xy"` grade when X or Y was; either may be absent.
        """
        from .precision_analyzer import PrecisionAnalyzer

        grades: Dict[str, str] = {}

        if "Pen" in stats.average_position_error:
            pen_error = stats.average_position_error["Pen"]
            if pen_error < 0.05:
                grade = "EXCELLENT"
            elif pen_error < 0.1:
                grade = "GOOD"
            elif pen_error < 0.2:
                grade = "FAIR"
            else:
                grade = "POOR"
            grades["height"] = PrecisionAnalyzer._cap_by_completeness(grade, stats)

        xy_errors = [
            stats.average_position_error[axis]
            for axis in ("X", "Y")
            if axis in stats.average_position_error
        ]
        if xy_errors:
            avg_xy_error = sum(xy_errors) / len(xy_errors)
            if avg_xy_error < 0.1:
                grade = "EXCELLENT"
            elif avg_xy_error < 0.5:
                grade = "GOOD"
            elif avg_xy_error < 1.0:
                grade = "FAIR"
            else:
                grade = "POOR"
            grades["xy"] = PrecisionAnalyzer._cap_by_completeness(grade, stats)

        return grades

    def _analyze_surface_mapping_precision(self, stats: PlaybackStats,
                                         sequence: DigitizedSequence) -> None:
        """Analyze precision specifically for surface mapping operations"""
        print("\n🗺️  SURFACE MAPPING PRECISION ANALYSIS")
        print("="*70)
        print(f"Points executed: {stats.executed_points}/{stats.planned_points}")

        grades = self.grade_surface_mapping_precision(stats)

        if "height" in grades:
            print("Height measurement precision:")
            print(f"  Average error: {stats.average_position_error['Pen']:.3f} units")
            print(f"  Maximum error: {stats.max_position_error['Pen']:.3f} units")
            print(f"  Height precision: {grades['height']}")

        if "xy" in grades:
            xy_errors = [
                stats.average_position_error[axis]
                for axis in ("X", "Y")
                if axis in stats.average_position_error
            ]
            print("XY positioning precision:")
            print(f"  Average XY error: {sum(xy_errors) / len(xy_errors):.3f} units")
            print(f"  XY precision: {grades['xy']}")
