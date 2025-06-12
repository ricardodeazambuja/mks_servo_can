"""
Surface Mapping Module for Motor Digitizer

This module provides enhanced height mapping capabilities that extend
the base MotorDigitizer with surface profiling and analysis features.
"""

import asyncio
import logging
import math
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass

from .base_digitizer import MotorDigitizer
from .data_structures import DigitizedSequence, PlaybackStats

logger = logging.getLogger("SurfaceMapping")


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
        print(f"\n" + "="*70)
        print(f"🗺️  INTERACTIVE HEIGHT MAPPING")
        print(f"="*70)
        print(f"Instructions:")
        print(f"1. Manually move the pen to touch surface points")
        print(f"2. Recording will capture X, Y, and pen positions")
        print(f"3. Press Enter when finished mapping")
        print(f"4. System will analyze and create surface map")
        print(f"-"*70)
        
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
        print(f"\n" + "="*70)
        print(f"🤖 AUTOMATED GRID MAPPING")
        print(f"="*70)
        
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
        print(f"-"*70)
        
        surface_points = []
        point_count = 0
        
        try:
            for y in y_points:
                # Alternate X direction for efficient serpentine pattern
                x_sequence = x_points if len(surface_points) % 2 == 0 else reversed(x_points)
                
                for x in x_sequence:
                    point_count += 1
                    print(f"Measuring point {point_count}/{total_points}: X={x:.1f}, Y={y:.1f}")
                    
                    # Move to X,Y position
                    await self.controller.move_all_to_positions_abs_user({
                        "X": x,
                        "Y": y,
                        "Pen": 0.0  # Start at zero height
                    })
                    await self.controller.wait_for_all_axes()
                    
                    # Probe down to find surface
                    surface_z = await self._probe_surface_height(pen_probe_depth)
                    
                    # Record surface point
                    surface_point = SurfacePoint(
                        x=x,
                        y=y, 
                        z=surface_z,
                        timestamp=point_count,  # Use point index as timestamp
                        metadata={"probe_depth": pen_probe_depth}
                    )
                    surface_points.append(surface_point)
                    
                    # Lift pen slightly
                    await self.controller.move_all_to_positions_abs_user({"Pen": surface_z + 2.0})
                    
        except KeyboardInterrupt:
            print(f"\n⚠️  Grid mapping interrupted by user")
        except Exception as e:
            logger.error(f"Error during automated mapping: {e}")
            
        # Create surface map
        surface_map = self._create_surface_map_from_points(surface_points)
        self.current_surface_map = surface_map
        
        print(f"\n✅ GRID MAPPING COMPLETE")
        print(f"   Points measured: {len(surface_points)}")
        self._display_surface_statistics(surface_map)
        
        return surface_map
    
    async def _probe_surface_height(self, max_depth: float) -> float:
        """
        Probe downward to find surface height.
        
        This is a simplified implementation - in a real system you'd use
        force sensing or contact detection.
        
        Args:
            max_depth: Maximum depth to probe
            
        Returns:
            Detected surface height
        """
        # For simulation/demo purposes, return a varying height
        # In real implementation, this would probe until contact is detected
        current_pos = await self.axes["Pen"].get_current_position_user()
        
        # Simulate probing with some artificial surface variation
        import random
        surface_variation = random.uniform(-1.0, 1.0)  # ±1mm variation
        detected_height = current_pos - max_depth + surface_variation
        
        # Move to detected surface height
        await self.axes["Pen"].move_to_position_abs_user(detected_height)
        await self.axes["Pen"].wait_for_move_complete()
        
        return detected_height
    
    def _sequence_to_surface_map(self, sequence: DigitizedSequence) -> SurfaceMap:
        """Convert a recorded sequence to a surface map"""
        if not sequence:
            raise ValueError("No sequence provided for surface mapping")
            
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
        
        return self._create_surface_map_from_points(surface_points)
    
    def _create_surface_map_from_points(self, points: List[SurfacePoint]) -> SurfaceMap:
        """Create a SurfaceMap from a list of SurfacePoints"""
        if not points:
            return SurfaceMap(points=[], bounds={}, statistics={}, measurement_date="")
            
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
        print(f"\n📊 SURFACE MAP STATISTICS")
        print(f"="*70)
        print(f"Points measured: {surface_map.statistics['point_count']}")
        print(f"Surface bounds:")
        for axis, (min_val, max_val) in surface_map.bounds.items():
            print(f"  {axis}: {min_val:.2f} to {max_val:.2f} ({max_val-min_val:.2f} range)")
        print(f"Height statistics:")
        print(f"  Z range: {surface_map.statistics['z_range']:.3f}")
        print(f"  Z mean: {surface_map.statistics['z_mean']:.3f}")
        print(f"  Z std dev: {surface_map.statistics['z_std_dev']:.3f}")
        print(f"  Estimated area: {surface_map.statistics['surface_area']:.1f}")
    
    def save_surface_map(self, surface_map: SurfaceMap, filepath: str) -> None:
        """Save surface map to JSON file"""
        import json
        from pathlib import Path
        from dataclasses import asdict
        
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
        print(f"\n" + "="*70)
        print(f"🎯 PRECISION TEST FROM RECORDING")
        print(f"="*70)
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
    
    def _analyze_surface_mapping_precision(self, stats: PlaybackStats, 
                                         sequence: DigitizedSequence) -> None:
        """Analyze precision specifically for surface mapping operations"""
        print(f"\n🗺️  SURFACE MAPPING PRECISION ANALYSIS")
        print(f"="*70)
        
        # Calculate surface-specific metrics
        if "Pen" in stats.average_position_error:
            pen_error = stats.average_position_error["Pen"]
            pen_max_error = stats.max_position_error["Pen"]
            
            print(f"Height measurement precision:")
            print(f"  Average error: {pen_error:.3f} units")
            print(f"  Maximum error: {pen_max_error:.3f} units")
            
            # Assess height measurement quality
            if pen_error < 0.05:
                assessment = "EXCELLENT"
            elif pen_error < 0.1:
                assessment = "GOOD"
            elif pen_error < 0.2:
                assessment = "FAIR"
            else:
                assessment = "POOR"
                
            print(f"  Height precision: {assessment}")
        
        # Calculate XY positioning precision
        xy_errors = []
        for axis in ["X", "Y"]:
            if axis in stats.average_position_error:
                xy_errors.append(stats.average_position_error[axis])
        
        if xy_errors:
            avg_xy_error = sum(xy_errors) / len(xy_errors)
            print(f"XY positioning precision:")
            print(f"  Average XY error: {avg_xy_error:.3f} units")
            
            if avg_xy_error < 0.1:
                xy_assessment = "EXCELLENT"
            elif avg_xy_error < 0.5:
                xy_assessment = "GOOD"
            elif avg_xy_error < 1.0:
                xy_assessment = "FAIR" 
            else:
                xy_assessment = "POOR"
                
            print(f"  XY precision: {xy_assessment}")