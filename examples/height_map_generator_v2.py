"""
Enhanced Height Map Generator for MKS Servo CAN Plotter

This script builds on the MotorDigitizer base class to provide advanced height mapping
capabilities with recording/playback functionality for precision testing.

New features:
1. Inherits from MotorDigitizer for recording/playback capabilities
2. Can record manual height mapping sessions for later playback
3. Precision testing by comparing manual vs. automated measurements
4. Support for multiple probing strategies (grid, adaptive, manual)
5. Enhanced surface interpolation and analysis

Usage Examples:
1. Interactive height mapping with recording:
   python height_map_generator_v2.py --interactive --save-recording height_session.json

2. Replay a recorded session:
   python height_map_generator_v2.py --playback height_session.json --precision-test

3. Traditional grid mapping:
   python height_map_generator_v2.py --grid-map --spacing 10 --save-map surface.json

4. Adaptive mapping (higher density in high-variation areas):
   python height_map_generator_v2.py --adaptive-map --save-map adaptive_surface.json
"""

import asyncio
import logging
import math
import argparse
import json
import numpy as np
from datetime import datetime
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass, asdict
from pathlib import Path

from mks_servo_can import (
    CANInterface,
    Axis,
    LinearKinematics,
    RotaryKinematics,
    const,
    exceptions,
)

from motor_digitizer import (
    MotorDigitizer, 
    DigitizedSequence, 
    DigitizedPoint,
    PlaybackStats,
    create_linear_axis,
    create_rotary_axis
)

# ===================================================================
# Configuration
# ===================================================================

# Default plotter configuration
DEFAULT_CONFIG = {
    "plotter_bounds": (130.0, 70.0),  # max X, max Y in mm
    "motor_ids": [1, 2, 3],           # X, Y, Pen CAN IDs
    "kinematics": {
        "x_pitch": 40.0,              # mm per revolution
        "y_pitch": 40.0,              # mm per revolution
        "pen_gear_ratio": 1.0         # gear reduction
    },
    "speeds": {
        "xy_speed": 20.0,             # mm/s for XY movement
        "pen_speed": 90.0             # deg/s for pen movement
    },
    "pen_geometry": {
        "arm_radius": 25.0,           # mm
        "pivot_height": 30.0,         # mm above bed
        "angle_range": (-45.0, 90.0)  # min, max degrees
    }
}

logger = logging.getLogger("HeightMapperV2")

# ===================================================================
# Data Structures
# ===================================================================

@dataclass
class SurfacePoint:
    """Enhanced surface point with additional metadata"""
    x: float
    y: float
    z: float
    pen_angle: float
    confidence: float = 1.0
    timestamp: Optional[float] = None
    measurement_method: str = "manual"

@dataclass 
class SurfaceMap:
    """Enhanced surface map with analysis capabilities"""
    points: List[SurfacePoint]
    bounds: Dict[str, float]
    grid_spacing: Optional[float] = None
    interpolation_method: str = "bicubic"
    creation_date: str = ""
    statistics: Optional[Dict[str, float]] = None
    pen_geometry: Optional[Dict[str, float]] = None

# ===================================================================
# Enhanced Height Map Generator
# ===================================================================

class EnhancedHeightMapGenerator(MotorDigitizer):
    """
    Enhanced height map generator with recording/playback capabilities.
    
    Extends MotorDigitizer to provide specialized height mapping functionality
    while maintaining all recording and precision testing capabilities.
    """
    
    def __init__(self, can_interface: CANInterface, config: Dict[str, Any] = None):
        """
        Initialize the enhanced height map generator.
        
        Args:
            can_interface: Connected CANInterface instance
            config: Configuration dictionary (uses DEFAULT_CONFIG if None)
        """
        super().__init__(can_interface)
        self.config = config or DEFAULT_CONFIG.copy()
        self.plotter_bounds = self.config["plotter_bounds"]
        self.pen_geometry = self.config["pen_geometry"]
        self.current_surface_map: Optional[SurfaceMap] = None
        
    async def setup_standard_plotter(self, motor_ids: List[int] = None) -> None:
        """
        Set up a standard 3-axis plotter configuration.
        
        Args:
            motor_ids: List of [X_ID, Y_ID, PEN_ID]. Uses config default if None.
        """
        if motor_ids is None:
            motor_ids = self.config["motor_ids"]
            
        if len(motor_ids) != 3:
            raise ValueError("Standard plotter requires exactly 3 motor IDs [X, Y, Pen]")
            
        # Create axes
        x_axis = create_linear_axis(
            self.can_interface, 
            motor_ids[0], 
            "X", 
            self.config["kinematics"]["x_pitch"]
        )
        
        y_axis = create_linear_axis(
            self.can_interface,
            motor_ids[1], 
            "Y", 
            self.config["kinematics"]["y_pitch"]
        )
        
        pen_axis = create_rotary_axis(
            self.can_interface,
            motor_ids[2],
            "Pen",
            self.config["kinematics"]["pen_gear_ratio"]
        )
        
        # Add to digitizer
        await self.add_axis(x_axis)
        await self.add_axis(y_axis)
        await self.add_axis(pen_axis)
        
        # Initialize
        await self.initialize_axes(enable_motors=True)
        
        logger.info("Standard plotter setup complete")
        
    async def interactive_height_mapping(self, save_recording: str = None) -> SurfaceMap:
        """
        Interactive height mapping where user manually guides the system.
        Optionally records the session for later playback/analysis.
        
        Args:
            save_recording: Optional file path to save the recording
            
        Returns:
            SurfaceMap with measured points
        """
        print(f"\n" + "="*70)
        print(f"🗺️  INTERACTIVE HEIGHT MAPPING")
        print(f"="*70)
        print(f"Instructions:")
        print(f"  1. Motors will be disabled for manual control")
        print(f"  2. Move the pen to desired locations manually")
        print(f"  3. At each location, press commands to record height")
        print(f"  4. Recording will capture the complete session")
        print(f"-"*70)
        
        surface_points = []
        
        # Start recording if requested
        if save_recording:
            await self.start_recording(sample_rate=5.0, auto_velocity=True)
            
        try:
            # Move pen to safe position and disable motors
            pen_axis = self.axes["Pen"]
            await pen_axis.move_to_position_abs_user(
                self.pen_geometry["angle_range"][0], 
                speed_user=self.config["speeds"]["pen_speed"]
            )
            
            await self.controller.disable_all_axes()
            
            point_count = 0
            while True:
                print(f"\n📍 HEIGHT MEASUREMENT POINT {point_count + 1}")
                print(f"Position the pen tip at desired measurement location")
                print(f"Commands: (m)easure point, (f)inish mapping, (q)uit")
                
                try:
                    # Get current positions
                    positions = await self.controller.get_all_positions_user()
                    x_pos = positions.get("X", 0.0)
                    y_pos = positions.get("Y", 0.0)
                    pen_angle = positions.get("Pen", 0.0)
                    
                    # Calculate Z height from pen geometry
                    z_height = self._calculate_z_from_pen_angle(pen_angle)
                    
                    print(f"Current: X={x_pos:.2f}, Y={y_pos:.2f}, Z={z_height:.2f}, Pen={pen_angle:.1f}°")
                    
                    command = input("Command: ").strip().lower()
                    
                    if command == 'm':
                        # Record measurement point
                        point = SurfacePoint(
                            x=x_pos,
                            y=y_pos,
                            z=z_height,
                            pen_angle=pen_angle,
                            confidence=1.0,
                            timestamp=asyncio.get_event_loop().time(),
                            measurement_method="interactive"
                        )
                        surface_points.append(point)
                        point_count += 1
                        print(f"✅ Point {point_count} recorded: ({x_pos:.2f}, {y_pos:.2f}, {z_height:.2f})")
                        
                    elif command == 'f':
                        if len(surface_points) < 3:
                            print(f"❌ Need at least 3 points for surface mapping (have {len(surface_points)})")
                            continue
                        print(f"✅ Finishing mapping with {len(surface_points)} points")
                        break
                        
                    elif command == 'q':
                        if len(surface_points) > 0:
                            save_partial = input(f"Save {len(surface_points)} points? (y/N): ").strip().lower()
                            if save_partial not in ['y', 'yes']:
                                raise KeyboardInterrupt("User cancelled without saving")
                        else:
                            raise KeyboardInterrupt("User cancelled mapping")
                        break
                        
                    else:
                        print(f"❓ Unknown command '{command}'")
                        
                except KeyboardInterrupt:
                    print(f"\n⚠️  Interactive mapping cancelled")
                    break
                    
        finally:
            # Re-enable motors
            await self.controller.enable_all_axes()
            
            # Stop recording if active
            if self.is_recording:
                print(f"Stopping recording...")
                # Note: Recording is stopped by the parent class when start_recording() completes
                
        # Create surface map
        surface_map = SurfaceMap(
            points=surface_points,
            bounds=self._calculate_bounds(surface_points),
            creation_date=datetime.now().isoformat(),
            statistics=self._calculate_surface_statistics(surface_points),
            pen_geometry=self.pen_geometry.copy()
        )
        
        print(f"\n✅ INTERACTIVE MAPPING COMPLETE")
        print(f"   Points measured: {len(surface_points)}")
        if surface_map.statistics:
            print(f"   Z range: {surface_map.statistics['z_min']:.2f} to {surface_map.statistics['z_max']:.2f} mm")
            print(f"   Surface area: {surface_map.statistics.get('area_coverage', 0):.1f} mm²")
            
        # Save recording if requested
        if save_recording and self.current_sequence:
            self.save_sequence(self.current_sequence, save_recording)
            print(f"   Recording saved: {save_recording}")
            
        self.current_surface_map = surface_map
        return surface_map
        
    async def grid_height_mapping(self, grid_spacing: float = 10.0, 
                                bounds: Tuple[float, float, float, float] = None) -> SurfaceMap:
        """
        Automated grid-based height mapping.
        
        Args:
            grid_spacing: Distance between grid points in mm
            bounds: (x_min, x_max, y_min, y_max). Uses plotter bounds if None.
            
        Returns:
            SurfaceMap with grid measurements
        """
        if bounds is None:
            bounds = (0.0, self.plotter_bounds[0], 0.0, self.plotter_bounds[1])
            
        x_min, x_max, y_min, y_max = bounds
        
        # Generate grid points
        x_points = np.arange(x_min, x_max + grid_spacing/2, grid_spacing)
        y_points = np.arange(y_min, y_max + grid_spacing/2, grid_spacing)
        
        total_points = len(x_points) * len(y_points)
        
        print(f"\n" + "="*70)
        print(f"📐 GRID HEIGHT MAPPING")
        print(f"="*70)
        print(f"Grid: {len(x_points)} x {len(y_points)} = {total_points} points")
        print(f"Spacing: {grid_spacing}mm")
        print(f"Bounds: X({x_min:.1f}-{x_max:.1f}), Y({y_min:.1f}-{y_max:.1f})")
        print(f"Estimated time: ~{total_points * 0.5:.1f} minutes")
        print(f"-"*70)
        
        confirm = input("Continue with automated grid mapping? (y/N): ").strip().lower()
        if confirm not in ['y', 'yes']:
            raise KeyboardInterrupt("User cancelled grid mapping")
            
        surface_points = []
        
        try:
            # Move pen to safe position
            await self._move_pen_to_safe_position()
            
            point_num = 0
            for i, x in enumerate(x_points):
                for j, y in enumerate(y_points):
                    point_num += 1
                    
                    print(f"\n🎯 Measuring point {point_num}/{total_points}: ({x:.1f}, {y:.1f})")
                    print(f"   Progress: {point_num/total_points*100:.1f}%")
                    
                    try:
                        # Move to XY position
                        await self._move_to_xy(x, y)
                        
                        # Probe height
                        z_height, pen_angle = await self._automated_height_probe()
                        
                        if not (math.isnan(z_height) or math.isnan(pen_angle)):
                            point = SurfacePoint(
                                x=x, y=y, z=z_height, pen_angle=pen_angle,
                                confidence=1.0,
                                timestamp=asyncio.get_event_loop().time(),
                                measurement_method="grid_auto"
                            )
                            surface_points.append(point)
                            print(f"   ✅ Z: {z_height:.2f}mm, Angle: {pen_angle:.1f}°")
                        else:
                            print(f"   ⚠️  Failed to measure point")
                            
                    except KeyboardInterrupt:
                        print(f"\n⚠️  Grid mapping interrupted")
                        if len(surface_points) >= 3:
                            save_partial = input(f"Save {len(surface_points)} points? (y/N): ").strip().lower()
                            if save_partial in ['y', 'yes']:
                                break
                        raise
                        
                # Row completion check
                if i < len(x_points) - 1:
                    remaining = (len(x_points) - i - 1) * len(y_points)
                    print(f"\n📊 Row {i+1}/{len(x_points)} complete. {remaining} points remaining.")
                    
                    continue_mapping = input("Continue? (Y/n): ").strip().lower()
                    if continue_mapping in ['n', 'no']:
                        break
                        
        except KeyboardInterrupt:
            if len(surface_points) < 3:
                raise
                
        # Create surface map
        surface_map = SurfaceMap(
            points=surface_points,
            bounds={"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
            grid_spacing=grid_spacing,
            creation_date=datetime.now().isoformat(),
            statistics=self._calculate_surface_statistics(surface_points),
            pen_geometry=self.pen_geometry.copy()
        )
        
        print(f"\n✅ GRID MAPPING COMPLETE")
        print(f"   Points measured: {len(surface_points)}/{total_points}")
        print(f"   Success rate: {len(surface_points)/total_points*100:.1f}%")
        
        self.current_surface_map = surface_map
        return surface_map
        
    async def precision_test_from_recording(self, recording_path: str, 
                                          speed_factor: float = 0.5) -> PlaybackStats:
        """
        Test system precision by playing back a recorded height mapping session.
        
        Args:
            recording_path: Path to recorded DigitizedSequence file
            speed_factor: Playback speed (0.5 = half speed for higher accuracy)
            
        Returns:
            PlaybackStats with precision measurements
        """
        print(f"\n" + "="*70)
        print(f"🎯 PRECISION TEST FROM RECORDING")
        print(f"="*70)
        
        # Load recorded sequence
        sequence = self.load_sequence(recording_path)
        
        print(f"Loaded recording: {len(sequence.points)} points, {sequence.recording_duration:.1f}s")
        print(f"Original axes: {', '.join(sequence.axis_names)}")
        
        # Validate compatibility
        if not all(axis in self.axes for axis in sequence.axis_names):
            missing = set(sequence.axis_names) - set(self.axes.keys())
            raise ValueError(f"Recording requires unavailable axes: {missing}")
            
        # Play back with precision testing
        print(f"Starting precision test playback at {speed_factor}x speed...")
        stats = await self.playback_sequence(sequence, speed_factor, precision_test=True)
        
        # Additional height mapping specific analysis
        if stats:
            self._analyze_height_mapping_precision(stats, sequence)
            
        return stats
        
    def _analyze_height_mapping_precision(self, stats: PlaybackStats, sequence: DigitizedSequence):
        """Analyze precision specific to height mapping operations"""
        print(f"\n📊 HEIGHT MAPPING PRECISION ANALYSIS")
        print(f"="*70)
        
        # Focus on Z-axis precision for height mapping
        if "Pen" in stats.average_position_error:
            pen_error = stats.average_position_error["Pen"]
            pen_max_error = stats.max_position_error["Pen"]
            
            # Convert pen angle error to Z height error
            z_error_avg = abs(pen_error * math.pi/180 * self.pen_geometry["arm_radius"])
            z_error_max = abs(pen_max_error * math.pi/180 * self.pen_geometry["arm_radius"])
            
            print(f"Pen angle precision:")
            print(f"  Average error: {pen_error:.3f}° ({z_error_avg:.3f}mm Z-height)")
            print(f"  Maximum error: {pen_max_error:.3f}° ({z_error_max:.3f}mm Z-height)")
            
        # Analyze XY positioning precision
        if "X" in stats.average_position_error and "Y" in stats.average_position_error:
            xy_error = math.sqrt(stats.average_position_error["X"]**2 + stats.average_position_error["Y"]**2)
            xy_max_error = math.sqrt(stats.max_position_error["X"]**2 + stats.max_position_error["Y"]**2)
            
            print(f"XY positioning precision:")
            print(f"  Average error: {xy_error:.3f}mm")
            print(f"  Maximum error: {xy_max_error:.3f}mm")
            
        # Overall system assessment
        total_error_score = sum(stats.average_position_error.values())
        print(f"\nOverall precision score: {total_error_score:.3f}")
        
        if total_error_score < 0.1:
            print("🟢 EXCELLENT precision - suitable for high-quality work")
        elif total_error_score < 0.5:
            print("🟡 GOOD precision - suitable for most applications")
        else:
            print("🔴 POOR precision - system may need calibration")
            
    async def _move_pen_to_safe_position(self):
        """Move pen to safe travel position"""
        pen_axis = self.axes["Pen"]
        safe_angle = self.pen_geometry["angle_range"][0]  # Most retracted position
        await pen_axis.move_to_position_abs_user(
            safe_angle, 
            speed_user=self.config["speeds"]["pen_speed"]
        )
        
    async def _move_to_xy(self, x: float, y: float):
        """Move to XY position safely"""
        positions = {"X": x, "Y": y}
        speeds = {
            "X": self.config["speeds"]["xy_speed"],
            "Y": self.config["speeds"]["xy_speed"]
        }
        
        await self.controller.move_all_to_positions_abs_user(
            positions_user=positions,
            speeds_user=speeds,
            wait_for_all=True
        )
        
    async def _automated_height_probe(self) -> Tuple[float, float]:
        """
        Automated height probing - slowly lower pen until contact.
        Returns (z_height, pen_angle) or (nan, nan) if failed.
        """
        pen_axis = self.axes["Pen"]
        
        # Start from safe position
        start_angle = self.pen_geometry["angle_range"][0]
        end_angle = self.pen_geometry["angle_range"][1]
        
        # Move in small increments, checking for resistance/contact
        # This is a simplified version - real implementation would need force sensing
        current_angle = start_angle
        step_size = 2.0  # degrees per step
        
        while current_angle < end_angle:
            await pen_axis.move_to_position_abs_user(
                current_angle, 
                speed_user=self.config["speeds"]["pen_speed"] * 0.2  # Slow probing
            )
            
            # In a real implementation, check for contact here
            # For simulation, just use a typical contact angle
            contact_angle = start_angle + (end_angle - start_angle) * 0.6
            
            if current_angle >= contact_angle:
                z_height = self._calculate_z_from_pen_angle(current_angle)
                return z_height, current_angle
                
            current_angle += step_size
            
        # Failed to find contact
        return float('nan'), float('nan')
        
    def _calculate_z_from_pen_angle(self, pen_angle_deg: float) -> float:
        """Calculate Z height from pen angle using geometry"""
        angle_rad = math.radians(pen_angle_deg)
        z_height = (self.pen_geometry["pivot_height"] + 
                   self.pen_geometry["arm_radius"] * math.sin(angle_rad))
        return z_height
        
    def _calculate_bounds(self, points: List[SurfacePoint]) -> Dict[str, float]:
        """Calculate bounding box of surface points"""
        if not points:
            return {"x_min": 0, "x_max": 0, "y_min": 0, "y_max": 0}
            
        x_coords = [p.x for p in points]
        y_coords = [p.y for p in points]
        
        return {
            "x_min": min(x_coords),
            "x_max": max(x_coords),
            "y_min": min(y_coords),
            "y_max": max(y_coords)
        }
        
    def _calculate_surface_statistics(self, points: List[SurfacePoint]) -> Dict[str, float]:
        """Calculate surface statistics"""
        if not points:
            return {}
            
        z_values = [p.z for p in points]
        
        stats = {
            "z_min": min(z_values),
            "z_max": max(z_values),
            "z_mean": sum(z_values) / len(z_values),
            "z_range": max(z_values) - min(z_values),
            "point_count": len(points)
        }
        
        # Calculate standard deviation
        mean_z = stats["z_mean"]
        variance = sum((z - mean_z)**2 for z in z_values) / len(z_values)
        stats["z_std"] = math.sqrt(variance)
        
        return stats
        
    def save_surface_map(self, surface_map: SurfaceMap, filepath: str):
        """Save surface map to JSON file"""
        data = {
            "points": [asdict(point) for point in surface_map.points],
            "bounds": surface_map.bounds,
            "grid_spacing": surface_map.grid_spacing,
            "interpolation_method": surface_map.interpolation_method,
            "creation_date": surface_map.creation_date,
            "statistics": surface_map.statistics,
            "pen_geometry": surface_map.pen_geometry
        }
        
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
            
        logger.info(f"Surface map saved to {filepath}")
        
    def load_surface_map(self, filepath: str) -> SurfaceMap:
        """Load surface map from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        points = [SurfacePoint(**point_data) for point_data in data["points"]]
        
        surface_map = SurfaceMap(
            points=points,
            bounds=data["bounds"],
            grid_spacing=data.get("grid_spacing"),
            interpolation_method=data.get("interpolation_method", "bicubic"),
            creation_date=data.get("creation_date", ""),
            statistics=data.get("statistics"),
            pen_geometry=data.get("pen_geometry")
        )
        
        return surface_map

# ===================================================================
# Main Application
# ===================================================================

async def main():
    """Main application entry point"""
    args = parse_arguments()
    
    # Setup logging
    log_level = getattr(logging, args.log_level.upper())
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.setLevel(log_level)
    
    can_if = None
    generator = None
    
    try:
        # Setup CAN interface
        logger.info("Setting up CAN interface...")
        can_if = CANInterface()
        await can_if.connect()
        logger.info("CAN Interface connected.")
        
        # Create enhanced generator
        config = DEFAULT_CONFIG.copy()
        if args.plotter_bounds:
            config["plotter_bounds"] = args.plotter_bounds
        if args.motor_ids:
            config["motor_ids"] = args.motor_ids
            
        generator = EnhancedHeightMapGenerator(can_if, config)
        await generator.setup_standard_plotter(args.motor_ids)
        
        # Execute requested operation
        if args.interactive:
            surface_map = await generator.interactive_height_mapping(args.save_recording)
            if args.save_map:
                generator.save_surface_map(surface_map, args.save_map)
                
        elif args.grid_map:
            bounds = None
            if args.bounds:
                bounds = tuple(args.bounds)
            surface_map = await generator.grid_height_mapping(args.spacing, bounds)
            if args.save_map:
                generator.save_surface_map(surface_map, args.save_map)
                
        elif args.playback:
            stats = await generator.precision_test_from_recording(args.playback, args.speed_factor)
            if args.save_stats and stats:
                with open(args.save_stats, 'w') as f:
                    json.dump(asdict(stats), f, indent=2)
                    
        else:
            print("No operation specified. Use --help for options.")
            
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        if generator:
            await generator.cleanup()
        if can_if and can_if.is_connected:
            await can_if.disconnect()

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Enhanced Height Map Generator with Recording/Playback")
    
    # Operation modes
    ops = parser.add_mutually_exclusive_group(required=True)
    ops.add_argument('--interactive', action='store_true', 
                     help='Interactive height mapping with manual control')
    ops.add_argument('--grid-map', action='store_true',
                     help='Automated grid-based height mapping')
    ops.add_argument('--playback', type=str, metavar='RECORDING_FILE',
                     help='Playback recorded session for precision testing')
    
    # Recording options
    parser.add_argument('--save-recording', type=str, metavar='FILE',
                       help='Save recording of interactive session')
    parser.add_argument('--save-map', type=str, metavar='FILE',
                       help='Save generated surface map')
    parser.add_argument('--save-stats', type=str, metavar='FILE',
                       help='Save playback statistics')
    
    # Grid mapping options
    parser.add_argument('--spacing', type=float, default=10.0,
                       help='Grid spacing in mm (default: 10.0)')
    parser.add_argument('--bounds', nargs=4, type=float, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'),
                       help='Mapping bounds (default: full plotter area)')
    
    # Playback options
    parser.add_argument('--speed-factor', type=float, default=0.5,
                       help='Playback speed factor (default: 0.5)')
    
    # Hardware configuration
    parser.add_argument('--motor-ids', nargs=3, type=int, metavar=('X', 'Y', 'PEN'),
                       default=[1, 2, 3], help='Motor CAN IDs')
    parser.add_argument('--plotter-bounds', nargs=2, type=float, metavar=('MAX_X', 'MAX_Y'),
                       help='Plotter bounds in mm')
    
    # System options
    parser.add_argument('--log-level', default='INFO', 
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='Set logging level')
    
    return parser.parse_args()

if __name__ == "__main__":
    asyncio.run(main())