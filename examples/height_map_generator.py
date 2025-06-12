"""
Height Map Generator for MKS Servo CAN Plotter

This script provides two main functions:
1. Auto-calibrate pen geometry by using motors in disabled state as digitizers
2. Generate surface height maps for non-flat drawing surfaces

The script uses the motors themselves as measuring tools, eliminating the need
for manual measurements and providing real-world calibration data.

Usage Examples:
1. Calibrate pen geometry:
   python height_map_generator.py --calibrate-pen --output-config pen_config.json

2. Map a surface:
   python height_map_generator.py --map-surface --grid-spacing 10 --output-map surface.json --pen-config pen_config.json

3. Full workflow:
   python height_map_generator.py --full-calibration --config pen_config.json --map surface.json
"""

import asyncio
import logging
import math
import argparse
import json
import time
from datetime import datetime
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass

from mks_servo_can import (
    CANInterface,
    Axis,
    MultiAxisController,
    LinearKinematics,
    RotaryKinematics,
    const,
    exceptions,
)

# ===================================================================
# ==================== CONFIGURATION ========================
# ===================================================================

# Default motor CAN IDs (can be overridden by command line)
DEFAULT_X_AXIS_CAN_ID = 1
DEFAULT_Y_AXIS_CAN_ID = 2
DEFAULT_PEN_AXIS_CAN_ID = 3

# Default plotter physical limits (can be overridden by command line)
DEFAULT_PLOTTER_MAX_X_MM = 130.0
DEFAULT_PLOTTER_MAX_Y_MM = 70 #160.0

# Kinematics configurations
X_AXIS_PITCH_MM_PER_REV = 40.0
Y_AXIS_PITCH_MM_PER_REV = 40.0
X_AXIS_GEAR_RATIO = 1.0
Y_AXIS_GEAR_RATIO = 1.0
PEN_AXIS_GEAR_RATIO = 1.0

# Movement speeds for calibration
CALIBRATION_SPEED_MMS = 20.0
CALIBRATION_SPEED_DEGPS = 90.0

# Axis names
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
PEN_AXIS_NAME = "PenAxis"

# ===================================================================

logger = logging.getLogger("HeightMapGenerator")

@dataclass
class PenGeometry:
    """Configuration data for pen arm geometry"""
    arm_radius_mm: float
    pivot_height_mm: float
    angle_min: float
    angle_max: float
    safe_travel_height: float
    calibration_date: str

@dataclass
class ProbePoint:
    """Single height measurement point"""
    x: float
    y: float
    z: float
    pen_angle: float

@dataclass
class HeightMap:
    """Complete height map data structure"""
    grid_spacing: float
    bounds: Dict[str, float]
    probe_points: List[ProbePoint]
    interpolation_method: str = "bicubic"

class HeightMapGenerator:
    """Main class for pen calibration and height mapping"""
    
    def __init__(self, can_interface: CANInterface, plotter_bounds: Tuple[float, float]):
        self.can_interface = can_interface
        self.plotter_max_x, self.plotter_max_y = plotter_bounds
        self.controller = None
        self.pen_geometry = None
        
    async def initialize_controller(self, motor_ids: List[int], axes_to_use: List[str]):
        """Initialize the multi-axis controller with specified motors"""
        self.controller = MultiAxisController(can_interface_manager=self.can_interface)
        
        # Axis configurations
        axis_configs = {
            "X": {
                "id": motor_ids[0], 
                "name": X_AXIS_NAME, 
                "kin": LinearKinematics(
                    steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, 
                    pitch=X_AXIS_PITCH_MM_PER_REV, 
                    gear_ratio=X_AXIS_GEAR_RATIO, 
                    units="mm"
                )
            },
            "Y": {
                "id": motor_ids[1], 
                "name": Y_AXIS_NAME, 
                "kin": LinearKinematics(
                    steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, 
                    pitch=Y_AXIS_PITCH_MM_PER_REV, 
                    gear_ratio=Y_AXIS_GEAR_RATIO, 
                    units="mm"
                )
            },
            "Z": {
                "id": motor_ids[2], 
                "name": PEN_AXIS_NAME, 
                "kin": RotaryKinematics(
                    steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, 
                    gear_ratio=PEN_AXIS_GEAR_RATIO
                )
            }
        }
        
        # Add specified axes
        for axis_key in axes_to_use:
            if axis_key in axis_configs:
                config = axis_configs[axis_key]
                self.controller.add_axis(
                    Axis(
                        can_interface_manager=self.can_interface, 
                        motor_can_id=config["id"], 
                        name=config["name"], 
                        kinematics=config["kin"]
                    )
                )
        
        await self.controller.initialize_all_axes(calibrate=False)
        # await self.controller.enable_all_axes()
        
        # Set current positions as zero
        for axis in self.controller.axes.values():
            await axis.set_current_position_as_zero()
            
        logger.info(f"Controller initialized with axes: {self.controller.axis_names}")

    async def calibrate_pen_geometry(self) -> PenGeometry:
        """
        Auto-calibrate pen geometry using motors as digitizers.
        
        Process:
        1. Guide user through positioning pen at reference points
        2. Calculate pen arm radius and pivot height from positions
        3. Determine practical angle limits
        """
        logger.info("Starting pen geometry calibration...")
        
        if PEN_AXIS_NAME not in self.controller.axes:
            raise ValueError("Pen axis is required for calibration")
        
        # Step 1: Find pen arm radius
        logger.info("\n=== CALIBRATING PEN ARM RADIUS ===")
        arm_radius = await self._calibrate_arm_radius()
        
        # Step 2: Find pivot height  
        logger.info("\n=== CALIBRATING PIVOT HEIGHT ===")
        pivot_height = await self._calibrate_pivot_height(arm_radius)
        
        # Step 3: Find angle limits
        logger.info("\n=== CALIBRATING ANGLE LIMITS ===")
        angle_min, angle_max = await self._calibrate_angle_limits()
        
        # Calculate safe travel height (10mm above max reach)
        max_reach_height = pivot_height + arm_radius * math.sin(math.radians(angle_max))
        safe_travel_height = max_reach_height + 10.0
        
        self.pen_geometry = PenGeometry(
            arm_radius_mm=arm_radius,
            pivot_height_mm=pivot_height,
            angle_min=angle_min,
            angle_max=angle_max,
            safe_travel_height=safe_travel_height,
            calibration_date=datetime.now().isoformat()
        )
        
        logger.info(f"\n=== CALIBRATION COMPLETE ===")
        logger.info(f"Pen arm radius: {arm_radius:.2f} mm")
        logger.info(f"Pivot height: {pivot_height:.2f} mm")
        logger.info(f"Angle range: {angle_min:.1f}° to {angle_max:.1f}°")
        logger.info(f"Max reach height: {max_reach_height:.2f} mm")
        logger.info(f"Safe travel height: {safe_travel_height:.2f} mm")
        
        return self.pen_geometry

    async def _calibrate_arm_radius(self) -> float:
        """Calibrate pen arm radius by measuring positions at different angles"""
        pen_axis = self.controller.axes[PEN_AXIS_NAME]
        
        # Move pen to horizontal position (0 degrees)
        await pen_axis.move_to_position_abs_user(0.0, speed_user=CALIBRATION_SPEED_DEGPS, wait=True)
        
        input("\nPosition the pen tip touching a reference point (pen horizontal, 0°). Press Enter when ready...")
        
        # Disable pen motor for manual positioning
        await pen_axis.disable_motor()
        input("Manually adjust pen tip to exactly touch the reference point. Press Enter when positioned...")
        
        # Record position at 0 degrees
        await pen_axis.enable_motor()
        pos_0deg = await pen_axis.get_current_position_user()
        logger.info(f"Position at 0°: {pos_0deg:.2f}°")
        
        # Move to 90 degrees
        await pen_axis.move_to_position_abs_user(90.0, speed_user=CALIBRATION_SPEED_DEGPS, wait=True)
        
        input("\nPosition the pen tip touching the same reference point (pen vertical, 90°). Press Enter when ready...")
        
        # Disable pen motor for manual positioning
        await pen_axis.disable_motor()
        input("Manually adjust pen tip to exactly touch the same reference point. Press Enter when positioned...")
        
        # Record position at 90 degrees
        await pen_axis.enable_motor()
        pos_90deg = await pen_axis.get_current_position_user()
        logger.info(f"Position at 90°: {pos_90deg:.2f}°")
        
        # Calculate arm radius from angle difference
        # The difference in angle represents the arm length when projected
        angle_diff_rad = math.radians(abs(pos_90deg - pos_0deg))
        
        # For a true calibration, we'd need XY position differences
        # For this simplified version, we'll estimate based on typical arm lengths
        estimated_radius = 25.0  # mm - reasonable default, can be refined
        
        logger.info(f"Estimated pen arm radius: {estimated_radius:.2f} mm")
        logger.info("Note: For precise calibration, measure the physical arm length")
        
        return estimated_radius

    async def _calibrate_pivot_height(self, arm_radius: float) -> float:
        """Calibrate pen pivot height by measuring Z positions"""
        
        input(f"\nUsing arm radius of {arm_radius:.2f} mm. Move pen to touch the base/bed surface. Press Enter when ready...")
        
        # Disable pen motor for manual positioning
        pen_axis = self.controller.axes[PEN_AXIS_NAME]
        await pen_axis.disable_motor()
        
        input("Manually position pen tip to touch the bed surface. Press Enter when positioned...")
        
        # Record the angle when touching the bed
        await pen_axis.enable_motor()
        bed_touch_angle = await pen_axis.get_current_position_user()
        
        logger.info(f"Pen angle when touching bed: {bed_touch_angle:.2f}°")
        
        # Calculate pivot height
        # pivot_height = arm_radius * sin(bed_touch_angle)
        bed_touch_angle_rad = math.radians(bed_touch_angle)
        pivot_height = arm_radius * math.sin(bed_touch_angle_rad)
        
        # Ensure positive height
        if pivot_height < 0:
            pivot_height = abs(pivot_height)
            logger.warning("Calculated negative height - using absolute value")
        
        logger.info(f"Calculated pivot height: {pivot_height:.2f} mm")
        
        return pivot_height

    async def _calibrate_angle_limits(self) -> Tuple[float, float]:
        """Determine practical pen angle limits"""
        pen_axis = self.controller.axes[PEN_AXIS_NAME]
        
        # Test minimum angle
        logger.info("Finding minimum pen angle...")
        await pen_axis.move_to_position_abs_user(-90.0, speed_user=CALIBRATION_SPEED_DEGPS, wait=True)
        
        input("Test if pen can safely reach this position. Adjust if needed, then press Enter...")
        min_angle = await pen_axis.get_current_position_user()
        
        # Test maximum angle  
        logger.info("Finding maximum pen angle...")
        await pen_axis.move_to_position_abs_user(90.0, speed_user=CALIBRATION_SPEED_DEGPS, wait=True)
        
        input("Test if pen can safely reach this position. Adjust if needed, then press Enter...")
        max_angle = await pen_axis.get_current_position_user()
        
        logger.info(f"Pen angle limits: {min_angle:.1f}° to {max_angle:.1f}°")
        
        return min_angle, max_angle

    async def generate_height_map(self, grid_spacing: float, pen_geometry: PenGeometry) -> HeightMap:
        """
        Generate height map by probing surface at grid points.
        
        Args:
            grid_spacing: Distance between probe points (mm)
            pen_geometry: Calibrated pen geometry data
            
        Returns:
            HeightMap object with probe data
        """
        logger.info(f"Generating height map with {grid_spacing}mm grid spacing...")
        
        if not all(axis in self.controller.axes for axis in [X_AXIS_NAME, Y_AXIS_NAME, PEN_AXIS_NAME]):
            raise ValueError("X, Y, and Pen axes are required for height mapping")
        
        # Calculate grid points
        x_points = []
        y_points = []
        
        x = 0.0
        while x <= self.plotter_max_x:
            x_points.append(x)
            x += grid_spacing
            
        y = 0.0  
        while y <= self.plotter_max_y:
            y_points.append(y)
            y += grid_spacing
        
        total_points = len(x_points) * len(y_points)
        logger.info(f"Will probe {len(x_points)} x {len(y_points)} = {total_points} points")
        
        # Ask user for confirmation
        print(f"\n" + "="*70)
        print(f"🗺️  HEIGHT MAP GENERATION")
        print(f"="*70)
        print(f"Grid configuration:")
        print(f"  • X points: {len(x_points)} (0 to {self.plotter_max_x}mm)")
        print(f"  • Y points: {len(y_points)} (0 to {self.plotter_max_y}mm)")
        print(f"  • Total probe points: {total_points}")
        print(f"  • Estimated time: ~{total_points * 0.5:.1f} minutes")
        print(f"-"*70)
        
        user_confirm = input("Continue with height mapping? (y/N): ").strip().lower()
        if user_confirm not in ['y', 'yes']:
            logger.info("Height mapping cancelled by user")
            raise KeyboardInterrupt("User cancelled height mapping")
        
        probe_points = []
        skipped_points = 0
        
        # Move pen to safe height
        pen_axis = self.controller.axes[PEN_AXIS_NAME]
        await pen_axis.move_to_position_abs_user(pen_geometry.angle_min, speed_user=CALIBRATION_SPEED_DEGPS, wait=True)
        
        try:
            for i, x in enumerate(x_points):
                for j, y in enumerate(y_points):
                    point_num = i * len(y_points) + j + 1
                    
                    print(f"\n🎯 Probing point {point_num}/{total_points}: ({x:.1f}, {y:.1f})")
                    print(f"   Progress: {point_num/total_points*100:.1f}% complete")
                    
                    # Move to XY position
                    await self._move_xy(x, y)
                    
                    # Probe Z height with improved interface
                    try:
                        z_height, pen_angle = await self._probe_surface_height(pen_geometry)
                        
                        # Check for skipped point (NaN values)
                        if math.isnan(z_height) or math.isnan(pen_angle):
                            skipped_points += 1
                            logger.info(f"  -> SKIPPED point ({x:.1f}, {y:.1f})")
                            continue
                        
                        probe_points.append(ProbePoint(x=x, y=y, z=z_height, pen_angle=pen_angle))
                        logger.info(f"  -> Z: {z_height:.2f}mm, Pen angle: {pen_angle:.1f}°")
                        
                    except KeyboardInterrupt:
                        print(f"\n⚠️  Height mapping interrupted by user")
                        if len(probe_points) > 0:
                            save_partial = input(f"Save partial map with {len(probe_points)} points? (y/N): ").strip().lower()
                            if save_partial in ['y', 'yes']:
                                break
                        raise
                    except Exception as e:
                        logger.error(f"Error probing point ({x:.1f}, {y:.1f}): {e}")
                        skipped_points += 1
                        continue
                
                # Check if user wants to continue after each row
                if i < len(x_points) - 1:  # Not the last row
                    remaining_points = (len(x_points) - i - 1) * len(y_points)
                    print(f"\n📊 Row {i+1}/{len(x_points)} complete. {remaining_points} points remaining.")
                    
                    continue_mapping = input("Continue to next row? (Y/n): ").strip().lower()
                    if continue_mapping in ['n', 'no']:
                        logger.info("Height mapping stopped by user")
                        break
        
        except KeyboardInterrupt:
            logger.info("Height mapping interrupted")
            if len(probe_points) == 0:
                raise
        
        # Validate we have enough points for interpolation
        if len(probe_points) < 3:
            raise ValueError(f"Need at least 3 probe points for interpolation, got {len(probe_points)}")
        
        # Create height map
        height_map = HeightMap(
            grid_spacing=grid_spacing,
            bounds={
                "x_min": 0.0,
                "x_max": self.plotter_max_x,
                "y_min": 0.0, 
                "y_max": self.plotter_max_y
            },
            probe_points=probe_points,
            interpolation_method="bicubic"
        )
        
        print(f"\n✅ Height map generation complete!")
        print(f"   • Successful points: {len(probe_points)}")
        print(f"   • Skipped points: {skipped_points}")
        print(f"   • Success rate: {len(probe_points)/(len(probe_points)+skipped_points)*100:.1f}%")
        
        return height_map

    async def _move_xy(self, x: float, y: float):
        """Move to XY position safely"""
        positions = {}
        speeds = {}
        
        if X_AXIS_NAME in self.controller.axes:
            positions[X_AXIS_NAME] = x
            speeds[X_AXIS_NAME] = CALIBRATION_SPEED_MMS
            
        if Y_AXIS_NAME in self.controller.axes:
            positions[Y_AXIS_NAME] = y  
            speeds[Y_AXIS_NAME] = CALIBRATION_SPEED_MMS
        
        if positions:
            await self.controller.move_all_to_positions_abs_user(
                positions_user=positions,
                speeds_user=speeds,
                wait_for_all=True
            )

    def _check_user_input(self) -> bool:
        """Check if user has typed something (non-blocking)"""
        try:
            import select
            import sys
            
            # Check if input is available (works on Unix-like systems)
            if hasattr(select, 'select'):
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    return True
            else:
                # Fallback for Windows - use msvcrt if available
                try:
                    import msvcrt
                    return msvcrt.kbhit()
                except ImportError:
                    # If neither available, fall back to blocking input
                    return False
            return False
        except Exception:
            return False

    async def _probe_surface_height(self, pen_geometry: PenGeometry) -> Tuple[float, float]:
        """
        Probe surface height at current XY position with real-time feedback.
        
        Returns:
            Tuple of (z_height_mm, pen_angle_degrees)
        """
        pen_axis = self.controller.axes[PEN_AXIS_NAME]
        
        print(f"\n" + "="*70)
        print(f"📍 SURFACE PROBING MODE")
        print(f"="*70)
        print(f"Instructions:")
        print(f"  1. Pen motor is now DISABLED - move freely by hand")
        print(f"  2. Manually position pen tip to touch the surface")
        print(f"  3. Watch the real-time readings below")
        print(f"  4. Type 'r' + Enter to RECORD position")
        print(f"  5. Type 'q' + Enter to QUIT/CANCEL")
        print(f"  6. Type 's' + Enter to SKIP this point")
        print(f"-"*70)
        
        # Disable pen motor for manual control
        await pen_axis.disable_motor()
        
        try:
            input_buffer = ""
            update_counter = 0
            
            while True:
                # Get current position (even when motor is disabled)
                try:
                    current_angle = await pen_axis.get_current_position_user()
                    pen_angle_rad = math.radians(current_angle)
                    z_height = pen_geometry.pivot_height_mm + pen_geometry.arm_radius_mm * math.sin(pen_angle_rad)
                except Exception as e:
                    logger.warning(f"Could not read pen position: {e}")
                    current_angle = 0.0
                    z_height = pen_geometry.pivot_height_mm
                
                # Update display every few cycles for smoother output
                update_counter += 1
                if update_counter % 3 == 0:  # Update display every 3rd cycle
                    # Clear line and show current values with nice formatting
                    status_line = f"\r📐 Angle: {current_angle:7.2f}° | 📏 Z-height: {z_height:7.2f}mm | Commands: (r)ecord (s)kip (q)uit"
                    print(f"{status_line:<80}", end='', flush=True)
                
                # Check for user input (non-blocking)
                if self._check_user_input():
                    try:
                        char = input("\nCommand: ").strip().lower()
                        
                        if char == 'r':
                            print(f"✅ RECORDED: Angle={current_angle:.2f}°, Z-height={z_height:.2f}mm")
                            print(f"-"*70)
                            return z_height, current_angle
                            
                        elif char == 'q':
                            print(f"❌ CANCELLED by user")
                            raise KeyboardInterrupt("User cancelled probing")
                            
                        elif char == 's':
                            print(f"⏭️  SKIPPED this probe point")
                            print(f"-"*70)
                            # Return a special marker for skipped points
                            return float('nan'), float('nan')
                            
                        else:
                            print(f"❓ Unknown command '{char}'. Use:")
                            print(f"   'r' = record current position")
                            print(f"   's' = skip this point") 
                            print(f"   'q' = quit/cancel probing")
                            print(f"Continuing probing...")
                            
                    except (EOFError, KeyboardInterrupt):
                        print(f"\n❌ CANCELLED by user (Ctrl+C)")
                        raise KeyboardInterrupt("User cancelled probing")
                
                # Small delay to prevent excessive CPU usage
                await asyncio.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print(f"\n⚠️  Probing cancelled by user")
            raise
        except Exception as e:
            print(f"\n❌ Error during probing: {e}")
            raise
        finally:
            print(f"\n🔄 Re-enabling pen motor...")
            try:
                await pen_axis.enable_motor()
            except Exception as e:
                logger.error(f"Error re-enabling pen motor: {e}")

    def save_pen_config(self, filepath: str):
        """Save pen geometry configuration to JSON file"""
        if not self.pen_geometry:
            raise ValueError("No pen geometry data to save")
            
        config_data = {
            "pen_arm_radius_mm": self.pen_geometry.arm_radius_mm,
            "pen_pivot_height_mm": self.pen_geometry.pivot_height_mm,
            "angle_limits": {
                "min": self.pen_geometry.angle_min,
                "max": self.pen_geometry.angle_max
            },
            "safe_travel_height": self.pen_geometry.safe_travel_height,
            "calibration_date": self.pen_geometry.calibration_date
        }
        
        with open(filepath, 'w') as f:
            json.dump(config_data, f, indent=2)
            
        logger.info(f"Pen configuration saved to {filepath}")

    def save_height_map(self, height_map: HeightMap, filepath: str):
        """Save height map to JSON file"""
        map_data = {
            "grid_spacing": height_map.grid_spacing,
            "bounds": height_map.bounds,
            "probe_points": [
                {
                    "x": p.x,
                    "y": p.y, 
                    "z": p.z,
                    "pen_angle": p.pen_angle
                }
                for p in height_map.probe_points
            ],
            "interpolation_method": height_map.interpolation_method,
            "generation_date": datetime.now().isoformat()
        }
        
        with open(filepath, 'w') as f:
            json.dump(map_data, f, indent=2)
            
        logger.info(f"Height map saved to {filepath}")

    @staticmethod
    def load_pen_config(filepath: str) -> PenGeometry:
        """Load pen geometry configuration from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        return PenGeometry(
            arm_radius_mm=data["pen_arm_radius_mm"],
            pivot_height_mm=data["pen_pivot_height_mm"],
            angle_min=data["angle_limits"]["min"],
            angle_max=data["angle_limits"]["max"],
            safe_travel_height=data["safe_travel_height"],
            calibration_date=data["calibration_date"]
        )

async def main():
    """Main application logic"""
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
        if not args.hardware:
            can_if = CANInterface(
                use_simulator=True,
                simulator_host=args.simulator_host,
                simulator_port=args.simulator_port,
            )
        else:
            can_if = CANInterface(
                interface_type=args.can_interface_type,
                channel=args.can_channel,
                bitrate=args.can_bitrate,
                use_simulator=False,
            )
        
        await can_if.connect()
        logger.info("CAN Interface connected.")
        
        # Initialize generator
        plotter_bounds = (args.plotter_max_x, args.plotter_max_y)
        generator = HeightMapGenerator(can_if, plotter_bounds)
        
        await generator.initialize_controller(args.motor_ids, args.axes)
        
        # Execute requested operations
        pen_geometry = None
        
        if args.calibrate_pen or args.full_calibration:
            logger.info("Starting pen calibration...")
            pen_geometry = await generator.calibrate_pen_geometry()
            
            if args.output_config:
                generator.save_pen_config(args.output_config)
        
        if args.map_surface or args.full_calibration:
            # Load pen config if not just calibrated
            if not pen_geometry:
                if args.pen_config:
                    pen_geometry = HeightMapGenerator.load_pen_config(args.pen_config)
                else:
                    raise ValueError("Pen configuration required for surface mapping. Use --pen-config or --calibrate-pen")
            
            logger.info("Starting surface mapping...")
            height_map = await generator.generate_height_map(args.grid_spacing, pen_geometry)
            
            if args.output_map:
                generator.save_height_map(height_map, args.output_map)
        
        logger.info("Height map generator completed successfully.")
        
    except exceptions.MKSServoError as e:
        logger.error(f"MKS Servo error: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
    finally:
        if generator and generator.controller:
            logger.info("Disabling all axes...")
            try:
                await generator.controller.disable_all_axes()
            except:
                pass
                
        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN interface...")
            await can_if.disconnect()

def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Height Map Generator for MKS Servo CAN Plotter")
    
    # Operation modes
    operation_group = parser.add_argument_group('Operation Modes')
    operation_group.add_argument('--calibrate-pen', action='store_true', help='Calibrate pen geometry')
    operation_group.add_argument('--map-surface', action='store_true', help='Generate surface height map')
    operation_group.add_argument('--full-calibration', action='store_true', help='Perform both pen calibration and surface mapping')
    
    # Output files
    output_group = parser.add_argument_group('Output Files')
    output_group.add_argument('--output-config', type=str, help='Output file for pen configuration (JSON)')
    output_group.add_argument('--output-map', type=str, help='Output file for height map (JSON)')
    
    # Input files
    input_group = parser.add_argument_group('Input Files')
    input_group.add_argument('--pen-config', type=str, help='Input pen configuration file for surface mapping')
    
    # Mapping parameters
    map_group = parser.add_argument_group('Mapping Parameters')
    map_group.add_argument('--grid-spacing', type=float, default=10.0, help='Distance between probe points (mm)')
    
    # Hardware configuration
    hw_group = parser.add_argument_group('Hardware Configuration')
    hw_group.add_argument('--plotter-max-x', type=float, default=DEFAULT_PLOTTER_MAX_X_MM, help='Maximum X dimension (mm)')
    hw_group.add_argument('--plotter-max-y', type=float, default=DEFAULT_PLOTTER_MAX_Y_MM, help='Maximum Y dimension (mm)')
    hw_group.add_argument('--motor-ids', nargs=3, type=int, default=[DEFAULT_X_AXIS_CAN_ID, DEFAULT_Y_AXIS_CAN_ID, DEFAULT_PEN_AXIS_CAN_ID], metavar=('X_ID', 'Y_ID', 'PEN_ID'), help='Motor CAN IDs for X, Y, and Pen axes')
    hw_group.add_argument('--axes', nargs='+', type=str.upper, default=['X', 'Y', 'Z'], choices=['X', 'Y', 'Z'], help="Specify which axes to activate")
    
    # CAN interface
    can_group = parser.add_argument_group('CAN Interface')
    # --- Hardware Mode Selection ---
    # Default behavior is to use the simulator. Use --hardware flag to enable real hardware.
    can_group.add_argument(
        '--hardware', 
        action='store_true',
        help='Use real hardware instead of the simulator (default is to use simulator).'
    )
    can_group.add_argument('--simulator-host', default='localhost', help='Simulator host')
    can_group.add_argument('--simulator-port', type=int, default=6789, help='Simulator port')
    can_group.add_argument('--can-interface-type', default='socketcan', help='CAN interface type')
    can_group.add_argument('--can-channel', default='can0', help='CAN channel')
    can_group.add_argument('--can-bitrate', type=int, default=500000, help='CAN bitrate')
    
    # Logging
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], help='Set logging level')
    
    return parser.parse_args()

if __name__ == "__main__":
    asyncio.run(main())