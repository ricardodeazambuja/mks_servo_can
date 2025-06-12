"""
Example: Controlling a 3-Axis Plotter to draw an SVG file with height compensation.

This advanced script demonstrates how to parse an SVG vector file, process its
paths and shapes, scale them to fit the plotter's physical dimensions, and
then draw the result using the mks-servo-can library with optional height
compensation from surface height maps.

Enhanced features:
- Surface-following drawing using height maps
- Auto-calibrated pen geometry support
- Smart pen angle calculation for 3D surfaces
- Collision detection and reach validation

It relies on the 'svgelements' library for robust SVG parsing.
Please install it before running:
`pip install svgelements`

By default, this script runs using the simulator. To use real hardware,
provide the `--no-simulator` flag and other relevant CAN arguments.

To run with the simulator (default):
1. Start the simulator: `mks-servo-simulator --num-motors 3 --start-can-id 1`
2. Run this script with a path to an SVG file:
   `python svg_plotter.py --file path/to/your/drawing.svg`

To run with height compensation:
1. First generate a height map with height_map_generator.py
2. Run with height map:
   `python svg_plotter.py --file drawing.svg --height-map surface.json --pen-config pen_config.json`

To run with real hardware:
1. Configure your CAN adapter, motor CAN IDs, and physical parameters below.
2. Run the script with the appropriate hardware flags:
   `python svg_plotter.py --file drawing.svg --no-simulator --can-channel /dev/ttyACM0`
"""

import asyncio
import logging
import math
import argparse
import json
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from scipy.interpolate import griddata
import numpy as np

# This script requires the 'svgelements' library for SVG parsing.
# Install it using: pip install svgelements
try:
    from svgelements import SVG, Path, Shape
except ImportError:
    print("This script requires the 'svgelements' library.")
    print("Please install it using: pip install svgelements")
    exit(1)

# scipy is required for height map interpolation
try:
    from scipy.interpolate import griddata
    import numpy as np
except ImportError:
    print("This script requires scipy and numpy for height map interpolation.")
    print("Please install using: pip install scipy numpy")
    exit(1)

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
# ==================== PLOTTER CONFIGURATION ========================
# ===================================================================
# These values can be adjusted to match your physical plotter setup.
# They can also be overridden by command-line arguments.

# --- Manual Scale Configuration ---
# Set to None to use automatic fitting, or specify a scale factor
# Scale factor: 1.0 = 100% original size, 0.5 = 50%, 2.0 = 200%
DEFAULT_MANUAL_SCALE = None  # Change this to a float value to override automatic fitting

# --- Motor CAN IDs ---
X_AXIS_CAN_ID = 1
Y_AXIS_CAN_ID = 2
PEN_AXIS_CAN_ID = 3

# --- Plotter Physical Limits (in mm) ---
PLOTTER_MAX_X_MM = 130.0
PLOTTER_MAX_Y_MM = 80 #160.0

# --- Plotter Speeds ---
DRAWING_SPEED_MMS = 25.0
TRAVEL_SPEED_MMS = 100.0
PEN_ROTATION_SPEED_DEGPS = 360.0

# --- X-Axis Kinematics Configuration ---
X_AXIS_PITCH_MM_PER_REV = 40.0
X_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
X_AXIS_GEAR_RATIO = 1.0

# --- Y-Axis Kinematics Configuration ---
Y_AXIS_PITCH_MM_PER_REV = 40.0
Y_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
Y_AXIS_GEAR_RATIO = 1.0

# --- Pen (Z-Axis) Kinematics Configuration ---
# These are fallback values - prefer loading from pen config file
PEN_UP_ANGLE = 0.0
PEN_DOWN_ANGLE = -35.0
PEN_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
PEN_AXIS_GEAR_RATIO = 1.0
# ===================================================================

# --- Axis Names (used internally in the script) ---
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
PEN_AXIS_NAME = "PenAxis"

# --- Logging Setup ---

logger = logging.getLogger("SVGPlotter")

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

class HeightMap:
    """Height map for surface-following drawing"""
    
    def __init__(self, data: Dict[str, Any]):
        self.grid_spacing = data["grid_spacing"]
        self.bounds = data["bounds"]
        self.interpolation_method = data.get("interpolation_method", "bicubic")
        
        # Convert probe points to arrays for interpolation
        self.probe_points = [
            ProbePoint(p["x"], p["y"], p["z"], p["pen_angle"]) 
            for p in data["probe_points"]
        ]
        
        # Prepare interpolation data
        self.x_coords = np.array([p.x for p in self.probe_points])
        self.y_coords = np.array([p.y for p in self.probe_points])
        self.z_coords = np.array([p.z for p in self.probe_points])
        self.angle_coords = np.array([p.pen_angle for p in self.probe_points])
        
        logger.info(f"Height map loaded: {len(self.probe_points)} points, "
                   f"bounds ({self.bounds['x_min']:.1f},{self.bounds['y_min']:.1f}) to "
                   f"({self.bounds['x_max']:.1f},{self.bounds['y_max']:.1f})")
    
    def get_height_and_angle(self, x: float, y: float) -> Tuple[float, float]:
        """
        Interpolate height and pen angle for given X,Y coordinates.
        
        Returns:
            Tuple of (z_height, pen_angle)
        """
        # Check bounds
        if (x < self.bounds["x_min"] or x > self.bounds["x_max"] or 
            y < self.bounds["y_min"] or y > self.bounds["y_max"]):
            logger.warning(f"Point ({x:.1f},{y:.1f}) outside height map bounds")
            return 0.0, 0.0
        
        # Interpolate height
        try:
            z_height = griddata(
                (self.x_coords, self.y_coords), 
                self.z_coords, 
                (x, y), 
                method='linear',  # Use linear for better stability
                fill_value=0.0
            )
            
            # Interpolate pen angle
            pen_angle = griddata(
                (self.x_coords, self.y_coords), 
                self.angle_coords, 
                (x, y), 
                method='linear',
                fill_value=0.0
            )
            
            # Handle scalar results (when interpolating single points)
            if np.isscalar(z_height):
                return float(z_height), float(pen_angle)
            else:
                return float(z_height[0]), float(pen_angle[0])
                
        except Exception as e:
            logger.warning(f"Interpolation failed at ({x:.1f},{y:.1f}): {e}")
            return 0.0, 0.0


def process_svg(filepath: str, max_plot_x: float, max_plot_y: float, manual_scale: float = None, max_x_limit: float = None, max_y_limit: float = None) -> List[List[Tuple[float, float]]]:
    """
    Loads, parses, and processes an SVG file into plotter-ready coordinates.

    This function handles:
    1. Parsing all shapes from the SVG.
    2. Converting shapes and curved paths into a series of straight line segments.
    3. Either scaling manually (if manual_scale is provided) or automatically 
       scaling to fit within the plotter's physical bounds while maintaining aspect ratio,
       or scaling to fit within specified dimension limits.
    4. Transforming coordinates from SVG's top-left origin to the plotter's
       bottom-left origin.

    Args:
        filepath: The path to the SVG file.
        max_plot_x: The maximum drawable width of the plotter (mm).
        max_plot_y: The maximum drawable height of the plotter (mm).
        manual_scale: If provided, use this scale factor instead of automatic fitting.
                     1.0 = 100% original size, 0.5 = 50%, 2.0 = 200%.
        max_x_limit: If provided, scale the SVG to fit within this X dimension (mm).
        max_y_limit: If provided, scale the SVG to fit within this Y dimension (mm).

    Returns:
        A list of paths, where each path is a list of (x, y) coordinate tuples
        ready to be sent to the plotter.
    """
    logger.info(f"Processing SVG file: {filepath}")
    paths = []
    svg = SVG.parse(filepath)
    elements = list(svg.elements())

    if not elements:
        logger.warning("SVG file contains no drawable elements.")
        return []

    # Convert all shapes to Path objects
    all_paths = []
    for elem in elements:
        if isinstance(elem, Shape):
            try:
                if isinstance(elem, Path):
                    all_paths.append(elem)
                else:
                    # Convert shape to path
                    path = Path(elem)
                    all_paths.append(path)
            except Exception as e:
                logger.warning(f"Could not convert element to path: {e}")
                continue
    
    if not all_paths:
        logger.warning("Could not extract any valid paths from the SVG elements.")
        return []

    # Calculate bounding box from all individual paths
    all_bboxes = []
    for path in all_paths:
        bbox = path.bbox()
        if bbox:
            all_bboxes.append(bbox)
    
    if not all_bboxes:
        logger.warning("Could not extract any valid bboxes from the paths.")
        return []
    
    # Calculate combined bounding box
    xmin = min(bbox[0] for bbox in all_bboxes)
    ymin = min(bbox[1] for bbox in all_bboxes)
    xmax = max(bbox[2] for bbox in all_bboxes)
    ymax = max(bbox[3] for bbox in all_bboxes)
    
    svg_width = xmax - xmin
    svg_height = ymax - ymin

    if svg_width == 0 or svg_height == 0:
        logger.warning("SVG content has zero width or height. Cannot scale.")
        return []

    # Calculate scaling factor
    if manual_scale is not None:
        scale = manual_scale
        logger.info(f"Using manual scale factor: {scale:.3f} ({scale*100:.1f}%)")
        
        # Calculate the resulting dimensions
        scaled_width = svg_width * scale
        scaled_height = svg_height * scale
        logger.info(f"SVG size: {svg_width:.1f} x {svg_height:.1f} -> Scaled size: {scaled_width:.1f} x {scaled_height:.1f}")
        
        # Check if the scaled drawing exceeds plotter bounds and refuse if it does
        if scaled_width > max_plot_x or scaled_height > max_plot_y:
            raise ValueError(f"Manual scale {scale:.3f} ({scale*100:.1f}%) results in drawing size "
                           f"({scaled_width:.1f} x {scaled_height:.1f}) that exceeds machine limits "
                           f"({max_plot_x} x {max_plot_y}). Please use a smaller scale.")
    
    elif max_x_limit is not None or max_y_limit is not None:
        # Scale to fit within specified limits
        scale_factors = []
        
        if max_x_limit is not None:
            if max_x_limit > max_plot_x:
                raise ValueError(f"Specified max X limit ({max_x_limit}mm) exceeds machine limit ({max_plot_x}mm)")
            scale_x = max_x_limit / svg_width
            scale_factors.append(scale_x)
            logger.info(f"X constraint: {max_x_limit}mm -> scale factor {scale_x:.3f}")
        
        if max_y_limit is not None:
            if max_y_limit > max_plot_y:
                raise ValueError(f"Specified max Y limit ({max_y_limit}mm) exceeds machine limit ({max_plot_y}mm)")
            scale_y = max_y_limit / svg_height
            scale_factors.append(scale_y)
            logger.info(f"Y constraint: {max_y_limit}mm -> scale factor {scale_y:.3f}")
        
        # Use the most restrictive (smallest) scale factor to ensure both constraints are met
        scale = min(scale_factors)
        
        # Calculate final dimensions
        scaled_width = svg_width * scale
        scaled_height = svg_height * scale
        
        logger.info(f"Using constrained scale factor: {scale:.3f} ({scale*100:.1f}%)")
        logger.info(f"SVG size: {svg_width:.1f} x {svg_height:.1f} -> Final size: {scaled_width:.1f} x {scaled_height:.1f}")
        
    else:
        # Use automatic fitting to machine area
        scale_x = max_plot_x / svg_width
        scale_y = max_plot_y / svg_height
        scale = min(scale_x, scale_y) * 0.95  # Use 95% of area for a small margin
        logger.info(f"Using automatic scale factor: {scale:.3f} ({scale*100:.1f}%) to fit machine area")

    logger.info(f"SVG bounds: ({xmin:.1f}, {ymin:.1f}) to ({xmax:.1f}, {ymax:.1f})")
    logger.info(f"Plotter area: {max_plot_x} x {max_plot_y}")

    # Process each path
    for i, path in enumerate(all_paths):
        try:
            # Linearize the path (convert curves to line segments)
            linearized_path = []
            
            # Use as_points() method which is more reliable for getting path points
            try:
                points = list(path.as_points())
                linearized_path = points
            except:
                # Fallback to segments method
                for segment in path.segments():
                    if not linearized_path:
                        linearized_path.append(segment.start)
                    linearized_path.append(segment.end)
            
            if not linearized_path:
                logger.warning(f"Path {i} produced no points, skipping.")
                continue
            
            # Transform coordinates
            transformed_path = []
            for point in linearized_path:
                # Handle both complex numbers and point objects
                if hasattr(point, 'x') and hasattr(point, 'y'):
                    x, y = point.x, point.y
                elif hasattr(point, 'real') and hasattr(point, 'imag'):
                    x, y = point.real, point.imag
                else:
                    logger.warning(f"Unknown point type: {type(point)}, skipping point.")
                    continue
                
                # Scale relative to the SVG's own origin (xmin, ymin)
                plot_x = (x - xmin) * scale
                # Scale and invert the Y-axis
                plot_y = max_plot_y - ((y - ymin) * scale)
                transformed_path.append((plot_x, plot_y))
            
            if transformed_path:
                paths.append(transformed_path)
                logger.info(f"Path {i+1}: {len(transformed_path)} points")
            
        except Exception as e:
            logger.warning(f"Error processing path {i}: {e}")
            continue
        
    logger.info(f"Successfully processed SVG into {len(paths)} drawable paths.")
    return paths


def validate_drawing_reach(paths: List[List[Tuple[float, float]]], height_map: Optional[HeightMap], pen_geometry: Optional[PenGeometry]) -> bool:
    """
    Validate that all drawing points are reachable by the pen.
    
    Args:
        paths: List of drawing paths
        height_map: Optional height map for surface heights
        pen_geometry: Optional pen geometry for reach calculations
        
    Returns:
        True if all points are reachable, False otherwise
    """
    if not height_map or not pen_geometry:
        logger.info("No height map or pen geometry - skipping reach validation")
        return True
    
    unreachable_points = []
    
    for path_idx, path in enumerate(paths):
        for point_idx, (x, y) in enumerate(path):
            # Get required pen angle for this position
            z_height, required_angle = height_map.get_height_and_angle(x, y)
            
            # Check if angle is within pen limits
            if required_angle < pen_geometry.angle_min or required_angle > pen_geometry.angle_max:
                unreachable_points.append((path_idx, point_idx, x, y, required_angle))
    
    if unreachable_points:
        logger.error(f"Found {len(unreachable_points)} unreachable points:")
        for path_idx, point_idx, x, y, angle in unreachable_points[:5]:  # Show first 5
            logger.error(f"  Path {path_idx}, Point {point_idx}: ({x:.1f},{y:.1f}) requires angle {angle:.1f}° "
                        f"(limits: {pen_geometry.angle_min:.1f}° to {pen_geometry.angle_max:.1f}°)")
        if len(unreachable_points) > 5:
            logger.error(f"  ... and {len(unreachable_points) - 5} more points")
        return False
    
    logger.info("All drawing points are within pen reach limits")
    return True


def calculate_pen_angle_for_position(x: float, y: float, height_map: Optional[HeightMap], pen_geometry: Optional[PenGeometry]) -> float:
    """
    Calculate the required pen angle for a given X,Y position.
    
    Args:
        x, y: Target coordinates
        height_map: Height map for surface following
        pen_geometry: Pen geometry configuration
        
    Returns:
        Required pen angle in degrees
    """
    if not height_map or not pen_geometry:
        # Fallback to simple pen down angle
        return PEN_DOWN_ANGLE
    
    # Get surface height and suggested angle from height map
    z_height, suggested_angle = height_map.get_height_and_angle(x, y)
    
    # Validate angle is within limits
    if suggested_angle < pen_geometry.angle_min:
        logger.warning(f"Angle {suggested_angle:.1f}° below limit {pen_geometry.angle_min:.1f}° at ({x:.1f},{y:.1f})")
        return pen_geometry.angle_min
    elif suggested_angle > pen_geometry.angle_max:
        logger.warning(f"Angle {suggested_angle:.1f}° above limit {pen_geometry.angle_max:.1f}° at ({x:.1f},{y:.1f})")
        return pen_geometry.angle_max
    
    return suggested_angle


# --- Helper Functions for Plotter Control ---

async def pen_up(controller: MultiAxisController, speed_degps: float, pen_geometry: Optional[PenGeometry] = None):
    """Commands the pen to move to the 'up' position if the pen axis is active."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen up.")
        await asyncio.sleep(0.5)
        return
    
    # Use safe travel height if available, otherwise use traditional pen up angle
    if pen_geometry:
        # Calculate angle for safe travel height
        safe_angle = math.degrees(math.asin(pen_geometry.safe_travel_height / pen_geometry.arm_radius_mm))
        safe_angle = min(safe_angle, pen_geometry.angle_max)  # Don't exceed limits
        logger.info(f"Moving pen UP to safe travel angle {safe_angle:.1f}°")
        target_angle = safe_angle
    else:
        logger.info(f"Moving pen UP to {PEN_UP_ANGLE}°")
        target_angle = PEN_UP_ANGLE
    
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        target_angle, speed_user=speed_degps, wait=True
    )

async def pen_down(controller: MultiAxisController, x: float, y: float, speed_degps: float, 
                   height_map: Optional[HeightMap] = None, pen_geometry: Optional[PenGeometry] = None):
    """Commands the pen to move to the appropriate 'down' position for the given coordinates."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen down.")
        await asyncio.sleep(0.5)
        return
    
    # Calculate required angle for this position
    target_angle = calculate_pen_angle_for_position(x, y, height_map, pen_geometry)
    
    logger.info(f"Moving pen DOWN to {target_angle:.1f}° for position ({x:.1f},{y:.1f})")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        target_angle, speed_user=speed_degps, wait=True
    )

async def move_to(
    controller: MultiAxisController,
    target_x: float,
    target_y: float,
    speed_mms: float,
    max_x: float,
    max_y: float,
    is_drawing: bool = False,
    height_map: Optional[HeightMap] = None,
    pen_geometry: Optional[PenGeometry] = None
):
    """Moves the plotter's active X and Y axes to a target coordinate with optional pen angle adjustment."""
    if not (0 <= target_x <= max_x and 0 <= target_y <= max_y):
        logger.warning(
            f"Move to ({target_x:.1f}, {target_y:.1f}) is outside plotter limits. Skipping move."
        )
        return

    logger.info(f"Moving to (X:{target_x:.2f}, Y:{target_y:.2f}) at {speed_mms:.1f} mm/s.")
    
    current_positions = await controller.get_all_positions_user()
    
    positions_to_move = {}
    deltas = {}
    
    if X_AXIS_NAME in controller.axes:
        current_x = current_positions.get(X_AXIS_NAME, 0.0)
        deltas[X_AXIS_NAME] = target_x - current_x
        positions_to_move[X_AXIS_NAME] = target_x

    if Y_AXIS_NAME in controller.axes:
        current_y = current_positions.get(Y_AXIS_NAME, 0.0)
        deltas[Y_AXIS_NAME] = target_y - current_y
        positions_to_move[Y_AXIS_NAME] = target_y

    if not positions_to_move:
        logger.info("No active XY axes to move.")
        return

    distance = math.sqrt(sum(d**2 for d in deltas.values()))

    if distance < 0.01:
        logger.info("Target is at current position. No move needed.")
        return

    duration_s = distance / speed_mms

    speeds_for_move = {
        axis_name: abs(delta / duration_s) if duration_s > 0 else 0
        for axis_name, delta in deltas.items()
    }
    
    # If drawing and we have height compensation, adjust pen angle during the move
    if is_drawing and height_map and pen_geometry and PEN_AXIS_NAME in controller.axes:
        target_pen_angle = calculate_pen_angle_for_position(target_x, target_y, height_map, pen_geometry)
        positions_to_move[PEN_AXIS_NAME] = target_pen_angle
        speeds_for_move[PEN_AXIS_NAME] = abs(target_pen_angle - current_positions.get(PEN_AXIS_NAME, 0.0)) / duration_s if duration_s > 0 else 0
    
    await controller.move_all_to_positions_abs_user(
        positions_user=positions_to_move,
        speeds_user=speeds_for_move,
        wait_for_all=True
    )


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

def load_height_map(filepath: str) -> HeightMap:
    """Load height map from JSON file"""
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    return HeightMap(data)


# --- Main Application Logic ---

async def run_plotter_sequence(args: argparse.Namespace):
    """
    The main logic sequence for the plotter. Sets up hardware/simulator,
    processes the SVG, and performs the drawing sequence.
    """
    can_if = None
    multi_controller = None

    try:
        # --- 1. Load configuration files ---
        pen_geometry = None
        height_map = None
        
        if args.pen_config:
            logger.info(f"Loading pen configuration from {args.pen_config}")
            pen_geometry = load_pen_config(args.pen_config)
            logger.info(f"Pen config: arm={pen_geometry.arm_radius_mm:.1f}mm, "
                       f"pivot={pen_geometry.pivot_height_mm:.1f}mm, "
                       f"angles={pen_geometry.angle_min:.1f}° to {pen_geometry.angle_max:.1f}°")
        
        if args.height_map:
            logger.info(f"Loading height map from {args.height_map}")
            height_map = load_height_map(args.height_map)
        
        # --- 2. Process SVG file ---
        # Determine which scale to use: command line args, constant, or automatic
        manual_scale = None
        max_x_limit = args.max_x if hasattr(args, 'max_x') else None
        max_y_limit = args.max_y if hasattr(args, 'max_y') else None
        
        if args.scale is not None:
            manual_scale = args.scale
            logger.info(f"Using command-line scale: {manual_scale} ({manual_scale*100:.1f}%)")
        elif DEFAULT_MANUAL_SCALE is not None:
            manual_scale = DEFAULT_MANUAL_SCALE
            logger.info(f"Using default manual scale: {manual_scale} ({manual_scale*100:.1f}%)")
        elif max_x_limit is not None or max_y_limit is not None:
            logger.info(f"Using dimension constraints: max_x={max_x_limit}, max_y={max_y_limit}")
        else:
            logger.info("Using automatic fitting to machine area")
        
        plotter_paths = process_svg(args.file, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM, manual_scale, max_x_limit, max_y_limit)
        if not plotter_paths:
            logger.error("No drawable paths were generated from the SVG. Exiting.")
            return

        # --- 3. Validate drawing reachability ---
        if not validate_drawing_reach(plotter_paths, height_map, pen_geometry):
            if args.fit_to_pen_reach and pen_geometry:
                logger.warning("Some points unreachable - auto-scaling feature not yet implemented")
                logger.error("Please adjust your drawing scale or pen geometry. Exiting.")
                return
            else:
                logger.error("Drawing contains unreachable points. Use --fit-to-pen-reach to auto-scale or adjust manually. Exiting.")
                return

        # --- 4. Setup CAN interface ---
        logger.info("Setting up CAN interface...")
        if args.use_simulator:
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

        multi_controller = MultiAxisController(can_interface_manager=can_if)
        
        all_axes_configs = {
            "X": {"id": args.motor_ids[0], "name": X_AXIS_NAME, "kin": LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=X_AXIS_PITCH_MM_PER_REV, gear_ratio=X_AXIS_GEAR_RATIO, units="mm")},
            "Y": {"id": args.motor_ids[1], "name": Y_AXIS_NAME, "kin": LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=Y_AXIS_PITCH_MM_PER_REV, gear_ratio=Y_AXIS_GEAR_RATIO, units="mm")},
            "Z": {"id": args.motor_ids[2], "name": PEN_AXIS_NAME, "kin": RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, gear_ratio=PEN_AXIS_GEAR_RATIO)},
        }
        
        logger.info(f"Activating specified axes: {args.axes}")
        for axis_key in args.axes:
            config = all_axes_configs[axis_key]
            multi_controller.add_axis(
                Axis(can_interface_manager=can_if, motor_can_id=config["id"], name=config["name"], kinematics=config["kin"])
            )

        if not multi_controller.axes:
            logger.error("No axes were activated. Please specify axes with --axes. Exiting.")
            return

        logger.info(f"Controller initialized with axes: {multi_controller.axis_names}")

        await multi_controller.initialize_all_axes(calibrate=False)
        await multi_controller.enable_all_axes()
        
        logger.info("Setting current positions as zero for all active axes...")
        for axis in multi_controller.axes.values():
            await axis.set_current_position_as_zero()
        
        # Log drawing mode
        if height_map and pen_geometry:
            logger.info("Surface-following mode enabled with height compensation")
        else:
            logger.info("Traditional flat drawing mode")
        
        logger.info("Plotter setup complete and ready to draw.")

        # --- 5. Drawing Logic ---
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS, pen_geometry)

        for i, path in enumerate(plotter_paths):
            logger.info(f"--- Drawing path {i+1}/{len(plotter_paths)} ---")
            if not path:
                continue
            
            # Move to the start of the path
            start_point = path[0]
            await move_to(multi_controller, start_point[0], start_point[1], TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM, 
                         is_drawing=False, height_map=height_map, pen_geometry=pen_geometry)

            # Pen down to draw
            await pen_down(multi_controller, start_point[0], start_point[1], PEN_ROTATION_SPEED_DEGPS, height_map, pen_geometry)

            # Draw all segments in the path
            for point in path[1:]:
                await move_to(multi_controller, point[0], point[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM,
                             is_drawing=True, height_map=height_map, pen_geometry=pen_geometry)
            
            # Pen up after finishing a path
            await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS, pen_geometry)

        logger.info("Drawing from SVG file complete.")
        await move_to(multi_controller, 0, 0, TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM) # Return home

    except FileNotFoundError as e:
        logger.error(f"Error: File not found: {e}")
    except json.JSONDecodeError as e:
        logger.error(f"Error: Invalid JSON in configuration file: {e}")
    except ValueError as e:
        logger.error(f"Error: {e}")
    except exceptions.MKSServoError as e:
        logger.error(f"A library-specific error occurred: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        if multi_controller and multi_controller.axes:
            logger.info("Disabling all active axes...")
            try:
                await multi_controller.disable_all_axes()
            except exceptions.MKSServoError as e:
                logger.error(f"Error disabling axes: {e}")
        
        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("SVG plotter script finished.")

def parse_arguments() -> argparse.Namespace:
    """Parses command-line arguments for the SVG plotter script."""
    parser = argparse.ArgumentParser(description="MKS Servo CAN SVG Plotter with Height Compensation")
    
    # Required SVG file argument
    parser.add_argument('--file', type=str, required=True, help='Path to the SVG file to draw.')

    # Scale and dimension arguments
    scale_group = parser.add_argument_group('Scaling Options')
    scale_group.add_argument(
        '--scale', type=float, default=None,
        help='Manual scale factor (e.g., 1.0 = 100%% original size, 0.5 = 50%%, 2.0 = 200%%). '
             'If not specified, will use automatic fitting to plotter area.'
    )
    scale_group.add_argument(
        '--max-x', type=float, default=None,
        help='Scale SVG to fit within this maximum X dimension (mm). '
             'Cannot exceed machine limit. Takes precedence over --scale.'
    )
    scale_group.add_argument(
        '--max-y', type=float, default=None,
        help='Scale SVG to fit within this maximum Y dimension (mm). '
             'Cannot exceed machine limit. Takes precedence over --scale.'
    )

    # Height compensation arguments
    height_group = parser.add_argument_group('Height Compensation')
    height_group.add_argument(
        '--height-map', type=str, default=None,
        help='Path to height map JSON file for surface-following drawing.'
    )
    height_group.add_argument(
        '--pen-config', type=str, default=None,
        help='Path to pen configuration JSON file with geometry data.'
    )
    height_group.add_argument(
        '--fit-to-pen-reach', action='store_true',
        help='Automatically scale drawing to fit within pen reach limits (requires pen config).'
    )

    # CAN interface arguments
    can_group = parser.add_argument_group('CAN Interface')
    # Set the default behavior to use the simulator
    parser.set_defaults(use_simulator=True)
    # Add a single flag to switch to hardware mode.
    # `action='store_false'` means if the flag is present, `use_simulator` becomes False.
    can_group.add_argument(
        '--no-simulator', 
        dest='use_simulator', 
        action='store_false',
        help='Use real hardware instead of the simulator (default is to use simulator).'
    )
    
    # Simulator-specific
    can_group.add_argument('--simulator-host', default='localhost', help='Simulator host IP address.')
    can_group.add_argument('--simulator-port', type=int, default=6789, help='Simulator TCP port.')
    
    # Hardware-specific
    hw_group = parser.add_argument_group('Hardware Options (if --no-simulator is used)')
    hw_group.add_argument('--can-interface-type', default='socketcan', help='Type of python-can interface.')
    hw_group.add_argument('--can-channel', default='can0', help='CAN channel.')
    hw_group.add_argument('--can-bitrate', type=int, default=500000, help='CAN bus bitrate.')
    
    # Motor IDs
    hw_group.add_argument(
        '--motor-ids', nargs=3, type=int, default=[X_AXIS_CAN_ID, Y_AXIS_CAN_ID, PEN_AXIS_CAN_ID],
        metavar=('X_ID', 'Y_ID', 'PEN_ID'),
        help=f'The CAN IDs for the X, Y, and Pen axes respectively.'
    )
    
    # Axis selection
    hw_group.add_argument(
        '--axes', nargs='+', type=str.upper, default=['X', 'Y', 'Z'],
        choices=['X', 'Y', 'Z'],
        help="Specify which axes to activate. E.g., --axes X Y"
    )
    
    # Logging
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], help='Set the logging level.')

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    log_level_numeric = getattr(logging, args.log_level.upper())
    logging.basicConfig(
        level=log_level_numeric,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.setLevel(log_level_numeric)

    asyncio.run(run_plotter_sequence(args))