"""
Example: Controlling a 3-Axis Plotter to draw an SVG file.

This advanced script demonstrates how to parse an SVG vector file, process its
paths and shapes, scale them to fit the plotter's physical dimensions, and
then draw the result using the mks-servo-can library.

It relies on the 'svgelements' library for robust SVG parsing.
Please install it before running:
`pip install svgelements`

To run with the simulator:
1. Start the simulator with at least 3 motors:
   `mks-servo-simulator --num-motors 3 --start-can-id 1`
2. Run this script with a path to an SVG file:
   `python svg_plotter.py --file path/to/your/drawing.svg`

To run with real hardware:
1. Configure your CAN adapter, motor CAN IDs, and physical parameters below.
2. Run the script with the appropriate hardware flags:
   `python svg_plotter.py --file path/to/your/drawing.svg --no-simulator --can-channel /dev/ttyACM0`
"""

import asyncio
import logging
import math
import argparse
from typing import List, Tuple

# This script requires the 'svgelements' library for SVG parsing.
# Install it using: pip install svgelements
try:
    from svgelements import SVG, Path, Shape
except ImportError:
    print("This script requires the 'svgelements' library.")
    print("Please install it using: pip install svgelements")
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
# These values define the default physical setup of your plotter.
# They can be overridden by command-line arguments.

# --- Motor CAN IDs ---
X_AXIS_CAN_ID = 1
Y_AXIS_CAN_ID = 2
PEN_AXIS_CAN_ID = 3

# --- Plotter Physical Limits (in mm) ---
PLOTTER_MAX_X_MM = 200.0
PLOTTER_MAX_Y_MM = 150.0

# --- Plotter Speeds ---
DRAWING_SPEED_MMS = 25.0
TRAVEL_SPEED_MMS = 100.0
PEN_ROTATION_SPEED_DEGPS = 360.0

# --- X-Axis Kinematics Configuration ---
X_AXIS_PITCH_MM_PER_REV = 8.0
X_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
X_AXIS_GEAR_RATIO = 1.0

# --- Y-Axis Kinematics Configuration ---
Y_AXIS_PITCH_MM_PER_REV = 8.0
Y_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
Y_AXIS_GEAR_RATIO = 1.0

# --- Pen (Z-Axis) Kinematics Configuration ---
PEN_UP_ANGLE = 0.0
PEN_DOWN_ANGLE = 45.0
PEN_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
PEN_AXIS_GEAR_RATIO = 1.0
# ===================================================================

# --- Axis Names (used internally in the script) ---
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
PEN_AXIS_NAME = "PenAxis"

# --- Logging Setup ---
logger = logging.getLogger("SVGPlotter")


def process_svg(filepath: str, max_plot_x: float, max_plot_y: float) -> List[List[Tuple[float, float]]]:
    """
    Loads, parses, and processes an SVG file into plotter-ready coordinates.

    This function handles:
    1. Parsing all shapes from the SVG.
    2. Converting shapes and curved paths into a series of straight line segments.
    3. Scaling the entire drawing to fit within the plotter's physical bounds
       while maintaining aspect ratio.
    4. Transforming coordinates from SVG's top-left origin to the plotter's
       bottom-left origin.

    Args:
        filepath: The path to the SVG file.
        max_plot_x: The maximum drawable width of the plotter (mm).
        max_plot_y: The maximum drawable height of the plotter (mm).

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

    # Convert all shapes to Path objects and get their total bounds
    all_paths = [Path(elem) if not isinstance(elem, Path) else elem for elem in elements if isinstance(elem, Shape)]
    if not all_paths:
        logger.warning("Could not extract any valid paths from the SVG elements.")
        return []
        
    master_path = Path(*all_paths)
    bbox = master_path.bbox()
    if not bbox:
        logger.warning("Could not extract any valid bbox from the master path.")
        return []
    xmin, ymin, xmax, ymax = bbox
    
    svg_width = xmax - xmin
    svg_height = ymax - ymin

    if svg_width == 0 or svg_height == 0:
        logger.warning("SVG content has zero width or height. Cannot scale.")
        return []

    # Calculate scaling factor to fit drawing within plotter area
    scale_x = max_plot_x / svg_width
    scale_y = max_plot_y / svg_height
    scale = min(scale_x, scale_y) * 0.95  # Use 95% of area for a small margin

    logger.info(f"SVG bounds: ({xmin:.1f}, {ymin:.1f}) to ({xmax:.1f}, {ymax:.1f})")
    logger.info(f"SVG size: {svg_width:.1f} x {svg_height:.1f}. Plotter size: {max_plot_x} x {max_plot_y}")
    logger.info(f"Calculated uniform scale factor: {scale:.3f}")

    # Process each path
    for path in all_paths:
        # Linearize the path (convert curves to line segments)
        # The 'segments' parameter controls the number of lines per curve segment.
        linearized_path = []
        for segment in path.segments():
            # The first point of the first segment is the start point
            if not linearized_path:
                linearized_path.append(segment.start)
            linearized_path.append(segment.end)
        
        # Transform coordinates
        transformed_path = []
        for point in linearized_path:
            # Scale relative to the SVG's own origin (xmin, ymin)
            plot_x = (point.x - xmin) * scale
            # Scale and invert the Y-axis
            plot_y = max_plot_y - ((point.y - ymin) * scale)
            transformed_path.append((plot_x, plot_y))
        
        paths.append(transformed_path)
        
    logger.info(f"Successfully processed SVG into {len(paths)} drawable paths.")
    return paths

# --- Helper Functions for Plotter Control (same as calligraphy_plotter.py) ---

async def pen_up(controller: MultiAxisController, speed_degps: float):
    logger.info(f"Moving pen UP to {PEN_UP_ANGLE} degrees.")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        PEN_UP_ANGLE, speed_user=speed_degps, wait=True
    )

async def pen_down(controller: MultiAxisController, speed_degps: float):
    logger.info(f"Moving pen DOWN to {PEN_DOWN_ANGLE} degrees.")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        PEN_DOWN_ANGLE, speed_user=speed_degps, wait=True
    )

async def move_to(
    controller: MultiAxisController,
    target_x: float,
    target_y: float,
    speed_mms: float,
    max_x: float,
    max_y: float,
):
    if not (0 <= target_x <= max_x and 0 <= target_y <= max_y):
        logger.warning(f"Move to ({target_x:.1f}, {target_y:.1f}) is outside plotter limits. Skipping move.")
        return

    current_positions = await controller.get_all_positions_user()
    current_x = current_positions.get(X_AXIS_NAME, 0.0)
    current_y = current_positions.get(Y_AXIS_NAME, 0.0)
    
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    distance = math.sqrt(delta_x**2 + delta_y**2)

    if distance < 0.01: return

    logger.info(f"Moving to (X:{target_x:.2f}, Y:{target_y:.2f}) at {speed_mms:.1f} mm/s.")
    duration_s = distance / speed_mms
    speed_x_mms = abs(delta_x / duration_s) if duration_s > 0 else 0
    speed_y_mms = abs(delta_y / duration_s) if duration_s > 0 else 0

    await controller.move_all_to_positions_abs_user(
        positions_user={X_AXIS_NAME: target_x, Y_AXIS_NAME: target_y},
        speeds_user={X_AXIS_NAME: speed_x_mms, Y_AXIS_NAME: speed_y_mms},
        wait_for_all=True
    )


# --- Main Application Logic ---

async def run_plotter_sequence(args: argparse.Namespace):
    """
    The main logic sequence for the plotter. Sets up hardware/simulator,
    processes the SVG, and performs the drawing sequence.
    """
    can_if = None
    multi_controller = None

    try:
        # --- 1. Process SVG file first ---
        plotter_paths = process_svg(args.file, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        if not plotter_paths:
            logger.error("No drawable paths were generated from the SVG. Exiting.")
            return

        # --- 2. Setup CANInterface ---
        logger.info("Setting up CAN interface...")
        # ... (Setup logic remains the same as previous example)
        if args.use_simulator:
            can_if = CANInterface(use_simulator=True, simulator_host=args.simulator_host, simulator_port=args.simulator_port)
        else:
            can_if = CANInterface(interface_type=args.can_interface_type, channel=args.can_channel, bitrate=args.can_bitrate)
        await can_if.connect()
        logger.info("CAN Interface connected.")

        # --- 3. Setup Axes and Controller ---
        multi_controller = MultiAxisController(can_if)
        kin_x = LinearKinematics(X_AXIS_STEPS_PER_REV, X_AXIS_PITCH_MM_PER_REV, X_AXIS_GEAR_RATIO, "mm")
        kin_y = LinearKinematics(Y_AXIS_STEPS_PER_REV, Y_AXIS_PITCH_MM_PER_REV, Y_AXIS_GEAR_RATIO, "mm")
        kin_pen = RotaryKinematics(PEN_AXIS_STEPS_PER_REV, PEN_AXIS_GEAR_RATIO)
        
        multi_controller.add_axis(Axis(can_if, args.motor_ids[0], X_AXIS_NAME, kin_x))
        multi_controller.add_axis(Axis(can_if, args.motor_ids[1], Y_AXIS_NAME, kin_y))
        multi_controller.add_axis(Axis(can_if, args.motor_ids[2], PEN_AXIS_NAME, kin_pen))

        await multi_controller.initialize_all_axes()
        await multi_controller.enable_all_axes()
        for axis in multi_controller.axes.values():
            await axis.set_current_position_as_zero()
        logger.info("Plotter setup complete and ready to draw.")

        # --- 4. Drawing Logic ---
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)

        for i, path in enumerate(plotter_paths):
            logger.info(f"--- Drawing path {i+1}/{len(plotter_paths)} ---")
            if not path:
                continue
            
            # Move to the start of the path
            start_point = path[0]
            await move_to(multi_controller, start_point[0], start_point[1], TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)

            # Pen down to draw
            await pen_down(multi_controller, PEN_ROTATION_SPEED_DEGPS)

            # Draw all segments in the path
            for point in path[1:]:
                await move_to(multi_controller, point[0], point[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
            
            # Pen up after finishing a path
            await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)

        logger.info("Drawing from SVG file complete.")
        await move_to(multi_controller, 0, 0, TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM) # Return home

    except FileNotFoundError:
        logger.error(f"Error: The input file was not found at '{args.file}'")
    except exceptions.MKSServoError as e:
        logger.error(f"A library-specific error occurred: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # --- 5. Shutdown ---
        if multi_controller and multi_controller.axes:
            logger.info("Disabling all axes...")
            await multi_controller.disable_all_axes()
        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("SVG plotter script finished.")

def parse_arguments() -> argparse.Namespace:
    """Parses command-line arguments for the SVG plotter script."""
    parser = argparse.ArgumentParser(description="MKS Servo CAN SVG Plotter Example")
    
    # Required SVG file argument
    parser.add_argument('--file', type=str, required=True, help='Path to the SVG file to draw.')

    # Simulator vs. Hardware
    sim_group = parser.add_mutually_exclusive_group()
    sim_group.add_argument('--use-simulator', action='store_true', default=True, help='Use the simulator (default).')
    sim_group.add_argument('--no-simulator', action='store_false', dest='use_simulator', help='Use real hardware.')
    
    # Hardware-specific
    hw_group = parser.add_argument_group('Hardware Options (if not using simulator)')
    hw_group.add_argument('--can-interface-type', default='canable', help='Type of python-can interface (e.g., socketcan).')
    hw_group.add_argument('--can-channel', default='/dev/ttyACM0', help='CAN channel (e.g., can0, COM3).')
    hw_group.add_argument('--can-bitrate', type=int, default=500000, help='CAN bus bitrate.')
    
    # Motor IDs
    parser.add_argument(
        '--motor-ids', nargs=3, type=int, default=[X_AXIS_CAN_ID, Y_AXIS_CAN_ID, PEN_AXIS_CAN_ID],
        metavar=('X_ID', 'Y_ID', 'PEN_ID'),
        help=f'The CAN IDs for the X, Y, and Pen axes respectively.'
    )
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], help='Set logging level.')
    
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
    