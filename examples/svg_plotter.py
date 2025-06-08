"""
Example: Controlling a 3-Axis Plotter to draw an SVG file.

This advanced script demonstrates how to parse an SVG vector file, process its
paths and shapes, scale them to fit the plotter's physical dimensions, and
then draw the result using the mks-servo-can library.

It relies on the 'svgelements' library for robust SVG parsing.
Please install it before running:
`pip install svgelements`

By default, this script runs using the simulator. To use real hardware,
provide the `--no-simulator` flag and other relevant CAN arguments.

To run with the simulator (default):
1. Start the simulator: `mks-servo-simulator --num-motors 3 --start-can-id 1`
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
# These values can be adjusted to match your physical plotter setup.
# They can also be overridden by command-line arguments.

# --- Motor CAN IDs ---
X_AXIS_CAN_ID = 1
Y_AXIS_CAN_ID = 2
PEN_AXIS_CAN_ID = 3

# --- Plotter Physical Limits (in mm) ---
PLOTTER_MAX_X_MM = 130.0
PLOTTER_MAX_Y_MM = 160.0

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

    # Calculate scaling factor to fit drawing within plotter area
    scale_x = max_plot_x / svg_width
    scale_y = max_plot_y / svg_height
    scale = min(scale_x, scale_y) * 0.95  # Use 95% of area for a small margin

    logger.info(f"SVG bounds: ({xmin:.1f}, {ymin:.1f}) to ({xmax:.1f}, {ymax:.1f})")
    logger.info(f"SVG size: {svg_width:.1f} x {svg_height:.1f}. Plotter size: {max_plot_x} x {max_plot_y}")
    logger.info(f"Calculated uniform scale factor: {scale:.3f}")

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


# --- Helper Functions for Plotter Control ---

async def pen_up(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'up' position if the pen axis is active."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen up.")
        await asyncio.sleep(0.5)
        return
    logger.info(f"Moving pen UP to {PEN_UP_ANGLE} degrees.")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        PEN_UP_ANGLE, speed_user=speed_degps, wait=True
    )

async def pen_down(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'down' position if the pen axis is active."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen down.")
        await asyncio.sleep(0.5)
        return
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
    max_y: float
):
    """Moves the plotter's active X and Y axes to a target coordinate."""
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
    
    await controller.move_all_to_positions_abs_user(
        positions_user=positions_to_move,
        speeds_user=speeds_for_move,
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

        # --- 2. Setup CAN interface ---
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
        
        logger.info("Plotter setup complete and ready to draw.")

        # --- 3. Drawing Logic ---
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
    parser = argparse.ArgumentParser(description="MKS Servo CAN SVG Plotter Example")
    
    # Required SVG file argument
    parser.add_argument('--file', type=str, required=True, help='Path to the SVG file to draw.')

    # --- MODIFIED PART ---
    # Set the default behavior to use the simulator
    parser.set_defaults(use_simulator=True)
    # Add a single flag to switch to hardware mode.
    # `action='store_false'` means if the flag is present, `use_simulator` becomes False.
    parser.add_argument(
        '--no-simulator', 
        dest='use_simulator', 
        action='store_false',
        help='Use real hardware instead of the simulator (default is to use simulator).'
    )
    
    # Simulator-specific
    parser.add_argument('--simulator-host', default='localhost', help='Simulator host IP address.')
    parser.add_argument('--simulator-port', type=int, default=6789, help='Simulator TCP port.')
    
    # Hardware-specific
    hw_group = parser.add_argument_group('Hardware Options (if --no-simulator is used)')
    hw_group.add_argument('--can-interface-type', default='socketcan', help='Type of python-can interface.')
    hw_group.add_argument('--can-channel', default='can0', help='CAN channel.')
    hw_group.add_argument('--can-bitrate', type=int, default=500000, help='CAN bus bitrate.')
    
    # Motor IDs
    parser.add_argument(
        '--motor-ids', nargs=3, type=int, default=[X_AXIS_CAN_ID, Y_AXIS_CAN_ID, PEN_AXIS_CAN_ID],
        metavar=('X_ID', 'Y_ID', 'PEN_ID'),
        help=f'The CAN IDs for the X, Y, and Pen axes respectively.'
    )
    
    # New argument to select active axes
    parser.add_argument(
        '--axes', nargs='+', type=str.upper, default=['X', 'Y', 'Z'],
        choices=['X', 'Y', 'Z'],
        help="Specify which axes to activate. E.g., --axes X Y"
    )
    
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