"""
Example: Controlling a 3-Axis (X, Y, Pen-Z) Calligraphy Plotter.

This script demonstrates how to control a 3-axis plotter using the
mks-servo-can library. It coordinates two linear axes (X, Y) for movement
and one rotary axis (Pen) for lifting and lowering a pen.

This version has been updated to support flexible command-line arguments
for debugging, such as activating only a subset of axes.

To run with the simulator (default):
1. Start the simulator: `mks-servo-simulator --num-motors 3 --start-can-id 1`
2. Run this script: `python calligraphy_plotter_manual_interpolation.py`

To run with only X and Y axes:
   `python calligraphy_plotter_manual_interpolation.py --axes X Y`
"""

import asyncio
import logging
import math
import argparse
from typing import Dict, Optional

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
PLOTTER_MAX_Y_MM = 70 #160.0

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
PEN_DOWN_ANGLE = -35.0
PEN_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
PEN_AXIS_GEAR_RATIO = 1.0
# ===================================================================

# --- Axis Names (used internally in the script) ---
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
PEN_AXIS_NAME = "PenAxis"

# --- Logging Setup ---
logger = logging.getLogger("CalligraphyPlotter")


# --- Helper Functions for Plotter Control (Updated for robustness) ---

async def pen_up(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'up' position if the pen axis is active."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen up.")
        await asyncio.sleep(0.5) # Simulate time for a pen lift
        return
    logger.info(f"Moving pen UP to {PEN_UP_ANGLE} degrees.")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        PEN_UP_ANGLE, speed_user=speed_degps, wait=True
    )

async def pen_down(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'down' position if the pen axis is active."""
    if PEN_AXIS_NAME not in controller.axes:
        logger.info("Pen axis not active, skipping pen down.")
        await asyncio.sleep(0.5) # Simulate time for a pen drop
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
    """
    Moves the plotter's active X and Y axes to a target coordinate in a straight line.
    If an axis is not active, its movement is ignored.
    """
    # Software Limit Check
    if not (0 <= target_x <= max_x and 0 <= target_y <= max_y):
        logger.warning(
            f"Move to ({target_x:.1f}, {target_y:.1f}) is outside plotter limits "
            f"(X: 0-{max_x}, Y: 0-{max_y}). Skipping move."
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

async def run_plotter_sequence(args: argparse.Namespace):
    """
    The main logic sequence for the plotter. Sets up specified hardware/simulator
    and performs a drawing sequence.
    """
    can_if = None
    multi_controller = None

    try:
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

        # --- MODIFIED: Setup Axes based on CLI args ---
        multi_controller = MultiAxisController(can_interface_manager=can_if)
        
        all_axes_configs = {
            "X": {"id": args.motor_ids[0], "name": X_AXIS_NAME, "kin": LinearKinematics(X_AXIS_STEPS_PER_REV, X_AXIS_PITCH_MM_PER_REV, X_AXIS_GEAR_RATIO, "mm")},
            "Y": {"id": args.motor_ids[1], "name": Y_AXIS_NAME, "kin": LinearKinematics(Y_AXIS_STEPS_PER_REV, Y_AXIS_PITCH_MM_PER_REV, Y_AXIS_GEAR_RATIO, "mm")},
            "Z": {"id": args.motor_ids[2], "name": PEN_AXIS_NAME, "kin": RotaryKinematics(PEN_AXIS_STEPS_PER_REV, PEN_AXIS_GEAR_RATIO)},
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

        # --- Drawing Logic (remains the same) ---
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)
        p1, p2, p3, p4 = (10.0, 10.0), (60.0, 10.0), (60.0, 40.0), (10.0, 40.0)
        
        await move_to(multi_controller, p1[0], p1[1], TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await pen_down(multi_controller, PEN_ROTATION_SPEED_DEGPS)
        
        await move_to(multi_controller, p2[0], p2[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p3[0], p3[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p4[0], p4[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p1[0], p1[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        
        logger.info("Drawing complete.")
        
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)
        await move_to(multi_controller, 0, 0, TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        
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
        logger.info("Plotter example finished.")

def parse_arguments() -> argparse.Namespace:
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(description="MKS Servo CAN Calligraphy Plotter Example")
    
    # --- Hardware Mode Selection ---
    # Default behavior is to use the simulator. Use --hardware flag to enable real hardware.
    parser.add_argument(
        '--hardware', 
        action='store_true',
        help='Use real hardware instead of the simulator (default is to use simulator).'
    )
    
    parser.add_argument('--simulator-host', default='localhost', help='Simulator host IP address.')
    parser.add_argument('--simulator-port', type=int, default=6789, help='Simulator TCP port.')
    
    hw_group = parser.add_argument_group('Hardware Options (if --hardware is used)')
    hw_group.add_argument('--can-interface-type', default='socketcan', help='Type of python-can interface.')
    hw_group.add_argument('--can-channel', default='can0', help='CAN channel.')
    hw_group.add_argument('--can-bitrate', type=int, default=500000, help='CAN bus bitrate.')
    
    parser.add_argument(
        '--motor-ids', nargs=3, type=int, default=[X_AXIS_CAN_ID, Y_AXIS_CAN_ID, PEN_AXIS_CAN_ID],
        metavar=('X_ID', 'Y_ID', 'PEN_ID'),
        help=f'The CAN IDs for the X, Y, and Pen axes respectively.'
    )
    
    # --- ADDED: Flexible axis selection ---
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
