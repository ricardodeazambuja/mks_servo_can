"""
Example: Controlling a 3-Axis (X, Y, Pen-Z) Calligraphy Plotter.

This script demonstrates how to control a 3-axis plotter using the
mks-servo-can library. It coordinates two linear axes (X, Y) for movement
and one rotary axis (Pen) for lifting and lowering a pen.

The script is designed to be easily configurable for different physical setups
by adjusting the parameters in the "Plotter Configuration" section or by using
command-line arguments.

To run with the simulator:
1. Start the simulator with at least 3 motors:
   `mks-servo-simulator --num-motors 3 --start-can-id 1`
2. Run this script with default settings:
   `python calligraphy_plotter.py`

To run with real hardware:
1. Configure your CAN adapter, motor CAN IDs, and physical parameters below.
2. Run the script with the `--no-simulator` flag and other relevant CAN args:
   `python calligraphy_plotter.py --no-simulator --can-channel /dev/ttyACM0`
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
# The unique CAN ID for each motor on the bus.
X_AXIS_CAN_ID = 1
Y_AXIS_CAN_ID = 2
PEN_AXIS_CAN_ID = 3

# --- Plotter Physical Limits (in mm) ---
# Defines the usable drawing area. Moves outside this area will be prevented.
PLOTTER_MAX_X_MM = 200.0
PLOTTER_MAX_Y_MM = 150.0

# --- Plotter Speeds ---
# Speed for movements when the pen is down (drawing).
DRAWING_SPEED_MMS = 25.0
# Speed for movements when the pen is up (traveling).
TRAVEL_SPEED_MMS = 100.0
# Speed for the pen up/down rotation.
PEN_ROTATION_SPEED_DEGPS = 360.0

# --- X-Axis Kinematics Configuration ---
# Pitch: The linear distance (mm) the X-axis travels for one full revolution
# of its drive mechanism (e.g., lead screw pitch or belt drive pulley circumference).
X_AXIS_PITCH_MM_PER_REV = 8.0
# The native encoder resolution of the X-axis motor.
X_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
# The gear ratio between the X-axis motor and its drive mechanism.
X_AXIS_GEAR_RATIO = 1.0

# --- Y-Axis Kinematics Configuration ---
Y_AXIS_PITCH_MM_PER_REV = 8.0
Y_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
Y_AXIS_GEAR_RATIO = 1.0

# --- Pen (Z-Axis) Kinematics Configuration ---
PEN_UP_ANGLE = 0.0      # The angle (degrees) for the "pen up" position.
PEN_DOWN_ANGLE = 45.0   # The angle (degrees) for the "pen down" position.
PEN_AXIS_STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION
PEN_AXIS_GEAR_RATIO = 1.0
# ===================================================================

# --- Axis Names (used internally in the script) ---
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
PEN_AXIS_NAME = "PenAxis"

# --- Logging Setup ---
logger = logging.getLogger("CalligraphyPlotter")


# --- Helper Functions for Plotter Control ---

async def pen_up(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'up' position and waits."""
    logger.info(f"Moving pen UP to {PEN_UP_ANGLE} degrees.")
    await controller.axes[PEN_AXIS_NAME].move_to_position_abs_user(
        PEN_UP_ANGLE, speed_user=speed_degps, wait=True
    )

async def pen_down(controller: MultiAxisController, speed_degps: float):
    """Commands the pen to move to the 'down' position and waits."""
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
    Moves the plotter's X and Y axes to a target coordinate in a straight line.

    This function calculates the individual speeds for the X and Y axes to ensure
    the tool moves at a constant velocity along the path. It also checks for
    software limits before initiating the move.

    Args:
        controller: The MultiAxisController instance.
        target_x: The target X coordinate in mm.
        target_y: The target Y coordinate in mm.
        speed_mms: The desired tool speed along the path in mm/s.
        max_x: The maximum allowed X coordinate.
        max_y: The maximum allowed Y coordinate.
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
    current_x = current_positions.get(X_AXIS_NAME, 0.0)
    current_y = current_positions.get(Y_AXIS_NAME, 0.0)

    delta_x = target_x - current_x
    delta_y = target_y - current_y

    distance = math.sqrt(delta_x**2 + delta_y**2)

    if distance < 0.01:
        logger.info("Target is at current position. No move needed.")
        return

    # Calculate time required for the move
    duration_s = distance / speed_mms

    # Calculate individual speeds to maintain a straight line path
    speed_x_mms = abs(delta_x / duration_s) if duration_s > 0 else 0
    speed_y_mms = abs(delta_y / duration_s) if duration_s > 0 else 0

    positions_to_move = {X_AXIS_NAME: target_x, Y_AXIS_NAME: target_y}
    speeds_for_move = {X_AXIS_NAME: speed_x_mms, Y_AXIS_NAME: speed_y_mms}
    
    await controller.move_all_to_positions_abs_user(
        positions_user=positions_to_move,
        speeds_user=speeds_for_move,
        wait_for_all=True
    )

async def run_plotter_sequence(args: argparse.Namespace):
    """
    The main logic sequence for the plotter. Sets up hardware/simulator
    and performs a drawing sequence.
    """
    can_if = None
    multi_controller = None

    try:
        # --- 1. Setup CANInterface ---
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

        # --- 2. Setup Axes and MultiAxisController ---
        multi_controller = MultiAxisController(can_interface_manager=can_if)
        
        kin_x = LinearKinematics(X_AXIS_STEPS_PER_REV, X_AXIS_PITCH_MM_PER_REV, X_AXIS_GEAR_RATIO, "mm")
        multi_controller.add_axis(Axis(can_if, args.motor_ids[0], X_AXIS_NAME, kin_x))
        
        kin_y = LinearKinematics(Y_AXIS_STEPS_PER_REV, Y_AXIS_PITCH_MM_PER_REV, Y_AXIS_GEAR_RATIO, "mm")
        multi_controller.add_axis(Axis(can_if, args.motor_ids[1], Y_AXIS_NAME, kin_y))
        
        kin_pen = RotaryKinematics(PEN_AXIS_STEPS_PER_REV, PEN_AXIS_GEAR_RATIO)
        multi_controller.add_axis(Axis(can_if, args.motor_ids[2], PEN_AXIS_NAME, kin_pen))

        logger.info(f"Controller initialized with axes: {multi_controller.axis_names}")

        # --- 3. Initialize and Home Axes ---
        await multi_controller.initialize_all_axes(calibrate=False)
        await multi_controller.enable_all_axes()
        
        # Set all axes to zero at their current position for this example
        logger.info("Setting current positions as zero for all axes...")
        for axis in multi_controller.axes.values():
            await axis.set_current_position_as_zero()
        
        logger.info("Plotter setup complete and ready to draw.")

        # --- 4. Drawing Logic ---
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)

        # Define the corners of a 50mm x 30mm rectangle
        p1 = (10.0, 10.0)
        p2 = (60.0, 10.0)
        p3 = (60.0, 40.0)
        p4 = (10.0, 40.0)
        
        # Go to the starting point
        await move_to(multi_controller, p1[0], p1[1], TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        
        # Pen down to start drawing
        await pen_down(multi_controller, PEN_ROTATION_SPEED_DEGPS)
        
        # Draw the rectangle
        await move_to(multi_controller, p2[0], p2[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p3[0], p3[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p4[0], p4[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        await move_to(multi_controller, p1[0], p1[1], DRAWING_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        
        logger.info("Drawing complete.")
        
        # Pen up and return to origin
        await pen_up(multi_controller, PEN_ROTATION_SPEED_DEGPS)
        await move_to(multi_controller, 0, 0, TRAVEL_SPEED_MMS, PLOTTER_MAX_X_MM, PLOTTER_MAX_Y_MM)
        
    except exceptions.MKSServoError as e:
        logger.error(f"A library-specific error occurred: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # --- 5. Shutdown ---
        if multi_controller and multi_controller.axes:
            logger.info("Disabling all axes...")
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
    
    # Simulator vs. Hardware
    sim_group = parser.add_mutually_exclusive_group()
    sim_group.add_argument(
        '--use-simulator', action='store_true', default=None,
        help='Use the simulator instead of real hardware (default if no hardware args given).'
    )
    sim_group.add_argument(
        '--no-simulator', action='store_false', dest='use_simulator',
        help='Use real hardware instead of the simulator.'
    )
    
    # Simulator-specific
    parser.add_argument('--simulator-host', default='localhost', help='Simulator host IP address.')
    parser.add_argument('--simulator-port', type=int, default=6789, help='Simulator TCP port.')
    
    # Hardware-specific
    parser.add_argument('--can-interface-type', default='canable', help='Type of python-can interface (e.g., canable, socketcan).')
    parser.add_argument('--can-channel', default='/dev/ttyACM0', help='CAN channel (e.g., /dev/ttyACM0, can0).')
    parser.add_argument('--can-bitrate', type=int, default=500000, help='CAN bus bitrate.')
    
    # Motor IDs
    parser.add_argument(
        '--motor-ids', nargs=3, type=int, default=[X_AXIS_CAN_ID, Y_AXIS_CAN_ID, PEN_AXIS_CAN_ID],
        metavar=('X_ID', 'Y_ID', 'PEN_ID'),
        help=f'The CAN IDs for the X, Y, and Pen axes respectively. Default: {X_AXIS_CAN_ID} {Y_AXIS_CAN_ID} {PEN_AXIS_CAN_ID}'
    )
    
    parser.add_argument(
        '--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        help='Set the logging level.'
    )

    args = parser.parse_args()
    
    # If --use-simulator is not explicitly set, decide based on whether CAN hardware args were changed
    if args.use_simulator is None:
        args.use_simulator = not any([
            args.can_interface_type != 'canable',
            args.can_channel != '/dev/ttyACM0',
            args.can_bitrate != 500000
        ])
    
    return args

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
