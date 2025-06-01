# This script demonstrates how to control a single MKS Servo motor using a real, physical CAN hardware interface.
# It serves as a practical guide for users looking to integrate the library with actual hardware.

"""
Example: Controlling a single MKS Servo motor using a real CAN
hardware interface.
"""
# This is a module-level docstring explaining the purpose of the example script.
# It clarifies that the script is for demonstrating control over real hardware.

import asyncio
# Imports the 'asyncio' library, which is essential for using the 'mks_servo_can' library
# as it is built for asynchronous operations.

import logging
# Imports the 'logging' module to enable detailed logging of the script's execution,
# which is helpful for debugging and understanding the flow of operations.

from mks_servo_can import Axis
# Imports the 'Axis' class from the 'mks_servo_can' library.
# The 'Axis' class provides a high-level interface for controlling an individual motor.

from mks_servo_can import CANInterface
# Imports the 'CANInterface' class, responsible for managing the connection
# to the CAN bus (either real or simulated). In this example, it's for real hardware.

from mks_servo_can import const
# Imports the 'const' module (likely 'constants.py') from the library.
# This module contains predefined constants like default bitrates, command codes,
# and encoder pulse counts, which are used for configuring and controlling the motor.

from mks_servo_can import exceptions
# Imports the 'exceptions' module, which defines custom exception classes
# for handling errors specific to the 'mks_servo_can' library.

from mks_servo_can import RotaryKinematics
# Imports the 'RotaryKinematics' class, which is used to convert between
# physical angular units (like degrees) and the motor's native encoder steps.

# Configure logging
# This section sets up basic logging for the script.
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
# Configures the root logger:
# - 'level=logging.INFO': Sets the minimum logging level to INFO. Messages with INFO, WARNING, ERROR, CRITICAL will be shown.
# - 'format="%(asctime)s - %(levelname)s - %(message)s"': Defines the format for log messages,
#   including timestamp, log level, and the message itself.

logger = logging.getLogger(__name__)
# Creates a logger instance specifically for this module (__name__ will be 'mks_servo_can.examples.single_axis_real_hw').
# This allows for more granular control over logging if needed.

# Hardware CAN interface configuration (MODIFY THESE FOR YOUR SETUP)
# This is a crucial comment prompting the user to update the following constants
# to match their specific hardware setup.
CAN_INTERFACE_TYPE = (
    "socketcan"  # e.g., 'canable', 'socketcan', 'kvaser', 'pcan', 'usb2can'
)
# Defines the type of CAN adapter being used. 'canable' is a common choice.
# Other examples are provided, indicating compatibility with various 'python-can' backends.

CAN_CHANNEL = (
    "can0"  # e.g., 'slcan0', '/dev/ttyUSB0', 'PCAN_USBBUS1', 'can0'
)
# Defines the specific channel or port for the CAN adapter.
# For 'canable' with slcan firmware on Linux, this is often a serial port like '/dev/ttyACM0' or '/dev/ttyUSB0'.
# For SocketCAN on Linux, it might be 'can0'. For PCAN on Windows, it could be 'PCAN_USBBUS1'.
# This value is highly dependent on the user's OS and adapter.

# For canable on Linux, it's often /dev/ttyACMx if using slcan firmware
# Or use gs_usb kernel module and 'can0'
# (need to bring up with ip link)
# This provides more specific guidance for users with 'canable' adapters on Linux,
# mentioning options like slcan firmware or the gs_usb kernel module (which would use 'can0' via SocketCAN).
# It also hints at the need to configure the interface (e.g., `sudo ip link set can0 up type can bitrate 500000`).

CAN_BITRATE = 500000  # Must match motor's CAN bitrate setting
# Defines the CAN bus bitrate in bits per second.
# 500,000 bps (500K) is a common default for MKS servos.
# It's critical that this matches the bitrate configured on the MKS servo motors themselves (often via DIP switches).

MOTOR_CAN_ID = 1  # CAN ID of your motor
# Defines the CAN ID of the specific MKS servo motor this script will control.
# This ID must be unique on the CAN bus and match the ID set on the motor hardware.

async def main():
    # Defines the main asynchronous function where the primary logic of the script resides.
    # Being an 'async' function, it allows the use of 'await' for asynchronous operations.
    """Main function to control a single real hardware motor."""
    # Docstring for the main function.
    logger.info("Starting single axis real hardware example...")
    # Logs the start of the example.

    can_if = CANInterface(
        interface_type=CAN_INTERFACE_TYPE, # Passes the user-configured interface type.
        channel=CAN_CHANNEL, # Passes the user-configured channel.
        bitrate=CAN_BITRATE, # Passes the user-configured bitrate.
        use_simulator=False, # Explicitly states that this is for real hardware, not the simulator.
    )
    # Creates an instance of 'CANInterface', configured to connect to physical CAN hardware.

    try:
        # This 'try' block attempts to connect to the CAN interface.
        await can_if.connect()
        # Asynchronously establishes the connection to the CAN bus.
        logger.info("CAN Interface connected successfully.")
        # Logs a successful connection.
    except exceptions.CANError as e:
        # Catches 'CANError' if the connection to the hardware fails.
        # 'CANError' is a specific exception from the library for CAN-related issues.
        logger.error("Failed to connect to CAN interface: %s", e)
        # Logs the error.
        logger.error(
            "Please ensure your CAN adapter is connected, configured, "
            "and the motor is powered."
        )
        # Provides common troubleshooting advice.
        logger.error(
            "Check CAN_INTERFACE_TYPE, CAN_CHANNEL, and CAN_BITRATE settings."
        )
        # Reminds the user to check their configuration constants.
        return # Exits the main function if connection fails.
    except Exception as e:  # pylint: disable=broad-except
        # Catches any other unexpected exceptions during connection.
        # disable=broad-except is used to acknowledge that catching generic 'Exception'
        # is sometimes necessary at a high level but should generally be more specific.
        logger.error(
            "An unexpected error occurred during CAN connection: %s", e
        )
        # Logs the unexpected error.
        return # Exits.

    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        # Default encoder resolution for MKS servos (16384).
    )
    # Creates a 'RotaryKinematics' object. This assumes the motor performs direct rotary motion
    # and the user wants to control it in degrees.
    # 'const.ENCODER_PULSES_PER_REVOLUTION' is the motor's native steps for one revolution.

    axis1 = Axis(
        can_interface_manager=can_if, # Passes the connected CANInterface instance.
        motor_can_id=MOTOR_CAN_ID, # Specifies the CAN ID of the target motor.
        name="MyMotorAxis", # Assigns a descriptive name to this axis instance.
        kinematics=kin, # Assigns the kinematics object for unit conversions.
    )
    # Creates an 'Axis' instance to represent and control the motor.

    try:
        # This 'try' block contains the motor control operations.
        logger.info(
            "Initializing axis... (This might involve communication)"
        )
        # Logs the start of axis initialization.
        await axis1.initialize(
            calibrate=False, home=False # Initializes the axis without performing calibration or homing.
                                         # Initialization usually pings the motor and reads initial status.
        )
        # Initializes the axis. This typically involves sending some commands to check
        # connectivity and retrieve initial motor state.

        logger.info("Enabling motor...")
        # Logs the intent to enable the motor.
        await axis1.enable_motor()
        # Sends the command to enable the motor (servo on).
        assert axis1.is_enabled() is True, "Motor should be enabled"
        # Checks the cached enabled state of the axis and asserts it's True.
        # For a definitive check, one might call 'await axis1.read_en_status()'.

        logger.info("Getting current position...")
        # Logs the intent to read the current position.
        pos_user = await axis1.get_current_position_user()
        # Reads the motor's current position in user units (degrees, due to RotaryKinematics).
        kin_units = (
            axis1.kinematics.units # Accesses the 'units' attribute from the kinematics object.
            if hasattr(axis1.kinematics, "units") # Checks if 'units' attribute exists.
            else "degrees" # Fallback if 'units' is not found (though it should be for standard kinematics).
        )
        # Retrieves the units string from the kinematics object for logging.
        logger.info("Current position: %.2f %s", pos_user, kin_units)
        # Logs the current position.

        # --- Example move ---
        # This section demonstrates a simple absolute move, but it's commented out by default.
        # Users can uncomment it to test actual movement.
        # target_angle_degrees = pos_user + 30.0
        # Calculates a target angle 30 degrees from the current position.
        # logger.info("Moving to %.2f degrees...", target_angle_degrees)
        # Logs the target move.
        # await axis1.move_to_position_abs_user(
        # target_angle_degrees, speed_user=90.0, wait=True
        # )
        # Commands the motor to move to the 'target_angle_degrees' at a speed of 90.0 degrees/second.
        # 'wait=True' means the function will await the completion of the move.
        # new_pos_log = await axis1.get_current_position_user()
        # Reads the position again after the move.
        # logger.info("Move complete. New position: %.2f %s",
        # new_pos_log, kin_units)
        # Logs the new position.

        logger.info("Example finished. Disabling motor.")
        # Logs that the example operations are done and the motor will be disabled.
        await axis1.disable_motor()
        # Sends the command to disable the motor (servo off).

    except exceptions.MKSServoError as e:
        # Catches any specific exceptions from the 'mks_servo_can' library.
        logger.error("An MKS Servo library error occurred: %s", e)
        # Logs the library-specific error.
    except Exception as e:  # pylint: disable=broad-except
        # Catches any other unexpected exceptions.
        logger.error(
            "An unexpected error occurred: %s", e, exc_info=True # 'exc_info=True' includes traceback information.
        )
        # Logs the unexpected error with traceback.
    finally:
        # The 'finally' block ensures that cleanup (disconnecting from CAN) happens
        # regardless of whether errors occurred or not.
        logger.info("Disconnecting CAN Interface...")
        # Logs the intent to disconnect.
        await can_if.disconnect()
        # Asynchronously disconnects from the CAN bus.
        logger.info("Program terminated.")
        # Logs the end of the script.

if __name__ == "__main__":
    # This standard Python construct checks if the script is being run directly
    # (not imported as a module into another script).
    asyncio.run(main())
    # Runs the main asynchronous function using 'asyncio.run()'.
    # This starts the asyncio event loop, runs 'main()', and handles loop cleanup.
    