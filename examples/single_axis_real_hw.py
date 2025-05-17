# mks_servo_can/examples/single_axis_real_hw.py
"""
Example: Controlling a single MKS Servo motor using a real CAN hardware interface.
"""
import asyncio
import logging

from mks_servo_can_library.mks_servo_can import Axis
from mks_servo_can_library.mks_servo_can import CANInterface
from mks_servo_can_library.mks_servo_can import const
from mks_servo_can_library.mks_servo_can import exceptions
from mks_servo_can_library.mks_servo_can import RotaryKinematics

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

# Hardware CAN interface configuration (MODIFY THESE FOR YOUR SETUP)
CAN_INTERFACE_TYPE = (
    "canable"  # e.g., 'canable', 'socketcan', 'kvaser', 'pcan', 'usb2can'
)
CAN_CHANNEL = (
    "/dev/ttyACM0"  # e.g., 'slcan0', '/dev/ttyUSB0', 'PCAN_USBBUS1', 'can0'
)
# For canable on Linux, it's often /dev/ttyACMx if using slcan firmware
# Or use gs_usb kernel module and 'can0' (need to bring up with ip link)
CAN_BITRATE = 500000  # Must match motor's CAN bitrate setting

MOTOR_CAN_ID = 1  # CAN ID of your motor


async def main():
    logging.info("Starting single axis real hardware example...")

    # Initialize CANInterface for real hardware
    # Ensure your CAN adapter is properly set up on your system.
    # For canable with slcan firmware:
    # You might need to ensure the serial port has correct settings if not auto-detected.
    # Or, if using gs_usb driver for canable:
    # sudo modprobe gs_usb
    # sudo ip link set can0 type can bitrate 500000
    # sudo ip link set can0 up
    # Then use interface_type='socketcan', channel='can0'

    can_if = CANInterface(
        interface_type=CAN_INTERFACE_TYPE,
        channel=CAN_CHANNEL,
        bitrate=CAN_BITRATE,
        use_simulator=False,
    )

    try:
        await can_if.connect()
        logging.info("CAN Interface connected successfully.")
    except exceptions.CANError as e:
        logging.error(f"Failed to connect to CAN interface: {e}")
        logging.error(
            "Please ensure your CAN adapter is connected, configured, and the motor is powered."
        )
        logging.error(
            "Check CAN_INTERFACE_TYPE, CAN_CHANNEL, and CAN_BITRATE settings."
        )
        return
    except Exception as e:
        logging.error(
            f"An unexpected error occurred during CAN connection: {e}"
        )
        return

    # Define kinematics (e.g., simple rotary, encoder pulses per revolution)
    # MKS Servos have high-resolution encoders (e.g., 16384 pulses/rev)
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )  # Using encoder pulses directly

    # Create an Axis
    axis1 = Axis(
        can_interface_manager=can_if,
        motor_can_id=MOTOR_CAN_ID,
        name="MyMotorAxis",
        kinematics=kin,
    )

    try:
        # --- Initialize Axis (IMPORTANT: Be careful with real hardware) ---
        # Before running, ensure motor is free to move and properly mounted.
        logging.info("Initializing axis... (This might involve communication)")
        await axis1.initialize(
            calibrate=False, home=False
        )  # Initial comms check
        # Set calibrate=True only if needed and safe.
        # Set home=True only if limits are set and safe.

        # --- Basic Operations ---
        logging.info("Enabling motor...")
        await axis1.enable_motor()
        assert axis1.is_enabled() is True, "Motor should be enabled"

        logging.info("Getting current position...")
        pos_user = await axis1.get_current_position_user()
        logging.info(
            f"Current position: {pos_user:.2f} {axis1.kinematics.units if hasattr(axis1.kinematics, 'units') else 'degrees'}"
        )

        # --- Perform a simple move (USE WITH CAUTION ON REAL HARDWARE) ---
        # Ensure the move is safe and within mechanical limits.
        # target_angle_degrees = pos_user + 30.0 # Move 30 degrees from current
        # logging.info(f"Moving to {target_angle_degrees:.2f} degrees...")
        # await axis1.move_to_position_abs_user(target_angle_degrees, speed_user=90.0, wait=True) # 90 deg/s
        # logging.info(f"Move complete. New position: {await axis1.get_current_position_user():.2f} degrees")

        # await asyncio.sleep(1)

        # logging.info("Moving back by 30 degrees (relative)...")
        # await axis1.move_relative_user(-30.0, speed_user=90.0, wait=True)
        # logging.info(f"Move complete. New position: {await axis1.get_current_position_user():.2f} degrees")

        # --- Example: Read some parameters (ensure motor supports reading these) ---
        # try:
        #     work_mode_data = await axis1.read_parameter(const.CMD_SET_WORK_MODE) # 0x82
        #     # work_mode is usually 1 byte in the response data (excluding cmd code and CRC)
        #     if work_mode_data:
        #         current_work_mode = work_mode_data[0]
        #         logging.info(f"Current Work Mode parameter value: 0x{current_work_mode:02X} ({const.WORK_MODES.get(current_work_mode, 'Unknown')})")
        # except exceptions.MotorError as e:
        #     logging.warning(f"Could not read work mode or parameter not readable: {e}")

        logging.info("Example finished. Disabling motor.")
        await axis1.disable_motor()

    except exceptions.MKSServoError as e:
        logging.error(f"An MKS Servo library error occurred: {e}")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        logging.info("Disconnecting CAN Interface...")
        await can_if.disconnect()
        logging.info("Program terminated.")


if __name__ == "__main__":
    # To run on systems where default event loop policy might cause issues with serial,
    # esp. on Windows with Prolific drivers, sometimes fixes are needed.
    # For most Linux/macOS or if using gs_usb, this is often not an issue.
    # if sys.platform.startswith("win"):
    # asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())
