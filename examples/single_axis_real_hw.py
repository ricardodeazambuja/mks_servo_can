# mks_servo_can/examples/single_axis_real_hw.py
"""
Example: Controlling a single MKS Servo motor using a real CAN
hardware interface.
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
logger = logging.getLogger(__name__)

# Hardware CAN interface configuration (MODIFY THESE FOR YOUR SETUP)
CAN_INTERFACE_TYPE = (
    "canable"  # e.g., 'canable', 'socketcan', 'kvaser', 'pcan', 'usb2can'
)
CAN_CHANNEL = (
    "/dev/ttyACM0"  # e.g., 'slcan0', '/dev/ttyUSB0', 'PCAN_USBBUS1', 'can0'
)
# For canable on Linux, it's often /dev/ttyACMx if using slcan firmware
# Or use gs_usb kernel module and 'can0'
# (need to bring up with ip link)
CAN_BITRATE = 500000  # Must match motor's CAN bitrate setting

MOTOR_CAN_ID = 1  # CAN ID of your motor


async def main():
    """Main function to control a single real hardware motor."""
    logger.info("Starting single axis real hardware example...")

    can_if = CANInterface(
        interface_type=CAN_INTERFACE_TYPE,
        channel=CAN_CHANNEL,
        bitrate=CAN_BITRATE,
        use_simulator=False,
    )

    try:
        await can_if.connect()
        logger.info("CAN Interface connected successfully.")
    except exceptions.CANError as e:
        logger.error("Failed to connect to CAN interface: %s", e)
        logger.error(
            "Please ensure your CAN adapter is connected, configured, "
            "and the motor is powered."
        )
        logger.error(
            "Check CAN_INTERFACE_TYPE, CAN_CHANNEL, and CAN_BITRATE settings."
        )
        return
    except Exception as e:  # pylint: disable=broad-except
        logger.error(
            "An unexpected error occurred during CAN connection: %s", e
        )
        return

    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )

    axis1 = Axis(
        can_interface_manager=can_if,
        motor_can_id=MOTOR_CAN_ID,
        name="MyMotorAxis",
        kinematics=kin,
    )

    try:
        logger.info(
            "Initializing axis... (This might involve communication)"
        )
        await axis1.initialize(
            calibrate=False, home=False
        )

        logger.info("Enabling motor...")
        await axis1.enable_motor()
        assert axis1.is_enabled() is True, "Motor should be enabled"

        logger.info("Getting current position...")
        pos_user = await axis1.get_current_position_user()
        kin_units = (
            axis1.kinematics.units
            if hasattr(axis1.kinematics, "units")
            else "degrees"
        )
        logger.info("Current position: %.2f %s", pos_user, kin_units)

        # --- Example move ---
        # target_angle_degrees = pos_user + 30.0
        # logger.info("Moving to %.2f degrees...", target_angle_degrees)
        # await axis1.move_to_position_abs_user(
        # target_angle_degrees, speed_user=90.0, wait=True
        # )
        # new_pos_log = await axis1.get_current_position_user()
        # logger.info("Move complete. New position: %.2f %s",
        # new_pos_log, kin_units)

        logger.info("Example finished. Disabling motor.")
        await axis1.disable_motor()

    except exceptions.MKSServoError as e:
        logger.error("An MKS Servo library error occurred: %s", e)
    except Exception as e:  # pylint: disable=broad-except
        logger.error(
            "An unexpected error occurred: %s", e, exc_info=True
        )
    finally:
        logger.info("Disconnecting CAN Interface...")
        await can_if.disconnect()
        logger.info("Program terminated.")


if __name__ == "__main__":
    asyncio.run(main())
    