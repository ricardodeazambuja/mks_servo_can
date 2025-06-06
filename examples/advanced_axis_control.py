"""
Example: Demonstrating Advanced Axis Control Features.

This script showcases some of the less-common but powerful methods of the Axis class,
such as changing motor parameters, using velocity mode, and handling non-blocking moves.

To run with the simulator:
Ensure the MKS Servo Simulator is running.
Example: `mks-servo-simulator --num-motors 1 --start-can-id 1`
"""

import asyncio
import logging

from mks_servo_can import (
    CANInterface,
    Axis,
    RotaryKinematics,
    const,
    exceptions
)

# --- Configuration ---
LOG_LEVEL = logging.INFO
USE_SIMULATOR = True
SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789
MOTOR_CAN_ID = 1

# --- Logging Setup ---
logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("AdvancedAxisExample")


async def main():
    """
    Main asynchronous function to demonstrate advanced Axis control.
    """
    can_if = None
    logger.info("Starting Advanced Axis Control example...")

    try:
        # --- 1. Setup and Connect ---
        can_if = CANInterface(use_simulator=USE_SIMULATOR, simulator_host=SIMULATOR_HOST, simulator_port=SIMULATOR_PORT)
        await can_if.connect()
        logger.info("CAN Interface connected.")

        kinematics = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
        axis = Axis(can_if, MOTOR_CAN_ID, name="AdvancedAxis", kinematics=kinematics)

        await axis.initialize(calibrate=False)
        await axis.enable_motor()
        logger.info(f"Axis '{axis.name}' initialized and enabled.")

        # --- 2. Change Motor Parameters ---
        logger.info("Demonstrating motor parameter changes...")
        # Note: Some modes may require a restart. This is a demonstration of the command.
        await axis.set_work_mode(const.MODE_SR_VFOC)
        logger.info(f"Set work mode to SR_VFOC (Serial, Field-Oriented Control).")

        # Change motor microstepping (subdivision). This affects pulse-based commands.
        # The default in the Axis class is 16. Let's change it to 32.
        await axis.set_motor_subdivision(32)
        logger.info(f"Set motor subdivision to {axis.mstep_value}.")
        logger.info(f"Microsteps per revolution for pulse commands is now: {axis._microsteps_per_motor_revolution_for_cmd}")

        await asyncio.sleep(1)

        # --- 3. Velocity Mode (Continuous Rotation) ---
        logger.info("\n--- Demonstrating Velocity (Speed) Mode ---")
        logger.info("Setting motor to run at 120 deg/s...")
        await axis.set_speed_user(120.0) # degrees per second

        # Let it run for a couple of seconds and poll status
        for i in range(4):
            await asyncio.sleep(0.5)
            speed_rpm = await axis.get_current_speed_rpm()
            speed_user = await axis.get_current_speed_user()
            status_dict = await axis.get_status_dict()
            logger.info(f"  Running... Current Speed: {speed_rpm} RPM ({speed_user:.2f} deg/s)")
            logger.debug(f"  Full status: {status_dict['motor_status_str']}")

        logger.info("Stopping motor with controlled deceleration...")
        await axis.stop_motor()
        logger.info("Motor stopped.")
        await asyncio.sleep(1)

        # --- 4. Non-Blocking Move and Wait ---
        logger.info("\n--- Demonstrating Non-Blocking Move ---")
        logger.info("Starting a 360-degree move with wait=False...")
        # This call returns immediately after dispatching the command
        await axis.move_relative_user(360.0, speed_user=360.0, wait=False)

        # We can perform other work while the motor is moving
        for i in range(4):
            if not axis.is_move_complete():
                logger.info(f"  Motor is moving... (Other work can happen here)")
                await asyncio.sleep(0.3)
            else:
                logger.info("  Move completed faster than expected!")
                break
        
        # Now, explicitly wait for the move to finish if it hasn't already
        if not axis.is_move_complete():
            logger.info("Waiting for move completion...")
            await axis.wait_for_move_completion(timeout=5.0)
            logger.info("Move completion confirmed.")

        final_pos = await axis.get_current_position_user()
        logger.info(f"Final position after non-blocking move: {final_pos:.2f} degrees")
        await asyncio.sleep(1)

        # --- 5. Emergency Stop ---
        logger.info("\n--- Demonstrating Emergency Stop ---")
        logger.info("Starting a fast continuous move to be interrupted...")
        await axis.set_speed_user(720.0) # High speed
        await asyncio.sleep(0.5) # Let it get up to speed

        logger.warning("Issuing EMERGENCY STOP!")
        await axis.emergency_stop()
        status_code = await axis.get_motor_status_code()
        logger.info(f"Motor status after E-Stop: {const.MOTOR_STATUS_MAP.get(status_code, 'Unknown')}")

        # The motor will likely be disabled after an E-Stop, re-enable it for next command
        await axis.enable_motor()


    except exceptions.MKSServoCANError as e:
        logger.error(f"A mks-servo-can library error occurred: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        if axis and axis.is_enabled():
            logger.info("Disabling axis...")
            await axis.disable_motor()
        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("Advanced Axis Control example finished.")


if __name__ == "__main__":
    asyncio.run(main())
    