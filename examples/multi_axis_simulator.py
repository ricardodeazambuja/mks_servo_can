# mks_servo_can/examples/multi_axis_simulator.py
"""
Example: Controlling multiple MKS Servo motors using the Simulator.
Make sure the simulator is running before executing this script.
(e.g., `mks-servo-simulator --num-motors 2 --start-can-id 1`)
"""
import asyncio
import logging

from mks_servo_can import Axis
from mks_servo_can import CANInterface
from mks_servo_can import const
from mks_servo_can import exceptions
from mks_servo_can import MultiAxisController
from mks_servo_can import RotaryKinematics

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789  # Default simulator port

# Define CAN IDs for simulated motors (must match simulator setup)
MOTOR_CAN_ID_1 = 1
MOTOR_CAN_ID_2 = 2


async def setup_and_connect_can_interface() -> CANInterface:
    """Sets up and connects the CAN interface to the simulator."""
    logger.info("Setting up CAN interface for simulator...")
    can_if_sim = CANInterface(
        use_simulator=True,
        simulator_host=SIMULATOR_HOST,
        simulator_port=SIMULATOR_PORT,
    )
    try:
        await can_if_sim.connect()
        logger.info("Connected to Simulator successfully.")
        return can_if_sim
    except exceptions.SimulatorError as e:
        logger.error("Failed to connect to simulator: %s", e)
        logger.error(
            "Please ensure the simulator is running at %s:%s "
            "with at least %d motors configured (e.g., --num-motors 2).",
            SIMULATOR_HOST,
            SIMULATOR_PORT,
            MOTOR_CAN_ID_2,
        )
        raise


def create_axes_and_controller(
    can_if: CANInterface,
) -> MultiAxisController:
    """Creates Axis instances and the MultiAxisController."""
    logger.info("Creating axes and multi-axis controller...")
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )

    axis1 = Axis(
        can_if,
        MOTOR_CAN_ID_1,
        "SimMotor1",
        kinematics=kin,
        default_speed_param=1000,
        default_accel_param=150,
    )
    axis2 = Axis(
        can_if,
        MOTOR_CAN_ID_2,
        "SimMotor2",
        kinematics=kin,
        default_speed_param=800,
        default_accel_param=100,
    )

    multi_controller = MultiAxisController(can_if)
    multi_controller.add_axis(axis1)
    multi_controller.add_axis(axis2)
    logger.info("Axes and controller created.")
    return multi_controller


async def perform_motor_operations(multi_controller: MultiAxisController):
    """Performs a sequence of operations on the motors."""
    logger.info("Initializing all simulated axes...")
    await multi_controller.initialize_all_axes(
        calibrate=False, home=False, concurrent=True
    )

    logger.info("Enabling all axes...")
    await multi_controller.enable_all_axes()

    initial_positions = await multi_controller.get_all_positions_user()
    logger.info("Initial positions: %s", initial_positions)

    target_positions = {
        "SimMotor1": 90.0,
        "SimMotor2": -45.0,
    }
    target_speeds = {
        "SimMotor1": 180.0,
        "SimMotor2": 90.0,
    }
    logger.info(
        "Moving all axes to: %s with speeds %s",
        target_positions,
        target_speeds,
    )
    await multi_controller.move_all_to_positions_abs_user(
        target_positions, speeds_user=target_speeds, wait_for_all=True
    )
    logger.info("Multi-axis move complete.")

    final_positions = await multi_controller.get_all_positions_user()
    logger.info("Final positions: %s", final_positions)

    for name, target in target_positions.items():
        if name in final_positions:
            logger.info(
                "Axis '%s': Target=%.1f, Actual=%.1f",
                name,
                target,
                final_positions[name],
            )
            assert (
                abs(final_positions[name] - target) < 1.0
            ), f"Axis {name} did not reach target."

    await asyncio.sleep(1)

    relative_distances = {"SimMotor1": -30.0, "SimMotor2": 15.0}
    logger.info("Moving all axes relatively by: %s", relative_distances)
    await multi_controller.move_all_relative_user(
        relative_distances, wait_for_all=True
    )

    current_positions = await multi_controller.get_all_positions_user()
    logger.info("Positions after relative move: %s", current_positions)

    logger.info("Disabling all axes...")
    await multi_controller.disable_all_axes()

    logger.info("Motor operations finished successfully.")


async def main():
    """Main execution function for the multi-axis simulator example."""
    logger.info("Starting multi-axis simulator example...")
    can_if_sim = None
    try:
        can_if_sim = await setup_and_connect_can_interface()
        multi_controller = create_axes_and_controller(can_if_sim)
        await perform_motor_operations(multi_controller)
        logger.info("Example finished successfully.")

    except exceptions.SimulatorError:
        logger.info("Exiting due to simulator connection failure.")
    except exceptions.MultiAxisError as e: # Catch MultiAxisError specifically
        logger.error("A Multi-Axis MKS Servo library error occurred: %s", e)
        # The E1101 was for this line (original 169)
        if e.individual_errors: # pylint: disable=no-member
            for axis_name, err in e.individual_errors.items():
                logger.error("  Error for axis '%s': %s", axis_name, err)
    except exceptions.MKSServoError as e:
        logger.error("An MKS Servo library error occurred: %s", e)
    except Exception as e:  # pylint: disable=broad-except
        logger.error("An unexpected error occurred: %s", e, exc_info=True)
    finally:
        if can_if_sim:
            logger.info("Disconnecting from Simulator...")
            await can_if_sim.disconnect()
        logger.info("Program terminated.")


if __name__ == "__main__":
    asyncio.run(main())
    