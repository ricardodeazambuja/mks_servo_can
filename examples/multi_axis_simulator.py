# mks_servo_can/examples/multi_axis_simulator.py
# This line indicates the path to an example script.
# This script demonstrates how to control multiple MKS Servo motors simultaneously
# using the mks-servo-simulator. It showcases the MultiAxisController for coordinated actions.

"""
Example: Controlling multiple MKS Servo motors using the Simulator.
Make sure the simulator is running before executing this script.
(e.g., `mks-servo-simulator --num-motors 2 --start-can-id 1`)
"""
# This is a module-level docstring explaining the purpose of the script.
# It emphasizes that this example uses the simulator and requires it to be running
# with a specific configuration (at least two motors).

import asyncio
# Imports the 'asyncio' library, essential for using the asynchronous features
# of the 'mks_servo_can' library.

import logging
# Imports the 'logging' module to provide informative output during script execution.

from mks_servo_can import Axis
# Imports the 'Axis' class, used to represent and control individual motors.

from mks_servo_can import CANInterface
# Imports the 'CANInterface' class, used to establish a connection (in this case, to the simulator).

from mks_servo_can import const
# Imports the 'const' module for library-defined constants (e.g., default encoder pulses).

from mks_servo_can import exceptions
# Imports the 'exceptions' module for handling library-specific errors.

from mks_servo_can import MultiAxisController
# Imports the 'MultiAxisController' class, which is key for managing and
# sending commands to multiple axes in a coordinated manner.

from mks_servo_can import RotaryKinematics
# Imports 'RotaryKinematics' as an example kinematics model for the motors.
# This defines the relationship between motor steps and user units (e.g., degrees).

# Configure logging
# Sets up basic logging for the script.
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
# Configures the root logger to show INFO level messages and above,
# with a specific format including timestamp, level, and message.

logger = logging.getLogger(__name__)
# Creates a logger instance for this specific module.

SIMULATOR_HOST = "localhost"
# Defines the hostname where the simulator is expected to be running.
# "localhost" indicates the same machine.

SIMULATOR_PORT = 6789  # Default simulator port
# Defines the TCP port number the simulator listens on.
# This must match the port the simulator was started with.

# Define CAN IDs for simulated motors (must match simulator setup)
# These constants define the CAN IDs that will be used for the simulated motors.
# The simulator must be configured to simulate motors with these IDs.
MOTOR_CAN_ID_1 = 1
MOTOR_CAN_ID_2 = 2

async def setup_and_connect_can_interface() -> CANInterface:
    # Defines an asynchronous function to set up and connect the CANInterface to the simulator.
    # It returns the connected CANInterface instance.
    """Sets up and connects the CAN interface to the simulator."""
    # Docstring for the function.
    logger.info("Setting up CAN interface for simulator...")
    # Logs the setup attempt.
    can_if_sim = CANInterface(
        use_simulator=True, # Explicitly tells CANInterface to use the simulator.
        simulator_host=SIMULATOR_HOST, # Specifies the simulator's hostname.
        simulator_port=SIMULATOR_PORT, # Specifies the simulator's port.
    )
    # Creates a CANInterface instance configured for the simulator.
    try:
        # This 'try' block attempts the connection.
        await can_if_sim.connect()
        # Asynchronously connects to the simulator.
        logger.info("Connected to Simulator successfully.")
        # Logs successful connection.
        return can_if_sim
        # Returns the connected interface.
    except exceptions.SimulatorError as e:
        # Catches 'SimulatorError' if the connection to the simulator fails.
        logger.error("Failed to connect to simulator: %s", e)
        # Logs the error.
        logger.error(
            "Please ensure the simulator is running at %s:%s "
            "with at least %d motors configured (e.g., --num-motors 2).",
            SIMULATOR_HOST,
            SIMULATOR_PORT,
            MOTOR_CAN_ID_2, # Ensures the message reminds about simulating enough motors.
        )
        # Provides troubleshooting advice, emphasizing simulator configuration.
        raise # Re-raises the caught exception to halt execution if connection fails.

def create_axes_and_controller(
    can_if: CANInterface,
) -> MultiAxisController:
    # Defines a synchronous function to create Axis instances and a MultiAxisController.
    # It takes a connected CANInterface as input.
    # It returns the configured MultiAxisController.
    """Creates Axis instances and the MultiAxisController."""
    # Docstring for the function.
    logger.info("Creating axes and multi-axis controller...")
    # Logs the creation process.
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        # Default encoder resolution (e.g., 16384 steps/rev).
    )
    # Creates a common RotaryKinematics object to be used by both axes in this example.
    # This means both axes will be controlled in terms of degrees by default.

    axis1 = Axis(
        can_if, # The shared, connected CANInterface.
        MOTOR_CAN_ID_1, # CAN ID for the first motor.
        "SimMotor1", # Name for the first axis.
        kinematics=kin, # Assigns the rotary kinematics.
        default_speed_param=1000, # Sets a default MKS speed parameter for this axis.
        default_accel_param=150, # Sets a default MKS acceleration parameter.
    )
    # Creates the first Axis instance.
    axis2 = Axis(
        can_if, # Shared CANInterface.
        MOTOR_CAN_ID_2, # CAN ID for the second motor.
        "SimMotor2", # Name for the second axis.
        kinematics=kin, # Also uses rotary kinematics.
        default_speed_param=800, # Different default speed for variety.
        default_accel_param=100, # Different default acceleration.
    )
    # Creates the second Axis instance.

    multi_controller = MultiAxisController(can_if)
    # Creates a MultiAxisController, passing it the CANInterface.
    multi_controller.add_axis(axis1)
    # Adds the first axis to the controller.
    multi_controller.add_axis(axis2)
    # Adds the second axis to the controller.
    logger.info("Axes and controller created.")
    # Logs successful creation.
    return multi_controller
    # Returns the configured controller.

async def perform_motor_operations(multi_controller: MultiAxisController):
    # Defines an asynchronous function to perform a sequence of operations on the motors
    # managed by the MultiAxisController.
    """Performs a sequence of operations on the motors."""
    # Docstring for the function.
    logger.info("Initializing all simulated axes...")
    # Logs the start of axis initialization.
    await multi_controller.initialize_all_axes(
        calibrate=False, home=False, concurrent=True
    )
    # Initializes all axes managed by the controller.
    # 'calibrate=False' and 'home=False' skip these steps for this example.
    # 'concurrent=True' means initialization commands are sent to all axes concurrently.

    logger.info("Enabling all axes...")
    # Logs the enabling process.
    await multi_controller.enable_all_axes()
    # Enables all axes concurrently.

    initial_positions = await multi_controller.get_all_positions_user()
    # Gets the current positions of all axes in their user units (degrees in this case).
    logger.info("Initial positions: %s", initial_positions)
    # Logs the initial positions. This will be a dictionary like {'SimMotor1': 0.0, 'SimMotor2': 0.0}.

    target_positions = {
        "SimMotor1": 90.0,  # Target 90 degrees for SimMotor1.
        "SimMotor2": -45.0, # Target -45 degrees for SimMotor2.
    }
    # Defines a dictionary of target absolute positions for each axis by name.
    target_speeds = {
        "SimMotor1": 180.0, # Speed for SimMotor1: 180 degrees/second.
        "SimMotor2": 90.0,  # Speed for SimMotor2: 90 degrees/second.
    }
    # Defines a dictionary of target speeds for each axis.
    logger.info(
        "Moving all axes to: %s with speeds %s",
        target_positions,
        target_speeds,
    )
    # Logs the move command details.
    await multi_controller.move_all_to_positions_abs_user(
        target_positions, speeds_user=target_speeds, wait_for_all=True
    )
    # Commands all axes to move to their respective absolute positions.
    # 'speeds_user' provides individual speeds for each axis.
    # 'wait_for_all=True' means this function call will block until all motors report completion.
    logger.info("Multi-axis move complete.")
    # Logs that the multi-axis move has finished.

    final_positions = await multi_controller.get_all_positions_user()
    # Gets the positions again after the move.
    logger.info("Final positions: %s", final_positions)
    # Logs the final positions.

    # Verifies that each motor reached its target position within a small tolerance.
    for name, target in target_positions.items():
        # Iterates through the target positions.
        if name in final_positions:
            # Checks if the axis name exists in the final positions result.
            logger.info(
                "Axis '%s': Target=%.1f, Actual=%.1f",
                name,
                target,
                final_positions[name],
            )
            # Logs the target vs. actual position for the axis.
            assert (
                abs(final_positions[name] - target) < 1.0
            ), f"Axis {name} did not reach target."
            # Asserts that the actual position is close to the target (tolerance of 1.0 degree).

    await asyncio.sleep(1) # Pauses for 1 second.

    relative_distances = {"SimMotor1": -30.0, "SimMotor2": 15.0}
    # Defines relative distances for another multi-axis move.
    # SimMotor1 will move -30 degrees from its current position.
    # SimMotor2 will move +15 degrees from its current position.
    logger.info("Moving all axes relatively by: %s", relative_distances)
    # Logs the relative move command.
    await multi_controller.move_all_relative_user(
        relative_distances, wait_for_all=True
    )
    # Commands all axes to perform relative moves.
    # 'wait_for_all=True' ensures the call blocks until completion.

    current_positions = await multi_controller.get_all_positions_user()
    # Gets positions after the relative move.
    logger.info("Positions after relative move: %s", current_positions)
    # Logs these new positions.

    logger.info("Disabling all axes...")
    # Logs the disabling process.
    await multi_controller.disable_all_axes()
    # Disables all managed axes concurrently.

    logger.info("Motor operations finished successfully.")
    # Logs the successful completion of all demonstrated operations.

async def main():
    # Defines the main asynchronous function that orchestrates the example.
    """Main execution function for the multi-axis simulator example."""
    # Docstring for the main function.
    logger.info("Starting multi-axis simulator example...")
    # Logs the start of the overall example.
    can_if_sim = None # Initialize to None for the finally block.
    try:
        # This 'try' block encompasses the main setup and operations.
        can_if_sim = await setup_and_connect_can_interface()
        # Sets up and connects the CAN interface.
        multi_controller = create_axes_and_controller(can_if_sim)
        # Creates the axes and the controller.
        await perform_motor_operations(multi_controller)
        # Runs the sequence of motor operations.
        logger.info("Example finished successfully.")
        # Logs overall success.

    except exceptions.SimulatorError:
        # Specifically catches SimulatorError if setup_and_connect_can_interface fails.
        logger.info("Exiting due to simulator connection failure.")
        # Logs the reason for exiting.
    except exceptions.MultiAxisError as e: # Catch MultiAxisError specifically
        # Catches errors specific to MultiAxisController operations (e.g., if one axis fails in a group command).
        logger.error("A Multi-Axis MKS Servo library error occurred: %s", e)
        # The E1101 (no-member) Pylint error on 'e.individual_errors' might occur if Pylint
        # cannot infer the type of 'e' precisely enough within the exception handler.
        # However, 'MultiAxisError' is defined to have 'individual_errors'.
        if e.individual_errors: # pylint: disable=no-member
            # If the error contains details about individual axis failures, log them.
            for axis_name, err in e.individual_errors.items():
                logger.error("  Error for axis '%s': %s", axis_name, err)
    except exceptions.MKSServoError as e:
        # Catches any other general MKS Servo library errors.
        logger.error("An MKS Servo library error occurred: %s", e)
    except Exception as e:  # pylint: disable=broad-except
        # Catches any unexpected non-library errors.
        logger.error("An unexpected error occurred: %s", e, exc_info=True)
        # 'exc_info=True' includes traceback information in the log.
    finally:
        # The 'finally' block ensures cleanup (disconnecting CAN interface) always occurs.
        if can_if_sim: # Checks if the CAN interface was successfully created.
            logger.info("Disconnecting from Simulator...")
            # Logs the disconnection attempt.
            await can_if_sim.disconnect()
            # Asynchronously disconnects from the simulator.
        logger.info("Program terminated.")
        # Logs the end of the program.

if __name__ == "__main__":
    # This standard Python construct ensures that 'asyncio.run(main())' is called
    # only when the script is executed directly (not when imported as a module).
    asyncio.run(main())
    # Runs the main asynchronous function, starting the asyncio event loop.
    