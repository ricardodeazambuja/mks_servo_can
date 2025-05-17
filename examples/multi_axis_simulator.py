# mks_servo_can/examples/multi_axis_simulator.py
"""
Example: Controlling multiple MKS Servo motors using the Simulator.
Make sure the simulator is running before executing this script.
(e.g., `mks-servo-simulator --num-motors 2 --start-can-id 1`)
"""
import asyncio
import logging

from mks_servo_can_library.mks_servo_can import (
    CANInterface, Axis, MultiAxisController, RotaryKinematics, const, exceptions
)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

SIMULATOR_HOST = 'localhost'
SIMULATOR_PORT = 6789 # Default simulator port

# Define CAN IDs for simulated motors (must match simulator setup)
MOTOR_CAN_ID_1 = 1
MOTOR_CAN_ID_2 = 2

async def main():
    logging.info("Starting multi-axis simulator example...")

    can_if_sim = CANInterface(
        use_simulator=True,
        simulator_host=SIMULATOR_HOST,
        simulator_port=SIMULATOR_PORT
    )

    try:
        await can_if_sim.connect()
        logging.info("Connected to Simulator successfully.")
    except exceptions.SimulatorError as e:
        logging.error(f"Failed to connect to simulator: {e}")
        logging.error(f"Please ensure the simulator is running at {SIMULATOR_HOST}:{SIMULATOR_PORT} "
                      f"with at least {MOTOR_CAN_ID_2} motors configured (e.g., --num-motors 2).")
        return

    # Create Kinematics (shared or individual)
    kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)

    # Create Axis instances
    axis1 = Axis(can_if_sim, MOTOR_CAN_ID_1, "SimMotor1", kinematics=kin, default_speed_param=1000, default_accel_param=150)
    axis2 = Axis(can_if_sim, MOTOR_CAN_ID_2, "SimMotor2", kinematics=kin, default_speed_param=800, default_accel_param=100)

    # Create MultiAxisController
    multi_controller = MultiAxisController(can_if_sim)
    multi_controller.add_axis(axis1)
    multi_controller.add_axis(axis2)

    try:
        logging.info("Initializing all simulated axes...")
        # For simulator, calibration/homing might be no-ops or have simple simulated behavior
        await multi_controller.initialize_all_axes(calibrate=False, home=False, concurrent=True)

        logging.info("Enabling all axes...")
        await multi_controller.enable_all_axes()

        # Get initial positions
        initial_positions = await multi_controller.get_all_positions_user()
        logging.info(f"Initial positions: {initial_positions}")

        # --- Coordinated move (conceptual - they start together) ---
        target_positions = {
            "SimMotor1": 90.0,  # degrees
            "SimMotor2": -45.0 # degrees
        }
        target_speeds = { # Optional
            "SimMotor1": 180.0, # deg/s
            "SimMotor2": 90.0   # deg/s
        }
        logging.info(f"Moving all axes to: {target_positions} with speeds {target_speeds}")
        await multi_controller.move_all_to_positions_abs_user(
            target_positions,
            speeds_user=target_speeds,
            wait_for_all=True
        )
        logging.info("Multi-axis move complete.")

        final_positions = await multi_controller.get_all_positions_user()
        logging.info(f"Final positions: {final_positions}")

        # Verify (approximate due to simulation steps)
        for name, target in target_positions.items():
            if name in final_positions:
                logging.info(f"Axis '{name}': Target={target:.1f}, Actual={final_positions[name]:.1f}")
                assert abs(final_positions[name] - target) < 1.0, f"Axis {name} did not reach target."

        await asyncio.sleep(1)

        # --- Relative move ---
        relative_distances = {
            "SimMotor1": -30.0,
            "SimMotor2": 15.0
        }
        logging.info(f"Moving all axes relatively by: {relative_distances}")
        await multi_controller.move_all_relative_user(relative_distances, wait_for_all=True)
        
        current_positions = await multi_controller.get_all_positions_user()
        logging.info(f"Positions after relative move: {current_positions}")


        logging.info("Disabling all axes...")
        await multi_controller.disable_all_axes()

        logging.info("Example finished successfully.")

    except exceptions.MKSServoError as e:
        logging.error(f"An MKS Servo library error occurred: {e}")
        if isinstance(e, exceptions.MultiAxisError) and e.individual_errors:
            for axis_name, err in e.individual_errors.items():
                logging.error(f"  Error for axis '{axis_name}': {err}")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        logging.info("Disconnecting from Simulator...")
        await can_if_sim.disconnect()
        logging.info("Program terminated.")

if __name__ == "__main__":
    asyncio.run(main())