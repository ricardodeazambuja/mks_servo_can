"""
Example script demonstrating control of a 3-DOF RRR (Revolute-Revolute-Revolute) arm.

This script showcases how to use the `RRRArm` class from the `robot_kinematics`
module, along with `MultiAxisController` and `Axis` objects, to command
the arm to a specific Cartesian pose.
"""
# In examples/rrr_arm_example.py

import asyncio
import logging

# Make sure to import your new RRRArm class
from mks_servo_can import (
    CANInterface, Axis, MultiAxisController, RotaryKinematics, const, exceptions, RRRArm
)
from mks_servo_can.robot_kinematics import CartesianPose # For type hinting

# --- Configuration ---
LOG_LEVEL = logging.INFO
USE_SIMULATOR = True # Set to False for real hardware and configure below

# Simulator Settings (if USE_SIMULATOR is True)
SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789 # Ensure simulator is running, e.g.: mks-servo-simulator --num-motors 3 --start-can-id 1

# Real Hardware Settings (Modify if USE_SIMULATOR is False)
CAN_INTERFACE_TYPE = "canable"
CAN_CHANNEL = "/dev/ttyACM0" # Adjust for your OS and adapter
CAN_BITRATE = 500000

# Robot Configuration
# These names must match the order in RRRArm's axis_names parameter
BASE_JOINT_AXIS_NAME = "J1_Base"
SHOULDER_JOINT_AXIS_NAME = "J2_Shoulder"
ELBOW_JOINT_AXIS_NAME = "J3_Elbow"

# CAN IDs for the motors
BASE_JOINT_CAN_ID = 1
SHOULDER_JOINT_CAN_ID = 2
ELBOW_JOINT_CAN_ID = 3

# RRR Arm Link Lengths - **UPDATE THESE TO MATCH YOUR ARM AND RRRArm CLASS PARAMETERS**
LINK_1_SHOULDER_ELBOW = 100.0  # Length from shoulder joint to elbow joint
LINK_2_ELBOW_EE = 80.0      # Length from elbow joint to end-effector

# Optional offset of the robot's base from the world origin
ORIGIN_OFFSET_XYZ = (0.0, 0.0, 0.0)

# --- Logging Setup ---
logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("RRRArmExample")

async def main():
    """Main execution function for the RRR arm example."""
    can_if = None
    multi_controller = None
    logger.info("Starting RRR Arm example...")

    try:
        # 1. Setup CANInterface
        if USE_SIMULATOR:
            logger.info(f"Using simulator at {SIMULATOR_HOST}:{SIMULATOR_PORT}")
            can_if = CANInterface(use_simulator=True, simulator_host=SIMULATOR_HOST, simulator_port=SIMULATOR_PORT)
        else:
            logger.info(f"Using real hardware: {CAN_INTERFACE_TYPE} on {CAN_CHANNEL}")
            can_if = CANInterface(interface_type=CAN_INTERFACE_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE, use_simulator=False)
        
        await can_if.connect()
        logger.info("CAN Interface connected.")

        # 2. Setup MultiAxisController
        multi_controller = MultiAxisController(can_interface_manager=can_if)
        logger.info("MultiAxisController initialized.")

        # 3. Add Axis objects for each RRR joint
        # All joints in an RRR arm are rotary
        rotary_kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
        # Add gear_ratio if your motors have gearboxes, e.g., RotaryKinematics(..., gear_ratio=5.0)

        axis_j1_base = Axis(can_if, BASE_JOINT_CAN_ID, BASE_JOINT_AXIS_NAME, kinematics=rotary_kin)
        axis_j2_shoulder = Axis(can_if, SHOULDER_JOINT_CAN_ID, SHOULDER_JOINT_AXIS_NAME, kinematics=rotary_kin)
        axis_j3_elbow = Axis(can_if, ELBOW_JOINT_CAN_ID, ELBOW_JOINT_AXIS_NAME, kinematics=rotary_kin)

        multi_controller.add_axis(axis_j1_base)
        multi_controller.add_axis(axis_j2_shoulder)
        multi_controller.add_axis(axis_j3_elbow)
        logger.info(f"Axes {BASE_JOINT_AXIS_NAME}, {SHOULDER_JOINT_AXIS_NAME}, {ELBOW_JOINT_AXIS_NAME} added.")

        # 4. Initialize and Enable All Axes
        await multi_controller.initialize_all_axes(calibrate=False, home=False) # Adjust as needed
        await multi_controller.enable_all_axes()
        logger.info("All axes initialized and enabled.")

        # 5. Define and Initialize the RRRArm Robot Model
        # The order of names in `axis_names` list here is critical and must match
        # how your RRRArm's kinematics methods expect them (e.g., theta1, theta2, theta3).
        rrr_arm = RRRArm(
            multi_axis_controller=multi_controller,
            axis_names=[BASE_JOINT_AXIS_NAME, SHOULDER_JOINT_AXIS_NAME, ELBOW_JOINT_AXIS_NAME],
            link1_length=LINK_1_SHOULDER_ELBOW,
            link2_length=LINK_2_ELBOW_EE,
            origin_offset=ORIGIN_OFFSET_XYZ
        )
        logger.info("RRRArm model initialized.")

        # 6. Perform Operations
        initial_joints = await rrr_arm.get_current_joint_states()
        logger.info(f"Initial Joint States (degrees): {initial_joints}")
        initial_pose = await rrr_arm.get_current_pose()
        logger.info(f"Initial Cartesian Pose (mm): {initial_pose}")

        # Define a target Cartesian pose (ensure it's reachable for your arm dimensions!)
        # Example: A point that should be reachable
        target_pose: CartesianPose = {'x': LINK_1_SHOULDER_ELBOW / 2, 'y': LINK_2_ELBOW_EE / 2, 'z': 50.0}
        logger.info(f"Target Cartesian Pose: {target_pose}")
        
        # Define speeds for each joint (in degrees/second as per RotaryKinematics)
        joint_speeds = {
            BASE_JOINT_AXIS_NAME: 20.0,      # deg/s
            SHOULDER_JOINT_AXIS_NAME: 15.0,  # deg/s
            ELBOW_JOINT_AXIS_NAME: 10.0      # deg/s
        }

        try:
            await rrr_arm.move_to_cartesian_pose(
                target_pose=target_pose,
                speeds_user=joint_speeds,
                wait_for_all=True
            )
            logger.info("Move to target Cartesian pose command finished.")
            
            final_joints = await rrr_arm.get_current_joint_states()
            logger.info(f"Final Joint States (degrees): {final_joints}")
            final_pose = await rrr_arm.get_current_pose()
            logger.info(f"Final Cartesian Pose (mm): {final_pose}")

        except exceptions.KinematicsError as e:
            logger.error(f"Kinematics Error during RRR arm move: {e}")
        except exceptions.MultiAxisError as e:
            logger.error(f"MultiAxis Error during RRR arm move: {e}")
            if e.individual_errors:
                for axis_name, err_detail in e.individual_errors.items():
                    logger.error(f"  Error for axis '{axis_name}': {err_detail}")

    except exceptions.MKSServoError as e:
        logger.error(f"A mks-servo-can library error occurred: {e}")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        if multi_controller and multi_controller.axes: # Check if initialized
            logger.info("Disabling all axes...")
            try:
                await multi_controller.disable_all_axes()
            except exceptions.MKSServoError as e_dis: # Catch potential error during disable
                logger.error(f"Error disabling axes: {e_dis}")
        
        if can_if and can_if.is_connected: # Check if initialized and connected
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("RRR Arm example finished.")

if __name__ == "__main__":
    asyncio.run(main())
