"""
Example: Controlling a Two-Link Planar Robotic Arm.

This script demonstrates how to use the `TwoLinkArmPlanar` class from the
`robot_kinematics` module to control a 2-DOF planar arm. It shows:
1. Setting up the CAN interface (simulator by default).
2. Creating Axis objects for each joint.
3. Initializing a MultiAxisController.
4. Defining a TwoLinkArmPlanar robot model.
5. Commanding the arm to move to a Cartesian pose.
6. Reading the current Cartesian pose.

To run with the simulator:
Ensure the MKS Servo Simulator is running with at least 2 motors.
Example: `mks-servo-simulator --num-motors 2 --start-can-id 1`
"""
import asyncio
import logging
import math

from mks_servo_can import (
    CANInterface,
    Axis,
    MultiAxisController,
    RotaryKinematics, # Assuming revolute joints
    const,
    exceptions
)
from mks_servo_can.robot_kinematics import TwoLinkArmPlanar, CartesianPose

# --- Configuration ---
# General Settings
LOG_LEVEL = logging.INFO

# Simulator Settings (if USE_SIMULATOR is True)
USE_SIMULATOR = True
SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789

# Real Hardware Settings (if USE_SIMULATOR is False)
# Ensure these match your physical setup if not using the simulator
CAN_INTERFACE_TYPE = "canable"  # e.g., 'socketcan', 'kvaser', 'pcan'
CAN_CHANNEL = "/dev/ttyACM0"    # e.g., 'can0', 'COM3'
CAN_BITRATE = 500000

# Robot Configuration
# Axis names must match what you'll use when adding them to MultiAxisController
BASE_JOINT_AXIS_NAME = "Joint1_Base"
ELBOW_JOINT_AXIS_NAME = "Joint2_Elbow"

BASE_JOINT_CAN_ID = 1
ELBOW_JOINT_CAN_ID = 2

# Robot arm dimensions (e.g., in millimeters)
LINK_1_LENGTH = 100.0  # Length of the first link (base to elbow)
LINK_2_LENGTH = 80.0   # Length of the second link (elbow to end-effector)
ORIGIN_OFFSET_XY = (0.0, 0.0) # (x, y) offset of the robot's base from the world origin

# --- Logging Setup ---
logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("TwoLinkArmExample")

async def main():
    """
    Main asynchronous function to demonstrate TwoLinkArmPlanar control.
    """
    can_if = None
    multi_controller = None

    logger.info("Starting Two-Link Planar Arm example...")

    try:
        # --- 1. Setup CANInterface ---
        if USE_SIMULATOR:
            logger.info(f"Using simulator at {SIMULATOR_HOST}:{SIMULATOR_PORT}")
            can_if = CANInterface(
                use_simulator=True,
                simulator_host=SIMULATOR_HOST,
                simulator_port=SIMULATOR_PORT,
            )
        else:
            logger.info(f"Using real hardware: {CAN_INTERFACE_TYPE} on {CAN_CHANNEL}")
            can_if = CANInterface(
                interface_type=CAN_INTERFACE_TYPE,
                channel=CAN_CHANNEL,
                bitrate=CAN_BITRATE,
                use_simulator=False,
            )
        
        await can_if.connect()
        logger.info("CAN Interface connected.")

        # --- 2. Setup MultiAxisController ---
        multi_controller = MultiAxisController(can_interface_manager=can_if)
        logger.info("MultiAxisController initialized.")

        # --- 3. Add Axis objects for each joint ---
        # Assuming both joints are rotary and their user units are degrees.
        # The kinematics steps_per_revolution should match the motor's encoder resolution.
        joint_kinematics = RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        )

        base_joint_axis = Axis(
            can_interface_manager=can_if,
            motor_can_id=BASE_JOINT_CAN_ID,
            name=BASE_JOINT_AXIS_NAME,
            kinematics=joint_kinematics
        )
        multi_controller.add_axis(base_joint_axis)

        elbow_joint_axis = Axis(
            can_interface_manager=can_if,
            motor_can_id=ELBOW_JOINT_CAN_ID,
            name=ELBOW_JOINT_AXIS_NAME,
            kinematics=joint_kinematics
        )
        multi_controller.add_axis(elbow_joint_axis)
        
        logger.info(f"Added axes: '{BASE_JOINT_AXIS_NAME}' (ID {BASE_JOINT_CAN_ID}), "
                    f"'{ELBOW_JOINT_AXIS_NAME}' (ID {ELBOW_JOINT_CAN_ID})")

        # --- 4. Initialize and Enable All Axes ---
        logger.info("Initializing all axes...")
        await multi_controller.initialize_all_axes(calibrate=False, home=False)
        logger.info("Enabling all axes...")
        await multi_controller.enable_all_axes()

        # --- 5. Define and Initialize the TwoLinkArmPlanar Robot Model ---
        arm = TwoLinkArmPlanar(
            multi_axis_controller=multi_controller,
            axis_names=[BASE_JOINT_AXIS_NAME, ELBOW_JOINT_AXIS_NAME], # Order matters!
            link1_length=LINK_1_LENGTH,
            link2_length=LINK_2_LENGTH,
            origin_offset=ORIGIN_OFFSET_XY
        )
        logger.info(f"TwoLinkArmPlanar model initialized: L1={arm.l1}, L2={arm.l2}")

        # --- 6. Perform Operations ---

        # Get current joint angles (degrees)
        current_joint_angles_dict = await arm.get_current_joint_states()
        logger.info(f"Current joint angles: {current_joint_angles_dict}")

        # Calculate and log current Cartesian pose
        current_pose = await arm.get_current_pose()
        logger.info(f"Initial Cartesian Pose: X={current_pose.get('x', 0.0):.2f}, Y={current_pose.get('y', 0.0):.2f}")

        # Define a target Cartesian pose
        target_cartesian_pose: CartesianPose = {'x': LINK_1_LENGTH * 0.7, 'y': LINK_2_LENGTH * 0.5} # Example target
        logger.info(f"Target Cartesian Pose: X={target_cartesian_pose['x']:.2f}, Y={target_cartesian_pose['y']:.2f}")

        # Calculate joint angles for the target pose (for logging/verification)
        try:
            target_joint_angles = await arm.inverse_kinematics(target_cartesian_pose)
            logger.info(f"Calculated target joint angles for pose: {target_joint_angles}")
        except exceptions.KinematicsError as e:
            logger.error(f"Inverse Kinematics Error for target pose: {e}")
            logger.error("Skipping move due to IK error.")
            return # Exit if IK fails for the first target

        # Command the arm to move to the target Cartesian pose
        # Speeds are in user units per second (degrees/s for these rotary joints)
        # For true coordinated Cartesian speed, joint speeds would need careful profiling.
        # Here, we just set a nominal speed for each joint.
        joint_speeds = {
            BASE_JOINT_AXIS_NAME: 30.0,  # degrees/s
            ELBOW_JOINT_AXIS_NAME: 45.0  # degrees/s
        }
        logger.info(f"Moving arm to target Cartesian pose with joint speeds: {joint_speeds}...")
        
        try:
            await arm.move_to_cartesian_pose(
                target_pose=target_cartesian_pose,
                speeds_user=joint_speeds,
                wait_for_all=True
            )
            logger.info("Arm move to Cartesian pose initiated and completed.")

            # Verify final pose
            final_pose = await arm.get_current_pose()
            logger.info(f"Final Cartesian Pose: X={final_pose.get('x', 0.0):.2f}, Y={final_pose.get('y', 0.0):.2f}")
            final_joint_angles = await arm.get_current_joint_states()
            logger.info(f"Final joint angles: {final_joint_angles}")

            # Check if close to target (allowing for small errors)
            if (math.isclose(final_pose.get('x', 0.0), target_cartesian_pose['x'], abs_tol=1.0) and
                math.isclose(final_pose.get('y', 0.0), target_cartesian_pose['y'], abs_tol=1.0)):
                logger.info("Arm successfully reached target Cartesian pose (within tolerance).")
            else:
                logger.warning("Arm may not have precisely reached the target Cartesian pose.")

        except exceptions.KinematicsError as e:
            logger.error(f"Kinematics error during move: {e}")
        except exceptions.MultiAxisError as e:
            logger.error(f"Multi-axis error during move: {e}")
            if e.individual_errors:
                for axis_name, err_detail in e.individual_errors.items():
                    logger.error(f"  Error for axis '{axis_name}': {err_detail}")
        
        # Example: Move to another pose
        await asyncio.sleep(1) # Pause
        target_cartesian_pose_2: CartesianPose = {'x': LINK_1_LENGTH * 0.5, 'y': -LINK_2_LENGTH * 0.3}
        logger.info(f"Moving to second Cartesian Pose: X={target_cartesian_pose_2['x']:.2f}, Y={target_cartesian_pose_2['y']:.2f}")
        try:
            await arm.move_to_cartesian_pose(target_cartesian_pose_2, speeds_user=joint_speeds, wait_for_all=True)
            final_pose_2 = await arm.get_current_pose()
            logger.info(f"Final Cartesian Pose 2: X={final_pose_2.get('x', 0.0):.2f}, Y={final_pose_2.get('y', 0.0):.2f}")
        except Exception as e: # Catch broad exception for the second move
            logger.error(f"Error during second move: {e}")


    except exceptions.MKSServoError as e:
        logger.error(f"A mks-servo-can library error occurred: {e}", exc_info=False)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        if multi_controller and multi_controller.axes:
            logger.info("Disabling all axes...")
            try:
                await multi_controller.disable_all_axes()
            except exceptions.MKSServoError as e_dis:
                logger.error(f"Error disabling axes: {e_dis}")
        
        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("Two-Link Planar Arm example finished.")

if __name__ == "__main__":
    # To run this example:
    # 1. Make sure the mks-servo-can library is installed/importable.
    # 2. If using the simulator (USE_SIMULATOR = True):
    #    Start it: `mks-servo-simulator --num-motors 2 --start-can-id 1`
    #    (or adjust CAN IDs in this script and simulator command accordingly)
    # 3. If using real hardware (USE_SIMULATOR = False):
    #    Configure CAN_INTERFACE_TYPE, CAN_CHANNEL, CAN_BITRATE,
    #    and motor CAN IDs (BASE_JOINT_CAN_ID, ELBOW_JOINT_CAN_ID) correctly.
    #    Ensure motors are powered and CAN bus is properly wired and terminated.
    asyncio.run(main())
