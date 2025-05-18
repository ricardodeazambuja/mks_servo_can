"""
Example: Controlling a 3-DOF Cartesian Robot (XYZ).

This script demonstrates how to use the `CartesianRobot` class from the
`robot_kinematics` module. It shows:
1. Setting up the CAN interface (simulator by default).
2. Creating Axis objects for X, Y, and Z axes, each with LinearKinematics.
3. Initializing a MultiAxisController.
4. Defining a CartesianRobot model.
5. Commanding the robot to move to a specific XYZ Cartesian pose.
6. Reading the current XYZ Cartesian pose.

To run with the simulator:
Ensure the MKS Servo Simulator is running with at least 3 motors.
Example: `mks-servo-simulator --num-motors 3 --start-can-id 1`
"""
import asyncio
import logging
import math

from mks_servo_can import (
    CANInterface,
    Axis,
    MultiAxisController,
    LinearKinematics, # Cartesian axes typically use linear kinematics
    const,
    exceptions
)
from mks_servo_can.robot_kinematics import CartesianRobot, CartesianPose

# --- Configuration ---
LOG_LEVEL = logging.INFO

USE_SIMULATOR = True
SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789

CAN_INTERFACE_TYPE = "canable"
CAN_CHANNEL = "/dev/ttyACM0"
CAN_BITRATE = 500000

# Robot Configuration
X_AXIS_NAME = "AxisX"
Y_AXIS_NAME = "AxisY"
Z_AXIS_NAME = "AxisZ"

X_AXIS_CAN_ID = 1
Y_AXIS_CAN_ID = 2
Z_AXIS_CAN_ID = 3

# Kinematics for each axis (e.g., mm per motor revolution)
# Ensure steps_per_revolution matches your motor's encoder resolution.
X_PITCH_MM_PER_REV = 10.0
Y_PITCH_MM_PER_REV = 10.0
Z_PITCH_MM_PER_REV = 5.0  # Example: Z-axis might have a different pitch

# Offset of the robot's coordinate system origin from the machine/world origin
ROBOT_ORIGIN_OFFSET_XYZ = (0.0, 0.0, 0.0) 

# --- Logging Setup ---
logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("CartesianRobotExample")

async def main():
    """
    Main asynchronous function to demonstrate CartesianRobot control.
    """
    can_if = None
    multi_controller = None

    logger.info("Starting Cartesian Robot (XYZ) example...")

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

        # --- 3. Add Axis objects for X, Y, Z ---
        kin_x = LinearKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
            pitch=X_PITCH_MM_PER_REV, units="mm"
        )
        axis_x = Axis(can_if, X_AXIS_CAN_ID, X_AXIS_NAME, kinematics=kin_x)
        multi_controller.add_axis(axis_x)

        kin_y = LinearKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
            pitch=Y_PITCH_MM_PER_REV, units="mm"
        )
        axis_y = Axis(can_if, Y_AXIS_CAN_ID, Y_AXIS_NAME, kinematics=kin_y)
        multi_controller.add_axis(axis_y)

        kin_z = LinearKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
            pitch=Z_PITCH_MM_PER_REV, units="mm"
        )
        axis_z = Axis(can_if, Z_AXIS_CAN_ID, Z_AXIS_NAME, kinematics=kin_z)
        multi_controller.add_axis(axis_z)
        
        logger.info(f"Added axes: X (ID {X_AXIS_CAN_ID}), Y (ID {Y_AXIS_CAN_ID}), Z (ID {Z_AXIS_CAN_ID})")

        # --- 4. Initialize and Enable All Axes ---
        logger.info("Initializing all axes...")
        await multi_controller.initialize_all_axes(calibrate=False, home=False)
        logger.info("Enabling all axes...")
        await multi_controller.enable_all_axes()

        # --- 5. Define and Initialize the CartesianRobot Model ---
        robot = CartesianRobot(
            multi_axis_controller=multi_controller,
            x_axis_name=X_AXIS_NAME,
            y_axis_name=Y_AXIS_NAME,
            z_axis_name=Z_AXIS_NAME,
            origin_offset=ROBOT_ORIGIN_OFFSET_XYZ
        )
        logger.info("CartesianRobot model initialized.")

        # --- 6. Perform Operations ---
        current_pose = await robot.get_current_pose()
        logger.info(f"Initial Cartesian Pose: X={current_pose.get('x',0):.2f}, Y={current_pose.get('y',0):.2f}, Z={current_pose.get('z',0):.2f} mm")

        # Define a target Cartesian pose (e.g., in mm)
        target_xyz_pose: CartesianPose = {'x': 50.0, 'y': 30.0, 'z': 10.0}
        logger.info(f"Target Cartesian Pose: X={target_xyz_pose['x']:.2f}, Y={target_xyz_pose['y']:.2f}, Z={target_xyz_pose['z']:.2f} mm")

        # Speeds for each axis in user units per second (mm/s)
        axis_speeds = {
            X_AXIS_NAME: 20.0,  # mm/s
            Y_AXIS_NAME: 15.0,  # mm/s
            Z_AXIS_NAME: 10.0   # mm/s
        }
        
        logger.info(f"Moving robot to target Cartesian pose with speeds: {axis_speeds}...")
        try:
            await robot.move_to_cartesian_pose(
                target_pose=target_xyz_pose,
                speeds_user=axis_speeds,
                wait_for_all=True
            )
            logger.info("Robot move to Cartesian pose initiated and completed.")

            final_pose = await robot.get_current_pose()
            logger.info(f"Final Cartesian Pose: X={final_pose.get('x',0):.2f}, Y={final_pose.get('y',0):.2f}, Z={final_pose.get('z',0):.2f} mm")

            # Check if close to target
            if (math.isclose(final_pose.get('x',0), target_xyz_pose['x'], abs_tol=0.5) and
                math.isclose(final_pose.get('y',0), target_xyz_pose['y'], abs_tol=0.5) and
                math.isclose(final_pose.get('z',0), target_xyz_pose['z'], abs_tol=0.5)):
                logger.info("Robot successfully reached target Cartesian pose (within tolerance).")
            else:
                logger.warning("Robot may not have precisely reached the target Cartesian pose.")
        
        except exceptions.KinematicsError as e:
            logger.error(f"Kinematics error during move: {e}")
        except exceptions.MultiAxisError as e:
            logger.error(f"Multi-axis error during move: {e}")
            if e.individual_errors:
                for axis_name, err_detail in e.individual_errors.items():
                    logger.error(f"  Error for axis '{axis_name}': {err_detail}")

        # Example: Move back towards origin
        await asyncio.sleep(1)
        target_xyz_pose_2: CartesianPose = {'x': 5.0, 'y': 2.0, 'z': 1.0}
        logger.info(f"Moving to second Cartesian Pose: X={target_xyz_pose_2['x']:.2f}, Y={target_xyz_pose_2['y']:.2f}, Z={target_xyz_pose_2['z']:.2f} mm")
        try:
            await robot.move_to_cartesian_pose(target_xyz_pose_2, speeds_user=axis_speeds, wait_for_all=True)
            final_pose_2 = await robot.get_current_pose()
            logger.info(f"Final Cartesian Pose 2: X={final_pose_2.get('x',0):.2f}, Y={final_pose_2.get('y',0):.2f}, Z={final_pose_2.get('z',0):.2f} mm")
        except Exception as e:
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
        logger.info("Cartesian Robot (XYZ) example finished.")

if __name__ == "__main__":
    # To run this example:
    # 1. Ensure mks-servo-can library is installed/importable.
    # 2. If using simulator (USE_SIMULATOR = True):
    #    Start it: `mks-servo-simulator --num-motors 3 --start-can-id 1`
    #    (Adjust CAN IDs in script and simulator command if needed)
    # 3. If using real hardware: Configure CAN settings and motor IDs.
    asyncio.run(main())
