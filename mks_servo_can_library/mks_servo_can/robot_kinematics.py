"""
Robot Kinematics Module for MKS Servo CAN Library.

This module provides classes for defining and controlling multi-axis robotic
systems by building upon the `Axis` and `MultiAxisController` classes.
It allows for the implementation of forward and inverse kinematics for
various robot geometries, enabling control in task space (e.g., Cartesian
coordinates) rather than just joint space.

Current implementations are illustrative and may require more detailed
kinematic models and error handling for production use.
"""

import asyncio
import math
import logging
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Optional, Union, Sequence

from .axis import Axis
from .multi_axis_controller import MultiAxisController
from .exceptions import KinematicsError, ConfigurationError, MKSServoError

logger = logging.getLogger(__name__)

# Define common types for poses and joint states for clarity
JointStates = Union[List[float], Tuple[float, ...], Dict[str, float]]
CartesianPose = Dict[str, float] # e.g., {'x': float, 'y': float, 'z': Optional[float], 'theta': Optional[float]} for 2D/3D


class RobotModelBase(ABC):
    """
    Abstract base class for robot kinematic models.

    This class defines the common interface for all robot models, including
    methods for forward and inverse kinematics. Subclasses must implement these
    methods based on the specific geometry and kinematics of the robot.

    Attributes:
        multi_axis_controller (MultiAxisController): The controller managing the axes
                                                     that form this robot model.
        axis_names_in_order (List[str]): A list of axis names from the
                                         MultiAxisController that correspond to the
                                         robot's joints, in the order expected by
                                         kinematic calculations (e.g., joint 1, joint 2,...).
        dof (int): Degrees of freedom of the robot model.
    """

    def __init__(
        self,
        multi_axis_controller: MultiAxisController,
        axis_names_in_order: List[str],
    ):
        """
        Initializes the RobotModelBase.

        Args:
            multi_axis_controller: The MultiAxisController instance that manages
                                   the physical or simulated motor axes.
            axis_names_in_order: A list of axis names (strings) as defined in the
                                 MultiAxisController. The order of these names must
                                 correspond to the order of joints used in the
                                 kinematic calculations (e.g., base joint first).

        Raises:
            ConfigurationError: If the number of provided axis names does not match
                                the expected degrees of freedom for the robot model,
                                or if any named axis is not found in the controller.
        """
        self.multi_axis_controller: MultiAxisController = multi_axis_controller
        self.axis_names_in_order: List[str] = axis_names_in_order
        self.dof: int = len(axis_names_in_order)

        if not self.axis_names_in_order:
            raise ConfigurationError("axis_names_in_order cannot be empty.")

        # Verify that all named axes exist in the controller
        for name in self.axis_names_in_order:
            if name not in self.multi_axis_controller.axes:
                raise ConfigurationError(
                    f"Axis '{name}' specified in axis_names_in_order "
                    f"not found in the MultiAxisController."
                )
        
        logger.info(
            f"Initialized {self.__class__.__name__} with DOF: {self.dof} "
            f"using axes: {self.axis_names_in_order}"
        )

    @abstractmethod
    async def forward_kinematics(
        self, joint_states: Union[Sequence[float], Dict[str, float]]
    ) -> CartesianPose:
        """
        Calculates the end-effector pose from given joint states.

        Args:
            joint_states: A sequence (list or tuple) of joint values (e.g., angles
                          in degrees/radians, linear positions in mm) in the order
                          defined by `self.axis_names_in_order`, or a dictionary
                          mapping axis names to their joint values.
                          Units should match what individual Axis kinematics expect.

        Returns:
            A dictionary representing the calculated end-effector pose in
            Cartesian coordinates (e.g., {'x': float, 'y': float, 'z': float, ...}).
            The specific keys depend on the robot's dimensionality.

        Raises:
            KinematicsError: If forward kinematics calculation fails or inputs are invalid.
        """
        pass

    @abstractmethod
    async def inverse_kinematics(
        self, target_pose: CartesianPose
    ) -> Dict[str, float]:
        """
        Calculates the required joint states to achieve a target end-effector pose.

        Args:
            target_pose: A dictionary representing the desired end-effector pose
                         in Cartesian coordinates.

        Returns:
            A dictionary mapping axis names (from `self.axis_names_in_order`)
            to the calculated joint values required to reach the target pose.

        Raises:
            KinematicsError: If the target pose is unreachable, if multiple solutions
                             exist and a selection strategy is not implemented, or if
                             inverse kinematics calculation fails.
        """
        pass

    def _resolve_joint_states_to_list(
        self, joint_states: Union[Sequence[float], Dict[str, float]]
    ) -> List[float]:
        """
        Helper to convert various input formats for joint states into an ordered list.
        """
        if isinstance(joint_states, dict):
            try:
                return [joint_states[name] for name in self.axis_names_in_order]
            except KeyError as e:
                raise KinematicsError(
                    f"Missing joint state for axis '{e.args[0]}' in provided dictionary."
                ) from e
        elif isinstance(joint_states, (list, tuple)):
            if len(joint_states) != self.dof:
                raise KinematicsError(
                    f"Provided joint_states sequence has {len(joint_states)} values, "
                    f"but robot DOF is {self.dof}."
                )
            return list(joint_states)
        else:
            raise TypeError(
                f"Unsupported type for joint_states: {type(joint_states)}. "
                "Expected List, Tuple, or Dict."
            )

    async def get_current_joint_states(self) -> Dict[str, float]:
        """
        Retrieves the current joint states (positions/angles) of all axes
        forming the robot, in their user-defined units.

        Returns:
            A dictionary mapping axis names to their current joint values.

        Raises:
            MultiAxisError: If fetching positions from one or more axes fails.
        """
        return await self.multi_axis_controller.get_all_positions_user()

    async def get_current_pose(self) -> CartesianPose:
        """
        Calculates the current end-effector pose based on current joint states.

        This method first reads the current joint positions from the motors
        and then applies forward kinematics.

        Returns:
            A dictionary representing the current end-effector pose.

        Raises:
            MKSServoError: If reading joint states or calculating FK fails.
        """
        try:
            current_joint_values_dict = await self.get_current_joint_states()
            # Ensure the joint values are in the correct order for forward_kinematics
            ordered_joint_values = [
                current_joint_values_dict[name] for name in self.axis_names_in_order
            ]
            return await self.forward_kinematics(ordered_joint_values)
        except Exception as e:
            logger.error(f"Failed to get current pose: {e}")
            raise KinematicsError(f"Failed to get current pose: {e}") from e

    async def move_to_cartesian_pose(
        self,
        target_pose: CartesianPose,
        speeds_user: Optional[Dict[str, float]] = None,
        wait_for_all: bool = True,
    ) -> None:
        """
        Calculates joint states for a target Cartesian pose and commands the axes.

        This method performs inverse kinematics to find the required joint angles/positions,
        then uses the `MultiAxisController` to move all axes to these target joint states.

        Args:
            target_pose: The desired end-effector pose as a CartesianPose dictionary.
            speeds_user: Optional dictionary mapping axis names to their desired speeds
                         in user units per second (as defined by individual Axis kinematics).
                         If None, default speeds for each axis will be used. For true
                         coordinated Cartesian speed, these joint speeds would need to be
                         profiled based on the path and Jacobian.
            wait_for_all: If True (default), waits for all axes to complete their moves.

        Raises:
            KinematicsError: If inverse kinematics fails (e.g., pose unreachable).
            MultiAxisError: If commanding one or more axes via MultiAxisController fails.
        """
        logger.info(f"Robot '{self.__class__.__name__}': Moving to Cartesian pose: {target_pose}")
        try:
            joint_targets_dict = await self.inverse_kinematics(target_pose)
            
            # Validate that IK returned targets for all necessary axes
            for axis_name in self.axis_names_in_order:
                if axis_name not in joint_targets_dict:
                    raise KinematicsError(f"Inverse kinematics did not return a target for axis '{axis_name}'.")

            logger.info(f"Calculated joint targets: {joint_targets_dict}")
            await self.multi_axis_controller.move_all_to_positions_abs_user(
                positions_user=joint_targets_dict,
                speeds_user=speeds_user,
                wait_for_all=wait_for_all,
            )
            logger.info(f"Robot '{self.__class__.__name__}': Move to Cartesian pose command sequence finished.")
        except KinematicsError as ke:
            logger.error(f"Kinematic error during move_to_cartesian_pose: {ke}")
            raise
        except MultiAxisError as mae:
            logger.error(f"MultiAxisError during move_to_cartesian_pose: {mae}")
            raise
        except Exception as e: # pylint: disable=broad-except
            logger.error(f"Unexpected error in move_to_cartesian_pose: {e}", exc_info=True)
            raise MKSServoError(f"Unexpected error in move_to_cartesian_pose: {e}")


class TwoLinkArmPlanar(RobotModelBase):
    """
    A simple 2-DOF planar articulated robot arm (RR configuration).
    The end-effector position is described by (x, y) coordinates.
    Joint 1 (axis_names_in_order[0]) is the base joint angle (theta1).
    Joint 2 (axis_names_in_order[1]) is the elbow joint angle (theta2).
    Angles are typically in degrees, as handled by individual Axis kinematics.
    """

    def __init__(
        self,
        multi_axis_controller: MultiAxisController,
        axis_names: List[str],  # Should contain two names, e.g., ["base_joint", "elbow_joint"]
        link1_length: float,
        link2_length: float,
        origin_offset: Tuple[float, float] = (0.0, 0.0)
    ):
        """
        Initializes a TwoLinkArmPlanar robot model.

        Args:
            multi_axis_controller: The controller for the arm's motors.
            axis_names: A list of two axis names (str) corresponding to the base
                        joint (joint 1) and the elbow joint (joint 2), in that order.
            link1_length: Length of the first link (base to elbow). Units should
                          be consistent (e.g., mm).
            link2_length: Length of the second link (elbow to end-effector).
            origin_offset: (x, y) offset of the robot's base from the world origin.
        
        Raises:
            ConfigurationError: If `axis_names` does not contain exactly two names.
        """
        if len(axis_names) != 2:
            raise ConfigurationError(
                "TwoLinkArmPlanar requires exactly two axis names (for base and elbow joints)."
            )
        super().__init__(multi_axis_controller, axis_names) # DOF will be 2
        
        if link1_length <= 0 or link2_length <= 0:
            raise ConfigurationError("Link lengths must be positive.")
            
        self.l1 = link1_length
        self.l2 = link2_length
        self.origin_x = origin_offset[0]
        self.origin_y = origin_offset[1]

        logger.info(
            f"Initialized TwoLinkArmPlanar: Link1={self.l1}, Link2={self.l2}, "
            f"Axes: Base='{self.axis_names_in_order[0]}', Elbow='{self.axis_names_in_order[1]}'"
        )

    async def forward_kinematics(
        self, joint_states: Union[Sequence[float], Dict[str, float]]
    ) -> CartesianPose:
        """
        Calculates the (x, y) position of the end-effector.

        Args:
            joint_states: A sequence [theta1_deg, theta2_deg] or a dictionary
                          mapping axis names to joint angles in degrees.
                          theta1 is the angle of the first link from the x-axis.
                          theta2 is the angle of the second link relative to the first link.

        Returns:
            A dictionary {'x': float, 'y': float} representing the end-effector position.
        """
        ordered_states = self._resolve_joint_states_to_list(joint_states)
        theta1_deg, theta2_deg = ordered_states[0], ordered_states[1]

        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg) # Angle of link 2 relative to link 1

        # x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
        # y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
        x_val = self.l1 * math.cos(theta1_rad) + self.l2 * math.cos(theta1_rad + theta2_rad)
        y_val = self.l1 * math.sin(theta1_rad) + self.l2 * math.sin(theta1_rad + theta2_rad)
        
        return {'x': x_val + self.origin_x, 'y': y_val + self.origin_y}

    async def inverse_kinematics(
        self, target_pose: CartesianPose
    ) -> Dict[str, float]:
        """
        Calculates joint angles (theta1, theta2) for a target (x, y) position.
        This implementation typically chooses one of two possible elbow configurations
        (e.g., elbow up or elbow down). Here, we'll aim for an "elbow up" like solution
        by convention (positive angle for arccos).

        Args:
            target_pose: A dictionary {'x': float, 'y': float} for the target position.

        Returns:
            A dictionary mapping axis names to joint angles [theta1_deg, theta2_deg] in degrees.
            Example: {'base_joint': theta1_deg, 'elbow_joint': theta2_deg}

        Raises:
            KinematicsError: If the target position is unreachable.
        """
        if 'x' not in target_pose or 'y' not in target_pose:
            raise KinematicsError("Target pose must include 'x' and 'y' coordinates.")

        x = target_pose['x'] - self.origin_x
        y = target_pose['y'] - self.origin_y

        # Calculate theta2
        # D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
        d_numerator = x**2 + y**2 - self.l1**2 - self.l2**2
        d_denominator = 2 * self.l1 * self.l2

        if d_denominator == 0: # Should not happen if l1, l2 > 0
            raise KinematicsError("Link lengths l1 or l2 are zero, cannot compute IK.")
        
        cos_theta2 = d_numerator / d_denominator

        if not (-1.0 <= cos_theta2 <= 1.0):
            dist_from_origin = math.sqrt(x**2 + y**2)
            max_reach = self.l1 + self.l2
            min_reach = abs(self.l1 - self.l2)
            msg = (
                f"Target position ({x}, {y}) is unreachable. "
                f"Distance from origin: {dist_from_origin:.2f}. "
                f"Arm reach: {min_reach:.2f} to {max_reach:.2f}."
            )
            logger.error(msg)
            raise KinematicsError(msg)

        # For elbow-up configuration, theta2_rad is typically chosen as positive acos
        theta2_rad = math.acos(cos_theta2) # This gives a solution in [0, pi] for theta2

        # Calculate theta1
        # k1 = l1 + l2 * cos(theta2)
        # k2 = l2 * sin(theta2)
        # theta1 = atan2(y, x) - atan2(k2, k1)
        k1 = self.l1 + self.l2 * math.cos(theta2_rad)
        k2 = self.l2 * math.sin(theta2_rad)
        
        # atan2(y, x) gives the angle of the target point from origin
        # atan2(k2, k1) gives the angle correction due to the elbow
        theta1_rad = math.atan2(y, x) - math.atan2(k2, k1)

        theta1_deg = math.degrees(theta1_rad)
        theta2_deg = math.degrees(theta2_rad) # This is relative angle of link2 to link1

        return {
            self.axis_names_in_order[0]: theta1_deg,
            self.axis_names_in_order[1]: theta2_deg,
        }


class CartesianRobot(RobotModelBase):
    """
    A simple 3-DOF Cartesian robot (XYZ).
    Assumes orthogonal axes where each motor directly controls one Cartesian dimension.
    The individual Axis kinematics should be LinearKinematics for each.
    """

    def __init__(
        self,
        multi_axis_controller: MultiAxisController,
        x_axis_name: str,
        y_axis_name: str,
        z_axis_name: str,
        origin_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ):
        """
        Initializes a 3-DOF Cartesian robot model.

        Args:
            multi_axis_controller: The controller for the robot's motors.
            x_axis_name: Name of the axis in MultiAxisController for X dimension.
            y_axis_name: Name of the axis in MultiAxisController for Y dimension.
            z_axis_name: Name of the axis in MultiAxisController for Z dimension.
            origin_offset: (x, y, z) offset of the robot's effective origin
                           from the world or machine origin.
        """
        axis_names = [x_axis_name, y_axis_name, z_axis_name]
        super().__init__(multi_axis_controller, axis_names)
        if self.dof != 3: # Should be caught by super if axis_names length is not 3.
             raise ConfigurationError("CartesianRobot requires exactly three axis names (X, Y, Z).")
        
        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name
        self.z_axis_name = z_axis_name
        self.origin_x, self.origin_y, self.origin_z = origin_offset
        logger.info(
            f"Initialized CartesianRobot: X='{x_axis_name}', Y='{y_axis_name}', Z='{z_axis_name}'"
        )

    async def forward_kinematics(
        self, joint_states: Union[Sequence[float], Dict[str, float]]
    ) -> CartesianPose:
        """
        For a Cartesian robot, FK is trivial: joint states are Cartesian positions
        relative to the robot's origin.

        Args:
            joint_states: A sequence [x_pos, y_pos, z_pos] or a dictionary mapping
                          axis names (x_axis_name, etc.) to their linear positions.
                          Units match individual Axis kinematics (e.g., mm).

        Returns:
            A dictionary {'x': float, 'y': float, 'z': float} for the end-effector position.
        """
        ordered_states = self._resolve_joint_states_to_list(joint_states)
        x_robot, y_robot, z_robot = ordered_states[0], ordered_states[1], ordered_states[2]
        
        return {
            'x': x_robot + self.origin_x,
            'y': y_robot + self.origin_y,
            'z': z_robot + self.origin_z
        }

    async def inverse_kinematics(
        self, target_pose: CartesianPose
    ) -> Dict[str, float]:
        """
        For a Cartesian robot, IK is trivial: Cartesian targets are joint targets
        (adjusted for origin offset).

        Args:
            target_pose: A dictionary {'x': float, 'y': float, 'z': float} for the target position.

        Returns:
            A dictionary mapping axis names (x_axis_name, etc.) to target linear positions.
        
        Raises:
            KinematicsError: If target_pose is missing required keys.
        """
        if 'x' not in target_pose or 'y' not in target_pose or 'z' not in target_pose:
            raise KinematicsError("Target pose for CartesianRobot must include 'x', 'y', and 'z'.")

        target_x_joint = target_pose['x'] - self.origin_x
        target_y_joint = target_pose['y'] - self.origin_y
        target_z_joint = target_pose['z'] - self.origin_z

        return {
            self.x_axis_name: target_x_joint,
            self.y_axis_name: target_y_joint,
            self.z_axis_name: target_z_joint,
        }

# Future robot models (e.g., SCARA, Delta, 6-DOF Articulated) would be added here.