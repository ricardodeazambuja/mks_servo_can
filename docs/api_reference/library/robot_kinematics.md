# API Reference: `robot_kinematics.py`

The `mks_servo_can.robot_kinematics` module provides classes for defining and controlling multi-axis robotic systems. It builds upon the `Axis` and `MultiAxisController` classes to enable control in task space (e.g., Cartesian coordinates) rather than just joint space.

## Module Overview

This module introduces a base class for robot models and provides implementations for a few common robot types. The primary goal is to abstract the kinematic calculations (both forward and inverse) required to translate between the robot's joint movements and its end-effector's position and orientation in a defined coordinate system.

## Key Classes

### `RobotModelBase`

This is an abstract base class that defines the common interface for all robot kinematic models.

**Attributes:**

* `multi_axis_controller (MultiAxisController)`: The controller managing the axes that form this robot model.
* `axis_names_in_order (List[str])`: A list of axis names from the `MultiAxisController` corresponding to the robot's joints, in the order expected by kinematic calculations (e.g., joint 1, joint 2,...).
* `dof (int)`: Degrees of freedom of the robot model, determined by the length of `axis_names_in_order`.

**Methods:**

* `__init__(self, multi_axis_controller: MultiAxisController, axis_names_in_order: List[str])`
    * Initializes the `RobotModelBase`.
    * **Args:**
        * `multi_axis_controller`: The `MultiAxisController` instance.
        * `axis_names_in_order`: Ordered list of axis names (strings) as defined in the `MultiAxisController`.
    * **Raises:** `ConfigurationError` if `axis_names_in_order` is empty or if any named axis is not found in the controller.

* `async def forward_kinematics(self, joint_states: Union[Sequence[float], Dict[str, float]]) -> CartesianPose`
    * **Abstract Method.** Calculates the end-effector pose from given joint states.
    * **Args:**
        * `joint_states`: A sequence (list/tuple) of joint values or a dictionary mapping axis names to joint values. Units should match individual `Axis` kinematics.
    * **Returns:** A dictionary representing the calculated end-effector pose (e.g., `{'x': float, 'y': float, 'z': float}`).
    * **Raises:** `KinematicsError` if calculation fails or inputs are invalid.

* `async def inverse_kinematics(self, target_pose: CartesianPose) -> Dict[str, float]`
    * **Abstract Method.** Calculates required joint states for a target end-effector pose.
    * **Args:**
        * `target_pose`: A dictionary representing the desired end-effector pose.
    * **Returns:** A dictionary mapping axis names to calculated joint values.
    * **Raises:** `KinematicsError` if the target pose is unreachable or calculation fails.

* `async def get_current_joint_states(self) -> Dict[str, float]`
    * Retrieves current joint states (positions/angles) of all axes in user-defined units.
    * **Returns:** A dictionary mapping axis names to current joint values.
    * **Raises:** `MultiAxisError` if fetching positions fails for one or more axes.

* `async def get_current_pose(self) -> CartesianPose`
    * Calculates the current end-effector pose based on current joint states (reads from motors, then applies FK).
    * **Returns:** A dictionary representing the current end-effector pose.
    * **Raises:** `MKSServoError` if reading joint states or calculating FK fails.

* `async def move_to_cartesian_pose(self, target_pose: CartesianPose, speeds_user: Optional[Dict[str, float]] = None, wait_for_all: bool = True) -> None`
    * Calculates joint states for a target Cartesian pose (using IK) and commands the axes using `MultiAxisController`.
    * **Args:**
        * `target_pose`: Desired end-effector pose.
        * `speeds_user` (optional): Dictionary mapping axis names to speeds in user units/sec.
        * `wait_for_all` (optional, default `True`): If `True`, waits for all axes to complete moves.
    * **Raises:** `KinematicsError` (if IK fails), `MultiAxisError` (if commanding axes fails).

### `TwoLinkArmPlanar(RobotModelBase)`

Implements kinematics for a 2-DOF planar articulated robot arm (RR configuration).

**Additional Attributes:**

* `l1 (float)`: Length of the first link.
* `l2 (float)`: Length of the second link.
* `origin_x (float)`, `origin_y (float)`: XY offset of the robot's base.

**Constructor:**

* `__init__(self, multi_axis_controller: MultiAxisController, axis_names: List[str], link1_length: float, link2_length: float, origin_offset: Tuple[float, float] = (0.0, 0.0))`
    * **Args:**
        * `multi_axis_controller`: The `MultiAxisController`.
        * `axis_names`: List of two axis names (base joint, elbow joint).
        * `link1_length`: Length of the first link.
        * `link2_length`: Length of the second link.
        * `origin_offset` (optional): (x, y) offset of the robot's base.
    * **Raises:** `ConfigurationError` if `axis_names` is not two, or link lengths are not positive.

**Implemented Methods:**

* `async def forward_kinematics(self, joint_states: Union[Sequence[float], Dict[str, float]]) -> CartesianPose`
    * Calculates `{'x': float, 'y': float}` from `[theta1_deg, theta2_deg]`.
* `async def inverse_kinematics(self, target_pose: CartesianPose) -> Dict[str, float]`
    * Calculates `{'base_joint_name': theta1_deg, 'elbow_joint_name': theta2_deg}` from `{'x': float, 'y': float}`.
    * Aims for an "elbow up" solution.
    * **Raises:** `KinematicsError` if target is unreachable.

### `CartesianRobot(RobotModelBase)`

Implements kinematics for a 3-DOF Cartesian robot (XYZ). Assumes orthogonal axes.

**Additional Attributes:**

* `x_axis_name (str)`, `y_axis_name (str)`, `z_axis_name (str)`: Names of the axes controlling each dimension.
* `origin_x (float)`, `origin_y (float)`, `origin_z (float)`: XYZ offset of the robot's origin.

**Constructor:**

* `__init__(self, multi_axis_controller: MultiAxisController, x_axis_name: str, y_axis_name: str, z_axis_name: str, origin_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0))`
    * **Args:**
        * `multi_axis_controller`: The `MultiAxisController`.
        * `x_axis_name`, `y_axis_name`, `z_axis_name`: Names of the X, Y, and Z axes.
        * `origin_offset` (optional): (x, y, z) offset of the robot's origin.

**Implemented Methods:**

* `async def forward_kinematics(self, joint_states: Union[Sequence[float], Dict[str, float]]) -> CartesianPose`
    * Calculates `{'x': world_x, 'y': world_y, 'z': world_z}` from joint positions `[x_robot, y_robot, z_robot]` considering the origin offset.
* `async def inverse_kinematics(self, target_pose: CartesianPose) -> Dict[str, float]`
    * Calculates joint target positions `{'x_axis_name': x_robot, ...}` from world Cartesian pose `{'x': world_x, ...}` by adjusting for the origin offset.
    * **Raises:** `KinematicsError` if `target_pose` is missing x, y, or z keys.

### `RRRArm(RobotModelBase)`

Implements kinematics for a 3-DOF RRR (Revolute-Revolute-Revolute) robot arm. Assumes a common RRR configuration (base rotation around Z, shoulder rotation, elbow rotation).

**Additional Attributes:**

* `l1 (float)`: Length of the link between the shoulder joint (J2) and elbow joint (J3).
* `l2 (float)`: Length of the link between the elbow joint (J3) and the end-effector.
* `origin_x (float)`, `origin_y (float)`, `origin_z (float)`: XYZ offset of the robot's base.

**Constructor:**

* `__init__(self, multi_axis_controller: MultiAxisController, axis_names: List[str], link1_length: float, link2_length: float, origin_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0))`
    * **Args:**
        * `multi_axis_controller`: The `MultiAxisController`.
        * `axis_names`: List of three axis names (base, shoulder, elbow).
        * `link1_length`: Length of the shoulder-to-elbow link.
        * `link2_length`: Length of the elbow-to-end-effector link.
        * `origin_offset` (optional): (x, y, z) offset of the robot's base.
    * **Raises:** `ConfigurationError` if `axis_names` is not three, or link lengths are not positive.

**Implemented Methods:**

* `async def forward_kinematics(self, joint_states: Union[Sequence[float], Dict[str, float]]) -> CartesianPose`
    * Calculates `{'x': float, 'y': float, 'z': float}` from `[theta1_deg, theta2_deg, theta3_deg]`.
* `async def inverse_kinematics(self, target_pose: CartesianPose) -> Dict[str, float]`
    * Calculates joint angles (degrees) for a target (x, y, z) position.
    * **Note**: The IK implementation is specific to a common RRR geometry and may need adjustment for different configurations.
    * **Raises:** `KinematicsError` if target is unreachable or calculation fails (e.g., due to link lengths).

## Type Aliases

* **`JointStates = Union[List[float], Tuple[float, ...], Dict[str, float]]`**
    * Represents the state of the robot's joints, which can be provided as an ordered list/tuple of floats or a dictionary mapping axis names to float values.
* **`CartesianPose = Dict[str, float]`**
    * Represents the end-effector's pose in Cartesian space. Typically, keys include 'x', 'y', and optionally 'z', 'rx', 'ry', 'rz' or 'theta' depending on the robot's dimensionality and capabilities (e.g., `{'x': 100.0, 'y': 50.0, 'z': 20.0}`).

## Usage Example

For detailed usage, refer to the example scripts:
* `examples/two_link_planar_arm.py`
* `examples/cartesian_3dof_robot.py`
* `examples/three_link_arm.py`

And the user guide:
* [Controlling Robot Models](user_guides/library/robot_control.md) (once created)