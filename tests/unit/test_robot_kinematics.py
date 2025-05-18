"""
Unit tests for the robot_kinematics module.

These tests verify the forward and inverse kinematic calculations for
implemented robot models like TwoLinkArmPlanar and CartesianRobot.
It also includes basic integration-style tests for methods like
move_to_cartesian_pose, using mocks for the underlying MultiAxisController
and Axis objects to isolate the robot model logic.
"""
import asyncio
import math
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import Dict

from mks_servo_can import CANInterface

from mks_servo_can import (
    Axis, # For type hinting and mock spec
    MultiAxisController, # For type hinting and mock spec
    RotaryKinematics,
    LinearKinematics,
    const,
    exceptions
)
from mks_servo_can.robot_kinematics import (
    RobotModelBase,
    TwoLinkArmPlanar,
    CartesianRobot,
    CartesianPose,
    KinematicsError,
    ConfigurationError
)

# --- Fixtures ---

@pytest.fixture
def mock_can_interface():
    """Mocks the CANInterface."""
    mock = MagicMock(spec=CANInterface) # Use MagicMock for potentially sync/async attributes if needed by Axis
    mock.is_connected = True
    # Mock methods that might be called by MultiAxisController or Axis during setup if not fully mocked out
    mock.connect = AsyncMock()
    mock.disconnect = AsyncMock()
    return mock

@pytest.fixture
def mock_multi_axis_controller(mock_can_interface: MagicMock):
    """Mocks the MultiAxisController and its managed axes."""
    controller = MultiAxisController(can_interface_manager=mock_can_interface)
    
    # Mock the axes dictionary and methods that RobotModelBase might call
    controller.axes = {} # Start with an empty dict, axes will be added by specific robot model tests
    controller.get_all_positions_user = AsyncMock(return_value={})
    controller.move_all_to_positions_abs_user = AsyncMock()
    controller.enable_all_axes = AsyncMock()
    controller.initialize_all_axes = AsyncMock()
    controller.disable_all_axes = AsyncMock()
    
    return controller

@pytest.fixture
def two_link_arm_axes(mock_can_interface: MagicMock) -> Dict[str, Axis]:
    """Creates mock Axis objects for a two-link arm."""
    kin_deg = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
    axis1 = MagicMock(spec=Axis)
    axis1.name = "J1_Base"
    axis1.can_id = 1
    axis1.kinematics = kin_deg
    axis1._can_if = mock_can_interface # For RobotModelBase constructor check

    axis2 = MagicMock(spec=Axis)
    axis2.name = "J2_Elbow"
    axis2.can_id = 2
    axis2.kinematics = kin_deg
    axis2._can_if = mock_can_interface

    return {axis1.name: axis1, axis2.name: axis2}


@pytest.fixture
def cartesian_robot_axes(mock_can_interface: MagicMock) -> Dict[str, Axis]:
    """Creates mock Axis objects for a Cartesian robot."""
    kin_mm = LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0, units="mm")
    axis_x = MagicMock(spec=Axis)
    axis_x.name = "AxisX"
    axis_x.can_id = 1
    axis_x.kinematics = kin_mm
    axis_x._can_if = mock_can_interface

    axis_y = MagicMock(spec=Axis)
    axis_y.name = "AxisY"
    axis_y.can_id = 2
    axis_y.kinematics = kin_mm
    axis_y._can_if = mock_can_interface

    axis_z = MagicMock(spec=Axis)
    axis_z.name = "AxisZ"
    axis_z.can_id = 3
    axis_z.kinematics = kin_mm
    axis_z._can_if = mock_can_interface
    
    return {axis_x.name: axis_x, axis_y.name: axis_y, axis_z.name: axis_z}


# --- Tests for RobotModelBase ---

def test_robot_model_base_init_success(mock_multi_axis_controller: MultiAxisController, two_link_arm_axes: Dict[str, Axis]):
    """Tests successful initialization of RobotModelBase."""
    mock_multi_axis_controller.axes = two_link_arm_axes # Populate controller with mock axes
    axis_names = ["J1_Base", "J2_Elbow"]
    
    class DummyRobot(RobotModelBase): # Concrete class for testing
        async def forward_kinematics(self, joint_states): return {}
        async def inverse_kinematics(self, target_pose): return {}

    robot = DummyRobot(mock_multi_axis_controller, axis_names)
    assert robot.dof == 2
    assert robot.axis_names_in_order == axis_names

def test_robot_model_base_init_empty_axis_names(mock_multi_axis_controller: MultiAxisController):
    """Tests RobotModelBase initialization with empty axis_names list."""
    class DummyRobot(RobotModelBase):
        async def forward_kinematics(self, joint_states): return {}
        async def inverse_kinematics(self, target_pose): return {}
    with pytest.raises(ConfigurationError, match="axis_names_in_order cannot be empty"):
        DummyRobot(mock_multi_axis_controller, [])

def test_robot_model_base_init_axis_not_in_controller(mock_multi_axis_controller: MultiAxisController):
    """Tests RobotModelBase initialization when an axis name is not in the controller."""
    class DummyRobot(RobotModelBase):
        async def forward_kinematics(self, joint_states): return {}
        async def inverse_kinematics(self, target_pose): return {}
    with pytest.raises(ConfigurationError, match="Axis 'NonExistentAxis' specified .* not found"):
        DummyRobot(mock_multi_axis_controller, ["NonExistentAxis"])


@pytest.mark.asyncio
async def test_robot_model_base_get_current_joint_states(mock_multi_axis_controller: MultiAxisController, two_link_arm_axes: Dict[str, Axis]):
    """Tests get_current_joint_states method."""
    mock_multi_axis_controller.axes = two_link_arm_axes
    axis_names = ["J1_Base", "J2_Elbow"]
    
    class DummyRobot(RobotModelBase):
        async def forward_kinematics(self, joint_states): return {}
        async def inverse_kinematics(self, target_pose): return {}

    robot = DummyRobot(mock_multi_axis_controller, axis_names)
    
    expected_states = {"J1_Base": 30.0, "J2_Elbow": 45.0}
    mock_multi_axis_controller.get_all_positions_user = AsyncMock(return_value=expected_states)
    
    states = await robot.get_current_joint_states()
    assert states == expected_states
    mock_multi_axis_controller.get_all_positions_user.assert_called_once()


# --- Tests for TwoLinkArmPlanar ---

@pytest.fixture
def arm_robot(mock_multi_axis_controller: MultiAxisController, two_link_arm_axes: Dict[str, Axis]) -> TwoLinkArmPlanar:
    """Fixture for a TwoLinkArmPlanar instance."""
    mock_multi_axis_controller.axes = two_link_arm_axes # Add mock axes to the controller
    return TwoLinkArmPlanar(
        multi_axis_controller=mock_multi_axis_controller,
        axis_names=["J1_Base", "J2_Elbow"],
        link1_length=100.0,
        link2_length=80.0
    )

@pytest.mark.asyncio
@pytest.mark.parametrize("theta1_deg, theta2_deg, expected_x, expected_y", [
    (0, 0, 180.0, 0.0),          # Fully extended along X-axis
    (90, 0, 0.0, 180.0),        # Fully extended along Y-axis
    (0, 90, 100.0, 80.0),       # L1 along X, L2 perpendicular up
    (90, -90, 80.0, 100.0),     # L1 along Y, L2 perpendicular right (elbow "down" visually)
    (45, 30, 
     100 * math.cos(math.radians(45)) + 80 * math.cos(math.radians(45 + 30)),
     100 * math.sin(math.radians(45)) + 80 * math.sin(math.radians(45 + 30))),
])
async def test_two_link_arm_forward_kinematics(
    arm_robot: TwoLinkArmPlanar, 
    theta1_deg: float, theta2_deg: float, 
    expected_x: float, expected_y: float
):
    """Tests TwoLinkArmPlanar forward kinematics."""
    joint_states = [theta1_deg, theta2_deg]
    pose = await arm_robot.forward_kinematics(joint_states)
    assert math.isclose(pose['x'], expected_x, abs_tol=1e-6)
    assert math.isclose(pose['y'], expected_y, abs_tol=1e-6)

    # Test with dict input
    joint_states_dict = {"J1_Base": theta1_deg, "J2_Elbow": theta2_deg}
    pose_dict = await arm_robot.forward_kinematics(joint_states_dict)
    assert math.isclose(pose_dict['x'], expected_x, abs_tol=1e-6)
    assert math.isclose(pose_dict['y'], expected_y, abs_tol=1e-6)


@pytest.mark.asyncio
@pytest.mark.parametrize("target_x, target_y, expected_t1_deg, expected_t2_deg", [
    (180.0, 0.0, 0.0, 0.0),                     # Fully extended
    (100.0, 80.0, 0.0, 90.0),                   # (0, 90) case from FK
    (20.0, 0.0, 0.0, 180.0),                    # Folded back along X (L1-L2)
    (math.cos(math.radians(30))*150, math.sin(math.radians(30))*150, 30.0, 38.94244), # Example point within reach
])
async def test_two_link_arm_inverse_kinematics(
    arm_robot: TwoLinkArmPlanar,
    target_x: float, target_y: float,
    expected_t1_deg: float, expected_t2_deg: float
):
    """Tests TwoLinkArmPlanar inverse kinematics for reachable points."""
    target_pose: CartesianPose = {'x': target_x, 'y': target_y}
    joint_angles = await arm_robot.inverse_kinematics(target_pose)
    
    # Verify by FK if the calculated joint angles result in the target pose
    calculated_pose = await arm_robot.forward_kinematics(joint_angles) # joint_angles is already a dict
    
    assert math.isclose(calculated_pose['x'], target_x, abs_tol=1e-5)
    assert math.isclose(calculated_pose['y'], target_y, abs_tol=1e-5)
    
    # Optionally, check specific angles if they are uniquely determined by the chosen IK solution
    # This depends on the IK solution implemented (e.g. elbow up/down)
    # For the example (100,80), expected is (0, 90)
    if target_x == 100.0 and target_y == 80.0:
         assert math.isclose(joint_angles[arm_robot.axis_names_in_order[0]], expected_t1_deg, abs_tol=1e-5)
         assert math.isclose(joint_angles[arm_robot.axis_names_in_order[1]], expected_t2_deg, abs_tol=1e-5)


@pytest.mark.asyncio
async def test_two_link_arm_ik_unreachable(arm_robot: TwoLinkArmPlanar):
    """Tests TwoLinkArmPlanar IK for an unreachable point."""
    target_pose_far: CartesianPose = {'x': arm_robot.l1 + arm_robot.l2 + 10, 'y': 0} # Too far
    with pytest.raises(KinematicsError, match="Target position .* is unreachable"):
        await arm_robot.inverse_kinematics(target_pose_far)

    target_pose_near: CartesianPose = {'x': abs(arm_robot.l1 - arm_robot.l2) -10, 'y': 0} # Too close (inside dead zone)
    if abs(arm_robot.l1 - arm_robot.l2) > 1e-6 : # Only test if there's a dead zone
        with pytest.raises(KinematicsError, match="Target position .* is unreachable"):
            await arm_robot.inverse_kinematics(target_pose_near)


@pytest.mark.asyncio
async def test_two_link_arm_move_to_cartesian_pose(arm_robot: TwoLinkArmPlanar, mock_multi_axis_controller: MultiAxisController):
    """Tests the move_to_cartesian_pose method of TwoLinkArmPlanar."""
    target_pose: CartesianPose = {'x': 150.0, 'y': 20.0}
    
    # Mock IK to return predictable joint targets
    expected_joint_targets = {arm_robot.axis_names_in_order[0]: 10.0, arm_robot.axis_names_in_order[1]: 30.0}
    arm_robot.inverse_kinematics = AsyncMock(return_value=expected_joint_targets)

    await arm_robot.move_to_cartesian_pose(target_pose, wait_for_all=True)

    arm_robot.inverse_kinematics.assert_called_once_with(target_pose)
    mock_multi_axis_controller.move_all_to_positions_abs_user.assert_called_once_with(
        positions_user=expected_joint_targets,
        speeds_user=None, # No speeds provided in this call
        wait_for_all=True
    )

# --- Tests for CartesianRobot ---

@pytest.fixture
def cartesian_robot_model(mock_multi_axis_controller: MultiAxisController, cartesian_robot_axes: Dict[str, Axis]) -> CartesianRobot:
    """Fixture for a CartesianRobot instance."""
    mock_multi_axis_controller.axes = cartesian_robot_axes
    return CartesianRobot(
        multi_axis_controller=mock_multi_axis_controller,
        x_axis_name="AxisX",
        y_axis_name="AxisY",
        z_axis_name="AxisZ",
        origin_offset=(10.0, 20.0, 5.0) # Example offset
    )

@pytest.mark.asyncio
@pytest.mark.parametrize("jx, jy, jz, ex, ey, ez", [
    (0, 0, 0, 10, 20, 5),       # Robot at its origin, pose is world offset
    (10, 5, 2, 20, 25, 7),      # Robot moved
    (-5, -10, -1, 5, 10, 4),    # Robot moved negatively
])
async def test_cartesian_robot_forward_kinematics(
    cartesian_robot_model: CartesianRobot,
    jx: float, jy: float, jz: float,
    ex: float, ey: float, ez: float
):
    """Tests CartesianRobot forward kinematics."""
    joint_states_list = [jx, jy, jz]
    pose = await cartesian_robot_model.forward_kinematics(joint_states_list)
    assert math.isclose(pose['x'], ex)
    assert math.isclose(pose['y'], ey)
    assert math.isclose(pose['z'], ez)

    joint_states_dict = {"AxisX": jx, "AxisY": jy, "AxisZ": jz}
    pose_dict = await cartesian_robot_model.forward_kinematics(joint_states_dict)
    assert math.isclose(pose_dict['x'], ex)
    assert math.isclose(pose_dict['y'], ey)
    assert math.isclose(pose_dict['z'], ez)


@pytest.mark.asyncio
@pytest.mark.parametrize("tx, ty, tz, ejx, ejy, ejz", [
    (10, 20, 5, 0, 0, 0),       # Target is robot's world origin
    (25, 30, 10, 15, 10, 5),    # Target in world, calculate joint moves
    (0, 0, 0, -10, -20, -5),    # Target at world (0,0,0)
])
async def test_cartesian_robot_inverse_kinematics(
    cartesian_robot_model: CartesianRobot,
    tx: float, ty: float, tz: float,
    ejx: float, ejy: float, ejz: float
):
    """Tests CartesianRobot inverse kinematics."""
    target_pose: CartesianPose = {'x': tx, 'y': ty, 'z': tz}
    joint_targets = await cartesian_robot_model.inverse_kinematics(target_pose)
    
    assert math.isclose(joint_targets[cartesian_robot_model.x_axis_name], ejx)
    assert math.isclose(joint_targets[cartesian_robot_model.y_axis_name], ejy)
    assert math.isclose(joint_targets[cartesian_robot_model.z_axis_name], ejz)

@pytest.mark.asyncio
async def test_cartesian_robot_move_to_cartesian_pose(cartesian_robot_model: CartesianRobot, mock_multi_axis_controller: MultiAxisController):
    """Tests the move_to_cartesian_pose method of CartesianRobot."""
    target_pose: CartesianPose = {'x': 25.0, 'y': 30.0, 'z': 10.0}
    
    # Expected joint targets after accounting for origin_offset (10,20,5)
    expected_joint_targets = {
        cartesian_robot_model.x_axis_name: 15.0, # 25-10
        cartesian_robot_model.y_axis_name: 10.0, # 30-20
        cartesian_robot_model.z_axis_name: 5.0   # 10-5
    }
    # We don't need to mock IK for Cartesian as it's direct.

    await cartesian_robot_model.move_to_cartesian_pose(target_pose, wait_for_all=True)

    mock_multi_axis_controller.move_all_to_positions_abs_user.assert_called_once_with(
        positions_user=expected_joint_targets,
        speeds_user=None,
        wait_for_all=True
    )

@pytest.mark.asyncio
async def test_robot_model_base_get_current_pose(arm_robot: TwoLinkArmPlanar, mock_multi_axis_controller: MultiAxisController):
    """Tests the get_current_pose method of RobotModelBase."""
    # Setup: current joint states and expected FK result from these states
    current_joint_states_dict = {"J1_Base": 30.0, "J2_Elbow": 60.0}
    
    # Manually calculate expected FK for these joint states
    l1, l2 = arm_robot.l1, arm_robot.l2
    t1_rad = math.radians(30.0)
    t2_rad = math.radians(60.0) # This is theta2 relative to link1
    expected_x = l1 * math.cos(t1_rad) + l2 * math.cos(t1_rad + t2_rad) + arm_robot.origin_x
    expected_y = l1 * math.sin(t1_rad) + l2 * math.sin(t1_rad + t2_rad) + arm_robot.origin_y
    expected_pose: CartesianPose = {'x': expected_x, 'y': expected_y}

    # Mock get_all_positions_user to return these states
    mock_multi_axis_controller.get_all_positions_user = AsyncMock(return_value=current_joint_states_dict)
    
    # Call get_current_pose
    actual_pose = await arm_robot.get_current_pose()
    
    # Assertions
    mock_multi_axis_controller.get_all_positions_user.assert_called_once()
    assert math.isclose(actual_pose['x'], expected_pose['x'], abs_tol=1e-6)
    assert math.isclose(actual_pose['y'], expected_pose['y'], abs_tol=1e-6)

