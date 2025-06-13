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
from typing import Dict, Tuple

from mks_servo_can import CANInterface

from mks_servo_can import (
    Axis, # For type hinting and mock spec
    MultiAxisController, # For type hinting and mock spec
    RotaryKinematics,
    LinearKinematics,
    const,
)
from mks_servo_can.robot_kinematics import (
    RobotModelBase,
    TwoLinkArmPlanar,
    CartesianRobot,
    RRRArm,
    CartesianPose,
    KinematicsError,
    ConfigurationError
)

LINK_1_LENGTH = 100
LINK_2_LENGTH = 50

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

@pytest.fixture
def rrr_arm_axes(mock_can_interface: MagicMock) -> Dict[str, Axis]:
    """Creates mock Axis objects for an RRR arm."""
    # All joints in an RRR arm are rotary
    kin_deg = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
    
    axis_j1 = MagicMock(spec=Axis)
    axis_j1.name = "J1_Base"
    axis_j1.can_id = 1 # Ensure unique CAN IDs if testing multiple robots simultaneously, though typically one robot per test class
    axis_j1.kinematics = kin_deg
    axis_j1._can_if = mock_can_interface # For RobotModelBase constructor check

    axis_j2 = MagicMock(spec=Axis)
    axis_j2.name = "J2_Shoulder"
    axis_j2.can_id = 2
    axis_j2.kinematics = kin_deg
    axis_j2._can_if = mock_can_interface

    axis_j3 = MagicMock(spec=Axis)
    axis_j3.name = "J3_Elbow"
    axis_j3.can_id = 3
    axis_j3.kinematics = kin_deg
    axis_j3._can_if = mock_can_interface
    
    return {axis_j1.name: axis_j1, axis_j2.name: axis_j2, axis_j3.name: axis_j3}

@pytest.fixture
def rrr_robot_model(mock_multi_axis_controller: MultiAxisController, rrr_arm_axes: Dict[str, Axis]) -> RRRArm:
    """Fixture for an RRRArm instance with specific link lengths."""
    mock_multi_axis_controller.axes = rrr_arm_axes # Add mock axes to the controller
    
    # Define your RRR arm's link lengths for testing
    # These should match the geometry for which you'll calculate test cases
    link1_len = LINK_1_LENGTH  # Example: Shoulder to Elbow
    link2_len = LINK_2_LENGTH   # Example: Elbow to End-Effector
    
    return RRRArm(
        multi_axis_controller=mock_multi_axis_controller,
        axis_names=["J1_Base", "J2_Shoulder", "J3_Elbow"], # Ensure order matches your kinematic implementation
        link1_length=link1_len,
        link2_length=link2_len,
        # origin_offset=(0.0, 0.0, 0.0) # Default, or specify if needed
    )

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


# In tests/unit/test_robot_kinematics.py

# ... (other imports and existing code up to the TestRRRArm class) ...

# These constants are already defined at the top of your file:
# LINK_1_LENGTH = 100
# LINK_2_LENGTH = 50

# Helper function to calculate FK for test parametrization (can be defined at module level or within the test class)
def _calculate_rrr_fk_for_test(theta1_deg: float, theta2_deg: float, theta3_deg: float, l1: float, l2: float, origin_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
    """
    Calculates the expected FK for the RRR arm based on the RRRArm class's FK logic.
    Ensure this matches the FK implementation in your RRRArm class.
    """
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    # These equations must exactly match your RRRArm.forward_kinematics implementation
    x_val = math.cos(t1) * (l1 * math.cos(t2) + l2 * math.cos(t2 + t3))
    y_val = math.sin(t1) * (l1 * math.cos(t2) + l2 * math.cos(t2 + t3))
    z_val = l1 * math.sin(t2) + l2 * math.sin(t2 + t3)
    
    return {
        'x': x_val + origin_offset[0],
        'y': y_val + origin_offset[1],
        'z': z_val + origin_offset[2]
    }

class TestRRRArm:
    @pytest.mark.asyncio
    @pytest.mark.parametrize("theta1_deg, theta2_deg, theta3_deg", [
        (0, 0, 0),
        (90, 0, 0),
        (0, 90, 0),
        (0, 45, -30),
        (45, 30, 45), # Another varied case
        (0, -30, 60), # Case with negative shoulder angle
        # Add more diverse test cases for angles
    ])
    async def test_rrr_arm_forward_kinematics(
        self,
        rrr_robot_model: RRRArm, # Uses the fixture which now uses LINK_1_LENGTH, LINK_2_LENGTH
        theta1_deg: float, theta2_deg: float, theta3_deg: float,
    ):
        """Tests RRRArm forward kinematics for various joint configurations."""
        joint_states_list = [theta1_deg, theta2_deg, theta3_deg]

        # Calculate expected pose using the helper and the constants
        # Assuming rrr_robot_model fixture correctly uses LINK_1_LENGTH and LINK_2_LENGTH
        # and has an origin_offset if your RRRArm class uses it.
        # The rrr_robot_model fixture will have self.l1 and self.l2 set from these constants.
        expected_pose_dict = _calculate_rrr_fk_for_test(
            theta1_deg, theta2_deg, theta3_deg,
            rrr_robot_model.l1, # Accessing the actual link lengths from the model instance
            rrr_robot_model.l2,
            (rrr_robot_model.origin_x, rrr_robot_model.origin_y, rrr_robot_model.origin_z) # Use model's origin
        )
        expected_x = expected_pose_dict['x']
        expected_y = expected_pose_dict['y']
        expected_z = expected_pose_dict['z']

        pose = await rrr_robot_model.forward_kinematics(joint_states_list)
        
        assert math.isclose(pose.get('x', float('nan')), expected_x, abs_tol=1e-3), f"FK X failed for {joint_states_list}"
        assert math.isclose(pose.get('y', float('nan')), expected_y, abs_tol=1e-3), f"FK Y failed for {joint_states_list}"
        assert math.isclose(pose.get('z', float('nan')), expected_z, abs_tol=1e-3), f"FK Z failed for {joint_states_list}"

        # Test with dict input
        joint_states_dict = {
            rrr_robot_model.axis_names_in_order[0]: theta1_deg,
            rrr_robot_model.axis_names_in_order[1]: theta2_deg,
            rrr_robot_model.axis_names_in_order[2]: theta3_deg
        }
        pose_dict_input = await rrr_robot_model.forward_kinematics(joint_states_dict)
        assert math.isclose(pose_dict_input.get('x', float('nan')), expected_x, abs_tol=1e-3)
        assert math.isclose(pose_dict_input.get('y', float('nan')), expected_y, abs_tol=1e-3)
        assert math.isclose(pose_dict_input.get('z', float('nan')), expected_z, abs_tol=1e-3)

    @pytest.mark.asyncio
    @pytest.mark.parametrize("target_pose_coords", [
        # Target poses derived from FK calculations with LINK_1_LENGTH, LINK_2_LENGTH
        # Example: (0,0,0) joints -> FK gives (L1+L2, 0, 0) if origin is (0,0,0)
        _calculate_rrr_fk_for_test(0, 0, 0, LINK_1_LENGTH, LINK_2_LENGTH),
        _calculate_rrr_fk_for_test(30, 20, 10, LINK_1_LENGTH, LINK_2_LENGTH),
        _calculate_rrr_fk_for_test(0, 45, 45, LINK_1_LENGTH, LINK_2_LENGTH), # Elbow bent forward
        _calculate_rrr_fk_for_test(0, 60, -30, LINK_1_LENGTH, LINK_2_LENGTH),
        # Add a few more varied, reachable points
    ])
    async def test_rrr_arm_inverse_kinematics_reachable(
        self,
        rrr_robot_model: RRRArm,
        target_pose_coords: Dict[str, float] # This will be {'x': val, 'y': val, 'z': val}
    ):
        """Tests RRRArm inverse kinematics for reachable points by checking FK(IK(pose)) == pose."""
        target_pose: CartesianPose = target_pose_coords # Already a dict
        
        try:
            joint_angles_dict = await rrr_robot_model.inverse_kinematics(target_pose)
            
            joint_angles_for_fk = [
                joint_angles_dict[rrr_robot_model.axis_names_in_order[0]],
                joint_angles_dict[rrr_robot_model.axis_names_in_order[1]],
                joint_angles_dict[rrr_robot_model.axis_names_in_order[2]],
            ]
            # Recalculate pose using the robot model's FK, which uses its configured link lengths
            calculated_pose = await rrr_robot_model.forward_kinematics(joint_angles_for_fk)
            
            assert math.isclose(calculated_pose.get('x', float('nan')), target_pose['x'], abs_tol=1e-3), f"IK->FK X mismatch for target {target_pose}"
            assert math.isclose(calculated_pose.get('y', float('nan')), target_pose['y'], abs_tol=1e-3), f"IK->FK Y mismatch for target {target_pose}"
            assert math.isclose(calculated_pose.get('z', float('nan')), target_pose['z'], abs_tol=1e-3), f"IK->FK Z mismatch for target {target_pose}"

        except KinematicsError as e:
            pytest.fail(f"IK failed for a supposedly reachable target {target_pose} (derived from FK): {e}")

    @pytest.mark.asyncio
    async def test_rrr_arm_ik_unreachable(self, rrr_robot_model: RRRArm):
        """Tests RRRArm IK for points that should be unreachable."""
        # Max reach is based on the link lengths in the rrr_robot_model instance
        max_reach = rrr_robot_model.l1 + rrr_robot_model.l2
        
        target_pose_far: CartesianPose = {
            'x': (max_reach + 50.0) + rrr_robot_model.origin_x, # Add origin offset to ensure it's far in world frame
            'y': 0.0 + rrr_robot_model.origin_y,
            'z': 0.0 + rrr_robot_model.origin_z
        }
        with pytest.raises(KinematicsError, match="(?i)unreachable"): # Case-insensitive match for "unreachable"
            await rrr_robot_model.inverse_kinematics(target_pose_far)

        # Example: A point potentially too close (inside dead zone, if one exists and IK handles it)
        # This depends on the specific RRR geometry (e.g. if l1 > l2 or vice-versa, how it folds)
        # and whether your IK implementation explicitly checks for this.
        # For a typical RRR, a point very close to the base along Z might be hard if l1 is large.
        # target_pose_near: CartesianPose = {
        #     'x': 0.1 + rrr_robot_model.origin_x,
        #     'y': 0.1 + rrr_robot_model.origin_y,
        #     'z': 0.1 + rrr_robot_model.origin_z
        # }
        # if abs(rrr_robot_model.l1 - rrr_robot_model.l2) > 0.2: # Simplified check for a potential dead zone
        #     with pytest.raises(KinematicsError, match="(?i)unreachable"):
        #         await rrr_robot_model.inverse_kinematics(target_pose_near)


    @pytest.mark.asyncio
    async def test_rrr_arm_move_to_cartesian_pose(
        self, rrr_robot_model: RRRArm, mock_multi_axis_controller: MultiAxisController
    ):
        """Tests the move_to_cartesian_pose method of RRRArm."""
        # Define a reachable target pose using the model's link lengths (which come from constants)
        # Use the FK helper to get a known reachable point if origin is zero
        target_coords = _calculate_rrr_fk_for_test(10, 30, 20, rrr_robot_model.l1, rrr_robot_model.l2,
                                                   (rrr_robot_model.origin_x, rrr_robot_model.origin_y, rrr_robot_model.origin_z))
        target_pose: CartesianPose = {'x': target_coords['x'], 'y': target_coords['y'], 'z': target_coords['z']}
        
        # Mock the IK to return predictable joint targets for this test
        expected_joint_targets_dict = {
            rrr_robot_model.axis_names_in_order[0]: 10.0, # Corresponds to theta1=10
            rrr_robot_model.axis_names_in_order[1]: 30.0, # Corresponds to theta2=30
            rrr_robot_model.axis_names_in_order[2]: 20.0  # Corresponds to theta3=20
        }
        # Patch the robot model's IK method for this test
        rrr_robot_model.inverse_kinematics = AsyncMock(return_value=expected_joint_targets_dict)

        await rrr_robot_model.move_to_cartesian_pose(target_pose, wait_for_all=True)

        rrr_robot_model.inverse_kinematics.assert_called_once_with(target_pose)
        mock_multi_axis_controller.move_all_to_positions_abs_user.assert_called_once_with(
            positions_user=expected_joint_targets_dict,
            speeds_user=None,
            wait_for_all=True
        )

    @pytest.mark.asyncio
    async def test_rrr_arm_get_current_pose(
        self, rrr_robot_model: RRRArm, mock_multi_axis_controller: MultiAxisController
    ):
        """Tests the get_current_pose method of RRRArm."""
        current_joint_states_dict = {
            rrr_robot_model.axis_names_in_order[0]: 15.0,
            rrr_robot_model.axis_names_in_order[1]: 25.0,
            rrr_robot_model.axis_names_in_order[2]: 35.0
        }
        
        mock_multi_axis_controller.get_all_positions_user = AsyncMock(return_value=current_joint_states_dict)
        
        # Calculate expected pose using the same method as in FK test, but with model's actual link lengths
        expected_pose_from_fk = _calculate_rrr_fk_for_test(
            current_joint_states_dict[rrr_robot_model.axis_names_in_order[0]],
            current_joint_states_dict[rrr_robot_model.axis_names_in_order[1]],
            current_joint_states_dict[rrr_robot_model.axis_names_in_order[2]],
            rrr_robot_model.l1,
            rrr_robot_model.l2,
            (rrr_robot_model.origin_x, rrr_robot_model.origin_y, rrr_robot_model.origin_z)
        )

        actual_pose = await rrr_robot_model.get_current_pose()
        
        mock_multi_axis_controller.get_all_positions_user.assert_called_once()
        assert math.isclose(actual_pose.get('x', float('nan')), expected_pose_from_fk['x'], abs_tol=1e-3)
        assert math.isclose(actual_pose.get('y', float('nan')), expected_pose_from_fk['y'], abs_tol=1e-3)
        assert math.isclose(actual_pose.get('z', float('nan')), expected_pose_from_fk['z'], abs_tol=1e-3)