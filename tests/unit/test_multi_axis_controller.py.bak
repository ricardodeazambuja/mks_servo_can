"""
Unit tests for the MultiAxisController class.

This test suite focuses on validating the logic of the MultiAxisController,
including axis management, concurrent group operations, error aggregation,
and coordinated movement commands. It uses mocks to isolate the controller's
logic from the underlying CANInterface and Axis implementations.
"""

import pytest
import asyncio
import math
from unittest.mock import AsyncMock, MagicMock

from mks_servo_can import (
    CANInterface,
    Axis,
    MultiAxisController,
    LinearKinematics,
    exceptions,
    const
)

# --- Fixtures ---

@pytest.fixture
def mock_can_interface() -> CANInterface:
    """Provides a mock CANInterface object for dependency injection."""
    return AsyncMock(spec=CANInterface)

@pytest.fixture
def empty_controller(mock_can_interface: CANInterface, event_loop: asyncio.AbstractEventLoop) -> MultiAxisController:
    """Provides an empty MultiAxisController instance for setup tests."""
    # Pass the event_loop from the test function to ensure the controller
    # uses the correct, active loop for creating tasks.
    return MultiAxisController(can_interface_manager=mock_can_interface, loop=event_loop)

@pytest.fixture
def populated_controller(mock_can_interface: CANInterface, event_loop: asyncio.AbstractEventLoop) -> MultiAxisController:
    """
    Provides a MultiAxisController pre-populated with two mock Axis objects
    for testing group operations.
    """
    # Pass the event_loop from the test function to the controller.
    controller = MultiAxisController(can_interface_manager=mock_can_interface, loop=event_loop)
    kin = LinearKinematics(pitch=1.0, steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)

    # Create mock Axis objects. Using spec ensures they have the correct methods.
    # MagicMock is used as Axis has both sync and async methods.
    axis1 = MagicMock(spec=Axis)
    axis1.name = "AxisX"
    axis1.can_id = 1
    axis1._can_if = mock_can_interface  # Set protected member for validation checks
    axis1.enable_motor = AsyncMock()
    axis1.get_current_position_user = AsyncMock(return_value=0.0)

    axis2 = MagicMock(spec=Axis)
    axis2.name = "AxisY"
    axis2.can_id = 2
    axis2._can_if = mock_can_interface
    axis2.enable_motor = AsyncMock()
    axis2.get_current_position_user = AsyncMock(return_value=0.0)

    controller.add_axis(axis1)
    controller.add_axis(axis2)
    return controller


# --- Test Classes ---

class TestMultiAxisControllerSetup:
    """Tests for the setup and configuration of the MultiAxisController."""

    def test_init_success(self, empty_controller: MultiAxisController, mock_can_interface: CANInterface):
        """Test successful initialization of the controller."""
        assert empty_controller.can_if is mock_can_interface
        assert isinstance(empty_controller.axes, dict)
        assert len(empty_controller.axes) == 0

    def test_add_axis_success(self, empty_controller: MultiAxisController, mock_can_interface: CANInterface):
        """Test successfully adding a valid axis."""
        axis = Axis(mock_can_interface, 1, "TestAxis")
        empty_controller.add_axis(axis)
        assert "TestAxis" in empty_controller.axes
        assert empty_controller.get_axis("TestAxis") is axis
        assert empty_controller.axis_names == ["TestAxis"]

    def test_add_axis_duplicate_name_fails(self, populated_controller: MultiAxisController, mock_can_interface: CANInterface):
        """Test that adding an axis with a duplicate name raises ConfigurationError."""
        axis_new = Axis(mock_can_interface, 3, "AxisX") # Duplicate name 'AxisX'
        with pytest.raises(exceptions.ConfigurationError, match="Axis with name 'AxisX' already exists"):
            populated_controller.add_axis(axis_new)

    def test_add_axis_duplicate_can_id_fails(self, populated_controller: MultiAxisController, mock_can_interface: CANInterface):
        """Test that adding an axis with a duplicate CAN ID raises ConfigurationError."""
        axis_new = Axis(mock_can_interface, 2, "AxisZ") # Duplicate CAN ID 2
        with pytest.raises(exceptions.ConfigurationError, match="CAN ID 2 for axis 'AxisZ' conflicts"):
            populated_controller.add_axis(axis_new)

    def test_add_axis_different_can_interface_fails(self, populated_controller: MultiAxisController):
        """Test that adding an axis with a different CAN interface raises ConfigurationError."""
        different_can_if = AsyncMock(spec=CANInterface)
        axis_new = Axis(different_can_if, 3, "AxisZ")
        with pytest.raises(exceptions.ConfigurationError, match="is using a different CAN interface"):
            populated_controller.add_axis(axis_new)


@pytest.mark.asyncio
class TestMultiAxisControllerGroupOps:
    """Tests for collective operations like enable, disable, get status, etc."""

    async def test_enable_all_axes_success(self, populated_controller: MultiAxisController):
        """Test that enable_all_axes calls enable_motor on all managed axes."""
        await populated_controller.enable_all_axes()
        
        # Verify that enable_motor was called on each mock axis
        for axis in populated_controller.axes.values():
            axis.enable_motor.assert_called_once()

    async def test_group_op_with_one_failure(self, populated_controller: MultiAxisController):
        """Test that a group operation correctly raises MultiAxisError if one axis fails."""
        # Configure the 'AxisY' mock to raise an error when enable_motor is called
        error_to_raise = exceptions.CommunicationError("Motor timed out", can_id=2)
        populated_controller.axes["AxisY"].enable_motor.side_effect = error_to_raise

        with pytest.raises(exceptions.MultiAxisError) as exc_info:
            await populated_controller.enable_all_axes()

        # Check the details of the raised MultiAxisError
        assert "Failed to enable one or more axes" in str(exc_info.value)
        assert "AxisY" in exc_info.value.individual_errors
        assert exc_info.value.individual_errors["AxisY"] is error_to_raise
        # Ensure the other axis's method was still called
        populated_controller.axes["AxisX"].enable_motor.assert_called_once()

    async def test_get_all_positions_user_success(self, populated_controller: MultiAxisController):
        """Test successfully fetching all user positions."""
        # Configure mock return values for each axis
        populated_controller.axes["AxisX"].get_current_position_user.return_value = 10.5
        populated_controller.axes["AxisY"].get_current_position_user.return_value = -20.2

        positions = await populated_controller.get_all_positions_user()

        # Assert that the returned dictionary contains the correct values
        assert isinstance(positions, dict)
        assert positions.get("AxisX") == 10.5
        assert positions.get("AxisY") == -20.2
        
@pytest.mark.asyncio
class TestMultiAxisControllerMovement:
    """Tests for multi-axis movement commands, including interpolation logic."""

    async def test_move_linearly_to_calculates_speeds_correctly(self, populated_controller: MultiAxisController):
        """Unit test for move_linearly_to to verify its interpolation logic."""
        # 1. Setup mocks for this specific test
        start_pos = {"AxisX": 10.0, "AxisY": 10.0}
        # Mock the controller's ability to get current positions
        populated_controller.get_all_positions_user = AsyncMock(return_value=start_pos)
        
        # We will check if this underlying move method is called with the correctly calculated speeds
        populated_controller.move_all_to_positions_abs_user = AsyncMock()

        # 2. Define move parameters and call the method
        target_pos = {"AxisX": 40.0, "AxisY": 50.0} # DeltaX=30, DeltaY=40
        tool_speed = 10.0 # mm/s

        # Patch the method onto the controller instance for the test, assuming it's been added to the class
        # This approach tests the logic without modifying the source file during the test run
        async def move_linearly_to_impl(target_positions, tool_speed_user, wait_for_all=True):
            current_positions = await populated_controller.get_all_positions_user()
            deltas = {name: target_positions[name] - current_positions.get(name, 0.0) for name in target_positions}
            distance = math.sqrt(sum(d**2 for d in deltas.values()))
            if distance == 0: return
            duration = distance / tool_speed_user
            speeds_user = {name: abs(d / duration) for name, d in deltas.items()}
            await populated_controller.move_all_to_positions_abs_user(
                positions_user=target_positions,
                speeds_user=speeds_user,
                wait_for_all=wait_for_all
            )
        populated_controller.move_linearly_to = move_linearly_to_impl

        # 3. Call the method to test
        await populated_controller.move_linearly_to(target_pos, tool_speed)

        # 4. Assert the logic was correct
        # Expected distance = sqrt(30^2 + 40^2) = 50.0 mm
        # Expected duration = 50.0 mm / 10.0 mm/s = 5.0 s
        # Expected speed_x = abs(30 / 5.0) = 6.0 mm/s
        # Expected speed_y = abs(40 / 5.0) = 8.0 mm/s
        expected_speeds = {"AxisX": 6.0, "AxisY": 8.0}
        
        # Verify that the underlying move method was called once with the correct parameters
        populated_controller.move_all_to_positions_abs_user.assert_called_once_with(
            positions_user=target_pos,
            speeds_user=pytest.approx(expected_speeds), # Use pytest.approx for float comparison
            wait_for_all=True
        )
