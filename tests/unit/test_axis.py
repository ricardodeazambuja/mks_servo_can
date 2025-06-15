"""Unit tests for the `Axis` class in the `mks_servo_can` library.

These tests focus on isolating the logic within the Axis class by mocking
its dependencies, such as `CANInterface` and `LowLevelAPI`.
"""
# mks_servo_can_project/tests/unit/test_axis.py
# This line indicates the path to the unit test file for the Axis class.
# These unit tests are designed to verify the functionality of the high-level 'Axis' class,
# which provides an abstraction for controlling a single MKS servo motor.
# It will typically involve mocking the LowLevelAPI and CANInterface to isolate Axis logic.

import asyncio
# Imports the 'asyncio' library, which is fundamental for the asynchronous operations
# used in the 'mks_servo_can' library and its tests.

import pytest
# Imports the 'pytest' framework, used for writing and running these tests.

from unittest.mock import AsyncMock
# Imports 'AsyncMock' from 'unittest.mock' to create mock objects for asynchronous methods/classes.

from unittest.mock import MagicMock
# Imports 'MagicMock', a versatile mock object that can mock synchronous methods and attributes.

from unittest.mock import patch
# Imports 'patch' from 'unittest.mock', used as a decorator or context manager
# to replace objects with mocks within a specific scope (e.g., a test function).

from unittest.mock import ANY # Import ANY for flexible argument matching
# Imports 'ANY' from 'unittest.mock', which is a special object that compares equal to anything.
# It's useful in assertions when the exact value of an argument doesn't matter,
# but its presence or type might.

from mks_servo_can import constants as const
# Imports the 'constants' module (aliased as 'const') from the 'mks_servo_can' library.
# This provides access to predefined constants used by the Axis class.

from mks_servo_can.axis import Axis
# Imports the 'Axis' class, which is the System Under Test (SUT) for this file.

from mks_servo_can.can_interface import CANInterface
# Imports 'CANInterface', which is a dependency of 'Axis' and will be mocked
# to simulate CAN communication without actual hardware or a full simulator.

from mks_servo_can.exceptions import CalibrationError
# Imports specific exception types. These are used to assert that the 'Axis' class
# raises or correctly handles these errors under various conditions.

from mks_servo_can.exceptions import CommunicationError
# For errors related to communication timeouts or failures.

from mks_servo_can.exceptions import LimitError
# For errors indicating a physical or software limit has been reached.

from mks_servo_can.exceptions import ParameterError
# For errors due to invalid parameters provided to Axis methods.

from mks_servo_can.exceptions import MotorError
# For general errors reported by the motor or during motor operations.

from mks_servo_can.kinematics import RotaryKinematics
# Imports 'RotaryKinematics' as it's the default kinematics type for an Axis
# and is used in setting up test instances.

from mks_servo_can.low_level_api import LowLevelAPI
# Imports 'LowLevelAPI', another dependency of 'Axis' that will be mocked.
# The Axis class delegates many hardware-specific commands to the LowLevelAPI.

try:
    # This try-except block attempts to import 'Message' from 'python-can'.
    from can import Message as CanMessage
except ImportError:
    # If 'python-can' is not available (e.g., in a minimal test environment
    # or if python-can is an optional dependency not installed),
    # a dummy 'CanMessage' class is defined.
    # This ensures tests can run without the full 'python-can' stack,
    # as the actual CAN communication is mocked in these unit tests.
    class CanMessage: # type: ignore
        # The dummy CanMessage class definition.
        # It includes attributes commonly accessed in the library or tests
        # to prevent AttributeError if the real 'can.Message' is unavailable.
        def __init__(
            self, arbitration_id=0, data=None, dlc=0, is_extended_id=False # type: ignore
        ):
            self.arbitration_id = arbitration_id
            # Stores the CAN message ID.
            self.data = data if data is not None else b""
            # Stores the data payload, defaulting to empty bytes.
            self.dlc = dlc if data is None else len(self.data) # type: ignore
            # Data Length Code, calculated from data length if not provided.
            self.is_extended_id = is_extended_id
            # Flag for extended CAN ID (MKS servos use standard 11-bit IDs).
            self.timestamp = 0.0
            # Timestamp of the message, initialized to 0.0 for the dummy.
            self.channel = None
            # CAN channel the message was received on or sent through.
            self.is_rx = True
            # Flag indicating if the message was received (True) or transmitted (False).
            # Dummy defaults to True as tests often mock received messages.
            self.is_error_frame = False
            # Flag for CAN error frame.
            self.is_remote_frame = False
            # Flag for Remote Transmission Request (RTR) frame.

@pytest.fixture
# Defines a pytest fixture named 'mock_can_interface_for_axis'.
# Pytest fixtures are reusable setup functions for tests.
# This fixture provides a mocked 'CANInterface' object.
def mock_can_interface_for_axis():
    # Creates an AsyncMock instance that specifically mimics the CANInterface.
    # 'spec=CANInterface' ensures the mock has the same attributes/methods as the real class,
    # raising an AttributeError if the test tries to access a non-existent attribute.
    mock = AsyncMock(spec=CANInterface)
    # Mocks the 'send_and_wait_for_response' async method of CANInterface.
    mock.send_and_wait_for_response = AsyncMock()
    # Mocks the 'send_message' async method of CANInterface.
    mock.send_message = AsyncMock()
    # 'CANInterface.create_response_future' is a SYNCHRONOUS method that returns an asyncio.Future.
    # Therefore, its mock should be a synchronous MagicMock, not an AsyncMock.
    mock.create_response_future = MagicMock()
    # Returns the configured mock CANInterface.
    return mock

@pytest.fixture
# Defines a pytest fixture named 'mock_low_level_api'.
# This provides a mocked 'LowLevelAPI' object.
def mock_low_level_api():
    # Creates an AsyncMock that mimics the LowLevelAPI.
    mock = AsyncMock(spec=LowLevelAPI)
    # Sets default return values for various LowLevelAPI methods that are likely
    # to be called during Axis operations. This simplifies test setup by providing
    # default "successful" or neutral responses.
    mock.read_encoder_value_addition = AsyncMock(return_value=0)
    # Mocks reading encoder value, returning 0 by default.
    mock.read_en_pin_status = AsyncMock(return_value=False)
    # Mocks reading enable pin status, returning False (disabled) by default.
    mock.query_motor_status = AsyncMock(return_value=const.MOTOR_STATUS_STOPPED)
    # Mocks querying motor status, returning 'MOTOR_STATUS_STOPPED' by default.
    mock.run_position_mode_relative_pulses = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    # Mocks initiating a relative pulse move, returning 'POS_RUN_STARTING' status.
    mock.run_position_mode_absolute_pulses = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    # Mocks initiating an absolute pulse move, returning 'POS_RUN_STARTING' status.
    mock.run_position_mode_relative_axis = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    mock.run_position_mode_absolute_axis = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    mock.enable_motor = AsyncMock()
    # Mocks enabling the motor. Since it's an action, no specific return value
    # is typically needed beyond not raising an error for a successful mock call.
    mock.calibrate_encoder = AsyncMock()
    # Mocks calibrating the encoder.
    mock.go_home = AsyncMock(return_value=const.HOME_START)
    # Mocks initiating a homing sequence, returning 'HOME_START' status.
    return mock
    # Returns the configured mock LowLevelAPI.

@pytest.fixture
# Defines a pytest fixture named 'axis_instance'.
# This fixture provides a fully initialized 'Axis' object with its dependencies (LowLevelAPI) mocked.
# It takes the previously defined mock fixtures ('mock_can_interface_for_axis', 'mock_low_level_api') as arguments.
# Pytest automatically resolves and injects these dependent fixtures.
def axis_instance(
    mock_can_interface_for_axis: MagicMock, # Type hint for the injected mock CANInterface
    mock_low_level_api: AsyncMock           # Type hint for the injected mock LowLevelAPI
):
    # The 'patch' context manager from 'unittest.mock' is used here.
    # It temporarily replaces the actual 'LowLevelAPI' class within the 'mks_servo_can.axis' module
    # with the 'mock_low_level_api' instance *specifically during the instantiation of an Axis object below*.
    # This ensures that when 'Axis' tries to create its own 'LowLevelAPI' instance internally,
    # it receives our pre-configured mock instead of a real one.
    with patch(
        "mks_servo_can.axis.LowLevelAPI", # The string path to the class to be patched.
        return_value=mock_low_level_api, # When 'LowLevelAPI(...)' is called inside Axis, return this mock instance.
    ):
        # Creates a default RotaryKinematics object for the test Axis instance.
        # This is used because Axis requires a kinematics object.
        kin = RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        )
        # Instantiates the 'Axis' class (the System Under Test - SUT).
        # It's provided with the mocked CANInterface manager.
        axis = Axis(
            can_interface_manager=mock_can_interface_for_axis,
            motor_can_id=1, # Assigns CAN ID 1 to this test axis.
            name="TestAxis", # Assigns a descriptive name.
            kinematics=kin, # Uses the default RotaryKinematics.
        )
        # Although 'patch' should handle the instantiation of '_low_level_api' within 'Axis'
        # to be 'mock_low_level_api', this line explicitly assigns it.
        # This ensures that the 'axis' instance definitely uses our provided mock for LowLevelAPI,
        # which can be useful for direct assertions on 'axis._low_level_api' if needed,
        # and provides an extra layer of certainty that the mock is in place.
        axis._low_level_api = mock_low_level_api
        return axis
        # Returns the fully configured Axis instance with mocked dependencies.

@pytest.mark.asyncio
# Marks this entire test class for asynchronous tests. Pytest-asyncio will handle running them.
class TestAxisBasicOps:
    # This test class groups tests for basic operations and initialization aspects of the Axis class.

    @pytest.mark.asyncio
    async def test_axis_init(self, mock_can_interface_for_axis: MagicMock):
        # Tests the successful initialization of an Axis object.
        # It verifies that attributes are set correctly based on constructor arguments.
        # This test does NOT use the 'axis_instance' fixture because it specifically tests
        # the Axis constructor logic without the automatic patching of LowLevelAPI by that fixture.
        kin = RotaryKinematics(steps_per_revolution=1000)
        # Creates a specific kinematics object for this test.
        axis = Axis(mock_can_interface_for_axis, 1, "MyAxis", kinematics=kin)
        # Instantiates the Axis with the mocked CAN interface and custom kinematics.
        assert axis.name == "MyAxis"
        # Verifies that the 'name' attribute is stored correctly.
        assert axis.can_id == 1
        # Verifies that the 'can_id' attribute is stored correctly.
        assert axis.kinematics == kin
        # Verifies that the provided 'kinematics' object is stored.
        assert axis._low_level_api is not None
        # Asserts that the internal '_low_level_api' instance is created by the Axis constructor.
        # In this test, it would be a real LowLevelAPI instance (or a patched one if global patching were active).
        assert axis.is_enabled() is False
        # Asserts that the default 'is_enabled' state (a cached flag) is False.
        assert axis.is_homed() is False
        # Asserts that the default 'is_homed' state is False.
        assert axis.is_calibrated() is False
        # Asserts that the default 'is_calibrated' state is False.

    @pytest.mark.asyncio
    async def test_axis_init_default_kinematics(
        self, mock_can_interface_for_axis: MagicMock
    ):
        # Tests that if no kinematics object is provided to the Axis constructor,
        # it correctly defaults to RotaryKinematics with the standard encoder pulse count.
        axis = Axis(mock_can_interface_for_axis, 1, "DefaultKinAxis")
        # Instantiates Axis without providing an explicit kinematics object.
        assert isinstance(axis.kinematics, RotaryKinematics)
        # Verifies that the type of the automatically created kinematics object is RotaryKinematics.
        assert (
            axis.kinematics.steps_per_revolution
            == const.ENCODER_PULSES_PER_REVOLUTION
        )
        # Verifies that the default 'steps_per_revolution' for the default kinematics
        # matches the library constant 'ENCODER_PULSES_PER_REVOLUTION'.

    @pytest.mark.asyncio
    async def test_axis_init_invalid_can_id(self, mock_can_interface_for_axis: MagicMock):
        # Tests that the Axis constructor raises a 'ParameterError' if an invalid CAN ID is provided.
        # Valid CAN IDs for MKS servos are typically 1 to 0x7FF (2047).
        with pytest.raises(ParameterError):
            # Asserts that creating an Axis with CAN ID 0 (broadcast address, not valid for a single axis object)
            # raises a ParameterError.
            Axis(mock_can_interface_for_axis, 0, "Axis0")
        with pytest.raises(ParameterError):
            # Asserts that creating an Axis with CAN ID 0x800 (2048, which is out of the standard 11-bit range)
            # raises a ParameterError.
            Axis(mock_can_interface_for_axis, 0x800, "AxisHigh")

    @pytest.mark.asyncio
    async def test_enable_motor(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests the 'enable_motor' method of the Axis class.
        # It uses the 'axis_instance' fixture (which has a mocked LowLevelAPI)
        # and the 'mock_low_level_api' fixture for assertions on the mock.
        await axis_instance.enable_motor()
        # Calls the 'enable_motor' method on the Axis instance.
        mock_low_level_api.enable_motor.assert_called_once_with(
            axis_instance.can_id, True # Expect LowLevelAPI.enable_motor(can_id, enable=True)
        )
        # Asserts that the LowLevelAPI's 'enable_motor' method was called exactly once
        # with the correct CAN ID of the axis and 'True' (to enable the motor).
        assert axis_instance._is_enabled is True # Internal flag should be updated
        # Asserts that the Axis's internal '_is_enabled' flag is set to True after the call.
        
        # Test idempotency: calling enable again when already enabled should not re-call low-level API
        mock_low_level_api.enable_motor.reset_mock() # Reset call count for next check
        # Resets the mock to clear previous call information (like call_count).
        await axis_instance.enable_motor()
        # Calls 'enable_motor' again on the already enabled axis.
        mock_low_level_api.enable_motor.assert_not_called()
        # Asserts that 'enable_motor' on the LowLevelAPI was NOT called again,
        # because the Axis class should optimize by checking its cached state first.

    @pytest.mark.asyncio
    async def test_disable_motor(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests the 'disable_motor' method, similar in structure to 'test_enable_motor'.
        axis_instance._is_enabled = True # Manually set the state to enabled for this test's starting condition.
        await axis_instance.disable_motor()
        # Calls the 'disable_motor' method on the Axis instance.
        mock_low_level_api.enable_motor.assert_called_once_with(
            axis_instance.can_id, False # Note: LowLevelAPI.enable_motor(..., False) is used for disabling.
        )
        # Asserts that LowLevelAPI's 'enable_motor' was called once with 'False' to disable.
        assert axis_instance._is_enabled is False # Internal flag should be updated.
        # Asserts that the Axis's internal '_is_enabled' flag is set to False.

        # Test idempotency: calling disable again when already disabled
        mock_low_level_api.enable_motor.reset_mock()
        # Resets the mock.
        await axis_instance.disable_motor()
        # Calls 'disable_motor' again on the already disabled axis.
        mock_low_level_api.enable_motor.assert_not_called()
        # Asserts that LowLevelAPI's 'enable_motor' was NOT called again.

    @pytest.mark.asyncio
    async def test_get_current_position_user(
        self, axis_instance: Axis, mock_low_level_api: AsyncMock
    ):
        # Tests the 'get_current_position_user' method.
        # This method should read the raw step count from the LowLevelAPI
        # and then convert it to user-defined units using the axis's kinematics object.
        mock_steps = 8192 # Example raw step count (e.g., half a revolution for 16384 steps/rev motor).
        # Configure the mock LowLevelAPI's 'read_encoder_value_addition' to return 'mock_steps'.
        mock_low_level_api.read_encoder_value_addition.return_value = mock_steps
        
        # The 'axis_instance' fixture by default uses RotaryKinematics with ENCODER_PULSES_PER_REVOLUTION (16384).
        # So, 8192 steps should correspond to 180.0 degrees.
        user_pos = await axis_instance.get_current_position_user()
        # Calls the method to get position in user units.
        assert user_pos == pytest.approx(180.0) # Using pytest.approx for float comparison.
        # Asserts that the converted user position is approximately 180.0.
        mock_low_level_api.read_encoder_value_addition.assert_called_once_with(
            axis_instance.can_id
        )
        # Asserts that the LowLevelAPI was called to read the raw steps.
        assert axis_instance._current_position_steps == mock_steps
        # Asserts that the Axis's internal cache for raw steps ('_current_position_steps') is updated.

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisInitialization:
    # This test class focuses on the 'initialize' method of the Axis class,
    # covering scenarios with and without calibration/homing, and error conditions.

    @pytest.mark.asyncio
    async def test_initialize_success_no_actions(
        self, axis_instance: Axis, mock_low_level_api: AsyncMock
    ):
        # Tests a successful initialization sequence where no calibration or homing is requested by the caller.
        # The 'initialize' method should still read initial motor status like position and enable state.
        mock_low_level_api.read_encoder_value_addition.return_value = 100 # Simulate initial position in steps.
        mock_low_level_api.read_en_pin_status.return_value = True      # Simulate initial enable state (True = enabled).
        
        await axis_instance.initialize(calibrate=False, home=False)
        # Calls initialize with 'calibrate' and 'home' flags set to False.

        mock_low_level_api.read_encoder_value_addition.assert_called_once() # Verify position was read.
        mock_low_level_api.read_en_pin_status.assert_called_once()         # Verify enable status was read.
        mock_low_level_api.calibrate_encoder.assert_not_called()           # Verify calibration was NOT called.
        # For 'home=False', the Axis's own 'home_axis' method should not be called.
        # We can assert this by patching 'axis_instance.home_axis' if 'initialize' directly calls it,
        # or by checking that 'mock_low_level_api.go_home' (if it's the direct path) isn't called.
        # A simpler check here is that the internal _is_homed flag remains False (its default).
        assert axis_instance._is_homed is False # Ensure homing didn't inadvertently occur.
        
        assert axis_instance._is_enabled is True # Verify the internal enabled flag is updated from read_en_pin_status.
        assert axis_instance._current_position_steps == 100 # Verify internal position steps cache is updated.
        # After initialization without calibration, the _is_calibrated flag should be True
        # (as per current Axis.initialize logic: if not calibrate, it sets _is_calibrated = True).
        assert axis_instance._is_calibrated is True 

    @pytest.mark.asyncio
    async def test_initialize_with_calibrate_and_home(
        self, axis_instance: Axis, mock_low_level_api: AsyncMock
    ):
        # Tests initialization with both calibration and homing requested.
        # Assumes calibration will succeed.
        mock_low_level_api.read_encoder_value_addition.return_value = 0 # Initial position before actions.
        mock_low_level_api.read_en_pin_status.return_value = False   # Assume motor starts disabled.
        
        # Configure the mock for calibrate_encoder to simulate success (no exception, no specific return needed).
        mock_low_level_api.calibrate_encoder = AsyncMock(return_value=None) 

        # The 'home_axis' method of the Axis class itself is a complex operation.
        # To isolate the 'initialize' method's logic of *calling* 'home_axis',
        # we patch 'axis_instance.home_axis' directly with a new AsyncMock.
        with patch.object(
            axis_instance, "home_axis", new_callable=AsyncMock
        ) as mock_axis_home:
            # Calls 'initialize' requesting both calibration and homing.
            await axis_instance.initialize(calibrate=True, home=True)
            # Assert that the (now mocked) 'home_axis' method of the 'axis_instance' was called once.
            mock_axis_home.assert_called_once()

        # Verify that the LowLevelAPI's 'calibrate_encoder' was called once with the correct CAN ID.
        mock_low_level_api.calibrate_encoder.assert_called_once_with(axis_instance.can_id)
        # Verify that the Axis's internal '_is_calibrated' flag is set to True after successful calibration.
        assert axis_instance._is_calibrated is True
        # Note: Whether '_is_homed' is True depends on the behavior of the mocked 'home_axis'.
        # Since 'home_axis' is mocked here, we don't assert '_is_homed' based on this mock directly,
        # but rather trust that 'initialize' correctly delegated the homing call.

    @pytest.mark.asyncio
    async def test_initialize_calibrate_fails_no_home(
        self, axis_instance: Axis, mock_low_level_api: AsyncMock
    ):
        # Tests the scenario where 'initialize' requests calibration, but the calibration process fails.
        # In this case, homing (if also requested) should NOT be attempted.
        mock_low_level_api.read_encoder_value_addition.return_value = 0
        mock_low_level_api.read_en_pin_status.return_value = False
        # Configure the mock 'calibrate_encoder' to raise a 'CalibrationError' when called.
        mock_low_level_api.calibrate_encoder.side_effect = CalibrationError(
            "Simulated calib fail"
        )

        # Patch 'axis_instance.home_axis' to ensure it's not called if calibration fails.
        with patch.object(axis_instance, "home_axis", new_callable=AsyncMock) as mock_axis_home:
            # Assert that calling 'initialize' (with calibrate=True and home=True)
            # re-raises the 'CalibrationError' that originated from 'calibrate_encoder'.
            with pytest.raises(CalibrationError, match="Simulated calib fail"):
                await axis_instance.initialize(calibrate=True, home=True)
            # Assert that the mocked 'home_axis' method was NOT called, because calibration failed.
            mock_axis_home.assert_not_called()

        # Verify that 'calibrate_encoder' on the LowLevelAPI was attempted once.
        mock_low_level_api.calibrate_encoder.assert_called_once_with(axis_instance.can_id)
        
        # Verify that the Axis's internal '_is_calibrated' flag is False due to the failure.
        assert axis_instance._is_calibrated is False
        # Verify that the Axis's internal '_is_homed' flag is also False, as homing was skipped.
        assert axis_instance._is_homed is False

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisMovement:
    # This test class focuses on the movement methods of the Axis class,
    # such as relative moves, absolute moves, handling of limits, and timeouts.

    @pytest.mark.asyncio
    async def test_move_relative_pulses_success(
        self,
        axis_instance: Axis, # The Axis instance with mocked dependencies.
        mock_low_level_api: AsyncMock, # Mock for LowLevelAPI for assertions.
        mock_can_interface_for_axis: MagicMock, # Mock for CANInterface, particularly for 'create_response_future'.
    ):
        # Tests a successful relative move specified in raw motor pulses.
        # This test simulates the entire asynchronous flow:
        # 1. Axis calls LowLevelAPI's 'run_position_mode_relative_pulses' to start the move.
        # 2. The mocked LowLevelAPI returns a "starting" status (e.g., const.POS_RUN_STARTING).
        # 3. Axis then calls CANInterface's 'create_response_future' to prepare for a completion signal from the motor.
        # 4. This test simulates the arrival of that "move complete" CAN message by resolving the future.
        # 5. Finally, it asserts that the Axis method completes successfully and its internal state (like position) is updated.
        
        relative_pulses = 1000      # Define the number of pulses for the relative move.
        speed_param = 500           # Define the MKS speed parameter for the move.
        accel_param = 100           # Define the MKS acceleration parameter.
        can_id = axis_instance.can_id # Get the CAN ID from the axis instance.
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES # The specific MKS command code for this move type.

        # Configure the mock LowLevelAPI to indicate the move is starting successfully.
        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )

        # Create an asyncio.Future object. This future will be used by the mock_can_interface
        # to simulate the asynchronous arrival of the "move complete" CAN message.
        # The Axis._execute_move method will await this future.
        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        # Configure the mocked 'create_response_future' on the CANInterface to return our test-controlled future.
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )

        # Create an asyncio task to run the 'move_relative_pulses' method on the Axis.
        # 'wait=True' means the Axis method itself will internally await the completion of the move.
        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(
                relative_pulses, speed_param, accel_param, wait=True
            )
        )

        # Allow the 'move_task' to start and reach the point where 'Axis._execute_move'
        # is awaiting 'current_test_completion_future'.
        await asyncio.sleep(0.01) 
        assert (
            not current_test_completion_future.done()
        ), "Completion future should be actively awaited by _execute_move at this point."
        # Asserts that the future (which signals move completion) is not yet done,
        # confirming that '_execute_move' is correctly waiting on it.

        # Simulate the "move complete" CAN message arriving from the motor.
        # Construct the data payload for this simulated CAN message.
        # It includes the echoed command code and the "position run complete" status.
        response_payload_data = [cmd_code, const.POS_RUN_COMPLETE] 
        # Calculate a simplified CRC for the test response.
        response_crc = (can_id + sum(response_payload_data)) & 0xFF 
        completion_msg_data = bytes(response_payload_data + [response_crc])
        
        # Create a dummy CanMessage object representing the completion signal.
        simulated_can_msg = CanMessage( 
            arbitration_id=can_id,      # The message comes from the motor's CAN ID.
            data=completion_msg_data,   # The constructed data payload.
            dlc=len(completion_msg_data), # Data Length Code.
        )
        # Set the result of 'current_test_completion_future' with the simulated CAN message.
        # This will unblock the 'await completion_future' line within 'Axis._execute_move'.
        current_test_completion_future.set_result(simulated_can_msg)


        # Wait for the 'move_relative_pulses' task (and thus _execute_move) to fully finish.
        # This should complete now that 'current_test_completion_future' has a result.
        await move_task 

        # Assert that the LowLevelAPI's 'run_position_mode_relative_pulses' was called once
        # with the correct arguments to initiate the move.
        # Direction is True (CCW) for positive 'relative_pulses'.
        mock_low_level_api.run_position_mode_relative_pulses.assert_called_once_with(
            can_id, True, speed_param, accel_param, abs(relative_pulses)
        )
        # Assert that 'CANInterface.create_response_future' was called once to set up
        # the wait for the completion signal.
        # ANY is used for 'response_predicate' because the exact function object can be hard to match,
        # but we verify it was called with the correct CAN ID and command code.
        mock_can_interface_for_axis.create_response_future.assert_called_once_with(
            can_id, cmd_code, response_predicate=ANY 
        )
        # Assert that the Axis now considers the move complete.
        assert axis_instance.is_move_complete()
        # Assert that the encoder value was read after the move (to update Axis's internal position).
        mock_low_level_api.read_encoder_value_addition.assert_called()

    @pytest.mark.asyncio
    async def test_move_relative_pulses_limit_hit(
        self,
        axis_instance: Axis,
        mock_low_level_api: AsyncMock,
        mock_can_interface_for_axis: MagicMock,
    ):
        # Tests the scenario where a relative move is initiated but is stopped by a limit switch
        # before reaching its target. The motor should report a specific status for this.
        relative_pulses = 2000 # A larger move distance.
        can_id = axis_instance.can_id
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES

        # Mock LowLevelAPI to indicate move starting.
        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )

        # Set up the future for the completion/limit signal.
        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )

        # Start the move task.
        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(relative_pulses, wait=True)
        )
        await asyncio.sleep(0.01) # Allow task to start waiting on the future.

        # Simulate the "limit hit" CAN message arriving.
        # The status byte 'const.POS_RUN_END_LIMIT_STOPPED' indicates a limit stop.
        response_payload_data = [cmd_code, const.POS_RUN_END_LIMIT_STOPPED] 
        response_crc = (can_id + sum(response_payload_data)) & 0xFF 
        completion_msg_data = bytes(response_payload_data + [response_crc])
        
        simulated_can_msg_limit = CanMessage(
            arbitration_id=can_id,
            data=completion_msg_data,
            dlc=len(completion_msg_data)
        )
        # Set the future's result with the message indicating a limit hit.
        current_test_completion_future.set_result(simulated_can_msg_limit)


        # Assert that awaiting the 'move_task' (which internally awaits _execute_move)
        # raises a 'LimitError' because the simulated CAN message indicated a limit stop.
        with pytest.raises(LimitError):
            await move_task

        # Assert that the Axis considers the move sequence complete (even though it ended in an error).
        assert axis_instance.is_move_complete()
        # Verify 'create_response_future' was called, expecting the predicate.
        mock_can_interface_for_axis.create_response_future.assert_called_once_with(
            can_id, cmd_code, response_predicate=ANY
        )


    @pytest.mark.asyncio
    async def test_move_timeout(
        self,
        axis_instance: Axis,
        mock_low_level_api: AsyncMock,
        mock_can_interface_for_axis: MagicMock,
    ): 
        # Tests the scenario where a move times out because no completion signal
        # (neither success nor error like limit hit) is received from the motor.
        relative_pulses = 500
        can_id = axis_instance.can_id 
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES 

        # Mock LowLevelAPI to indicate move starting.
        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )

        # Set up the future for completion signal, but this future will NOT be given a result in this test,
        # which should lead to a timeout within 'Axis._execute_move'.
        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )
        
        # Patch the timeout constant used in 'Axis._execute_move' for this specific test.
        # This temporarily changes the value of 'const.CAN_TIMEOUT_SECONDS' (which influences
        # the dynamic timeout calculation or serves as a base for it) to a very short value (0.001s)
        # to make the timeout occur quickly during the test.
        with patch( 
            "mks_servo_can.axis.const.CAN_TIMEOUT_SECONDS", 0.001 # New, very short timeout value.
        ):
            # Assert that calling 'move_relative_pulses' (with wait=True)
            # raises a 'CommunicationError' due to the timeout.
            # The error message should indicate a timeout waiting for the move.
            with pytest.raises(
                CommunicationError, match="Timeout waiting for move" 
            ):
                await axis_instance.move_relative_pulses(
                    relative_pulses, wait=True
                )

        # Assert that the Axis considers the move sequence complete (due to the timeout/error).
        assert axis_instance.is_move_complete()
        # Assert that the internal future representing the move ('_active_move_future') exists.
        assert axis_instance._active_move_future is not None 
        # Assert that this internal future is marked as done (it should have an exception set).
        assert axis_instance._active_move_future.done()
        
        # Further assert that awaiting this internal future directly also raises CommunicationError.
        # This verifies that the future was correctly set with the CommunicationError exception by _execute_move.
        with pytest.raises(CommunicationError): 
            await axis_instance._active_move_future

        # Verify 'create_response_future' was called, expecting the predicate.
        mock_can_interface_for_axis.create_response_future.assert_called_once_with(
            can_id, cmd_code, response_predicate=ANY
        )


    @pytest.mark.asyncio
    async def test_move_relative_pulses_calls_execute_move(
        self, axis_instance: Axis, mock_low_level_api: AsyncMock # mock_low_level_api not strictly needed for this test but fine as fixture
    ):
        # This test specifically verifies that the public 'move_relative_pulses' method
        # correctly calls the internal '_execute_move' method with the expected parameters.
        # It does this by patching '_execute_move' itself with an AsyncMock.
        with patch.object(
            axis_instance, "_execute_move", new_callable=AsyncMock 
        ) as mock_exec_move:
            # Calls 'move_relative_pulses' with 'wait=False'.
            # If 'wait=True', the outer call would await the (now mocked) '_execute_move'.
            # With 'wait=False', the call to '_execute_move' is made, and 'move_relative_pulses' returns quickly
            # without awaiting the mock_exec_move's completion (which doesn't matter here as we only check the call).
            await axis_instance.move_relative_pulses(
                100, speed_param=100, accel_param=50, wait=False
            )
            # Assert that the mocked '_execute_move' was called exactly once.
            mock_exec_move.assert_called_once()
            
            # Retrieve the arguments with which '_execute_move' was called.
            # 'call_args' is a tuple (positional_args, keyword_args).
            args, kwargs = mock_exec_move.call_args
            
            # Assert that specific arguments were passed correctly.
            # 'command_const' is passed as the second positional argument (index 1) to _execute_move.
            # args[0] would be the 'move_command_func' (a lambda).
            assert args[1] == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES
            # 'pulses_to_move' and 'speed_param_for_calc' are passed as keyword arguments
            # to _execute_move by the move_relative_pulses method.
            assert kwargs.get("pulses_to_move_for_timeout") == 100 # abs(relative_pulses)
            assert kwargs.get("speed_param_for_calc") == 100 # speed_param

    @pytest.mark.asyncio
    async def test_move_to_position_abs_axis_success(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        """Tests that move_to_position_abs_axis calls the correct low-level command (0xF5)."""
        # Patch the internal helper to isolate this method's logic
        with patch.object(axis_instance, "_execute_move", new_callable=AsyncMock) as mock_exec_move:
            target_steps = 10000
            speed = 500
            await axis_instance.move_to_position_abs_axis(target_steps, speed, wait=False)
            
            # Verify _execute_move was called with the right command constant and parameters
            mock_exec_move.assert_called_once_with(
                ANY,  # the lambda function
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
                pulses_to_move_for_timeout=target_steps,
                speed_param_for_calc=speed
            )
            
            # Verify the lambda calls the correct low-level API method
            cmd_func_lambda = mock_exec_move.call_args[0][0]
            await cmd_func_lambda() # Execute the captured lambda
            mock_low_level_api.run_position_mode_absolute_axis.assert_called_with(
                axis_instance.can_id, speed, axis_instance.default_accel_param, target_steps
            )

    @pytest.mark.asyncio
    async def test_move_relative_axis_success(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        """Tests that move_relative_axis calls the correct low-level command (0xF4)."""
        with patch.object(axis_instance, "_execute_move", new_callable=AsyncMock) as mock_exec_move:
            relative_steps = -5000
            speed = 600
            await axis_instance.move_relative_axis(relative_steps, speed, wait=False)
            
            mock_exec_move.assert_called_once_with(
                ANY,
                const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS,
                pulses_to_move_for_timeout=relative_steps,
                speed_param_for_calc=speed
            )
            
            cmd_func_lambda = mock_exec_move.call_args[0][0]
            await cmd_func_lambda()
            mock_low_level_api.run_position_mode_relative_axis.assert_called_with(
                axis_instance.can_id, speed, axis_instance.default_accel_param, relative_steps
            )
            
@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisPing: 
    # This test class is dedicated to testing the 'ping' functionality of the Axis.
    # Ping is typically used to check if a motor is responsive using a lightweight command.

    @pytest.mark.asyncio
    async def test_ping_success(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests a successful ping operation.
        """Test that ping returns True when the low-level call succeeds."""
        # Configure the mock LowLevelAPI's 'read_en_pin_status' (used by Axis.ping)
        # to simulate a successful read. The actual boolean value returned doesn't matter
        # for ping success, only that the command completes without error.
        mock_low_level_api.read_en_pin_status.return_value = True 

        # Call the ping method on the Axis instance.
        result = await axis_instance.ping()

        # Assert that ping returns True on success.
        assert result is True
        # Assert that the underlying LowLevelAPI method ('read_en_pin_status')
        # was called once with the correct CAN ID.
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    @pytest.mark.asyncio
    async def test_ping_failure_communication_error(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests that ping returns False if the underlying low-level call
        # (e.g., read_en_pin_status) raises a CommunicationError (like a timeout).
        """Test that ping returns False when a CommunicationError occurs."""
        # Configure the mock to simulate a communication error by raising CommunicationError.
        mock_low_level_api.read_en_pin_status.side_effect = CommunicationError("Simulated communication failure")

        # Call ping.
        result = await axis_instance.ping()

        # Assert that ping returns False because the underlying call failed.
        assert result is False
        # Verify the LowLevelAPI method was still called (even though it raised an error).
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    @pytest.mark.asyncio
    async def test_ping_failure_other_mks_servo_error(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests that ping returns False if the underlying low-level call raises
        # any other MKSServoError (e.g., MotorError).
        """Test that ping returns False when another MKSServoError occurs."""
        # Configure the mock to simulate a different MKS Servo error.
        mock_low_level_api.read_en_pin_status.side_effect = MotorError("Simulated MKS error during ping")

        result = await axis_instance.ping()

        assert result is False
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    @pytest.mark.asyncio
    async def test_ping_failure_unexpected_exception(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests that ping returns False if the underlying low-level call raises
        # an unexpected non-MKSServoError Exception (e.g., a standard Python Exception).
        """Test that ping returns False when an unexpected Exception occurs."""
        # Configure the mock to simulate an unexpected error.
        mock_low_level_api.read_en_pin_status.side_effect = Exception("Unexpected problem during ping")

        result = await axis_instance.ping()

        assert result is False
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    @pytest.mark.asyncio
    async def test_ping_with_timeout_param(self, axis_instance: Axis, mock_low_level_api: AsyncMock):
        # Tests that the ping method can be called with an explicit 'timeout' parameter.
        # The Axis.ping() method uses asyncio.wait_for with this timeout to cap the overall ping operation.
        # The underlying low-level CAN read itself will use its own default timeout.
        """
        Test that ping can be called with a timeout parameter.
        This timeout is for the asyncio.wait_for() call within Axis.ping().
        """
        mock_low_level_api.read_en_pin_status.return_value = True # Simulate success for the underlying call.

        # Call ping with an explicit timeout for the ping operation.
        result = await axis_instance.ping(timeout=0.5) 

        assert result is True # Asserts successful ping.
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)
        # Asserts the underlying LowLevelAPI call was made.
        # If Axis.ping were modified to pass its timeout parameter down to the
        # low_level_api.read_en_pin_status call, that interaction would be asserted here.
        # Currently, the timeout in ping() is for the wrapper around the internal _do_ping call.
        