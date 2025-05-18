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

from mks_servo_can import constants as const
# Imports the 'constants' module (aliased as 'const') from the 'mks_servo_can' library.
# This provides access to predefined constants used by the Axis class.

from mks_servo_can.axis import Axis
# Imports the 'Axis' class, which is the System Under Test (SUT) for this file.

from mks_servo_can.can_interface import CANInterface
# Imports 'CANInterface', which is a dependency of 'Axis' and will be mocked.

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
    # If 'python-can' is not available, a dummy 'CanMessage' class is defined.
    # This ensures tests can run in environments without the full 'python-can' stack,
    # as the actual CAN communication is mocked in these unit tests.
    class CanMessage:
        # The dummy CanMessage class definition.
        def __init__(
            self, arbitration_id=0, data=None, dlc=0, is_extended_id=False
        ):
            self.arbitration_id = arbitration_id
            # Stores the CAN message ID.
            self.data = data if data is not None else b""
            # Stores the data payload, defaulting to empty bytes.
            self.dlc = dlc if data is None else len(self.data)
            # Data Length Code, calculated if not provided.
            self.is_extended_id = is_extended_id
            # Flag for extended CAN ID.
            self.timestamp = 0.0
            # Timestamp of the message.
            self.channel = None
            # CAN channel.
            self.is_rx = True
            # Flag indicating if the message was received.
            self.is_error_frame = False
            # Flag for CAN error frame.
            self.is_remote_frame = False
            # Flag for Remote Transmission Request frame.

@pytest.fixture
# Defines a pytest fixture named 'mock_can_interface_for_axis'.
# This fixture will provide a mocked 'CANInterface' object for tests that require it.
def mock_can_interface_for_axis():
    # Creates an AsyncMock instance that specifically mimics the CANInterface.
    mock = AsyncMock(spec=CANInterface)
    # Mocks the 'send_and_wait_for_response' async method.
    mock.send_and_wait_for_response = AsyncMock()
    # Mocks the 'send_message' async method.
    mock.send_message = AsyncMock()
    # CANInterface.create_response_future is a SYNCHRONOUS method returning a future.
    # So, its mock should be MagicMock, not AsyncMock.
    # This comment clarifies why 'MagicMock' is used for 'create_response_future'.
    # 'create_response_future' is a synchronous method that sets up an asyncio.Future
    # for receiving responses, so it needs a synchronous mock.
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
    # to be called during Axis operations. This simplifies test setup.
    mock.read_encoder_value_addition = AsyncMock(return_value=0)
    # Mocks reading encoder value, returning 0 by default.
    mock.read_en_pin_status = AsyncMock(return_value=False)
    # Mocks reading enable pin status, returning False (disabled) by default.
    mock.query_motor_status = AsyncMock(return_value=const.MOTOR_STATUS_STOPPED)
    # Mocks querying motor status, returning 'MOTOR_STATUS_STOPPED' by default.
    mock.run_position_mode_relative_pulses = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    # Mocks initiating a relative pulse move, returning 'POS_RUN_STARTING'.
    mock.run_position_mode_absolute_pulses = AsyncMock(
        return_value=const.POS_RUN_STARTING
    )
    # Mocks initiating an absolute pulse move, returning 'POS_RUN_STARTING'.
    mock.enable_motor = AsyncMock()
    # Mocks enabling the motor (no specific return value needed beyond not erroring).
    mock.calibrate_encoder = AsyncMock()
    # Mocks calibrating the encoder.
    mock.go_home = AsyncMock(return_value=const.HOME_START)
    # Mocks initiating a homing sequence, returning 'HOME_START'.
    return mock
    # Returns the configured mock LowLevelAPI.

@pytest.fixture
# Defines a pytest fixture named 'axis_instance'.
# This fixture provides a fully initialized 'Axis' object with its dependencies (LowLevelAPI) mocked.
# It takes the previously defined mock fixtures as arguments.
def axis_instance(
    mock_can_interface_for_axis: MagicMock, mock_low_level_api: AsyncMock
):  # Type hint for mock_can_interface
    # The 'patch' context manager is used here to replace the actual 'LowLevelAPI'
    # class with 'mock_low_level_api' specifically when an 'Axis' is instantiated.
    # This ensures that the Axis object under test uses the mock, not the real LowLevelAPI.
    with patch(
        "mks_servo_can.axis.LowLevelAPI", # The path to the class to be patched.
        return_value=mock_low_level_api, # When 'LowLevelAPI' is called, return this mock instance.
    ):
        # Creates default RotaryKinematics for the test Axis instance.
        kin = RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        )
        # Instantiates the 'Axis' class (the SUT).
        axis = Axis(
            can_interface_manager=mock_can_interface_for_axis,  # Pass the MagicMock here
            # Uses the mocked CANInterface.
            motor_can_id=1, # Assigns CAN ID 1 to this test axis.
            name="TestAxis", # Assigns a name.
            kinematics=kin, # Uses the default kinematics.
        )
        # Manually sets the '_low_level_api' attribute of the created Axis instance
        # to the 'mock_low_level_api'. While patch should handle the instantiation,
        # this assignment ensures direct access for assertions if needed and overrides
        # any potential issues with the patch not taking effect as expected inside the constructor.
        axis._low_level_api = mock_low_level_api
        return axis
        # Returns the configured Axis instance.

# ... (TestAxisBasicOps and TestAxisInitialization should remain the same as they passed) ...
# This comment indicates that some test classes that were previously passing are not shown here for brevity,
# but their structure and purpose are assumed to be understood.

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisBasicOps:
    # This test class groups tests for basic operations and initialization of the Axis class.

    async def test_axis_init(self, mock_can_interface_for_axis):
        # Tests the successful initialization of an Axis object.
        # It verifies that attributes are set correctly based on constructor arguments.
        kin = RotaryKinematics(steps_per_revolution=1000)
        # Creates a specific kinematics object for this test.
        axis = Axis(mock_can_interface_for_axis, 1, "MyAxis", kinematics=kin)
        # Instantiates the Axis.
        assert axis.name == "MyAxis"
        # Verifies the name.
        assert axis.can_id == 1
        # Verifies the CAN ID.
        assert axis.kinematics == kin
        # Verifies the kinematics object.
        assert axis._low_level_api is not None
        # Asserts that the internal LowLevelAPI instance is created (even if it's the real one, not patched here).
        assert axis.is_enabled() is False
        # Asserts the default enabled state is False.
        assert axis.is_homed() is False
        # Asserts the default homed state is False.
        assert axis.is_calibrated() is False
        # Asserts the default calibrated state is False.

    async def test_axis_init_default_kinematics(
        self, mock_can_interface_for_axis
    ):
        # Tests that if no kinematics object is provided to the Axis constructor,
        # it defaults to RotaryKinematics with the standard encoder pulse count.
        axis = Axis(mock_can_interface_for_axis, 1, "DefaultKinAxis")
        # Instantiates Axis without providing a kinematics object.
        assert isinstance(axis.kinematics, RotaryKinematics)
        # Verifies the type of the default kinematics.
        assert (
            axis.kinematics.steps_per_revolution
            == const.ENCODER_PULSES_PER_REVOLUTION
        )
        # Verifies the default steps_per_revolution.

    async def test_axis_init_invalid_can_id(self, mock_can_interface_for_axis):
        # Tests that the Axis constructor raises a 'ParameterError' if an invalid CAN ID is provided.
        # Valid CAN IDs are typically 1-2047 (0x7FF).
        with pytest.raises(ParameterError):
            # Asserts that creating an Axis with CAN ID 0 raises ParameterError.
            Axis(mock_can_interface_for_axis, 0, "Axis0")
        with pytest.raises(ParameterError):
            # Asserts that creating an Axis with CAN ID 0x800 (2048) raises ParameterError.
            Axis(mock_can_interface_for_axis, 0x800, "AxisHigh")

    async def test_enable_motor(self, axis_instance, mock_low_level_api):
        # Tests the 'enable_motor' method of the Axis.
        # It uses the 'axis_instance' (which has a mocked LowLevelAPI) and 'mock_low_level_api' fixtures.
        await axis_instance.enable_motor()
        # Calls the method to enable the motor.
        mock_low_level_api.enable_motor.assert_called_once_with(
            axis_instance.can_id, True
        )
        # Asserts that the LowLevelAPI's 'enable_motor' method was called once
        # with the correct CAN ID and 'True' (to enable).
        assert axis_instance._is_enabled is True
        # Asserts that the Axis's internal enabled flag is set to True.
        mock_low_level_api.enable_motor.reset_mock()
        # Resets the mock to clear the call count for the next assertion.
        await axis_instance.enable_motor()
        # Calls 'enable_motor' again.
        mock_low_level_api.enable_motor.assert_not_called()
        # Asserts that 'enable_motor' on the LowLevelAPI was NOT called again,
        # because the Axis should know it's already enabled and optimize the call.

    async def test_disable_motor(self, axis_instance, mock_low_level_api):
        # Tests the 'disable_motor' method, similar in structure to 'test_enable_motor'.
        axis_instance._is_enabled = True # Manually set the state to enabled for the test.
        await axis_instance.disable_motor()
        # Calls the method to disable the motor.
        mock_low_level_api.enable_motor.assert_called_once_with(
            axis_instance.can_id, False # Note: LowLevelAPI uses 'enable_motor(..., False)' for disabling.
        )
        # Asserts LowLevelAPI's 'enable_motor' was called with False.
        assert axis_instance._is_enabled is False
        # Asserts the internal flag is False.
        mock_low_level_api.enable_motor.reset_mock()
        # Resets the mock.
        await axis_instance.disable_motor()
        # Calls 'disable_motor' again.
        mock_low_level_api.enable_motor.assert_not_called()
        # Asserts LowLevelAPI was not called again due to already being disabled.

    async def test_get_current_position_user(
        self, axis_instance, mock_low_level_api
    ):
        # Tests the 'get_current_position_user' method, which should read steps from
        # the LowLevelAPI and convert them to user units using kinematics.
        mock_steps = 8192 # 16384 / 2, so half a revolution.
        # Sets the mock LowLevelAPI's 'read_encoder_value_addition' to return these steps.
        mock_low_level_api.read_encoder_value_addition.return_value = mock_steps
        # Calls the method to get position in user units. The axis_instance fixture by default
        # uses RotaryKinematics with ENCODER_PULSES_PER_REVOLUTION (16384).
        # So, 8192 steps should correspond to 180 degrees.
        axis_instance.kinematics.steps_to_user(mock_steps) # This line seems to be just calling the kinematics, not used for assertion directly.
        user_pos = await axis_instance.get_current_position_user()
        # Asserts the converted user position is approximately 180.0.
        assert user_pos == pytest.approx(180.0)
        mock_low_level_api.read_encoder_value_addition.assert_called_once_with(
            axis_instance.can_id
        )
        # Asserts the LowLevelAPI was called to read steps.
        assert axis_instance._current_position_steps == mock_steps
        # Asserts the Axis's internal cache for steps is updated.

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisInitialization:
    # This test class focuses on the 'initialize' method of the Axis class.

    async def test_initialize_success_no_actions(
        self, axis_instance, mock_low_level_api
    ):
        # Tests a successful initialization sequence where no calibration or homing is requested.
        # It should read initial status (position, enable state).
        mock_low_level_api.read_encoder_value_addition.return_value = 100
        # Mocks initial position.
        mock_low_level_api.read_en_pin_status.return_value = True
        # Mocks initial enable state (True = enabled).
        await axis_instance.initialize(calibrate=False, home=False)
        # Calls initialize without requesting calibration or homing.
        mock_low_level_api.read_encoder_value_addition.assert_called_once()
        # Verifies position was read.
        mock_low_level_api.read_en_pin_status.assert_called_once()
        # Verifies enable status was read.
        mock_low_level_api.calibrate_encoder.assert_not_called()
        # Verifies calibration was not called.
        mock_low_level_api.go_home.assert_not_called()
        # Verifies homing was not called.
        assert axis_instance._is_enabled is True
        # Verifies the internal enabled flag is updated.
        assert axis_instance._current_position_steps == 100
        # Verifies the internal position steps cache is updated.

    async def test_initialize_with_calibrate_and_home(
        self, axis_instance, mock_low_level_api
    ):
        # Tests initialization with both calibration and homing requested.
        mock_low_level_api.read_encoder_value_addition.return_value = 0
        mock_low_level_api.read_en_pin_status.return_value = False # Assume starts disabled
        mock_low_level_api.calibrate_encoder.return_value = None # Mock successful calibration
        # The 'home_axis' method of the Axis class itself is complex and involves further calls.
        # Here, 'home_axis' is directly mocked on the 'axis_instance' to isolate the
        # 'initialize' method's logic for calling it.
        with patch.object(
            axis_instance, "home_axis", new_callable=AsyncMock
        ) as mock_axis_home:
            # Calls initialize requesting calibration and homing.
            await axis_instance.initialize(calibrate=True, home=True)
            # Asserts that the (mocked) 'home_axis' method of the axis was called.
            mock_axis_home.assert_called_once()

        # Verifies that LowLevelAPI's calibrate_encoder was called.
        mock_low_level_api.calibrate_encoder.assert_called_once()
        # Verifies the internal calibration flag is set.
        assert axis_instance._is_calibrated is True

    async def test_initialize_calibrate_fails_no_home(
        self, axis_instance, mock_low_level_api
    ):
        # Tests initialization where calibration is requested but fails.
        # Homing should not be attempted if calibration fails.
        mock_low_level_api.read_encoder_value_addition.return_value = 0
        mock_low_level_api.read_en_pin_status.return_value = False
        mock_low_level_api.calibrate_encoder.side_effect = CalibrationError(
            "Simulated calib fail"
        )
        # Mocks 'calibrate_encoder' to raise a CalibrationError.

        with pytest.raises(CalibrationError, match="Simulated calib fail"):
            # Asserts that the 'initialize' call re-raises the CalibrationError.
            await axis_instance.initialize(calibrate=True, home=True)

        mock_low_level_api.calibrate_encoder.assert_called_once()
        # Verifies calibration was attempted.
        mock_low_level_api.go_home.assert_not_called()
        # Verifies homing (via LowLevelAPI) was NOT called.
        # Note: If initialize calls axis_instance.home_axis, then that mock interaction should be checked too.
        # Here, the test assumes that if calibrate_encoder raises, home_axis won't be called.
        assert axis_instance._is_calibrated is False
        # Verifies calibration flag is False.
        assert axis_instance._is_homed is False
        # Verifies homed flag is False.

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisMovement:
    # This test class focuses on the movement methods of the Axis class.

    async def test_move_relative_pulses_success(
        self,
        axis_instance: Axis,
        mock_low_level_api: AsyncMock,
        mock_can_interface_for_axis: MagicMock,
    ):  # Corrected mock type
        # Tests a successful relative move in pulses.
        # This test simulates the entire flow:
        # 1. Axis calls LowLevelAPI to start the move.
        # 2. LowLevelAPI returns "starting".
        # 3. Axis creates a future to wait for the "move complete" signal.
        # 4. Test simulates the "move complete" CAN message arriving.
        # 5. Asserts the move completes successfully and state is updated.
        relative_pulses = 1000
        # Number of pulses for the relative move.
        speed_param = 500
        # Speed parameter for the move.
        accel_param = 100
        # Acceleration parameter for the move.
        can_id = axis_instance.can_id
        # CAN ID of the axis.
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES
        # Command code for relative pulse move.

        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )
        # Mocks the LowLevelAPI to indicate the move is starting.

        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        # Creates an asyncio Future. This future will be used by the mock_can_interface
        # to simulate the arrival of the move completion message.
        # Set the return_value of the MagicMock
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )
        # Mocks 'create_response_future' on the CANInterface to return our test-controlled future.
        # The Axis's '_execute_move' method will use this future to wait for the completion signal.

        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(
                relative_pulses, speed_param, accel_param, wait=True
            )
        )
        # Creates an asyncio task to run the 'move_relative_pulses' method.
        # 'wait=True' means the Axis method itself will internally await completion.

        await asyncio.sleep(0.01) # Allow the move_task to start and reach the point where it awaits the future.
        assert (
            not current_test_completion_future.done()
        ), "Completion future should be awaited by _execute_move"
        # Asserts that the future (which signals move completion) is not yet done,
        # meaning '_execute_move' is correctly waiting on it.

        # Simulate the "move complete" CAN message arriving.
        response_payload_data = [cmd_code, const.POS_RUN_COMPLETE] # Echoed command, "complete" status.
        response_crc = (can_id + sum(response_payload_data)) & 0xFF # Simplified CRC for test.
        completion_msg_data = bytes(response_payload_data + [response_crc])
        # Constructs the data for the simulated CAN message.

        current_test_completion_future.set_result(
            CanMessage( # This is the CanMessage object the Axis._execute_move will receive.
                arbitration_id=can_id,
                data=completion_msg_data,
                dlc=len(completion_msg_data),
            )
        )
        # Sets the result of the future, simulating the arrival of the CAN message.
        # This will unblock the '_execute_move' method in the Axis.

        await move_task # Wait for the 'move_relative_pulses' task to finish.
        # This should complete now that the future has a result.

        mock_low_level_api.run_position_mode_relative_pulses.assert_called_once_with(
            can_id, True, speed_param, accel_param, abs(relative_pulses)
            # Arguments: can_id, direction_ccw (True for positive pulses), speed, accel, absolute pulses.
        )
        # Asserts the LowLevelAPI was called correctly to initiate the move.
        mock_can_interface_for_axis.create_response_future.assert_called_once_with(
            can_id, cmd_code
        )
        # Asserts that 'create_response_future' was called to set up the wait for completion.
        assert axis_instance.is_move_complete()
        # Asserts the Axis now considers the move complete.
        mock_low_level_api.read_encoder_value_addition.assert_called()
        # Asserts that the encoder value was read after the move (to update internal position).

    async def test_move_relative_pulses_limit_hit(
        self,
        axis_instance: Axis,
        mock_low_level_api: AsyncMock,
        mock_can_interface_for_axis: MagicMock,
    ):  # Corrected mock type
        # Tests the scenario where a move is stopped by a limit switch.
        # The structure is similar to the success test, but the simulated completion
        # message will indicate a limit stop.
        relative_pulses = 2000
        can_id = axis_instance.can_id
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES

        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )
        # Mocks move starting.

        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )
        # Sets up the future for completion signal.

        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(relative_pulses, wait=True)
        )
        # Starts the move task.
        await asyncio.sleep(0.01) # Allow task to start waiting.

        # Simulate the "limit hit" CAN message arriving.
        response_payload_data = [cmd_code, const.POS_RUN_END_LIMIT_STOPPED] # Status indicates limit stop.
        response_crc = (can_id + sum(response_payload_data)) & 0xFF # Simplified CRC.
        completion_msg_data = bytes(response_payload_data + [response_crc])
        current_test_completion_future.set_result(
            CanMessage(
                arbitration_id=can_id,
                data=completion_msg_data,
                dlc=len(completion_msg_data),
            )
        )
        # Sets the future's result to simulate the limit hit message.

        with pytest.raises(LimitError):
            # Asserts that awaiting the 'move_task' (which internally awaits _execute_move)
            # raises a 'LimitError' because the simulated message indicated a limit stop.
            await move_task

        assert axis_instance.is_move_complete()
        # Asserts the Axis considers the move sequence complete (even though it ended in an error).

    async def test_move_timeout(
        self,
        axis_instance: Axis,
        mock_low_level_api: AsyncMock,
        mock_can_interface_for_axis: MagicMock,
    ):  # Corrected mock type
        # Tests the scenario where a move times out waiting for a completion signal.
        relative_pulses = 500

        mock_low_level_api.run_position_mode_relative_pulses.return_value = (
            const.POS_RUN_STARTING
        )
        # Mocks move starting.

        current_test_completion_future = (
            asyncio.get_event_loop().create_future()
        )
        mock_can_interface_for_axis.create_response_future.return_value = (
            current_test_completion_future
        )
        # Sets up the future, but this future will NOT be given a result in this test,
        # leading to a timeout.

        # Patch the timeout constant used in Axis._execute_move for this test
        # This 'patch' temporarily changes the value of 'const.CAN_TIMEOUT_SECONDS'
        # specifically within the 'Axis._execute_move' method for the duration of this test.
        # It's set to a very short value (0.001s) to make the timeout occur quickly.
        with patch(
            "mks_servo_can.axis.const.CAN_TIMEOUT_SECONDS", # Path to the constant to patch.
            0.001, # New, very short timeout value.
        ):
            with pytest.raises(
                CommunicationError, match="Timeout waiting for move" # Check for specific exception message part.
            ):
                # Asserts that calling 'move_relative_pulses' (with wait=True)
                # raises a 'CommunicationError' due to the timeout.
                await axis_instance.move_relative_pulses(
                    relative_pulses, wait=True
                )

        assert axis_instance.is_move_complete()
        # Asserts the move sequence is considered complete (due to timeout/error).
        assert axis_instance._active_move_future.done()
        # Asserts the internal future representing the move is marked as done.
        # Check that the future has the correct exception
        with pytest.raises(CommunicationError):
            # Further asserts that awaiting this internal future directly also raises CommunicationError.
            # This verifies the future was correctly set with the exception.
            await axis_instance._active_move_future

    async def test_move_relative_pulses_calls_execute_move(
        self, axis_instance, mock_low_level_api # mock_low_level_api is not strictly needed here but fine as fixture
    ):
        # This test verifies that 'move_relative_pulses' correctly calls the internal '_execute_move' method.
        # It does this by patching '_execute_move' itself with a mock.
        with patch.object(
            axis_instance, "_execute_move", new_callable=AsyncMock # Patches the method on the 'axis_instance' object.
        ) as mock_exec_move:
            # Calls 'move_relative_pulses' with 'wait=False'.
            # This is important because if 'wait=True', the outer call would await the (now mocked) '_execute_move'.
            # With 'wait=False', the call to '_execute_move' is made, and the parent method returns quickly.
            await axis_instance.move_relative_pulses(
                100, speed_param=100, accel_param=50, wait=False
            )
            mock_exec_move.assert_called_once()
            # Asserts that the mocked '_execute_move' was called exactly once.
            # This confirms the delegation of move logic.
@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestAxisPing: # Or add to an existing test class like TestAxisBasicOps
    # This test class is dedicated to testing the 'ping' functionality of the Axis.
    # Ping is typically used to check if a motor is responsive.

    async def test_ping_success(self, axis_instance, mock_low_level_api):
        # Tests a successful ping operation.
        """Test that ping returns True when the low-level call succeeds."""
        # Configure the mock to simulate a successful read
        # The 'read_en_pin_status' method of LowLevelAPI is often used as a lightweight command for ping.
        # Its actual return value (True/False for enable status) doesn't matter for ping success,
        # only that the command completes without error.
        mock_low_level_api.read_en_pin_status.return_value = True # Actual return value doesn't matter, just no error

        result = await axis_instance.ping()
        # Calls the ping method.

        assert result is True
        # Asserts that ping returns True on success.
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)
        # Asserts that the underlying LowLevelAPI method was called with the correct CAN ID.

    async def test_ping_failure_communication_error(self, axis_instance, mock_low_level_api):
        # Tests that ping returns False if the underlying low-level call raises a CommunicationError (e.g., timeout).
        """Test that ping returns False when a CommunicationError occurs."""
        # Configure the mock to simulate a communication error
        mock_low_level_api.read_en_pin_status.side_effect = CommunicationError("Simulated communication failure")
        # Sets the mock to raise CommunicationError when called.

        result = await axis_instance.ping()
        # Calls ping.

        assert result is False
        # Asserts ping returns False.
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)
        # Verifies the LowLevelAPI method was still called.

    async def test_ping_failure_other_mks_servo_error(self, axis_instance, mock_low_level_api):
        # Tests that ping returns False if the underlying low-level call raises any other MKSServoError.
        """Test that ping returns False when another MKSServoError occurs."""
        # Configure the mock to simulate a different MKS Servo error
        # Using MotorError as an example, though read_en_pin_status might not typically raise this specific one.
        # This tests the general catch-all for MKSServoError in the ping method.
        mock_low_level_api.read_en_pin_status.side_effect = MotorError("Simulated MKS error during ping")

        result = await axis_instance.ping()

        assert result is False
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    async def test_ping_failure_unexpected_exception(self, axis_instance, mock_low_level_api):
        # Tests that ping returns False if the underlying low-level call raises an unexpected non-MKSServoError Exception.
        """Test that ping returns False when an unexpected Exception occurs."""
        # Configure the mock to simulate an unexpected error
        mock_low_level_api.read_en_pin_status.side_effect = Exception("Unexpected problem during ping")

        result = await axis_instance.ping()

        assert result is False
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)

    async def test_ping_with_timeout_param(self, axis_instance, mock_low_level_api):
        # Tests that the ping method can be called with a 'timeout' parameter.
        # While the current Axis.ping implementation might not pass this timeout down to the LowLevelAPI call directly,
        # this test ensures the method signature accepts it without error.
        # If ping were to use this timeout, assertions on the mock call would be more specific.
        """
        Test that ping can be called with a timeout parameter.
        Note: This test assumes the underlying low-level call could theoretically use the timeout.
        The current ping implementation doesn't pass the timeout through, so this test mainly
        checks that the ping method accepts the argument without error.
        """
        mock_low_level_api.read_en_pin_status.return_value = True # Simulate success.

        # Call with a timeout value.
        result = await axis_instance.ping(timeout=0.5) # Call ping with an explicit timeout.

        assert result is True
        # Asserts successful ping.
        mock_low_level_api.read_en_pin_status.assert_called_once_with(axis_instance.can_id)
        # Asserts the underlying call.
        # If you modify ping to pass the timeout to the low_level_api,
        # you would also assert that `read_en_pin_status` was called with that timeout.
        # This comment suggests a future improvement or check if the timeout parameter becomes functional.
        