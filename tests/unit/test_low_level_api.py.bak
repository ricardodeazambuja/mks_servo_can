# These tests are designed to verify the functionality of individual methods within the LowLevelAPI,
# focusing on how it constructs CAN messages, processes responses, and handles errors,
# typically by mocking the CANInterface.

import pytest
# Imports the pytest framework for writing and running tests.

import struct
# Imports the 'struct' module, used for packing and unpacking binary data.
# This is relevant because CAN commands often involve multi-byte data fields
# representing numbers (integers, etc.) that need to be converted to/from byte sequences.

from unittest.mock import AsyncMock, call, ANY # Added ANY
# From the 'unittest.mock' module, 'AsyncMock' is imported to create mock objects
# for asynchronous functions/methods. 'call' is used to make assertions about how
# mocks were called. 'ANY' is a matcher that can be used when asserting calls
# if the exact value of an argument doesn't matter,
# but its presence or type might.

from typing import List
# Imports 'List' from the 'typing' module for type hinting, specifically for lists.

from mks_servo_can import constants as const
# Imports the 'constants' module (aliased as 'const') from the mks_servo_can library.
# This provides access to predefined constants like command codes, status codes, default values, etc.

from mks_servo_can.can_interface import CANInterface
# Imports the 'CANInterface' class, which the LowLevelAPI uses for actual CAN communication.
# In these unit tests, CANInterface will be mocked.

from mks_servo_can.exceptions import CalibrationError
# Imports specific exception types from the library's 'exceptions' module.
# These are used to assert that the LowLevelAPI raises the correct errors under certain conditions.

from mks_servo_can.exceptions import CommandError
# For errors related to command formatting or unexpected responses.

from mks_servo_can.exceptions import CommunicationError
# For general communication issues like timeouts.

from mks_servo_can.exceptions import CRCError
# For CRC validation failures.

from mks_servo_can.exceptions import MotorError
# For errors reported by the motor itself.

from mks_servo_can.exceptions import ParameterError
# For invalid parameters passed to API methods.

from mks_servo_can.low_level_api import LowLevelAPI
# Imports the 'LowLevelAPI' class, which is the System Under Test (SUT) for this file.

from mks_servo_can.crc import calculate_crc
# Imports 'calculate_crc' to generate expected CRC values for asserting sent messages
# and for creating mock responses with correct CRCs.

try:
    # This try-except block attempts to import 'Message' from the 'can' library (python-can).
    # This is the actual CAN message object used when interacting with real hardware.
    from can import Message as CanMessage
except ImportError:
    # If 'python-can' is not installed (e.g., in a minimal testing environment),
    # a dummy 'CanMessage' class is defined. This allows the tests to run without
    # python-can, as the CAN interface is mocked anyway in these unit tests.
    class CanMessage: # type: ignore
        # The dummy CanMessage class with basic attributes to satisfy type hinting and attribute access in tests.
        def __init__(
            self, arbitration_id=0, data=None, is_extended_id=False, dlc=0
        ):
            # 'arbitration_id': The CAN ID of the message.
            self.arbitration_id = arbitration_id
            # 'data': The data payload of the CAN message, defaults to an empty byte string.
            self.data = data if data is not None else b""
            # 'is_extended_id': Boolean indicating if it's an extended (29-bit) CAN ID. MKS servos use standard (11-bit).
            self.is_extended_id = is_extended_id
            # 'dlc': Data Length Code, the number of bytes in the data payload.
            self.dlc = dlc if data is None else len(self.data) # type: ignore
            # 'timestamp': Time the message was received or created.
            self.timestamp = 0.0
            # 'channel': The CAN channel it was received on.
            self.channel = None
            # 'is_rx': Boolean indicating if the message was received (True) or transmitted (False).
            # Dummy defaults to True as tests often mock received messages.
            self.is_rx = True
            # 'is_error_frame': Boolean indicating if it's a CAN error frame.
            self.is_error_frame = False
            # 'is_remote_frame': Boolean indicating if it's a Remote Transmission Request (RTR) frame.
            self.is_remote_frame = False

@pytest.fixture
# Defines a pytest fixture named 'mock_can_interface'.
# Fixtures are reusable setup/teardown functions for tests.
def mock_can_interface():
    # This docstring explains that the fixture creates a mock 'CANInterface'.
    """Fixture to create a mock CANInterface."""
    # Creates an 'AsyncMock' instance that mimics the 'CANInterface' class.
    # 'spec=CANInterface' ensures the mock has the same attributes/methods as the real class.
    mock = AsyncMock(spec=CANInterface)
    # Specifically mocks 'send_and_wait_for_response' as an 'AsyncMock' since it's an async method.
    mock.send_and_wait_for_response = AsyncMock()
    # Mocks 'send_message' as an 'AsyncMock'.
    mock.send_message = AsyncMock()
    # Returns the configured mock object.
    return mock

@pytest.fixture
# Defines a pytest fixture named 'low_level_api'.
def low_level_api(mock_can_interface):
    # This docstring explains that the fixture creates a 'LowLevelAPI' instance
    # initialized with the 'mock_can_interface' fixture.
    """Fixture to create a LowLevelAPI instance with a mock CANInterface."""
    # Instantiates the 'LowLevelAPI' (the SUT) with the mocked CAN interface.
    return LowLevelAPI(mock_can_interface)

def _assert_message_properties(sent_msg_arg, expected_can_id, expected_data_bytes):
    # This is a helper function used within tests to assert common properties of a sent CAN message.
    # It's not a test itself but helps reduce boilerplate in test methods.
    # 'sent_msg_arg': The 'CanMessage' object that was captured from a mock call.
    # 'expected_can_id': The CAN ID the message should have.
    # 'expected_data_bytes': The byte string the message's data payload should match.

    # Asserts that the message's arbitration ID matches the expected CAN ID.
    assert sent_msg_arg.arbitration_id == expected_can_id
    # Asserts that the message's data payload matches the expected byte string.
    assert sent_msg_arg.data == expected_data_bytes
    # Asserts that the message is not using an extended CAN ID (MKS servos use standard 11-bit IDs).
    assert not sent_msg_arg.is_extended_id

def _pack_int24_be(value: int) -> bytes:
    """Packs a signed 24-bit integer into 3 bytes, big-endian."""
    # Ensure the value is within the 24-bit signed range for consistency
    if not (-8388608 <= value <= 8388607):
        raise ValueError(f"Value {value} out of signed 24-bit range.")
    return struct.pack('>i', value)[1:] # Get last 3 bytes of 32-bit big-endian int

@pytest.mark.asyncio
# Decorator to mark this class as containing asynchronous tests that pytest-asyncio should handle.
class TestLowLevelAPIReads:
    # This test class groups tests specifically for the read operations of the LowLevelAPI.

    async def test_read_encoder_value_addition_success(
        self, low_level_api, mock_can_interface
    ):
        # Tests the 'read_encoder_value_addition' method for a successful scenario.
        # It verifies that the method sends the correct CAN command and correctly parses the response.
        can_id = 0x01
        # Defines the target CAN ID for the motor.
        command_code = const.CMD_READ_ENCODER_ADDITION
        # Specifies the command code for reading the encoder's accumulated value.
        encoder_val = 16384
        # The simulated encoder value to be returned by the mock motor.
        
        # MKS uses 6 bytes for 48-bit value, big-endian.
        # Python's struct.pack('>q', ...) packs to 8 bytes, so slice [2:] for 6 bytes (or pad for 48-bit handling)
        # LowLevelAPI's read_encoder_value_addition unpacks >q after prepending 0x0000 or 0xFFFF
        val_bytes_48bit_be = struct.pack(">q", encoder_val)[2:] # Get 6 bytes, big-endian
        
        response_payload_data = [command_code] + list(val_bytes_48bit_be)
        # Constructs the data part of the mock response: echoed command code + 6 bytes of the packed encoder value.
        response_crc = calculate_crc(can_id, response_payload_data)
        # Calculates the CRC for this mock response.
        full_response_data_bytes = bytes(response_payload_data + [response_crc])
        # Creates the complete data payload for the mock CAN response message (including CRC).

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, # The response comes from the motor's CAN ID.
            data=full_response_data_bytes, # The constructed data payload.
            dlc=len(full_response_data_bytes), # DLC is the length of the data payload.
        )
        # Configures the mock 'send_and_wait_for_response' method to return a 'CanMessage'
        # object simulating a successful response from the motor.

        result_val = await low_level_api.read_encoder_value_addition(can_id)
        # Calls the method under test.
        assert result_val == encoder_val
        # Asserts that the parsed result matches the simulated encoder value.

        sent_payload_data = [command_code]
        # The command sent to the motor only contains the command code (no additional data bytes for this specific read command).
        sent_crc = calculate_crc(can_id, sent_payload_data)
        # Calculates the CRC for the message that should have been sent.
        expected_sent_data_bytes = bytes(sent_payload_data + [sent_crc])
        # Constructs the expected data payload of the sent CAN message.

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        # Asserts that the mock 'send_and_wait_for_response' was called exactly once.
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        # Gets the list of all calls made to the mock.
        args, _ = call_args_list[0] # Get args from the first (and only) call
        # Extracts the positional arguments from the first call.

        sent_msg_arg = args[0]
        # The first argument should be the 'CanMessage' object that was sent.
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        # Uses the helper function to verify properties of the sent message.
        assert args[1] == can_id
        # Asserts that the second argument (expected response CAN ID) is correct.
        assert args[2] == command_code
        # Asserts that the third argument (expected response command code) is correct.
        assert args[3] == const.CAN_TIMEOUT_SECONDS
        # Asserts that the fourth argument (timeout) matches the default CAN timeout.

    async def test_read_motor_speed_rpm_success(
        self, low_level_api, mock_can_interface
    ):
        # Tests the 'read_motor_speed_rpm' method for a successful scenario.
        # This test follows a similar pattern to the previous one: set up mock response, call method, verify result and sent message.
        can_id = 0x02
        command_code = const.CMD_READ_MOTOR_SPEED_RPM
        rpm_val = -1200 # Simulating a negative RPM (e.g., reverse direction).
        rpm_bytes = struct.pack(">h", rpm_val) # Speed is a 2-byte signed short (big-endian).

        response_payload_data = [command_code] + list(rpm_bytes)
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )
        result_rpm = await low_level_api.read_motor_speed_rpm(can_id)
        assert result_rpm == rpm_val
        # (Assertions for the sent message would be similar to the previous test if needed for full coverage,
        # but here the focus is on parsing the response for this specific command.)

    async def test_read_command_communication_error(
        self, low_level_api, mock_can_interface
    ):
        # Tests how a read command handles a 'CommunicationError' (e.g., timeout)
        # raised by the underlying CAN interface.
        can_id = 0x01
        mock_can_interface.send_and_wait_for_response.side_effect = (
            CommunicationError("Simulated timeout")
        )
        # Configures the mock to raise a 'CommunicationError' when called.
        with pytest.raises(CommunicationError, match="Simulated timeout"):
            # Asserts that calling the method results in a 'CommunicationError' being raised,
            # and that the error message matches.
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_crc_error(
        self, low_level_api, mock_can_interface
    ):
        # Tests how a read command handles a response with an incorrect CRC.
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00] # Example valid data part
        wrong_crc = 0x73 # An intentionally incorrect CRC.
        full_response_data_bytes = bytes(response_payload_data + [wrong_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )
        # Configures the mock to return a message with this bad CRC.
        with pytest.raises( CRCError, match=f"Invalid CRC in response to command {command_code:02X}"):
            # Asserts that a 'CRCError' is raised with the expected message format.
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_dlc_mismatch(
        self, low_level_api, mock_can_interface
    ):
        # Tests how a read command handles a response with a DLC that doesn't match
        # the expected length for that command (if an 'expected_dlc' is specified internally by the API method).
        # Note: The LowLevelAPI._send_command_and_get_response method has an 'expected_dlc' parameter.
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION # This command expects DLC=8 in LowLevelAPI
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes) - 1, # Simulate a DLC that is one byte too short.
        )
        # The LowLevelAPI methods that call _send_command_and_get_response often pass an `expected_dlc`.
        # For CMD_READ_ENCODER_ADDITION, the LowLevelAPI method `read_encoder_value_addition` specifies `expected_dlc=8`.
        # The mock response has DLC 7 here.
        with pytest.raises( CommandError, match=f"Response DLC mismatch for command {command_code:02X}"):
            # Asserts that a 'CommandError' related to DLC mismatch is raised.
            await low_level_api.read_encoder_value_addition(can_id)

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestLowLevelAPISets:
    # This test class groups tests for the 'set' operations of the LowLevelAPI.

    async def test_set_work_mode_success(
        self, low_level_api, mock_can_interface
    ):
        # Tests the 'set_work_mode' method for a successful scenario.
        # Verifies correct message construction and parsing of a success response.
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE
        mode_to_set = const.MODE_SR_VFOC # Example work mode.

        sent_payload_data = [command_code, mode_to_set] # Command + mode byte.
        sent_crc = calculate_crc(can_id, sent_payload_data)
        expected_sent_data_bytes = bytes(sent_payload_data + [sent_crc])

        response_payload_data = [command_code, const.STATUS_SUCCESS] # Echoed command + success status.
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        await low_level_api.set_work_mode(can_id, mode_to_set)
        # Calls the method under test. No return value is expected on success, only no exception.

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        # Verifies the CAN interface was called.
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]

        sent_msg_arg = args[0]
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        # Verifies the sent message.
        assert args[1] == can_id
        # Verifies expected response CAN ID.
        assert args[2] == command_code
        # Verifies expected echoed command code in response.
        assert args[3] == const.CAN_TIMEOUT_SECONDS
        # Verifies default timeout.

    async def test_set_work_mode_failure_response(
        self, low_level_api, mock_can_interface
    ):
        # Tests 'set_work_mode' when the motor responds with a failure status.
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE
        mode_to_set = const.MODE_CR_OPEN
        status_failure = const.STATUS_FAILURE # Simulate a failure response from the motor.

        response_payload_data = [command_code, status_failure]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        expected_error_msg = (
            f"Set work mode to {mode_to_set} failed for CAN ID {can_id}. Status: {status_failure}"
        )
        # Constructs the expected error message.
        with pytest.raises(MotorError, match=expected_error_msg):
            # Asserts that a 'MotorError' is raised with the correct message
            # when the motor indicates the operation failed.
            await low_level_api.set_work_mode(can_id, mode_to_set)

    async def test_set_work_mode_invalid_param(self, low_level_api):
        # Tests that 'set_work_mode' raises a 'ParameterError' if an invalid work mode value is provided.
        with pytest.raises(ParameterError, match="Invalid work mode: 99"):
            # 99 is not a valid work mode constant.
            await low_level_api.set_work_mode(0x01, 99)

    async def test_calibrate_encoder_failure(
        self, low_level_api, mock_can_interface
    ):
        # Tests the 'calibrate_encoder' method when the motor reports a calibration failure.
        can_id = 0x01
        command_code = const.CMD_CALIBRATE_ENCODER

        response_payload_data = [command_code, const.STATUS_CALIBRATING_FAIL] # Calibration failed status.
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )
        with pytest.raises(CalibrationError, match="Encoder calibration failed."):
            # Asserts that a 'CalibrationError' (a subclass of MotorError) is raised.
            await low_level_api.calibrate_encoder(can_id)

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestLowLevelAPISetSystemParamsExtended:
    # This test class extends testing for various "set system parameter" commands.
    # It uses pytest's parameterization to test multiple scenarios with less boilerplate.

    @pytest.mark.parametrize("direction_code, direction_name", [
        (const.DIR_CW, "CW"), (const.DIR_CCW, "CCW")
    ])
    # Parameterizes the test with two sets of values: Clockwise (0) and Counter-Clockwise (1).
    async def test_set_motor_direction(self, low_level_api, mock_can_interface, direction_code, direction_name):
        # Tests the 'set_motor_direction' method.
        can_id = 0x01
        command_code = const.CMD_SET_MOTOR_DIRECTION

        sent_payload_data = [command_code, direction_code]
        sent_crc = calculate_crc(can_id, sent_payload_data)
        expected_sent_bytes = bytes(sent_payload_data + [sent_crc])

        response_payload = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload + [response_crc_val]), dlc=3
        )

        await low_level_api.set_motor_direction(can_id, direction_code)
        # Calls the method with the parameterized direction code.

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        # Verifies the sent message.
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    async def test_set_motor_direction_invalid_param(self, low_level_api):
        # Tests that an invalid direction code raises a ParameterError.
        with pytest.raises(ParameterError, match="Invalid motor direction code: 2"):
            await low_level_api.set_motor_direction(0x01, 2) # 2 is not a valid direction code.

    @pytest.mark.parametrize("enable_auto_off, enable_code", [(True, 0x01), (False, 0x00)])
    # Parameterizes for enabling (True, 0x01) and disabling (False, 0x00) auto screen off.
    async def test_set_auto_screen_off(self, low_level_api, mock_can_interface, enable_auto_off, enable_code):
        # Tests the 'set_auto_screen_off' method.
        can_id = 0x01
        command_code = const.CMD_SET_AUTO_SCREEN_OFF

        sent_payload_data = [command_code, enable_code]
        sent_crc = calculate_crc(can_id, sent_payload_data)
        expected_sent_bytes = bytes(sent_payload_data + [sent_crc])

        response_payload = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload + [response_crc_val]), dlc=3
        )
        await low_level_api.set_auto_screen_off(can_id, enable_auto_off)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    async def test_set_can_id_success(self, low_level_api, mock_can_interface):
        # Tests the 'set_can_id' method for changing the motor's CAN ID.
        current_can_id = 0x01
        new_can_id = 0x05
        command_code = const.CMD_SET_CAN_ID

        id_bytes = list(struct.pack(">H", new_can_id)) # New CAN ID is a 2-byte short, big-endian.
        sent_payload_data = [command_code] + id_bytes
        sent_crc = calculate_crc(current_can_id, sent_payload_data) # CRC uses the *current* CAN ID.
        expected_sent_bytes = bytes(sent_payload_data + [sent_crc])

        response_payload = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(current_can_id, response_payload) # Response comes from the *current* CAN ID.
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=current_can_id, data=bytes(response_payload + [response_crc_val]), dlc=3
        )

        await low_level_api.set_can_id(current_can_id, new_can_id)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], current_can_id, expected_sent_bytes)
        assert args[1:] == (current_can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    async def test_set_can_id_invalid_new_id(self, low_level_api):
        # Tests that setting an invalid new CAN ID (e.g., > 0x7FF) raises a ParameterError.
        with pytest.raises(ParameterError, match="Invalid new CAN ID: 2048"):
            await low_level_api.set_can_id(0x01, 0x800) # 0x800 (2048) is out of range.

    async def test_set_slave_respond_active_success(self, low_level_api, mock_can_interface):
        # Tests the 'set_slave_respond_active' method, which configures if the motor sends
        # responses and if it actively initiates data transmission.
        can_id = 0x01
        command_code = const.CMD_SET_SLAVE_RESPOND_ACTIVE
        respond_enabled = True # Corresponds to data byte 0x01 for this setting.
        active_enabled = False # Corresponds to data byte 0x00 for this setting.

        data_payload_sent_list = [command_code, 0x01, 0x00] # Command, respond_code, active_code.
        sent_crc = calculate_crc(can_id, data_payload_sent_list)
        expected_sent_bytes = bytes(data_payload_sent_list + [sent_crc])

        response_payload_resp_list = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload_resp_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_resp_list + [response_crc_val]), dlc=3
        )

        await low_level_api.set_slave_respond_active(can_id, respond_enabled, active_enabled)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestLowLevelAPIWriteAndHomeCommands:
    # This class tests commands related to writing to IO ports and homing procedures.

    async def test_write_io_port_success(self, low_level_api, mock_can_interface):
        # Tests the 'write_io_port' command. This command allows setting the state of output pins.
        can_id = 0x01
        command_code = const.CMD_WRITE_IO_PORT
        # Example values for setting OUT1 to 1 and OUT2 to 0, with mask action 1 (set value).
        # The data byte encoding is:
        # Bit 7-6: OUT2 mask (01 = use value)
        # Bit 5-4: OUT1 mask (01 = use value)
        # Bit 3:   OUT2 value (0)
        # Bit 2:   OUT1 value (1)
        # Bit 1-0: Not used for OUT setting.
        # So, (01 << 6) | (01 << 4) | (0 << 3) | (1 << 2) = 0x40 | 0x10 | 0x00 | 0x04 = 0x54.
        expected_data_byte = 0x54

        sent_payload_list = [command_code, expected_data_byte]
        sent_crc = calculate_crc(can_id, sent_payload_list)
        expected_sent_bytes = bytes(sent_payload_list + [sent_crc])

        response_payload_list = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )

        # Call the API method with parameters that should result in the 'expected_data_byte'.
        await low_level_api.write_io_port(can_id, out1_value=1, out2_value=0, out1_mask_action=1, out2_mask_action=1)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    async def test_set_home_parameters_success(self, low_level_api, mock_can_interface):
        # Tests the 'set_home_parameters' command, which configures how the homing process behaves.
        can_id = 0x01
        command_code = const.CMD_SET_HOME_PARAMETERS
        home_trig = 0  # Trigger level (e.g., Low).
        home_dir_val = 1  # Homing direction (e.g., CCW).
        home_speed = 100  # Homing speed parameter.
        end_limit_en = True # Whether end limits are enabled during homing.
        home_mode_val = 0  # Homing mode (e.g., use limit switch).

        speed_bytes = list(struct.pack(">H", home_speed)) # Homing speed is a 2-byte value, big-endian.
        # Construct the data payload according to the MKS manual for this command.
        data_to_send_list = [command_code, home_trig, home_dir_val] + speed_bytes + [0x01 if end_limit_en else 0x00, home_mode_val]
        sent_crc = calculate_crc(can_id, data_to_send_list)
        expected_sent_bytes = bytes(data_to_send_list + [sent_crc])

        response_payload_resp_list = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload_resp_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_resp_list + [response_crc_val]), dlc=3
        )
        await low_level_api.set_home_parameters(can_id, home_trig, home_dir_val, home_speed, end_limit_en, home_mode_val)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    async def test_go_home_starts(self, low_level_api, mock_can_interface):
        # Tests the 'go_home' command, specifically checking if it correctly handles the "homing started" response.
        can_id = 0x01
        command_code = const.CMD_GO_HOME
        sent_payload_list = [command_code] # This command has no additional data bytes.
        sent_crc = calculate_crc(can_id, sent_payload_list)
        expected_sent_bytes = bytes(sent_payload_list + [sent_crc])

        response_payload_list = [command_code, const.HOME_START] # Motor responds that homing has started.
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )
        status = await low_level_api.go_home(can_id)
        # The API method should return the status code from the response.
        assert status == const.HOME_START

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestLowLevelAPIRunCommandsExtended:
    # This class contains more tests for various "run motor" commands, like positional moves.

    def _pack_int24_for_test(self, value: int) -> bytes:
        # Helper function to pack a signed 24-bit integer into a list of 3 bytes (big-endian).
        # This is used because some MKS commands represent positions or relative moves as 24-bit values.
        """Helper to pack signed 24-bit int for test comparison."""
        # Use struct.pack('>i', value) to get a 4-byte big-endian representation, then slice for 3 bytes.
        # This implicitly handles sign extension for 24-bit value to 32-bit.
        return struct.pack('>i', value)[1:]

    @pytest.mark.parametrize("relative_axis_val", [-1000, 0, 1000])
    # Parameterizes the test to run with different relative axis values.
    async def test_run_position_mode_relative_axis_success(self, low_level_api, mock_can_interface, relative_axis_val):
        # Tests the 'run_position_mode_relative_axis' command (0xF4).
        # This command moves the motor by a relative amount specified in "axis units" (often encoder pulses).
        can_id = 0x01
        command_code = const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS
        speed = 500 # Speed parameter (0-3000).
        accel = 100 # Acceleration parameter (0-255).

        speed_bytes = list(struct.pack(">H", speed)) # Speed is 2 bytes, big-endian.
        accel_byte = accel & 0xFF # Acceleration is 1 byte.
        axis_bytes_packed = list(self._pack_int24_for_test(relative_axis_val)) # Relative axis value is 3 bytes (signed 24-bit), big-endian.

        # Construct the full data payload for the sent message.
        sent_data_list = [command_code] + speed_bytes + [accel_byte] + axis_bytes_packed
        sent_crc = calculate_crc(can_id, sent_data_list)
        expected_sent_bytes = bytes(sent_data_list + [sent_crc])

        # Simulate the motor's response: echoed command and "position run starting" status.
        response_payload_list = [command_code, const.POS_RUN_STARTING]
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )

        status = await low_level_api.run_position_mode_relative_axis(can_id, speed, accel, relative_axis_val)
        assert status == const.POS_RUN_STARTING # Verify the returned status.

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)

    @pytest.mark.parametrize("save_action, action_code", [(True, const.SPEED_MODE_PARAM_SAVE), (False, const.SPEED_MODE_PARAM_CLEAN)])
    # Parameterizes for saving (True) and cleaning (False) speed mode parameters.
    async def test_save_or_clean_speed_mode_params(self, low_level_api, mock_can_interface, save_action, action_code):
        # Tests the command 0xFF, which is used to save or clear parameters related to speed mode.
        # This command has a peculiar response: it echoes the 'action_code' (0xC8 or 0xCA) instead of 0xFF.
        can_id = 0x01
        command_code_sent = const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS # This is 0xFF.
        expected_echoed_command_in_response = action_code # The motor echoes 0xC8 or 0xCA.

        sent_data_list = [command_code_sent, action_code] # Sent data: [0xFF, action_code].
        sent_crc = calculate_crc(can_id, sent_data_list)
        expected_sent_bytes = bytes(sent_data_list + [sent_crc])

        # Simulate the motor's response: echoed 'action_code' and success status.
        response_payload_resp_list = [action_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload_resp_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_resp_list + [response_crc_val]), dlc=3
        )

        await low_level_api.save_or_clean_speed_mode_params(can_id, save_action)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1] == can_id # Expected response CAN ID.
        # Verify that the LowLevelAPI correctly expects the 'action_code' to be echoed in the response, not 0xFF.
        assert args[2] == expected_echoed_command_in_response
        assert args[3] == const.CAN_TIMEOUT_SECONDS

@pytest.mark.asyncio
# Marks this class for asynchronous tests.
class TestLowLevelAPIReadSystemParameter:
    # This class tests the 'read_system_parameter' method (command prefix 0x00).
    # This command is used to read various system parameters by sending 0x00 followed by the
    # command code of the parameter to be read.

    async def test_read_system_parameter_success(
        self, low_level_api, mock_can_interface
    ):
        # Tests a successful read of a system parameter.
        can_id = 0x01
        # 'param_to_read_cmd_code' is the command code associated with the parameter we want to read (e.g., work mode 0x82).
        param_to_read_cmd_code = const.CMD_SET_WORK_MODE # Using 0x82 as an example readable parameter.

        # The message sent to the motor uses 0x00 as the main command,
        # and 'param_to_read_cmd_code' as its data byte.
        sent_payload_data_for_00 = [const.CMD_READ_SYSTEM_PARAMETER_PREFIX, param_to_read_cmd_code]
        sent_crc_for_00 = calculate_crc(can_id, sent_payload_data_for_00)
        expected_sent_data_bytes = bytes(sent_payload_data_for_00 + [sent_crc_for_00])

        current_mode_val = const.MODE_SR_CLOSE # The simulated current value of the work mode.
        # The motor's response will echo 'param_to_read_cmd_code' as the first byte of its data,
        # followed by the parameter's value.
        response_payload_data = [param_to_read_cmd_code, current_mode_val]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        echoed_code, param_data = await low_level_api.read_system_parameter(
            can_id, param_to_read_cmd_code
        )
        # The method should return the echoed command code and the parameter data.

        assert echoed_code == param_to_read_cmd_code
        # Verifies the echoed command code.
        assert param_data == bytes([current_mode_val])
        # Verifies the parameter data.

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]

        sent_msg_arg = args[0]
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        # Verifies the properties of the message sent by the LowLevelAPI.
        assert args[1] == can_id # Expected response CAN ID.
        # Verifies that the LowLevelAPI expected the 'param_to_read_cmd_code' to be echoed in the response.
        assert args[2] == param_to_read_cmd_code
        assert args[3] == const.CAN_TIMEOUT_SECONDS # Default timeout.

    async def test_read_system_parameter_not_readable(
        self, low_level_api, mock_can_interface
    ):
        # Tests the scenario where the motor indicates a parameter is not readable
        # by responding with [echoed_param_code, 0xFF, 0xFF].
        can_id = 0x01
        param_to_read_cmd_code = 0xAA # An arbitrary, assumed non-readable parameter code for testing.

        # Simulate the motor's "not readable" response.
        response_payload_data = [param_to_read_cmd_code, 0xFF, 0xFF]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        with pytest.raises( MotorError, match=f"Parameter with code {param_to_read_cmd_code:02X} cannot be read"):
            # Asserts that a 'MotorError' is raised with a message indicating the parameter cannot be read.
            await low_level_api.read_system_parameter(
                can_id, param_to_read_cmd_code
            )
            