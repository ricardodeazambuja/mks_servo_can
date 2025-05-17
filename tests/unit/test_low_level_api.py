# mks_servo_can_project/tests/unit/test_low_level_api.py
import pytest
import asyncio
import struct
from unittest.mock import AsyncMock, MagicMock, patch

from mks_servo_can_library.mks_servo_can.low_level_api import LowLevelAPI
from mks_servo_can_library.mks_servo_can.can_interface import CANInterface
from mks_servo_can_library.mks_servo_can import constants as const
from mks_servo_can_library.mks_servo_can.exceptions import (
    CommandError, CRCError, ParameterError, MotorError, CommunicationError,
    CalibrationError
)

try:
    from can import Message as CanMessage
except ImportError:
    class CanMessage:
        def __init__(self, arbitration_id=0, data=None, is_extended_id=False, dlc=0):
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b''
            self.is_extended_id = is_extended_id
            self.dlc = dlc if data is None else len(self.data)
            self.timestamp = 0.0 # Added for completeness if used by SUT
            self.channel = None
            self.is_rx = True
            self.is_error_frame = False
            self.is_remote_frame = False


@pytest.fixture
def mock_can_interface():
    """Fixture to create a mock CANInterface."""
    mock = AsyncMock(spec=CANInterface)
    mock.send_and_wait_for_response = AsyncMock()
    mock.send_message = AsyncMock()
    return mock

@pytest.fixture
def low_level_api(mock_can_interface):
    """Fixture to create a LowLevelAPI instance with a mock CANInterface."""
    return LowLevelAPI(mock_can_interface)

@pytest.mark.asyncio
class TestLowLevelAPIReads:
    async def test_read_encoder_value_addition_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION 
        encoder_val = 16384
        val_bytes = bytearray(8)
        struct.pack_into('<q', val_bytes, 0, encoder_val) 
        response_payload_data = [command_code] + list(val_bytes[:6])
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes)
        )

        result_val = await low_level_api.read_encoder_value_addition(can_id)
        assert result_val == encoder_val

        expected_sent_payload_data = [command_code]
        expected_sent_crc = (can_id + command_code) & 0xFF
        expected_sent_data_bytes = bytes(expected_sent_payload_data + [expected_sent_crc])

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args = mock_can_interface.send_and_wait_for_response.call_args[0]
        sent_msg = call_args[0] 
        assert sent_msg.arbitration_id == can_id
        assert sent_msg.data == expected_sent_data_bytes
        assert call_args[1] == can_id 
        assert call_args[2] == command_code 


    async def test_read_motor_speed_rpm_success(self, low_level_api, mock_can_interface):
        can_id = 0x02
        command_code = const.CMD_READ_MOTOR_SPEED_RPM 
        rpm_val = -1200 
        rpm_bytes = struct.pack('<h', rpm_val) 

        response_payload_data = [command_code] + list(rpm_bytes)
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )

        result_rpm = await low_level_api.read_motor_speed_rpm(can_id)
        assert result_rpm == rpm_val

    async def test_read_command_communication_error(self, low_level_api, mock_can_interface):
        can_id = 0x01
        mock_can_interface.send_and_wait_for_response.side_effect = CommunicationError("Simulated timeout")

        with pytest.raises(CommunicationError, match="Simulated timeout"):
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_crc_error(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00] 
        wrong_crc = 0x73
        full_response_data_bytes = bytes(response_payload_data + [wrong_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )
        with pytest.raises(CRCError, match=f"Invalid CRC in response to command {command_code:02X}"):
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_dlc_mismatch(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION 
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes) - 1 
        )
        with pytest.raises(CommandError, match=f"Response DLC mismatch for command {command_code:02X}"):
            await low_level_api.read_encoder_value_addition(can_id)


@pytest.mark.asyncio
class TestLowLevelAPISets:
    async def test_set_work_mode_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE 
        mode_to_set = const.MODE_SR_VFOC 

        response_payload_data = [command_code, const.STATUS_SUCCESS]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )

        await low_level_api.set_work_mode(can_id, mode_to_set)

        expected_sent_payload_data = [command_code, mode_to_set]
        expected_sent_crc = (can_id + sum(expected_sent_payload_data)) & 0xFF
        expected_sent_data_bytes = bytes(expected_sent_payload_data + [expected_sent_crc])

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args = mock_can_interface.send_and_wait_for_response.call_args[0]
        sent_msg = call_args[0]
        assert sent_msg.arbitration_id == can_id
        assert sent_msg.data == expected_sent_data_bytes
        assert call_args[2] == command_code 

    async def test_set_work_mode_failure_response(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE
        mode_to_set = const.MODE_CR_OPEN # This is 0
        status_failure = const.STATUS_FAILURE # This is 0

        response_payload_data = [command_code, status_failure] 
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )
        
        # Corrected expected error message format to match MotorError.__str__
        # The message from LowLevelAPI.set_work_mode is:
        # f"Set work mode to {mode} failed for CAN ID {can_id}. Status: {response.data[1]}"
        # The MotorError then appends " - CAN ID: {self.can_id} - Error Code: {self.error_code}"
        expected_error_msg_regex = (
            f"Set work mode to {mode_to_set} failed for CAN ID {can_id}\\. "
            f"Status: {status_failure} - CAN ID: {can_id} - Error Code: {status_failure}"
        )
        # Escape the '.' for regex. For this specific string, it's simple enough.
        # If messages become more complex, more robust regex construction might be needed.

        with pytest.raises(MotorError, match=expected_error_msg_regex):
            await low_level_api.set_work_mode(can_id, mode_to_set)

    async def test_set_work_mode_invalid_param(self, low_level_api):
        with pytest.raises(ParameterError, match="Invalid work mode: 99"):
            await low_level_api.set_work_mode(0x01, 99) 

    async def test_calibrate_encoder_failure(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_CALIBRATE_ENCODER
        
        response_payload_data = [command_code, const.STATUS_CALIBRATING_FAIL] 
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )
        with pytest.raises(CalibrationError, match="Encoder calibration failed."):
            await low_level_api.calibrate_encoder(can_id)

@pytest.mark.asyncio
class TestLowLevelAPIReadSystemParameter:
    async def test_read_system_parameter_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        param_to_read_cmd_code = const.CMD_SET_WORK_MODE 
        
        current_mode_val = const.MODE_SR_CLOSE 
        response_payload_data = [param_to_read_cmd_code, current_mode_val]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )

        echoed_code, param_data = await low_level_api.read_system_parameter(can_id, param_to_read_cmd_code)

        assert echoed_code == param_to_read_cmd_code
        assert param_data == bytes([current_mode_val])

        expected_sent_payload_data = [const.CMD_READ_SYSTEM_PARAMETER_PREFIX, param_to_read_cmd_code]
        expected_sent_crc = (can_id + sum(expected_sent_payload_data)) & 0xFF
        expected_sent_data_bytes = bytes(expected_sent_payload_data + [expected_sent_crc])
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args = mock_can_interface.send_and_wait_for_response.call_args[0]
        sent_msg = call_args[0]
        assert sent_msg.arbitration_id == can_id
        assert sent_msg.data == expected_sent_data_bytes
        assert call_args[1] == can_id 
        assert call_args[2] == param_to_read_cmd_code 

    async def test_read_system_parameter_not_readable(self, low_level_api, mock_can_interface):
        can_id = 0x01
        param_to_read_cmd_code = 0xAA 

        response_payload_data = [param_to_read_cmd_code, 0xFF, 0xFF]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=full_response_data_bytes, dlc=len(full_response_data_bytes)
        )

        with pytest.raises(MotorError, match=f"Parameter with code {param_to_read_cmd_code:02X} cannot be read"):
            await low_level_api.read_system_parameter(can_id, param_to_read_cmd_code)