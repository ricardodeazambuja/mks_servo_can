# mks_servo_can_project/tests/unit/test_low_level_api.py
import pytest
import struct
from unittest.mock import AsyncMock, call, ANY # Added ANY
from typing import List


from mks_servo_can import constants as const
from mks_servo_can.can_interface import CANInterface
from mks_servo_can.exceptions import CalibrationError
from mks_servo_can.exceptions import CommandError
from mks_servo_can.exceptions import CommunicationError
from mks_servo_can.exceptions import CRCError
from mks_servo_can.exceptions import MotorError
from mks_servo_can.exceptions import ParameterError
from mks_servo_can.low_level_api import LowLevelAPI
from mks_servo_can.crc import calculate_crc

try:
    from can import Message as CanMessage
except ImportError:

    class CanMessage: # type: ignore
        def __init__(
            self, arbitration_id=0, data=None, is_extended_id=False, dlc=0
        ):
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b""
            self.is_extended_id = is_extended_id
            self.dlc = dlc if data is None else len(self.data) # type: ignore
            self.timestamp = 0.0
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

def _assert_message_properties(sent_msg_arg, expected_can_id, expected_data_bytes):
    assert sent_msg_arg.arbitration_id == expected_can_id
    assert sent_msg_arg.data == expected_data_bytes
    assert not sent_msg_arg.is_extended_id


@pytest.mark.asyncio
class TestLowLevelAPIReads:
    async def test_read_encoder_value_addition_success(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION
        encoder_val = 16384 
        val_bytes_payload = bytearray(8) 
        struct.pack_into("<q", val_bytes_payload, 0, encoder_val) 
        
        response_payload_data = [command_code] + list(val_bytes_payload[:6]) 
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        result_val = await low_level_api.read_encoder_value_addition(can_id)
        assert result_val == encoder_val

        sent_payload_data = [command_code]
        sent_crc = calculate_crc(can_id, sent_payload_data)
        expected_sent_data_bytes = bytes(sent_payload_data + [sent_crc])

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0] # Get args from the first call
        
        sent_msg_arg = args[0]
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        assert args[1] == can_id 
        assert args[2] == command_code
        assert args[3] == const.CAN_TIMEOUT_SECONDS


    async def test_read_motor_speed_rpm_success(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x02
        command_code = const.CMD_READ_MOTOR_SPEED_RPM
        rpm_val = -1200
        rpm_bytes = struct.pack("<h", rpm_val)

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

    async def test_read_command_communication_error(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        mock_can_interface.send_and_wait_for_response.side_effect = (
            CommunicationError("Simulated timeout")
        )
        with pytest.raises(CommunicationError, match="Simulated timeout"):
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_crc_error(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00]
        wrong_crc = 0x73 
        full_response_data_bytes = bytes(response_payload_data + [wrong_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )
        with pytest.raises( CRCError, match=f"Invalid CRC in response to command {command_code:02X}"):
            await low_level_api.read_encoder_value_addition(can_id)

    async def test_read_command_response_dlc_mismatch(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_READ_ENCODER_ADDITION
        response_payload_data = [command_code, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes) - 1, 
        )
        with pytest.raises( CommandError, match=f"Response DLC mismatch for command {command_code:02X}"):
            await low_level_api.read_encoder_value_addition(can_id)


@pytest.mark.asyncio
class TestLowLevelAPISets: 
    async def test_set_work_mode_success(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE
        mode_to_set = const.MODE_SR_VFOC

        sent_payload_data = [command_code, mode_to_set]
        sent_crc = calculate_crc(can_id, sent_payload_data)
        expected_sent_data_bytes = bytes(sent_payload_data + [sent_crc])
        
        response_payload_data = [command_code, const.STATUS_SUCCESS]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )

        await low_level_api.set_work_mode(can_id, mode_to_set)

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        
        sent_msg_arg = args[0]
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        assert args[1] == can_id 
        assert args[2] == command_code 
        assert args[3] == const.CAN_TIMEOUT_SECONDS


    async def test_set_work_mode_failure_response(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_SET_WORK_MODE
        mode_to_set = const.MODE_CR_OPEN
        status_failure = const.STATUS_FAILURE

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
        with pytest.raises(MotorError, match=expected_error_msg):
            await low_level_api.set_work_mode(can_id, mode_to_set)

    async def test_set_work_mode_invalid_param(self, low_level_api):
        with pytest.raises(ParameterError, match="Invalid work mode: 99"):
            await low_level_api.set_work_mode(0x01, 99)

    async def test_calibrate_encoder_failure(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        command_code = const.CMD_CALIBRATE_ENCODER

        response_payload_data = [command_code, const.STATUS_CALIBRATING_FAIL]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes),
        )
        with pytest.raises(CalibrationError, match="Encoder calibration failed."):
            await low_level_api.calibrate_encoder(can_id)


@pytest.mark.asyncio
class TestLowLevelAPISetSystemParamsExtended:
    @pytest.mark.parametrize("direction_code, direction_name", [
        (const.DIR_CW, "CW"), (const.DIR_CCW, "CCW")
    ])
    async def test_set_motor_direction(self, low_level_api, mock_can_interface, direction_code, direction_name):
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
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)


    async def test_set_motor_direction_invalid_param(self, low_level_api):
        with pytest.raises(ParameterError, match="Invalid motor direction code: 2"):
            await low_level_api.set_motor_direction(0x01, 2)

    @pytest.mark.parametrize("enable_auto_off, enable_code", [(True, 0x01), (False, 0x00)])
    async def test_set_auto_screen_off(self, low_level_api, mock_can_interface, enable_auto_off, enable_code):
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
        current_can_id = 0x01
        new_can_id = 0x05
        command_code = const.CMD_SET_CAN_ID
        
        id_bytes = list(struct.pack("<H", new_can_id))
        sent_payload_data = [command_code] + id_bytes
        sent_crc = calculate_crc(current_can_id, sent_payload_data)
        expected_sent_bytes = bytes(sent_payload_data + [sent_crc])

        response_payload = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(current_can_id, response_payload)
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
        with pytest.raises(ParameterError, match="Invalid new CAN ID: 2048"):
            await low_level_api.set_can_id(0x01, 0x800)

    async def test_set_slave_respond_active_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_SET_SLAVE_RESPOND_ACTIVE
        respond_enabled = True
        active_enabled = False
        
        data_payload_sent_list = [command_code, 0x01, 0x00] 
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
class TestLowLevelAPIWriteAndHomeCommands:
    async def test_write_io_port_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_WRITE_IO_PORT
        expected_data_byte = 0x54 

        sent_payload_list = [command_code, expected_data_byte]
        sent_crc = calculate_crc(can_id, sent_payload_list)
        expected_sent_bytes = bytes(sent_payload_list + [sent_crc])

        response_payload_list = [command_code, const.STATUS_SUCCESS]
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )

        await low_level_api.write_io_port(can_id, out1_value=1, out2_value=0, out1_mask_action=1, out2_mask_action=1)
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)


    async def test_set_home_parameters_success(self, low_level_api, mock_can_interface):
        can_id = 0x01
        command_code = const.CMD_SET_HOME_PARAMETERS
        home_trig = 0 
        home_dir_val = 1 
        home_speed = 100 
        end_limit_en = True
        home_mode_val = 0 

        speed_bytes = list(struct.pack("<H", home_speed))
        data_to_send_list = [command_code, home_trig, home_dir_val] + speed_bytes + [0x01, home_mode_val]
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
        can_id = 0x01
        command_code = const.CMD_GO_HOME
        sent_payload_list = [command_code]
        sent_crc = calculate_crc(can_id, sent_payload_list)
        expected_sent_bytes = bytes(sent_payload_list + [sent_crc])

        response_payload_list = [command_code, const.HOME_START]
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )
        status = await low_level_api.go_home(can_id)
        assert status == const.HOME_START
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)


@pytest.mark.asyncio
class TestLowLevelAPIRunCommandsExtended:
    def _pack_int24_for_test(self, value: int) -> List[int]:
        """Helper to pack signed 24-bit int for test comparison."""
        unsigned_val = value & 0xFFFFFF
        return [
            unsigned_val & 0xFF,
            (unsigned_val >> 8) & 0xFF,
            (unsigned_val >> 16) & 0xFF,
        ]

    @pytest.mark.parametrize("relative_axis_val", [-1000, 0, 1000])
    async def test_run_position_mode_relative_axis_success(self, low_level_api, mock_can_interface, relative_axis_val):
        can_id = 0x01
        command_code = const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS 
        speed = 500
        accel = 100

        speed_bytes = list(struct.pack("<H", speed))
        accel_byte = accel & 0xFF
        axis_bytes_packed = self._pack_int24_for_test(relative_axis_val)
        
        sent_data_list = [command_code] + speed_bytes + [accel_byte] + axis_bytes_packed
        sent_crc = calculate_crc(can_id, sent_data_list)
        expected_sent_bytes = bytes(sent_data_list + [sent_crc])

        response_payload_list = [command_code, const.POS_RUN_STARTING]
        response_crc_val = calculate_crc(can_id, response_payload_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_list + [response_crc_val]), dlc=3
        )

        status = await low_level_api.run_position_mode_relative_axis(can_id, speed, accel, relative_axis_val)
        assert status == const.POS_RUN_STARTING
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1:] == (can_id, command_code, const.CAN_TIMEOUT_SECONDS)
    
    @pytest.mark.parametrize("save_action, action_code", [(True, const.SPEED_MODE_PARAM_SAVE), (False, const.SPEED_MODE_PARAM_CLEAN)])
    async def test_save_or_clean_speed_mode_params(self, low_level_api, mock_can_interface, save_action, action_code):
        can_id = 0x01
        # This command (0xFF) echoes the data byte (action_code) in response, not the command code 0xFF
        command_code_sent = const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS 
        expected_echoed_command_in_response = action_code 

        sent_data_list = [command_code_sent, action_code]
        sent_crc = calculate_crc(can_id, sent_data_list)
        expected_sent_bytes = bytes(sent_data_list + [sent_crc])

        response_payload_resp_list = [action_code, const.STATUS_SUCCESS] # Echoes action_code
        response_crc_val = calculate_crc(can_id, response_payload_resp_list)
        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id, data=bytes(response_payload_resp_list + [response_crc_val]), dlc=3
        )

        await low_level_api.save_or_clean_speed_mode_params(can_id, save_action)
        
        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        _assert_message_properties(args[0], can_id, expected_sent_bytes)
        assert args[1] == can_id 
        assert args[2] == expected_echoed_command_in_response # Check that it expects the echoed action code
        assert args[3] == const.CAN_TIMEOUT_SECONDS


@pytest.mark.asyncio
class TestLowLevelAPIReadSystemParameter: 
    async def test_read_system_parameter_success(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        param_to_read_cmd_code = const.CMD_SET_WORK_MODE 

        sent_payload_data_for_00 = [const.CMD_READ_SYSTEM_PARAMETER_PREFIX, param_to_read_cmd_code]
        sent_crc_for_00 = calculate_crc(can_id, sent_payload_data_for_00)
        expected_sent_data_bytes = bytes(sent_payload_data_for_00 + [sent_crc_for_00])

        current_mode_val = const.MODE_SR_CLOSE 
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

        assert echoed_code == param_to_read_cmd_code
        assert param_data == bytes([current_mode_val])

        mock_can_interface.send_and_wait_for_response.assert_called_once()
        call_args_list = mock_can_interface.send_and_wait_for_response.call_args_list
        args, _ = call_args_list[0]
        
        sent_msg_arg = args[0]
        _assert_message_properties(sent_msg_arg, can_id, expected_sent_data_bytes)
        assert args[1] == can_id 
        assert args[2] == param_to_read_cmd_code 
        assert args[3] == const.CAN_TIMEOUT_SECONDS


    async def test_read_system_parameter_not_readable(
        self, low_level_api, mock_can_interface
    ):
        can_id = 0x01
        param_to_read_cmd_code = 0xAA 

        response_payload_data = [param_to_read_cmd_code, 0xFF, 0xFF]
        response_crc = calculate_crc(can_id, response_payload_data)
        full_response_data_bytes = bytes(response_payload_data + [response_crc])

        mock_can_interface.send_and_wait_for_response.return_value = CanMessage(
            arbitration_id=can_id,
            data=full_response_data_bytes,
            dlc=len(full_response_data_bytes), 
        )

        with pytest.raises( MotorError, match=f"Parameter with code {param_to_read_cmd_code:02X} cannot be read"):
            await low_level_api.read_system_parameter(
                can_id, param_to_read_cmd_code
            )
