# mks_servo_can_project/tests/unit/test_axis.py
import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

from mks_servo_can_library.mks_servo_can.axis import Axis
from mks_servo_can_library.mks_servo_can.low_level_api import LowLevelAPI
from mks_servo_can_library.mks_servo_can.can_interface import CANInterface
from mks_servo_can_library.mks_servo_can.kinematics import RotaryKinematics
from mks_servo_can_library.mks_servo_can import constants as const
from mks_servo_can_library.mks_servo_can.exceptions import (
    ParameterError, MotorError, ConfigurationError, HomingError, CalibrationError,
    CommunicationError, LimitError, MKSServoError
)

try:
    from can import Message as CanMessage
except ImportError:
    class CanMessage:
        def __init__(self, arbitration_id=0, data=None, dlc=0, is_extended_id=False):
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b''
            self.dlc = dlc if data is None else len(self.data)
            self.is_extended_id = is_extended_id
            self.timestamp = 0.0
            self.channel = None
            self.is_rx = True
            self.is_error_frame = False
            self.is_remote_frame = False

@pytest.fixture
def mock_can_interface_for_axis():
    mock = AsyncMock(spec=CANInterface)
    mock.send_and_wait_for_response = AsyncMock()
    mock.send_message = AsyncMock()
    # CANInterface.create_response_future is a SYNCHRONOUS method returning a future.
    # So, its mock should be MagicMock, not AsyncMock.
    mock.create_response_future = MagicMock()
    return mock

@pytest.fixture
def mock_low_level_api():
    mock = AsyncMock(spec=LowLevelAPI)
    mock.read_encoder_value_addition = AsyncMock(return_value=0)
    mock.read_en_pin_status = AsyncMock(return_value=False)
    mock.query_motor_status = AsyncMock(return_value=const.MOTOR_STATUS_STOPPED)
    mock.run_position_mode_relative_pulses = AsyncMock(return_value=const.POS_RUN_STARTING)
    mock.run_position_mode_absolute_pulses = AsyncMock(return_value=const.POS_RUN_STARTING)
    mock.enable_motor = AsyncMock()
    mock.calibrate_encoder = AsyncMock()
    mock.go_home = AsyncMock(return_value=const.HOME_START)
    return mock

@pytest.fixture
def axis_instance(mock_can_interface_for_axis: MagicMock, mock_low_level_api: AsyncMock): # Type hint for mock_can_interface
    with patch('mks_servo_can_library.mks_servo_can.axis.LowLevelAPI', return_value=mock_low_level_api):
        kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
        axis = Axis(
            can_interface_manager=mock_can_interface_for_axis, # Pass the MagicMock here
            motor_can_id=1, name="TestAxis", kinematics=kin
        )
        axis._low_level_api = mock_low_level_api
        return axis

# ... (TestAxisBasicOps and TestAxisInitialization should remain the same as they passed) ...
@pytest.mark.asyncio
class TestAxisBasicOps:
    async def test_axis_init(self, mock_can_interface_for_axis):
        kin = RotaryKinematics(steps_per_revolution=1000)
        axis = Axis(mock_can_interface_for_axis, 1, "MyAxis", kinematics=kin)
        assert axis.name == "MyAxis"
        assert axis.can_id == 1
        assert axis.kinematics == kin
        assert axis._low_level_api is not None
        assert axis.is_enabled() is False 
        assert axis.is_homed() is False
        assert axis.is_calibrated() is False

    async def test_axis_init_default_kinematics(self, mock_can_interface_for_axis):
        axis = Axis(mock_can_interface_for_axis, 1, "DefaultKinAxis")
        assert isinstance(axis.kinematics, RotaryKinematics)
        assert axis.kinematics.steps_per_revolution == const.ENCODER_PULSES_PER_REVOLUTION

    async def test_axis_init_invalid_can_id(self, mock_can_interface_for_axis):
        with pytest.raises(ParameterError):
            Axis(mock_can_interface_for_axis, 0, "Axis0")
        with pytest.raises(ParameterError):
            Axis(mock_can_interface_for_axis, 0x800, "AxisHigh")

    async def test_enable_motor(self, axis_instance, mock_low_level_api):
        await axis_instance.enable_motor()
        mock_low_level_api.enable_motor.assert_called_once_with(axis_instance.can_id, True)
        assert axis_instance._is_enabled is True 
        mock_low_level_api.enable_motor.reset_mock()
        await axis_instance.enable_motor()
        mock_low_level_api.enable_motor.assert_not_called()

    async def test_disable_motor(self, axis_instance, mock_low_level_api):
        axis_instance._is_enabled = True 
        await axis_instance.disable_motor()
        mock_low_level_api.enable_motor.assert_called_once_with(axis_instance.can_id, False)
        assert axis_instance._is_enabled is False
        mock_low_level_api.enable_motor.reset_mock()
        await axis_instance.disable_motor() 
        mock_low_level_api.enable_motor.assert_not_called()

    async def test_get_current_position_user(self, axis_instance, mock_low_level_api):
        mock_steps = 8192 
        mock_low_level_api.read_encoder_value_addition.return_value = mock_steps
        expected_user_pos = axis_instance.kinematics.steps_to_user(mock_steps) 
        user_pos = await axis_instance.get_current_position_user()
        assert user_pos == pytest.approx(180.0)
        mock_low_level_api.read_encoder_value_addition.assert_called_once_with(axis_instance.can_id)
        assert axis_instance._current_position_steps == mock_steps

@pytest.mark.asyncio
class TestAxisInitialization:
    async def test_initialize_success_no_actions(self, axis_instance, mock_low_level_api):
        mock_low_level_api.read_encoder_value_addition.return_value = 100 
        mock_low_level_api.read_en_pin_status.return_value = True 
        await axis_instance.initialize(calibrate=False, home=False)
        mock_low_level_api.read_encoder_value_addition.assert_called_once()
        mock_low_level_api.read_en_pin_status.assert_called_once()
        mock_low_level_api.calibrate_encoder.assert_not_called()
        mock_low_level_api.go_home.assert_not_called()
        assert axis_instance._is_enabled is True
        assert axis_instance._current_position_steps == 100

    async def test_initialize_with_calibrate_and_home(self, axis_instance, mock_low_level_api):
        mock_low_level_api.read_encoder_value_addition.return_value = 0
        mock_low_level_api.read_en_pin_status.return_value = False 
        mock_low_level_api.calibrate_encoder.return_value = None 
        
        with patch.object(axis_instance, 'home_axis', new_callable=AsyncMock) as mock_axis_home:
            await axis_instance.initialize(calibrate=True, home=True)
            mock_axis_home.assert_called_once()

        mock_low_level_api.calibrate_encoder.assert_called_once()
        assert axis_instance._is_calibrated is True

    async def test_initialize_calibrate_fails_no_home(self, axis_instance, mock_low_level_api):
        mock_low_level_api.read_encoder_value_addition.return_value = 0
        mock_low_level_api.read_en_pin_status.return_value = False
        mock_low_level_api.calibrate_encoder.side_effect = CalibrationError("Simulated calib fail")

        with pytest.raises(CalibrationError, match="Simulated calib fail"):
            await axis_instance.initialize(calibrate=True, home=True)

        mock_low_level_api.calibrate_encoder.assert_called_once()
        mock_low_level_api.go_home.assert_not_called() 
        assert axis_instance._is_calibrated is False
        assert axis_instance._is_homed is False


@pytest.mark.asyncio
class TestAxisMovement:
    async def test_move_relative_pulses_success(self, axis_instance: Axis, mock_low_level_api: AsyncMock, mock_can_interface_for_axis: MagicMock): # Corrected mock type
        relative_pulses = 1000
        speed_param = 500
        accel_param = 100
        can_id = axis_instance.can_id
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES

        mock_low_level_api.run_position_mode_relative_pulses.return_value = const.POS_RUN_STARTING
        
        current_test_completion_future = asyncio.get_event_loop().create_future()
        # Set the return_value of the MagicMock
        mock_can_interface_for_axis.create_response_future.return_value = current_test_completion_future

        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(relative_pulses, speed_param, accel_param, wait=True)
        )
        
        await asyncio.sleep(0.01) 
        assert not current_test_completion_future.done(), "Completion future should be awaited by _execute_move"

        response_payload_data = [cmd_code, const.POS_RUN_COMPLETE]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF 
        completion_msg_data = bytes(response_payload_data + [response_crc])
        
        current_test_completion_future.set_result(
            CanMessage(arbitration_id=can_id, data=completion_msg_data, dlc=len(completion_msg_data))
        )
        
        await move_task 

        mock_low_level_api.run_position_mode_relative_pulses.assert_called_once_with(
            can_id, True, speed_param, accel_param, abs(relative_pulses)
        )
        mock_can_interface_for_axis.create_response_future.assert_called_once_with(can_id, cmd_code)
        assert axis_instance.is_move_complete() 
        mock_low_level_api.read_encoder_value_addition.assert_called()


    async def test_move_relative_pulses_limit_hit(self, axis_instance: Axis, mock_low_level_api: AsyncMock, mock_can_interface_for_axis: MagicMock): # Corrected mock type
        relative_pulses = 2000
        can_id = axis_instance.can_id
        cmd_code = const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES

        mock_low_level_api.run_position_mode_relative_pulses.return_value = const.POS_RUN_STARTING
        
        current_test_completion_future = asyncio.get_event_loop().create_future()
        mock_can_interface_for_axis.create_response_future.return_value = current_test_completion_future

        move_task = asyncio.create_task(
            axis_instance.move_relative_pulses(relative_pulses, wait=True)
        )
        await asyncio.sleep(0.01)

        response_payload_data = [cmd_code, const.POS_RUN_END_LIMIT_STOPPED]
        response_crc = (can_id + sum(response_payload_data)) & 0xFF
        completion_msg_data = bytes(response_payload_data + [response_crc])
        current_test_completion_future.set_result(
            CanMessage(arbitration_id=can_id, data=completion_msg_data, dlc=len(completion_msg_data))
        )

        with pytest.raises(LimitError): 
            await move_task
        
        assert axis_instance.is_move_complete()


    async def test_move_timeout(self, axis_instance: Axis, mock_low_level_api: AsyncMock, mock_can_interface_for_axis: MagicMock): # Corrected mock type
        relative_pulses = 500
        
        mock_low_level_api.run_position_mode_relative_pulses.return_value = const.POS_RUN_STARTING
        
        current_test_completion_future = asyncio.get_event_loop().create_future() 
        mock_can_interface_for_axis.create_response_future.return_value = current_test_completion_future

        # Patch the timeout constant used in Axis._execute_move for this test
        with patch('mks_servo_can_library.mks_servo_can.axis.const.CAN_TIMEOUT_SECONDS', 0.001): # Very short timeout
            with pytest.raises(CommunicationError, match="Timeout waiting for move"):
                await axis_instance.move_relative_pulses(relative_pulses, wait=True)
        
        assert axis_instance.is_move_complete() 
        assert axis_instance._active_move_future.done()
        # Check that the future has the correct exception
        with pytest.raises(CommunicationError):
            await axis_instance._active_move_future


    async def test_move_relative_pulses_calls_execute_move(self, axis_instance, mock_low_level_api):
        with patch.object(axis_instance, '_execute_move', new_callable=AsyncMock) as mock_exec_move:
            await axis_instance.move_relative_pulses(100, speed_param=100, accel_param=50, wait=False)
            mock_exec_move.assert_called_once()

