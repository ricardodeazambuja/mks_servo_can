# mks_servo_can_project/mks_servo_can_library/mks_servo_can/low_level_api.py
"""
Low-Level API for MKS Servo CAN commands.
Implements functions to construct, send, and parse responses for each
CAN command specified in the MKS SERVO42D/57D_CAN User Manual.
"""
from typing import Dict, List, Optional, Tuple

import logging
import struct

try:
    from can import Message as CanMessage
except ImportError:

    class CanMessage:  # type: ignore
        def __init__(
            self, arbitration_id=0, data=None, is_extended_id=False, dlc=0 # type: ignore
        ):
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b""
            self.is_extended_id = is_extended_id
            self.dlc = (
                dlc if data is None else len(self.data)
            )
            self.timestamp = 0.0
            self.channel = None
            self.is_rx = True
            self.is_error_frame = False
            self.is_remote_frame = False


from . import constants as const
from .can_interface import CANInterface
from .crc import calculate_crc
from .crc import verify_crc
from .exceptions import CalibrationError
from .exceptions import CommandError
from .exceptions import CommunicationError
from .exceptions import CRCError
from .exceptions import MotorError
from .exceptions import ParameterError

logger = logging.getLogger(__name__)


class LowLevelAPI:
    """
    Provides methods to interact with MKS Servos at the CAN command level.
    """

    def __init__(self, can_interface: CANInterface):
        self.can_if = can_interface

    async def _send_command_and_get_response(
        self,
        can_id: int,
        command_code: int,
        data: Optional[List[int]] = None,
        expected_dlc: Optional[int] = None,
        timeout: float = const.CAN_TIMEOUT_SECONDS,
    ) -> CanMessage:
        if not (const.BROADCAST_ADDRESS <= can_id <= 0x7FF):
            raise ParameterError(
                f"Invalid CAN ID: {can_id}. Must be 0-2047 (0x7FF)."
            )

        payload_data = [command_code]
        if data:
            payload_data.extend(data)

        crc_id_for_calc = can_id
        crc = calculate_crc(crc_id_for_calc, payload_data)
        full_payload = payload_data + [crc]

        msg_to_send = CanMessage(
            arbitration_id=can_id,
            data=bytes(full_payload),
            is_extended_id=False,
        )

        # Ensure this logging line has the corrected f-string for msg_to_send.data[0]
        # (as you mentioned you fixed previously)
        cmd_byte_str = f"{msg_to_send.data[0]:02X}" if msg_to_send.data else "N/A"
        payload_str = msg_to_send.data.hex() if msg_to_send.data else "N/A"
        logger.info(
            f"LowLevelAPI: Attempting to send to CAN ID {can_id:03X}: "
            f"CMD={cmd_byte_str}, FullPayload={payload_str}, DLC={msg_to_send.dlc}"
        )

        expected_response_can_id = can_id
        expected_response_command_code = data[0] if command_code == const.CMD_READ_SYSTEM_PARAMETER_PREFIX and data else command_code

        try:
            response_msg = await self.can_if.send_and_wait_for_response(
                msg_to_send,
                expected_response_can_id,
                expected_response_command_code,
                timeout,
            )
        except CommunicationError as e:
            logger.warning(
                f"Communication error for cmd {command_code:02X} to ID {can_id:03X}: {e}"
            )
            raise

        if (
            not response_msg.data or len(response_msg.data) < 2
        ):
            raise CommandError(
                f"Response to command {command_code:02X} (expected echo {expected_response_command_code:02X}) "
                f"from ID {can_id:03X} is too short or empty. Data: {response_msg.data.hex()}"
            )

        if response_msg.data[0] != expected_response_command_code:
            raise CommandError(
                f"Response command code mismatch for command {command_code:02X} from ID {can_id:03X}. "
                f"Expected echo {expected_response_command_code:02X}, got {response_msg.data[0]:02X}. Data: {response_msg.data.hex()}"
            )

        if not verify_crc(response_msg.arbitration_id, list(response_msg.data)):
            raise CRCError(
                f"Invalid CRC in response to command {command_code:02X} (echoed {expected_response_command_code:02X}) "
                f"from ID {can_id:03X}. Data: {response_msg.data.hex()}"
            )

        if expected_dlc is not None and response_msg.dlc != expected_dlc:
            raise CommandError(
                f"Response DLC mismatch for command {command_code:02X} (echoed {expected_response_command_code:02X}) "
                f"from ID {can_id:03X}. Expected DLC {expected_dlc}, got {response_msg.dlc}. Data: {response_msg.data.hex()}"
            )
        return response_msg

    async def _send_command_no_response(
        self,
        can_id: int,
        command_code: int,
        data: Optional[List[int]] = None,
        timeout: float = const.CAN_TIMEOUT_SECONDS,
    ):
        if not (const.BROADCAST_ADDRESS <= can_id <= 0x7FF):
            raise ParameterError(
                f"Invalid CAN ID: {can_id}. Must be 0-2047 (0x7FF)."
            )
        payload_data = [command_code]
        if data:
            payload_data.extend(data)
        crc = calculate_crc(can_id, payload_data)
        full_payload = payload_data + [crc]
        msg_to_send = CanMessage(
            arbitration_id=can_id,
            data=bytes(full_payload),
            is_extended_id=False,
        )
        cmd_byte_str = f"{msg_to_send.data[0]:02X}" if msg_to_send.data else "N/A"
        payload_str = msg_to_send.data.hex() if msg_to_send.data else "N/A"
        logger.info(
            f"LowLevelAPI: Attempting to send (no response expected) to CAN ID {can_id:03X}: "
            f"CMD={cmd_byte_str}, FullPayload={payload_str}, DLC={msg_to_send.dlc}"
        )
        await self.can_if.send_message(msg_to_send, timeout=timeout)

    # --- Part 5.1: Read Status Parameter Commands ---
    async def read_encoder_value_carry(self, can_id: int) -> Tuple[int, int]:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_ENCODER_CARRY, expected_dlc=8
        )
        carry = struct.unpack("<i", response.data[1:5])[0]
        value = struct.unpack("<H", response.data[5:7])[0]
        return carry, value

    async def read_encoder_value_addition(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_ENCODER_ADDITION, expected_dlc=8
        )
        val_bytes = bytearray(response.data[1:7]) 
        if val_bytes[5] & 0x80:
            val_bytes += b"\xff\xff"
        else:
            val_bytes += b"\x00\x00"
        return struct.unpack("<q", val_bytes)[0]

    async def read_motor_speed_rpm(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_SPEED_RPM, expected_dlc=4
        )
        speed = struct.unpack("<h", response.data[1:3])[0]
        return speed

    async def read_pulses_received(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_PULSES_RECEIVED, expected_dlc=6
        )
        pulses = struct.unpack("<i", response.data[1:5])[0]
        return pulses

    async def read_io_status(self, can_id: int) -> Dict[str, int]:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_IO_STATUS, expected_dlc=3
        )
        status_byte = response.data[1]
        return {
            "IN_1": (status_byte >> 0) & 0x01,
            "IN_2": (status_byte >> 1) & 0x01,
            "OUT_1": (status_byte >> 2) & 0x01,
            "OUT_2": (status_byte >> 3) & 0x01,
            "raw_byte": status_byte,
        }

    async def read_raw_encoder_value_addition(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_RAW_ENCODER_ADDITION, expected_dlc=8
        )
        val_bytes = bytearray(response.data[1:7]) 
        if val_bytes[5] & 0x80:
            val_bytes += b"\xff\xff"
        else:
            val_bytes += b"\x00\x00"
        return struct.unpack("<q", val_bytes)[0]

    async def read_shaft_angle_error(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_SHAFT_ANGLE_ERROR, expected_dlc=6
        )
        return struct.unpack("<i", response.data[1:5])[0]

    async def read_en_pin_status(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_EN_PIN_STATUS, expected_dlc=3
        )
        return response.data[1] == 0x01

    async def read_power_on_zero_status(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_POWER_ON_ZERO_STATUS, expected_dlc=3
        )
        status = response.data[1]
        if status == 2:
            logger.warning(
                f"Power on zero status for CAN ID {can_id} reported failure (status 2)."
            )
        return status

    async def release_stall_protection(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RELEASE_STALL_PROTECTION, expected_dlc=3
        )
        if response.data[1] == 0x01:
            return True
        elif response.data[1] == 0x00:
            logger.warning(
                f"Release stall protection failed for CAN ID {can_id} (status 0)."
            )
            return False
        else:
            raise CommandError(
                f"Unexpected status from release_stall_protection: {response.data[1]}"
            )

    async def read_motor_protection_state(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_PROTECTION_STATE, expected_dlc=3
        )
        return response.data[1] == 0x01

    async def calibrate_encoder(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_CALIBRATE_ENCODER, data=[0x00], expected_dlc=3
        )
        status = response.data[1]
        if status == const.STATUS_CALIBRATING_FAIL:
            raise CalibrationError(
                "Encoder calibration failed.", error_code=status, can_id=can_id
            )
        elif status == const.STATUS_CALIBRATED_SUCCESS:
            logger.info(f"CAN ID {can_id}: Encoder calibration successful.")
        elif status == const.STATUS_CALIBRATING:
            logger.info(f"CAN ID {can_id}: Encoder calibration started/in progress.")
        else:
            raise CommandError(
                f"Unexpected status from calibrate_encoder: {status}"
            )

    async def set_work_mode(self, can_id: int, mode: int) -> None:
        if not (0 <= mode <= 5):
            raise ParameterError(f"Invalid work mode: {mode}.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_WORK_MODE, data=[mode], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set work mode to {mode} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_working_current(self, can_id: int, current_ma: int) -> None:
        if not (0 <= current_ma <= 5200):
            raise ParameterError(f"Invalid working current: {current_ma}mA.")
        current_bytes = list(struct.pack("<H", current_ma))
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_WORKING_CURRENT,
            data=current_bytes,
            expected_dlc=3,
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set working current to {current_ma}mA failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_holding_current_percentage(
        self, can_id: int, percentage_code: int
    ) -> None:
        if not (0x00 <= percentage_code <= 0x08):
            raise ParameterError(
                f"Invalid holding current code: {percentage_code} (must be 0-8)."
            )
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_HOLDING_CURRENT_PERCENTAGE,
            data=[percentage_code],
            expected_dlc=3,
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set holding current percentage failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_subdivision(self, can_id: int, microsteps: int) -> None:
        if not (1 <= microsteps <= 255):
            raise ParameterError(f"Invalid microsteps: {microsteps}.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SUBDIVISION, data=[microsteps], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set subdivision failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
    
    async def set_subdivision_interpolation(self, can_id: int, enable: bool) -> None:
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SUBDIVISION_INTERPOLATION, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set subdivision interpolation (enable={enable}) failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_en_pin_active_level(self, can_id: int, level_code: int) -> None:
        if level_code not in [
            const.EN_ACTIVE_LOW,
            const.EN_ACTIVE_HIGH,
            const.EN_ACTIVE_ALWAYS,
        ]:
            raise ParameterError(
                f"Invalid En pin active level code: {level_code}."
            )
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_EN_PIN_ACTIVE_LEVEL,
            data=[level_code],
            expected_dlc=3,
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set En pin active level to {level_code} failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    async def go_home(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_GO_HOME, expected_dlc=3
        )
        status = response.data[1] 
        if status == const.HOME_FAIL:
             raise MotorError(
                f"Go home command failed to start. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status

    async def query_motor_status(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_QUERY_MOTOR_STATUS, expected_dlc=3
        )
        status = response.data[1]
        if status == const.MOTOR_STATUS_QUERY_FAIL: 
             raise MotorError(
                f"Query motor status command failed. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status

    async def enable_motor(self, can_id: int, enable: bool) -> None:
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_ENABLE_MOTOR, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Enable motor command (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def _run_motor_command(
        self, can_id: int, command_code: int, data: List[int]
    ) -> int:
        """Helper for motor run commands that return status 0,1,2,3."""
        # ADDED LOGGING HERE
        logger.info(f"LowLevelAPI._run_motor_command: Preparing to send CMD={command_code:02X} to CAN_ID={can_id:03X} with DataForCmd={data}")
        response = await self._send_command_and_get_response(
            can_id, command_code, data=data, expected_dlc=3
        )
        status = response.data[1]
        # ADDED LOGGING HERE
        logger.info(f"LowLevelAPI._run_motor_command: CMD={command_code:02X} for CAN_ID={can_id:03X} received response status={status:02X}")
        if status == const.POS_RUN_FAIL: # 0x00
            raise MotorError(
                f"Motor run command {command_code:02X} for CAN_ID={can_id:03X} failed to start (status 0).",
                error_code=status,
                can_id=can_id,
            )
        return status

    async def read_system_parameter(
        self, can_id: int, parameter_command_code: int
    ) -> Tuple[int, bytes]:
        payload_data_for_send = [
            const.CMD_READ_SYSTEM_PARAMETER_PREFIX,
            parameter_command_code,
        ]
        response_msg = await self._send_command_and_get_response(
            can_id,
            const.CMD_READ_SYSTEM_PARAMETER_PREFIX,
            data=[parameter_command_code], 
        )

        if (
            response_msg.dlc == 4 
            and response_msg.data[0] == parameter_command_code 
            and response_msg.data[1] == 0xFF
            and response_msg.data[2] == 0xFF
        ):
            raise MotorError(
                f"Parameter with code {parameter_command_code:02X} cannot be read from CAN ID {can_id} (responded FF FF).",
                error_code=0xFFFF, 
                can_id=can_id,
            )
        param_data_bytes = response_msg.data[1:-1]
        return response_msg.data[0], param_data_bytes


    async def set_current_axis_to_zero(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_CURRENT_AXIS_TO_ZERO, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set current axis to zero failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def run_position_mode_relative_pulses(
        self,
        can_id: int,
        ccw_direction: bool,
        speed: int, 
        acceleration: int, 
        pulses: int, 
    ) -> int:
        # ADDED LOGGING HERE
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_pulses: CAN_ID={can_id:03X}, "
            f"CCW={ccw_direction}, Speed={speed}, Accel={acceleration}, Pulses={pulses}"
        )
        if not (0 <= speed <= 0xFFF): 
            logger.error(f"LowLevelAPI: Speed validation failed for 0xFD. Speed={speed}")
            raise ParameterError(f"Speed {speed} out of range for 0xFD (0-4095).")
        if not (0 <= acceleration <= 255):
            logger.error(f"LowLevelAPI: Accel validation failed for 0xFD. Accel={acceleration}")
            raise ParameterError(f"Acceleration {acceleration} out of range (0-255).")
        if not (0 <= pulses <= 0xFFFFFF): 
            logger.error(f"LowLevelAPI: Pulses validation failed for 0xFD. Pulses={pulses}")
            raise ParameterError(f"Pulses {pulses} out of range for 0xFD (0-16,777,215).")

        byte2 = (speed >> 8) & 0x0F 
        if not ccw_direction: 
            byte2 |= 0x80 

        byte3 = speed & 0xFF 
        byte4 = acceleration & 0xFF

        pulse_bytes = [
            pulses & 0xFF,
            (pulses >> 8) & 0xFF,
            (pulses >> 16) & 0xFF,
        ]
        data = [byte2, byte3, byte4] + pulse_bytes 
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_pulses: Prepared data for 0xFD: {data}"
        )
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data
        )

    async def run_position_mode_absolute_pulses(
        self, can_id: int, speed: int, acceleration: int, absolute_pulses: int
    ) -> int:
        # ADDED LOGGING HERE
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_pulses: CAN_ID={can_id:03X}, "
            f"Speed={speed}, Accel={acceleration}, AbsPulses={absolute_pulses}"
        )

        if not (0 <= speed <= 3000):
            logger.error(f"LowLevelAPI: Speed validation failed for 0xFE. Speed={speed}")
            raise ParameterError(f"Speed {speed} out of range for 0xFE (0-3000).")
        if not (0 <= acceleration <= 255):
            logger.error(f"LowLevelAPI: Accel validation failed for 0xFE. Accel={acceleration}")
            raise ParameterError(f"Acceleration {acceleration} out of range (0-255).")
        if not (-0x800000 <= absolute_pulses <= 0x7FFFFF): 
            logger.error(f"LowLevelAPI: AbsPulses validation failed for 0xFE. AbsPulses={absolute_pulses}")
            raise ParameterError(f"Absolute pulses {absolute_pulses} out of range for signed 24-bit for 0xFE.")

        speed_bytes = list(struct.pack("<H", speed)) 
        acc_byte = acceleration & 0xFF
        
        val_to_pack = absolute_pulses
        if val_to_pack < 0:
            val_to_pack = (1 << 24) + val_to_pack 
        val_to_pack &= 0xFFFFFF 

        abs_pulse_bytes = [
            val_to_pack & 0xFF,
            (val_to_pack >> 8) & 0xFF,
            (val_to_pack >> 16) & 0xFF,
        ]
        data = speed_bytes + [acc_byte] + abs_pulse_bytes 
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_pulses: Prepared data for 0xFE: {data}"
        )
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data
        )

    async def run_speed_mode(
        self, can_id: int, ccw_direction: bool, speed: int, acceleration: int
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_speed_mode: CAN_ID={can_id:03X}, CCW={ccw_direction}, "
            f"Speed={speed}, Accel={acceleration}"
        )
        if not (0 <= speed <= 0xFFF): 
            logger.error(f"LowLevelAPI: Speed validation failed for 0xF6. Speed={speed}")
            raise ParameterError(f"Speed {speed} out of range for 0xF6 (0-4095).")
        if not (0 <= acceleration <= 255):
            logger.error(f"LowLevelAPI: Accel validation failed for 0xF6. Accel={acceleration}")
            raise ParameterError(f"Acceleration {acceleration} out of range (0-255).")

        byte2 = (speed >> 8) & 0x0F
        if not ccw_direction: 
            byte2 |= 0x80

        byte3 = speed & 0xFF
        byte4 = acceleration
        data = [byte2, byte3, byte4] 
        logger.info(f"LowLevelAPI.run_speed_mode: Prepared data for 0xF6: {data}")
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data
        )

    async def stop_speed_mode(self, can_id: int, acceleration: int) -> int:
        logger.info(f"LowLevelAPI.stop_speed_mode: CAN_ID={can_id:03X}, Accel={acceleration}")
        if not (0 <= acceleration <= 255):
            logger.error(f"LowLevelAPI: Accel validation failed for stop_speed_mode (0xF6 with speed 0). Accel={acceleration}")
            raise ParameterError(f"Acceleration {acceleration} out of range (0-255).")

        byte2 = 0x00 
        byte3 = 0x00 
        byte4 = acceleration
        data = [byte2, byte3, byte4]
        logger.info(f"LowLevelAPI.stop_speed_mode: Prepared data for 0xF6 (stop): {data}")
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data 
        )
        
    async def emergency_stop(self, can_id: int) -> None:
        logger.info(f"LowLevelAPI.emergency_stop: CAN_ID={can_id:03X}")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_EMERGENCY_STOP, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: 
            raise MotorError(
                f"Emergency stop command failed. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        