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

        cmd_byte_str = f"{msg_to_send.data[0]:02X}" if msg_to_send.data else "N/A"
        payload_str = msg_to_send.data.hex() if msg_to_send.data else "N/A"
        logger.info(
            f"LowLevelAPI: Attempting to send to CAN ID {can_id:03X}: "
            f"CMD={cmd_byte_str}, FullPayload={payload_str}, DLC={msg_to_send.dlc}"
        )

        expected_response_can_id = can_id
        
        # Determine the expected command code in the response
        if command_code == const.CMD_READ_SYSTEM_PARAMETER_PREFIX and data:
            expected_response_command_code = data[0]
        elif command_code == const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS and data:
            expected_response_command_code = data[0] # Motor echoes the data byte (0xC8 or 0xCA)
        else:
            expected_response_command_code = command_code

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
        ): # Must have at least echoed command and CRC
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
        status = response.data[1]
        if status == 0x01:
            return True
        elif status == 0x00:
            logger.warning(
                f"Release stall protection failed for CAN ID {can_id} (status 0)."
            )
            return False
        else:
            raise CommandError(
                f"Unexpected status from release_stall_protection: {status}"
            )

    async def read_motor_protection_state(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_PROTECTION_STATE, expected_dlc=3
        )
        return response.data[1] == 0x01

    # --- Part 5.2: Set system parameters command ---
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
        if not (const.MODE_CR_OPEN <= mode <= const.MODE_SR_VFOC):
            raise ParameterError(f"Invalid work mode: {mode}. Must be 0-5.")
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
        if not (0 <= current_ma <= 6000):
            raise ParameterError(f"Invalid working current: {current_ma}mA. Seems out of typical range.")
        current_bytes = list(struct.pack("<H", current_ma))
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_WORKING_CURRENT,
            data=current_bytes,
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set working current to {current_ma}mA failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_holding_current_percentage(
        self, can_id: int, percentage_code: int
    ) -> None:
        if not (0x00 <= percentage_code <= 0x08):
            raise ParameterError(
                f"Invalid holding current code: {percentage_code}. Must be 0-8."
            )
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_HOLDING_CURRENT_PERCENTAGE,
            data=[percentage_code],
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set holding current percentage to code {percentage_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_subdivision(self, can_id: int, microsteps: int) -> None:
        if not (0 <= microsteps <= 255):
            raise ParameterError(f"Invalid microsteps value: {microsteps}. Must be 0-255.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SUBDIVISION, data=[microsteps], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set subdivision to {microsteps} failed for CAN ID {can_id}. Status: {response.data[1]}",
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
                f"Invalid En pin active level code: {level_code}. Must be 0, 1, or 2."
            )
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_EN_PIN_ACTIVE_LEVEL,
            data=[level_code],
            expected_dlc=3,
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set En pin active level to code {level_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_motor_direction(self, can_id: int, direction_code: int) -> None:
        if direction_code not in [const.DIR_CW, const.DIR_CCW]:
            raise ParameterError(
                f"Invalid motor direction code: {direction_code}. Must be 0 (CW) or 1 (CCW)."
            )
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_MOTOR_DIRECTION, data=[direction_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set motor direction to code {direction_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_auto_screen_off(self, can_id: int, enable: bool) -> None:
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_AUTO_SCREEN_OFF, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set auto screen off (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_stall_protection(self, can_id: int, enable: bool) -> None:
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_STALL_PROTECTION, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set stall protection (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
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
                f"Set subdivision interpolation (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_can_bitrate(self, can_id: int, bitrate_code: int) -> None:
        if bitrate_code not in [
            const.CAN_BITRATE_125K,
            const.CAN_BITRATE_250K,
            const.CAN_BITRATE_500K,
            const.CAN_BITRATE_1M,
        ]:
            raise ParameterError(f"Invalid CAN bitrate code: {bitrate_code}.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_CAN_BITRATE, data=[bitrate_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set CAN bitrate to code {bitrate_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_can_id(self, current_can_id: int, new_can_id: int) -> None:
        if not (0 <= new_can_id <= 0x7FF):
            raise ParameterError(f"Invalid new CAN ID: {new_can_id}. Must be 0-2047.")
        id_bytes = list(struct.pack("<H", new_can_id))
        
        response = await self._send_command_and_get_response(
            current_can_id,
            const.CMD_SET_CAN_ID,
            data=id_bytes,
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set CAN ID to {new_can_id} failed for motor currently at {current_can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=current_can_id,
            )

    async def set_slave_respond_active(
        self, can_id: int, respond_enabled: bool, active_enabled: bool
    ) -> None:
        respond_code = 0x01 if respond_enabled else 0x00
        active_code = 0x01 if active_enabled else 0x00
        data = [respond_code, active_code]
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SLAVE_RESPOND_ACTIVE, data=data, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set slave respond/active (respond={respond_enabled}, active={active_enabled}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_group_id(self, can_id: int, group_id: int) -> None:
        if not (0x01 <= group_id <= 0x7FF):
            raise ParameterError(f"Invalid group ID: {group_id}. Must be 1-2047.")
        group_id_bytes = list(struct.pack("<H", group_id))

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_GROUP_ID, data=group_id_bytes, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set group ID to {group_id} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_key_lock(self, can_id: int, lock_enabled: bool) -> None:
        enable_code = 0x01 if lock_enabled else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_KEY_LOCK, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set key lock (enable={lock_enabled}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    # --- Part 5.3: Write IO port command ---
    async def write_io_port(
        self,
        can_id: int,
        out1_value: Optional[int] = None,
        out2_value: Optional[int] = None,
        out1_mask_action: int = 2,
        out2_mask_action: int = 2
    ) -> None:
        if not (0 <= out1_mask_action <= 2 and 0 <= out2_mask_action <= 2):
            raise ParameterError("Mask actions must be 0, 1, or 2.")

        data_byte = 0
        
        if out2_mask_action == 1: 
            if out2_value not in [0, 1]:
                raise ParameterError("out2_value must be 0 or 1 when out2_mask_action is 1.")
            data_byte |= (0b01 << 6) 
            if out2_value == 1: data_byte |= (1 << 3)
        elif out2_mask_action == 0: 
            data_byte |= (0b00 << 6)
        elif out2_mask_action == 2: 
            data_byte |= (0b10 << 6)
        
        if out1_mask_action == 1: 
            if out1_value not in [0, 1]:
                raise ParameterError("out1_value must be 0 or 1 when out1_mask_action is 1.")
            data_byte |= (0b01 << 4)
            if out1_value == 1: data_byte |= (1 << 2)
        elif out1_mask_action == 0: 
            data_byte |= (0b00 << 4)
        elif out1_mask_action == 2: 
            data_byte |= (0b10 << 4)
        
        response = await self._send_command_and_get_response(
            can_id, const.CMD_WRITE_IO_PORT, data=[data_byte], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Write IO port failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    # --- Part 5.4: Set Home command ---
    async def set_home_parameters(
        self,
        can_id: int,
        home_trig_level: int,
        home_dir: int,
        home_speed_rpm: int,
        end_limit_enabled: bool,
        home_mode: int
    ) -> None:
        if home_trig_level not in [0, 1]:
            raise ParameterError("home_trig_level must be 0 (Low) or 1 (High).")
        if home_dir not in [0, 1]:
            raise ParameterError("home_dir must be 0 (CW) or 1 (CCW).")
        if not (0 <= home_speed_rpm <= 3000):
            raise ParameterError("home_speed_rpm must be between 0 and 3000.")
        if home_mode not in [0, 1]:
            raise ParameterError("home_mode must be 0 (Use Limit) or 1 (No Limit).")

        byte2_level = home_trig_level & 0x01
        byte3_dir = home_dir & 0x01
        byte4_5_speed = list(struct.pack("<H", home_speed_rpm))
        byte6_endlimit = 0x01 if end_limit_enabled else 0x00
        byte7_hmMode = home_mode & 0x01
        
        data_payload = [byte2_level, byte3_dir] + byte4_5_speed + [byte6_endlimit, byte7_hmMode]

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_HOME_PARAMETERS, data=data_payload, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set home parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
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
                f"Go home command failed to start or reported immediate failure for CAN ID {can_id}. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status
        
    async def set_current_axis_to_zero(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_CURRENT_AXIS_TO_ZERO, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set current axis to zero failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_nolimit_home_params(
        self, can_id: int, reverse_angle_pulses: int, home_current_ma: int
    ) -> None:
        if not (0 <= reverse_angle_pulses <= 0xFFFFFFFF):
            raise ParameterError("reverse_angle_pulses must be a uint32_t value.")
        if not (0 <= home_current_ma <= 0xFFFF):
            raise ParameterError("home_current_ma must be a uint16_t value.")

        ret_val_bytes = list(struct.pack("<I", reverse_angle_pulses))
        ma_bytes = list(struct.pack("<H", home_current_ma))
        data_payload = ret_val_bytes + ma_bytes

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_NOLIMIT_HOME_PARAMS, data=data_payload, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set no-limit home parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_limit_port_remap(self, can_id: int, enable_remap: bool) -> None:
        enable_code = 0x01 if enable_remap else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_LIMIT_PORT_REMAP, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set limit port remap (enable={enable_remap}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.5: Set 0_Mode command ---
    async def set_zero_mode_parameters(
        self,
        can_id: int,
        mode: int,
        set_zero_action: int,
        speed_code: int,
        direction_code: int
    ) -> None:
        if mode not in [0, 1, 2]:
            raise ParameterError("Invalid 0_Mode 'mode'. Must be 0, 1, or 2.")
        if set_zero_action not in [0, 1, 2]:
            raise ParameterError("Invalid 0_Mode 'set_zero_action'. Must be 0, 1, or 2.")
        if not (0 <= speed_code <= 4):
            raise ParameterError("Invalid 0_Mode 'speed_code'. Must be 0-4.")
        if direction_code not in [0, 1]:
            raise ParameterError("Invalid 0_Mode 'direction_code'. Must be 0 (CW) or 1 (CCW).")
        
        data_payload = [mode, set_zero_action, speed_code, direction_code]
        
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_ZERO_MODE_PARAMETERS, data=data_payload, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set 0_Mode parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.6: Restore Default Parameters ---
    async def restore_default_parameters(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RESTORE_DEFAULT_PARAMETERS, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Restore default parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        logger.info(f"CAN ID {can_id}: Default parameters restored. Motor will restart and requires calibration.")

    # --- Part 5.7: Restart Motor ---
    async def restart_motor(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RESTART_MOTOR, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Restart motor command failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        logger.info(f"CAN ID {can_id}: Motor restart command sent successfully.")

    # --- Part 5.8: En triggers single-turn zero return and position error protection ---
    async def set_en_trigger_and_pos_error_protection(
        self,
        can_id: int,
        enable_en_trigger_zero: bool,
        enable_pos_error_protection: bool,
        error_detection_time_ms_units: int,
        error_threshold_pulses: int
    ) -> None:
        if not (0 <= error_detection_time_ms_units <= 0xFFFF):
            raise ParameterError("error_detection_time_ms_units out of range (uint16_t).")
        if not (0 <= error_threshold_pulses <= 0xFFFF):
            raise ParameterError("error_threshold_pulses out of range (uint16_t).")

        byte2_config = 0x00
        if enable_en_trigger_zero: byte2_config |= (1 << 1)
        if enable_pos_error_protection: byte2_config |= (1 << 0)
        
        tim_bytes = list(struct.pack("<H", error_detection_time_ms_units))
        errors_bytes = list(struct.pack("<H", error_threshold_pulses))
        data_payload = [byte2_config] + tim_bytes + errors_bytes

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION, data=data_payload, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(
                f"Set En trigger and Pos Error Protection failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.9: Read system Parameter command ---
    async def read_system_parameter(
        self, can_id: int, parameter_command_code: int
    ) -> Tuple[int, bytes]:
        response_msg = await self._send_command_and_get_response(
            can_id,
            const.CMD_READ_SYSTEM_PARAMETER_PREFIX,
            data=[parameter_command_code],
        )
        if len(response_msg.data) == 4 and \
           response_msg.data[0] == parameter_command_code and \
           response_msg.data[1] == 0xFF and \
           response_msg.data[2] == 0xFF:
            raise MotorError(
                f"Parameter with code {parameter_command_code:02X} cannot be read from CAN ID {can_id} (motor responded 0xFF 0xFF).",
                error_code=0xFFFF,
                can_id=can_id,
            )
        echoed_code = response_msg.data[0]
        param_data_bytes = response_msg.data[1:-1]
        return echoed_code, param_data_bytes
        
    # --- Part 6: Run Motor Commands ---
    async def _run_motor_command(
        self, can_id: int, command_code: int, data: List[int]
    ) -> int:
        logger.info(f"LowLevelAPI._run_motor_command: Preparing to send CMD={command_code:02X} to CAN_ID={can_id:03X} with DataForCmd={data}")
        response = await self._send_command_and_get_response(
            can_id, command_code, data=data, expected_dlc=3
        )
        status = response.data[1]
        logger.info(f"LowLevelAPI._run_motor_command: CMD={command_code:02X} for CAN_ID={can_id:03X} received response status={status:02X}")
        if status == const.POS_RUN_FAIL:
            raise MotorError(
                f"Motor run command {command_code:02X} for CAN ID {can_id:03X} failed to start or reported immediate failure (status 0x00).",
                error_code=status,
                can_id=can_id,
            )
        return status

    async def run_position_mode_relative_pulses(
        self,
        can_id: int,
        ccw_direction: bool,
        speed: int,
        acceleration: int,
        pulses: int,
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_pulses: CAN_ID={can_id:03X}, "
            f"CCW={ccw_direction}, SpeedParam={speed}, AccelParam={acceleration}, Pulses={pulses}"
        )
        if not (0 <= speed <= 3000):
            raise ParameterError(f"Speed parameter {speed} out of range for 0xFD (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (0 <= pulses <= 0xFFFFFF):
            raise ParameterError(f"Pulses {pulses} out of range for 0xFD (0-16,777,215).")

        byte2 = (speed >> 8) & 0x0F 
        if not ccw_direction: byte2 |= 0x80
        byte3 = speed & 0xFF
        byte4 = acceleration & 0xFF
        pulse_bytes = [pulses & 0xFF, (pulses >> 8) & 0xFF, (pulses >> 16) & 0xFF]
        data_payload = [byte2, byte3, byte4] + pulse_bytes 
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data_payload
        )

    async def run_position_mode_absolute_pulses(
        self, can_id: int, speed: int, acceleration: int, absolute_pulses: int
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_pulses: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, AbsPulses={absolute_pulses}"
        )
        if not (0 <= speed <= 3000):
            raise ParameterError(f"Speed parameter {speed} out of range for 0xFE (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= absolute_pulses <= 8388607):
             raise ParameterError(f"Absolute pulses {absolute_pulses} out of range for signed 24-bit for 0xFE.")

        speed_bytes = list(struct.pack("<H", speed)) 
        acc_byte = acceleration & 0xFF
        val_to_pack_unsigned_24bit = absolute_pulses & 0xFFFFFF
        abs_pulse_bytes = [
            val_to_pack_unsigned_24bit & 0xFF,
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,
            (val_to_pack_unsigned_24bit >> 16) & 0xFF,
        ]
        data_payload = speed_bytes + [acc_byte] + abs_pulse_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data_payload
        )

    async def run_speed_mode(
        self, can_id: int, ccw_direction: bool, speed: int, acceleration: int
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_speed_mode: CAN_ID={can_id:03X}, CCW={ccw_direction}, "
            f"SpeedParam={speed}, AccelParam={acceleration}"
        )
        if not (0 <= speed <= 3000): # Manual for 0xF6 states speed 0-3000 for parameter
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF6 (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")

        byte2 = (speed >> 8) & 0x0F
        if not ccw_direction: byte2 |= 0x80
        byte3 = speed & 0xFF
        byte4 = acceleration
        data_payload = [byte2, byte3, byte4] 
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data_payload
        )

    async def stop_speed_mode(self, can_id: int, acceleration: int) -> int:
        logger.info(f"LowLevelAPI.stop_speed_mode: CAN_ID={can_id:03X}, AccelParam={acceleration}")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        byte2 = 0x00 
        byte3 = 0x00 
        byte4 = acceleration
        data_payload = [byte2, byte3, byte4]
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data_payload
        )
        
    async def emergency_stop(self, can_id: int) -> None:
        logger.info(f"LowLevelAPI.emergency_stop: CAN_ID={can_id:03X}")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_EMERGENCY_STOP, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: 
            raise MotorError(
                f"Emergency stop command failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
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

    async def query_motor_status(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_QUERY_MOTOR_STATUS, expected_dlc=3
        )
        status = response.data[1]
        if status == const.MOTOR_STATUS_QUERY_FAIL:
             raise MotorError(
                f"Query motor status command failed for CAN ID {can_id}. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status

    async def run_position_mode_relative_axis(
        self, can_id: int, speed: int, acceleration: int, relative_axis: int
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_axis: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, RelAxis={relative_axis}"
        )
        if not (0 <= speed <= 3000):
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF4 (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= relative_axis <= 8388607): 
             raise ParameterError(f"Relative axis value {relative_axis} out of range for signed 24-bit for 0xF4.")

        speed_bytes = list(struct.pack("<H", speed)) 
        acc_byte = acceleration & 0xFF
        
        val_to_pack_unsigned_24bit = relative_axis & 0xFFFFFF
        
        rel_axis_bytes = [
            val_to_pack_unsigned_24bit & 0xFF,
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,
            (val_to_pack_unsigned_24bit >> 16) & 0xFF,
        ]
        data_payload = speed_bytes + [acc_byte] + rel_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data_payload
        )

    async def stop_position_mode_relative_axis(self, can_id: int, acceleration: int) -> int:
        logger.info(
            f"LowLevelAPI.stop_position_mode_relative_axis: CAN_ID={can_id:03X}, AccelParam={acceleration}"
        )
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        speed_bytes = [0x00, 0x00] 
        acc_byte = acceleration & 0xFF
        rel_axis_bytes = [0x00, 0x00, 0x00] 
        data_payload = speed_bytes + [acc_byte] + rel_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data_payload
        )

    async def run_position_mode_absolute_axis(
        self, can_id: int, speed: int, acceleration: int, absolute_axis: int
    ) -> int:
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_axis: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, AbsAxis={absolute_axis}"
        )
        if not (0 <= speed <= 3000):
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF5 (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= absolute_axis <= 8388607): 
             raise ParameterError(f"Absolute axis value {absolute_axis} out of range for signed 24-bit for 0xF5.")

        speed_bytes = list(struct.pack("<H", speed))
        acc_byte = acceleration & 0xFF
        
        val_to_pack_unsigned_24bit = absolute_axis & 0xFFFFFF
        
        abs_axis_bytes = [
            val_to_pack_unsigned_24bit & 0xFF,
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,
            (val_to_pack_unsigned_24bit >> 16) & 0xFF,
        ]
        data_payload = speed_bytes + [acc_byte] + abs_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data_payload
        )

    async def stop_position_mode_absolute_axis(self, can_id: int, acceleration: int) -> int:
        logger.info(
            f"LowLevelAPI.stop_position_mode_absolute_axis: CAN_ID={can_id:03X}, AccelParam={acceleration}"
        )
        if not (0 <= acceleration <= 255):
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        speed_bytes = [0x00, 0x00] 
        acc_byte = acceleration & 0xFF
        abs_axis_bytes = [0x00, 0x00, 0x00] 
        data_payload = speed_bytes + [acc_byte] + abs_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data_payload
        )
        
    async def save_or_clean_speed_mode_params(self, can_id: int, save: bool) -> None:
        """
        Saves or cleans the parameters in speed mode (Command 0xFF).
        Manual Ref: Page 42
        """
        action_code = const.SPEED_MODE_PARAM_SAVE if save else const.SPEED_MODE_PARAM_CLEAN
        
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS, data=[action_code], expected_dlc=3
        )
        # Note: _send_command_and_get_response already checks if response.data[0] matches
        # the expected_response_command_code. For 0xFF, this is action_code.
        if response.data[1] != const.STATUS_SUCCESS: # Status is the second byte of data field
            action_str = "Save" if save else "Clean"
            raise MotorError(
                f"{action_str} speed mode parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        logger.info(f"CAN ID {can_id}: {'Save' if save else 'Clean'} speed mode parameters successful.")
