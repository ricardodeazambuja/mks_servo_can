# mks_servo_can_project/mks_servo_can_library/mks_servo_can/low_level_api.py
"""
Low-Level API for MKS Servo CAN commands.
Implements functions to construct, send, and parse responses for each
CAN command specified in the MKS SERVO42D/57D_CAN User Manual.
"""
import asyncio
import struct
import logging
from typing import Tuple, Optional, Union, Dict, Any, List

try:
    from can import Message as CanMessage
except ImportError:
    # Create a dummy CanMessage if python-can is not installed
    # This allows type hinting and basic loading without the full library.
    class CanMessage: # type: ignore
        def __init__(self, arbitration_id=0, data=None, is_extended_id=False):
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else []
            self.dlc = len(self.data)
            self.is_extended_id = is_extended_id

from .can_interface import CANInterface
from .crc import calculate_crc, verify_crc
from .exceptions import (
    CommandError, CRCError, ParameterError, MotorError, CommunicationError,
    HomingError, CalibrationError
)
from . import constants as const

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
        expected_dlc: Optional[int] = None, # Expected DLC of the response frame
        timeout: float = const.CAN_TIMEOUT_SECONDS
    ) -> CanMessage:
        """
        Helper function to send a command and wait for its specific response.

        Args:
            can_id: Target CAN ID.
            command_code: The command code byte.
            data: Optional list of data bytes for the command (excluding command_code and CRC).
            expected_dlc: Optional expected DLC for the response. If None, DLC check is skipped.
                          If provided, the response's command code byte itself is part of the DLC.
            timeout: Timeout for the operation.

        Returns:
            The received CanMessage response.

        Raises:
            ParameterError: For invalid input parameters.
            CRCError: If the response CRC is invalid.
            CommandError: If the response indicates a command failure or is malformed.
            CommunicationError: On timeout or other communication issues.
            MotorError: For specific errors reported by the motor.
        """
        if not (const.BROADCAST_ADDRESS <= can_id <= 0x7FF):
            raise ParameterError(f"Invalid CAN ID: {can_id}. Must be 0-2047 (0x7FF).")

        payload_data = [command_code]
        if data:
            payload_data.extend(data)

        # Calculate CRC. For broadcast, the CRC is still calculated with the broadcast ID (00).
        # The manual implies CRC is always (ID + byte1 + ... + byte(n)) & 0xFF.
        # If sending to broadcast ID (0x00), the CRC is calculated with 0x00 as 'ID'.
        # However, the device's actual CAN ID should be used for expecting a response if it's not a broadcast.
        # For commands sent to a specific ID, that ID is used for CRC.
        crc_id_for_calc = can_id # Use the target ID for CRC calculation of the sent packet
        
        crc = calculate_crc(crc_id_for_calc, payload_data)
        full_payload = payload_data + [crc]

        msg_to_send = CanMessage(
            arbitration_id=can_id, # The actual CAN ID to send to
            data=bytes(full_payload),
            is_extended_id=False
        )

        # If it's a broadcast or group address, motor might not respond (as per manual section 4, and 5.2.15)
        # We should not wait for a future in such cases unless explicitly stated otherwise.
        # This helper is for commands that DO expect a response.
        # Specific methods calling this should handle broadcast logic.
        if can_id == const.BROADCAST_ADDRESS or \
           (const.DEFAULT_CAN_ID < can_id < 0x50 and can_id != const.DEFAULT_CAN_ID) : # Basic check for group IDs
            # A better check would be to know if a given can_id is a configured group ID
            # For now, we assume commands to broadcast won't use this waiting function directly
            # or the calling function will handle the lack of direct response.
            # Manual: "Slave does not answer if broadcast address or group address is used."
            # So, this function should ideally not be used for broadcast or group IDs expecting no reply.
            # However, some commands might be sent to broadcast but a specific device replies with its own ID.
            # This needs careful handling per command.
            # For now, we proceed, but if no response is expected, the caller should not use this.
            pass


        # The expected response will come from the motor's actual CAN ID.
        # If we sent to a specific ID, we expect response from that ID.
        expected_response_can_id = can_id
        expected_response_command_code = command_code # Usually, response has same command code

        try:
            response_msg = await self.can_if.send_and_wait_for_response(
                msg_to_send,
                expected_response_can_id,
                expected_response_command_code, # The response usually echoes the command code
                timeout
            )
        except CommunicationError as e:
            logger.warning(f"Communication error for cmd {command_code:02X} to ID {can_id:03X}: {e}")
            raise

        # Validate response
        if not response_msg.data or len(response_msg.data) < 2: # Must have at least echoed cmd_code and CRC
            raise CommandError(f"Response to command {command_code:02X} from ID {can_id:03X} is too short or empty. Data: {response_msg.data}")

        if response_msg.data[0] != command_code:
            # Special case: Read System Parameter (0x00)
            # For command 0x00 (Read System Parameter), the response byte1 is the CODE of the parameter being read.
            # The original command_code sent was 0x00, but the data[1] was the parameter's actual code.
            # The response_msg.data[0] will be that parameter's code.
            # This needs to be handled by the calling function (e.g., read_system_parameter)
            # This generic handler assumes response command code matches sent command code.
            # If not read_system_parameter, then it's an error.
            if command_code != const.CMD_READ_SYSTEM_PARAMETER_PREFIX:
                 raise CommandError(f"Response command code mismatch for command {command_code:02X} from ID {can_id:03X}. "
                                 f"Expected {command_code:02X}, got {response_msg.data[0]:02X}. Data: {response_msg.data}")

        if not verify_crc(response_msg.arbitration_id, list(response_msg.data)):
            raise CRCError(f"Invalid CRC in response to command {command_code:02X} from ID {can_id:03X}. Data: {response_msg.data}")

        if expected_dlc is not None and response_msg.dlc != expected_dlc:
            raise CommandError(
                f"Response DLC mismatch for command {command_code:02X} from ID {can_id:03X}. "
                f"Expected {expected_dlc}, got {response_msg.dlc}. Data: {response_msg.data}"
            )
        
        # Check for generic 'status' byte if applicable (many commands have this as byte2)
        # This is a basic check; specific commands will parse more detail.
        # Typically, byte after command code. If data has cmd_code, status, crc -> len 3
        if len(response_msg.data) == 3 and response_msg.data[1] == const.STATUS_FAILURE: # Status byte index 1
            # This is a generic failure. Specific commands might provide more info.
            raise MotorError(f"Command {command_code:02X} to ID {can_id:03X} failed with status 00.",
                             error_code=const.STATUS_FAILURE, can_id=can_id)
        
        return response_msg

    async def _send_command_no_response(
        self,
        can_id: int, # This could be broadcast ID 0x00
        command_code: int,
        data: Optional[List[int]] = None,
        timeout: float = const.CAN_TIMEOUT_SECONDS
    ):
        """Sends a command that does not expect a direct, waited-for response (e.g., broadcast)."""
        if not (const.BROADCAST_ADDRESS <= can_id <= 0x7FF):
            raise ParameterError(f"Invalid CAN ID: {can_id}. Must be 0-2047 (0x7FF).")

        payload_data = [command_code]
        if data:
            payload_data.extend(data)
        
        crc_id_for_calc = can_id # Use the target ID (even if broadcast) for CRC calc of sent packet.
                                 # The manual implies CRC is always (ID + byte1 + ... + byte(n)) & 0xFF.
                                 # If ID is 00 (broadcast), CRC is calculated with 00.
        crc = calculate_crc(crc_id_for_calc, payload_data)
        full_payload = payload_data + [crc]

        msg_to_send = CanMessage(
            arbitration_id=can_id,
            data=bytes(full_payload),
            is_extended_id=False
        )
        await self.can_if.send_message(msg_to_send, timeout=timeout)


    # --- Part 5.1: Read Status Parameter Commands ---

    async def read_encoder_value_carry(self, can_id: int) -> Tuple[int, int]:
        """Cmd 0x30: Read encoder value (carry). Returns (carry, value)."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_ENCODER_CARRY, expected_dlc=8)
        # byte1=code(30), byte2-5=carry(int32_t), byte6-7=value(uint16_t), byte8=CRC
        carry = struct.unpack('<i', response.data[1:5])[0]
        value = struct.unpack('<H', response.data[5:7])[0]
        return carry, value

    async def read_encoder_value_addition(self, can_id: int) -> int:
        """Cmd 0x31: Read encoder value (addition). Returns int48_t value."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_ENCODER_ADDITION, expected_dlc=8)
        # byte1=code(31), byte2-7=value(int48_t), byte8=CRC
        # Python doesn't have int48. We'll read 6 bytes and handle sign if needed.
        # The manual shows positive examples. Let's assume it's little-endian.
        val_bytes = response.data[1:7]
        # Pad to 8 bytes for int64 struct unpacking, then check if negative
        if val_bytes[5] & 0x80: # Check sign bit of the 6th byte (MSB of int48)
            val_bytes += b'\xff\xff' # Sign extend for negative number
        else:
            val_bytes += b'\x00\x00' # Pad with zeros for positive
        value = struct.unpack('<q', val_bytes)[0] # Unpack as int64
        return value

    async def read_motor_speed_rpm(self, can_id: int) -> int:
        """Cmd 0x32: Read real-time motor speed (RPM). Returns int16_t speed."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_MOTOR_SPEED_RPM, expected_dlc=4)
        # byte1=code(32), byte2-3=speed(int16_t), byte4=CRC
        speed = struct.unpack('<h', response.data[1:3])[0]
        return speed

    async def read_pulses_received(self, can_id: int) -> int:
        """Cmd 0x33: Read number of pulses received. Returns int32_t pulses."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_PULSES_RECEIVED, expected_dlc=6)
        # byte1=code(33), byte2-5=pulses(int32_t), byte6=CRC
        pulses = struct.unpack('<i', response.data[1:5])[0]
        return pulses

    async def read_io_status(self, can_id: int) -> Dict[str, int]:
        """Cmd 0x34: Read IO Port status. Returns a dict of port states."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_IO_STATUS, expected_dlc=3)
        # byte1=code(34), byte2=status(uint8_t), byte3=CRC
        status_byte = response.data[1]
        # bit0: IN_1, bit1: IN_2, bit2: OUT_1, bit3: OUT_2 (Note: Manual has IN_1, IN_2, OUT_1, OUT_2 order reversed in some places)
        # Manual table for 0x34: bit0=IN_1, bit1=IN_2, bit2=OUT_1, bit3=OUT_2. Others reserved.
        # "status bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0"
        # "reserved OUT_2 OUT_1 IN_2 IN_1"
        # This means: bit0=IN_1, bit1=IN_2 (if present), bit2=OUT_1 (if present), bit3=OUT_2 (if present)
        # The manual table order for bits seems to be:
        # bit0: IN_1
        # bit1: IN_2
        # bit2: OUT_1
        # bit3: OUT_2
        # bit4: (reserved)
        # bit5: (reserved)
        # bit6: (reserved)
        # bit7: (reserved)
        # Note: SERVO42D has fewer IOs (IN_1 only, OUT_1 mapped to En for stall on some docs)
        # Document MKS SERVO42&57D_CAN User Manual V1.0.6.pdf, page 17, table for status:
        # bit0 = IN_1
        # bit1 = IN_2
        # bit2 = OUT_1 (0-protected, 1-unprotected)
        # bit3 = OUT_2 (reserved)
        # Other bits reserved.
        return {
            "IN_1": (status_byte >> 0) & 0x01,
            "IN_2": (status_byte >> 1) & 0x01, # Only for 57D
            "OUT_1": (status_byte >> 2) & 0x01, # Only for 57D (0-protected, 1-unprotected)
            "OUT_2": (status_byte >> 3) & 0x01, # Only for 57D (reserved)
            "raw_byte": status_byte
        }

    async def read_raw_encoder_value_addition(self, can_id: int) -> int:
        """Cmd 0x35: Read RAW encoder value (addition). Returns int48_t value. (V1.0.6)"""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_RAW_ENCODER_ADDITION, expected_dlc=8)
        val_bytes = response.data[1:7]
        if val_bytes[5] & 0x80: val_bytes += b'\xff\xff'
        else: val_bytes += b'\x00\x00'
        return struct.unpack('<q', val_bytes)[0]

    async def read_shaft_angle_error(self, can_id: int) -> int:
        """Cmd 0x39: Read motor shaft angle error. Returns int32_t error.
        0-51200 corresponds to 0-360 degrees.
        """
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_SHAFT_ANGLE_ERROR, expected_dlc=6) # Manual says DLC 6 (code, error_int32, CRC)
        # byte1=code(39), byte2-5=error(int32_t), byte6=CRC
        error_val = struct.unpack('<i', response.data[1:5])[0]
        return error_val

    async def read_en_pin_status(self, can_id: int) -> bool:
        """Cmd 0x3A: Read En pin status. Returns True if enabled, False if disabled."""
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_EN_PIN_STATUS, expected_dlc=3)
        # byte1=code(3A), byte2=enable(uint8_t), byte3=CRC. enable=1 Enabled, enable=0 Disabled.
        return response.data[1] == 0x01

    async def read_power_on_zero_status(self, can_id: int) -> int:
        """Cmd 0x3B: Read power-on go back to zero status.
        Returns: 0 (going to zero), 1 (success), 2 (fail).
        """
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_POWER_ON_ZERO_STATUS, expected_dlc=3)
        # byte1=code(3B), byte2=status(uint8_t), byte3=CRC
        return response.data[1]

    async def release_stall_protection(self, can_id: int) -> bool:
        """Cmd 0x3D: Release motor shaft locked-rotor protection state. Returns True on success."""
        response = await self._send_command_and_get_response(can_id, const.CMD_RELEASE_STALL_PROTECTION, expected_dlc=3)
        # byte1=code(3D), byte2=status(uint8_t), byte3=CRC. status=1 release success, status=0 release fail.
        if response.data[1] == 0x01:
            return True
        elif response.data[1] == 0x00:
            logger.warning(f"Release stall protection failed for CAN ID {can_id}.")
            return False
        else:
            raise CommandError(f"Unexpected status from release_stall_protection: {response.data[1]}")


    async def read_motor_protection_state(self, can_id: int) -> bool:
        """Cmd 0x3E: Read motor shaft protection state. Returns True if protected."""
        # Manual page 19: "FA 01 3E CRC" - this seems like a typo.
        # Should be "01 3E CRC" for downlink according to structure.
        # The command code is 3E.
        response = await self._send_command_and_get_response(can_id, const.CMD_READ_MOTOR_PROTECTION_STATE, expected_dlc=3)
        # byte1=code(3E), byte2=status(uint8_t), byte3=CRC. status=1 protected, status=0 no protected.
        return response.data[1] == 0x01

    # --- Part 5.2: Set System Parameter Commands ---

    async def calibrate_encoder(self, can_id: int) -> None:
        """Cmd 0x80: Calibrate encoder. Raises CalibrationError on failure."""
        # Data byte for this command is 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_CALIBRATE_ENCODER, data=[0x00], expected_dlc=3)
        # status=0 Calibrating, status=1 Success, status=2 Fail
        status = response.data[1]
        if status == const.STATUS_CALIBRATING_FAIL: # 0x02
            raise CalibrationError("Encoder calibration failed.", error_code=status, can_id=can_id)
        elif status == const.STATUS_CALIBRATED_SUCCESS: # 0x01
            logger.info(f"CAN ID {can_id}: Encoder calibration successful.")
            return
        elif status == const.STATUS_CALIBRATING: # 0x00 (This might be an intermediate response, but API expects completion)
            # This implies the command might return multiple statuses.
            # For a simple async call, we expect final status. The manual example shows only one response.
            # If it can return 0 (calibrating) then 1 (success), this API needs adjustment.
            # Assuming it waits and returns final.
            logger.info(f"CAN ID {can_id}: Encoder calibration in progress (unexpected intermediate response).")
            # We might need to wait longer or listen for a subsequent message. For now, treat as incomplete.
            raise CalibrationError("Encoder calibration started but did not confirm success in single response.", error_code=status, can_id=can_id)
        else:
            raise CommandError(f"Unexpected status from calibrate_encoder: {status}")

    async def set_work_mode(self, can_id: int, mode: int) -> None:
        """Cmd 0x82: Set work mode."""
        if not (0 <= mode <= 5):
            raise ParameterError(f"Invalid work mode: {mode}. Must be 0-5.")
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_WORK_MODE, data=[mode], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set work mode to {mode} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_working_current(self, can_id: int, current_ma: int) -> None:
        """Cmd 0x83: Set working current (mA)."""
        # SERVO42D/28D/35D: Max 3000mA, SERVO57D: Max 5200mA
        # Add checks based on motor type if known, otherwise use a general reasonable max
        if not (0 <= current_ma <= 5200): # General max for 57D
            raise ParameterError(f"Invalid working current: {current_ma}mA. Must be 0-5200mA (check motor type).")
        current_bytes = list(struct.pack('<H', current_ma)) # uint16_t
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_WORKING_CURRENT, data=current_bytes, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set working current to {current_ma}mA failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_holding_current_percentage(self, can_id: int, percentage_code: int) -> None:
        """Cmd 0x9B: Set holding current percentage (0-8 for 10%-90%). (V1.0.4)"""
        if not (0x00 <= percentage_code <= 0x08): # 00 = 10%, 01 = 20%, ..., 08 = 90%
            raise ParameterError(f"Invalid holding current percentage code: {percentage_code}. Must be 0-8.")
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_HOLDING_CURRENT_PERCENTAGE, data=[percentage_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set holding current percentage to code {percentage_code} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_subdivision(self, can_id: int, microsteps: int) -> None:
        """Cmd 0x84: Set subdivision (microsteps)."""
        if not (1 <= microsteps <= 255): # 00 to FF in menu, but usually 1-256. Manual cmd data is 0-255.
            raise ParameterError(f"Invalid microsteps value: {microsteps}. Must be 1-255.")
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_SUBDIVISION, data=[microsteps], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set subdivision to {microsteps} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_en_pin_active_level(self, can_id: int, level_code: int) -> None:
        """Cmd 0x85: Set En pin active level (00:Low, 01:High, 02:Always)."""
        if level_code not in [const.EN_ACTIVE_LOW, const.EN_ACTIVE_HIGH, const.EN_ACTIVE_ALWAYS]:
            raise ParameterError(f"Invalid En pin active level code: {level_code}.")
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_EN_PIN_ACTIVE_LEVEL, data=[level_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set En pin active level to {level_code} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_motor_direction(self, can_id: int, direction_code: int) -> None:
        """Cmd 0x86: Set motor rotation direction (00:CW, 01:CCW). (Pulse interface only)"""
        if direction_code not in [const.DIR_CW, const.DIR_CCW]:
            raise ParameterError(f"Invalid motor direction code: {direction_code}.")
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_MOTOR_DIRECTION, data=[direction_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set motor direction to {direction_code} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_auto_screen_off(self, can_id: int, enable: bool) -> None:
        """Cmd 0x87: Set auto screen off function."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_AUTO_SCREEN_OFF, data=[enable_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set auto screen off to {enable} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_stall_protection(self, can_id: int, enable: bool) -> None:
        """Cmd 0x88: Set motor shaft locked-rotor protection function."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_STALL_PROTECTION, data=[enable_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set stall protection to {enable} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_subdivision_interpolation(self, can_id: int, enable: bool) -> None:
        """Cmd 0x89: Set subdivision interpolation function."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_SUBDIVISION_INTERPOLATION, data=[enable_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set subdivision interpolation to {enable} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_can_bitrate(self, can_id: int, bitrate_code: int) -> None:
        """Cmd 0x8A: Set CAN bitrate (00:125K, 01:250K, 02:500K, 03:1M)."""
        if bitrate_code not in [const.CAN_BITRATE_125K, const.CAN_BITRATE_250K, const.CAN_BITRATE_500K, const.CAN_BITRATE_1M]:
            raise ParameterError(f"Invalid CAN bitrate code: {bitrate_code}.")
        # This command will likely cause the device to restart or change comms.
        # A response might not be received at the current bitrate or at all.
        # Consider sending without expecting a response, or handling timeout gracefully.
        try:
            response = await self._send_command_and_get_response(
                can_id, const.CMD_SET_CAN_BITRATE, data=[bitrate_code], expected_dlc=3, timeout=1.0 # Shorter timeout
            )
            if response.data[1] != const.STATUS_SUCCESS:
                raise MotorError(f"Set CAN bitrate to code {bitrate_code} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)
        except CommunicationError:
            logger.warning(f"Timeout or communication error after setting CAN bitrate for ID {can_id}. This may be expected if bitrate changed.")
            # Assume success if no immediate error from device, but connection may need re-init

    async def set_can_id(self, can_id: int, new_can_id: int) -> None:
        """Cmd 0x8B: Set CAN ID (0x00-0x7FF)."""
        if not (const.BROADCAST_ADDRESS <= new_can_id <= 0x7FF): # 0 is broadcast, 1-0x7FF are valid IDs
             raise ParameterError(f"Invalid new CAN ID: {new_can_id}. Must be 0-2047.")
        # Data is ID(00~7FF) -> uint16_t technically, but CAN ID is 11-bit.
        # Manual shows byte2-3 for ID. Assume it's little endian short for the data payload.
        id_bytes = list(struct.pack('<H', new_can_id))[:2] # Take first two bytes, as CAN ID fits
                                                           # Careful if new_can_id > 255, this might be just one byte in some interpretations
                                                           # The manual example (page 25) is `ID(00~7FF)`.
                                                           # It implies up to 0x7FF, fitting in 11 bits.
                                                           # DLC is 4 (code, ID_byte1, ID_byte2, CRC)
                                                           # This means new_can_id is split into two bytes for the command data field
        # The command data for ID (byte2 byte3) is likely new_can_id itself, fitting into two bytes if needed
        # Let's assume the "ID(00~7FF)" refers to the value, and it's packed as uint16_t for data.
        new_id_data = list(struct.pack('<H', new_can_id)) # Pack as uint16_t, send 2 bytes
        
        # Important: After this command, the device will respond with its OLD ID, then change.
        # Or it might change then respond with NEW ID. The manual is not explicit.
        # "the new address will show in the screen"
        # For safety, expect response from OLD ID.
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_CAN_ID, data=new_id_data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set CAN ID to {new_can_id} failed for current ID {can_id}.", error_code=response.data[1], can_id=can_id)
        logger.info(f"CAN ID for device formerly {can_id} changed to {new_can_id}. Re-initialize communication for this device if needed.")

    async def set_slave_respond_active(self, can_id: int, respond_enabled: bool, active_enabled: bool) -> None:
        """Cmd 0x8C: Set slave respond and active behavior."""
        respon_code = 0x01 if respond_enabled else 0x00
        active_code = 0x01 if active_enabled else 0x00
        data = [respon_code, active_code]
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_SLAVE_RESPOND_ACTIVE, data=data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set slave respond/active failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_group_id(self, can_id: int, group_id: int) -> None:
        """Cmd 0x8D: Set group ID (0x01-0x7FF)."""
        # Group ID 0 is not sensible as it's broadcast. Should be 1-0x7FF.
        if not (0x01 <= group_id <= 0x7FF):
            raise ParameterError(f"Invalid group ID: {group_id}. Must be 1-2047.")
        # Data format: ID(01~0x7FF) as byte2-3.
        group_id_data = list(struct.pack('<H', group_id)) # uint16_t
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_GROUP_ID, data=group_id_data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set group ID to {group_id} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_key_lock(self, can_id: int, lock: bool) -> None:
        """Cmd 0x8F: Set key lock or unlock."""
        lock_code = 0x01 if lock else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_KEY_LOCK, data=[lock_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set key lock to {lock} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    # --- Part 5.3: Write IO Port Command ---
    async def write_io_port(self, can_id: int, out1_value: Optional[int] = None, out2_value: Optional[int] = None) -> None:
        """
        Cmd 0x36: Write IO port (V1.0.6).
        Args:
            can_id: The CAN ID of the motor.
            out1_value: Value for OUT_1 (0 or 1). If None, OUT_1 is not changed.
            out2_value: Value for OUT_2 (0 or 1). If None, OUT_2 is not changed.
        """
        # byte2 structure:
        # bit 7: OUT2_mask (0: don't write, 1: write, 2: unchanged - this is strange, typically 0/1 for mask)
        # Manual says "2: OUT_2 IO port value remains unchanged" - This means mask bit could be 2 bits.
        # Let's assume:
        #   if value is None, mask=0 (do not write, effectively). Or "unchanged" implies a specific mask value.
        #   The manual table for the command:
        #   OUT2_mask (bit7-6), OUT1_mask (bit5-4), OUT_2 (bit3), OUT_1 (bit2), reserved (bit1-0)
        #   Mask values: 0: Do not write, 1: Write value, 2: Value remains unchanged.
        #   This structure is complex. Let's simplify based on common usage: either write or don't.
        #   If user provides a value, we want to write it (mask=1). If None, we want it unchanged (mask=2).

        # Re-interpreting based on the "Byte2" table in manual section 5.3 (page 28 of V1.0.6):
        # byte2: [OUT2_mask_b1, OUT2_mask_b0, OUT1_mask_b1, OUT1_mask_b0, OUT_2_val, OUT_1_val, res, res]
        # This is more plausible.
        # OUT2_mask: bit7-6
        # OUT1_mask: bit5-4
        # OUT_2 value: bit3
        # OUT_1 value: bit2
        # bit1, bit0: reserved (set to 0)

        byte2 = 0b00000000

        if out2_value is not None:
            byte2 |= (0b01 << 6) # OUT2_mask = Write value
            if out2_value == 1:
                byte2 |= (1 << 3) # OUT_2 value = 1
        else:
            byte2 |= (0b10 << 6) # OUT2_mask = Unchanged

        if out1_value is not None:
            byte2 |= (0b01 << 4) # OUT1_mask = Write value
            if out1_value == 1:
                byte2 |= (1 << 2) # OUT_1 value = 1
        else:
            byte2 |= (0b10 << 4) # OUT1_mask = Unchanged

        response = await self._send_command_and_get_response(can_id, const.CMD_WRITE_IO_PORT, data=[byte2], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Write IO port failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)


    # --- Part 5.4: Set Home Command ---
    async def set_home_parameters(self, can_id: int, home_trig_low: bool, home_dir_ccw: bool,
                                  home_speed_rpm: int, end_limit_enabled: bool, no_limit_home_mode: bool) -> None:
        """Cmd 0x90: Set parameters for homing."""
        level = 0x00 if home_trig_low else 0x01  # 0:Low, 1:High
        direction = 0x01 if home_dir_ccw else 0x00 # 0:CW, 1:CCW
        if not (0 <= home_speed_rpm <= 3000):
            raise ParameterError("Home speed RPM out of range (0-3000).")
        speed_bytes = list(struct.pack('<H', home_speed_rpm)) # uint16_t
        
        endlimit_byte = 0x01 if end_limit_enabled else 0x00
        hm_mode_byte = 0x01 if no_limit_home_mode else 0x00 # 0: use limit, 1: no limit

        # byte2:level, byte3:dir, byte4-5:speed, byte6:EndLimit, byte7:hmMode
        data = [level, direction] + speed_bytes + [endlimit_byte, hm_mode_byte]
        
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_HOME_PARAMETERS, data=data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set home parameters failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def go_home(self, can_id: int) -> None:
        """Cmd 0x91: Perform G0 Homing sequence. Raises HomingError on failure."""
        response = await self._send_command_and_get_response(can_id, const.CMD_GO_HOME, expected_dlc=3)
        # status=0 fail, status=1 start, status=2 success
        status = response.data[1]
        if status == const.HOME_FAIL: # 0x00
            raise HomingError("Go home command failed to start or completed with failure.", error_code=status, can_id=can_id)
        elif status == const.HOME_START: # 0x01
            logger.info(f"CAN ID {can_id}: Homing sequence started.")
            # This implies we need to wait for another message (status 2) or query.
            # For now, we treat 'start' as the command being accepted.
            # A higher-level API would manage polling for completion.
            pass # Command accepted
        elif status == const.HOME_SUCCESS: # 0x02 (This might be a response after completion)
            logger.info(f"CAN ID {can_id}: Homing sequence successful (immediate response).")
            pass
        else:
            raise CommandError(f"Unexpected status from go_home: {status}")

    async def set_current_axis_to_zero(self, can_id: int) -> None:
        """Cmd 0x92: Set current axis position to zero."""
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_CURRENT_AXIS_TO_ZERO, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set current axis to zero failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_nolimit_home_parameters(self, can_id: int, reverse_angle_pulses: int, homing_current_ma: int) -> None:
        """Cmd 0x94: Set parameters for 'noLimit' go home (V1.0.5)."""
        # retValue: 0~0xFFFFFFFF (reverse angle in pulses, e.g., 0x4000 for 360 deg)
        # ma: current for noLimit home (uint16_t)
        if not (0 <= reverse_angle_pulses <= 0xFFFFFFFF):
            raise ParameterError("Reverse angle pulses out of range (0-0xFFFFFFFF).")
        if not (0 <= homing_current_ma <= 5200): # Max current for 57D
            raise ParameterError("Homing current (noLimit) out of range.")

        ret_val_bytes = list(struct.pack('<I', reverse_angle_pulses)) # uint32_t
        ma_bytes = list(struct.pack('<H', homing_current_ma)) # uint16_t
        
        # Data: byte2-5=retValue, byte6-7=ma
        data = ret_val_bytes + ma_bytes
        
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_NOLIMIT_HOME_PARAMS, data=data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set noLimit home parameters failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def set_limit_port_remap(self, can_id: int, enable: bool) -> None:
        """Cmd 0x9E: Set limit port remap (V1.0.4)."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_LIMIT_PORT_REMAP, data=[enable_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set limit port remap to {enable} failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    # --- Part 5.5: Set 0_Mode Command ---
    async def set_zero_mode_parameters(self, can_id: int, mode: int, set_zero_action: int, speed_code: int, dir_ccw: bool) -> None:
        """Cmd 0x9A: Set parameters for 0_Mode (power-on auto-zeroing)."""
        # mode: 0=Disable, 1=DirMode, 2=NearMode
        # enable (set_zero_action): 0=clean zero, 1=set current as zero, 2=not modify
        # speed: 0-4 (slowest-fastest)
        # dir: 0=CW, 1=CCW
        if mode not in [const.OMODE_DISABLE, const.OMODE_DIR_MODE, const.OMODE_NEAR_MODE]:
            raise ParameterError(f"Invalid 0_Mode mode: {mode}")
        if set_zero_action not in [const.OMODE_SET_ZERO_CLEAN, const.OMODE_SET_ZERO_SET, const.OMODE_SET_ZERO_NO_MODIFY]:
            raise ParameterError(f"Invalid 0_Mode set_zero_action: {set_zero_action}")
        if not (0 <= speed_code <= 4):
            raise ParameterError(f"Invalid 0_Mode speed_code: {speed_code}")
        
        dir_code = const.OMODE_DIR_CCW if dir_ccw else const.OMODE_DIR_CW
        
        # Data: byte2=mode, byte3=enable(set_zero_action), byte4=speed, byte5=dir
        data = [mode, set_zero_action, speed_code, dir_code]
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_ZERO_MODE_PARAMETERS, data=data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set 0_Mode parameters failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    # --- Part 5.6: Restore Default Parameters ---
    async def restore_default_parameters(self, can_id: int) -> None:
        """Cmd 0x3F: Restore default parameters."""
        # This command will cause a reboot. Expect potential communication disruption.
        try:
            response = await self._send_command_and_get_response(can_id, const.CMD_RESTORE_DEFAULT_PARAMETERS, expected_dlc=3, timeout=2.0) # Longer timeout
            if response.data[1] != const.STATUS_SUCCESS:
                 raise MotorError(f"Restore default parameters failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)
        except CommunicationError:
            logger.warning(f"Timeout or communication error after restoring defaults for ID {can_id}. This may be expected due to reboot.")
            # Assume success if no immediate error from device, but re-init needed.

    # --- Part 5.7: Restart Motor ---
    async def restart_motor(self, can_id: int) -> None:
        """Cmd 0x41: Restart the motor (V1.0.5)."""
        # This command will cause a reboot.
        try:
            response = await self._send_command_and_get_response(can_id, const.CMD_RESTART_MOTOR, expected_dlc=3, timeout=2.0)
            if response.data[1] != const.STATUS_SUCCESS:
                raise MotorError(f"Restart motor failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)
        except CommunicationError:
            logger.warning(f"Timeout or communication error after restarting motor ID {can_id}. This may be expected due to reboot.")

    # --- Part 5.8: En Triggers and Position Error Protection ---
    async def set_en_trigger_pos_error_protection(self, can_id: int, enable_en_trigger_zero: bool,
                                                 enable_pos_error_protection: bool,
                                                 error_time_units: int, error_threshold_pulses: int) -> None:
        """Cmd 0x9D: Set En trigger zero return and position error protection (V1.0.6)."""
        # byte2: b7-b2=0, b1=g0En, b0=pEn
        # g0En (enable_en_trigger_zero): 0=Disable, 1=Enable
        # pEn (enable_pos_error_protection): 0=Disable, 1=Enable
        # byte3-4: Tim (uint16_t, error_time_units, 1 unit = ~15ms)
        # byte5-6: Errors (uint16_t, error_threshold_pulses)
        
        byte2 = 0x00
        if enable_en_trigger_zero: byte2 |= 0x02 # bit1
        if enable_pos_error_protection: byte2 |= 0x01 # bit0

        if not (0 <= error_time_units <= 0xFFFF):
            raise ParameterError("Error time units out of range (0-65535).")
        if not (0 <= error_threshold_pulses <= 0xFFFF): # Example shows 28000 for 360deg
            raise ParameterError("Error threshold pulses out of range (0-65535).")

        tim_bytes = list(struct.pack('<H', error_time_units))
        errors_bytes = list(struct.pack('<H', error_threshold_pulses))

        data = [byte2] + tim_bytes + errors_bytes
        response = await self._send_command_and_get_response(can_id, const.CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION, data=data, expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Set En trigger/pos error protection failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    # --- Part 5.9: Read System Parameter Command ---
    async def read_system_parameter(self, can_id: int, parameter_command_code: int) -> Tuple[int, bytes]:
        """
        Cmd 0x00 (prefix): Read a system parameter.
        Args:
            can_id: The CAN ID of the motor.
            parameter_command_code: The command code of the parameter to read (e.g., 0x82 for work mode).
        Returns:
            A tuple (actual_param_code_echoed, parameter_data_bytes).
            parameter_data_bytes excludes the echoed code and CRC.
        Raises:
            MotorError if parameter cannot be read (FF FF response).
        """
        # Downlink: ID, DLC=3, byte1=0x00, byte2=parameter_command_code, byte3=CRC
        # Uplink: ID, DLC=n+1, byte1=parameter_command_code_echoed, byte2-n=params, byte(n+1)=CRC
        # Or if not readable: ID, DLC=4, byte1=param_code_echoed, byte2-3=FFFF, byte4=CRC

        # Note: The _send_command_and_get_response needs to know the expected response command code.
        # Here, the response's command code will be `parameter_command_code`, not `0x00`.
        
        # We need a slightly different send/receive logic for this one due to command code in response.
        payload_data = [const.CMD_READ_SYSTEM_PARAMETER_PREFIX, parameter_command_code]
        crc = calculate_crc(can_id, payload_data)
        full_payload = payload_data + [crc]

        msg_to_send = CanMessage(arbitration_id=can_id, data=bytes(full_payload), is_extended_id=False)

        # Expect response with `parameter_command_code` as its command byte
        response_msg = await self.can_if.send_and_wait_for_response(
            msg_to_send,
            can_id, # Expected response from this ID
            parameter_command_code, # Expected response will echo this code
            const.CAN_TIMEOUT_SECONDS
        )

        # Basic validation (already done by send_and_wait_for_response regarding CRC and echoed command code)
        # Now check for the FF FF error case specifically for this read command.
        # Response structure: param_code_echoed, data..., CRC
        
        if response_msg.data[0] != parameter_command_code:
             raise CommandError(f"Read system parameter response code mismatch. Expected {parameter_command_code:02X}, got {response_msg.data[0]:02X}.")


        # If DLC is 4 and data is [param_code, 0xFF, 0xFF, CRC]
        if response_msg.dlc == 4 and response_msg.data[1] == 0xFF and response_msg.data[2] == 0xFF:
            raise MotorError(f"Parameter with code {parameter_command_code:02X} cannot be read from CAN ID {can_id}.",
                             error_code=0xFFFF, can_id=can_id)

        # Return the echoed parameter code and the actual parameter data (excluding code and CRC)
        param_data_bytes = response_msg.data[1:-1] # data between echoed code and CRC
        return response_msg.data[0], param_data_bytes


    # --- Part 6: Run the Motor by CAN Command ---
    # Note: These commands require serial work mode (SR_OPEN, SR_CLOSE, SR_VFOC)

    async def query_motor_status(self, can_id: int) -> int:
        """Cmd 0xF1: Query motor status."""
        response = await self._send_command_and_get_response(can_id, const.CMD_QUERY_MOTOR_STATUS, expected_dlc=3)
        # status=0 fail, 1 stop, 2 speed up, 3 speed down, 4 full speed, 5 homing, 6 Cal...
        return response.data[1]

    async def enable_motor(self, can_id: int, enable: bool) -> None:
        """Cmd 0xF3: Enable or disable the motor."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(can_id, const.CMD_ENABLE_MOTOR, data=[enable_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Enable motor ({enable}) failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def emergency_stop(self, can_id: int) -> None:
        """Cmd 0xF7: Emergency stop the motor (V1.0.4)."""
        # No data bytes for this command, just command code and CRC
        response = await self._send_command_and_get_response(can_id, const.CMD_EMERGENCY_STOP, expected_dlc=3)
        # status=0 stop fail, status=1 stop success
        if response.data[1] != const.STATUS_SUCCESS: # Assuming 1 is success
            raise MotorError(f"Emergency stop failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def _run_motor_command(self, can_id: int, command_code: int, data: List[int]) -> int:
        """Helper for motor run commands that return status 0,1,2,3."""
        # Most run commands might have an initial "accepted" response, and then a "completed" response.
        # The CanRSP and Active settings (0x8C) control this.
        # Default (respon=1, active=1): Slave returns start status (0 or 1), then completion status (2 or 3).
        # This low-level API will return the *first* status received.
        # Higher level API should manage waiting for completion if active=1.
        
        response = await self._send_command_and_get_response(can_id, command_code, data=data, expected_dlc=3)
        # status=0 fail, 1 starting, 2 complete, 3 end limit
        status = response.data[1]
        if status == const.POS_RUN_FAIL: # 0x00
            raise MotorError(f"Motor run command {command_code:02X} failed to start.", error_code=status, can_id=can_id)
        return status # Return 1 (starting), 2 (complete), or 3 (limit)

    async def run_speed_mode(self, can_id: int, ccw_direction: bool, speed: int, acceleration: int) -> int:
        """Cmd 0xF6: Run motor in speed mode. Returns initial status."""
        # byte2: b7=dir, b6-b4=Rev(0), b3-b0=speed_high_nibble
        # byte3: speed_low_byte
        # byte4: acc
        # speed: 0-3000. Fits in 12 bits. (speed_high_nibble << 8) | speed_low_byte
        if not (0 <= speed <= 0xFFF): # Max speed is 3000, fits in 12 bits.
            raise ParameterError("Speed out of range 0-3000 (0xFFF for 12-bit field).")
        if not (0 <= acceleration <= 255):
            raise ParameterError("Acceleration out of range 0-255.")

        byte2 = 0x00
        if ccw_direction: byte2 |= 0x80 # bit 7 for direction
        byte2 |= ((speed >> 8) & 0x0F) # speed high nibble in bits 3-0

        byte3 = speed & 0xFF # speed low byte
        byte4 = acceleration
        
        data = [byte2, byte3, byte4]
        return await self._run_motor_command(can_id, const.CMD_RUN_SPEED_MODE, data)

    async def stop_speed_mode(self, can_id: int, acceleration: int) -> int:
        """Cmd 0xF6 (with speed=0): Stop motor in speed mode. Returns initial status."""
        # To stop, send speed = 0. Direction bit and Rev are 0.
        if not (0 <= acceleration <= 255):
            raise ParameterError("Acceleration out of range 0-255.")
        byte2 = 0x00 # dir=0, speed_high=0
        byte3 = 0x00 # speed_low=0
        byte4 = acceleration
        data = [byte2, byte3, byte4]
        return await self._run_motor_command(can_id, const.CMD_RUN_SPEED_MODE, data)


    async def save_or_clean_speed_mode_params(self, can_id: int, save: bool) -> None:
        """Cmd 0xFF: Save or Clean speed mode parameters. """
        # state = C8 for Save, CA for Clean
        state_code = 0xC8 if save else 0xCA
        response = await self._send_command_and_get_response(can_id, const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS, data=[state_code], expected_dlc=3)
        if response.data[1] != const.STATUS_SUCCESS:
            action = "Save" if save else "Clean"
            raise MotorError(f"{action} speed mode params failed for CAN ID {can_id}.", error_code=response.data[1], can_id=can_id)

    async def run_position_mode_relative_pulses(self, can_id: int, ccw_direction: bool, speed: int,
                                                acceleration: int, pulses: int) -> int:
        """Cmd 0xFD: Run motor in position mode (relative pulses). Returns initial status."""
        # byte2: dir, speed_high
        # byte3: speed_low
        # byte4: acc
        # byte5-7: pulses (uint24_t)
        if not (0 <= speed <= 0xFFF): raise ParameterError("Speed out of range.")
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        if not (0 <= pulses <= 0xFFFFFF): raise ParameterError("Pulses out of range for uint24_t.")

        byte2 = 0x00
        if ccw_direction: byte2 |= 0x80
        byte2 |= ((speed >> 8) & 0x0F)
        byte3 = speed & 0xFF
        byte4 = acceleration
        pulse_bytes = list(struct.pack('<I', pulses))[:3] # uint24_t, little endian (use LSB of uint32)

        data = [byte2, byte3, byte4] + pulse_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data)

    async def stop_position_mode_relative_pulses(self, can_id: int, acceleration: int) -> int:
        """Cmd 0xFD (with pulses=0, speed=0): Stop motor in position mode. Returns initial status."""
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        byte2 = 0x00 # dir=0, speed_high=0
        byte3 = 0x00 # speed_low=0
        byte4 = acceleration
        pulse_bytes = [0,0,0] # pulses = 0
        data = [byte2, byte3, byte4] + pulse_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data)

    async def run_position_mode_absolute_pulses(self, can_id: int, speed: int,
                                                acceleration: int, absolute_pulses: int) -> int:
        """Cmd 0xFE: Run motor to absolute pulse position (V1.0.4). Returns initial status."""
        # byte2-3: speed (uint16_t)
        # byte4: acc (uint8_t)
        # byte5-7: absPulses (int24_t)
        if not (0 <= speed <= 3000): raise ParameterError("Speed out of range (0-3000).") # Manual says 0-3000 for speed param
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        
        # absPulses: int24_t (-8388607 to +8388607)
        if not (-0x800000 <= absolute_pulses <= 0x7FFFFF):
            raise ParameterError("Absolute pulses out of range for int24_t.")

        speed_bytes = list(struct.pack('<H', speed))
        acc_byte = acceleration
        
        # Pack int24_t: use struct for signed int, then take relevant 3 bytes
        # Python struct doesn't have 24-bit. Pack as 32-bit signed int, ensure it fits, then take LSB.
        if absolute_pulses < 0:
            abs_pulse_val_for_pack = (1 << 24) + absolute_pulses # 2's complement for 24-bit
        else:
            abs_pulse_val_for_pack = absolute_pulses
        abs_pulse_bytes = list(struct.pack('<I', abs_pulse_val_for_pack))[:3]


        data = speed_bytes + [acc_byte] + abs_pulse_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data)

    async def stop_position_mode_absolute_pulses(self, can_id: int, acceleration: int) -> int:
        """Cmd 0xFE (with speed=0, pulses=0): Stop motor in absolute pulse position mode. Returns initial status."""
        # To stop: speed=0, acc=deceleration, absolute axis=0 (or current axis?)
        # Manual "Stop motor in position mode3" (typo for mode2/FE?) says speed=0, acc=acc, absolute axis=0
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        speed_bytes = list(struct.pack('<H', 0)) # speed = 0
        acc_byte = acceleration
        abs_pulse_bytes = [0,0,0] # pulses = 0
        data = speed_bytes + [acc_byte] + abs_pulse_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data)


    async def run_position_mode_relative_axis(self, can_id: int, speed: int,
                                              acceleration: int, relative_axis: int) -> int:
        """Cmd 0xF4: Run motor relative to current axis (encoder value) (V1.0.5). Returns initial status."""
        # byte2-3: speed (uint16_t)
        # byte4: acc (uint8_t)
        # byte5-7: relAxis (int24_t)
        if not (0 <= speed <= 3000): raise ParameterError("Speed out of range.")
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        if not (-0x800000 <= relative_axis <= 0x7FFFFF): raise ParameterError("Relative axis out of range for int24_t.")

        speed_bytes = list(struct.pack('<H', speed))
        acc_byte = acceleration
        if relative_axis < 0: rel_axis_val_for_pack = (1 << 24) + relative_axis
        else: rel_axis_val_for_pack = relative_axis
        rel_axis_bytes = list(struct.pack('<I', rel_axis_val_for_pack))[:3]
        
        data = speed_bytes + [acc_byte] + rel_axis_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data)

    async def stop_position_mode_relative_axis(self, can_id: int, acceleration: int) -> int:
        """Cmd 0xF4 (speed=0, relAxis=0): Stop motor in relative axis mode. Returns initial status."""
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        speed_bytes = list(struct.pack('<H', 0))
        acc_byte = acceleration
        rel_axis_bytes = [0,0,0]
        data = speed_bytes + [acc_byte] + rel_axis_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data)

    async def run_position_mode_absolute_axis(self, can_id: int, speed: int,
                                              acceleration: int, absolute_axis: int) -> int:
        """Cmd 0xF5: Run motor to absolute axis (encoder value) (V1.0.5). Returns initial status."""
        # byte2-3: speed (uint16_t)
        # byte4: acc (uint8_t)
        # byte5-7: absAxis (int24_t)
        # Supports real-time updates.
        if not (0 <= speed <= 3000): raise ParameterError("Speed out of range.")
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        if not (-0x800000 <= absolute_axis <= 0x7FFFFF): raise ParameterError("Absolute axis out of range for int24_t.")

        speed_bytes = list(struct.pack('<H', speed))
        acc_byte = acceleration
        if absolute_axis < 0: abs_axis_val_for_pack = (1 << 24) + absolute_axis
        else: abs_axis_val_for_pack = absolute_axis
        abs_axis_bytes = list(struct.pack('<I', abs_axis_val_for_pack))[:3]
        
        data = speed_bytes + [acc_byte] + abs_axis_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data)

    async def stop_position_mode_absolute_axis(self, can_id: int, acceleration: int) -> int:
        """Cmd 0xF5 (speed=0, absAxis=0): Stop motor in absolute axis mode. Returns initial status."""
        if not (0 <= acceleration <= 255): raise ParameterError("Acceleration out of range.")
        speed_bytes = list(struct.pack('<H', 0))
        acc_byte = acceleration
        abs_axis_bytes = [0,0,0] # Target axis 0 for stop
        data = speed_bytes + [acc_byte] + abs_axis_bytes
        return await self._run_motor_command(can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data)

    # ... Implement other commands from the manual as needed ...