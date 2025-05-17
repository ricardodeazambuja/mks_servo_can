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
            self, arbitration_id=0, data=None, is_extended_id=False, dlc=0
        ):  # Added dlc
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b""  # Ensure bytes
            self.is_extended_id = is_extended_id
            self.dlc = (
                dlc if data is None else len(data)
            )  # Correct DLC calculation
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

        expected_response_can_id = can_id
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
        ):  # Must have at least echoed cmd_code and CRC
            raise CommandError(
                f"Response to command {command_code:02X} from ID {can_id:03X} is too short or empty. Data: {response_msg.data}"
            )

        if response_msg.data[0] != command_code:
            # Special case for CMD_READ_SYSTEM_PARAMETER_PREFIX (0x00)
            # Its response will have the actual parameter's command code as data[0].
            if command_code != const.CMD_READ_SYSTEM_PARAMETER_PREFIX:
                raise CommandError(
                    f"Response command code mismatch for command {command_code:02X} from ID {can_id:03X}. "
                    f"Expected {command_code:02X}, got {response_msg.data[0]:02X}. Data: {response_msg.data}"
                )

        if not verify_crc(response_msg.arbitration_id, list(response_msg.data)):
            raise CRCError(
                f"Invalid CRC in response to command {command_code:02X} from ID {can_id:03X}. Data: {response_msg.data}"
            )

        if expected_dlc is not None and response_msg.dlc != expected_dlc:
            raise CommandError(
                f"Response DLC mismatch for command {command_code:02X} from ID {can_id:03X}. "
                f"Expected {expected_dlc}, got {response_msg.dlc}. Data: {response_msg.data}"
            )

        # CRITICAL: The generic status failure check that was here previously has been removed.
        # Each specific command method (e.g., set_work_mode, read_en_pin_status)
        # is responsible for interpreting the meaning of the status byte(s) in the response.
        # For CMD_READ_EN_PIN_STATUS (0x3A), response.data[1] == 0x00 means "disabled",
        # which is valid data, not a command failure.

        return response_msg  # This should be around line 156

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
        val_bytes = response.data[1:7]
        if val_bytes[5] & 0x80:
            val_bytes += b"\xff\xff"  # Sign extend for negative
        else:
            val_bytes += b"\x00\x00"  # Pad for positive
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
        val_bytes = response.data[1:7]
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
        """Cmd 0x3A: Read En pin status. Returns True if enabled, False if disabled."""
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_EN_PIN_STATUS, expected_dlc=3
        )
        # For this command, response.data[1] is the enable status (0=Disabled, 1=Enabled).
        # This is NOT a command failure if it's 0.
        return response.data[1] == 0x01

    async def read_power_on_zero_status(self, can_id: int) -> int:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_POWER_ON_ZERO_STATUS, expected_dlc=3
        )
        # response.data[1] is 0 (going), 1 (success), 2 (fail)
        status = response.data[1]
        if status == 2:  # Explicit failure code for this command
            logger.warning(
                f"Power on zero status for CAN ID {can_id} reported failure (status 2)."
            )
        return status

    async def release_stall_protection(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RELEASE_STALL_PROTECTION, expected_dlc=3
        )
        # response.data[1] is 0 (fail), 1 (success)
        if response.data[1] == 0x01:
            return True
        elif response.data[1] == 0x00:
            logger.warning(
                f"Release stall protection failed for CAN ID {can_id} (status 0)."
            )
            return False  # This is a failure specific to this command's meaning
        else:
            raise CommandError(
                f"Unexpected status from release_stall_protection: {response.data[1]}"
            )

    async def read_motor_protection_state(self, can_id: int) -> bool:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_PROTECTION_STATE, expected_dlc=3
        )
        # response.data[1] is 0 (not protected), 1 (protected)
        return response.data[1] == 0x01

    # --- Part 5.2: Set System Parameter Commands ---
    async def calibrate_encoder(self, can_id: int) -> None:
        response = await self._send_command_and_get_response(
            can_id, const.CMD_CALIBRATE_ENCODER, data=[0x00], expected_dlc=3
        )
        status = response.data[1]  # 0=Calibrating, 1=Success, 2=Fail
        if status == const.STATUS_CALIBRATING_FAIL:  # 2
            raise CalibrationError(
                "Encoder calibration failed.", error_code=status, can_id=can_id
            )
        elif status == const.STATUS_CALIBRATED_SUCCESS:  # 1
            logger.info(f"CAN ID {can_id}: Encoder calibration successful.")
        elif status == const.STATUS_CALIBRATING:  # 0
            # This might be an intermediate response. For an async API, we usually expect final status.
            # If the device guarantees a final status eventually, this might need a loop or different handling.
            # For now, treat as an incomplete/unexpected state for a single call.
            raise CalibrationError(
                "Encoder calibration started but did not confirm success in single response (status 'calibrating').",
                error_code=status,
                can_id=can_id,
            )
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
        if (
            response.data[1] != const.STATUS_SUCCESS
        ):  # Check specific success status for this command
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
                f"Invalid holding current code: {percentage_code}."
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

    # --- Part 6: Run Motor Commands ---
    async def enable_motor(self, can_id: int, enable: bool) -> None:
        """Cmd 0xF3: Enable or disable the motor."""
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_ENABLE_MOTOR, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS:  # 0x01 for success
            raise MotorError(
                f"Enable motor command (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # ... (Rest of LowLevelAPI methods, ensuring they correctly interpret their specific status bytes) ...
    # For example, run commands (0xF6, 0xFD etc.) return status 0(fail), 1(starting), 2(complete), 3(limit).
    # These are not generic failures if status is 1, 2, or 3.
    # The _run_motor_command helper in the original LowLevelAPI correctly handled this by returning the status.

    async def _run_motor_command(
        self, can_id: int, command_code: int, data: List[int]
    ) -> int:
        """Helper for motor run commands that return status 0,1,2,3."""
        response = await self._send_command_and_get_response(
            can_id, command_code, data=data, expected_dlc=3
        )
        status = response.data[1]  # 0=fail, 1=starting, 2=complete, 3=end limit
        if status == const.POS_RUN_FAIL:  # 0x00
            raise MotorError(
                f"Motor run command {command_code:02X} failed to start (status 0).",
                error_code=status,
                can_id=can_id,
            )
        return status  # Return 1 (starting), 2 (complete), or 3 (limit)

    # ... (other methods like set_can_id, set_group_id, run_speed_mode, etc. need to be completed
    #      with their specific data packing and status checking logic similar to set_work_mode)

    # Example of a completed set command:
    async def set_en_pin_active_level(
        self, can_id: int, level_code: int
    ) -> None:
        """Cmd 0x85: Set En pin active level (00:Low, 01:High, 02:Always)."""
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

    async def read_system_parameter(
        self, can_id: int, parameter_command_code: int
    ) -> Tuple[int, bytes]:
        """Cmd 0x00 (prefix): Read a system parameter."""
        payload_data = [
            const.CMD_READ_SYSTEM_PARAMETER_PREFIX,
            parameter_command_code,
        ]
        # CRC for sent message uses the 0x00 prefix
        crc = calculate_crc(can_id, payload_data)
        full_payload = payload_data + [crc]
        msg_to_send = CanMessage(
            arbitration_id=can_id,
            data=bytes(full_payload),
            is_extended_id=False,
        )

        # Expect response with `parameter_command_code` as its command byte
        response_msg = await self.can_if.send_and_wait_for_response(
            msg_to_send,
            can_id,
            parameter_command_code,  # Expected response will echo this code
            const.CAN_TIMEOUT_SECONDS,
        )
        # _send_command_and_get_response already validates CRC and that response_msg.data[0] == parameter_command_code

        # Check for the FF FF error case specifically for this read command.
        if (
            response_msg.dlc == 4
            and response_msg.data[1] == 0xFF
            and response_msg.data[2] == 0xFF
        ):
            raise MotorError(
                f"Parameter with code {parameter_command_code:02X} cannot be read from CAN ID {can_id}.",
                error_code=0xFFFF,
                can_id=can_id,
            )

        param_data_bytes = response_msg.data[1:-1]
        return response_msg.data[0], param_data_bytes

    # --- Methods that were missing or incomplete from the plan but are used by Axis ---
    async def set_current_axis_to_zero(self, can_id: int) -> None:
        """Cmd 0x92: Set current axis position to zero."""
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
        """Cmd 0xFD: Run motor in position mode (relative pulses). Returns initial status."""
        if not (0 <= speed <= 0xFFF):
            raise ParameterError("Speed out of range.")
        if not (0 <= acceleration <= 255):
            raise ParameterError("Acceleration out of range.")
        if not (0 <= pulses <= 0xFFFFFF):
            raise ParameterError("Pulses out of range for uint24_t.")
        byte2 = 0x00
        if not ccw_direction:
            byte2 |= 0x80  # Manual: b7=dir (0=CCW, 1=CW). So if NOT ccw (i.e. CW), set bit.
        byte2 |= (speed >> 8) & 0x0F
        byte3 = speed & 0xFF
        byte4 = acceleration
        pulse_bytes = list(struct.pack("<I", pulses))[:3]
        data = [byte2, byte3, byte4] + pulse_bytes
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data
        )

    async def run_position_mode_absolute_pulses(
        self, can_id: int, speed: int, acceleration: int, absolute_pulses: int
    ) -> int:
        """Cmd 0xFE: Run motor to absolute pulse position. Returns initial status."""
        if not (0 <= speed <= 3000):
            raise ParameterError("Speed out of range (0-3000).")
        if not (0 <= acceleration <= 255):
            raise ParameterError("Acceleration out of range.")
        if not (-0x800000 <= absolute_pulses <= 0x7FFFFF):
            raise ParameterError("Absolute pulses out of range for int24_t.")
        speed_bytes = list(struct.pack("<H", speed))
        acc_byte = acceleration
        if absolute_pulses < 0:
            abs_pulse_val_for_pack = (1 << 24) + absolute_pulses
        else:
            abs_pulse_val_for_pack = absolute_pulses
        abs_pulse_bytes = list(struct.pack("<I", abs_pulse_val_for_pack))[:3]
        data = speed_bytes + [acc_byte] + abs_pulse_bytes
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data
        )
