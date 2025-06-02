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
        """
        A dummy class representing a CAN message, used when python-can is not installed.
        This allows the library to be imported and type-checked even if the full
        CAN hardware support is unavailable.
        """
        def __init__(
            self, arbitration_id=0, data=None, is_extended_id=False, dlc=0 # type: ignore
        ):
            """
            Initializes a dummy CanMessage object.

            Args:
                arbitration_id: The CAN message arbitration ID.
                data: The data payload of the message. Defaults to empty bytes.
                is_extended_id: Whether the message uses an extended ID.
                dlc: Data Length Code. Calculated if not provided.
            """
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
        """
        Initializes the LowLevelAPI with a CANInterface instance.

        Args:
            can_interface: An instance of CANInterface for sending/receiving messages.
        """
        self.can_if = can_interface

    async def _send_command_and_get_response(
        self,
        can_id: int,
        command_code: int,
        data: Optional[List[int]] = None,
        expected_dlc: Optional[int] = None,
        timeout: float = const.CAN_TIMEOUT_SECONDS,
    ) -> CanMessage:
        """
        Sends a command to the motor and waits for a response.

        This method constructs a CAN message with the given command code and data,
        calculates the CRC, sends it, and then waits for a matching response
        from the motor. It validates the response's command code echo, CRC,
        and optionally the DLC.

        Args:
            can_id: The CAN ID of the target motor (0-2047).
            command_code: The command code byte.
            data: An optional list of integer data bytes to send with the command.
            expected_dlc: An optional integer specifying the expected Data Length Code
                          of the response message. If provided, the response DLC
                          will be validated against this value.
            timeout: Timeout in seconds for waiting for the response.

        Returns:
            A CanMessage object representing the valid response from the motor.

        Raises:
            ParameterError: If the provided CAN ID is invalid.
            CommunicationError: If a timeout occurs while waiting for the response,
                                or other communication issues arise.
            CommandError: If the response is malformed (e.g., too short, command code
                          mismatch, DLC mismatch).
            CRCError: If the CRC of the received response message is invalid.
        """
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
        # For CMD_READ_SYSTEM_PARAMETER_PREFIX (0x00), the motor echoes the parameter code being read.
        if command_code == const.CMD_READ_SYSTEM_PARAMETER_PREFIX and data:
            expected_response_command_code = data[0]
        # For most commands, the motor echoes the original command code.
        # For CMD_SAVE_CLEAN_SPEED_MODE_PARAMS (0xFF), the motor also echoes 0xFF. [MKS Servo42D CAN Manual] (Page 42, Uplink frame table shows byte1 (code) = FF)
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
        """
        Sends a command to the motor without waiting for or processing a specific response.

        This is typically used for commands where no direct acknowledgment is expected
        or when sending broadcast messages.

        Args:
            can_id: The CAN ID of the target motor or broadcast address (0-2047).
            command_code: The command code byte.
            data: An optional list of integer data bytes to send with the command.
            timeout: Timeout in seconds for the send operation itself (if supported
                     by the underlying CAN interface).

        Raises:
            ParameterError: If the provided CAN ID is invalid.
            CANError: If an error occurs during the CAN send operation.
        """
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
        """
        Reads the encoder value with carry (Command 0x30).

        This command returns the encoder's carry count and the current value within
        a single revolution range (0-0x3FFF).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            A tuple (carry, value), where 'carry' is the int32_t carry value
            and 'value' is the uint16_t current encoder value (0-16383).

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_ENCODER_CARRY, expected_dlc=8
        )
        # Manual indicates big-endian for multi-byte data. [MKS Servo42D CAN Manual] (Page 15, example data for 0x30)
        carry = struct.unpack(">i", response.data[1:5])[0]
        value = struct.unpack(">H", response.data[5:7])[0]
        return carry, value

    async def read_encoder_value_addition(self, can_id: int) -> int:
        """
        Reads the accumulated encoder value (addition) (Command 0x31).

        This command returns the total accumulated encoder pulses as a signed
        48-bit integer (transmitted as 6 bytes, MSB first).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The accumulated encoder value as an integer.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_ENCODER_ADDITION, expected_dlc=8
        )
        # Data is int48_t, transmitted as 6 bytes, MSB first. [MKS Servo42D CAN Manual] (Page 16)
        raw_bytes = response.data[1:7] # 6 bytes, MSB first
        # Perform sign extension for the 48-bit value to 64-bit for Python's struct.unpack
        # Check sign bit of the MSB of the 48-bit number (which is the MSB of raw_bytes[0])
        if raw_bytes[0] & 0x80:
            # Prepend 0xFFFF for negative numbers
            padded_bytes = b"\xff\xff" + raw_bytes
        else:
            # Prepend 0x0000 for positive numbers
            padded_bytes = b"\x00\x00" + raw_bytes
        # Unpack as big-endian 64-bit signed integer
        return struct.unpack(">q", padded_bytes)[0]

    async def read_motor_speed_rpm(self, can_id: int) -> int:
        """
        Reads the real-time speed of the motor in RPM (Command 0x32).

        The speed is returned as a signed 16-bit integer. Positive for CCW,
        negative for CW rotation.

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The motor speed in RPM as an integer.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_SPEED_RPM, expected_dlc=4
        )
        # Manual indicates big-endian for multi-byte data. [MKS Servo42D CAN Manual] (Page 16)
        speed = struct.unpack(">h", response.data[1:3])[0]
        return speed

    async def read_pulses_received(self, can_id: int) -> int:
        """
        Reads the number of pulses received by the motor (Command 0x33).

        This typically applies when the motor is in a pulse control mode.
        Returns as a signed 32-bit integer.

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The number of pulses received as an integer.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_PULSES_RECEIVED, expected_dlc=6
        )
        # Manual indicates big-endian for multi-byte data. [MKS Servo42D CAN Manual] (Page 17)
        pulses = struct.unpack(">i", response.data[1:5])[0]
        return pulses

    async def read_io_status(self, can_id: int) -> Dict[str, int]:
        """
        Reads the status of the I/O ports (Command 0x34).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            A dictionary containing the status of IN_1, IN_2, OUT_1, OUT_2,
            and the raw status byte. Values are 0 or 1.
            Example: {'IN_1': 0, 'IN_2': 1, 'OUT_1': 0, 'OUT_2': 0, 'raw_byte': 0x02}

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
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
        """
        Reads the RAW accumulated encoder value (addition) (Command 0x35).

        Similar to command 0x31, this returns the total accumulated encoder pulses
        as a signed 48-bit integer (transmitted as 6 bytes, MSB first).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The raw accumulated encoder value as an integer.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_RAW_ENCODER_ADDITION, expected_dlc=8
        )
        # Data is int48_t, transmitted as 6 bytes, MSB first. [MKS Servo42D CAN Manual] (Page 17)
        raw_bytes = response.data[1:7] # 6 bytes, MSB first
        # Perform sign extension for the 48-bit value to 64-bit for Python's struct.unpack
        # Check sign bit of the MSB of the 48-bit number (which is the MSB of raw_bytes[0])
        if raw_bytes[0] & 0x80:
            # Prepend 0xFFFF for negative numbers
            padded_bytes = b"\xff\xff" + raw_bytes
        else:
            # Prepend 0x0000 for positive numbers
            padded_bytes = b"\x00\x00" + raw_bytes
        # Unpack as big-endian 64-bit signed integer
        return struct.unpack(">q", padded_bytes)[0]

    async def read_shaft_angle_error(self, can_id: int) -> int:
        """
        Reads the error of the motor shaft angle (Command 0x39).

        Returns the difference between the target angle and the real-time angle
        as a signed 32-bit integer. The scaling is 51200 counts for 360 degrees.

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The shaft angle error value.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_SHAFT_ANGLE_ERROR, expected_dlc=6
        )
        # Manual indicates big-endian for multi-byte data. [MKS Servo42D CAN Manual] (Page 18)
        return struct.unpack(">i", response.data[1:5])[0]

    async def read_en_pin_status(self, can_id: int) -> bool:
        """
        Reads the EN (enable) pin status (Command 0x3A).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            True if the motor is enabled, False otherwise.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_EN_PIN_STATUS, expected_dlc=3
        )
        return response.data[1] == 0x01

    async def read_power_on_zero_status(self, can_id: int) -> int:
        """
        Reads the status of the "go back to zero when power on" function (Command 0x3B).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            An integer status code:
            - 0: Going to zero.
            - 1: Go back to zero success.
            - 2: Go back to zero fail.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_POWER_ON_ZERO_STATUS, expected_dlc=3
        )
        status = response.data[1]
        if status == 2: # [MKS Servo42D CAN Manual] (Page 19, status = 2 go back to zero fail)
            logger.warning(
                f"Power on zero status for CAN ID {can_id} reported failure (status 2)."
            )
        return status

    async def release_stall_protection(self, can_id: int) -> bool:
        """
        Releases the motor shaft locked-rotor protection state (Command 0x3D).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            True if release was successful, False if release failed (status 0).

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            CommandError: If an unexpected status is returned.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RELEASE_STALL_PROTECTION, expected_dlc=3
        )
        status = response.data[1]
        if status == 0x01: # [MKS Servo42D CAN Manual] (Page 19, status = 1 release success)
            return True
        elif status == 0x00: # [MKS Servo42D CAN Manual] (Page 19, status = 0 release fail)
            logger.warning(
                f"Release stall protection failed for CAN ID {can_id} (status 0)."
            )
            return False
        else:
            raise CommandError(
                f"Unexpected status from release_stall_protection: {status}"
            )

    async def read_motor_protection_state(self, can_id: int) -> bool:
        """
        Reads the motor shaft protection state (Command 0x3E).
        Indicates if stall or position error protection is active.

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            True if the motor is in a protected state, False otherwise.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_READ_MOTOR_PROTECTION_STATE, expected_dlc=3
        )
        return response.data[1] == 0x01 # [MKS Servo42D CAN Manual] (Page 19, status = 1 protected)

    # --- Part 5.2: Set system parameters command ---
    async def calibrate_encoder(self, can_id: int) -> None:
        """
        Calibrates the encoder (Command 0x80).
        The motor must be unloaded for calibration.

        Args:
            can_id: The CAN ID of the target motor.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            CalibrationError: If the motor reports calibration failure.
            CommandError: If an unexpected status is returned from the motor.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_CALIBRATE_ENCODER, data=[0x00], expected_dlc=3
        )
        status = response.data[1]
        if status == const.STATUS_CALIBRATING_FAIL: # status = 2 [MKS Servo42D CAN Manual] (Page 20)
            raise CalibrationError(
                "Encoder calibration failed.", error_code=status, can_id=can_id
            )
        elif status == const.STATUS_CALIBRATED_SUCCESS: # status = 1 [MKS Servo42D CAN Manual] (Page 20)
            logger.info(f"CAN ID {can_id}: Encoder calibration successful.")
        elif status == const.STATUS_CALIBRATING: # status = 0 [MKS Servo42D CAN Manual] (Page 20)
            logger.info(f"CAN ID {can_id}: Encoder calibration started/in progress.")
        else:
            raise CommandError(
                f"Unexpected status from calibrate_encoder: {status}"
            )
            
    async def set_work_mode(self, can_id: int, mode: int) -> None:
        """
        Sets the working mode of the motor (Command 0x82).

        Args:
            can_id: The CAN ID of the target motor.
            mode: The work mode code (0-5). Refer to `const.MODE_*`.

        Raises:
            ParameterError: If the mode value is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the work mode.
        """
        if not (const.MODE_CR_OPEN <= mode <= const.MODE_SR_VFOC): # Modes 0-5 [MKS Servo42D CAN Manual] (Page 20)
            raise ParameterError(f"Invalid work mode: {mode}. Must be 0-5.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_WORK_MODE, data=[mode], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 20)
            raise MotorError(
                f"Set work mode to {mode} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_working_current(self, can_id: int, current_ma: int) -> None:
        """
        Sets the working current of the motor (Command 0x83).

        Args:
            can_id: The CAN ID of the target motor.
            current_ma: The working current in mA (uint16_t). Max values vary by motor.
                        Data is sent big-endian.

        Raises:
            ParameterError: If the current value seems out of a typical safe range.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the working current.
        """
        if not (0 <= current_ma <= 6000): # Max current 3000mA or 5200mA depending on motor [MKS Servo42D CAN Manual] (Page 21)
            raise ParameterError(f"Invalid working current: {current_ma}mA. Seems out of typical range.")
        # Pack current_ma as big-endian (MSB first) uint16_t [MKS Servo42D CAN Manual] (Page 21, byte2-3 ma (uint16_t))
        current_bytes = list(struct.pack(">H", current_ma))
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_WORKING_CURRENT,
            data=current_bytes,
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 21)
            raise MotorError(
                f"Set working current to {current_ma}mA failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_holding_current_percentage(
        self, can_id: int, percentage_code: int
    ) -> None:
        """
        Sets the holding current percentage (Command 0x9B).
        Valid for OPEN and CLOSE modes.

        Args:
            can_id: The CAN ID of the target motor.
            percentage_code: Code for holding current (0x00 for 10% to 0x08 for 90%). [MKS Servo42D CAN Manual] (Page 21)

        Raises:
            ParameterError: If the percentage_code is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the holding current.
        """
        if not (0x00 <= percentage_code <= 0x08): # [MKS Servo42D CAN Manual] (Page 21, holdMa (00~08))
            raise ParameterError(
                f"Invalid holding current code: {percentage_code}. Must be 0-8."
            )
        response = await self._send_command_and_get_response(
            can_id,
            const.CMD_SET_HOLDING_CURRENT_PERCENTAGE,
            data=[percentage_code],
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 21)
            raise MotorError(
                f"Set holding current percentage to code {percentage_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_subdivision(self, can_id: int, microsteps: int) -> None:
        """
        Sets the motor subdivision (microsteps) (Command 0x84).

        Args:
            can_id: The CAN ID of the target motor.
            microsteps: The subdivision value (0-255). [MKS Servo42D CAN Manual] (Page 22, micstep(00~FF))

        Raises:
            ParameterError: If the microsteps value is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the subdivision.
        """
        if not (0 <= microsteps <= 255):
            raise ParameterError(f"Invalid microsteps value: {microsteps}. Must be 0-255.")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SUBDIVISION, data=[microsteps], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 22)
            raise MotorError(
                f"Set subdivision to {microsteps} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    async def set_en_pin_active_level(self, can_id: int, level_code: int) -> None:
        """
        Sets the active level of the EN (enable) pin (Command 0x85).

        Args:
            can_id: The CAN ID of the target motor.
            level_code: The active level code:
                        - `const.EN_ACTIVE_LOW` (0x00): Low level active. [MKS Servo42D CAN Manual] (Page 22)
                        - `const.EN_ACTIVE_HIGH` (0x01): High level active. [MKS Servo42D CAN Manual] (Page 22)
                        - `const.EN_ACTIVE_ALWAYS` (0x02): Always active (Hold). [MKS Servo42D CAN Manual] (Page 22)

        Raises:
            ParameterError: If the level_code is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the EN pin active level.
        """
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
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 22)
            raise MotorError(
                f"Set En pin active level to code {level_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_motor_direction(self, can_id: int, direction_code: int) -> None:
        """
        Sets the positive direction of motor rotation for pulse interface (Command 0x86).

        Args:
            can_id: The CAN ID of the target motor.
            direction_code: The direction code:
                            - `const.DIR_CW` (0x00): Clockwise rotation is positive. [MKS Servo42D CAN Manual] (Page 23)
                            - `const.DIR_CCW` (0x01): Counter-clockwise rotation is positive. [MKS Servo42D CAN Manual] (Page 23)

        Raises:
            ParameterError: If the direction_code is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the direction.
        """
        if direction_code not in [const.DIR_CW, const.DIR_CCW]:
            raise ParameterError(
                f"Invalid motor direction code: {direction_code}. Must be 0 (CW) or 1 (CCW)."
            )
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_MOTOR_DIRECTION, data=[direction_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 23)
            raise MotorError(
                f"Set motor direction to code {direction_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_auto_screen_off(self, can_id: int, enable: bool) -> None:
        """
        Sets the auto screen off function for the motor's display (Command 0x87).

        Args:
            can_id: The CAN ID of the target motor.
            enable: True to enable auto screen off (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 23)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the auto screen off function.
        """
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_AUTO_SCREEN_OFF, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 23)
            raise MotorError(
                f"Set auto screen off (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_stall_protection(self, can_id: int, enable: bool) -> None:
        """
        Sets the motor shaft locked-rotor protection function (Command 0x88).

        Args:
            can_id: The CAN ID of the target motor.
            enable: True to enable stall protection (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 24)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set stall protection.
        """
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_STALL_PROTECTION, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 24)
            raise MotorError(
                f"Set stall protection (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    async def set_subdivision_interpolation(self, can_id: int, enable: bool) -> None:
        """
        Sets the internal 256 subdivision interpolation function (Command 0x89).

        Args:
            can_id: The CAN ID of the target motor.
            enable: True to enable interpolation (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 24)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set subdivision interpolation.
        """
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SUBDIVISION_INTERPOLATION, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 24)
            raise MotorError(
                f"Set subdivision interpolation (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_can_bitrate(self, can_id: int, bitrate_code: int) -> None:
        """
        Sets the CAN bus bitrate for the motor (Command 0x8A).

        Args:
            can_id: The CAN ID of the target motor.
            bitrate_code: The bitrate code:
                          - `const.CAN_BITRATE_125K` (0x00) [MKS Servo42D CAN Manual] (Page 25)
                          - `const.CAN_BITRATE_250K` (0x01) [MKS Servo42D CAN Manual] (Page 25)
                          - `const.CAN_BITRATE_500K` (0x02) [MKS Servo42D CAN Manual] (Page 25)
                          - `const.CAN_BITRATE_1M` (0x03) [MKS Servo42D CAN Manual] (Page 25)

        Raises:
            ParameterError: If the bitrate_code is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the CAN bitrate.
        """
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
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 25)
            raise MotorError(
                f"Set CAN bitrate to code {bitrate_code} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_can_id(self, current_can_id: int, new_can_id: int) -> None:
        """
        Sets a new CAN ID for the motor (Command 0x8B).

        Args:
            current_can_id: The current CAN ID of the motor to address.
            new_can_id: The new CAN ID to set (0-2047). ID(00~7FF) [MKS Servo42D CAN Manual] (Page 25)
                        Data is sent big-endian.

        Raises:
            ParameterError: If the new_can_id is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the new CAN ID.
        """
        if not (0 <= new_can_id <= 0x7FF): # [MKS Servo42D CAN Manual] (Page 25, ID(00~7FF))
            raise ParameterError(f"Invalid new CAN ID: {new_can_id}. Must be 0-2047.")
        # Pack new_can_id as big-endian (MSB first) uint16_t [MKS Servo42D CAN Manual] (Page 25, byte2 byte3 ID(00~7FF))
        id_bytes = list(struct.pack(">H", new_can_id))
        
        response = await self._send_command_and_get_response(
            current_can_id,
            const.CMD_SET_CAN_ID,
            data=id_bytes,
            expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 25)
            raise MotorError(
                f"Set CAN ID to {new_can_id} failed for motor currently at {current_can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=current_can_id,
            )

    async def set_slave_respond_active(
        self, can_id: int, respond_enabled: bool, active_enabled: bool
    ) -> None:
        """
        Sets the slave response and active data initiation behavior (Command 0x8C).

        Args:
            can_id: The CAN ID of the target motor.
            respond_enabled: True to enable motor responses (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 26)
            active_enabled: True to enable active data initiation by the motor (0x01)
                            (e.g., sending move completion messages), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 26)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the parameters.
        """
        respond_code = 0x01 if respond_enabled else 0x00
        active_code = 0x01 if active_enabled else 0x00
        data = [respond_code, active_code]
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_SLAVE_RESPOND_ACTIVE, data=data, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 26)
            raise MotorError(
                f"Set slave respond/active (respond={respond_enabled}, active={active_enabled}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_group_id(self, can_id: int, group_id: int) -> None:
        """
        Sets the group ID for the motor (Command 0x8D).
        Allows the motor to respond to commands sent to this group ID.
        Data is sent big-endian.

        Args:
            can_id: The specific CAN ID of the target motor.
            group_id: The group ID to set (1-2047). ID(01~0x7FF) [MKS Servo42D CAN Manual] (Page 27)

        Raises:
            ParameterError: If the group_id is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the group ID.
        """
        if not (0x01 <= group_id <= 0x7FF): # [MKS Servo42D CAN Manual] (Page 27, ID(01~0x7FF))
            raise ParameterError(f"Invalid group ID: {group_id}. Must be 1-2047.")
        # Pack group_id as big-endian (MSB first) uint16_t [MKS Servo42D CAN Manual] (Page 27, byte2 byte3 ID(01~0x7FF))
        group_id_bytes = list(struct.pack(">H", group_id))

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_GROUP_ID, data=group_id_bytes, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 27)
            raise MotorError(
                f"Set group ID to {group_id} failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_key_lock(self, can_id: int, lock_enabled: bool) -> None:
        """
        Locks or unlocks the motor's physical keys/buttons (Command 0x8F).

        Args:
            can_id: The CAN ID of the target motor.
            lock_enabled: True to lock the keys (0x01), False to unlock (0x00). [MKS Servo42D CAN Manual] (Page 27)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the key lock state.
        """
        enable_code = 0x01 if lock_enabled else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_KEY_LOCK, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 27)
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
        out1_mask_action: int = 2, # 0: Do not write, 1: Write, 2: Unchanged [MKS Servo42D CAN Manual] (Page 28)
        out2_mask_action: int = 2  # 0: Do not write, 1: Write, 2: Unchanged [MKS Servo42D CAN Manual] (Page 28)
    ) -> None:
        """
        Writes to the motor's output IO ports (OUT1, OUT2) (Command 0x36).

        Args:
            can_id: The CAN ID of the target motor.
            out1_value: Value for OUT1 (0 or 1). Required if out1_mask_action is 1.
            out2_value: Value for OUT2 (0 or 1). Required if out2_mask_action is 1.
            out1_mask_action: Action for OUT1.
                              0: Do not write to OUT1.
                              1: Write out1_value to OUT1.
                              2: OUT1 value remains unchanged (default).
            out2_mask_action: Action for OUT2.
                              0: Do not write to OUT2.
                              1: Write out2_value to OUT2.
                              2: OUT2 value remains unchanged (default).

        Raises:
            ParameterError: If mask actions or values are invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to write to the IO port.
        """
        if not (0 <= out1_mask_action <= 2 and 0 <= out2_mask_action <= 2):
            raise ParameterError("Mask actions must be 0, 1, or 2.")

        data_byte = 0 # byte2 in the manual [MKS Servo42D CAN Manual] (Page 28)
        
        # OUT2_mask (bits 7-6), OUT2 value (bit 3)
        if out2_mask_action == 1: # Write value
            if out2_value not in [0, 1]:
                raise ParameterError("out2_value must be 0 or 1 when out2_mask_action is 1.")
            data_byte |= (0b01 << 6) # OUT2_mask = 1
            if out2_value == 1: data_byte |= (1 << 3) # OUT2 value bit
        elif out2_mask_action == 0: # Do not write
            data_byte |= (0b00 << 6) # OUT2_mask = 0
        elif out2_mask_action == 2: # Unchanged
            data_byte |= (0b10 << 6) # OUT2_mask = 2
        
        # OUT1_mask (bits 5-4), OUT1 value (bit 2)
        if out1_mask_action == 1: # Write value
            if out1_value not in [0, 1]:
                raise ParameterError("out1_value must be 0 or 1 when out1_mask_action is 1.")
            data_byte |= (0b01 << 4) # OUT1_mask = 1
            if out1_value == 1: data_byte |= (1 << 2) # OUT1 value bit
        elif out1_mask_action == 0: # Do not write
            data_byte |= (0b00 << 4) # OUT1_mask = 0
        elif out1_mask_action == 2: # Unchanged
            data_byte |= (0b10 << 4) # OUT1_mask = 2
        
        # Bits 1 and 0 are reserved (0) according to manual table [MKS Servo42D CAN Manual] (Page 28)
        
        response = await self._send_command_and_get_response(
            can_id, const.CMD_WRITE_IO_PORT, data=[data_byte], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 28)
            raise MotorError(
                f"Write IO port failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    # --- Part 5.4: Set Home command ---
    async def set_home_parameters(
        self,
        can_id: int,
        home_trig_level: int, # byte2: 0 Low, 1 High [MKS Servo42D CAN Manual] (Page 28)
        home_dir: int,        # byte3: 0 CW, 1 CCW [MKS Servo42D CAN Manual] (Page 28)
        home_speed_rpm: int,  # byte4-5: 0-3000 RPM, uint16_t big-endian [MKS Servo42D CAN Manual] (Page 28)
        end_limit_enabled: bool, # byte6: 0 disable, 1 enable [MKS Servo42D CAN Manual] (Page 28)
        home_mode: int        # byte7: 0 Use Limit, 1 No Limit [MKS Servo42D CAN Manual] (Page 28)
    ) -> None:
        """
        Sets the parameters for the homing operation (Command 0x90).
        Data for speed is sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            home_trig_level: Effective level of the end stop (0: Low, 1: High).
            home_dir: Direction of homing (0: CW, 1: CCW).
            home_speed_rpm: Speed of homing in RPM (0-3000).
            end_limit_enabled: True to enable endstop limit during homing, False otherwise.
            home_mode: Homing method (0: Use Limit switch, 1: No Limit switch).

        Raises:
            ParameterError: If any parameter value is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the home parameters.
        """
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
        # Pack home_speed_rpm as big-endian (MSB first) uint16_t [MKS Servo42D CAN Manual] (Page 28)
        byte4_5_speed = list(struct.pack(">H", home_speed_rpm))
        byte6_endlimit = 0x01 if end_limit_enabled else 0x00
        byte7_hmMode = home_mode & 0x01
        
        data_payload = [byte2_level, byte3_dir] + byte4_5_speed + [byte6_endlimit, byte7_hmMode]

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_HOME_PARAMETERS, data=data_payload, expected_dlc=3 # Response DLC is 3 [MKS Servo42D CAN Manual] (Page 29)
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 29)
            raise MotorError(
                f"Set home parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def go_home(self, can_id: int) -> int:
        """
        Commands the motor to perform the homing sequence (Command 0x91).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The initial status code from the motor:
            - `const.HOME_START` (1): Homing started. [MKS Servo42D CAN Manual] (Page 30)
            - `const.HOME_SUCCESS` (2): Homing completed successfully (if immediate). [MKS Servo42D CAN Manual] (Page 30)
            (Note: `const.HOME_FAIL` (0) will raise MotorError). [MKS Servo42D CAN Manual] (Page 30)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor reports immediate failure to start homing.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_GO_HOME, expected_dlc=3
        )
        status = response.data[1] 
        if status == const.HOME_FAIL: # status = 0 [MKS Servo42D CAN Manual] (Page 30)
             raise MotorError(
                f"Go home command failed to start or reported immediate failure for CAN ID {can_id}. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status
        
    async def set_current_axis_to_zero(self, can_id: int) -> None:
        """
        Sets the current motor position as the zero point (Command 0x92).
        This is like "GoHome" without running the motor.

        Args:
            can_id: The CAN ID of the target motor.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the current axis to zero.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_CURRENT_AXIS_TO_ZERO, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 30)
            raise MotorError(
                f"Set current axis to zero failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_nolimit_home_params(
        self, can_id: int, reverse_angle_pulses: int, home_current_ma: int
    ) -> None:
        """
        Sets parameters for "no limit switch" homing (Command 0x94).
        Data for angle and current is sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            reverse_angle_pulses: The angle (in pulses, uint32_t) to reverse
                                  after stall during no-limit homing. [MKS Servo42D CAN Manual] (Page 31, retValue)
            home_current_ma: The current (mA, uint16_t) to use during no-limit homing stall. [MKS Servo42D CAN Manual] (Page 31, ma)

        Raises:
            ParameterError: If parameter values are out of range.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the parameters.
        """
        if not (0 <= reverse_angle_pulses <= 0xFFFFFFFF):
            raise ParameterError("reverse_angle_pulses must be a uint32_t value.")
        if not (0 <= home_current_ma <= 0xFFFF): # Max current depends on motor type, but param is uint16_t
            raise ParameterError("home_current_ma must be a uint16_t value.")

        # Pack as big-endian (MSB first) [MKS Servo42D CAN Manual] (Page 31, Byte2-5 retValue, Byte6-7 Hm_ma)
        ret_val_bytes = list(struct.pack(">I", reverse_angle_pulses))
        ma_bytes = list(struct.pack(">H", home_current_ma))
        data_payload = ret_val_bytes + ma_bytes

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_NOLIMIT_HOME_PARAMS, data=data_payload, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 31)
            raise MotorError(
                f"Set no-limit home parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def set_limit_port_remap(self, can_id: int, enable_remap: bool) -> None:
        """
        Enables or disables limit port remapping (Command 0x9E).
        In serial control mode, this can remap IN_1 to En and IN_2 to Dir.

        Args:
            can_id: The CAN ID of the target motor.
            enable_remap: True to enable remapping (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 31)

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the limit port remap state.
        """
        enable_code = 0x01 if enable_remap else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_LIMIT_PORT_REMAP, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 31)
            raise MotorError(
                f"Set limit port remap (enable={enable_remap}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.5: Set 0_Mode command ---
    async def set_zero_mode_parameters(
        self,
        can_id: int,
        mode: int,             # byte2: 0 Disable, 1 DirMode, 2 NearMode [MKS Servo42D CAN Manual] (Page 32)
        set_zero_action: int,  # byte3: 0 Clean, 1 Set Current, 2 No Modify [MKS Servo42D CAN Manual] (Page 32)
        speed_code: int,       # byte4: 0 slowest to 4 fastest [MKS Servo42D CAN Manual] (Page 32)
        direction_code: int    # byte5: 0 CW, 1 CCW [MKS Servo42D CAN Manual] (Page 32)
    ) -> None:
        """
        Sets parameters for the power-on auto-zero mode (Command 0x9A).

        Args:
            can_id: The CAN ID of the target motor.
            mode: 0_Mode behavior (0: Disable, 1: DirMode, 2: NearMode).
            set_zero_action: Action for setting zero point (0: Clean, 1: Set Current, 2: No Modify).
            speed_code: Speed for auto-zero (0: slowest to 4: fastest).
            direction_code: Direction for auto-zero (0: CW, 1: CCW).

        Raises:
            ParameterError: If any parameter value is invalid.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the 0_Mode parameters.
        """
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
            can_id, const.CMD_SET_ZERO_MODE_PARAMETERS, data=data_payload, expected_dlc=3 # Response DLC is 3 [MKS Servo42D CAN Manual] (Page 32)
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 32)
            raise MotorError(
                f"Set 0_Mode parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.6: Restore Default Parameters ---
    async def restore_default_parameters(self, can_id: int) -> None:
        """
        Restores the motor's parameters to their factory defaults (Command 0x3F).
        The motor will restart and require calibration after this command.

        Args:
            can_id: The CAN ID of the target motor.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to restore default parameters.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RESTORE_DEFAULT_PARAMETERS, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 33)
            raise MotorError(
                f"Restore default parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        logger.info(f"CAN ID {can_id}: Default parameters restored. Motor will restart and requires calibration.")

    # --- Part 5.7: Restart Motor ---
    async def restart_motor(self, can_id: int) -> None:
        """
        Restarts the motor controller (Command 0x41).

        Args:
            can_id: The CAN ID of the target motor.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to restart.
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_RESTART_MOTOR, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 33)
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
        enable_en_trigger_zero: bool,    # byte2 bit1 (g0En) [MKS Servo42D CAN Manual] (Page 34)
        enable_pos_error_protection: bool, # byte2 bit0 (pEn) [MKS Servo42D CAN Manual] (Page 34)
        error_detection_time_ms_units: int, # byte3-4 (Tim uint16_t), big-endian [MKS Servo42D CAN Manual] (Page 34)
        error_threshold_pulses: int         # byte5-6 (Errors uint16_t), big-endian [MKS Servo42D CAN Manual] (Page 34)
    ) -> None:
        """
        Configures EN-triggered zero return and position error protection (Command 0x9D).
        Data for time and pulses sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            enable_en_trigger_zero: True to enable EN-triggered single-turn zero return.
            enable_pos_error_protection: True to enable position error protection.
            error_detection_time_ms_units: Time length for error statistics (uint16_t,
                                           1 unit approx. 15ms).
            error_threshold_pulses: Number of error pulses to trigger protection (uint16_t).

        Raises:
            ParameterError: If time or pulse values are out of uint16_t range.
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor fails to set the parameters.
        """
        if not (0 <= error_detection_time_ms_units <= 0xFFFF):
            raise ParameterError("error_detection_time_ms_units out of range (uint16_t).")
        if not (0 <= error_threshold_pulses <= 0xFFFF):
            raise ParameterError("error_threshold_pulses out of range (uint16_t).")

        byte2_config = 0x00 # byte2 in manual [MKS Servo42D CAN Manual] (Page 34)
        # bit1 g0En, bit0 pEn. Other bits reserved (0).
        if enable_en_trigger_zero: byte2_config |= (1 << 1) # g0En [MKS Servo42D CAN Manual] (Page 34)
        if enable_pos_error_protection: byte2_config |= (1 << 0) # pEn [MKS Servo42D CAN Manual] (Page 34)
        
        # Pack as big-endian (MSB first) [MKS Servo42D CAN Manual] (Page 34)
        tim_bytes = list(struct.pack(">H", error_detection_time_ms_units))
        errors_bytes = list(struct.pack(">H", error_threshold_pulses))
        data_payload = [byte2_config] + tim_bytes + errors_bytes

        response = await self._send_command_and_get_response(
            can_id, const.CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION, data=data_payload, expected_dlc=3 # Response DLC is 3 [MKS Servo42D CAN Manual] (Page 34)
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 34)
            raise MotorError(
                f"Set En trigger and Pos Error Protection failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    # --- Part 5.9: Read system Parameter command ---
    async def read_system_parameter(
        self, can_id: int, parameter_command_code: int
    ) -> Tuple[int, bytes]:
        """
        Reads a system parameter from the motor using the generic read command (0x00).

        Args:
            can_id: The CAN ID of the target motor.
            parameter_command_code: The command code of the parameter to be read
                                    (e.g., `const.CMD_SET_WORK_MODE` (0x82) to read work mode).
                                    This code is echoed as byte1 of the response data payload. [MKS Servo42D CAN Manual] (Page 35)

        Returns:
            A tuple (echoed_command_code, parameter_data_bytes).
            'echoed_command_code' should match `parameter_command_code`.
            'parameter_data_bytes' is a bytes object containing the value(s) of the parameter,
            in big-endian format if multi-byte.

        Raises:
            CommunicationError, CommandError, CRCError: On communication or response issues.
            MotorError: If the motor indicates the parameter cannot be read (responds with 0xFF 0xFF). [MKS Servo42D CAN Manual] (Page 35)
        """
        response_msg = await self._send_command_and_get_response(
            can_id,
            const.CMD_READ_SYSTEM_PARAMETER_PREFIX, # This is 0x00
            data=[parameter_command_code],
        )
        # Per manual page 35, if parameter cannot be read, data is FF FF after echoed code
        if len(response_msg.data) == 4 and \
           response_msg.data[0] == parameter_command_code and \
           response_msg.data[1] == 0xFF and \
           response_msg.data[2] == 0xFF:
            raise MotorError(
                f"Parameter with code {parameter_command_code:02X} cannot be read from CAN ID {can_id} (motor responded 0xFF 0xFF).",
                error_code=0xFFFF,
                can_id=can_id,
            )
        echoed_code = response_msg.data[0] # This should be parameter_command_code
        param_data_bytes = response_msg.data[1:-1] # Parameter data, CRC is last byte
        return echoed_code, param_data_bytes
        
    # --- Part 6: Run Motor Commands ---
    async def _run_motor_command(
        self, can_id: int, command_code: int, data: List[int]
    ) -> int:
        """
        Internal helper to send a motor run command and process its initial response.

        Args:
            can_id: The CAN ID of the target motor.
            command_code: The specific run command code (e.g., 0xFD, 0xFE).
            data: The data payload for the run command.

        Returns:
            The status code from the motor's initial response (e.g., POS_RUN_STARTING).

        Raises:
            MotorError: If the motor reports immediate failure (status 0x00).
            CommunicationError, CommandError, CRCError: On communication or response issues.
        """
        logger.info(f"LowLevelAPI._run_motor_command: Preparing to send CMD={command_code:02X} to CAN_ID={can_id:03X} with DataForCmd={data}")
        response = await self._send_command_and_get_response(
            can_id, command_code, data=data, expected_dlc=3
        )
        status = response.data[1]
        logger.info(f"LowLevelAPI._run_motor_command: CMD={command_code:02X} for CAN_ID={can_id:03X} received response status={status:02X}")
        # For run commands like FD, FE, F4, F5, F6, status 0 generally means failure.
        # Example: 0xFD status 0 = run fail [MKS Servo42D CAN Manual] (Page 43)
        if status == const.POS_RUN_FAIL: # Typically 0x00 for run commands
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
        speed: int,         # 12-bit value for speed parameter (0-3000 range) [MKS Servo42D CAN Manual] (Page 43)
        acceleration: int,  # 8-bit value (0-255) [MKS Servo42D CAN Manual] (Page 43)
        pulses: int,        # 24-bit unsigned value (0-0xFFFFFF), sent MSB first [MKS Servo42D CAN Manual] (Page 43, byte5-7 pulses)
    ) -> int:
        """
        Runs the motor in position mode by a relative number of pulses (Command 0xFD).
        Pulses are sent big-endian (MSB first).

        Args:
            can_id: The CAN ID of the target motor.
            ccw_direction: True for Counter-Clockwise (positive direction, dir=0),
                           False for Clockwise (negative direction, dir=1). [MKS Servo42D CAN Manual] (Page 43, dir 0=CCW, 1=CW. Note: This seems opposite to typical conventions, but matches manual example interpretation)
                           The manual's description for "dir" on page 43 indicates 0 for CCW, 1 for CW.
                           The example sends "01 FD 01 40..." for forward (CCW interpretation in example), where byte2's dir bit (b7) is 0.
                           The example "01 FD 81 40..." for reverse (CW), where byte2's dir bit (b7) is 1.
                           So, ccw_direction=True (dir=0) means byte2 bit7 = 0.
                           ccw_direction=False (dir=1) means byte2 bit7 = 1.
            speed: Speed parameter (0-3000).
            acceleration: Acceleration parameter (0-255).
            pulses: Number of pulses to move (0 - 16,777,215).

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`).

        Raises:
            ParameterError: If input parameters are out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_pulses: CAN_ID={can_id:03X}, "
            f"CCW={ccw_direction}, SpeedParam={speed}, AccelParam={acceleration}, Pulses={pulses}"
        )
        if not (0 <= speed <= 3000): # Speed param range [MKS Servo42D CAN Manual] (Page 43)
            raise ParameterError(f"Speed parameter {speed} out of range for 0xFD (0-3000).")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 43)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (0 <= pulses <= 0xFFFFFF): # Pulses param range [MKS Servo42D CAN Manual] (Page 43)
            raise ParameterError(f"Pulses {pulses} out of range for 0xFD (0-16,777,215).")

        # byte2: b7=dir, b6-b4=Rev(0), b3-b0=speed_high_nibble [MKS Servo42D CAN Manual] (Page 43)
        # byte3: b7-b0=speed_low_byte [MKS Servo42D CAN Manual] (Page 43)
        # Speed is a 12-bit value (0xFFF max from these fields).
        byte2 = (speed >> 8) & 0x0F 
        if not ccw_direction: # CW direction, dir bit = 1
            byte2 |= 0x80
        # if ccw_direction, dir bit = 0, already handled by masking.
        byte3 = speed & 0xFF
        byte4 = acceleration & 0xFF
        # Pulses: 24-bit unsigned, send MSB first [MKS Servo42D CAN Manual] (Page 43, example 00 FA 00 for pulses)
        pulse_bytes = [(pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF]
        data_payload = [byte2, byte3, byte4] + pulse_bytes 
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, data_payload
        )

    async def run_position_mode_absolute_pulses(
        self, can_id: int, speed: int, acceleration: int, absolute_pulses: int
    ) -> int:
        """
        Runs the motor in position mode to an absolute pulse position (Command 0xFE).
        Speed (uint16_t) and pulses (int24_t) are sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            speed: Speed parameter (0-3000). Sent as uint16_t. [MKS Servo42D CAN Manual] (Page 45)
            acceleration: Acceleration parameter (0-255). [MKS Servo42D CAN Manual] (Page 45)
            absolute_pulses: Target absolute pulse position (signed 24-bit,
                             -8,388,608 to +8,388,607). Sent as int24_t. [MKS Servo42D CAN Manual] (Page 45)

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`).

        Raises:
            ParameterError: If input parameters are out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_pulses: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, AbsPulses={absolute_pulses}"
        )
        if not (0 <= speed <= 3000): # Speed param range [MKS Servo42D CAN Manual] (Page 45)
            raise ParameterError(f"Speed parameter {speed} out of range for 0xFE (0-3000).")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 45)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= absolute_pulses <= 8388607): # absPulses range [MKS Servo42D CAN Manual] (Page 45)
             raise ParameterError(f"Absolute pulses {absolute_pulses} out of range for signed 24-bit for 0xFE.")

        # speed: byte2-3 (uint16_t) [MKS Servo42D CAN Manual] (Page 45)
        # Pack speed as big-endian (MSB first)
        speed_bytes = list(struct.pack(">H", speed)) 
        acc_byte = acceleration & 0xFF # byte4 [MKS Servo42D CAN Manual] (Page 45)
        
        # absolute_pulses: byte5-7 (int24_t), send MSB of 24-bit value first
        # Convert to unsigned 24-bit for masking/shifting
        val_to_pack_unsigned_24bit = absolute_pulses & 0xFFFFFF
        abs_pulse_bytes = [
            (val_to_pack_unsigned_24bit >> 16) & 0xFF, # MSB
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,  # Mid
            val_to_pack_unsigned_24bit & 0xFF,         # LSB
        ]
        data_payload = speed_bytes + [acc_byte] + abs_pulse_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, data_payload
        )

    async def run_speed_mode(
        self, can_id: int, ccw_direction: bool, speed: int, acceleration: int
    ) -> int:
        """
        Runs the motor in speed mode (Command 0xF6).

        Args:
            can_id: The CAN ID of the target motor.
            ccw_direction: True for Counter-Clockwise (dir=0), False for Clockwise (dir=1). [MKS Servo42D CAN Manual] (Page 40, dir 0=CCW, 1=CW)
            speed: Speed parameter (0-3000). This is a 12-bit value. [MKS Servo42D CAN Manual] (Page 40)
            acceleration: Acceleration parameter (0-255). [MKS Servo42D CAN Manual] (Page 40)

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`).

        Raises:
            ParameterError: If input parameters are out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.run_speed_mode: CAN_ID={can_id:03X}, CCW={ccw_direction}, "
            f"SpeedParam={speed}, AccelParam={acceleration}"
        )
        if not (0 <= speed <= 3000): # Manual for 0xF6 states speed 0-3000 for parameter [MKS Servo42D CAN Manual] (Page 40)
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF6 (0-3000).")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 40)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")

        # byte2: b7=dir, b6-b4=Rev(0), b3-b0=speed_high_nibble [MKS Servo42D CAN Manual] (Page 40)
        # byte3: b7-b0=speed_low_byte [MKS Servo42D CAN Manual] (Page 40)
        byte2 = (speed >> 8) & 0x0F # Speed high part (4 bits)
        if not ccw_direction: # CW direction, dir bit = 1
            byte2 |= 0x80
        byte3 = speed & 0xFF # Speed low part (8 bits)
        byte4 = acceleration # Accel (8 bits)
        data_payload = [byte2, byte3, byte4] 
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data_payload
        )

    async def stop_speed_mode(self, can_id: int, acceleration: int) -> int:
        """
        Stops the motor when in speed mode (uses Command 0xF6 with speed 0).

        Args:
            can_id: The CAN ID of the target motor.
            acceleration: Deceleration parameter (0-255). 0 for immediate stop. [MKS Servo42D CAN Manual] (Page 41)

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`
            indicating stop has started).

        Raises:
            ParameterError: If acceleration is out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(f"LowLevelAPI.stop_speed_mode: CAN_ID={can_id:03X}, AccelParam={acceleration}")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 41)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        # For stop, speed is 0. Direction bit in byte2 doesn't matter.
        byte2 = 0x00 # dir=0 (CCW), speed_high=0
        byte3 = 0x00 # speed_low=0
        byte4 = acceleration # deceleration
        data_payload = [byte2, byte3, byte4]
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_SPEED_MODE, data_payload
        )
        
    async def emergency_stop(self, can_id: int) -> None:
        """
        Commands an emergency stop for the motor (Command 0xF7).

        Args:
            can_id: The CAN ID of the target motor.

        Raises:
            CommunicationError, CommandError, CRCError: On communication issues.
            MotorError: If the motor reports failure to stop.
        """
        logger.info(f"LowLevelAPI.emergency_stop: CAN_ID={can_id:03X}")
        response = await self._send_command_and_get_response(
            can_id, const.CMD_EMERGENCY_STOP, expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 39)
            raise MotorError(
                f"Emergency stop command failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
            
    async def enable_motor(self, can_id: int, enable: bool) -> None:
        """
        Enables or disables the motor (Command 0xF3).

        Args:
            can_id: The CAN ID of the target motor.
            enable: True to enable the motor (0x01), False to disable (0x00). [MKS Servo42D CAN Manual] (Page 38)

        Raises:
            CommunicationError, CommandError, CRCError: On communication issues.
            MotorError: If the motor fails to enable/disable.
        """
        enable_code = 0x01 if enable else 0x00
        response = await self._send_command_and_get_response(
            can_id, const.CMD_ENABLE_MOTOR, data=[enable_code], expected_dlc=3
        )
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 38)
            raise MotorError(
                f"Enable motor command (enable={enable}) failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )

    async def query_motor_status(self, can_id: int) -> int:
        """
        Queries the current status of the motor (Command 0xF1).

        Args:
            can_id: The CAN ID of the target motor.

        Returns:
            The motor status code (e.g., `const.MOTOR_STATUS_STOPPED`,
            `const.MOTOR_STATUS_FULL_SPEED`). See manual page 38 for statuses.

        Raises:
            CommunicationError, CommandError, CRCError: On communication issues.
            MotorError: If the motor reports a query failure (status 0). [MKS Servo42D CAN Manual] (Page 38)
        """
        response = await self._send_command_and_get_response(
            can_id, const.CMD_QUERY_MOTOR_STATUS, expected_dlc=3
        )
        status = response.data[1]
        if status == const.MOTOR_STATUS_QUERY_FAIL: # status = 0 [MKS Servo42D CAN Manual] (Page 38)
             raise MotorError(
                f"Query motor status command failed for CAN ID {can_id}. Status: {status}",
                error_code=status, can_id=can_id
            )
        return status

    async def run_position_mode_relative_axis(
        self, can_id: int, speed: int, acceleration: int, relative_axis: int
    ) -> int:
        """
        Runs the motor in position mode by a relative axis value (Command 0xF4).
        The 'axis' value is typically raw encoder counts.
        Speed (uint16_t) and axis value (int24_t) are sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            speed: Speed parameter (0-3000). Sent as uint16_t. [MKS Servo42D CAN Manual] (Page 47)
            acceleration: Acceleration parameter (0-255). [MKS Servo42D CAN Manual] (Page 47)
            relative_axis: Relative axis value to move (signed 24-bit,
                           -8,388,608 to +8,388,607). Sent as int24_t. [MKS Servo42D CAN Manual] (Page 47)

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`).

        Raises:
            ParameterError: If input parameters are out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.run_position_mode_relative_axis: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, RelAxis={relative_axis}"
        )
        if not (0 <= speed <= 3000): # Speed param range [MKS Servo42D CAN Manual] (Page 47)
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF4 (0-3000).")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 47)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= relative_axis <= 8388607): # relAxis range [MKS Servo42D CAN Manual] (Page 47)
             raise ParameterError(f"Relative axis value {relative_axis} out of range for signed 24-bit for 0xF4.")

        # speed: byte2-3 (uint16_t) [MKS Servo42D CAN Manual] (Page 47)
        # Pack speed as big-endian (MSB first)
        speed_bytes = list(struct.pack(">H", speed)) 
        acc_byte = acceleration & 0xFF # byte4 [MKS Servo42D CAN Manual] (Page 47)
        
        # relative_axis: byte5-7 (int24_t), send MSB of 24-bit value first
        # Convert to unsigned 24-bit for masking/shifting
        val_to_pack_unsigned_24bit = relative_axis & 0xFFFFFF
        
        rel_axis_bytes = [
            (val_to_pack_unsigned_24bit >> 16) & 0xFF, # MSB
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,  # Mid
            val_to_pack_unsigned_24bit & 0xFF,         # LSB
        ]
        data_payload = speed_bytes + [acc_byte] + rel_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data_payload
        )

    async def stop_position_mode_relative_axis(self, can_id: int, acceleration: int) -> int:
        """
        Stops the motor when in relative axis position mode (uses Command 0xF4
        with speed 0 and relative_axis 0).

        Args:
            can_id: The CAN ID of the target motor.
            acceleration: Deceleration parameter (0-255). 0 for immediate stop. [MKS Servo42D CAN Manual] (Page 48)

        Returns:
            The initial status code from the motor.

        Raises:
            ParameterError: If acceleration is out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.stop_position_mode_relative_axis: CAN_ID={can_id:03X}, AccelParam={acceleration}"
        )
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 48)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        speed_bytes = [0x00, 0x00] # Speed = 0
        acc_byte = acceleration & 0xFF
        rel_axis_bytes = [0x00, 0x00, 0x00] # Relative axis = 0
        data_payload = speed_bytes + [acc_byte] + rel_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, data_payload
        )

    async def run_position_mode_absolute_axis(
        self, can_id: int, speed: int, acceleration: int, absolute_axis: int
    ) -> int:
        """
        Runs the motor in position mode to an absolute axis value (Command 0xF5).
        The 'axis' value is typically raw encoder counts. Supports real-time updates.
        Speed (uint16_t) and axis value (int24_t) are sent big-endian.

        Args:
            can_id: The CAN ID of the target motor.
            speed: Speed parameter (0-3000). Sent as uint16_t. [MKS Servo42D CAN Manual] (Page 49)
            acceleration: Acceleration parameter (0-255). [MKS Servo42D CAN Manual] (Page 49)
            absolute_axis: Target absolute axis value (signed 24-bit,
                           -8,388,608 to +8,388,607). Sent as int24_t. [MKS Servo42D CAN Manual] (Page 49)

        Returns:
            The initial status code from the motor (e.g., `const.POS_RUN_STARTING`).

        Raises:
            ParameterError: If input parameters are out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.run_position_mode_absolute_axis: CAN_ID={can_id:03X}, "
            f"SpeedParam={speed}, AccelParam={acceleration}, AbsAxis={absolute_axis}"
        )
        if not (0 <= speed <= 3000): # Speed param range [MKS Servo42D CAN Manual] (Page 49)
            raise ParameterError(f"Speed parameter {speed} out of range for 0xF5 (0-3000).")
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 49)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255).")
        if not (-8388608 <= absolute_axis <= 8388607): # absAxis range [MKS Servo42D CAN Manual] (Page 49)
             raise ParameterError(f"Absolute axis value {absolute_axis} out of range for signed 24-bit for 0xF5.")

        # speed: byte2-3 (uint16_t) [MKS Servo42D CAN Manual] (Page 49)
        # Pack speed as big-endian (MSB first)
        speed_bytes = list(struct.pack(">H", speed))
        acc_byte = acceleration & 0xFF # byte4 [MKS Servo42D CAN Manual] (Page 49)
        
        # absolute_axis: byte5-7 (int24_t), send MSB of 24-bit value first
        # Convert to unsigned 24-bit for masking/shifting
        val_to_pack_unsigned_24bit = absolute_axis & 0xFFFFFF
        
        abs_axis_bytes = [
            (val_to_pack_unsigned_24bit >> 16) & 0xFF, # MSB
            (val_to_pack_unsigned_24bit >> 8) & 0xFF,  # Mid
            val_to_pack_unsigned_24bit & 0xFF,         # LSB
        ]
        data_payload = speed_bytes + [acc_byte] + abs_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data_payload
        )

    async def stop_position_mode_absolute_axis(self, can_id: int, acceleration: int) -> int:
        """
        Stops the motor when in absolute axis position mode (uses Command 0xF5
        with speed 0 and absolute_axis 0).

        Args:
            can_id: The CAN ID of the target motor.
            acceleration: Deceleration parameter (0-255). 0 for immediate stop. [MKS Servo42D CAN Manual] (Page 50)

        Returns:
            The initial status code from the motor.

        Raises:
            ParameterError: If acceleration is out of range.
            CommunicationError, CommandError, CRCError, MotorError: On issues.
        """
        logger.info(
            f"LowLevelAPI.stop_position_mode_absolute_axis: CAN_ID={can_id:03X}, AccelParam={acceleration}"
        )
        if not (0 <= acceleration <= 255): # Accel param range [MKS Servo42D CAN Manual] (Page 50)
            raise ParameterError(f"Acceleration parameter {acceleration} out of range (0-255) for stop.")

        speed_bytes = [0x00, 0x00] # Speed = 0
        acc_byte = acceleration & 0xFF
        abs_axis_bytes = [0x00, 0x00, 0x00] # Absolute axis = 0
        data_payload = speed_bytes + [acc_byte] + abs_axis_bytes
        
        return await self._run_motor_command(
            can_id, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, data_payload
        )
        
    async def save_or_clean_speed_mode_params(self, can_id: int, save: bool) -> None:
        """
        Saves or cleans the parameters in speed mode (Command 0xFF).
        If saved, motor may run at constant speed on power-on.

        Args:
            can_id: The CAN ID of the target motor.
            save: True to save current speed mode parameters (action 0xC8), [MKS Servo42D CAN Manual] (Page 42)
                  False to clean them (action 0xCA). [MKS Servo42D CAN Manual] (Page 42)

        Raises:
            CommunicationError, CommandError, CRCError: On communication issues.
            MotorError: If the motor fails to save/clean parameters.
        """
        action_code = const.SPEED_MODE_PARAM_SAVE if save else const.SPEED_MODE_PARAM_CLEAN
        
        response = await self._send_command_and_get_response(
            can_id, const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS, data=[action_code], expected_dlc=3
        )
        # For CMD 0xFF, _send_command_and_get_response expects response.data[0] to be 0xFF.
        # The actual status is in response.data[1]. [MKS Servo42D CAN Manual] (Page 42, Uplink frame: byte1=code(FF), byte2=status)
        if response.data[1] != const.STATUS_SUCCESS: # status = 1 for success [MKS Servo42D CAN Manual] (Page 42)
            action_str = "Save" if save else "Clean"
            raise MotorError(
                f"{action_str} speed mode parameters failed for CAN ID {can_id}. Status: {response.data[1]}",
                error_code=response.data[1],
                can_id=can_id,
            )
        logger.info(f"CAN ID {can_id}: {'Save' if save else 'Clean'} speed mode parameters successful.")
