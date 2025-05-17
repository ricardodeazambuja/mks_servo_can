# mks_servo_can_project/mks_servo_simulator/mks_simulator/motor_model.py
"""
Simulated Motor Model for MKS SERVO42D/57D.
Models the internal state and behavior of a motor responding to CAN commands.
"""
from typing import Callable, List, Optional, Tuple

import asyncio
import logging
import struct
import time

# Attempt to import from the main library
try:
    from mks_servo_can_library.mks_servo_can import constants as const_module
    from mks_servo_can_library.mks_servo_can.crc import calculate_crc
    from mks_servo_can_library.mks_servo_can.exceptions import LimitError
    from mks_servo_can_library.mks_servo_can.exceptions import MKSServoError
    from mks_servo_can_library.mks_servo_can.exceptions import ParameterError

    const = const_module
    print(
        f"SIMULATOR: Successfully imported constants from mks_servo_can_library (e.g., const.MODE_SR_VFOC={getattr(const, 'MODE_SR_VFOC', 'Not Found')})."
    )
except ImportError:
    print(
        "SIMULATOR WARNING: Could not import from mks_servo_can_library. Using placeholder constants/crc."
    )

    class _ConstPlaceholder:
        MOTOR_STATUS_STOPPED = 1
        MOTOR_STATUS_SPEED_UP = 2
        MOTOR_STATUS_SPEED_DOWN = 3
        MOTOR_STATUS_FULL_SPEED = 4
        MOTOR_STATUS_HOMING = 5
        MOTOR_STATUS_CALIBRATING = 6
        POS_RUN_FAIL = 0
        POS_RUN_STARTING = 1
        POS_RUN_COMPLETE = 2
        POS_RUN_END_LIMIT_STOPPED = 3
        STATUS_SUCCESS = 1
        STATUS_FAILURE = 0
        STATUS_CALIBRATING_FAIL = 2
        MODE_CR_OPEN = 0
        MODE_CR_CLOSE = 1
        MODE_CR_VFOC = 2
        MODE_SR_OPEN = 3
        MODE_SR_CLOSE = 4
        MODE_SR_VFOC = 5
        MAX_RPM_OPEN_MODE = 400
        MAX_RPM_CLOSE_MODE = 1500
        MAX_RPM_VFOC_MODE = 3000
        ENCODER_PULSES_PER_REVOLUTION = 16384
        CMD_READ_ENCODER_ADDITION = 0x31
        CMD_READ_MOTOR_SPEED_RPM = 0x32
        CMD_READ_EN_PIN_STATUS = 0x3A  # Added missing constant
        CMD_SET_CURRENT_AXIS_TO_ZERO = 0x92  # Added for fix
        CMD_QUERY_MOTOR_STATUS = 0xF1
        CMD_SET_WORK_MODE = 0x82
        CMD_ENABLE_MOTOR = 0xF3
        CMD_RUN_SPEED_MODE = 0xF6
        CMD_RUN_POSITION_MODE_RELATIVE_PULSES = 0xFD
        CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES = 0xFE
        CMD_RUN_POSITION_MODE_RELATIVE_AXIS = 0xF4
        CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS = 0xF5
        CMD_EMERGENCY_STOP = 0xF7
        EN_ACTIVE_LOW = 0x00
        DIR_CW = 0x00

    const = _ConstPlaceholder()

    def calculate_crc(can_id: int, data_bytes: list[int]) -> int:
        checksum = can_id
        for byte_val in data_bytes:
            checksum += byte_val
        return checksum & 0xFF

    class ParameterError(Exception):
        pass

    class LimitError(Exception):
        pass

    class MKSServoError(Exception):
        pass


logger = logging.getLogger(__name__)
SIM_TIME_STEP_MS = 10
SIM_MAX_SPEED_PARAM = 3000
SIM_MAX_ACCEL_PARAM = 255


def mks_speed_param_to_rpm(param: int, mode: int = const.MODE_SR_VFOC) -> float:
    max_rpm_for_mode = const.MAX_RPM_VFOC_MODE
    if mode in [const.MODE_SR_OPEN, const.MODE_CR_OPEN]:
        max_rpm_for_mode = const.MAX_RPM_OPEN_MODE
    elif mode in [const.MODE_SR_CLOSE, const.MODE_CR_CLOSE]:
        max_rpm_for_mode = const.MAX_RPM_CLOSE_MODE
    if SIM_MAX_SPEED_PARAM == 0:
        return 0.0
    return (param / SIM_MAX_SPEED_PARAM) * max_rpm_for_mode


def mks_accel_param_to_rpm_per_sec_sq(
    param: int, current_rpm: float, target_rpm: float
) -> float:
    if param == 0:
        return float("inf")
    if param > SIM_MAX_ACCEL_PARAM:
        param = SIM_MAX_ACCEL_PARAM
    time_for_1_rpm_change_sec = (256 - param) * 50e-6
    if time_for_1_rpm_change_sec <= 0:
        return float("inf")
    return 1.0 / time_for_1_rpm_change_sec


class SimulatedMotor:
    def __init__(
        self,
        can_id: int,
        loop: asyncio.AbstractEventLoop,
        motor_type: str = "SERVO42D",
        initial_pos_steps: int = 0,
        steps_per_rev_encoder: int = const.ENCODER_PULSES_PER_REVOLUTION,
        min_pos_limit_steps: Optional[int] = None,
        max_pos_limit_steps: Optional[int] = None,
    ):
        self.can_id = can_id
        self.motor_type = motor_type
        self._loop = loop
        self.is_running_task: Optional[asyncio.Task] = None
        self.position_steps: float = float(initial_pos_steps)
        self.target_position_steps: Optional[float] = None
        self.steps_per_rev_encoder = steps_per_rev_encoder
        self.current_rpm: float = 0.0
        self.target_rpm: float = 0.0
        self.target_accel_mks: int = 100
        self.work_mode: int = const.MODE_SR_VFOC
        self.is_enabled: bool = False  # Start disabled, tests will enable it.
        self.is_calibrated: bool = True
        self.is_homed: bool = False
        self.stall_protection_enabled: bool = False
        self.is_stalled: bool = False
        self.min_pos_limit_steps = min_pos_limit_steps
        self.max_pos_limit_steps = max_pos_limit_steps
        self.motor_status_code: int = const.MOTOR_STATUS_STOPPED
        self._last_update_time: float = time.monotonic()
        self._current_move_task: Optional[asyncio.Future] = None
        self._current_move_command_code: Optional[int] = None
        self._send_completion_callback: Optional[
            Callable[[int, bytes], asyncio.Task]
        ] = None
        self.working_current_ma: int = 1600
        self.holding_current_percentage_code: int = 0x04
        self.microsteps: int = 16
        self.en_pin_active_level: int = const.EN_ACTIVE_LOW
        self.motor_direction_setting: int = const.DIR_CW
        logger.info(
            f"SimulatedMotor CAN ID {self.can_id:03X} initialized. Pos: {self.position_steps} steps. Work Mode: {self.work_mode}. Enabled: {self.is_enabled}"
        )

    def _generate_response(
        self, request_command_code: int, data: List[int]
    ) -> Tuple[int, bytes]:
        payload_with_cmd_echo = [request_command_code] + data
        crc = calculate_crc(self.can_id, payload_with_cmd_echo)
        return self.can_id, bytes(payload_with_cmd_echo + [crc])

    def _generate_simple_status_response(
        self, command_code: int, success: bool = True
    ) -> Tuple[int, bytes]:
        status = const.STATUS_SUCCESS if success else const.STATUS_FAILURE
        return self._generate_response(command_code, [status])

    async def _send_completion_if_callback(
        self, command_code: Optional[int], status_byte: int
    ):  # Made command_code optional
        if self._send_completion_callback and command_code is not None:
            logger.debug(
                f"Motor {self.can_id}: Sending async completion for CMD {command_code:02X} with status {status_byte:02X}"
            )
            _id, response_can_payload = self._generate_response(
                command_code, [status_byte]
            )
            # Ensure the callback is awaited if it's an async function that returns a task
            # For now, just create_task as before.
            self._loop.create_task(
                self._send_completion_callback(
                    self.can_id, response_can_payload
                )
            )

    async def _update_state(self):
        while True:
            await asyncio.sleep(SIM_TIME_STEP_MS / 1000.0)
            current_time = time.monotonic()
            delta_t = current_time - self._last_update_time
            if delta_t <= 0:
                self._last_update_time = current_time
                continue
            self._last_update_time = current_time

            if not self.is_enabled or self.is_stalled:
                if self.current_rpm != 0.0:
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    if (
                        self._current_move_task
                        and not self._current_move_task.done()
                    ):
                        logger.warning(
                            f"Motor {self.can_id}: Move interrupted by disable/stall."
                        )
                        await self._send_completion_if_callback(
                            self._current_move_command_code, const.POS_RUN_FAIL
                        )
                        self._current_move_task.set_exception(
                            MKSServoError(
                                f"Move failed due to motor disable/stall for motor {self.can_id}"
                            )
                        )
                        self.target_position_steps = None
                continue

            if self.current_rpm != self.target_rpm:
                accel_rate_rpm_s = mks_accel_param_to_rpm_per_sec_sq(
                    self.target_accel_mks, self.current_rpm, self.target_rpm
                )
                if accel_rate_rpm_s == float("inf"):
                    self.current_rpm = self.target_rpm
                else:
                    rpm_change = accel_rate_rpm_s * delta_t
                    if self.target_rpm > self.current_rpm:
                        self.current_rpm = min(
                            self.target_rpm, self.current_rpm + rpm_change
                        )
                    else:
                        self.current_rpm = max(
                            self.target_rpm, self.current_rpm - rpm_change
                        )
                if abs(self.current_rpm - self.target_rpm) < 0.01:
                    self.current_rpm = self.target_rpm

            if (
                self.current_rpm == 0
                and self.target_rpm == 0
                and self.target_position_steps is None
            ):
                self.motor_status_code = const.MOTOR_STATUS_STOPPED
            elif self.current_rpm != self.target_rpm:
                self.motor_status_code = (
                    const.MOTOR_STATUS_SPEED_UP
                    if self.target_rpm > self.current_rpm
                    else const.MOTOR_STATUS_SPEED_DOWN
                )
            elif self.current_rpm != 0:
                self.motor_status_code = const.MOTOR_STATUS_FULL_SPEED

            if self.current_rpm != 0:
                steps_change = (
                    (self.current_rpm / 60.0)
                    * self.steps_per_rev_encoder
                    * delta_t
                )
                new_pos = self.position_steps + steps_change
                limit_hit_type = None
                if (
                    self.min_pos_limit_steps is not None
                    and new_pos < self.min_pos_limit_steps
                ):
                    new_pos = float(self.min_pos_limit_steps)
                    limit_hit_type = "min"
                if (
                    self.max_pos_limit_steps is not None
                    and new_pos > self.max_pos_limit_steps
                ):
                    new_pos = float(self.max_pos_limit_steps)
                    limit_hit_type = "max"
                self.position_steps = new_pos
                if limit_hit_type:
                    logger.warning(
                        f"Motor {self.can_id}: Soft limit '{limit_hit_type}' hit at {self.position_steps:.2f} steps."
                    )
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    if self.target_position_steps is not None:
                        await self._send_completion_if_callback(
                            self._current_move_command_code,
                            const.POS_RUN_END_LIMIT_STOPPED,
                        )
                        if (
                            self._current_move_task
                            and not self._current_move_task.done()
                        ):
                            self._current_move_task.set_exception(
                                LimitError(
                                    f"Simulated limit hit for motor {self.can_id}"
                                )
                            )
                        self.target_position_steps = None

            if self.target_position_steps is not None:
                is_moving_positive = self.current_rpm > 0
                is_moving_negative = self.current_rpm < 0
                target_reached = False
                if (
                    is_moving_positive
                    and self.position_steps >= self.target_position_steps
                ):
                    target_reached = True
                elif (
                    is_moving_negative
                    and self.position_steps <= self.target_position_steps
                ):
                    target_reached = True
                elif (
                    abs(self.current_rpm) < 0.1
                    and abs(self.position_steps - self.target_position_steps)
                    < 1.0
                ):
                    target_reached = True
                if target_reached:
                    logger.info(
                        f"Motor {self.can_id}: Target position {self.target_position_steps:.2f} reached. Current: {self.position_steps:.2f}"
                    )
                    self.position_steps = float(self.target_position_steps)
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    self.target_position_steps = None
                    await self._send_completion_if_callback(
                        self._current_move_command_code, const.POS_RUN_COMPLETE
                    )
                    if (
                        self._current_move_task
                        and not self._current_move_task.done()
                    ):
                        self._current_move_task.set_result(True)

    async def _handle_positional_move(
        self,
        target_pos_abs_steps: float,
        speed_mks: int,
        accel_mks: int,
        command_code: int,
    ):
        if self._current_move_task and not self._current_move_task.done():
            logger.warning(
                f"Motor {self.can_id}: Cancelling previous move to start new one."
            )
            await self._send_completion_if_callback(
                self._current_move_command_code, const.POS_RUN_FAIL
            )  # Fail previous
            self._current_move_task.cancel("Superseded by new move command")

        self.target_position_steps = target_pos_abs_steps
        delta_pos = target_pos_abs_steps - self.position_steps
        if abs(delta_pos) < 0.5:
            logger.info(
                f"Motor {self.can_id}: Already at target position {target_pos_abs_steps:.2f}."
            )
            self.current_rpm = 0.0
            self.target_rpm = 0.0
            self.motor_status_code = const.MOTOR_STATUS_STOPPED
            self.target_position_steps = None
            await self._send_completion_if_callback(
                command_code, const.POS_RUN_COMPLETE
            )
            # Set a new future and immediately resolve it for the calling _execute_move
            self._current_move_task = self._loop.create_future()
            self._current_move_task.set_result(True)
            return

        target_speed_rpm_magnitude = mks_speed_param_to_rpm(
            speed_mks, self.work_mode
        )
        self.target_rpm = (
            target_speed_rpm_magnitude
            if delta_pos > 0
            else -target_speed_rpm_magnitude
        )
        self.target_accel_mks = accel_mks
        self._current_move_command_code = command_code
        self._current_move_task = self._loop.create_future()

        distance = abs(delta_pos)
        avg_speed_rpm = (
            target_speed_rpm_magnitude / 2.0
            if target_speed_rpm_magnitude > 0
            else 1.0
        )
        avg_speed_steps_per_sec = (
            avg_speed_rpm / 60.0
        ) * self.steps_per_rev_encoder
        est_duration = 1.0
        if avg_speed_steps_per_sec > 0:
            est_duration = (distance / avg_speed_steps_per_sec) + 1.0
        logger.info(
            f"Motor {self.can_id}: Positional move to {target_pos_abs_steps:.2f} (delta: {delta_pos:.2f}) initiated. Target RPM: {self.target_rpm:.2f}. Est. duration: {est_duration:.2f}s"
        )

        try:
            await asyncio.wait_for(
                self._current_move_task, timeout=est_duration + 2.0
            )
        except asyncio.TimeoutError:
            logger.warning(
                f"Motor {self.can_id}: Positional move future timed out (sim). Current: {self.position_steps:.2f}, Target: {self.target_position_steps}"
            )
            if self.target_position_steps is not None:
                await self._send_completion_if_callback(
                    self._current_move_command_code, const.POS_RUN_FAIL
                )
                self.target_position_steps = None
                self.target_rpm = 0.0
        except asyncio.CancelledError:
            logger.info(
                f"Motor {self.can_id}: Positional move future cancelled."
            )
        except Exception as e:
            logger.error(
                f"Motor {self.can_id}: Exception in positional move future: {e}"
            )
            if self.target_position_steps is not None:
                await self._send_completion_if_callback(
                    self._current_move_command_code, const.POS_RUN_FAIL
                )
                self.target_position_steps = None
                self.target_rpm = 0.0
        finally:
            if (
                self._current_move_task
                and self._current_move_task.done()
                or (
                    self._current_move_task
                    and self._current_move_task.cancelled()
                )
            ):  # Also clear if cancelled
                self._current_move_command_code = (
                    None  # Clear only if this command truly finished/cancelled
                )
            # Do not nullify _current_move_task here if it might be awaited by another part.
            # It's better if the task that creates it also clears its reference once done.

    def process_command(
        self,
        command_code: int,
        data_from_payload: bytes,
        send_completion_callback: Callable[[int, bytes], asyncio.Task],
    ) -> Optional[Tuple[int, bytes]]:
        self._send_completion_callback = send_completion_callback
        response_data_for_payload: Optional[List[int]] = None

        if command_code == const.CMD_READ_ENCODER_ADDITION:
            pos_bytes = bytearray(8)
            struct.pack_into(
                "<q", pos_bytes, 0, int(round(self.position_steps))
            )
            response_data_for_payload = list(pos_bytes[:6])
        elif command_code == const.CMD_READ_MOTOR_SPEED_RPM:
            response_data_for_payload = list(
                struct.pack("<h", int(round(self.current_rpm)))
            )
        elif command_code == const.CMD_READ_EN_PIN_STATUS:  # Handle 0x3A
            enable_status_byte = 0x01 if self.is_enabled else 0x00
            response_data_for_payload = [enable_status_byte]
            logger.debug(
                f"Motor {self.can_id}: Responding to CMD_READ_EN_PIN_STATUS (0x3A) with status: {enable_status_byte}"
            )
        elif command_code == const.CMD_QUERY_MOTOR_STATUS:
            response_data_for_payload = [self.motor_status_code]
        elif command_code == const.CMD_SET_WORK_MODE:
            if len(data_from_payload) >= 1:
                new_mode = data_from_payload[0]
                if 0 <= new_mode <= 5:
                    self.work_mode = new_mode
                    logger.info(
                        f"Motor {self.can_id}: Work mode set to {self.work_mode}"
                    )
                    return self._generate_simple_status_response(
                        command_code, True
                    )
            return self._generate_simple_status_response(command_code, False)
        elif command_code == const.CMD_SET_CURRENT_AXIS_TO_ZERO:  # Command 0x92
            logger.info(
                f"Motor {self.can_id}: Setting current position to zero (CMD 0x92)."
            )
            self.position_steps = 0.0
            self.is_homed = True  # Setting position to zero usually implies a known reference
            return self._generate_simple_status_response(command_code, True)
        elif command_code == const.CMD_ENABLE_MOTOR:
            if len(data_from_payload) >= 1:
                self.is_enabled = data_from_payload[0] == 0x01
                if not self.is_enabled:
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0
                if (
                    self._current_move_task
                    and not self._current_move_task.done()
                    and not self.is_enabled
                ):
                    self._current_move_task.cancel("Motor disabled during move")
                self.target_position_steps = (
                    None if not self.is_enabled else self.target_position_steps
                )
                logger.info(
                    f"Motor {self.can_id}: Enable set to {self.is_enabled}"
                )
                return self._generate_simple_status_response(command_code, True)
            return self._generate_simple_status_response(command_code, False)
        elif command_code == const.CMD_RUN_SPEED_MODE:
            if len(data_from_payload) >= 3:
                byte2, byte3, byte4 = (
                    data_from_payload[0],
                    data_from_payload[1],
                    data_from_payload[2],
                )
                mks_speed_param = ((byte2 & 0x0F) << 8) | byte3
                mks_accel_param = byte4
                calculated_target_rpm = mks_speed_param_to_rpm(
                    mks_speed_param, self.work_mode
                )
                self.target_rpm = (
                    calculated_target_rpm
                    if (byte2 & 0x80) == 0
                    else -calculated_target_rpm
                )
                self.target_accel_mks = mks_accel_param
                self.target_position_steps = None
                if (
                    self._current_move_task
                    and not self._current_move_task.done()
                ):
                    self._current_move_task.cancel("New speed command")
                logger.info(
                    f"Motor {self.can_id}: Speed mode. Target RPM: {self.target_rpm:.2f}, Accel Param: {self.target_accel_mks}"
                )
                return self._generate_response(
                    command_code, [const.POS_RUN_STARTING]
                )
            return self._generate_response(command_code, [const.POS_RUN_FAIL])
        elif command_code in [
            const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES,
            const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,
            const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS,
            const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
        ]:
            target_pos_abs_final: Optional[float] = None
            mks_speed_val: int = 0
            mks_accel_val: int = 0
            parsed_ok = False
            try:
                if command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES:
                    if len(data_from_payload) >= 6:
                        b2, b3, b4 = (
                            data_from_payload[0],
                            data_from_payload[1],
                            data_from_payload[2],
                        )
                        pB = bytes(data_from_payload[3:6]) + b"\x00"
                        nP = struct.unpack("<I", pB)[0]
                        is_ccw = (b2 & 0x80) == 0
                        mks_speed_val = ((b2 & 0x0F) << 8) | b3
                        mks_accel_val = b4
                        delta = float(nP) if is_ccw else -float(nP)
                        target_pos_abs_final = self.position_steps + delta
                        parsed_ok = True
                elif (
                    command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES
                ):
                    if len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(
                            "<H", bytes(data_from_payload[0:2])
                        )[0]
                        mks_accel_val = data_from_payload[2]
                        apB = bytes(data_from_payload[3:6])
                        if apB[2] & 0x80:
                            val_u = apB[0] | (apB[1] << 8) | (apB[2] << 16)
                            target_pos_abs_final = float(val_u - (1 << 24))
                        else:
                            target_pos_abs_final = float(
                                struct.unpack("<I", apB + b"\x00")[0] & 0xFFFFFF
                            )
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS:
                    if len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(
                            "<H", bytes(data_from_payload[0:2])
                        )[0]
                        mks_accel_val = data_from_payload[2]
                        raB = bytes(data_from_payload[3:6])
                        val_s24 = 0.0
                        if raB[2] & 0x80:
                            val_u = raB[0] | (raB[1] << 8) | (raB[2] << 16)
                            val_s24 = float(val_u - (1 << 24))
                        else:
                            val_s24 = float(
                                struct.unpack("<I", raB + b"\x00")[0] & 0xFFFFFF
                            )
                        target_pos_abs_final = self.position_steps + val_s24
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS:
                    if len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(
                            "<H", bytes(data_from_payload[0:2])
                        )[0]
                        mks_accel_val = data_from_payload[2]
                        aaB = bytes(data_from_payload[3:6])
                        if aaB[2] & 0x80:
                            val_u = aaB[0] | (aaB[1] << 8) | (aaB[2] << 16)
                            target_pos_abs_final = float(val_u - (1 << 24))
                        else:
                            target_pos_abs_final = float(
                                struct.unpack("<I", aaB + b"\x00")[0] & 0xFFFFFF
                            )
                        parsed_ok = True
            except struct.error as e:
                logger.error(
                    f"Motor {self.can_id}: Parse error CMD {command_code:02X}: {e}"
                )
                return self._generate_response(
                    command_code, [const.POS_RUN_FAIL]
                )
            if parsed_ok and target_pos_abs_final is not None:
                self._loop.create_task(
                    self._handle_positional_move(
                        target_pos_abs_final,
                        mks_speed_val,
                        mks_accel_val,
                        command_code,
                    )
                )
                return self._generate_response(
                    command_code, [const.POS_RUN_STARTING]
                )
            else:
                logger.warning(
                    f"Motor {self.can_id}: Parse fail CMD {command_code:02X}. Data: {data_from_payload.hex()}"
                )
                return self._generate_response(
                    command_code, [const.POS_RUN_FAIL]
                )
        elif command_code == const.CMD_EMERGENCY_STOP:
            self.current_rpm = 0.0
            self.target_rpm = 0.0
            self.target_position_steps = None
            if self._current_move_task and not self._current_move_task.done():
                self._current_move_task.cancel("Emergency stop")
            logger.warning(f"Motor {self.can_id}: Emergency stop processed.")
            return self._generate_simple_status_response(command_code, True)

        if response_data_for_payload is not None:
            return self._generate_response(
                command_code, response_data_for_payload
            )
        else:
            logger.warning(
                f"Motor {self.can_id}: Unhandled CMD 0x{command_code:02X}, data: {data_from_payload.hex()}. No direct response."
            )
            return None

    async def start(self):
        if self.is_running_task and not self.is_running_task.done():
            logger.warning(f"Motor {self.can_id} sim task running.")
            return
        self._last_update_time = time.monotonic()
        self.is_running_task = self._loop.create_task(self._update_state())
        logger.info(f"SimulatedMotor {self.can_id} update task started.")

    async def stop_simulation(self):
        if self._current_move_task and not self._current_move_task.done():
            self._current_move_task.cancel("Sim stopping")
            await asyncio.gather(
                self._current_move_task, return_exceptions=True
            )
        self._current_move_task = None
        if self.is_running_task and not self.is_running_task.done():
            self.is_running_task.cancel()
            await asyncio.gather(self.is_running_task, return_exceptions=True)
        self.is_running_task = None
        logger.info(f"SimulatedMotor {self.can_id} update task stopped.")
