# mks_servo_can_project/mks_servo_simulator/mks_simulator/motor_model.py
"""
Simulated Motor Model for MKS SERVO42D/57D.
Models the internal state and behavior of a motor responding to CAN commands.
"""
from typing import Callable, List, Optional, Tuple, Dict, Any

import asyncio
import logging
import struct
import time

logger = logging.getLogger("SimulatedMotor") # Changed from __name__ for clarity if file is moved/copied

# Attempt to import from the main library
try:
    from mks_servo_can import constants as const_module
    from mks_servo_can.crc import calculate_crc
    # Assuming exceptions are not directly raised by the simulator to the lib,
    # but used for internal logic if needed.
    # from mks_servo_can.exceptions import ParameterError

    const = const_module
except ImportError:
    logger.warning(
        "SIMULATOR WARNING: Could not import from mks_servo_can. Using placeholder constants/crc. Ensure library is installed or in PYTHONPATH."
    )

    class _ConstPlaceholder:
        # Part 5.1 - Read Status
        CMD_READ_ENCODER_CARRY = 0x30
        CMD_READ_ENCODER_ADDITION = 0x31
        CMD_READ_MOTOR_SPEED_RPM = 0x32
        CMD_READ_PULSES_RECEIVED = 0x33
        CMD_READ_IO_STATUS = 0x34
        CMD_READ_RAW_ENCODER_ADDITION = 0x35
        CMD_READ_SHAFT_ANGLE_ERROR = 0x39
        CMD_READ_EN_PIN_STATUS = 0x3A
        CMD_READ_POWER_ON_ZERO_STATUS = 0x3B
        CMD_RELEASE_STALL_PROTECTION = 0x3D
        CMD_READ_MOTOR_PROTECTION_STATE = 0x3E

        # Part 5.2 - Set System Parameters
        CMD_CALIBRATE_ENCODER = 0x80
        CMD_SET_WORK_MODE = 0x82
        CMD_SET_WORKING_CURRENT = 0x83
        CMD_SET_SUBDIVISION = 0x84
        CMD_SET_EN_PIN_ACTIVE_LEVEL = 0x85
        CMD_SET_MOTOR_DIRECTION = 0x86
        CMD_SET_AUTO_SCREEN_OFF = 0x87
        CMD_SET_STALL_PROTECTION = 0x88
        CMD_SET_SUBDIVISION_INTERPOLATION = 0x89
        CMD_SET_CAN_BITRATE = 0x8A
        CMD_SET_CAN_ID = 0x8B
        CMD_SET_SLAVE_RESPOND_ACTIVE = 0x8C
        CMD_SET_GROUP_ID = 0x8D
        CMD_SET_KEY_LOCK = 0x8F
        CMD_SET_HOLDING_CURRENT_PERCENTAGE = 0x9B

        # Part 5.3 - Write IO Port
        CMD_WRITE_IO_PORT = 0x36

        # Part 5.4 - Set Home Command
        CMD_SET_HOME_PARAMETERS = 0x90
        CMD_GO_HOME = 0x91
        CMD_SET_CURRENT_AXIS_TO_ZERO = 0x92
        CMD_SET_NOLIMIT_HOME_PARAMS = 0x94
        CMD_SET_LIMIT_PORT_REMAP = 0x9E

        # Part 5.5 - Set 0_Mode Command
        CMD_SET_ZERO_MODE_PARAMETERS = 0x9A

        # Part 5.6 - Restore Default Parameters
        CMD_RESTORE_DEFAULT_PARAMETERS = 0x3F

        # Part 5.7 - Restart Motor
        CMD_RESTART_MOTOR = 0x41
        
        # Part 5.8 - En Triggers and Position Error Protection
        CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION = 0x9D

        # Part 5.9 - Read System Parameter
        CMD_READ_SYSTEM_PARAMETER_PREFIX = 0x00

        # Part 6 - Run Motor Commands
        CMD_QUERY_MOTOR_STATUS = 0xF1
        CMD_ENABLE_MOTOR = 0xF3
        CMD_EMERGENCY_STOP = 0xF7
        CMD_RUN_SPEED_MODE = 0xF6
        CMD_SAVE_CLEAN_SPEED_MODE_PARAMS = 0xFF
        SPEED_MODE_PARAM_SAVE = 0xC8
        SPEED_MODE_PARAM_CLEAN = 0xCA
        CMD_RUN_POSITION_MODE_RELATIVE_PULSES = 0xFD
        CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES = 0xFE
        CMD_RUN_POSITION_MODE_RELATIVE_AXIS = 0xF4
        CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS = 0xF5

        # Statuses
        STATUS_SUCCESS = 0x01
        STATUS_FAILURE = 0x00
        STATUS_CALIBRATING = 0x00
        STATUS_CALIBRATED_SUCCESS = 0x01
        STATUS_CALIBRATING_FAIL = 0x02
        
        MOTOR_STATUS_QUERY_FAIL = 0x00
        MOTOR_STATUS_STOPPED = 0x01
        MOTOR_STATUS_SPEED_UP = 0x02
        MOTOR_STATUS_SPEED_DOWN = 0x03
        MOTOR_STATUS_FULL_SPEED = 0x04
        MOTOR_STATUS_HOMING = 0x05
        MOTOR_STATUS_CALIBRATING = 0x06 # Matches STATUS_CALIBRATING

        POS_RUN_FAIL = 0x00
        POS_RUN_STARTING = 0x01
        POS_RUN_COMPLETE = 0x02
        POS_RUN_END_LIMIT_STOPPED = 0x03

        HOME_FAIL = 0x00
        HOME_START = 0x01
        HOME_SUCCESS = 0x02
        
        MODE_CR_OPEN = 0
        MODE_SR_VFOC = 5
        MAX_RPM_OPEN_MODE = 400
        MAX_RPM_CLOSE_MODE = 1500
        MAX_RPM_VFOC_MODE = 3000
        ENCODER_PULSES_PER_REVOLUTION = 16384
        EN_ACTIVE_LOW = 0x00
        EN_ACTIVE_HIGH = 0x01
        EN_ACTIVE_ALWAYS = 0x02
        DIR_CW = 0x00
        DIR_CCW = 0x01
        CAN_BITRATE_500K = 0x02


    const = _ConstPlaceholder()

    def calculate_crc(can_id: int, data_bytes: list[int]) -> int:
        checksum = can_id
        for byte_val in data_bytes:
            checksum += byte_val
        return checksum & 0xFF

    class ParameterError(Exception): # Basic fallback
        pass
    class LimitError(Exception): # Basic fallback
        pass
    class MKSServoError(Exception): # Basic fallback
        pass

SIM_TIME_STEP_MS = 10
SIM_MAX_SPEED_PARAM = 3000 # Used for RPM conversion, matches VFOC for SR_VFOC
SIM_MAX_ACCEL_PARAM = 255


def mks_speed_param_to_rpm(param: int, mode: int = const.MODE_SR_VFOC) -> float:
    max_rpm_for_mode = const.MAX_RPM_VFOC_MODE # Default
    if mode in [const.MODE_CR_OPEN, getattr(const, 'MODE_SR_OPEN', -1)]: # Check if SR_OPEN exists
        max_rpm_for_mode = const.MAX_RPM_OPEN_MODE
    elif mode in [getattr(const, 'MODE_CR_CLOSE', -1), getattr(const, 'MODE_SR_CLOSE', -1)]:
        max_rpm_for_mode = const.MAX_RPM_CLOSE_MODE
    
    if SIM_MAX_SPEED_PARAM == 0: # Avoid division by zero
        return 0.0
    # The speed parameter (0-3000) maps to the max RPM of the current mode
    return (param / SIM_MAX_SPEED_PARAM) * max_rpm_for_mode


def mks_accel_param_to_rpm_per_sec_sq(
    param: int, current_rpm: float, target_rpm: float
) -> float:
    if param == 0: # Instantaneous
        return float("inf") 
    if param > SIM_MAX_ACCEL_PARAM:
        param = SIM_MAX_ACCEL_PARAM
    
    # Time for 1 RPM change = (256 - acc_param) * 50 us
    # So, RPMs per 1 second = 1 / ((256 - acc_param) * 50e-6)
    time_for_1_rpm_change_sec = (256 - param) * 50e-6
    if time_for_1_rpm_change_sec <= 1e-9: # effectively zero or negative, treat as infinite accel
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
        self.can_id = can_id  # This is the ID the motor listens to
        self.original_can_id = can_id # To respond from, even if self.can_id is changed by 0x8B
        self.motor_type = motor_type
        self._loop = loop
        self.is_running_task: Optional[asyncio.Task] = None
        
        # Core state
        self.position_steps: float = float(initial_pos_steps)
        self.target_position_steps: Optional[float] = None
        self.steps_per_rev_encoder = steps_per_rev_encoder
        self.current_rpm: float = 0.0
        self.target_rpm: float = 0.0
        self.current_accel_rpm_per_sec_sq: float = 0.0 # For _update_state
        self.target_accel_mks: int = 100 # Default MKS acceleration parameter
        
        self.is_enabled: bool = False
        self.motor_status_code: int = const.MOTOR_STATUS_STOPPED
        self._last_update_time: float = time.monotonic()
        self._current_move_task: Optional[asyncio.Future] = None
        self._current_move_command_code: Optional[int] = None
        self._send_completion_callback: Optional[
            Callable[[int, bytes], asyncio.Task]
        ] = None

        # Settable Parameters (with defaults from manual or common sense)
        self.work_mode: int = const.MODE_SR_VFOC # Defaulting to serial FOC for simulator
        self.working_current_ma: int = 1600 if "42D" in motor_type else 3200
        self.holding_current_percentage_code: int = 0x04 # 50% (index 4 for 0-8 range -> 10% to 90%)
        self.microsteps: int = 16 # Subdivision
        self.en_pin_active_level: int = const.EN_ACTIVE_LOW
        self.motor_direction_setting: int = const.DIR_CW # For pulse mode, CAN direction is per command
        self.auto_screen_off_enabled: bool = False
        self.stall_protection_enabled: bool = False
        self.subdivision_interpolation_enabled: bool = True
        self.can_bitrate_code: int = const.CAN_BITRATE_500K # Store the code
        self.slave_respond_enabled: bool = True
        self.slave_active_initiation_enabled: bool = True
        self.group_id: int = 0x00 # Default to no specific group or use its own ID? Manual implies can be set.
        self.is_key_locked: bool = False
        
        self.io_out1_value: int = 0
        self.io_out2_value: int = 0 # Note: OUT2 N/A for 42D

        # Homing parameters
        self.home_trig_level: int = 0 # Low
        self.home_dir: int = const.DIR_CW
        self.home_speed_rpm: int = 60
        self.end_limit_enabled_setting: bool = False
        self.home_mode_setting: int = 0 # 0: Use Limit Switch
        self.nolimit_home_reverse_angle: int = 0x2000 # 180 deg default
        self.nolimit_home_current_ma: int = 800 if "42D" in motor_type else 400

        self.limit_port_remap_enabled: bool = False

        # 0_Mode (Power-on auto zero) parameters
        self.zero_mode_behavior: int = 0 # Disable
        self.zero_mode_set_zero_action: int = 2 # No modify
        self.zero_mode_speed_code: int = 2 # Medium speed
        self.zero_mode_direction: int = const.DIR_CW
        self.power_on_zero_status: int = const.HOME_SUCCESS # Assume done or N/A if disabled

        # En Trigger & Pos Error Protection
        self.enable_en_trigger_zero: bool = False
        self.enable_pos_error_protection: bool = False
        self.error_detection_time_ms_units: int = 100 # ~1.5s
        self.error_threshold_pulses: int = 2800 # ~20 degrees for 16384 PPR

        self.is_calibrated: bool = True # Assume calibrated for simulator start
        self.is_homed: bool = False
        self.is_stalled: bool = False # Internal stall state
        self.is_protected_by_stall: bool = False # If stall protection has triggered
        self.is_protected_by_pos_error: bool = False

        self.saved_speed_mode_active: bool = False # For 0xFF command
        self.saved_speed_mode_params: Optional[Dict[str, Any]] = None


        logger.info(
            f"SimulatedMotor CAN ID {self.can_id:03X} initialized. Pos: {self.position_steps} steps."
        )

    def _generate_response(
        self, request_command_code: int, data: List[int]
    ) -> Tuple[int, bytes]:
        # Always respond with the original CAN ID the command was sent to,
        # even if self.can_id (the listening ID) has been "changed" by 0x8B.
        # The virtual CAN bus routes messages to this motor instance based on its current self.can_id.
        payload_with_cmd_echo = [request_command_code] + data
        crc = calculate_crc(self.original_can_id, payload_with_cmd_echo)
        return self.original_can_id, bytes(payload_with_cmd_echo + [crc])

    def _generate_simple_status_response(
        self, command_code: int, success: bool = True
    ) -> Tuple[int, bytes]:
        status = const.STATUS_SUCCESS if success else const.STATUS_FAILURE
        return self._generate_response(command_code, [status])

    async def _send_completion_if_callback(
        self, command_code: Optional[int], status_byte: int
    ):
        if self._send_completion_callback and command_code is not None and self.slave_active_initiation_enabled:
            logger.debug(
                f"Motor {self.original_can_id}: Sending async completion for CMD {command_code:02X} with status {status_byte:02X}"
            )
            _id, response_can_payload = self._generate_response(
                command_code, [status_byte]
            )
            # Call the callback without creating a new task if it's already async
            await self._send_completion_callback(
                self.original_can_id, response_can_payload
            )
            
    def _pack_int24(self, value: int) -> List[int]:
        """Packs a signed 24-bit integer into 3 bytes, little-endian."""
        unsigned_val = value & 0xFFFFFF
        return [
            unsigned_val & 0xFF,
            (unsigned_val >> 8) & 0xFF,
            (unsigned_val >> 16) & 0xFF,
        ]

    def _get_param_data_bytes(self, param_cmd_code: int) -> Optional[List[int]]:
        """ Helper to get data bytes for read_system_parameter. """
        if param_cmd_code == const.CMD_SET_WORK_MODE: return [self.work_mode]
        if param_cmd_code == const.CMD_SET_WORKING_CURRENT: return list(struct.pack("<H", self.working_current_ma))
        if param_cmd_code == const.CMD_SET_SUBDIVISION: return [self.microsteps]
        if param_cmd_code == const.CMD_SET_EN_PIN_ACTIVE_LEVEL: return [self.en_pin_active_level]
        if param_cmd_code == const.CMD_SET_MOTOR_DIRECTION: return [self.motor_direction_setting]
        if param_cmd_code == const.CMD_SET_AUTO_SCREEN_OFF: return [0x01 if self.auto_screen_off_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_STALL_PROTECTION: return [0x01 if self.stall_protection_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_SUBDIVISION_INTERPOLATION: return [0x01 if self.subdivision_interpolation_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_CAN_BITRATE: return [self.can_bitrate_code]
        if param_cmd_code == const.CMD_SET_CAN_ID: return list(struct.pack("<H", self.can_id)) # Current listening ID
        if param_cmd_code == const.CMD_SET_SLAVE_RESPOND_ACTIVE:
            return [0x01 if self.slave_respond_enabled else 0x00, 0x01 if self.slave_active_initiation_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_GROUP_ID: return list(struct.pack("<H", self.group_id))
        if param_cmd_code == const.CMD_SET_KEY_LOCK: return [0x01 if self.is_key_locked else 0x00]
        if param_cmd_code == const.CMD_SET_HOLDING_CURRENT_PERCENTAGE: return [self.holding_current_percentage_code]
        # Add more readable parameters here as needed
        logger.warning(f"Simulator: Parameter 0x{param_cmd_code:02X} not implemented for reading via 0x00.")
        return None


    async def _update_state(self):
        # (Existing _update_state logic - mostly unchanged for now unless new commands affect continuous behavior)
        # This loop primarily handles motion. Other state changes are discrete.
        while True:
            await asyncio.sleep(SIM_TIME_STEP_MS / 1000.0)
            current_time = time.monotonic()
            delta_t = current_time - self._last_update_time
            if delta_t <= 0:
                self._last_update_time = current_time
                continue
            self._last_update_time = current_time

            if not self.is_enabled or self.is_stalled or self.is_protected_by_stall or self.is_protected_by_pos_error:
                if self.current_rpm != 0.0:
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0 # Also reset target RPM
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    if self._current_move_task and not self._current_move_task.done():
                        logger.warning(f"Motor {self.original_can_id}: Move interrupted by disable/stall/protection.")
                        await self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL)
                        self._current_move_task.set_exception(MKSServoError(f"Move failed due to motor disable/stall/protection for motor {self.original_can_id}"))
                        self.target_position_steps = None
                continue
            
            # Acceleration/Deceleration
            if self.current_rpm != self.target_rpm:
                self.current_accel_rpm_per_sec_sq = mks_accel_param_to_rpm_per_sec_sq(
                    self.target_accel_mks, self.current_rpm, self.target_rpm
                )
                if self.current_accel_rpm_per_sec_sq == float("inf"): # Instantaneous
                    self.current_rpm = self.target_rpm
                else:
                    rpm_change = self.current_accel_rpm_per_sec_sq * delta_t
                    if self.target_rpm > self.current_rpm:
                        self.current_rpm = min(self.target_rpm, self.current_rpm + rpm_change)
                        self.motor_status_code = const.MOTOR_STATUS_SPEED_UP
                    else: # target_rpm < self.current_rpm
                        self.current_rpm = max(self.target_rpm, self.current_rpm - rpm_change)
                        self.motor_status_code = const.MOTOR_STATUS_SPEED_DOWN
                
                if abs(self.current_rpm - self.target_rpm) < 0.1: # Close enough
                    self.current_rpm = self.target_rpm
                    if self.target_rpm == 0 and self.target_position_steps is None: # Stopped in speed mode
                         self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    elif self.target_rpm != 0 :
                         self.motor_status_code = const.MOTOR_STATUS_FULL_SPEED


            elif self.current_rpm == 0 and self.target_position_steps is None: # Idle
                self.motor_status_code = const.MOTOR_STATUS_STOPPED
            elif self.current_rpm !=0 and self.target_position_steps is None: # Running at speed
                 self.motor_status_code = const.MOTOR_STATUS_FULL_SPEED


            # Position update
            if self.current_rpm != 0:
                steps_change = (self.current_rpm / 60.0) * self.steps_per_rev_encoder * delta_t
                self.position_steps += steps_change
                # Soft limits (if enabled) could be checked here

            # Target position reaching logic
            if self.target_position_steps is not None:
                # Simplified check: if moving towards target and passed it, or very close
                is_moving_positive = self.current_rpm > 0
                is_moving_negative = self.current_rpm < 0
                target_reached = False

                if is_moving_positive and self.position_steps >= self.target_position_steps:
                    target_reached = True
                elif is_moving_negative and self.position_steps <= self.target_position_steps:
                    target_reached = True
                elif abs(self.position_steps - self.target_position_steps) < 1.0 : # Close enough
                    if abs(self.current_rpm) < 1.0: # And nearly stopped
                        target_reached = True
                
                if target_reached:
                    logger.info(
                        f"Motor {self.original_can_id}: Target position {self.target_position_steps:.2f} reached. Current: {self.position_steps:.2f}"
                    )
                    self.position_steps = float(self.target_position_steps) # Snap to target
                    self.current_rpm = 0.0
                    self.target_rpm = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    
                    if self._current_move_task and not self._current_move_task.done():
                        self._current_move_task.set_result(True)
                    await self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_COMPLETE)
                    self.target_position_steps = None # Clear target
                    # self._current_move_command_code = None # Cleared in _handle_positional_move


    async def _handle_positional_move(
        self,
        target_pos_abs_steps: float,
        speed_mks: int,
        accel_mks: int,
        command_code: int,
    ):
        # (Existing _handle_positional_move logic - largely unchanged but uses original_can_id for logging)
        if self._current_move_task and not self._current_move_task.done():
            logger.warning(f"Motor {self.original_can_id}: Cancelling previous move for new one.")
            await self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL)
            self._current_move_task.cancel("Superseded by new move command")

        self.target_position_steps = target_pos_abs_steps
        delta_pos = target_pos_abs_steps - self.position_steps

        if abs(delta_pos) < 0.5:
            logger.info(f"Motor {self.original_can_id}: Already at target {target_pos_abs_steps:.2f}.")
            self.position_steps = target_pos_abs_steps
            self.current_rpm = 0.0
            self.target_rpm = 0.0
            self.motor_status_code = const.MOTOR_STATUS_STOPPED
            self.target_position_steps = None
            
            # For immediate completion, send STARTING then COMPLETE if active responses are on
            if self.slave_respond_enabled:
                 # Generate STARTING response from the original command call site
                 # Then, if active, send COMPLETE
                if self.slave_active_initiation_enabled:
                    await self._send_completion_if_callback(command_code, const.POS_RUN_COMPLETE)

            self._current_move_task = self._loop.create_future() # New future
            self._current_move_task.set_result(True) # Immediately resolve
            self._current_move_command_code = None # This specific "move" is done
            return # Return for the immediate response

        target_speed_rpm_magnitude = mks_speed_param_to_rpm(speed_mks, self.work_mode)
        if abs(target_speed_rpm_magnitude) < 0.1 and abs(delta_pos) >=0.5 : # If speed is zero but move is needed
            target_speed_rpm_magnitude = 10.0 # Use a minimal RPM to ensure movement
            logger.warning(f"Motor {self.original_can_id}: Requested speed for move is 0, using minimal RPM {target_speed_rpm_magnitude}.")


        self.target_rpm = target_speed_rpm_magnitude if delta_pos > 0 else -target_speed_rpm_magnitude
        self.target_accel_mks = accel_mks
        self._current_move_command_code = command_code
        self._current_move_task = self._loop.create_future()

        est_duration = 1.0
        if abs(self.target_rpm) > 0.1:
            # Simplified duration: time to reach full speed + time at full speed + time to decel
            # For now, a simpler estimation:
            avg_speed_rpm = abs(self.target_rpm) / 2.0 
            avg_speed_steps_sec = (avg_speed_rpm / 60.0) * self.steps_per_rev_encoder
            if avg_speed_steps_sec > 0:
                est_duration = (abs(delta_pos) / avg_speed_steps_sec)
        
        est_duration = max(1.0, est_duration + 1.0) # Add buffer, min 1s

        logger.info(
            f"Motor {self.original_can_id}: Positional move to {target_pos_abs_steps:.2f} (delta: {delta_pos:.2f}) initiated. Target RPM: {self.target_rpm:.2f}. Est. duration: {est_duration:.2f}s"
        )

        try:
            await asyncio.wait_for(self._current_move_task, timeout=est_duration)
        except asyncio.TimeoutError:
            logger.warning(
                f"Motor {self.original_can_id}: Positional move future timed out (sim). Current: {self.position_steps:.2f}, Target: {self.target_position_steps}"
            )
            if self.target_position_steps is not None:
                await self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL)
            # Do not reset target_rpm here, let _update_state handle it if motor becomes disabled/stalled
        except asyncio.CancelledError:
            logger.info(f"Motor {self.original_can_id}: Positional move future cancelled.")
        except Exception as e:
            logger.error(f"Motor {self.original_can_id}: Exception in positional move future: {e}")
            if self.target_position_steps is not None:
                await self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL)
        finally:
             if self._current_move_task and (self._current_move_task.done() or self._current_move_task.cancelled()):
                if self._current_move_command_code == command_code: # Ensure it's the future for *this* command
                    self._current_move_command_code = None


    def process_command(
        self,
        command_code: int,
        data_from_payload: bytes, # This is data ONLY (no command code, no CRC)
        send_completion_callback: Callable[[int, bytes], asyncio.Task],
    ) -> Optional[Tuple[int, bytes]]:
        self._send_completion_callback = send_completion_callback
        response_data_for_payload: Optional[List[int]] = None
        response_status_override: Optional[int] = None # For commands that return specific status, not just success/fail

        logger.info(f"Motor {self.original_can_id}: Processing CMD 0x{command_code:02X}, Data: {data_from_payload.hex() if data_from_payload else 'None'}")

        # --- Part 5.9: Read System Parameter (0x00) ---
        if command_code == const.CMD_READ_SYSTEM_PARAMETER_PREFIX:
            if not data_from_payload or len(data_from_payload) < 1:
                logger.error(f"Motor {self.original_can_id}: Read System Parameter (0x00) missing actual parameter code in data.")
                return self._generate_simple_status_response(const.CMD_READ_SYSTEM_PARAMETER_PREFIX, False) # Or some other error indication
            
            actual_param_cmd_code = data_from_payload[0]
            logger.info(f"Motor {self.original_can_id}: Read System Parameter for internal CMD 0x{actual_param_cmd_code:02X}")
            param_data = self._get_param_data_bytes(actual_param_cmd_code)

            if param_data is not None:
                return self._generate_response(actual_param_cmd_code, param_data) # Responds with echoed actual_param_cmd_code
            else:
                # Parameter not readable or not implemented for reading in simulator
                return self._generate_response(actual_param_cmd_code, [0xFF, 0xFF])


        # --- Part 5.1: Read Status Parameter Commands ---
        elif command_code == const.CMD_READ_ENCODER_CARRY: # 0x30
            carry = 0 # Simplified
            value = int(round(self.position_steps)) & 0x3FFF # Lower 14 bits
            response_data_for_payload = list(struct.pack("<iH", carry, value))
        elif command_code == const.CMD_READ_ENCODER_ADDITION: # 0x31
            pos_bytes_48bit = bytearray(8)
            struct.pack_into("<q", pos_bytes_48bit, 0, int(round(self.position_steps)))
            response_data_for_payload = list(pos_bytes_48bit[:6])
        elif command_code == const.CMD_READ_MOTOR_SPEED_RPM: # 0x32
            response_data_for_payload = list(struct.pack("<h", int(round(self.current_rpm))))
        elif command_code == const.CMD_READ_PULSES_RECEIVED: # 0x33
            response_data_for_payload = list(struct.pack("<i", 0)) # Placeholder
        elif command_code == const.CMD_READ_IO_STATUS: # 0x34
            status_byte = (self.io_out2_value << 3) | (self.io_out1_value << 2) | (0 << 1) | (0 << 0) # Assuming IN are 0
            response_data_for_payload = [status_byte]
        elif command_code == const.CMD_READ_RAW_ENCODER_ADDITION: # 0x35 (Same as 0x31 for sim)
            pos_bytes_48bit = bytearray(8)
            struct.pack_into("<q", pos_bytes_48bit, 0, int(round(self.position_steps)))
            response_data_for_payload = list(pos_bytes_48bit[:6])
        elif command_code == const.CMD_READ_SHAFT_ANGLE_ERROR: # 0x39
            error_val = 0 # Placeholder for shaft angle error
            response_data_for_payload = list(struct.pack("<i", error_val))
        elif command_code == const.CMD_READ_EN_PIN_STATUS: # 0x3A
            response_data_for_payload = [0x01 if self.is_enabled else 0x00]
        elif command_code == const.CMD_READ_POWER_ON_ZERO_STATUS: # 0x3B
            response_data_for_payload = [self.power_on_zero_status]
        elif command_code == const.CMD_RELEASE_STALL_PROTECTION: # 0x3D
            self.is_stalled = False; self.is_protected_by_stall = False
            response_status_override = const.STATUS_SUCCESS
        elif command_code == const.CMD_READ_MOTOR_PROTECTION_STATE: # 0x3E
            is_protected = self.is_protected_by_stall or self.is_protected_by_pos_error
            response_data_for_payload = [0x01 if is_protected else 0x00]

        # --- Part 5.2: Set System Parameters ---
        elif command_code == const.CMD_CALIBRATE_ENCODER: # 0x80
            self.is_calibrated = True
            response_status_override = const.STATUS_CALIBRATED_SUCCESS
        elif command_code == const.CMD_SET_WORK_MODE: # 0x82
            if data_from_payload and 0 <= data_from_payload[0] <= 5:
                self.work_mode = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_WORKING_CURRENT: # 0x83
            if data_from_payload and len(data_from_payload) >= 2:
                self.working_current_ma = struct.unpack("<H", data_from_payload[:2])[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_HOLDING_CURRENT_PERCENTAGE: # 0x9B
            if data_from_payload and 0x00 <= data_from_payload[0] <= 0x08:
                self.holding_current_percentage_code = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_SUBDIVISION: # 0x84
            if data_from_payload and 0 <= data_from_payload[0] <= 255:
                self.microsteps = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_EN_PIN_ACTIVE_LEVEL: # 0x85
            if data_from_payload and data_from_payload[0] in [0,1,2]:
                self.en_pin_active_level = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_MOTOR_DIRECTION: # 0x86
            if data_from_payload and data_from_payload[0] in [0,1]:
                self.motor_direction_setting = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_AUTO_SCREEN_OFF: # 0x87
            if data_from_payload and data_from_payload[0] in [0,1]:
                self.auto_screen_off_enabled = (data_from_payload[0] == 0x01)
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_STALL_PROTECTION: # 0x88
            if data_from_payload and data_from_payload[0] in [0,1]:
                self.stall_protection_enabled = (data_from_payload[0] == 0x01)
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_SUBDIVISION_INTERPOLATION: # 0x89
             if data_from_payload and data_from_payload[0] in [0,1]:
                self.subdivision_interpolation_enabled = (data_from_payload[0] == 0x01)
                response_status_override = const.STATUS_SUCCESS
             else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_CAN_BITRATE: # 0x8A
            if data_from_payload and 0 <= data_from_payload[0] <= 3:
                self.can_bitrate_code = data_from_payload[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_CAN_ID: # 0x8B
            if data_from_payload and len(data_from_payload) >= 2:
                new_id = struct.unpack("<H", data_from_payload[:2])[0]
                if 0 <= new_id <= 0x7FF:
                    logger.info(f"Motor {self.original_can_id}: CAN ID changed to {new_id:03X} by command. Simulator will still respond on original ID for this ack, but listens on new ID after.")
                    self.can_id = new_id # Update listening ID for VirtualCANBus (conceptual)
                    response_status_override = const.STATUS_SUCCESS
                else: response_status_override = const.STATUS_FAILURE
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_SLAVE_RESPOND_ACTIVE: # 0x8C
            if data_from_payload and len(data_from_payload) >= 2:
                self.slave_respond_enabled = (data_from_payload[0] == 0x01)
                self.slave_active_initiation_enabled = (data_from_payload[1] == 0x01)
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_GROUP_ID: # 0x8D
            if data_from_payload and len(data_from_payload) >= 2:
                self.group_id = struct.unpack("<H", data_from_payload[:2])[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_KEY_LOCK: # 0x8F
            if data_from_payload and data_from_payload[0] in [0,1]:
                self.is_key_locked = (data_from_payload[0] == 0x01)
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE

        # --- Part 5.3: Write IO Port ---
        elif command_code == const.CMD_WRITE_IO_PORT: # 0x36
            # Simplified: just acknowledge. Real sim would change self.io_out1/2_value
            if data_from_payload and len(data_from_payload) >= 1:
                 # byte_val = data_from_payload[0]
                 # out2_mask = (byte_val >> 6) & 0x03
                 # out1_mask = (byte_val >> 4) & 0x03
                 # out2_val_cmd = (byte_val >> 3) & 0x01
                 # out1_val_cmd = (byte_val >> 2) & 0x01
                 # if out2_mask == 1: self.io_out2_value = out2_val_cmd
                 # if out1_mask == 1: self.io_out1_value = out1_val_cmd
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE

        # --- Part 5.4: Set Home Command ---
        elif command_code == const.CMD_SET_HOME_PARAMETERS: # 0x90
            if data_from_payload and len(data_from_payload) >= 6:
                self.home_trig_level = data_from_payload[0]
                self.home_dir = data_from_payload[1]
                self.home_speed_rpm = struct.unpack("<H", data_from_payload[2:4])[0]
                self.end_limit_enabled_setting = (data_from_payload[4] == 0x01)
                self.home_mode_setting = data_from_payload[5]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_GO_HOME: # 0x91
            # Simulate homing start, actual homing logic is complex
            self.is_homed = False # Mark as not homed until completion
            self.motor_status_code = const.MOTOR_STATUS_HOMING
            # In a real scenario, this would trigger an async operation.
            # For now, just return "starting". Completion handled by active response.
            response_status_override = const.HOME_START 
            # Simulate completion after a delay if active responses on
            if self.slave_active_initiation_enabled:
                async def _complete_homing():
                    await asyncio.sleep(0.5) # Simulate homing time
                    self.is_homed = True
                    self.position_steps = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    await self._send_completion_if_callback(const.CMD_GO_HOME, const.HOME_SUCCESS)
                self._loop.create_task(_complete_homing())
        elif command_code == const.CMD_SET_CURRENT_AXIS_TO_ZERO: # 0x92
            self.position_steps = 0.0
            self.is_homed = True # Typically setting zero implies homed state
            response_status_override = const.STATUS_SUCCESS
        elif command_code == const.CMD_SET_NOLIMIT_HOME_PARAMS: # 0x94
            if data_from_payload and len(data_from_payload) >= 6:
                self.nolimit_home_reverse_angle = struct.unpack("<I", data_from_payload[0:4])[0]
                self.nolimit_home_current_ma = struct.unpack("<H", data_from_payload[4:6])[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_SET_LIMIT_PORT_REMAP: # 0x9E
            if data_from_payload and data_from_payload[0] in [0,1]:
                self.limit_port_remap_enabled = (data_from_payload[0] == 0x01)
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE

        # --- Part 5.5: Set 0_Mode Command ---
        elif command_code == const.CMD_SET_ZERO_MODE_PARAMETERS: # 0x9A
            if data_from_payload and len(data_from_payload) >= 4:
                self.zero_mode_behavior = data_from_payload[0]
                self.zero_mode_set_zero_action = data_from_payload[1]
                self.zero_mode_speed_code = data_from_payload[2]
                self.zero_mode_direction = data_from_payload[3]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE

        # --- Part 5.6: Restore Default Parameters ---
        elif command_code == const.CMD_RESTORE_DEFAULT_PARAMETERS: # 0x3F
            # Simulate restoring defaults - re-init some params to typical defaults
            self.__init__(self.original_can_id, self._loop, self.motor_type) # Re-init with original ID
            self.is_calibrated = False # Needs recalibration
            self.is_homed = False
            response_status_override = const.STATUS_SUCCESS

        # --- Part 5.7: Restart Motor ---
        elif command_code == const.CMD_RESTART_MOTOR: # 0x41
            logger.info(f"Motor {self.original_can_id}: Simulating restart.")
            # Similar to restore, re-init parts of state, but keep CAN ID etc.
            self.motor_status_code = const.MOTOR_STATUS_STOPPED
            self.is_enabled = False
            response_status_override = const.STATUS_SUCCESS
            
        # --- Part 5.8: En Triggers and Position Error Protection ---
        elif command_code == const.CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION: # 0x9D
            if data_from_payload and len(data_from_payload) >= 5:
                byte2 = data_from_payload[0]
                self.enable_en_trigger_zero = (byte2 & 0x02) != 0
                self.enable_pos_error_protection = (byte2 & 0x01) != 0
                self.error_detection_time_ms_units = struct.unpack("<H", data_from_payload[1:3])[0]
                self.error_threshold_pulses = struct.unpack("<H", data_from_payload[3:5])[0]
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE

        # --- Part 6: Run Motor Commands ---
        elif command_code == const.CMD_QUERY_MOTOR_STATUS: # 0xF1
            response_data_for_payload = [self.motor_status_code]
        elif command_code == const.CMD_ENABLE_MOTOR: # 0xF3
            if data_from_payload and len(data_from_payload) >= 1:
                self.is_enabled = (data_from_payload[0] == 0x01)
                if not self.is_enabled: # If disabling
                    self.current_rpm = 0.0; self.target_rpm = 0.0
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    if self._current_move_task and not self._current_move_task.done():
                        self._current_move_task.cancel("Motor disabled")
                        if self._current_move_command_code:
                             self._loop.create_task(self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL))
                        self.target_position_steps = None
                response_status_override = const.STATUS_SUCCESS
            else: response_status_override = const.STATUS_FAILURE
        elif command_code == const.CMD_EMERGENCY_STOP: # 0xF7
            self.current_rpm = 0.0; self.target_rpm = 0.0
            self.motor_status_code = const.MOTOR_STATUS_STOPPED
            self.target_position_steps = None
            if self._current_move_task and not self._current_move_task.done():
                self._current_move_task.cancel("Emergency stop")
                if self._current_move_command_code:
                    self._loop.create_task(self._send_completion_if_callback(self._current_move_command_code, const.POS_RUN_FAIL)) # Or a specific E-stop status if exists
            response_status_override = const.STATUS_SUCCESS
        elif command_code == const.CMD_RUN_SPEED_MODE: # 0xF6
            if data_from_payload and len(data_from_payload) >= 3:
                b2, b3, b4 = data_from_payload[0], data_from_payload[1], data_from_payload[2]
                mks_speed_param = ((b2 & 0x0F) << 8) | b3
                mks_accel_param = b4
                is_ccw = (b2 & 0x80) == 0
                
                calculated_target_rpm = mks_speed_param_to_rpm(mks_speed_param, self.work_mode)
                self.target_rpm = calculated_target_rpm if is_ccw else -calculated_target_rpm
                self.target_accel_mks = mks_accel_param
                self.target_position_steps = None # Clear any positional target
                if self._current_move_task and not self._current_move_task.done(): # Cancel existing positional move
                    self._current_move_task.cancel("Speed mode started")
                response_status_override = const.POS_RUN_STARTING # Manual says 0 or 1
            else: response_status_override = const.POS_RUN_FAIL
            
        elif command_code == const.CMD_SAVE_CLEAN_SPEED_MODE_PARAMS: # 0xFF
            if data_from_payload and len(data_from_payload) >= 1:
                action_code = data_from_payload[0]
                if action_code == const.SPEED_MODE_PARAM_SAVE:
                    self.saved_speed_mode_active = True
                    self.saved_speed_mode_params = {"rpm": self.target_rpm, "accel": self.target_accel_mks}
                    logger.info(f"Motor {self.original_can_id}: Speed mode params saved.")
                    response_status_override = const.STATUS_SUCCESS
                elif action_code == const.SPEED_MODE_PARAM_CLEAN:
                    self.saved_speed_mode_active = False
                    self.saved_speed_mode_params = None
                    logger.info(f"Motor {self.original_can_id}: Speed mode params cleaned.")
                    response_status_override = const.STATUS_SUCCESS
                else:
                    response_status_override = const.STATUS_FAILURE
            else: response_status_override = const.STATUS_FAILURE

        elif command_code in [const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, # 0xFD
                              const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, # 0xFE
                              const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS,   # 0xF4
                              const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS]:  # 0xF5
            target_pos_abs_final: Optional[float] = None
            mks_speed_val = 0
            mks_accel_val = 0
            parsed_ok = False
            
            try:
                if command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES:
                    if data_from_payload and len(data_from_payload) >= 6:
                        b2,b3,b4 = data_from_payload[0],data_from_payload[1],data_from_payload[2]
                        pulses_val = struct.unpack("<I", data_from_payload[3:6] + b'\x00')[0] & 0xFFFFFF
                        is_ccw = (b2 & 0x80) == 0
                        mks_speed_val = ((b2 & 0x0F) << 8) | b3
                        mks_accel_val = b4
                        delta = float(pulses_val) if is_ccw else -float(pulses_val)
                        target_pos_abs_final = self.position_steps + delta
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES:
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack("<H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        abs_pulses_bytes = data_from_payload[3:6]
                        target_pos_abs_final = float(struct.unpack("<i", abs_pulses_bytes + (b'\xff' if abs_pulses_bytes[2] & 0x80 else b'\x00'))[0] & 0xFFFFFF | (~0xFFFFFF if abs_pulses_bytes[2] & 0x80 else 0) ) # signed 24bit
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS: # 0xF4
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack("<H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        rel_axis_bytes = data_from_payload[3:6]
                        # Signed 24-bit:
                        val_u24 = rel_axis_bytes[0] | (rel_axis_bytes[1] << 8) | (rel_axis_bytes[2] << 16)
                        rel_axis_val = val_u24 if not (val_u24 & 0x800000) else val_u24 - (1 << 24)
                        target_pos_abs_final = self.position_steps + float(rel_axis_val)
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS: # 0xF5
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack("<H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        abs_axis_bytes = data_from_payload[3:6]
                        val_u24 = abs_axis_bytes[0] | (abs_axis_bytes[1] << 8) | (abs_axis_bytes[2] << 16)
                        target_pos_abs_final = float(val_u24 if not (val_u24 & 0x800000) else val_u24 - (1 << 24))
                        parsed_ok = True
            except struct.error as e:
                 logger.error(f"Motor {self.original_can_id}: Parse error for CMD {command_code:02X}, data {data_from_payload.hex()}: {e}")
                 response_status_override = const.POS_RUN_FAIL

            if parsed_ok and target_pos_abs_final is not None:
                # Check if it's a stop command variant (speed=0, pulses/axis=0)
                # Manual pages 44, 46, 48, 50 describe stop for these modes
                is_stop_command = False
                if command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES or \
                   command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS:
                    # For relative, stop means pulses/axis = 0, speed = 0
                    if mks_speed_val == 0 and ( (command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES and pulses_val == 0) or \
                                                (command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS and rel_axis_val == 0) ):
                        is_stop_command = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES or \
                     command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS:
                    # For absolute, stop also means speed=0, target_axis=0 (as per manual depiction for stop)
                    if mks_speed_val == 0 and target_pos_abs_final == 0: # Assuming stop means target axis 0
                        is_stop_command = True
                
                if is_stop_command:
                    logger.info(f"Motor {self.original_can_id}: Processing CMD {command_code:02X} as STOP. Accel: {mks_accel_val}")
                    self.target_rpm = 0.0 # Stop
                    self.target_accel_mks = mks_accel_val
                    self.target_position_steps = None # Clear positional target
                    if self._current_move_task and not self._current_move_task.done():
                        self._current_move_task.cancel("Positional move stopped by command")
                    response_status_override = const.POS_RUN_STARTING # "stop starting"
                    # Simulate completion of stop
                    if self.slave_active_initiation_enabled:
                        async def _complete_stop():
                            await asyncio.sleep(0.1 + (255 - mks_accel_val) * 0.001) # Sim stop time
                            await self._send_completion_if_callback(command_code, const.POS_RUN_COMPLETE) # "stop complete"
                        self._loop.create_task(_complete_stop())
                else: # It's a move command
                    self._loop.create_task(
                        self._handle_positional_move(target_pos_abs_final, mks_speed_val, mks_accel_val, command_code)
                    )
                    response_status_override = const.POS_RUN_STARTING
            elif not response_status_override: # if not already set to fail by parsing
                response_status_override = const.POS_RUN_FAIL
        else:
            logger.warning(f"Motor {self.original_can_id}: Unhandled CMD 0x{command_code:02X}. Data: {data_from_payload.hex()}")
            return self._generate_simple_status_response(command_code, False) # Generic fail for unhandled

        # Generate response based on override or default success/fail
        if response_status_override is not None:
            return self._generate_response(command_code, [response_status_override])
        elif response_data_for_payload is not None:
            return self._generate_response(command_code, response_data_for_payload)
        else: # Should have been handled by setting response_status_override for set commands
            logger.error(f"Motor {self.original_can_id}: Command 0x{command_code:02X} fell through response logic.")
            return self._generate_simple_status_response(command_code, False)


    async def start(self):
        if self.is_running_task and not self.is_running_task.done():
            logger.warning(f"Motor {self.original_can_id} simulation task already running.")
            return
        self._last_update_time = time.monotonic()
        self.is_running_task = self._loop.create_task(self._update_state())
        logger.info(f"SimulatedMotor {self.original_can_id} update task started.")

    async def stop_simulation(self):
        if self._current_move_task and not self._current_move_task.done():
            self._current_move_task.cancel("Simulation stopping")
            await asyncio.sleep(0) 
        self._current_move_task = None 

        if self.is_running_task and not self.is_running_task.done():
            self.is_running_task.cancel()
            try:
                await self.is_running_task 
            except asyncio.CancelledError:
                logger.info(f"SimulatedMotor {self.original_can_id} update task successfully cancelled.")
            except Exception as e:
                logger.error(f"SimulatedMotor {self.original_can_id} update task error during stop: {e}")
        self.is_running_task = None
        logger.info(f"SimulatedMotor {self.original_can_id} update task stopped.")
        