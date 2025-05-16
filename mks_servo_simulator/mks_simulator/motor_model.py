# mks_servo_can_project/mks_servo_simulator/mks_simulator/motor_model.py
"""
Simulated Motor Model for MKS SERVO42D/57D.
Models the internal state and behavior of a motor responding to CAN commands.
"""
import time
import logging
import struct
import asyncio
from typing import Optional, List, Tuple, Dict, Any

# Assuming the main library's constants and crc are accessible
# This might require adjusting Python path or how the simulator is run/packaged
try:
    from mks_servo_can_library.mks_servo_can import constants as const
    from mks_servo_can_library.mks_servo_can.crc import calculate_crc
    from mks_servo_can_library.mks_servo_can.exceptions import ParameterError
except ImportError:
    # Fallback if running simulator standalone without library installed in path
    # This is not ideal for production, but helps during development.
    # You would typically ensure the library is installed or in PYTHONPATH.
    print("Warning: Could not import from mks_servo_can_library. Using placeholder constants/crc for simulator.")
    # Add minimal placeholder constants needed by the motor model if the library isn't found
    class const: # type: ignore
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
        # Add other constants used by motor model here
        CMD_READ_ENCODER_ADDITION = 0x31
        CMD_READ_MOTOR_SPEED_RPM = 0x32
        # ... (add all command codes used in process_command)

    def calculate_crc(can_id: int, data_bytes: list[int]) -> int:
        checksum = can_id
        for byte_val in data_bytes:
            checksum += byte_val
        return checksum & 0xFF
    class ParameterError(Exception): pass


logger = logging.getLogger(__name__)

# Default simulation parameters
SIM_TIME_STEP_MS = 10  # How often the motor updates its internal state (e.g., position)
SIM_MAX_SPEED_PARAM = 3000 # Corresponds to the 0-3000 MKS speed parameter
SIM_MAX_ACCEL_PARAM = 255 # Corresponds to the 0-255 MKS accel parameter

# More realistic RPM for a given speed parameter (e.g. 3000 param = 3000 RPM in VFOC)
# This is a simplification.
def mks_speed_param_to_rpm(param: int, mode: int = const.MODE_SR_VFOC) -> float:
    """Approximates RPM from MKS speed parameter."""
    # Max RPMs by mode (simplified)
    if mode in [const.MODE_SR_OPEN, const.MODE_CR_OPEN]: max_rpm_for_mode = const.MAX_RPM_OPEN_MODE
    elif mode in [const.MODE_SR_CLOSE, const.MODE_CR_CLOSE]: max_rpm_for_mode = const.MAX_RPM_CLOSE_MODE
    else: max_rpm_for_mode = const.MAX_RPM_VFOC_MODE # Default to VFOC

    return (param / SIM_MAX_SPEED_PARAM) * max_rpm_for_mode

def mks_accel_param_to_rpm_per_sec_sq(param: int, current_rpm: float, target_rpm: float) -> float:
    """
    Approximates acceleration in RPM/s^2 from MKS accel parameter.
    Manual: t2-t1 = (256-acc) * 50 uS for 1 RPM change.
    So, 1 RPM change takes (256-param) * 50e-6 seconds.
    Rate = 1 RPM / ((256-param) * 50e-6 seconds) = RPM/sec.
    This is RPM/sec, not RPM/sec^2. This is the rate of change of RPM.
    So, it's effectively already RPM/sec.
    To get to RPM/sec^2, we'd need to define how this "rate" changes.
    The manual implies this is a constant rate of RPM increase/decrease.
    """
    if param == 0: # Instantaneous change (or very fast)
        return float('inf') # Effectively infinite for simulation step
    if param > SIM_MAX_ACCEL_PARAM : param = SIM_MAX_ACCEL_PARAM

    time_for_1_rpm_change_sec = (256 - param) * 50e-6
    if time_for_1_rpm_change_sec == 0: return float('inf')
    
    rpm_change_rate_rpm_per_sec = 1.0 / time_for_1_rpm_change_sec
    return rpm_change_rate_rpm_per_sec


class SimulatedMotor:
    def __init__(self, can_id: int, loop: asyncio.AbstractEventLoop,
                 motor_type: str = "SERVO42D",
                 initial_pos_steps: int = 0,
                 steps_per_rev_encoder: int = const.ENCODER_PULSES_PER_REVOLUTION,
                 min_pos_limit_steps: Optional[int] = None, # Soft limits
                 max_pos_limit_steps: Optional[int] = None):
        self.can_id = can_id
        self.motor_type = motor_type
        self._loop = loop
        self.is_running_task: Optional[asyncio.Task] = None

        # --- Physical/State Parameters ---
        self.position_steps: float = float(initial_pos_steps) # Current position in encoder steps
        self.target_position_steps: Optional[float] = None
        self.steps_per_rev_encoder = steps_per_rev_encoder

        self.current_rpm: float = 0.0  # Current speed in RPM
        self.target_rpm: float = 0.0   # Target speed for speed mode
        
        self.current_accel_rpm_per_s: float = 0.0 # Current acceleration (RPM/s)
        self.target_accel_mks: int = 100 # MKS accel parameter (0-255) for current move

        self.work_mode: int = const.MODE_SR_VFOC # Default to a serial mode
        self.is_enabled: bool = True # Initial state
        self.is_calibrated: bool = True # Assume calibrated for simplicity
        self.is_homed: bool = False # Assume not homed
        self.stall_protection_enabled: bool = False
        self.is_stalled: bool = False
        self.limit_switches_active: bool = False # Placeholder
        self.min_pos_limit_steps = min_pos_limit_steps
        self.max_pos_limit_steps = max_pos_limit_steps
        
        self.motor_status_code: int = const.MOTOR_STATUS_STOPPED # From 0xF1 query
        self._last_update_time: float = time.monotonic()
        self._current_move_task: Optional[asyncio.Task] = None
        self._current_move_command_code: Optional[int] = None # To echo in completion message

        # Parameters that can be set by commands
        self.working_current_ma: int = 1600
        self.holding_current_percentage_code: int = 0x04 # 50%
        self.microsteps: int = 16
        self.en_pin_active_level: int = const.EN_ACTIVE_LOW
        self.motor_direction_setting: int = const.DIR_CW # For pulse mode primarily
        # ... other configurable parameters

        logger.info(f"SimulatedMotor CAN ID {self.can_id:03X} initialized. Pos: {self.position_steps} steps.")

    def _generate_response(self, request_command_code: int, data: List[int]) -> Tuple[int, bytes]:
        """Generates response payload (data + CRC) for a given command code."""
        payload = [request_command_code] + data
        crc = calculate_crc(self.can_id, payload)
        return self.can_id, bytes(payload + [crc])

    def _generate_simple_status_response(self, command_code: int, success: bool = True) -> Tuple[int, bytes]:
        status = const.STATUS_SUCCESS if success else const.STATUS_FAILURE
        return self._generate_response(command_code, [status])

    async def _update_state(self):
        """Coroutine to periodically update motor state (position, speed)."""
        while True:
            await asyncio.sleep(SIM_TIME_STEP_MS / 1000.0)
            current_time = time.monotonic()
            delta_t = current_time - self._last_update_time
            self._last_update_time = current_time

            if not self.is_enabled or self.is_stalled:
                self.current_rpm = 0.0
                self.motor_status_code = const.MOTOR_STATUS_STOPPED
                continue

            # Apply acceleration to current_rpm to reach target_rpm
            if self.current_rpm != self.target_rpm:
                accel_rate_rpm_s = mks_accel_param_to_rpm_per_sec_sq(self.target_accel_mks, self.current_rpm, self.target_rpm)
                
                if accel_rate_rpm_s == float('inf'): # Instantaneous
                    self.current_rpm = self.target_rpm
                else:
                    rpm_change = accel_rate_rpm_s * delta_t
                    if self.target_rpm > self.current_rpm:
                        self.current_rpm = min(self.target_rpm, self.current_rpm + rpm_change)
                        self.motor_status_code = const.MOTOR_STATUS_SPEED_UP if self.target_rpm != 0 else const.MOTOR_STATUS_STOPPED
                    else: # Decelerating
                        self.current_rpm = max(self.target_rpm, self.current_rpm - rpm_change)
                        self.motor_status_code = const.MOTOR_STATUS_SPEED_DOWN if self.target_rpm != 0 else const.MOTOR_STATUS_STOPPED
            
            elif self.current_rpm == 0 and self.target_rpm == 0 and self.target_position_steps is None:
                 self.motor_status_code = const.MOTOR_STATUS_STOPPED
            elif self.current_rpm != 0 : # and current_rpm == target_rpm (not accelerating)
                 self.motor_status_code = const.MOTOR_STATUS_FULL_SPEED


            # Update position based on current_rpm
            if self.current_rpm != 0:
                steps_change = (self.current_rpm / 60.0) * self.steps_per_rev_encoder * delta_t
                new_pos = self.position_steps + steps_change
                
                # Check soft limits
                limit_hit = False
                if self.min_pos_limit_steps is not None and new_pos < self.min_pos_limit_steps:
                    new_pos = self.min_pos_limit_steps
                    limit_hit = True
                if self.max_pos_limit_steps is not None and new_pos > self.max_pos_limit_steps:
                    new_pos = self.max_pos_limit_steps
                    limit_hit = True
                
                self.position_steps = new_pos

                if limit_hit:
                    logger.warning(f"Motor {self.can_id}: Soft limit hit at {self.position_steps:.2f} steps.")
                    self.current_rpm = 0
                    self.target_rpm = 0
                    if self.target_position_steps is not None: # If it was a positional move
                         # Notify completion with limit status
                        if self._current_move_task and not self._current_move_task.done():
                            # This is tricky, how does the virtual_can_bus get this message back?
                            # The motor model should return the CAN frame to virtual_can_bus.
                            # For now, just log and set future.
                            if hasattr(self, '_send_completion_callback'):
                                completion_data, _ = self._generate_response(self._current_move_command_code, [const.POS_RUN_END_LIMIT_STOPPED])
                                self._loop.create_task(self._send_completion_callback(self.can_id, bytes(completion_data)))

                            self._current_move_task.set_exception(ParameterError(f"Simulated limit hit for motor {self.can_id}")) # TODO: Specific LimitError
                        self.target_position_steps = None # Stop trying to reach
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED


            # Check if target position is reached for positional moves
            if self.target_position_steps is not None:
                if abs(self.position_steps - self.target_position_steps) < 0.5: # Tolerance of 0.5 step
                    logger.info(f"Motor {self.can_id}: Target position {self.target_position_steps:.2f} reached. Current: {self.position_steps:.2f}")
                    self.position_steps = self.target_position_steps # Snap to target
                    self.current_rpm = 0
                    self.target_rpm = 0
                    self.target_position_steps = None
                    self.motor_status_code = const.MOTOR_STATUS_STOPPED
                    if self._current_move_task and not self._current_move_task.done():
                        if hasattr(self, '_send_completion_callback'):
                            completion_data, _ = self._generate_response(self._current_move_command_code, [const.POS_RUN_COMPLETE])
                            self._loop.create_task(self._send_completion_callback(self.can_id, bytes(completion_data)))
                        self._current_move_task.set_result(True) # Mark task as done


    async def _handle_positional_move(self, target_pos: float, speed_mks: int, accel_mks: int, command_code: int):
        """Internal handler to simulate a positional move."""
        self.target_position_steps = target_pos
        self.target_rpm = mks_speed_param_to_rpm(speed_mks, self.work_mode) # Target speed to reach during move
        self.target_accel_mks = accel_mks
        self._current_move_command_code = command_code # Store for completion message

        # If current speed is 0, we need to accelerate.
        # The _update_state will handle acceleration towards target_rpm, then constant speed, then deceleration.
        # This needs a more sophisticated profile generator (trapezoidal).
        # Simplification: _update_state aims for target_rpm. When nearing target_position_steps, it should start decelerating.

        # For simulation, we can estimate move duration for the future timeout.
        # This is complex for trapezoidal. For now, assume a simple duration.
        # This future is just for the library to await if it wants.
        # The actual stop condition is handled in _update_state.
        if self._current_move_task and not self._current_move_task.done():
            self._current_move_task.cancel()
        
        self._current_move_task = self._loop.create_future()
        
        # Estimate duration:
        current_pos = self.position_steps
        distance = abs(target_pos - current_pos)
        if self.target_rpm > 0:
            # Simplified: time = distance / average_speed. Assume average speed is target_rpm/2 for accel/decel phase.
            # This is very rough.
            avg_speed_steps_per_sec = (self.target_rpm / 2 / 60.0) * self.steps_per_rev_encoder
            if avg_speed_steps_per_sec > 0:
                est_duration = distance / avg_speed_steps_per_sec + 1.0 # +1s buffer
            else: # target_rpm is 0, should not happen for a move, unless it's a stop command
                est_duration = 1.0 
        else: # Target RPM is 0, perhaps it's a stop command disguised as a move to current pos
            est_duration = 1.0 # Short duration if not actually moving

        logger.info(f"Motor {self.can_id}: Positional move to {target_pos:.2f} initiated. Est. duration: {est_duration:.2f}s")

        try:
            await asyncio.wait_for(self._current_move_task, timeout=est_duration + 2.0) # Wait slightly longer
        except asyncio.TimeoutError:
            logger.warning(f"Motor {self.can_id}: Positional move future timed out internally (sim).")
            if self.target_position_steps is not None: # If still trying to move
                 self.target_position_steps = None # Stop it
                 self.target_rpm = 0
            # Send failure response?
            if hasattr(self, '_send_completion_callback'):
                fail_data, _ = self._generate_response(self._current_move_command_code, [const.POS_RUN_FAIL])
                self._loop.create_task(self._send_completion_callback(self.can_id, bytes(fail_data)))

        except asyncio.CancelledError:
            logger.info(f"Motor {self.can_id}: Positional move future cancelled.")
            if self.target_position_steps is not None:
                 self.target_position_steps = None
                 self.target_rpm = 0
        finally:
            self._current_move_task = None
            self._current_move_command_code = None


    def process_command(self, command_code: int, data: bytes, send_completion_callback: Optional[Callable] = None) -> Optional[Tuple[int, bytes]]:
        """
        Processes a CAN command and returns the response frame (CAN ID, payload_with_crc).
        The payload_with_crc includes the echoed command code, data, and CRC.
        If no direct response is expected (e.g. for some broadcast messages), returns None.
        """
        self._send_completion_callback = send_completion_callback # Store for async completion messages
        response_data_payload: Optional[List[int]] = None # Data part of response, excluding echoed cmd and CRC

        # --- Part 5.1: Read Status ---
        if command_code == const.CMD_READ_ENCODER_ADDITION: # 0x31
            # Return int48 (6 bytes)
            pos_bytes = bytearray(8) # Work with 8 bytes for long long
            struct.pack_into('<q', pos_bytes, 0, int(round(self.position_steps)))
            response_data_payload = list(pos_bytes[:6])
        elif command_code == const.CMD_READ_MOTOR_SPEED_RPM: # 0x32
            response_data_payload = list(struct.pack('<h', int(round(self.current_rpm))))
        elif command_code == const.CMD_QUERY_MOTOR_STATUS: # 0xF1
            response_data_payload = [self.motor_status_code]
        # ... Add other read commands from LowLevelAPI that motor model should respond to

        # --- Part 5.2: Set System Parameters ---
        elif command_code == const.CMD_SET_WORK_MODE: # 0x82
            if len(data) >= 1:
                self.work_mode = data[0]
                logger.info(f"Motor {self.can_id}: Work mode set to {self.work_mode}")
                return self._generate_simple_status_response(command_code, True)
            return self._generate_simple_status_response(command_code, False)
        elif command_code == const.CMD_ENABLE_MOTOR: # 0xF3
            if len(data) >= 1:
                self.is_enabled = (data[0] == 0x01)
                if not self.is_enabled: self.current_rpm = 0; self.target_rpm = 0; # Stop if disabled
                logger.info(f"Motor {self.can_id}: Enable set to {self.is_enabled}")
                return self._generate_simple_status_response(command_code, True)
            return self._generate_simple_status_response(command_code, False)

        # --- Part 6: Run Motor ---
        elif command_code == const.CMD_RUN_SPEED_MODE: # 0xF6
            # data = [byte2, byte3, byte4] -> byte2: dir/speed_hi, byte3: speed_lo, byte4: acc
            if len(data) >= 3:
                byte2, byte3, byte4 = data[0], data[1], data[2]
                # ccw_direction = (byte2 & 0x80) != 0
                mks_speed_param = ((byte2 & 0x0F) << 8) | byte3
                mks_accel_param = byte4

                self.target_rpm = mks_speed_param_to_rpm(mks_speed_param, self.work_mode)
                if (byte2 & 0x80) == 0: # CW direction (typically negative RPM in simulation)
                    self.target_rpm *= -1 # Assuming CW is negative RPM, CCW is positive
                
                self.target_accel_mks = mks_accel_param
                self.target_position_steps = None # Clear any positional target
                logger.info(f"Motor {self.can_id}: Speed mode. Target RPM: {self.target_rpm:.2f}, Accel Param: {self.target_accel_mks}")
                # Initial response: starting
                return self._generate_response(command_code, [const.POS_RUN_STARTING]) # status 1 = run success/starting
            return self._generate_response(command_code, [const.POS_RUN_FAIL])


        elif command_code in [const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, # 0xFD
                              const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, # 0xFE
                              const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS,   # 0xF4
                              const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS]:  # 0xF5
            
            target_pos_abs: Optional[float] = None
            mks_speed: int = 0
            mks_accel: int = 0

            if command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES: # 0xFD
                # byte2:dir/speed_hi, byte3:speed_lo, byte4:acc, byte5-7:pulses(U24)
                if len(data) >= 6: # cmd code is not in data here
                    byte2, byte3, byte4 = data[0], data[1], data[2]
                    pulses_bytes = bytes(data[3:6]) + b'\x00' # Pad to 4 bytes for unpack
                    num_pulses = struct.unpack('<I', pulses_bytes)[0]
                    
                    ccw = (byte2 & 0x80) != 0
                    mks_speed = ((byte2 & 0x0F) << 8) | byte3
                    mks_accel = byte4
                    
                    delta_steps = float(num_pulses) if ccw else -float(num_pulses)
                    target_pos_abs = self.position_steps + delta_steps
                else: return self._generate_response(command_code, [const.POS_RUN_FAIL])

            elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES: # 0xFE
                # byte2-3:speed(U16), byte4:acc(U8), byte5-7:absPulses(S24)
                if len(data) >= 6:
                    mks_speed = struct.unpack('<H', bytes(data[0:2]))[0]
                    mks_accel = data[2]
                    abs_pulse_bytes = bytes(data[3:6])
                    # Handle int24
                    if abs_pulse_bytes[2] & 0x80: # Negative
                        val = struct.unpack('<I', abs_pulse_bytes + b'\xFF')[0] # Sign extend
                        target_pos_abs = float(val - (1 << 24))
                    else:
                        target_pos_abs = float(struct.unpack('<I', abs_pulse_bytes + b'\x00')[0])
                else: return self._generate_response(command_code, [const.POS_RUN_FAIL])

            # Simplified handling for F4, F5 - assume similar structure to FE for relevant parts
            elif command_code in [const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS]:
                 # speed(U16), acc(U8), axis_val(S24)
                if len(data) >= 6:
                    mks_speed = struct.unpack('<H', bytes(data[0:2]))[0]
                    mks_accel = data[2]
                    axis_val_bytes = bytes(data[3:6])
                    if axis_val_bytes[2] & 0x80: # Negative S24
                        val = struct.unpack('<I', axis_val_bytes + b'\xFF')[0]
                        axis_s24 = float(val - (1 << 24))
                    else:
                        axis_s24 = float(struct.unpack('<I', axis_val_bytes + b'\x00')[0])

                    if command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS: # F4
                        target_pos_abs = self.position_steps + axis_s24
                    else: # F5
                        target_pos_abs = axis_s24
                else: return self._generate_response(command_code, [const.POS_RUN_FAIL])


            if target_pos_abs is not None:
                # Launch the move task (do not await it here, it runs in background)
                self._loop.create_task(self._handle_positional_move(target_pos_abs, mks_speed, mks_accel, command_code))
                # Initial response: starting
                return self._generate_response(command_code, [const.POS_RUN_STARTING])
            else: # Should not happen if parsing is correct
                 return self._generate_response(command_code, [const.POS_RUN_FAIL])


        elif command_code == const.CMD_EMERGENCY_STOP: # 0xF7
            self.current_rpm = 0
            self.target_rpm = 0
            self.target_position_steps = None
            if self._current_move_task and not self._current_move_task.done():
                self._current_move_task.cancel()
            logger.warning(f"Motor {self.can_id}: Emergency stop received.")
            return self._generate_simple_status_response(command_code, True)


        # Default: if command not handled, return failure or nothing
        if response_data_payload is not None:
            return self._generate_response(command_code, response_data_payload)
        else:
            logger.warning(f"Motor {self.can_id}: Unhandled command code 0x{command_code:02X} in simulator.")
            # For unhandled commands, we could return a generic failure or no response
            # To mimic real hardware, some unhandled commands might be ignored.
            # Others might return an error. Let's return nothing.
            return None


    async def start(self):
        """Starts the motor's internal simulation loop."""
        if self.is_running_task and not self.is_running_task.done():
            logger.warning(f"Motor {self.can_id} simulation task already running.")
            return
        self._last_update_time = time.monotonic()
        self.is_running_task = self._loop.create_task(self._update_state())
        logger.info(f"SimulatedMotor {self.can_id} task started.")

    async def stop_simulation(self):
        """Stops the motor's internal simulation loop."""
        if self.is_running_task:
            self.is_running_task.cancel()
            try:
                await self.is_running_task
            except asyncio.CancelledError:
                logger.info(f"SimulatedMotor {self.can_id} task cancelled.")
            self.is_running_task = None
        if self._current_move_task and not self._current_move_task.done():
            self._current_move_task.cancel() # Cancel any ongoing move future