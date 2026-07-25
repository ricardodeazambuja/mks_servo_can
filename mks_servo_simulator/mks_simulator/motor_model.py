"""
Simulated Motor Model for MKS SERVO42D/57D.
Models the internal state and behavior of a motor responding to CAN commands.
"""
import asyncio
import logging
import math
import struct
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from .clock import RealTimeClock

logger = logging.getLogger("SimulatedMotor") # Changed from __name__ for clarity if file is moved/copied

# The simulator emulates the very protocol this library speaks, and shares its
# constants, CRC and motion model so the two cannot diverge. A stubbed fallback
# here would let the simulator validate the library against a second, silently
# different implementation - which is exactly the failure mode a simulator is
# supposed to prevent. So this import is deliberately hard.
try:
    from mks_servo_can import constants as const
    from mks_servo_can import motor_profile as _profile
    from mks_servo_can.crc import calculate_crc
    from mks_servo_can.exceptions import ConfigurationError, MKSServoError
except ImportError as exc:  # pragma: no cover - install-time failure
    raise ImportError(
        "mks-servo-simulator requires the mks-servo-can library. Install it "
        "with 'pip install -e ./mks_servo_can_library' from the project root."
    ) from exc

SIM_TIME_STEP_MS = 10

# Commands whose uplink frame CanRSP can suppress. Manual V1.0.6 sections
# 6.4-6.8 only; every other command answers regardless.
SUPPRESSIBLE_RESPONSE_COMMANDS = frozenset(
    {
        const.CMD_RUN_SPEED_MODE,                        # 6.4  0xF6
        const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES,     # 6.5  0xFD
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,     # 6.6  0xFE
        const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS,       # 6.7  0xF4
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,       # 6.8  0xF5
    }
)
SIM_MAX_SPEED_PARAM = 3000 # Used for RPM conversion, matches VFOC for SR_VFOC
SIM_MAX_ACCEL_PARAM = 255

# Degrees of shaft rotation per revolution, for reporting angles.
_DEGREES_PER_REV = 360.0


def _json_safe(value: float) -> Optional[float]:
    """
    Returns `value`, or None if it cannot survive JSON.

    `inf` and `nan` are legitimate results in the motion model - an
    acceleration parameter of 0 means "no ramp", whose acceleration really is
    infinite - but they are not JSON. Starlette serialises responses with
    `allow_nan=False`, so one such value anywhere in a snapshot turns `/status`
    into an HTTP 500 and takes the browser dashboard, which renders `/status`,
    down with it.

    Args:
        value: The number to check.

    Returns:
        The value if finite, otherwise None.
    """
    return value if math.isfinite(value) else None


def mks_speed_param_to_rpm(param: int, mode: int = const.MODE_SR_VFOC) -> float:
    """
    Converts an MKS speed parameter (0-3000) to motor RPM.

    Thin delegate to `mks_servo_can.motor_profile.speed_param_to_rpm` so that the
    simulator cannot develop its own idea of what a speed parameter means. If the
    library's model is wrong, the simulator is wrong in the same way and the
    discrepancy shows up against real hardware rather than hiding here.

    Args:
        param: The MKS speed parameter (0-3000).
        mode: The motor's work mode, which sets the RPM ceiling.

    Returns:
        The motor speed in RPM.
    """
    # The simulator models the calibrated 16-microstep case; per-motor
    # subdivision is applied by the caller where it matters.
    return _profile.speed_param_to_rpm(param, microsteps=16, work_mode=mode)


def mks_accel_param_to_rpm_per_sec_sq(
    param: int, current_rpm: float = 0.0, target_rpm: float = 0.0
) -> float:
    """
    Converts an MKS acceleration parameter (0-255) to RPM per second.

    Thin delegate to `mks_servo_can.motor_profile.accel_param_to_rpm_per_second`.
    Manual section 6.1 defines the ramp as 1 RPM every (256 - acc) * 50 us, so
    the parameter is an inverse rate rather than an acceleration.

    Args:
        param: The MKS acceleration parameter (0-255).
        current_rpm: Unused; retained for call-site compatibility.
        target_rpm: Unused; retained for call-site compatibility.

    Returns:
        Acceleration in RPM/s, or float('inf') when param is 0 (no ramp).
    """
    del current_rpm, target_rpm  # The MKS ramp rate does not depend on either.
    return _profile.accel_param_to_rpm_per_second(
        max(0, min(int(param), SIM_MAX_ACCEL_PARAM))
    )


@dataclass(frozen=True)
class MotorSnapshot:
    """
    A complete, consistent reading of one simulated motor's state.

    This is the *only* supported way to observe a `SimulatedMotor` from
    outside. Every reporting surface - the JSON event stream, the HTTP debug
    API, the browser dashboard - renders this and nothing else.

    That rule exists because the alternative was tried and failed. Each surface
    used to reach into the motor for whatever attribute names it assumed
    existed, and none of them matched: the debug interface read `enabled`,
    `encoder_position`, `current_speed` and thirteen other names the motor has
    never had. Because those reads were wrapped in `getattr(..., default)`, the
    mismatch did not raise - it reported zeros. The one field without a default,
    `name`, turned the whole endpoint into an HTTP 500. Routing every observer
    through one frozen dataclass makes that class of drift impossible: a renamed
    attribute breaks this file loudly, in one place, instead of silently
    degrading every report.

    Angles are derived from the motor's own encoder resolution rather than
    assumed, so a motor configured with a non-standard `steps_per_rev_encoder`
    still reports truthfully.

    Attributes:
        can_id: The ID the motor was created with, and the one it answers on.
        listening_can_id: The ID it currently listens on, which differs from
            `can_id` after a successful 0x8B.
        motor_type: The modelled hardware variant, e.g. "SERVO42D".
        enabled: Whether the servo loop is engaged.
        calibrated / homed: Encoder calibration and homing state.
        status_code: Raw MKS status byte, as command 0xF1 would report it.
        status_text: Human-readable rendering of `status_code`.
        position_steps: Current position in raw encoder counts. Fractional
            because the simulation integrates continuously.
        position_degrees: `position_steps` expressed as shaft rotation.
        target_position_steps: Target of the move in flight, or None when no
            positional move is active.
        target_position_degrees: `target_position_steps` in degrees, or None.
        position_error_steps: Signed distance still to travel, or None.
        current_rpm / target_rpm: Present and commanded shaft speed. Signed;
            positive is counter-clockwise.
        speed_deg_per_s: `current_rpm` expressed as shaft angular rate.
        moving: True when the shaft is turning.
        work_mode / work_mode_name: The 0x82 work mode.
        microsteps: Subdivision setting, as set by 0x84.
        steps_per_rev_encoder: Encoder counts per shaft revolution.
        working_current_ma: Configured phase current.
        holding_current_percent: Holding current as a percentage of working
            current, decoded from the 0x9B register code.
        stalled: The rotor is stalled right now.
        protected: Stall or position-error protection has latched.
        responses_enabled: CanRSP - whether run commands are acknowledged.
        active_notifications_enabled: CanACT - whether asynchronous completion
            frames are emitted.
        accel_param: The acceleration parameter of the most recent move.
        accel_deg_per_s2: `accel_param` converted to engineering units, or
            None when `accel_param` is 0, which the manual defines as "no ramp,
            jump straight to speed" - an infinite acceleration. `math.inf` is
            the correct engineering answer and is what `motor_profile` returns,
            but it is not representable in JSON: Starlette serialises with
            `allow_nan=False`, so a single motor with accel_param 0 made
            `/status` return HTTP 500 and took the browser dashboard down with
            it. None says the same thing and survives the wire.
    """

    can_id: int
    listening_can_id: int
    motor_type: str
    enabled: bool
    calibrated: bool
    homed: bool
    status_code: int
    status_text: str
    position_steps: float
    position_degrees: float
    target_position_steps: Optional[float]
    target_position_degrees: Optional[float]
    position_error_steps: Optional[float]
    current_rpm: float
    target_rpm: float
    speed_deg_per_s: float
    moving: bool
    work_mode: int
    work_mode_name: str
    microsteps: int
    steps_per_rev_encoder: int
    working_current_ma: int
    holding_current_percent: int
    stalled: bool
    protected: bool
    responses_enabled: bool
    active_notifications_enabled: bool
    accel_param: int
    accel_deg_per_s2: Optional[float]

    def as_dict(self) -> Dict[str, Any]:
        """
        Returns the snapshot as a plain JSON-serialisable dictionary.

        Returns:
            A mapping of field name to value, with no nesting.
        """
        return asdict(self)


class SimulatedMotor:
    """
    Models the internal state and behavior of an MKS SERVO42D/57D motor.

    This class simulates responses to CAN commands, updates its position, speed,
    and other parameters based on received commands and internal timing.
    It is designed to work with the VirtualCANBus for testing the mks-servo-can
    library without physical hardware.
    """
    def __init__(
        self,
        can_id: int,
        loop: asyncio.AbstractEventLoop,
        motor_type: str = "SERVO42D",
        initial_pos_steps: int = 0,
        steps_per_rev_encoder: int = const.ENCODER_PULSES_PER_REVOLUTION,
        base_motor_steps_per_rev: int = 200,
        mstep_value: int = 16,
        min_pos_limit_steps: Optional[int] = None,
        max_pos_limit_steps: Optional[int] = None,
        clock=None,
    ):
        """
        Initializes a new simulated MKS servo motor instance.

        Args:
            can_id: The CAN ID this simulated motor will respond to.
            loop: The asyncio event loop this motor's simulation will run in.
            motor_type: The type of motor being simulated (e.g., "SERVO42D").
                        This can influence default parameters like current.
            initial_pos_steps: The starting position of the motor in encoder steps.
            steps_per_rev_encoder: The number of encoder steps per one full revolution.
                                   Defaults to `const.ENCODER_PULSES_PER_REVOLUTION`.
            base_motor_steps_per_rev: The number of full steps for the motor (e.g., 200).
            mstep_value: The microstepping setting (e.g., 16).
            min_pos_limit_steps: Optional minimum software position limit in steps.
            max_pos_limit_steps: Optional maximum software position limit in steps.
            clock: Where simulated time comes from. Defaults to a `RealTimeClock`,
                   which is the behaviour this has always had. Pass a
                   `SteppedClock` to make the motion model advance only when it
                   is told to - see `mks_simulator.clock`.
        """
        self.can_id = can_id
        self.original_can_id = can_id
        self.motor_type = motor_type
        self._loop = loop
        self.is_running_task: Optional[asyncio.Task] = None

        # Core state
        self.position_steps: float = float(initial_pos_steps)
        self.target_position_steps: Optional[float] = None
        self.steps_per_rev_encoder = steps_per_rev_encoder
        self.current_rpm: float = 0.0
        self.target_rpm: float = 0.0
        self.current_accel_rpm_per_sec_sq: float = 0.0
        self.target_accel_mks: int = 100

        self.is_enabled: bool = False
        self.motor_status_code: int = const.MOTOR_STATUS_STOPPED
        self._clock = clock if clock is not None else RealTimeClock(
            step_seconds=SIM_TIME_STEP_MS / 1000.0
        )
        self._last_update_time: float = self._clock.now()
        self._current_move_task: Optional[asyncio.Future] = None
        self._current_move_command_code: Optional[int] = None
        self._send_completion_callback: Optional[
            Callable[[int, bytes], asyncio.Task]
        ] = None
        # Set by VirtualCANBus. Lets the motor surface protocol-level events
        # that a client cannot see for itself - see report_anomaly().
        self._anomaly_sink: Optional[
            Callable[[int, str, str, Dict[str, Any]], None]
        ] = None

        # Parameters for command conversion
        self.base_motor_steps_per_rev = base_motor_steps_per_rev
        self.microsteps = mstep_value
        self._microsteps_per_motor_revolution_for_cmd = self.base_motor_steps_per_rev * self.microsteps

        # Settable Parameters
        self.work_mode: int = const.MODE_SR_VFOC
        self.working_current_ma: int = 1600 if "42D" in motor_type else 3200
        self.holding_current_percentage_code: int = 0x04
        self.en_pin_active_level: int = const.EN_ACTIVE_LOW
        self.motor_direction_setting: int = const.DIR_CW
        self.auto_screen_off_enabled: bool = False
        self.stall_protection_enabled: bool = False
        self.subdivision_interpolation_enabled: bool = True
        self.can_bitrate_code: int = const.CAN_BITRATE_500K
        self.slave_respond_enabled: bool = True
        self.slave_active_initiation_enabled: bool = True
        self.group_id: int = 0x00
        self.is_key_locked: bool = False

        self.io_out1_value: int = 0
        self.io_out2_value: int = 0

        # Homing parameters
        self.home_trig_level: int = 0
        self.home_dir: int = const.DIR_CW
        self.home_speed_rpm: int = 60
        self.end_limit_enabled_setting: bool = False
        self.home_mode_setting: int = 0
        self.nolimit_home_reverse_angle: int = 0x2000
        self.nolimit_home_current_ma: int = 800 if "42D" in motor_type else 400
        self.limit_port_remap_enabled: bool = False

        # 0_Mode parameters
        self.zero_mode_behavior: int = 0
        self.zero_mode_set_zero_action: int = 2
        self.zero_mode_speed_code: int = 2
        self.zero_mode_direction: int = const.DIR_CW
        self.power_on_zero_status: int = const.HOME_SUCCESS

        # Error Protection
        self.enable_en_trigger_zero: bool = False
        self.enable_pos_error_protection: bool = False
        self.error_detection_time_ms_units: int = 100
        self.error_threshold_pulses: int = 2800

        self.is_calibrated: bool = True
        self.is_homed: bool = False
        self.is_stalled: bool = False
        self.is_protected_by_stall: bool = False
        self.is_protected_by_pos_error: bool = False

        self.saved_speed_mode_active: bool = False
        self.saved_speed_mode_params: Optional[Dict[str, Any]] = None


        logger.info(
            f"SimulatedMotor CAN ID {self.can_id:03X} initialized. Pos: {self.position_steps} steps."
        )

    def set_anomaly_sink(
        self, sink: Optional[Callable[[int, str, str, Dict[str, Any]], None]]
    ) -> None:
        """
        Registers where this motor reports protocol anomalies.

        Args:
            sink: Called as `sink(motor_id, type, description, context)`, or
                None to disable reporting.
        """
        self._anomaly_sink = sink

    def report_anomaly(
        self, anomaly_type: str, description: str, **context: Any
    ) -> None:
        """
        Records something the client would otherwise have no way to observe.

        The motivating case is a superseded move. When a new positional command
        arrives while one is still running, the motor abandons the old move and
        emits a failure frame for it - and because the MKS protocol reuses one
        command byte for both acknowledgements and completions, that frame is
        indistinguishable on the wire from the acknowledgement of the command
        that superseded it. A client that mismatches the two sees a move fail
        for no visible reason.

        The simulator knows exactly which frame is which, so it says so here.
        That turns "my moves randomly fail" into a labelled event with the
        superseding target attached, which is the difference between a
        debugging tool and a black box.

        Args:
            anomaly_type: Short category, e.g. "move_superseded".
            description: Human-readable explanation.
            **context: Structured detail for a machine reader.
        """
        logger.debug(
            "Motor %s: anomaly %s - %s", self.original_can_id, anomaly_type, description
        )
        if self._anomaly_sink is not None:
            self._anomaly_sink(
                self.original_can_id, anomaly_type, description, dict(context)
            )

    def _command_microsteps_to_raw_encoder_steps(self, command_microsteps: float) -> float:
        """Converts command microsteps to equivalent raw encoder steps."""
        if self._microsteps_per_motor_revolution_for_cmd == 0:
             raise ConfigurationError("_microsteps_per_motor_revolution_for_cmd cannot be zero for conversion.")
        motor_revolutions = float(command_microsteps) / self._microsteps_per_motor_revolution_for_cmd
        raw_encoder_steps = motor_revolutions * const.ENCODER_PULSES_PER_REVOLUTION
        return raw_encoder_steps

    def _generate_response(
        self, request_command_code: int, data: List[int]
    ) -> Tuple[int, bytes]:
        """
        Generates a complete CAN response payload including echoed command and CRC.

        The response is always generated using the motor's `original_can_id`.

        Args:
            request_command_code: The command code that was received and is being responded to
                                  (this will be the first byte of the response data).
            data: A list of integer data bytes forming the core of the response payload.

        Returns:
            A tuple containing the CAN ID to respond from (original_can_id) and
            the fully formed response payload as bytes (echoed_command + data + crc).
        """
        payload_with_cmd_echo = [request_command_code] + data
        crc = calculate_crc(self.original_can_id, payload_with_cmd_echo)
        return self.original_can_id, bytes(payload_with_cmd_echo + [crc])

    def _generate_simple_status_response(
        self, command_code: int, success: bool = True
    ) -> Tuple[int, bytes]:
        """
        Generates a common success/failure status response payload.

        Many MKS commands respond with a single status byte (0x01 for success,
        0x00 for failure) after the echoed command code.

        Args:
            command_code: The command code being responded to.
            success: True if the operation was successful, False otherwise.

        Returns:
            A tuple (original_can_id, response_payload_bytes) for a status response.
        """
        status = const.STATUS_SUCCESS if success else const.STATUS_FAILURE
        return self._generate_response(command_code, [status])

    async def _send_completion_if_callback(
        self, command_code: Optional[int], status_byte: int
    ):
        """
        Sends an asynchronous completion message via the registered callback, if available.

        This is used for commands that have multi-stage responses, like move commands
        that first acknowledge start and then later signal completion or failure.
        Only sends if `slave_active_initiation_enabled` is True.

        Args:
            command_code: The original command code for which this is a completion message.
                          If None, no message is sent.
            status_byte: The status code for the completion (e.g., POS_RUN_COMPLETE).
        """
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

    def _pack_int24_be(self, value: int) -> List[int]:
        """Packs a signed 24-bit integer into 3 bytes, big-endian (MSB first)."""
        unsigned_val = value & 0xFFFFFF
        return [
            (unsigned_val >> 16) & 0xFF, # MSB
            (unsigned_val >> 8) & 0xFF,  # Mid
            unsigned_val & 0xFF,         # LSB
        ]

    def _get_param_data_bytes(self, param_cmd_code: int) -> Optional[List[int]]:
        """ Helper to get data bytes for read_system_parameter. """
        if param_cmd_code == const.CMD_SET_WORK_MODE: return [self.work_mode]
        if param_cmd_code == const.CMD_SET_WORKING_CURRENT: return list(struct.pack(">H", self.working_current_ma))
        if param_cmd_code == const.CMD_SET_SUBDIVISION: return [self.microsteps]
        if param_cmd_code == const.CMD_SET_EN_PIN_ACTIVE_LEVEL: return [self.en_pin_active_level]
        if param_cmd_code == const.CMD_SET_MOTOR_DIRECTION: return [self.motor_direction_setting]
        if param_cmd_code == const.CMD_SET_AUTO_SCREEN_OFF: return [0x01 if self.auto_screen_off_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_STALL_PROTECTION: return [0x01 if self.stall_protection_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_SUBDIVISION_INTERPOLATION: return [0x01 if self.subdivision_interpolation_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_CAN_BITRATE: return [self.can_bitrate_code]
        if param_cmd_code == const.CMD_SET_CAN_ID: return list(struct.pack(">H", self.can_id)) # Current listening ID
        if param_cmd_code == const.CMD_SET_SLAVE_RESPOND_ACTIVE:
            return [0x01 if self.slave_respond_enabled else 0x00, 0x01 if self.slave_active_initiation_enabled else 0x00]
        if param_cmd_code == const.CMD_SET_GROUP_ID: return list(struct.pack(">H", self.group_id))
        if param_cmd_code == const.CMD_SET_KEY_LOCK: return [0x01 if self.is_key_locked else 0x00]
        if param_cmd_code == const.CMD_SET_HOLDING_CURRENT_PERCENTAGE: return [self.holding_current_percentage_code]
        # Add more readable parameters here as needed
        logger.warning(f"Simulator: Parameter 0x{param_cmd_code:02X} not implemented for reading via 0x00.")
        return None


    async def _update_state(self):
        """
        Asynchronous task that simulates the motor's continuous state updates.

        This loop runs periodically, updating the motor's position based on its
        current and target RPM, and configured acceleration. It also handles
        reaching target positions for positional moves and manages the motor's
        status code (e.g., speeding up, stopped).
        This task is started by `start()` and cancelled by `stop_simulation()`.
        """
        while True:
            # Both the pacing and the timebase come from the clock. Under
            # `--step` this parks until `advance()` releases it, so the motion
            # model integrates exactly the simulated interval it is given rather
            # than however long the machine happened to take.
            await self._clock.tick()
            current_time = self._clock.now()
            delta_t = current_time - self._last_update_time
            if delta_t <= 0:
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
        """
        Manages the simulation of a positional move.

        This involves setting the target position, calculating target RPM based on
        the provided MKS speed parameter, and managing an asyncio.Future that
        resolves when the simulated move is considered complete.
        It cancels any pre-existing move.

        Args:
            target_pos_abs_steps: The absolute target position in encoder steps.
            speed_mks: The MKS speed parameter (0-3000) for this move.
            accel_mks: The MKS acceleration parameter (0-255) for this move.
            command_code: The original MKS command code that initiated this move
                          (e.g., 0xFD, 0xFE), used for logging and completion messages.
        """
        # (Existing _handle_positional_move logic - largely unchanged but uses original_can_id for logging)
        if self._current_move_task and not self._current_move_task.done():
            logger.warning(f"Motor {self.original_can_id}: Cancelling previous move for new one.")
            self.report_anomaly(
                "move_superseded",
                (
                    f"A move to {self.target_position_steps} was abandoned because a "
                    f"new command retargeted to {target_pos_abs_steps:.0f}. The "
                    f"abort frame for the old move carries command byte "
                    f"0x{(self._current_move_command_code or 0):02X}, the same byte "
                    "as the acknowledgement of the new one - a client that does "
                    "not distinguish them will see the new move fail."
                ),
                superseded_command=self._current_move_command_code,
                superseded_target_steps=self.target_position_steps,
                new_command=command_code,
                new_target_steps=target_pos_abs_steps,
                position_steps=self.position_steps,
            )
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
        """
        Processes an incoming CAN command for this simulated motor.

        It updates the motor's internal state based on the command and its data,
        and generates an appropriate response. For commands that involve prolonged
        actions (like movement or homing), it may schedule asynchronous completion
        messages via the `send_completion_callback`.

        Args:
            command_code: The MKS CAN command code byte.
            data_from_payload: The data bytes accompanying the command (excluding command code and CRC).
            send_completion_callback: A callback function provided by the VirtualCANBus
                                      to send asynchronous completion messages back to the client.
                                      Signature: `callback(response_can_id: int, response_payload_with_crc: bytes)`

        Returns:
            An optional tuple `(response_can_id, response_payload_with_crc)` if an
            immediate response is to be sent. Returns None if the response is handled
            asynchronously or if no direct response is required for the command.
        """
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
            response_data_for_payload = list(struct.pack(">iH", carry, value)) # Changed to big-endian
        elif command_code == const.CMD_READ_ENCODER_ADDITION: # 0x31
            pos_bytes_48bit = bytearray(8)
            struct.pack_into(">q", pos_bytes_48bit, 0, int(round(self.position_steps))) # Changed to big-endian
            response_data_for_payload = list(pos_bytes_48bit[2:])
        elif command_code == const.CMD_READ_MOTOR_SPEED_RPM: # 0x32
            response_data_for_payload = list(struct.pack(">h", int(round(self.current_rpm)))) # Changed to big-endian
        elif command_code == const.CMD_READ_PULSES_RECEIVED: # 0x33
            response_data_for_payload = list(struct.pack(">i", 0)) # Placeholder # Changed to big-endian
        elif command_code == const.CMD_READ_IO_STATUS: # 0x34
            status_byte = (self.io_out2_value << 3) | (self.io_out1_value << 2) | (0 << 1) | (0 << 0) # Assuming IN are 0
            response_data_for_payload = [status_byte]
        elif command_code == const.CMD_READ_RAW_ENCODER_ADDITION: # 0x35 (Same as 0x31 for sim)
            pos_bytes_48bit = bytearray(8)
            struct.pack_into(">q", pos_bytes_48bit, 0, int(round(self.position_steps))) # Changed to big-endian
            response_data_for_payload = list(pos_bytes_48bit[2:])
        elif command_code == const.CMD_READ_SHAFT_ANGLE_ERROR: # 0x39
            error_val = 0 # Placeholder for shaft angle error
            response_data_for_payload = list(struct.pack(">i", error_val)) # Changed to big-endian
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
                self.working_current_ma = struct.unpack(">H", data_from_payload[:2])[0] # Changed to big-endian
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
                self._microsteps_per_motor_revolution_for_cmd = self.base_motor_steps_per_rev * self.microsteps
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
                new_id = struct.unpack(">H", data_from_payload[:2])[0] # Changed to big-endian
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
                self.group_id = struct.unpack(">H", data_from_payload[:2])[0] # Changed to big-endian
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
                self.home_speed_rpm = struct.unpack(">H", data_from_payload[2:4])[0] # Changed to big-endian
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
                    """Simulates the completion of the homing sequence."""
                    # Simulated time, so `--step` does not leave homing on the wall clock.
                    await self._clock.sleep(0.5)
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
                self.nolimit_home_reverse_angle = struct.unpack(">I", data_from_payload[0:4])[0] # Changed to big-endian
                self.nolimit_home_current_ma = struct.unpack(">H", data_from_payload[4:6])[0] # Changed to big-endian
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
                self.error_detection_time_ms_units = struct.unpack(">H", data_from_payload[1:3])[0] # Changed to big-endian
                self.error_threshold_pulses = struct.unpack(">H", data_from_payload[3:5])[0] # Changed to big-endian
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
                        pulses_val = (data_from_payload[3] << 16) | (data_from_payload[4] << 8) | data_from_payload[5]
                        is_ccw = (b2 & 0x80) == 0
                        mks_speed_val = ((b2 & 0x0F) << 8) | b3
                        mks_accel_val = b4
                        delta_command_microsteps = float(pulses_val) if is_ccw else -float(pulses_val)
                        delta_raw_encoder_steps = self._command_microsteps_to_raw_encoder_steps(delta_command_microsteps)
                        target_pos_abs_final = self.position_steps + delta_raw_encoder_steps
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES:
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(">H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        abs_pulses_bytes = data_from_payload[3:6]
                        val_u24 = (abs_pulses_bytes[0] << 16) | (abs_pulses_bytes[1] << 8) | abs_pulses_bytes[2]
                        target_command_microsteps = float(val_u24 if not (val_u24 & 0x800000) else val_u24 - (1 << 24))
                        target_pos_abs_final = self._command_microsteps_to_raw_encoder_steps(target_command_microsteps)
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS: # 0xF4
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(">H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        rel_axis_bytes = data_from_payload[3:6]
                        val_u24 = (rel_axis_bytes[0] << 16) | (rel_axis_bytes[1] << 8) | rel_axis_bytes[2]
                        rel_axis_val = val_u24 if not (val_u24 & 0x800000) else val_u24 - (1 << 24)
                        target_pos_abs_final = self.position_steps + float(rel_axis_val)
                        parsed_ok = True
                elif command_code == const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS: # 0xF5
                    if data_from_payload and len(data_from_payload) >= 6:
                        mks_speed_val = struct.unpack(">H", data_from_payload[0:2])[0]
                        mks_accel_val = data_from_payload[2]
                        abs_axis_bytes = data_from_payload[3:6]
                        val_u24 = (abs_axis_bytes[0] << 16) | (abs_axis_bytes[1] << 8) | abs_axis_bytes[2]
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
                    if mks_speed_val == 0 and ( \
                        (command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES and 'pulses_val' in locals() and pulses_val == 0) or \
                        (command_code == const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS and 'rel_axis_val' in locals() and rel_axis_val == 0) ):
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
                            """Simulates the completion of a stop command."""
                            # Simulated time; see the homing delay above.
                            await self._clock.sleep(0.1 + (255 - mks_accel_val) * 0.001)
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

        # CanRSP (0x8C byte1) suppresses the uplink frame, but only for the
        # speed and position mode commands. Manual V1.0.6 attaches the note
        # "the Uplink frame can be disabled by Menu CanRSP" to sections 6.4
        # through 6.8 and to nothing else: reads (5.1), system parameter writes
        # (5.2) and enable/query (6.2) always answer. Getting this wrong makes
        # a "fire-and-forget" control loop still pay for every reply, and makes
        # it impossible to re-enable responses once disabled.
        if command_code in SUPPRESSIBLE_RESPONSE_COMMANDS and not self.slave_respond_enabled:
            logger.debug(
                "Motor %s: suppressing uplink for CMD %02X (CanRSP disabled)",
                self.original_can_id,
                command_code,
            )
            return None

        # Generate response based on override or default success/fail
        if response_status_override is not None:
            return self._generate_response(command_code, [response_status_override])
        elif response_data_for_payload is not None:
            return self._generate_response(command_code, response_data_for_payload)
        else: # Should have been handled by setting response_status_override for set commands
            logger.error(f"Motor {self.original_can_id}: Command 0x{command_code:02X} fell through response logic.")
            return self._generate_simple_status_response(command_code, False)


    async def start(self):
        """
        Starts the asynchronous simulation loop for this motor.

        This creates and schedules the `_update_state` coroutine, which will
        periodically update the motor's internal state (position, speed, etc.).
        Does nothing if the simulation is already running.
        """
        if self.is_running_task and not self.is_running_task.done():
            logger.warning(f"Motor {self.original_can_id} simulation task already running.")
            return
        self._last_update_time = self._clock.now()
        # Registered before the task exists: a stepped clock must not advance
        # past a motor that has been started but has not yet reached its first
        # tick, or that motor misses an interval of simulated time.
        self._clock.register()
        self.is_running_task = self._loop.create_task(self._update_state())
        logger.info(f"SimulatedMotor {self.original_can_id} update task started.")

    async def stop_simulation(self):
        """
        Stops the asynchronous simulation loop for this motor.

        This cancels the `_update_state` task and any active move future.
        """
        if self._current_move_task and not self._current_move_task.done():
            self._current_move_task.cancel("Simulation stopping")
            await asyncio.sleep(0)
        self._current_move_task = None

        if self.is_running_task and not self.is_running_task.done():
            self.is_running_task.cancel()
            # Withdraw from the barrier before awaiting the cancellation. A
            # stepped clock waits for every registered motor to reach its tick;
            # one that is being torn down never will, and would hang the next
            # `advance()` forever.
            self._clock.unregister()
            try:
                await self.is_running_task
            except asyncio.CancelledError:
                logger.info(f"SimulatedMotor {self.original_can_id} update task successfully cancelled.")
            except Exception as e:
                logger.error(f"SimulatedMotor {self.original_can_id} update task error during stop: {e}")
        self.is_running_task = None
        logger.info(f"SimulatedMotor {self.original_can_id} update task stopped.")

    def status_snapshot(self) -> MotorSnapshot:
        """
        Captures the motor's complete observable state.

        Every value is read from a real attribute of this object; nothing is
        defaulted or invented. See `MotorSnapshot` for why that guarantee is
        stated so emphatically.

        The read is synchronous and non-blocking, so it is safe to call from a
        request handler or a render loop at any rate without perturbing the
        simulation.

        Returns:
            A frozen `MotorSnapshot` describing this motor right now.
        """
        degrees_per_step = (
            _DEGREES_PER_REV / self.steps_per_rev_encoder
            if self.steps_per_rev_encoder
            else 0.0
        )
        target_steps = self.target_position_steps
        return MotorSnapshot(
            can_id=self.original_can_id,
            listening_can_id=self.can_id,
            motor_type=self.motor_type,
            enabled=self.is_enabled,
            calibrated=self.is_calibrated,
            homed=self.is_homed,
            status_code=self.motor_status_code,
            status_text=const.MOTOR_STATUS_MAP.get(
                self.motor_status_code, f"Unknown ({self.motor_status_code})"
            ),
            position_steps=self.position_steps,
            position_degrees=self.position_steps * degrees_per_step,
            target_position_steps=target_steps,
            target_position_degrees=(
                None if target_steps is None else target_steps * degrees_per_step
            ),
            position_error_steps=(
                None if target_steps is None else target_steps - self.position_steps
            ),
            current_rpm=self.current_rpm,
            target_rpm=self.target_rpm,
            speed_deg_per_s=(self.current_rpm / 60.0) * _DEGREES_PER_REV,
            moving=self.current_rpm != 0.0,
            work_mode=self.work_mode,
            work_mode_name=self.work_mode_str,
            microsteps=self.microsteps,
            steps_per_rev_encoder=self.steps_per_rev_encoder,
            working_current_ma=self.working_current_ma,
            # The 0x9B register holds a code, not a percentage: 0 means 10%,
            # rising in 10-point steps to 90% at code 8.
            holding_current_percent=(self.holding_current_percentage_code + 1) * 10,
            stalled=self.is_stalled,
            protected=self.is_protected_by_stall or self.is_protected_by_pos_error,
            responses_enabled=self.slave_respond_enabled,
            active_notifications_enabled=self.slave_active_initiation_enabled,
            accel_param=self.target_accel_mks,
            accel_deg_per_s2=_json_safe(
                _profile.accel_param_to_deg_per_s2(
                    max(0, min(int(self.target_accel_mks), SIM_MAX_ACCEL_PARAM))
                )
            ),
        )

    @property
    def kinematics_units(self) -> str:
        # Simplified: Assumes rotary kinematics for dashboard display
        return "deg"

    @property
    def current_speed_user_units_per_sec(self) -> float:
        # Simplified: Assumes rotary kinematics with 360 deg/rev for dashboard display
        # This mimics a basic conversion from RPM to deg/sec.
        # (RPM / 60) gives RPS. RPS * 360 gives deg/sec.
        return (self.current_rpm / 60.0) * 360.0

    @property
    def work_mode_str(self) -> str:
        # Local fallback map, primarily for when mks_servo_can library might not be fully available
        # or if its const.WORK_MODES is missing/malformed.
        _work_modes_map = {
            getattr(const, 'MODE_CR_OPEN', 0): "CR_OPEN",
            getattr(const, 'MODE_CR_CLOSE', 1): "CR_CLOSE",
            getattr(const, 'MODE_SR_OPEN', 2): "SR_OPEN",
            getattr(const, 'MODE_SR_CLOSE', 3): "SR_CLOSE",
            getattr(const, 'MODE_PL_VFOC', 4): "PL_VFOC",
            getattr(const, 'MODE_SR_VFOC', 5): "SR_VFOC",
        }

        # Prefer const.WORK_MODES from the library if available and correctly formatted
        if hasattr(const, 'WORK_MODES') and isinstance(const.WORK_MODES, dict) and const.WORK_MODES:
            # Assumes const.WORK_MODES is {integer_key: "String Value"}
            return const.WORK_MODES.get(self.work_mode, f"Unknown ({self.work_mode})")

        # Fallback to the locally defined map
        return _work_modes_map.get(self.work_mode, f"Unknown ({self.work_mode})")
