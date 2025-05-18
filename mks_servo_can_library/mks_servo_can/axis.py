"""
High-Level Axis Class for individual MKS Servo motor control.

This module defines the `Axis` class, which provides a user-friendly,
asynchronous interface for controlling a single MKS SERVO42D/57D motor.
It abstracts away the low-level CAN command details and manages motor state,
kinematic conversions, and error handling.
"""
from typing import Any, Dict, Optional, Callable # Added Callable
import math
import asyncio
import logging
import time

try:
    from can import Message as CanMessage # For predicate type hint
except ImportError:
    class CanMessage: # type: ignore # Dummy for type hint
        """
        Dummy CanMessage for type hinting when python-can is not available.
        Allows the library to be imported and type-checked even if python-can
        is not installed, which is useful for environments where CAN hardware
        access is not present or needed (e.g., some CI stages, offline analysis).
        """
        def __init__(self, arbitration_id=0, data=None, dlc=0):
            """
            Initializes a dummy CanMessage object.

            Args:
                arbitration_id (int): The arbitration ID of the message.
                data (Optional[bytes]): The data payload of the message. Defaults to empty bytes.
                dlc (int): Data Length Code.
            """
            self.arbitration_id = arbitration_id
            self.data = data if data is not None else b""
            self.dlc = dlc


from . import constants as const
from .can_interface import CANInterface
from .exceptions import CalibrationError
from .exceptions import CommunicationError
from .exceptions import ConfigurationError
from .exceptions import HomingError
from .exceptions import LimitError
from .exceptions import MKSServoError
from .exceptions import MotorError
from .exceptions import ParameterError
from .kinematics import Kinematics
from .kinematics import RotaryKinematics
from .low_level_api import LowLevelAPI

logger = logging.getLogger(__name__)


class Axis:
    """
    Represents a single motor axis, providing a high-level interface for control.

    This class encapsulates the logic for sending commands to an MKS servo motor,
    managing its state (e.g., position, speed, enabled status), and handling
    communication timeouts and errors. It integrates with a kinematics system
    for unit conversions.
    """

    def __init__(
        self,
        can_interface_manager: CANInterface,
        motor_can_id: int,
        name: str = "default_axis",
        motor_type: str = const.MOTOR_TYPE_SERVO42D,
        kinematics: Optional[Kinematics] = None,
        default_speed_param: int = 500,
        default_accel_param: int = 100,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ):
        """
        Initializes an Axis object representing a single MKS servo motor.

        Args:
            can_interface_manager: The `CANInterface` instance used for communication.
                                   This object must be connected before most axis
                                   operations can be performed.
            motor_can_id: The CAN ID of the motor this axis will control (typically 1-2047).
            name: A descriptive name for this axis instance (e.g., "X-Axis", "Feeder").
                  Defaults to "default_axis".
            motor_type: The type of MKS servo motor (e.g., `const.MOTOR_TYPE_SERVO42D`).
                        This may influence default parameters or behavior in future extensions.
                        Defaults to `const.MOTOR_TYPE_SERVO42D`.
            kinematics: A `Kinematics` object for converting between user-defined physical
                        units (e.g., mm, degrees) and raw motor steps. If None, defaults
                        to `RotaryKinematics` using `const.ENCODER_PULSES_PER_REVOLUTION`.
            default_speed_param: Default MKS speed parameter (0-3000) to be used for
                                 movement commands if no speed is explicitly provided.
                                 Defaults to 500.
            default_accel_param: Default MKS acceleration parameter (0-255) to be used for
                                 movement commands if no acceleration is explicitly provided.
                                 Defaults to 100.
            loop: The asyncio event loop to use for this axis's operations. If None,
                  `asyncio.get_event_loop()` is used.

        Raises:
            ParameterError: If `motor_can_id` is outside the valid range (1-2047).
        """
        if not (
            const.BROADCAST_ADDRESS < motor_can_id <= 0x7FF # type: ignore[operator] # const.BROADCAST_ADDRESS can be an int
        ):
            raise ParameterError(
                f"Invalid motor_can_id: {motor_can_id}. Must be 1-2047."
            )

        self.name = name
        self.can_id = motor_can_id
        self.motor_type = motor_type
        self._can_if = can_interface_manager
        self._low_level_api = LowLevelAPI(self._can_if)
        self._loop = loop if loop else asyncio.get_event_loop()

        if kinematics is None:
            logger.info(
                f"Axis '{name}': No kinematics provided, defaulting to RotaryKinematics "
                f"with {const.ENCODER_PULSES_PER_REVOLUTION} steps/rev (encoder pulses)."
            )
            self.kinematics: Kinematics = RotaryKinematics( # Explicitly type hint self.kinematics
                steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
            )
        else:
            self.kinematics = kinematics

        self.default_speed_param = default_speed_param
        self.default_accel_param = default_accel_param

        self._current_position_steps: Optional[int] = None
        self._current_speed_rpm: Optional[int] = None
        self._is_enabled: bool = False
        self._is_homed: bool = False
        self._is_calibrated: bool = False # Assume not calibrated until explicitly done
        self._active_move_future: Optional[asyncio.Future] = None
        self._error_state: Optional[MKSServoError] = None


    async def initialize(
        self, calibrate: bool = False, home: bool = False
    ) -> None:
        """
        Performs initial communication and setup for the axis.

        This method typically includes:
        1. A basic communication test by reading the current position.
        2. Reading the initial enabled status of the motor.
        3. Optionally, performing encoder calibration.
        4. Optionally, performing a homing sequence. Homing will only proceed
           if calibration was successful (if `calibrate=True`) or if calibration
           was not requested and the axis is considered calibrated.

        Sets the internal `_is_calibrated` flag to True after a successful
        communication test if calibration is not explicitly performed and fails.

        Args:
            calibrate: If True, attempts to calibrate the motor's encoder.
                       This may be required for some motors before precise operation.
            home: If True, attempts to home the axis. Homing typically sets a
                  defined zero position.

        Raises:
            MKSServoError: Or its subclasses (e.g., `CommunicationError`, `CalibrationError`,
                           `HomingError`) if any part of the initialization process fails.
        """
        logger.info(
            f"Axis '{self.name}' (CAN ID: {self.can_id:03X}): Initializing..."
        )
        try:
            await self.get_current_position_steps() # Basic comms test
            logger.info(f"Axis '{self.name}': Communication test successful.")
            self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
            logger.info(
                f"Axis '{self.name}': Initial EN pin status: {'Enabled' if self._is_enabled else 'Disabled'}."
            )
            if calibrate:
                await self.calibrate_encoder() # Sets _is_calibrated on success
            else:
                # If not calibrating, assume it's usable (e.g., previously calibrated).
                # This behavior might need refinement based on how "calibrated" status is strictly defined.
                # For now, successful communication implies it's in a usable state if calibration not requested.
                self._is_calibrated = True 
            
            if home:
                if self._is_calibrated: # Home only if considered calibrated
                    await self.home_axis()
                else:
                    logger.warning(
                        f"Axis '{self.name}': Skipping homing because calibration was requested but failed or not performed successfully."
                    )
        except MKSServoError as e:
            self._error_state = e
            logger.error(f"Axis '{self.name}': Initialization failed: {e}")
            raise
        logger.info(f"Axis '{self.name}' initialization complete.")

    async def set_work_mode(self, mode_const: int) -> None:
        """
        Sets the working mode of the MKS servo motor.

        The work mode determines the control strategy (e.g., open-loop, closed-loop, FOC)
        and whether the motor responds to pulse/direction signals or serial (CAN) commands.
        Refer to `mks_servo_can.constants.WORK_MODES` for available mode constants.

        Args:
            mode_const: The work mode constant (e.g., `const.MODE_SR_VFOC` for
                        Serial FOC mode, `const.MODE_CR_OPEN` for Pulse Open-loop).

        Raises:
            ParameterError: If the provided `mode_const` is not a valid work mode.
            MotorError: If the motor reports failure to set the work mode.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        if mode_const not in const.WORK_MODES:
            raise ParameterError(f"Invalid work mode constant: {mode_const}")
        logger.info(
            f"Axis '{self.name}': Setting work mode to {const.WORK_MODES.get(mode_const, 'Unknown')} ({mode_const})."
        )
        await self._low_level_api.set_work_mode(self.can_id, mode_const)

    async def calibrate_encoder(self) -> None:
        """
        Initiates the encoder calibration sequence on the motor.

        Calibration is often required for closed-loop operation to ensure the
        encoder's readings are correctly interpreted. The motor must typically
        be unloaded for successful calibration. This method updates the
        internal `_is_calibrated` status of the axis.

        Raises:
            CalibrationError: If the motor reports a calibration failure (e.g., status code 0x02).
            MotorError: For other motor-reported errors during the command.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        logger.info(f"Axis '{self.name}': Starting encoder calibration...")
        self._is_calibrated = False # Mark as not calibrated until success
        try:
            await self._low_level_api.calibrate_encoder(self.can_id)
            self._is_calibrated = True
            self._error_state = None
            logger.info(f"Axis '{self.name}': Encoder calibration successful.")
        except CalibrationError as e:
            # self._is_calibrated remains False
            self._error_state = e
            logger.error(f"Axis '{self.name}': Encoder calibration failed: {e}")
            raise
        except MKSServoError as e: # Catch other communication/motor errors
            self._error_state = e
            logger.error(f"Axis '{self.name}': Error during encoder calibration: {e}")
            raise


    async def home_axis(
        self, wait_for_completion: bool = True, timeout: float = 30.0
    ) -> None:
        """
        Commands the motor to perform its homing sequence.

        Homing procedures vary by motor and configuration but typically involve
        moving the motor to a known reference position (e.g., against a limit switch
        or using a sensorless stall detection method). This method ensures the motor
        is enabled before starting the sequence.

        If `wait_for_completion` is True, this method will monitor the motor's status
        or wait for a completion signal (if supported and active responses are enabled)
        until the homing is finished or the `timeout` is reached.
        On successful completion, the axis's internal `_is_homed` flag is set to True,
        and its position is typically set to 0.

        Args:
            wait_for_completion: If True (default), the method waits for the homing
                                 process to complete. If False, it initiates homing
                                 and returns, allowing other operations to proceed.
            timeout: Maximum time in seconds to wait for homing completion if
                     `wait_for_completion` is True. Defaults to 30.0 seconds.

        Raises:
            CalibrationError: If attempting to home an axis that is not marked as calibrated.
            HomingError: If the homing process fails (e.g., motor reports failure,
                         or times out waiting for completion signal).
            CommunicationError: On CAN communication issues or timeout waiting for responses.
            MotorError: For other motor-reported errors during the homing attempt.
        """
        if not self._is_calibrated:
            msg = f"Axis '{self.name}': Cannot home, axis is not calibrated."
            logger.error(msg)
            raise CalibrationError(msg, can_id=self.can_id) # Raise CalibrationError instead of HomingError

        if not self.is_enabled():
            await self.enable_motor()

        logger.info(f"Axis '{self.name}': Starting homing sequence...")
        self._is_homed = False # Mark as not homed until confirmed
        try:
            initial_status = await self._low_level_api.go_home(self.can_id)
            
            if initial_status == const.HOME_START:
                if wait_for_completion:
                    logger.info(
                        f"Axis '{self.name}': Homing started, waiting for completion (timeout: {timeout}s)..."
                    )
                    
                    def home_completion_predicate(msg: CanMessage) -> bool:
                        # Check if data exists and has at least 2 bytes (command_echo, status)
                        return msg.data is not None and len(msg.data) >= 2 and \
                               msg.data[0] == const.CMD_GO_HOME and \
                               msg.data[1] in [const.HOME_SUCCESS, const.HOME_FAIL]

                    completion_future = self._can_if.create_response_future(
                        self.can_id, const.CMD_GO_HOME, response_predicate=home_completion_predicate
                    )
                    final_response_msg = await asyncio.wait_for(completion_future, timeout=timeout)
                    final_status = final_response_msg.data[1]

                    if final_status == const.HOME_SUCCESS:
                        self._is_homed = True
                        self._current_position_steps = 0 # Typically homing sets zero
                        self._error_state = None
                        logger.info(f"Axis '{self.name}': Homing successful via async completion.")
                        return
                    else: # HOME_FAIL or unexpected
                        raise HomingError(
                            f"Axis '{self.name}': Homing failed or unexpected async status {final_status}.",
                            error_code=final_status, can_id=self.can_id
                        )
                else: # Not waiting for completion
                    # self._is_homed remains False as completion is not confirmed
                    logger.info(f"Axis '{self.name}': Homing initiated, not waiting for completion.")
            elif initial_status == const.HOME_SUCCESS: # Immediate success
                self._is_homed = True
                self._current_position_steps = 0
                self._error_state = None
                logger.info(f"Axis '{self.name}': Homing successful (immediate).")
            else: # HOME_FAIL or other unexpected initial status
                raise HomingError(
                    f"Axis '{self.name}': Homing command failed to start or reported immediate failure. Status: {initial_status}",
                    error_code=initial_status, can_id=self.can_id
                )
        except HomingError as e:
            self._is_homed = False # Ensure homed status is false on any homing error
            self._error_state = e
            logger.error(f"Axis '{self.name}': Homing failed: {e}")
            raise
        except asyncio.TimeoutError as e_timeout: # Specific catch for asyncio.TimeoutError
            self._is_homed = False
            msg = f"Axis '{self.name}': Timeout waiting for homing completion signal."
            logger.error(msg)
            self._error_state = HomingError(msg, can_id=self.can_id)
            raise self._error_state from e_timeout
        except MKSServoError as e_mks: # Catch other MKS errors
            self._is_homed = False
            self._error_state = e_mks
            logger.error(f"Axis '{self.name}': Error during homing: {e_mks}")
            raise


    async def set_current_position_as_zero(self) -> None:
        """
        Defines the motor's current physical position as the zero reference point (origin).

        This command directly instructs the motor to consider its current location
        as the 0-pulse or 0-unit position. It typically also sets the homed status
        of the axis to True, as a defined zero point is established.

        Raises:
            MotorError: If the motor reports failure to set the current position as zero.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        logger.info(f"Axis '{self.name}': Setting current position as zero.")
        await self._low_level_api.set_current_axis_to_zero(self.can_id)
        self._current_position_steps = 0
        self._is_homed = True # Setting zero often implies a homed state relative to this new zero
        self._error_state = None

    async def _execute_move(
        self,
        move_command_func, # type: ignore
        command_const: int,
        success_status: int = const.POS_RUN_COMPLETE,
        pulses_to_move: Optional[int] = None, 
        speed_param_for_calc: Optional[int] = None,
    ) -> None:
        """
        Internal helper to execute a positional move command and manage its lifecycle.

        This method handles sending the low-level move command, creating an
        `asyncio.Future` to track its completion, and processing the motor's
        response(s). It supports dynamic timeout calculation based on the move
        distance and speed if relevant parameters are provided.

        Args:
            move_command_func (Callable[[], Awaitable[int]]): An async callable that, when executed,
                sends the specific low-level move command to the motor and returns
                the initial status code from the motor (e.g., `POS_RUN_STARTING`).
            command_const (int): The MKS CAN command constant for this type of move
                                 (e.g., `const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES`).
            success_status (int): The expected status code from the motor upon successful
                                  completion of the move. Defaults to `const.POS_RUN_COMPLETE`.
            pulses_to_move (Optional[int]): The absolute number of pulses for the move.
                                            Used for calculating a dynamic timeout. If None,
                                            a default timeout is used.
            speed_param_for_calc (Optional[int]): The MKS speed parameter (0-3000) used for the move.
                                                  Also used for dynamic timeout calculation.

        Raises:
            CommunicationError: If the initial command send or response wait times out,
                                or if the listener is not active.
            MotorError: If the motor reports an immediate failure to start the move,
                        or if the move completes with a failure status.
            LimitError: If the motor reports that the move was stopped by an end limit.
            MKSServoError: For other unexpected errors during the move execution.
        """
        if self._active_move_future and not self._active_move_future.done():
            logger.warning(f"Axis '{self.name}': Active move future exists. Cancelling previous move.")
            self._active_move_future.cancel("New move initiated")
            await asyncio.sleep(0)


        if not self.is_enabled():
            await self.enable_motor()

        self._active_move_future = self._loop.create_future()
        
        logger.info(f"Axis '{self.name}'._execute_move: Called for CMD={command_const:02X}. Current active_move_future: {self._active_move_future}")

        # Dynamic timeout calculation
        move_timeout: float = const.CAN_TIMEOUT_SECONDS * 10.0 # Default timeout

        if pulses_to_move is not None and speed_param_for_calc is not None and speed_param_for_calc > 0:
            try:
                # Max motor RPM from constant, e.g., const.MAX_RPM_VFOC_MODE for VFOC
                # The MKS speed parameter (0-3000) mapping to RPM depends on the work mode.
                # We use MAX_RPM_VFOC_MODE as a general reference if not mode-specific.
                max_rpm_reference = const.MAX_RPM_VFOC_MODE
                
                estimated_motor_rpm = (float(speed_param_for_calc) / 3000.0) * max_rpm_reference
                
                if estimated_motor_rpm > 0.1: # Avoid division by zero or negligible speeds
                    motor_rps = estimated_motor_rpm / 60.0
                    # pulses_to_move are raw motor steps (e.g. from _relative_pulses)
                    # Use kinematics.steps_per_revolution which is motor shaft steps
                    motor_shaft_steps_per_sec = motor_rps * self.kinematics.steps_per_revolution
                    
                    if motor_shaft_steps_per_sec > 1.0: # Ensure meaningful speed
                        estimated_duration_s = abs(pulses_to_move) / motor_shaft_steps_per_sec
                        # Buffer: e.g., 3x CAN_TIMEOUT_SECONDS + 25% of duration for processing/acceleration
                        buffer_s = const.CAN_TIMEOUT_SECONDS * 3.0 
                        calculated_timeout = estimated_duration_s * 1.25 + buffer_s 
                        # Ensure a minimum sensible timeout, e.g., 5 times the base CAN timeout
                        move_timeout = max(calculated_timeout, const.CAN_TIMEOUT_SECONDS * 5.0) 
                        logger.debug(
                            f"Axis '{self.name}': Calculated move_timeout: {move_timeout:.2f}s "
                            f"for {abs(pulses_to_move)} pulses at speed_param {speed_param_for_calc} "
                            f"(est. duration: {estimated_duration_s:.2f}s)"
                        )
            except Exception as e_timeout_calc: # pylint: disable=broad-except
                logger.warning(
                    f"Axis '{self.name}': Error calculating dynamic timeout, using default {move_timeout:.2f}s. Error: {e_timeout_calc}"
                )
        
        logger.info(f"Axis '{self.name}': Using move timeout of {move_timeout:.2f}s for CMD {command_const:02X}.")


        try:
            logger.info(f"Axis '{self.name}'._execute_move: Awaiting move_command_func() for CMD={command_const:02X}...")
            initial_status = await move_command_func()
            logger.info(f"Axis '{self.name}'._execute_move: move_command_func() for CMD={command_const:02X} returned initial_status={initial_status:02X}")


            if initial_status == const.POS_RUN_STARTING: # 0x01
                logger.debug(
                    f"Axis '{self.name}': Move (CMD={command_const:02X}) reported STARTING. Waiting for completion signal..."
                )
                
                def move_completion_predicate(msg: CanMessage) -> bool:
                    return (msg.data is not None and len(msg.data) >= 2 and
                            msg.data[0] == command_const and # Echoed command
                            msg.data[1] in [success_status, const.POS_RUN_END_LIMIT_STOPPED, const.POS_RUN_FAIL])

                completion_future = self._can_if.create_response_future(
                    self.can_id, command_const, response_predicate=move_completion_predicate
                )
                final_response_msg = await asyncio.wait_for(
                    completion_future, timeout=move_timeout
                )
                final_status = final_response_msg.data[1]
                logger.info(f"Axis '{self.name}': Move (CMD={command_const:02X}) async completion received with status={final_status:02X}")

                if final_status == success_status:
                    logger.info(
                        f"Axis '{self.name}': Move (CMD={command_const:02X}) completed successfully (async)."
                    )
                    if not self._active_move_future.done(): self._active_move_future.set_result(True)
                elif final_status == const.POS_RUN_END_LIMIT_STOPPED:
                    msg = f"Axis '{self.name}': Move (CMD={command_const:02X}) stopped by end limit (async)."
                    logger.warning(msg)
                    if not self._active_move_future.done(): self._active_move_future.set_exception(
                        LimitError(msg, error_code=final_status, can_id=self.can_id)
                    )
                else: # Includes POS_RUN_FAIL
                    msg = f"Axis '{self.name}': Move (CMD={command_const:02X}) failed or completed with unexpected async status {final_status}."
                    logger.error(msg)
                    if not self._active_move_future.done(): self._active_move_future.set_exception(
                        MotorError(msg, error_code=final_status, can_id=self.can_id)
                    )

            elif initial_status == success_status: 
                logger.info(
                    f"Axis '{self.name}': Move (CMD={command_const:02X}) reported COMPLETE in initial response."
                )
                if not self._active_move_future.done(): self._active_move_future.set_result(True)
            elif initial_status == const.POS_RUN_END_LIMIT_STOPPED: 
                msg = f"Axis '{self.name}': Move (CMD={command_const:02X}) reported END LIMIT in initial response."
                logger.warning(msg)
                if not self._active_move_future.done(): self._active_move_future.set_exception(
                    LimitError(msg, error_code=initial_status, can_id=self.can_id)
                )
            else:  # Includes POS_RUN_FAIL (0x00) or other unexpected initial status
                msg = f"Axis '{self.name}': Move (CMD={command_const:02X}) command failed to start properly or other error. Initial status: {initial_status:02X}"
                logger.error(msg)
                if not self._active_move_future.done(): self._active_move_future.set_exception(
                    MotorError(msg, error_code=initial_status, can_id=self.can_id)
                )
            
            await self.get_current_position_steps()

        except asyncio.TimeoutError:
            msg = f"Axis '{self.name}': Timeout waiting for move (CMD={command_const:02X}) response/completion."
            logger.error(msg)
            if not self._active_move_future.done():
                self._active_move_future.set_exception(CommunicationError(msg, can_id=self.can_id))
        except MKSServoError as e: 
            logger.error(
                f"Axis '{self.name}': MKSServoError during move (CMD={command_const:02X}): {e}"
            )
            if not self._active_move_future.done():
                self._active_move_future.set_exception(e)
        except Exception as e: 
            logger.error(
                f"Axis '{self.name}': Unexpected Exception during move (CMD={command_const:02X}): {e}", exc_info=True
            )
            if not self._active_move_future.done():
                self._active_move_future.set_exception(MKSServoError(f"Unexpected move error: {e}", can_id=self.can_id))
        finally:
            if self._active_move_future and not self._active_move_future.done():
                logger.error(f"Axis '{self.name}'._execute_move: Future for CMD={command_const:02X} was not resolved. Setting generic error.")
                self._active_move_future.set_exception(
                    MKSServoError(f"Move future for {command_const:02X} ended without explicit result/error.", can_id=self.can_id)
                )


    async def move_to_position_abs_pulses(
        self,
        target_pulses: int,
        speed_param: Optional[int] = None,
        accel_param: Optional[int] = None,
        wait: bool = True,
    ):
        """
        Moves the motor to an absolute target position specified in raw motor pulses/steps.

        This command uses the MKS "absolute motion by pulses" (e.g., 0xFE).

        Args:
            target_pulses: The absolute target position in motor pulses (signed 24-bit).
            speed_param: MKS speed parameter (0-3000). If None, uses `self.default_speed_param`.
            accel_param: MKS acceleration parameter (0-255). If None, uses `self.default_accel_param`.
            wait: If True (default), the method waits for the move to complete before returning.
                  If False, the move is initiated, and the method returns, allowing other
                  operations to proceed concurrently. The completion can be awaited later
                  via `wait_for_move_completion()` or by checking `is_move_complete()`.

        Raises:
            ParameterError: If `speed_param`, `accel_param`, or `target_pulses` are out of range.
            CommunicationError: If CAN communication fails or times out.
            MotorError: If the motor reports an error starting or during the move.
            LimitError: If the motor reports hitting a limit switch during the move.
        """
        sp = speed_param if speed_param is not None else self.default_speed_param
        ac = accel_param if accel_param is not None else self.default_accel_param
        
        # For dynamic timeout, estimate pulses_to_move
        current_pos_for_delta = self._current_position_steps
        if current_pos_for_delta is None: # Fetch if not available
            try:
                current_pos_for_delta = await self.get_current_position_steps()
            except MKSServoError: # If fetching fails, cannot calculate delta
                current_pos_for_delta = None # Fallback to default timeout later
        
        delta_pulses_for_timeout = abs(target_pulses - current_pos_for_delta) if current_pos_for_delta is not None else None


        logger.info(
            f"Axis '{self.name}': Moving to absolute pulse position {target_pulses} (SpeedP: {sp}, AccelP: {ac})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(
            self.can_id, sp, ac, target_pulses
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,
                                 pulses_to_move=delta_pulses_for_timeout, speed_param_for_calc=sp)
        if wait and self._active_move_future:
            await self._active_move_future

    async def move_to_position_abs_user(
        self,
        target_pos_user: float,
        speed_user: Optional[float] = None,
        wait: bool = True,
    ):
        """
        Moves the motor to an absolute target position specified in user-defined units.

        The user-defined position and speed are converted to motor pulses and MKS speed
        parameters using the axis's configured kinematics object. This command then uses
        the MKS "absolute motion by pulses" (e.g., 0xFE).

        Args:
            target_pos_user: The absolute target position in user units (e.g., mm, degrees).
            speed_user: The desired speed in user units per second. If None, the axis's
                        `default_speed_param` (converted via kinematics) is used.
            wait: If True (default), waits for the move to complete. If False, initiates
                  the move and returns.

        Raises:
            KinematicsError: If conversion between user units and motor steps/speed fails.
            ParameterError: If converted parameters are out of the motor's valid range.
            CommunicationError: On CAN communication issues.
            MotorError: If the motor reports an error.
            LimitError: If a limit is hit.
        """
        target_steps = self.kinematics.user_to_steps(target_pos_user)
        mks_speed_param = (
            self.kinematics.user_speed_to_motor_speed(speed_user)
            if speed_user is not None
            else self.default_speed_param
        )
        mks_accel_param = self.default_accel_param # Kept for command, not used in current timeout calc
        
        current_pos_steps_for_delta = self._current_position_steps
        if current_pos_steps_for_delta is None:
            try:
                current_pos_steps_for_delta = await self.get_current_position_steps()
            except MKSServoError:
                current_pos_steps_for_delta = None
        
        delta_steps_for_timeout = abs(target_steps - current_pos_steps_for_delta) if current_pos_steps_for_delta is not None else None

        logger.info(
            f"Axis '{self.name}': Moving to user position {target_pos_user} ({getattr(self.kinematics, 'units', 'units')}) "
            f"-> {target_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(
            self.can_id, mks_speed_param, mks_accel_param, target_steps
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,
                                 pulses_to_move=delta_steps_for_timeout, speed_param_for_calc=mks_speed_param)
        if wait and self._active_move_future:
            await self._active_move_future

    async def move_relative_pulses(
        self,
        relative_pulses: int,
        speed_param: Optional[int] = None,
        accel_param: Optional[int] = None,
        wait: bool = True,
    ):
        """
        Moves the motor by a relative distance specified in raw motor pulses/steps.

        This command uses the MKS "relative motion by pulses" (e.g., 0xFD).
        A positive `relative_pulses` value typically corresponds to counter-clockwise (CCW)
        motion, and a negative value to clockwise (CW), though this can be influenced
        by motor configuration or wiring.

        Args:
            relative_pulses: The number of pulses to move relative to the current position.
                             Positive for one direction, negative for the other.
            speed_param: MKS speed parameter (0-3000). If None, uses `self.default_speed_param`.
            accel_param: MKS acceleration parameter (0-255). If None, uses `self.default_accel_param`.
            wait: If True (default), waits for the move to complete.

        Raises:
            ParameterError: If parameters are out of range.
            CommunicationError: On CAN communication issues.
            MotorError: If the motor reports an error.
            LimitError: If a limit is hit.
        """
        sp = speed_param if speed_param is not None else self.default_speed_param
        ac = accel_param if accel_param is not None else self.default_accel_param
        direction_ccw = relative_pulses >= 0 # Treat 0 as CCW for consistency if it means no move
        num_pulses_for_cmd = abs(relative_pulses) # Command takes magnitude
        num_pulses_for_timeout = num_pulses_for_cmd

        logger.info(
            f"Axis '{self.name}': Moving relative by {relative_pulses} pulses (SpeedP: {sp}, AccelP: {ac})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, sp, ac, num_pulses_for_cmd
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES,
                                 pulses_to_move=num_pulses_for_timeout, speed_param_for_calc=sp)
        if wait and self._active_move_future:
            await self._active_move_future

    async def move_relative_user(
        self,
        relative_dist_user: float,
        speed_user: Optional[float] = None,
        wait: bool = True,
    ):
        """
        Moves the motor by a relative distance specified in user-defined units.

        The user-defined distance and speed are converted to motor pulses and MKS speed
        parameters using the axis's kinematics. This command then uses the MKS
        "relative motion by pulses" (e.g., 0xFD).

        Args:
            relative_dist_user: The distance to move in user units (e.g., mm, degrees).
                                Positive for one direction, negative for the other.
            speed_user: The desired speed in user units per second. If None, the
                        axis's default speed is used (converted via kinematics).
            wait: If True (default), waits for the move to complete.

        Raises:
            KinematicsError: If conversion fails.
            ParameterError: If converted parameters are out of range.
            CommunicationError: On CAN issues.
            MotorError: If the motor reports an error.
            LimitError: If a limit is hit.
        """
        relative_steps = self.kinematics.user_to_steps(relative_dist_user)
        mks_speed_param = (
            self.kinematics.user_speed_to_motor_speed(speed_user)
            if speed_user is not None
            else self.default_speed_param
        )
        mks_accel_param = self.default_accel_param # Kept for command
        direction_ccw = relative_steps >= 0
        num_steps_for_cmd = abs(relative_steps)
        num_steps_for_timeout = num_steps_for_cmd

        logger.info(
            f"Axis '{self.name}': Moving relative by {relative_dist_user} {getattr(self.kinematics, 'units', 'units')} "
            f"-> {relative_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, mks_speed_param, mks_accel_param, num_steps_for_cmd
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES,
                                 pulses_to_move=num_steps_for_timeout, speed_param_for_calc=mks_speed_param)
        if wait and self._active_move_future:
            await self._active_move_future
            
    async def set_speed_user(
        self, speed_user: float, accel_user: Optional[float] = None 
    ) -> None:
        """
        Sets the motor to run continuously at a specified speed in user units (speed/velocity mode).

        This command uses the MKS speed mode command (0xF6). To stop the motor
        when in speed mode, call this method again with `speed_user = 0.0`, or use
        the `stop_motor()` method.

        The direction of rotation is determined by the sign of `speed_user`.

        Args:
            speed_user: The desired speed in user-defined units per second (e.g., mm/s, deg/s).
                        A positive value typically results in counter-clockwise (CCW) rotation,
                        and a negative value in clockwise (CW) rotation, subject to motor
                        and kinematics configuration.
            accel_user (Optional[float]): Desired acceleration in user units per second squared.
                                          If provided, this would ideally be converted to an MKS
                                          acceleration parameter via kinematics.
                                          (Note: Current implementation uses `default_accel_param`
                                          as direct conversion from `accel_user` to MKS `accel_param`
                                          is not fully implemented in standard kinematics classes.)

        Raises:
            MotorError: If the motor fails to enter speed mode or if the command is rejected.
            CommunicationError: On CAN communication issues (e.g., timeout).
            KinematicsError: If an error occurs during the conversion of `speed_user`
                             to MKS speed parameters via the kinematics object.
        """
        mks_speed_param = self.kinematics.user_speed_to_motor_speed(abs(speed_user))
        # TODO: Convert accel_user to MKS accel_param if accel_user is provided.
        # This would require kinematics.acceleration_to_motor_acceleration_param()
        # For now, using default_accel_param.
        mks_accel_param = self.default_accel_param 
        direction_ccw = speed_user >= 0

        if abs(speed_user) < 1e-6: # Effectively zero speed (using a small epsilon)
            logger.info(
                f"Axis '{self.name}': Stopping continuous speed mode (AccelP: {mks_accel_param})."
            )
            await self._low_level_api.stop_speed_mode(self.can_id, mks_accel_param)
        else:
            logger.info(
                f"Axis '{self.name}': Setting continuous speed to {speed_user} {getattr(self.kinematics, 'units', 'units')}/s "
                f"(Dir: {'CCW' if direction_ccw else 'CW'}, SpeedP: {mks_speed_param}, AccelP: {mks_accel_param})."
            )
            await self._low_level_api.run_speed_mode(
                self.can_id, direction_ccw, mks_speed_param, mks_accel_param
            )
        
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Speed mode initiated, cancelling positional move future.")
        self._active_move_future = None


    async def stop_motor(self, deceleration_param: Optional[int] = None) -> None:
        """
        Commands the motor to stop its current movement with controlled deceleration.

        This method primarily targets stopping a motor running in speed mode (using MKS
        command 0xF6 with speed 0). For positional moves, it cancels any active
        move future. If the graceful stop command fails, an emergency stop might be
        attempted as a fallback.

        Args:
            deceleration_param (Optional[int]): The MKS deceleration parameter (0-255).
                If None, the axis's `default_accel_param` is used for deceleration.
                A value of 0 typically implies an immediate stop, though the motor's
                firmware ultimately controls the exact behavior.

        Raises:
            MotorError: If the stop command fails (e.g., motor rejects the command).
            CommunicationError: On CAN communication issues.
        """
        decel = deceleration_param if deceleration_param is not None else self.default_accel_param
        logger.info(f"Axis '{self.name}': Commanding stop (decel_param: {decel}).")
        try:
            # Command 0xF6 with speed 0 is the standard way to stop speed mode.
            await self._low_level_api.stop_speed_mode(self.can_id, decel)
            # For positional moves, need to send a stop for those commands (e.g., 0xFD with 0 pulses/speed)
            # This stop_motor is more for speed mode. An explicit "cancel_move" might be better for positional.
            # For now, ensure any active positional move future is cancelled.
            if self._active_move_future and not self._active_move_future.done():
                self._active_move_future.cancel("Motor stopped by explicit command")
        except MKSServoError as e:
            logger.warning(
                f"Axis '{self.name}': Graceful stop command failed ({e}), attempting emergency stop."
            )
            await self.emergency_stop() # This will also cancel active_move_future
        finally: 
            if self._active_move_future and not self._active_move_future.done():
                 self._active_move_future.cancel("Motor stop initiated")
            self._active_move_future = None


    async def emergency_stop(self) -> None:
        """
        Commands an immediate emergency stop of the motor (MKS command 0xF7).

        This command is intended to halt all motor activity as quickly as possible,
        potentially bypassing controlled deceleration ramps. Any active move future
        associated with this axis will be cancelled.

        Raises:
            MotorError: If the motor reports failure to process the emergency stop command.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        logger.warning(f"Axis '{self.name}': Performing EMERGENCY STOP.")
        await self._low_level_api.emergency_stop(self.can_id)
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Motor emergency stopped")
        self._active_move_future = None # Clear the future as the move is aborted

    async def enable_motor(self) -> None:
        """
        Enables the motor's servo loop, allowing it to hold position and accept movement commands.

        This method sends the MKS "enable motor" command (0xF3 with enable=1).
        If the motor is already considered enabled (based on its cached state),
        this method logs that and does nothing further to avoid redundant commands.
        On successful enabling, any previous error state cached in `self._error_state`
        is cleared. Updates the internal `_is_enabled` state.

        Raises:
            MotorError: If the motor reports failure to enable.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        if not self._is_enabled:
            logger.info(f"Axis '{self.name}': Enabling motor.")
            await self._low_level_api.enable_motor(self.can_id, True)
            self._is_enabled = True
            self._error_state = None # Clear previous error on successful enable
        else:
            logger.info(f"Axis '{self.name}': Motor already enabled.")

    async def disable_motor(self) -> None:
        """
        Disables the motor's servo loop, releasing it from actively holding position.

        This method sends the MKS "disable motor" command (0xF3 with enable=0).
        If the motor is already considered disabled (based on its cached state),
        this method logs that and does nothing further. If a positional move
        was in progress (`_active_move_future` was active), its future is
        cancelled with a `MotorError` indicating it was interrupted by the disable command.
        Updates the internal `_is_enabled` state.

        Raises:
            MotorError: If the motor reports failure to disable.
            CommunicationError: On CAN communication issues (e.g., timeout).
        """
        if self._is_enabled:
            logger.info(f"Axis '{self.name}': Disabling motor.")
            await self._low_level_api.enable_motor(self.can_id, False)
            self._is_enabled = False
            if self._active_move_future and not self._active_move_future.done():
                logger.info(f"Axis '{self.name}': Cancelling active move due to disable command.")
                self._active_move_future.set_exception(MotorError("Motor disabled during move", can_id=self.can_id))
        else:
            logger.info(f"Axis '{self.name}': Motor already disabled.")

    def is_enabled(self) -> bool:
        """
        Returns the cached enabled state of the motor.

        This method does not send a command to the motor; it reflects the last known
        state as updated by methods like `enable_motor()`, `disable_motor()`,
        `read_en_status()`, or `initialize()`. To query the motor's current
        enable status directly, use `await axis.read_en_status()`.

        Returns:
            True if the motor is believed to be enabled based on its cached state,
            False otherwise.
        """
        return self._is_enabled

    async def read_en_status(self) -> bool:
        """
        Actively reads and returns the current enable (EN pin/servo) status from the motor.

        This method queries the motor directly using command 0x3A.
        The internal `_is_enabled` state of the axis object is updated with the result.

        Returns:
            True if the motor reports it is currently enabled, False otherwise.

        Raises:
            CommunicationError: On CAN communication issues (e.g., timeout).
            MotorError: For motor-reported errors during the read attempt.
        """
        self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
        return self._is_enabled

    def is_homed(self) -> bool:
        """
        Returns the cached homed state of the motor.

        This state is typically updated to True after a successful `home_axis()`
        or `set_current_position_as_zero()` operation. It reflects whether
        the axis has a known and established zero reference point.

        Returns:
            True if the motor is considered homed based on its cached state,
            False otherwise.
        """
        return self._is_homed

    def is_calibrated(self) -> bool:
        """
        Returns the cached calibration state of the motor's encoder.

        This state is updated to True after a successful `calibrate_encoder()`
        operation or is assumed True on successful `initialize()` if calibration
        is not explicitly requested or fails. An uncalibrated motor may not
        perform accurately in closed-loop modes.

        Returns:
            True if the motor's encoder is considered calibrated based on its
            cached state, False otherwise.
        """
        return self._is_calibrated

    async def get_current_position_steps(self) -> int:
        """
        Reads and returns the motor's current position in raw encoder steps/pulses.

        This method actively queries the motor using the MKS command 0x31
        (Read Encoder Accumulated Value). The internal `_current_position_steps`
        state of the axis object is updated with the result.

        Returns:
            The current motor position as an integer number of encoder steps.

        Raises:
            CommunicationError: On CAN communication issues (e.g., timeout).
            MotorError: For motor-reported errors during the read attempt.
        """
        pos_steps = await self._low_level_api.read_encoder_value_addition(self.can_id)
        self._current_position_steps = pos_steps
        return pos_steps

    async def get_current_position_user(self) -> float:
        """
        Reads the motor's current position and converts it to user-defined units.

        This method first calls `get_current_position_steps()` to obtain the raw
        step count from the motor. It then applies the conversion logic defined
        in the axis's `kinematics` object (e.g., `steps_to_user` method) to
        translate this step count into physical units like millimeters or degrees.

        Returns:
            The current motor position in user-defined units (e.g., mm, degrees),
            as a floating-point number.

        Raises:
            CommunicationError: If reading raw steps from the motor fails.
            MotorError: For motor-reported errors during the raw position read.
            KinematicsError: If an error occurs during the kinematic conversion
                             (e.g., division by zero if kinematics are misconfigured).
        """
        steps = await self.get_current_position_steps()
        return self.kinematics.steps_to_user(steps)

    async def get_current_speed_rpm(self) -> int:
        """
        Reads and returns the motor's current speed in Revolutions Per Minute (RPM).

        This method actively queries the motor using MKS command 0x32 (Read Motor Speed RPM).
        The internal `_current_speed_rpm` state of the axis object is updated.
        The speed is typically returned as a signed integer by the motor, where
        positive values may indicate one direction (e.g., CCW) and negative values
        the other (e.g., CW).

        Returns:
            The current motor speed in RPM as an integer.

        Raises:
            CommunicationError: On CAN communication issues (e.g., timeout).
            MotorError: For motor-reported errors during the read attempt.
        """
        rpm = await self._low_level_api.read_motor_speed_rpm(self.can_id)
        self._current_speed_rpm = rpm
        return rpm

    async def get_current_speed_user(self) -> float:
        """
        Reads the motor's current speed in RPM and converts it to user-defined speed units.

        This method first queries the motor for its speed in RPM using
        `get_current_speed_rpm()`. It then converts this RPM value into user-defined
        speed units (e.g., mm/s, degrees/s) using the axis's configured
        `kinematics` object.

        The conversion logic depends on the type of kinematics:
        - For `RotaryKinematics`, it uses the `degrees_per_output_revolution` and `gear_ratio`.
        - For `LinearKinematics`, it uses the `pitch` and `gear_ratio`.
        - For other or custom kinematics, the conversion might be an approximation if
          a direct RPM-to-user-speed method isn't explicitly defined.

        Returns:
            The current motor speed in user-defined units per second (e.g., mm/s, deg/s),
            as a floating-point number. Returns 0.0 if conversion is not clearly defined
            for the kinematics type or if essential parameters (like gear_ratio) are zero.

        Raises:
            CommunicationError: If reading RPM from the motor fails.
            MotorError: For motor-reported errors during the RPM read.
            KinematicsError: If an error occurs during kinematic conversion.
        """
        rpm = await self.get_current_speed_rpm()
        
        # The kinematics.motor_speed_to_user_speed typically expects MKS speed param (0-3000).
        # Directly converting RPM to user speed requires reversing the logic in
        # kinematics.user_speed_to_motor_speed or having a dedicated RPM input method.
        # For a more direct conversion based on RPM:
        motor_revs_per_second = float(rpm) / 60.0
        if self.kinematics.gear_ratio == 0: # Avoid division by zero
            logger.warning(f"Axis '{self.name}': Gear ratio is zero, cannot convert RPM to user speed.")
            return 0.0
        output_revs_per_second = motor_revs_per_second / self.kinematics.gear_ratio

        # Check specific kinematics types for their properties
        if hasattr(self.kinematics, 'degrees_per_output_revolution'): # RotaryKinematics
            if self.kinematics.degrees_per_output_revolution == 0: # type: ignore
                logger.warning(f"Axis '{self.name}': degrees_per_output_revolution is zero in RotaryKinematics.")
                return 0.0
            return output_revs_per_second * self.kinematics.degrees_per_output_revolution # type: ignore
        elif hasattr(self.kinematics, 'pitch'): # LinearKinematics
            if self.kinematics.pitch == 0: # type: ignore
                logger.warning(f"Axis '{self.name}': pitch is zero in LinearKinematics.")
                return 0.0
            return output_revs_per_second * self.kinematics.pitch # type: ignore
        elif hasattr(self.kinematics, 'arm_length') and hasattr(self.kinematics, '_motor_angle_deg_to_user'): # EccentricKinematics like
            logger.warning(
                f"Axis '{self.name}': get_current_speed_user for {type(self.kinematics).__name__} from RPM is an approximation (simplified for center motion)."
            )
            if hasattr(self.kinematics, 'arm_length') and self.kinematics.arm_length != 0: # type: ignore
                output_angular_speed_rad_per_sec = output_revs_per_second * 2 * math.pi
                return self.kinematics.arm_length * output_angular_speed_rad_per_sec # type: ignore
            return 0.0 # Fallback for eccentric if arm_length is zero


        logger.warning(f"User speed calculation from RPM not precisely defined for {type(self.kinematics)}.")
        return 0.0 # Fallback if no clear conversion path


    async def get_status_dict(self) -> Dict[str, Any]:
        """
        Retrieves a comprehensive dictionary containing various status information about the axis.

        This method actively queries the motor for core status parameters like
        current position, enable state, and motor status code. It then combines
        this with cached information (e.g., homed state, calibrated state, last error)
        and kinematic details to provide a snapshot of the axis's current condition.

        Returns:
            A dictionary where keys are status item names (strings) and values
            are their corresponding states or values. Example keys include:
            `'name'`, `'can_id'`, `'timestamp'`, `'position_steps'`, `'position_user'`,
            `'position_units'`, `'is_enabled'`, `'is_homed'`, `'is_calibrated'`,
            `'motor_status_code'`, `'motor_status_str'`, `'error_state'`,
            `'active_move_in_progress'`.
            If fetching status fails, a partial dictionary with an
            `'error_during_status_fetch'` key may be returned.

        Raises:
            CommunicationError: If querying essential motor status (like position or
                                motor status code) from the hardware fails.
            MotorError: For motor-reported errors during these status queries.
        """
        timestamp = time.time() # Get timestamp at the beginning
        try:
            # Actively query essential information
            pos_steps = await self.get_current_position_steps() # Updates self._current_position_steps
            is_en = await self.read_en_status() # Updates self._is_enabled
            raw_motor_status = await self.get_motor_status_code()
            # self._error_state might be stale, consider reading error parameter if needed for fresh status
            # For now, use cached error state.
        except MKSServoError as e:
            logger.error(f"Axis '{self.name}': Failed to get full status due to: {e}")
            # Return partial status or re-raise, depending on desired behavior
            # For now, return what we have plus the error that occurred
            return {
                "name": self.name,
                "can_id": self.can_id,
                "timestamp": timestamp,
                "error_during_status_fetch": str(e),
                "is_enabled": self._is_enabled, # Cached
                "is_homed": self._is_homed,     # Cached
                "is_calibrated": self._is_calibrated, # Cached
                "active_move_in_progress": self._active_move_future is not None and not self._active_move_future.done(),
            }


        pos_user = self.kinematics.steps_to_user(pos_steps)
        status_map = const.MOTOR_STATUS_MAP
        
        return {
            "name": self.name,
            "can_id": self.can_id,
            "timestamp": timestamp,
            "position_steps": pos_steps,
            "position_user": pos_user,
            "position_units": getattr(self.kinematics, "units", "unknown"),
            "is_enabled": is_en,
            "is_homed": self._is_homed, # Uses cached after homing operations
            "is_calibrated": self._is_calibrated, # Uses cached after calibration
            "motor_status_code": raw_motor_status,
            "motor_status_str": status_map.get(raw_motor_status, f"Unknown code {raw_motor_status}"),
            "error_state": str(self._error_state) if self._error_state else None, # Last caught error
            "active_move_in_progress": self._active_move_future is not None and not self._active_move_future.done(),
        }

    async def get_motor_status_code(self) -> int:
        """
        Queries and returns the raw motor status code.

        This method sends the MKS "Query Motor Status" command (0xF1) to the motor.
        The returned code indicates the motor's operational state (e.g., stopped,
        speeding up, homing). Refer to `const.MOTOR_STATUS_MAP` for interpretations.

        Returns:
            The raw motor status code as an integer.

        Raises:
            MotorError: If the motor reports a failure to query its status.
            CommunicationError: On CAN communication issues.
        """
        return await self._low_level_api.query_motor_status(self.can_id)

    def is_move_complete(self) -> bool:
        """
        Checks if the last commanded positional move for this axis has completed.

        This status is based on an internal `asyncio.Future` (`_active_move_future`)
        that is created when a move command (like `move_absolute`, `move_relative`)
        is initiated. The future is resolved (marked as "done") when the motor
        signals completion or if an error/timeout occurs.

        Returns:
            True if no move is currently active (i.e., no `_active_move_future` exists
            or it is already done).
            False if a move is currently in progress (i.e., `_active_move_future` exists
            and is not yet done).
        """
        if self._active_move_future:
            return self._active_move_future.done()
        return True # No active move, so trivially complete

    async def wait_for_move_completion(self, timeout: Optional[float] = None) -> None:
        """
        Asynchronously waits until the current move operation for this axis is complete
        or a timeout occurs.

        If no move is currently active (i.e., `_active_move_future` is None or already done),
        this method returns immediately. Otherwise, it awaits the completion of the
        internal move future.

        If the move future itself completes with an exception (e.g., `MotorError`
        if the move failed, `LimitError` if a limit was hit, `CommunicationError` if
        the motor stopped responding), that exception will be re-raised by this method.

        Args:
            timeout (Optional[float]): The maximum time in seconds to wait for the
                                       move to complete. If None (default), waits
                                       indefinitely until the move future resolves
                                       (which might itself have an internal timeout
                                       based on the move parameters).

        Raises:
            asyncio.TimeoutError: If this specific `wait_for_move_completion` call
                                  times out (i.e., the provided `timeout` is reached
                                  before the internal move future completes). This is
                                  distinct from a `CommunicationError` that might occur
                                  within the move operation itself due to CAN-level timeouts.
            MKSServoError (or subclasses like MotorError, LimitError): If the move
                                          itself failed and its tracking future
                                          was resolved with an exception.
        """
        if self._active_move_future and not self._active_move_future.done():
            try:
                # asyncio.wait_for will raise asyncio.TimeoutError on timeout
                await asyncio.wait_for(self._active_move_future, timeout=timeout)
            except asyncio.TimeoutError as e:
                # If asyncio.wait_for times out, we need to make sure the future itself reflects this
                # or is cancelled to prevent it from resolving later unexpectedly.
                # However, the future might still complete normally or with its own error from CAN timeout.
                # The primary purpose here is to limit the synchronous-style wait.
                # If the future ISN'T done after this timeout, it implies the _execute_move's own timeout
                # hasn't been hit OR the move is just very long.
                if self._active_move_future and not self._active_move_future.done():
                    logger.warning(f"Axis '{self.name}': wait_for_move_completion timed out after {timeout}s, "
                                   f"but internal move future still pending.")
                    # Optionally, you could try to cancel the future here if this timeout implies a hard stop
                    # self._active_move_future.cancel("wait_for_move_completion external timeout")
                raise # Re-raise asyncio.TimeoutError to signal the caller's wait timed out
            # If _active_move_future completed with an exception, it will be raised here by the await.
        # If future is already done, this will return immediately or raise stored exception if any.
        
    async def ping(self, timeout: Optional[float] = None) -> bool: # Changed return from Optional[float] to bool
        """
        Pings the motor to check for basic communication and responsiveness.

        This method sends a simple command (typically reading the EN pin status)
        to the motor and waits for a response. It's a lightweight way to verify
        that the motor is on the bus, powered, and able to communicate.

        Args:
            timeout (Optional[float]): The maximum time in seconds for the entire
                                       ping operation (including the underlying CAN
                                       read and any `asyncio.wait_for` logic). If None,
                                       the behavior relies on default timeouts in
                                       lower layers (e.g., `const.CAN_TIMEOUT_SECONDS`
                                       for the CAN read itself).

        Returns:
            True if the motor responds successfully within the timeout, False otherwise
            (e.g., due to communication timeout, motor error, or unexpected issues).
        """
        logger.debug(f"Axis '{self.name}': Pinging motor (CAN ID: {self.can_id:03X})...")
        
        # The actual timeout for the CAN read will be const.CAN_TIMEOUT_SECONDS
        # unless low_level_api.read_en_pin_status itself takes and uses a timeout parameter.
        # If `timeout` here is meant to cap the entire operation:
        async def _do_ping():
            await self._low_level_api.read_en_pin_status(self.can_id)
            return True # Indicate success

        try:
            if timeout is not None:
                await asyncio.wait_for(_do_ping(), timeout=timeout)
            else: # Use default underlying timeouts
                await _do_ping()
            
            logger.info(f"Axis '{self.name}': Ping successful.")
            return True
        except asyncio.TimeoutError: # This catches timeout from asyncio.wait_for
            logger.warning(f"Axis '{self.name}': Ping operation timed out after {timeout}s.")
            return False
        except CommunicationError as e: # This catches CAN level timeouts from _low_level_api
            logger.warning(f"Axis '{self.name}': Ping failed due to communication error: {e}")
            return False
        except MKSServoError as e: 
            logger.warning(f"Axis '{self.name}': Ping failed due to MKS error: {e}")
            return False
        except Exception as e: # pylint: disable=broad-except
            logger.error(f"Axis '{self.name}': Ping failed due to unexpected error: {e}", exc_info=True)
            return False
        