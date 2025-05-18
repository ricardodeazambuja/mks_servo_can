"""
High-Level Axis Class for individual MKS Servo motor control.
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
        """Dummy CanMessage for type hinting when python-can is not available."""
        def __init__(self, arbitration_id=0, data=None, dlc=0):
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
            can_interface_manager: The CANInterface instance used for communication.
            motor_can_id: The CAN ID of the motor (1-2047).
            name: A descriptive name for this axis instance.
            motor_type: The type of MKS servo motor (e.g., const.MOTOR_TYPE_SERVO42D).
            kinematics: A Kinematics object for converting between user units and motor steps.
                        If None, defaults to RotaryKinematics with standard encoder pulses.
            default_speed_param: Default MKS speed parameter (0-3000) for moves if not specified.
            default_accel_param: Default MKS acceleration parameter (0-255) for moves if not specified.
            loop: The asyncio event loop to use. If None, the default loop is used.

        Raises:
            ParameterError: If motor_can_id is invalid.
        """
        if not (
            const.BROADCAST_ADDRESS < motor_can_id <= 0x7FF
        ): # type: ignore[operator] # const.BROADCAST_ADDRESS can be an int
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
        Performs initial communication with the axis.

        This includes a basic communication test, reading the initial enable status,
        and optionally performing encoder calibration and homing.
        Sets `_is_calibrated` to True after successful communication test if not calibrating.

        Args:
            calibrate: If True, attempts to calibrate the encoder.
            home: If True, attempts to home the axis. Homing will only proceed if
                  calibration was successful (if calibrate=True) or if calibration
                  was not requested.

        Raises:
            MKSServoError: Or its subclasses (e.g., CommunicationError, CalibrationError, HomingError)
                           if any part of the initialization process fails.
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
        Sets the working mode of the motor.

        Args:
            mode_const: The work mode constant (e.g., `const.MODE_SR_VFOC`).
                        Refer to `const.WORK_MODES` for available modes.

        Raises:
            ParameterError: If the provided mode_const is not a valid work mode.
            MotorError: If the motor fails to set the work mode.
            CommunicationError: On CAN communication issues.
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

        The motor must typically be unloaded for successful calibration.
        Updates the `_is_calibrated` status of the axis.

        Raises:
            CalibrationError: If the motor reports a calibration failure.
            MotorError: For other motor-reported errors during the command.
            CommunicationError: On CAN communication issues.
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

        Ensures the motor is enabled before starting. If `wait_for_completion` is True,
        this method will wait for the motor to signal completion of the homing process.
        Updates the `_is_homed` status and sets position to zero on success.

        Args:
            wait_for_completion: If True, waits for the homing process to complete.
                                 If False, initiates homing and returns.
            timeout: Maximum time in seconds to wait for homing completion if
                     `wait_for_completion` is True.

        Raises:
            HomingError: If the homing process fails or times out.
            CalibrationError: If attempting to home a non-calibrated axis.
            CommunicationError: On CAN communication issues or timeout waiting for completion signal.
            MotorError: For other motor-reported errors.
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
        Defines the motor's current physical position as the zero reference point.
        This typically also sets the homed status to True.

        Raises:
            MotorError: If the motor fails to set the current position as zero.
            CommunicationError: On CAN communication issues.
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
        Handles sending the command, creating a future for completion, and processing responses.
        The dynamic timeout is calculated if pulses_to_move and speed_param_for_calc are provided.

        Args:
            move_command_func: Callable that executes the low-level move command and returns initial status.
            command_const: The MKS CAN command constant for this move type.
            success_status: Expected status code from the motor upon successful completion.
            pulses_to_move: Absolute number of pulses for the move (for timeout calculation).
            speed_param_for_calc: MKS speed parameter used for the move (for timeout calculation).
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
            
    # ... (rest of Axis class, no changes to other methods for these specific recommendations) ...

    async def set_speed_user(
        self, speed_user: float, accel_user: Optional[float] = None 
    ) -> None:
        """
        Sets the motor to run continuously at a specified speed in user units (speed/velocity mode).

        Uses MKS command 0xF6. To stop, call with speed_user = 0 or use `stop_motor()`.

        Args:
            speed_user: The desired speed in user units per second.
                        Positive for one direction, negative for the other.
            accel_user: Desired acceleration in user units per second squared.
                        (Currently, this converts to a default MKS acceleration parameter).

        Raises:
            MotorError: If the command fails.
            CommunicationError: On CAN communication issues.
            KinematicsError: If kinematics conversion fails.
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

        This typically uses the MKS speed mode stop command (0xF6 with speed 0).
        If that fails, an emergency stop may be attempted.

        Args:
            deceleration_param: MKS deceleration parameter (0-255). If None,
                                the axis default acceleration parameter is used.
                                A value of 0 implies an immediate stop.

        Raises:
            MotorError: If the stop command fails.
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

        This should halt motor activity as quickly as possible.
        Any active move future will be cancelled.

        Raises:
            MotorError: If the emergency stop command fails.
            CommunicationError: On CAN communication issues.
        """
        logger.warning(f"Axis '{self.name}': Performing EMERGENCY STOP.")
        await self._low_level_api.emergency_stop(self.can_id)
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Motor emergency stopped")
        self._active_move_future = None # Clear the future as the move is aborted

    async def enable_motor(self) -> None:
        """
        Enables the motor's servo loop (MKS command 0xF3 with enable=1).

        If the motor is already enabled, this method logs and does nothing further.
        Updates the internal `_is_enabled` state.

        Raises:
            MotorError: If the enable command fails.
            CommunicationError: On CAN communication issues.
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
        Disables the motor's servo loop (MKS command 0xF3 with enable=0).

        If the motor is already disabled, this method logs and does nothing further.
        Updates the internal `_is_enabled` state. If a move was in progress,
        its future is cancelled.

        Raises:
            MotorError: If the disable command fails.
            CommunicationError: On CAN communication issues.
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

        This does not send a command to the motor; it reflects the last known state.
        Use `read_en_status()` to actively query the motor.

        Returns:
            True if the motor is believed to be enabled, False otherwise.
        """
        return self._is_enabled

    async def read_en_status(self) -> bool:
        """
        Actively reads and returns the current enable (EN pin) status from the motor.

        Updates the internal `_is_enabled` state.

        Returns:
            True if the motor is enabled, False otherwise.

        Raises:
            CommunicationError: On CAN communication issues.
            MotorError: For motor-reported errors.
        """
        self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
        return self._is_enabled

    def is_homed(self) -> bool:
        """
        Returns the cached homed state of the motor.

        This state is typically updated after a successful `home_axis()` or
        `set_current_position_as_zero()` operation.

        Returns:
            True if the motor is considered homed, False otherwise.
        """
        return self._is_homed

    def is_calibrated(self) -> bool:
        """
        Returns the cached calibration state of the motor's encoder.

        This state is updated after a `calibrate_encoder()` operation or
        implicitly assumed True on successful initialization if calibration is not requested.

        Returns:
            True if the motor's encoder is considered calibrated, False otherwise.
        """
        return self._is_calibrated

    async def get_current_position_steps(self) -> int:
        """
        Reads and returns the motor's current position in raw encoder steps.

        This actively queries the motor using command 0x31 (Read Encoder Addition).
        Updates the internal `_current_position_steps` state.

        Returns:
            The current motor position in encoder steps.

        Raises:
            CommunicationError: On CAN communication issues.
            MotorError: For motor-reported errors.
        """
        pos_steps = await self._low_level_api.read_encoder_value_addition(self.can_id)
        self._current_position_steps = pos_steps
        return pos_steps

    async def get_current_position_user(self) -> float:
        """
        Reads the motor's current position and converts it to user-defined units.

        This method first calls `get_current_position_steps()` to get the raw
        step count, then applies the axis's kinematics for conversion.

        Returns:
            The current motor position in user-defined units (e.g., mm, degrees).

        Raises:
            CommunicationError: On CAN communication issues.
            MotorError: For motor-reported errors.
            KinematicsError: If an error occurs during kinematic conversion.
        """
        steps = await self.get_current_position_steps()
        return self.kinematics.steps_to_user(steps)

    async def get_current_speed_rpm(self) -> int:
        """
        Reads and returns the motor's current speed in Revolutions Per Minute (RPM).

        This actively queries the motor using command 0x32.
        Updates the internal `_current_speed_rpm` state.

        Returns:
            The current motor speed in RPM.

        Raises:
            CommunicationError: On CAN communication issues.
            MotorError: For motor-reported errors.
        """
        rpm = await self._low_level_api.read_motor_speed_rpm(self.can_id)
        self._current_speed_rpm = rpm
        return rpm

    async def get_current_speed_user(self) -> float:
        """
        Reads the motor's current speed in RPM and converts it to user-defined speed units.

        This method queries the motor for its speed in RPM and then uses the
        `motor_speed_to_user_speed` method of the configured kinematics object
        for the conversion. The kinematics method typically expects an MKS speed
        parameter (0-3000), so this direct RPM to user_speed conversion via kinematics
        might be an approximation if the kinematics class doesn't explicitly handle RPM input.
        For simplicity and directness, this implementation now uses a more direct path if
        the kinematics object supports it, or falls back to an RPM-based calculation.

        Returns:
            The current motor speed in user-defined units per second (e.g., mm/s, deg/s).

        Raises:
            CommunicationError: On CAN communication issues.
            MotorError: For motor-reported errors.
            KinematicsError: If an error occurs during kinematic conversion.
        """
        rpm = await self.get_current_speed_rpm()
        
        # The kinematics.motor_speed_to_user_speed typically expects MKS speed param (0-3000).
        # Directly converting RPM to user speed requires reversing the logic in
        # kinematics.user_speed_to_motor_speed or having a dedicated RPM input method.
        # For a more direct conversion based on RPM:
        motor_revs_per_second = float(rpm) / 60.0
        output_revs_per_second = motor_revs_per_second / self.kinematics.gear_ratio

        # Check specific kinematics types for their properties
        if hasattr(self.kinematics, 'degrees_per_output_revolution'): # RotaryKinematics
            return output_revs_per_second * self.kinematics.degrees_per_output_revolution # type: ignore
        elif hasattr(self.kinematics, 'pitch'): # LinearKinematics
            return output_revs_per_second * self.kinematics.pitch # type: ignore
        elif hasattr(self.kinematics, 'arm_length') and hasattr(self.kinematics, '_motor_angle_deg_to_user'): # EccentricKinematics like
            # For non-linear, speed depends on current position (Jacobian)
            # This is a simplified approximation for EccentricKinematics assuming motion near center
            # or if its motor_speed_to_user_speed can internally handle an RPM-like input.
            # The existing eccentric_kinematics.motor_speed_to_user_speed IS based on MKS param.
            # So we should use the RPM to MKS param estimation, then call its method.
            # This is less direct. Let's keep the direct calculation for now.
            # output_angular_speed_rad_per_sec = output_revs_per_second * 2 * math.pi
            # return self.kinematics.arm_length * output_angular_speed_rad_per_sec # Very simplified for eccentric at center
            logger.warning(
                f"Axis '{self.name}': get_current_speed_user for {type(self.kinematics).__name__} from RPM is an approximation."
            )
            # Fallback to calling its motor_speed_to_user_speed with RPM as if it were param, which is not ideal
            # but better than a complex jacobian here. Or return the RPM converted to some generic units if known.
            # For now, stick to the direct calculation style for linear/rotary:
            if isinstance(self.kinematics, RotaryKinematics) or isinstance(self.kinematics, LinearKinematics) : # Check specific known types
                 # This path is already covered by hasattr checks above.
                 pass # Redundant but for clarity
            else: # For other types like Eccentric, the above direct calculation might be too simple
                logger.warning(f"User speed calculation from RPM for {type(self.kinematics)} is a broad approximation.")
                # As a very rough fallback for EccentricKinematics:
                if hasattr(self.kinematics, 'arm_length'):
                    output_angular_speed_rad_per_sec = output_revs_per_second * 2 * math.pi
                    return self.kinematics.arm_length * output_angular_speed_rad_per_sec # type: ignore

        logger.warning(f"User speed calculation from RPM not precisely defined for {type(self.kinematics)}.")
        return 0.0 # Fallback if no clear conversion path


    async def get_status_dict(self) -> Dict[str, Any]:
        """
        Retrieves a dictionary containing various status information about the axis.

        This includes name, CAN ID, timestamp, position (steps and user units),
        enabled/homed/calibrated states, motor status code and string,
        any error state, and whether a move is currently in progress.
        It actively queries core statuses like position and motor status code.

        Returns:
            A dictionary with axis status information.

        Raises:
            CommunicationError: If querying motor status or position fails.
            MotorError: For motor-reported errors during status queries.
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
        """Queries and returns the raw motor status code (from 0xF1)."""
        return await self._low_level_api.query_motor_status(self.can_id)

    def is_move_complete(self) -> bool:
        """
        Checks if the last commanded move for this axis has completed.

        This is based on the state of an internal future (`_active_move_future`)
        that tracks move completion.

        Returns:
            True if no move is active or the last move has completed (successfully or with error).
            False if a move is currently in progress.
        """
        if self._active_move_future:
            return self._active_move_future.done()
        return True # No active move, so trivially complete

    async def wait_for_move_completion(self, timeout: Optional[float] = None) -> None:
        """
        Asynchronously waits until the current move operation for this axis is complete
        or a timeout occurs.

        If no move is active, this method returns immediately.
        If the move future completes with an exception (e.g., MotorError, LimitError),
        that exception will be re-raised by this method.

        Args:
            timeout: Optional maximum time in seconds to wait for completion.
                     If None, waits indefinitely (or until the move naturally completes/fails).

        Raises:
            asyncio.TimeoutError: If the wait times out (distinct from CommunicationError for CAN timeout).
            MKSServoError (or subclasses): If the move itself failed and its future
                                          was set with an exception.
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
        Pings the motor to check for basic communication.

        This method attempts to read a simple status from the motor.
        If the read is successful within the timeout, it indicates
        the motor is responsive. The timeout for the low-level CAN call
        is typically handled by `const.CAN_TIMEOUT_SECONDS` or overridden
        if the low-level API supports it. This `timeout` param is for the
        overall ping operation if it involved multiple steps or a specific wait.

        Args:
            timeout: Optional timeout in seconds for the communication attempt.
                     If None, the default CAN timeout configured in the system will be used
                     for the underlying CAN read operation.

        Returns:
            True if the motor responds successfully, False otherwise.
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
        