# mks_servo_can/mks_servo_can_library/mks_servo_can/axis.py
"""
High-Level Axis Class for individual MKS Servo motor control.
"""
import asyncio
import time
import logging
from typing import Optional, Union, Dict, Any, Callable

from .can_interface import CANInterface
from .low_level_api import LowLevelAPI
from .kinematics import Kinematics, RotaryKinematics # Default kinematics
from .exceptions import (
    ParameterError, MotorError, CommunicationError, ConfigurationError,
    HomingError, CalibrationError, LimitError, StallError, MKSServoError
)
from . import constants as const

logger = logging.getLogger(__name__)

class Axis:
    """
    Represents a single motor axis, providing a high-level interface for control.
    """
    def __init__(self,
                 can_interface_manager: CANInterface, # Changed from can_interface
                 motor_can_id: int,
                 name: str = "default_axis",
                 motor_type: str = const.MOTOR_TYPE_SERVO42D, # For default current limits etc.
                 kinematics: Optional[Kinematics] = None,
                 default_speed_param: int = 500, # Default MKS speed parameter (0-3000)
                 default_accel_param: int = 100, # Default MKS acceleration parameter (0-255)
                 loop: Optional[asyncio.AbstractEventLoop] = None):
        """
        Initialize an Axis.

        Args:
            can_interface_manager: The CANInterface instance.
            motor_can_id: The CAN ID of this motor.
            name: A user-friendly name for this axis.
            motor_type: Type of motor (e.g., const.MOTOR_TYPE_SERVO42D).
            kinematics: A Kinematics object for unit conversions. If None, defaults to
                        RotaryKinematics with ENCODER_PULSES_PER_REVOLUTION.
            default_speed_param: Default speed parameter for MKS commands.
            default_accel_param: Default acceleration parameter for MKS commands.
            loop: The asyncio event loop.
        """
        if not (const.BROADCAST_ADDRESS < motor_can_id <= 0x7FF): # Axis must have a specific ID
            raise ParameterError(f"Invalid motor_can_id: {motor_can_id}. Must be 1-2047.")

        self.name = name
        self.can_id = motor_can_id
        self.motor_type = motor_type
        self._can_if = can_interface_manager
        self._low_level_api = LowLevelAPI(self._can_if)
        self._loop = loop if loop else asyncio.get_event_loop()

        if kinematics is None:
            logger.info(f"Axis '{name}': No kinematics provided, defaulting to RotaryKinematics "
                        f"with {const.ENCODER_PULSES_PER_REVOLUTION} steps/rev (encoder pulses).")
            self.kinematics = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
        else:
            self.kinematics = kinematics
        
        self.default_speed_user = 0 # User units, will be set by set_default_speed
        self.default_accel_user = 0 # User units/s^2 placeholder, accel is more complex

        # Internal MKS parameters
        self.default_speed_param = default_speed_param
        self.default_accel_param = default_accel_param


        # State variables
        self._current_position_steps: Optional[int] = None # In motor steps/encoder counts
        self._current_speed_rpm: Optional[int] = None # RPM
        self._is_enabled: bool = False # Assumed disabled initially
        self._is_homed: bool = False
        self._is_calibrated: bool = False # Assume needs calibration
        self._last_status_query_time: float = 0.0
        self._active_move_future: Optional[asyncio.Future] = None
        self._error_state: Optional[MotorError] = None

        # Add specific message handler for this axis's CAN ID for unsolicited messages
        # or completed move notifications if not handled by futures.
        # self._can_if.add_message_handler(self.can_id, self._handle_axis_message)
        # For now, specific responses are handled by futures in _send_command_and_get_response

    async def initialize(self, calibrate: bool = False, home: bool = False) -> None:
        """
        Initializes the axis: checks communication, optionally calibrates and homes.
        """
        logger.info(f"Initializing axis '{self.name}' (CAN ID: {self.can_id:03X})...")
        try:
            # Try a simple read command to verify communication
            await self.get_current_position_steps() # This will also update internal state
            logger.info(f"Axis '{self.name}': Communication test successful.")

            # Check current enabled state
            self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
            logger.info(f"Axis '{self.name}': Initial EN pin status: {'Enabled' if self._is_enabled else 'Disabled'}.")


            # Assume not calibrated unless explicitly told or status read
            # Some motors might retain calibration status. A read_parameter for calibration status would be good.
            # For now, if calibrate flag is true, perform it.
            if calibrate:
                await self.calibrate_encoder()
            
            # Check homing status? Or just home if requested.
            # A read_parameter for homing status would be good.
            if home:
                if not self._is_calibrated and calibrate: # Only home if calibration was successful
                    await self.home_axis()
                elif self._is_calibrated: # If already considered calibrated (e.g. from previous session)
                     await self.home_axis()
                else:
                    logger.warning(f"Axis '{self.name}': Skipping homing because calibration is required but not performed/successful.")


        except MKSServoError as e:
            self._error_state = e
            logger.error(f"Axis '{self.name}': Initialization failed: {e}")
            raise
        logger.info(f"Axis '{self.name}' initialization complete.")


    # --- Configuration Methods ---
    async def set_work_mode(self, mode_const: int) -> None:
        """Sets the working mode of the motor (e.g., const.MODE_SR_VFOC)."""
        if mode_const not in const.WORK_MODES:
            raise ParameterError(f"Invalid work mode constant: {mode_const}")
        logger.info(f"Axis '{self.name}': Setting work mode to {const.WORK_MODES.get(mode_const, 'Unknown')} ({mode_const}).")
        await self._low_level_api.set_work_mode(self.can_id, mode_const)
        # Verify? Some settings require a reboot. Work mode does not seem to.

    async def set_currents(self, working_current_ma: int, holding_current_percentage_code: Optional[int] = None) -> None:
        """Sets the working current (mA) and optionally the holding current percentage code."""
        # Add motor_type specific max current checks
        max_curr = const.MAX_CURRENT_SERVO57D if self.motor_type == const.MOTOR_TYPE_SERVO57D else const.MAX_CURRENT_SERVO42D
        if not (0 <= working_current_ma <= max_curr):
            raise ParameterError(f"Working current {working_current_ma}mA out of range for {self.motor_type} (0-{max_curr}mA).")
        
        logger.info(f"Axis '{self.name}': Setting working current to {working_current_ma}mA.")
        await self._low_level_api.set_working_current(self.can_id, working_current_ma)

        if holding_current_percentage_code is not None:
            if not (0x00 <= holding_current_percentage_code <= 0x08):
                 raise ParameterError("Holding current percentage code out of range (0-8).")
            logger.info(f"Axis '{self.name}': Setting holding current percentage code to {holding_current_percentage_code}.")
            await self._low_level_api.set_holding_current_percentage(self.can_id, holding_current_percentage_code)

    async def set_microstepping(self, microsteps: int, interpolation: Optional[bool] = None) -> None:
        """Sets the microstepping subdivision and optionally enables/disables interpolation."""
        if not (1 <= microsteps <= 255):
            raise ParameterError("Microsteps out of range (1-255).")
        logger.info(f"Axis '{self.name}': Setting microstepping to {microsteps}.")
        await self._low_level_api.set_subdivision(self.can_id, microsteps)
        if interpolation is not None:
            logger.info(f"Axis '{self.name}': Setting subdivision interpolation to {interpolation}.")
            await self._low_level_api.set_subdivision_interpolation(self.can_id, interpolation)

    async def configure_homing_parameters(self, home_trig_low: bool, home_dir_ccw: bool,
                                          home_speed_rpm: int, end_limit_enabled: bool,
                                          no_limit_home_mode: bool = False,
                                          nolimit_reverse_angle_pulses: Optional[int] = None,
                                          nolimit_homing_current_ma: Optional[int] = None) -> None:
        """Configures all homing related parameters."""
        logger.info(f"Axis '{self.name}': Configuring homing parameters.")
        await self._low_level_api.set_home_parameters(
            self.can_id, home_trig_low, home_dir_ccw, home_speed_rpm, end_limit_enabled, no_limit_home_mode
        )
        if no_limit_home_mode:
            if nolimit_reverse_angle_pulses is None or nolimit_homing_current_ma is None:
                raise ParameterError("For no-limit homing, reverse angle pulses and homing current must be provided.")
            await self._low_level_api.set_nolimit_home_parameters(
                self.can_id, nolimit_reverse_angle_pulses, nolimit_homing_current_ma
            )
        self._is_homed = False # Re-homing will be required

    # --- Core Methods ---
    async def calibrate_encoder(self) -> None:
        """Calibrates the motor's encoder."""
        if self._is_enabled:
            logger.warning(f"Axis '{self.name}': Motor is enabled. Disabling temporarily for calibration is often recommended but not enforced by this library.")
            # await self.disable_motor() # Optionally disable first
        
        logger.info(f"Axis '{self.name}': Starting encoder calibration...")
        try:
            await self._low_level_api.calibrate_encoder(self.can_id)
            self._is_calibrated = True
            self._error_state = None
            logger.info(f"Axis '{self.name}': Encoder calibration successful.")
        except CalibrationError as e:
            self._is_calibrated = False
            self._error_state = e
            logger.error(f"Axis '{self.name}': Encoder calibration failed: {e}")
            raise
        finally:
            # Re-enable if we disabled it? Or leave it to user.
            pass

    async def home_axis(self, wait_for_completion: bool = True, timeout: float = 30.0) -> None:
        """Homes the axis."""
        if not self._is_calibrated:
            logger.warning(f"Axis '{self.name}': Attempting to home non-calibrated axis. Calibration is usually required first.")
            # raise ConfigurationError("Axis must be calibrated before homing.")

        if not self.is_enabled(): # Motor must be enabled to home
            await self.enable_motor()

        logger.info(f"Axis '{self.name}': Starting homing sequence...")
        try:
            # The go_home command (0x91) might return status 1 (start)
            # and then later status 2 (success) or 0 (fail) if CanRSP/Active is set.
            # For now, this low-level call returns the first response.
            initial_status = await self._low_level_api.go_home(self.can_id)

            if initial_status == const.HOME_START: # 0x01
                if wait_for_completion:
                    logger.info(f"Axis '{self.name}': Homing started, waiting for completion...")
                    # Poll for completion status
                    start_time = time.monotonic()
                    while time.monotonic() - start_time < timeout:
                        await asyncio.sleep(0.2) # Polling interval
                        current_motor_status = await self._low_level_api.query_motor_status(self.can_id)
                        # query_motor_status returns 5 for homing.
                        # We need to check if it's NO LONGER 5, and then check if home was successful.
                        # The go_home command itself should eventually report success/fail with a specific message
                        # if active mode is on. This polling is a fallback.
                        # A better way: listen for the actual 0x91 response with status 2.
                        # For now, we assume if it's not 'homing' (5) anymore, it's done.
                        if current_motor_status != const.MOTOR_STATUS_HOMING:
                            # How to confirm success here? go_home might send a final status message.
                            # If CanRSP is enabled, the 0x91 command should eventually yield status 2 or 0.
                            # This requires a more robust message handling for specific command completion.
                            # For now, if it stops homing, we assume it's done, and hope for the best.
                            # A more robust way: if active=1, the original 0x91 command should yield multiple responses.
                            # Our current _send_command_and_get_response only gets the *first* one.
                            # This needs architecture enhancement for multi-response commands.
                            # Temporary hack: assume if initial status was START, and it's no longer HOMING, it was okay.
                            # This is not robust.
                            logger.info(f"Axis '{self.name}': Homing process appears complete (no longer in homing state).")
                            self._is_homed = True # Assume success if it started and finished.
                            self._error_state = None
                            # Update position after homing - typically it's zero
                            self._current_position_steps = 0
                            return
                    raise HomingError(f"Axis '{self.name}': Timeout waiting for homing completion.", can_id=self.can_id)
                else: # Not waiting for completion
                    self._is_homed = False # Status is uncertain
                    logger.info(f"Axis '{self.name}': Homing initiated, not waiting for completion.")
                    return
            elif initial_status == const.HOME_SUCCESS: # 0x02 (if it completes immediately)
                self._is_homed = True
                self._error_state = None
                self._current_position_steps = 0
                logger.info(f"Axis '{self.name}': Homing successful (immediate).")
                return
            else: # Includes HOME_FAIL (0x00)
                raise HomingError(f"Axis '{self.name}': Homing command failed to start or reported failure immediately. Status: {initial_status}", error_code=initial_status, can_id=self.can_id)

        except HomingError as e:
            self._is_homed = False
            self._error_state = e
            logger.error(f"Axis '{self.name}': Homing failed: {e}")
            raise

    async def set_current_position_as_zero(self) -> None:
        """Sets the motor's current position as the zero point."""
        logger.info(f"Axis '{self.name}': Setting current position as zero.")
        await self._low_level_api.set_current_axis_to_zero(self.can_id)
        self._current_position_steps = 0
        self._is_homed = True # Usually, setting zero implies a known (homed) state.

    async def _execute_move(self, move_command_func, command_const, success_status=const.POS_RUN_COMPLETE, timeout_factor=1.2) -> None:
        """Helper to execute a move and wait for its completion."""
        if self._active_move_future and not self._active_move_future.done():
            raise MKSServoError(f"Axis '{self.name}' is already moving.")

        if not self.is_enabled():
            await self.enable_motor()

        self._active_move_future = self._loop.create_future()
        estimated_duration = timeout_factor # Default, should be improved
        # TODO: Estimate move duration based on distance and speed for a better timeout.
        # For now, use a fixed multiplier on default CAN timeout or a larger fixed value.
        move_timeout = const.CAN_TIMEOUT_SECONDS * 5 # Example: 5s for a move, adjust as needed

        try:
            # The low-level command returns the *initial* status (e.g., 0x01 = starting)
            initial_status = await move_command_func()

            if initial_status == const.POS_RUN_STARTING:
                logger.debug(f"Axis '{self.name}': Move ({command_const:02X}) started. Waiting for completion...")
                # Now, we need to wait for the actual completion message.
                # This assumes CanRSP and Active flags are set for the motor to send completion.
                # The can_interface.send_and_wait_for_response is for the *initial* ack.
                # We need a new future for the *completion* of this specific command.

                # A better approach:
                # 1. LowLevelAPI sends command.
                # 2. It gets initial ACK (e.g., "starting").
                # 3. If initial ACK is "starting", Axis layer now waits for a *second* message from motor
                #    with same command code but status "complete" or "limit_stop" or "error".
                # This requires the CANInterface to allow waiting for a message matching criteria
                # *after* an initial exchange, or Axis to register a temporary handler.

                # Simplification for now: If initial status is "starting", poll query_motor_status
                # This is NOT ideal as it adds CAN traffic and delay.
                # This part needs refinement based on how CanRSP Active=1 truly behaves for multi-stage responses.
                # If the _low_level_api._run_motor_command can be adapted to return a future for the *final* status,
                # that would be cleaner.
                
                # Let's assume for now that if active=1, the motor WILL send a subsequent frame
                # with the same command code and the final status.
                # We need a way to wait for THAT frame.
                # This is where `can_interface.create_response_future` for the *completion* is needed.

                completion_future = self._can_if.create_response_future(self.can_id, command_const)
                final_response_msg = await asyncio.wait_for(completion_future, timeout=move_timeout)
                
                final_status = final_response_msg.data[1]
                if final_status == success_status: # e.g., POS_RUN_COMPLETE
                    logger.info(f"Axis '{self.name}': Move ({command_const:02X}) completed successfully.")
                    self._active_move_future.set_result(True)
                elif final_status == const.POS_RUN_END_LIMIT_STOPPED:
                    msg = f"Axis '{self.name}': Move ({command_const:02X}) stopped by end limit."
                    logger.warning(msg)
                    self._active_move_future.set_exception(LimitError(msg, error_code=final_status, can_id=self.can_id))
                else: # Any other status is an error for this context
                    msg = f"Axis '{self.name}': Move ({command_const:02X}) failed or completed with unexpected status {final_status}."
                    logger.error(msg)
                    self._active_move_future.set_exception(MotorError(msg, error_code=final_status, can_id=self.can_id))

            elif initial_status == success_status: # Completed in the first response
                logger.info(f"Axis '{self.name}': Move ({command_const:02X}) completed successfully (immediate).")
                self._active_move_future.set_result(True)
            elif initial_status == const.POS_RUN_END_LIMIT_STOPPED:
                msg = f"Axis '{self.name}': Move ({command_const:02X}) stopped by end limit (immediate)."
                logger.warning(msg)
                self._active_move_future.set_exception(LimitError(msg, error_code=initial_status, can_id=self.can_id))
            else: # POS_RUN_FAIL or other
                msg = f"Axis '{self.name}': Move ({command_const:02X}) command failed to start properly. Initial status: {initial_status}"
                logger.error(msg)
                self._active_move_future.set_exception(MotorError(msg, error_code=initial_status, can_id=self.can_id))
            
            await self.get_current_position_steps() # Update position after move

        except asyncio.TimeoutError:
            msg = f"Axis '{self.name}': Timeout waiting for move ({command_const:02X}) completion."
            logger.error(msg)
            if not self._active_move_future.done():
                self._active_move_future.set_exception(CommunicationError(msg))
            # Consider sending a stop command if a move times out
            # await self.stop_motor()
        except MKSServoError as e:
            logger.error(f"Axis '{self.name}': Error during move ({command_const:02X}): {e}")
            if not self._active_move_future.done():
                self._active_move_future.set_exception(e)
        finally:
            if self._active_move_future.done(): # Check if it was set by success/specific error
                 pass # Don't overwrite
            else: # Should not happen if logic above is correct
                 self._active_move_future.set_exception(MKSServoError(f"Move future for {command_const:02X} ended without explicit result/error."))
            # self._active_move_future = None # Clear after handling, or allow user to await it

    async def move_to_position_abs_pulses(self, target_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True):
        """Moves the motor to an absolute position specified in raw encoder pulses (using 0xFE)."""
        sp = speed_param if speed_param is not None else self.default_speed_param
        ac = accel_param if accel_param is not None else self.default_accel_param
        logger.info(f"Axis '{self.name}': Moving to absolute pulse position {target_pulses} (SpeedP: {sp}, AccelP: {ac}).")
        
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(self.can_id, sp, ac, target_pulses)
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES)
        if wait and self._active_move_future: # Ensure future exists
            await self._active_move_future

    async def move_to_position_abs_user(self, target_pos_user: float, speed_user: Optional[float] = None, wait: bool = True):
        """Moves the motor to an absolute position specified in user units."""
        target_steps = self.kinematics.user_to_steps(target_pos_user)
        
        # Convert user speed to MKS speed parameter
        if speed_user is not None:
            mks_speed_param = self.kinematics.user_speed_to_motor_speed(speed_user)
        else:
            mks_speed_param = self.default_speed_param
        
        # Acceleration is trickier, use default MKS accel param for now
        mks_accel_param = self.default_accel_param

        logger.info(f"Axis '{self.name}': Moving to user position {target_pos_user} ({self.kinematics.units}) "
                    f"-> {target_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param}).")

        # Using CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES (0xFE) as it takes absolute pulse target
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(
            self.can_id, mks_speed_param, mks_accel_param, target_steps
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES)
        if wait and self._active_move_future:
            await self._active_move_future


    async def move_relative_pulses(self, relative_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True):
        """Moves the motor by a relative number of raw encoder pulses (using 0xFD)."""
        sp = speed_param if speed_param is not None else self.default_speed_param
        ac = accel_param if accel_param is not None else self.default_accel_param
        
        direction_ccw = relative_pulses > 0
        num_pulses = abs(relative_pulses)

        logger.info(f"Axis '{self.name}': Moving relative by {relative_pulses} pulses (SpeedP: {sp}, AccelP: {ac}).")
        
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, sp, ac, num_pulses
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES)
        if wait and self._active_move_future:
            await self._active_move_future

    async def move_relative_user(self, relative_dist_user: float, speed_user: Optional[float] = None, wait: bool = True):
        """Moves the motor by a relative distance specified in user units."""
        relative_steps = self.kinematics.user_to_steps(relative_dist_user)

        if speed_user is not None:
            mks_speed_param = self.kinematics.user_speed_to_motor_speed(speed_user)
        else:
            mks_speed_param = self.default_speed_param
        mks_accel_param = self.default_accel_param # Use default MKS accel param

        direction_ccw = relative_steps > 0 # Positive steps for CCW if kinematics define it so.
                                         # Or, more simply, if user_dist is positive, map to "forward"
                                         # This depends on how kinematics maps positive user values to steps.
                                         # And how 0xFD direction bit (CCW/CW) aligns with "positive steps".
                                         # Assuming positive steps for 0xFD maps to CCW if dir bit is 1.
                                         # Let's define: positive user distance means CCW.
        num_steps = abs(relative_steps)

        logger.info(f"Axis '{self.name}': Moving relative by {relative_dist_user} {self.kinematics.units} "
                    f"-> {relative_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param}).")
        
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, mks_speed_param, mks_accel_param, num_steps
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES)
        if wait and self._active_move_future:
            await self._active_move_future


    async def set_speed_user(self, speed_user: float, accel_user: Optional[float] = None) -> None:
        """
        Sets the motor to run at a continuous speed in user units/sec (using 0xF6).
        If speed_user is 0, it stops the motor with specified deceleration.
        """
        mks_speed_param = self.kinematics.user_speed_to_motor_speed(speed_user)
        # Acceleration is complex for 0xF6 as it's a direct MKS param.
        # For now, use default MKS accel param if not converting from user accel.
        mks_accel_param = self.default_accel_param
        # TODO: Convert accel_user to mks_accel_param if a model exists.

        direction_ccw = speed_user > 0 # Assuming positive user speed means CCW for 0xF6.

        if speed_user == 0:
            logger.info(f"Axis '{self.name}': Stopping continuous speed mode (AccelP: {mks_accel_param}).")
            await self._low_level_api.stop_speed_mode(self.can_id, mks_accel_param)
        else:
            logger.info(f"Axis '{self.name}': Setting continuous speed to {speed_user} {self.kinematics.units}/s "
                        f"(Dir: {'CCW' if direction_ccw else 'CW'}, SpeedP: {mks_speed_param}, AccelP: {mks_accel_param}).")
            await self._low_level_api.run_speed_mode(self.can_id, direction_ccw, mks_speed_param, mks_accel_param)


    async def stop_motor(self, deceleration_param: Optional[int] = None) -> None:
        """Stops any ongoing motion, using specified MKS deceleration parameter."""
        # This needs to choose the correct stop command based on current mode.
        # Simplest: use 0xF6 (stop_speed_mode) with a deceleration.
        # Or use 0xFD (stop_position_mode) if it's more appropriate.
        # Or an emergency stop if needed.
        # For a general stop, stopping speed mode is a reasonable default.
        decel = deceleration_param if deceleration_param is not None else self.default_accel_param
        logger.info(f"Axis '{self.name}': Commanding stop (decel_param: {decel}).")
        try:
            # Attempt a graceful stop first. This assumes it was in a speed or position mode.
            # The specific stop commands (F6, FD, FE, F4, F5 with speed=0) are better.
            # This is a generic stop.
            await self._low_level_api.stop_speed_mode(self.can_id, decel)
            if self._active_move_future and not self._active_move_future.done():
                self._active_move_future.cancel("Motor stopped by explicit command")
        except MKSServoError as e:
            logger.warning(f"Axis '{self.name}': Graceful stop command failed ({e}), attempting emergency stop.")
            await self.emergency_stop() # Fallback
        self._active_move_future = None # Clear any active move

    async def emergency_stop(self) -> None:
        """Performs an emergency stop."""
        logger.warning(f"Axis '{self.name}': Performing EMERGENCY STOP.")
        await self._low_level_api.emergency_stop(self.can_id)
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Motor emergency stopped")
        self._active_move_future = None

    async def enable_motor(self) -> None:
        """Enables the motor driver."""
        if not self._is_enabled:
            logger.info(f"Axis '{self.name}': Enabling motor.")
            await self._low_level_api.enable_motor(self.can_id, True)
            self._is_enabled = True
        else:
            logger.info(f"Axis '{self.name}': Motor already enabled.")

    async def disable_motor(self) -> None:
        """Disables the motor driver."""
        if self._is_enabled:
            logger.info(f"Axis '{self.name}': Disabling motor.")
            await self._low_level_api.enable_motor(self.can_id, False)
            self._is_enabled = False
        else:
            logger.info(f"Axis '{self.name}': Motor already disabled.")

    def is_enabled(self) -> bool:
        """Returns the cached enabled state of the motor."""
        return self._is_enabled # For quick check, or query if needed: await self.read_en_status()

    async def read_en_status(self) -> bool:
        """Reads and updates the enabled state from the motor."""
        self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
        return self._is_enabled

    def is_homed(self) -> bool:
        """Returns the cached homed state."""
        return self._is_homed

    def is_calibrated(self) -> bool:
        """Returns the cached calibrated state."""
        return self._is_calibrated

    # --- Status & Feedback ---
    async def get_current_position_steps(self) -> int:
        """Reads and returns the current motor position in raw encoder steps/pulses."""
        # Using 0x31 (addition) is generally preferred for absolute position.
        pos_steps = await self._low_level_api.read_encoder_value_addition(self.can_id)
        self._current_position_steps = pos_steps
        return pos_steps

    async def get_current_position_user(self) -> float:
        """Reads and returns the current motor position in user units."""
        steps = await self.get_current_position_steps()
        return self.kinematics.steps_to_user(steps)

    async def get_current_speed_rpm(self) -> int:
        """Reads and returns the current motor speed in RPM."""
        rpm = await self._low_level_api.read_motor_speed_rpm(self.can_id)
        self._current_speed_rpm = rpm
        return rpm

    async def get_current_speed_user(self) -> float:
        """Reads and returns the current motor speed in user units per second."""
        # This requires converting RPM (from 0x32) to user speed.
        # The MKS speed parameter (0-3000) is different from the reported RPM (0x32).
        rpm = await self.get_current_speed_rpm()
        
        # Convert RPM to user speed (e.g., mm/s or deg/s)
        # motor_rpm -> motor_revs_per_sec -> output_revs_per_sec -> user_speed
        motor_revs_per_sec = rpm / 60.0
        output_revs_per_sec = motor_revs_per_sec / self.kinematics.gear_ratio
        
        if self.kinematics.__class__.__name__ == "LinearKinematics":
            user_speed = output_revs_per_sec * self.kinematics.pitch
        elif self.kinematics.__class__.__name__ == "RotaryKinematics":
            user_speed = output_revs_per_sec * self.kinematics.degrees_per_output_revolution
        elif self.kinematics.__class__.__name__ == "EccentricKinematics":
            # For eccentric, speed depends on current angle. This is instantaneous speed.
            # (d_user / dt) = (d_user / d_theta_out) * (d_theta_out / dt)
            # We have d_theta_out / dt = output_angular_speed_rad_per_sec
            # We need d_user / d_theta_out.
            # From user_to_steps -> _user_to_motor_angle_deg, and _motor_angle_deg_to_user
            # This is complex to derive generally here.
            # For EccentricKinematics's specific model where user = L*sin(theta_out):
            # d_user/d_theta_out = L*cos(theta_out)
            # We need current theta_out.
            current_steps = await self.get_current_position_steps() # Use cached if available
            # steps_to_user gives user_pos. We need theta_out from steps.
            theta_out_deg = current_steps / (self.kinematics.effective_steps_per_output_revolution / 360.0)
            theta_out_rad = math.radians(theta_out_deg)
            
            output_angular_speed_rad_per_sec = math.radians(output_revs_per_sec * 360.0)
            
            user_speed = self.kinematics.arm_length * math.cos(theta_out_rad) * output_angular_speed_rad_per_sec
        else:
            logger.warning(f"User speed calculation not fully implemented for kinematics type {self.kinematics.__class__.__name__}")
            user_speed = 0.0 # Fallback or raise error
            
        return user_speed


    async def get_motor_status_code(self) -> int:
        """Queries and returns the raw motor status code (from 0xF1)."""
        return await self._low_level_api.query_motor_status(self.can_id)
        
    async def get_status_dict(self) -> Dict[str, Any]:
        """Returns a comprehensive dictionary of the axis's current status."""
        # Could be optimized to read multiple values if MKS supports bundled status reads
        pos_steps = await self.get_current_position_steps()
        pos_user = self.kinematics.steps_to_user(pos_steps)
        # speed_rpm = await self.get_current_speed_rpm() # Optional, can be slow
        # speed_user = self.motor_speed_to_user_speed(speed_rpm) # MKS param -> user speed
        raw_motor_status = await self.get_motor_status_code()
        is_en = await self.read_en_status() # Query actual EN state

        return {
            "name": self.name,
            "can_id": self.can_id,
            "timestamp": time.time(),
            "position_steps": pos_steps,
            "position_user": pos_user,
            "position_units": self.kinematics.units if hasattr(self.kinematics, 'units') else 'degrees',
            # "speed_rpm": speed_rpm,
            # "speed_user": speed_user, # Implement this conversion if useful
            "is_enabled": is_en,
            "is_homed": self._is_homed,
            "is_calibrated": self._is_calibrated,
            "motor_status_code": raw_motor_status,
            "motor_status_str": const.MOTOR_STATUS_MAP.get(raw_motor_status, "Unknown") \
                                if hasattr(const, 'MOTOR_STATUS_MAP') else "N/A", # Add MOTOR_STATUS_MAP to consts
            "error_state": str(self._error_state) if self._error_state else None,
            "active_move_in_progress": self._active_move_future is not None and not self._active_move_future.done()
        }

    async def get_io_status(self) -> Dict[str, int]:
        """Reads and returns the status of IO ports."""
        return await self._low_level_api.read_io_status(self.can_id)

    async def set_io_output(self, out1_value: Optional[int] = None, out2_value: Optional[int] = None) -> None:
        """Sets the state of output IO ports (for motors that support it, e.g., 57D)."""
        # Check if motor type supports this (e.g. 57D has OUT_1, OUT_2)
        if self.motor_type not in [const.MOTOR_TYPE_SERVO57D]: # Add other types if they get outputs
            raise ConfigurationError(f"Motor type {self.motor_type} may not support direct IO output control via CMD 0x36.")
        await self._low_level_api.write_io_port(self.can_id, out1_value=out1_value, out2_value=out2_value)

    async def read_parameter(self, parameter_command_code: int) -> bytes:
        """Reads a raw system parameter using its command code."""
        # This is a low-level passthrough. Interpretation is up to the caller.
        _, param_data = await self._low_level_api.read_system_parameter(self.can_id, parameter_command_code)
        return param_data

    async def write_parameter_raw(self, command_code:int, data_bytes: list[int]):
        """
        Writes a raw parameter to the motor. Use with extreme caution.
        This sends a command expecting a simple success/fail status in response.
        """
        # This assumes the command structure: ID, CMD, DATA..., CRC -> Response: ID, CMD, STATUS, CRC
        logger.warning(f"Axis '{self.name}': Writing raw parameter CMD {command_code:02X} with data {data_bytes}. Use with caution.")
        response = await self._low_level_api._send_command_and_get_response(
            self.can_id, command_code, data=data_bytes, expected_dlc=3 # Assuming simple status response
        )
        if response.data[1] != const.STATUS_SUCCESS:
            raise MotorError(f"Raw write parameter CMD {command_code:02X} failed.", error_code=response.data[1], can_id=self.can_id)
        logger.info(f"Axis '{self.name}': Raw parameter CMD {command_code:02X} written successfully.")

    def is_move_complete(self) -> bool:
        """Checks if the last initiated move is complete."""
        if self._active_move_future:
            return self._active_move_future.done()
        return True # No active move

    async def wait_for_move_completion(self, timeout: Optional[float] = None) -> None:
        """Waits for the current move to complete."""
        if self._active_move_future and not self._active_move_future.done():
            try:
                await asyncio.wait_for(self._active_move_future, timeout=timeout)
            except asyncio.TimeoutError:
                raise CommunicationError(f"Axis '{self.name}': Timeout waiting for move completion.")
            except MKSServoError: # Catch errors set on the future by _execute_move
                raise # Re-raise the original move error
        # If future is already done, this will return immediately or raise stored exception.