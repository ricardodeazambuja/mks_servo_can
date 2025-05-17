# mks_servo_can/mks_servo_can_library/mks_servo_can/axis.py
"""
High-Level Axis Class for individual MKS Servo motor control.
"""
from typing import Any, Dict, Optional

import asyncio
import logging
import math # Added for EccentricKinematics speed calculation if used
import time

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
        if not (
            const.BROADCAST_ADDRESS < motor_can_id <= 0x7FF
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
            self.kinematics = RotaryKinematics(
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
        self._is_calibrated: bool = False
        self._active_move_future: Optional[asyncio.Future] = None
        self._error_state: Optional[MKSServoError] = None


    async def initialize(
        self, calibrate: bool = False, home: bool = False
    ) -> None:
        logger.info(
            f"Axis '{self.name}' (CAN ID: {self.can_id:03X}): Initializing..."
        )
        try:
            await self.get_current_position_steps()
            logger.info(f"Axis '{self.name}': Communication test successful.")
            self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
            logger.info(
                f"Axis '{self.name}': Initial EN pin status: {'Enabled' if self._is_enabled else 'Disabled'}."
            )
            if calibrate:
                await self.calibrate_encoder()
            if home:
                if self._is_calibrated or not calibrate: # Home if calibrated or if not attempting calibration
                    await self.home_axis()
                else:
                    logger.warning(
                        f"Axis '{self.name}': Skipping homing because calibration was requested but failed or not performed."
                    )
        except MKSServoError as e:
            self._error_state = e
            logger.error(f"Axis '{self.name}': Initialization failed: {e}")
            raise
        logger.info(f"Axis '{self.name}' initialization complete.")

    async def set_work_mode(self, mode_const: int) -> None:
        if mode_const not in const.WORK_MODES:
            raise ParameterError(f"Invalid work mode constant: {mode_const}")
        logger.info(
            f"Axis '{self.name}': Setting work mode to {const.WORK_MODES.get(mode_const, 'Unknown')} ({mode_const})."
        )
        await self._low_level_api.set_work_mode(self.can_id, mode_const)

    async def calibrate_encoder(self) -> None:
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

    async def home_axis(
        self, wait_for_completion: bool = True, timeout: float = 30.0
    ) -> None:
        if not self._is_calibrated:
            logger.warning(
                f"Axis '{self.name}': Attempting to home non-calibrated axis."
            )
        if not self.is_enabled():
            await self.enable_motor()

        logger.info(f"Axis '{self.name}': Starting homing sequence...")
        try:
            initial_status = await self._low_level_api.go_home(self.can_id)
            if initial_status == const.HOME_START:
                if wait_for_completion:
                    logger.info(
                        f"Axis '{self.name}': Homing started, waiting for completion..."
                    )
                    # This relies on future enhancements for multi-stage response handling.
                    # For now, we assume if active response is on, a follow-up 0x91 with status 2 or 0 will come.
                    # This part needs a robust way to await the specific completion message for 0x91.
                    # Using a temporary future for the completion of the 0x91 command.
                    completion_future = self._can_if.create_response_future(self.can_id, const.CMD_GO_HOME)
                    final_response_msg = await asyncio.wait_for(completion_future, timeout=timeout)
                    final_status = final_response_msg.data[1]

                    if final_status == const.HOME_SUCCESS:
                        self._is_homed = True
                        self._current_position_steps = 0 # Typically homing sets zero
                        logger.info(f"Axis '{self.name}': Homing successful via async completion.")
                        return
                    else: # HOME_FAIL or unexpected
                        raise HomingError(
                            f"Axis '{self.name}': Homing failed or unexpected async status {final_status}.",
                            error_code=final_status, can_id=self.can_id
                        )
                else:
                    self._is_homed = False
                    logger.info(f"Axis '{self.name}': Homing initiated, not waiting for completion.")
            elif initial_status == const.HOME_SUCCESS:
                self._is_homed = True
                self._current_position_steps = 0
                logger.info(f"Axis '{self.name}': Homing successful (immediate).")
            else: # HOME_FAIL or other unexpected initial status
                raise HomingError(
                    f"Axis '{self.name}': Homing command failed to start or reported immediate failure. Status: {initial_status}",
                    error_code=initial_status, can_id=self.can_id
                )
        except HomingError as e:
            self._is_homed = False
            self._error_state = e
            logger.error(f"Axis '{self.name}': Homing failed: {e}")
            raise
        except asyncio.TimeoutError:
            self._is_homed = False
            msg = f"Axis '{self.name}': Timeout waiting for homing completion signal."
            logger.error(msg)
            raise HomingError(msg, can_id=self.can_id) from None


    async def set_current_position_as_zero(self) -> None:
        logger.info(f"Axis '{self.name}': Setting current position as zero.")
        await self._low_level_api.set_current_axis_to_zero(self.can_id)
        self._current_position_steps = 0
        self._is_homed = True

    async def _execute_move(
        self,
        move_command_func, # type: ignore
        command_const: int,
        success_status=const.POS_RUN_COMPLETE,
        # timeout_factor=1.2, # Not currently used effectively
    ) -> None:
        if self._active_move_future and not self._active_move_future.done():
            # Cancel previous move if a new one is initiated for the same axis
            logger.warning(f"Axis '{self.name}': Active move future exists. Cancelling previous move.")
            self._active_move_future.cancel("New move initiated")
            # Allow cancellation to process
            await asyncio.sleep(0)


        if not self.is_enabled():
            await self.enable_motor()

        self._active_move_future = self._loop.create_future()
        
        # ADDED LOGGING
        logger.info(f"Axis '{self.name}'._execute_move: Called for CMD={command_const:02X}. Current active_move_future: {self._active_move_future}")

        move_timeout = const.CAN_TIMEOUT_SECONDS * 10  # Extended default timeout for moves

        try:
            # ADDED LOGGING
            logger.info(f"Axis '{self.name}'._execute_move: Awaiting move_command_func() for CMD={command_const:02X}...")
            initial_status = await move_command_func()
            # ADDED LOGGING
            logger.info(f"Axis '{self.name}'._execute_move: move_command_func() for CMD={command_const:02X} returned initial_status={initial_status:02X}")


            if initial_status == const.POS_RUN_STARTING: # 0x01
                logger.debug(
                    f"Axis '{self.name}': Move (CMD={command_const:02X}) reported STARTING. Waiting for completion signal..."
                )
                completion_future = self._can_if.create_response_future(
                    self.can_id, command_const # Expecting same command code in completion
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
                else:
                    msg = f"Axis '{self.name}': Move (CMD={command_const:02X}) failed or completed with unexpected async status {final_status}."
                    logger.error(msg)
                    if not self._active_move_future.done(): self._active_move_future.set_exception(
                        MotorError(msg, error_code=final_status, can_id=self.can_id)
                    )

            elif initial_status == success_status: # e.g. POS_RUN_COMPLETE (0x02)
                logger.info(
                    f"Axis '{self.name}': Move (CMD={command_const:02X}) reported COMPLETE in initial response."
                )
                if not self._active_move_future.done(): self._active_move_future.set_result(True)
            elif initial_status == const.POS_RUN_END_LIMIT_STOPPED: # 0x03
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
            
            # Always update position after a move attempt, successful or not
            await self.get_current_position_steps()

        except asyncio.TimeoutError:
            msg = f"Axis '{self.name}': Timeout waiting for move (CMD={command_const:02X}) response/completion."
            logger.error(msg)
            if not self._active_move_future.done():
                self._active_move_future.set_exception(CommunicationError(msg, can_id=self.can_id))
        except MKSServoError as e: # Catch errors from move_command_func() or CAN interface
            logger.error(
                f"Axis '{self.name}': MKSServoError during move (CMD={command_const:02X}): {e}"
            )
            if not self._active_move_future.done():
                self._active_move_future.set_exception(e)
        except Exception as e: # Catch any other unexpected error
            logger.error(
                f"Axis '{self.name}': Unexpected Exception during move (CMD={command_const:02X}): {e}", exc_info=True
            )
            if not self._active_move_future.done():
                self._active_move_future.set_exception(MKSServoError(f"Unexpected move error: {e}", can_id=self.can_id))
        finally:
            # Ensure the future is always resolved to prevent deadlocks if it wasn't set in try/except
            if not self._active_move_future.done():
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
        logger.info(
            f"Axis '{self.name}': Moving to absolute pulse position {target_pulses} (SpeedP: {sp}, AccelP: {ac})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(
            self.can_id, sp, ac, target_pulses
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES)
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
        mks_accel_param = self.default_accel_param
        logger.info(
            f"Axis '{self.name}': Moving to user position {target_pos_user} ({getattr(self.kinematics, 'units', 'units')}) "
            f"-> {target_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_absolute_pulses(
            self.can_id, mks_speed_param, mks_accel_param, target_steps
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES)
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
        direction_ccw = relative_pulses > 0
        num_pulses = abs(relative_pulses)
        logger.info(
            f"Axis '{self.name}': Moving relative by {relative_pulses} pulses (SpeedP: {sp}, AccelP: {ac})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, sp, ac, num_pulses
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES)
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
        mks_accel_param = self.default_accel_param
        direction_ccw = relative_steps > 0
        num_steps = abs(relative_steps)
        logger.info(
            f"Axis '{self.name}': Moving relative by {relative_dist_user} {getattr(self.kinematics, 'units', 'units')} "
            f"-> {relative_steps} pulses (SpeedP: {mks_speed_param}, AccelP: {mks_accel_param})."
        )
        cmd_func = lambda: self._low_level_api.run_position_mode_relative_pulses(
            self.can_id, direction_ccw, mks_speed_param, mks_accel_param, num_steps
        )
        await self._execute_move(cmd_func, const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES)
        if wait and self._active_move_future:
            await self._active_move_future

    async def set_speed_user(
        self, speed_user: float, accel_user: Optional[float] = None # accel_user not yet implemented for conversion
    ) -> None:
        mks_speed_param = self.kinematics.user_speed_to_motor_speed(abs(speed_user))
        mks_accel_param = self.default_accel_param # TODO: Convert accel_user
        direction_ccw = speed_user >= 0

        if abs(speed_user) < 1e-3: # Effectively zero speed
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
        # For speed mode, there's no specific "completion" future beyond the initial command ACK.
        # Active move future should be cleared if it was for a positional move.
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Speed mode initiated, cancelling positional move future.")
        self._active_move_future = None


    async def stop_motor(self, deceleration_param: Optional[int] = None) -> None:
        decel = deceleration_param if deceleration_param is not None else self.default_accel_param
        logger.info(f"Axis '{self.name}': Commanding stop (decel_param: {decel}).")
        try:
            await self._low_level_api.stop_speed_mode(self.can_id, decel) # Generic stop using speed mode stop
            if self._active_move_future and not self._active_move_future.done():
                self._active_move_future.cancel("Motor stopped by explicit command")
        except MKSServoError as e:
            logger.warning(
                f"Axis '{self.name}': Graceful stop command failed ({e}), attempting emergency stop."
            )
            await self.emergency_stop()
        finally: # Ensure future is cleared on stop
            if self._active_move_future and not self._active_move_future.done():
                 self._active_move_future.cancel("Motor stop initiated")
            self._active_move_future = None


    async def emergency_stop(self) -> None:
        logger.warning(f"Axis '{self.name}': Performing EMERGENCY STOP.")
        await self._low_level_api.emergency_stop(self.can_id)
        if self._active_move_future and not self._active_move_future.done():
            self._active_move_future.cancel("Motor emergency stopped")
        self._active_move_future = None

    async def enable_motor(self) -> None:
        if not self._is_enabled:
            logger.info(f"Axis '{self.name}': Enabling motor.")
            await self._low_level_api.enable_motor(self.can_id, True)
            self._is_enabled = True
        else:
            logger.info(f"Axis '{self.name}': Motor already enabled.")

    async def disable_motor(self) -> None:
        if self._is_enabled:
            logger.info(f"Axis '{self.name}': Disabling motor.")
            await self._low_level_api.enable_motor(self.can_id, False)
            self._is_enabled = False
            # If a move was in progress, it should be considered failed/cancelled
            if self._active_move_future and not self._active_move_future.done():
                self._active_move_future.set_exception(MotorError("Motor disabled during move", can_id=self.can_id))
        else:
            logger.info(f"Axis '{self.name}': Motor already disabled.")

    def is_enabled(self) -> bool:
        return self._is_enabled

    async def read_en_status(self) -> bool:
        self._is_enabled = await self._low_level_api.read_en_pin_status(self.can_id)
        return self._is_enabled

    def is_homed(self) -> bool:
        return self._is_homed

    def is_calibrated(self) -> bool:
        return self._is_calibrated

    async def get_current_position_steps(self) -> int:
        pos_steps = await self._low_level_api.read_encoder_value_addition(self.can_id)
        self._current_position_steps = pos_steps
        return pos_steps

    async def get_current_position_user(self) -> float:
        steps = await self.get_current_position_steps()
        return self.kinematics.steps_to_user(steps)

    async def get_current_speed_rpm(self) -> int:
        rpm = await self._low_level_api.read_motor_speed_rpm(self.can_id)
        self._current_speed_rpm = rpm
        return rpm

    async def get_current_speed_user(self) -> float:
        rpm = await self.get_current_speed_rpm()
        motor_revs_per_sec = rpm / 60.0
        # Assuming motor_speed_to_user_speed in kinematics takes motor RPM.
        # This is a slight conceptual mismatch as kinematics' motor_speed_to_user_speed
        # often expects the MKS speed parameter (0-3000).
        # For now, we'll do a direct conversion from RPM via kinematics logic.
        output_revs_per_sec = motor_revs_per_sec / self.kinematics.gear_ratio

        if isinstance(self.kinematics, RotaryKinematics):
             # Assuming RotaryKinematics has degrees_per_output_revolution
            return output_revs_per_sec * self.kinematics.degrees_per_output_revolution
        elif hasattr(self.kinematics, 'pitch'): # For LinearKinematics
            return output_revs_per_sec * self.kinematics.pitch # type: ignore
        elif hasattr(self.kinematics, 'arm_length'): # For EccentricKinematics (simplified)
            # This requires current angle for accuracy, using simplified approach
            current_steps = self._current_position_steps if self._current_position_steps is not None else await self.get_current_position_steps()
            theta_out_deg = current_steps / (self.kinematics.effective_steps_per_output_revolution / 360.0)
            theta_out_rad = math.radians(theta_out_deg)
            output_angular_speed_rad_per_sec = math.radians(output_revs_per_sec * 360.0)
            return self.kinematics.arm_length * math.cos(theta_out_rad) * output_angular_speed_rad_per_sec # type: ignore

        logger.warning(f"User speed calculation not fully supported for {type(self.kinematics)} from RPM.")
        return 0.0


    async def get_status_dict(self) -> Dict[str, Any]:
        pos_steps = self._current_position_steps if self._current_position_steps is not None else await self.get_current_position_steps()
        pos_user = self.kinematics.steps_to_user(pos_steps)
        is_en = self._is_enabled # Use cached, or await self.read_en_status() for fresh
        raw_motor_status = await self.get_motor_status_code()
        
        status_map = getattr(const, "MOTOR_STATUS_MAP", {}) # Ensure MOTOR_STATUS_MAP exists
        
        return {
            "name": self.name,
            "can_id": self.can_id,
            "timestamp": time.time(),
            "position_steps": pos_steps,
            "position_user": pos_user,
            "position_units": getattr(self.kinematics, "units", "degrees"),
            "is_enabled": is_en,
            "is_homed": self._is_homed,
            "is_calibrated": self._is_calibrated,
            "motor_status_code": raw_motor_status,
            "motor_status_str": status_map.get(raw_motor_status, "Unknown"),
            "error_state": str(self._error_state) if self._error_state else None,
            "active_move_in_progress": self._active_move_future is not None and not self._active_move_future.done(),
        }

    async def get_motor_status_code(self) -> int:
        """Queries and returns the raw motor status code (from 0xF1)."""
        return await self._low_level_api.query_motor_status(self.can_id)

    def is_move_complete(self) -> bool:
        if self._active_move_future:
            return self._active_move_future.done()
        return True

    async def wait_for_move_completion(self, timeout: Optional[float] = None) -> None:
        if self._active_move_future and not self._active_move_future.done():
            try:
                await asyncio.wait_for(self._active_move_future, timeout=timeout)
            except asyncio.TimeoutError as e:
                # Ensure future is cleaned up or error set if it's still this axis's active one
                if self._active_move_future and not self._active_move_future.done():
                    self._active_move_future.set_exception(CommunicationError(
                        f"Axis '{self.name}': Timeout waiting for move completion in wait_for_move_completion.",
                        can_id=self.can_id
                    ))
                raise CommunicationError(
                    f"Axis '{self.name}': Timeout waiting for move completion."
                ) from e
            except MKSServoError: # Includes MotorError, LimitError etc. set by _execute_move
                raise # Re-raise the original move error
        # If future is already done, this will return immediately or raise stored exception if any.
        