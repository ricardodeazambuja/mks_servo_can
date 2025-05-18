"""
Linear kinematics: converts between linear distance (e.g., mm) and motor steps.
"""

import logging # Added for warning

from mks_servo_can.constants import \
    MAX_RPM_VFOC_MODE  # Default max RPM
from mks_servo_can.exceptions import KinematicsError

from .base_kinematics import Kinematics

logger = logging.getLogger(__name__) # Added for warning

class LinearKinematics(Kinematics):
    """
    Handles conversion for linear motion systems (e.g., lead screw, belt and pinion).
    User units are typically millimeters (mm).
    """

    def __init__(
        self,
        steps_per_revolution: int,
        pitch: float,
        gear_ratio: float = 1.0,
        units: str = "mm",
    ):
        """
        Initialize linear kinematics.

        Args:
            steps_per_revolution: Motor steps for one full revolution of its shaft.
            pitch: Linear distance moved for one full revolution of the final output
                   (e.g., mm per revolution of a lead screw after gearing).
            gear_ratio: Gear ratio between motor and the element producing linear motion.
            units: The user-facing units for distance and speed (e.g., "mm").
        """
        super().__init__(steps_per_revolution, gear_ratio)
        if pitch <= 0:
            raise KinematicsError("Pitch must be positive.")
        self.pitch = pitch
        self.units = units

        # Steps per unit of linear travel (e.g., steps per mm)
        self.steps_per_user_unit = (
            self.effective_steps_per_output_revolution / self.pitch
        )

    def user_to_steps(self, user_value: float) -> int:
        """
        Convert linear distance (user_value in self.units) to motor steps.
        """
        return round(user_value * self.steps_per_user_unit)

    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert motor steps to linear distance (in self.units).
        """
        if self.steps_per_user_unit == 0:
            # Avoid division by zero if pitch or steps_per_revolution were such that this becomes zero.
            # This case should ideally be caught by earlier validation (e.g. pitch > 0).
            raise KinematicsError("steps_per_user_unit is zero, cannot convert steps to user units.")
        return steps_value / self.steps_per_user_unit

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert linear speed (user_value_per_second) to MKS motor speed parameter (0-3000).

        Note:
            This conversion is an approximation. The MKS speed parameter (0-3000)
            is not perfectly linear with physical RPM across all work modes and microstepping
            settings. The MKS manual (V1.0.6, Part 6.1) states the speed parameter is
            calibrated based on 16/32/64 subdivisions. This implementation assumes a
            simplified proportional mapping, typically to `const.MAX_RPM_VFOC_MODE` (3000 RPM)
            when the parameter is 3000, which is most applicable in VFOC modes.
            For high precision, users might need to calibrate this mapping for their
            specific setup and motor configuration if not using VFOC mode or if using
            other microstepping values.

        Args:
            user_speed: Linear speed in user_units / second.

        Returns:
            MKS motor speed parameter (0-3000), clamped to this range.
        """
        if self.pitch == 0: # Avoid division by zero
            return 0
            
        # Revolutions of the output shaft per second
        output_revs_per_second = user_speed / self.pitch
        # Revolutions of the motor per second
        motor_revs_per_second = output_revs_per_second * self.gear_ratio
        # Motor RPM
        motor_rpm = motor_revs_per_second * 60

        mks_speed_param = int(
            round(motor_rpm)
        )

        mks_speed_param = max(0, min(mks_speed_param, 3000))
        return mks_speed_param

    def motor_speed_to_user_speed(self, motor_speed_param: int) -> float:
        """
        Convert MKS motor speed parameter (0-3000) to linear speed (user_units / second).

        Note:
            This conversion shares the same approximations and assumptions as
            `user_speed_to_motor_speed`. It's most representative in VFOC modes
            where the 0-3000 parameter range often corresponds more directly to RPM.
            The conversion assumes `motor_speed_param` is roughly equivalent to motor RPM
            in VFOC mode or similar high-performance modes.

        Args:
            motor_speed_param: MKS motor speed parameter (0-3000).

        Returns:
            Equivalent linear speed in user_units / second.
        """
        if not (0 <= motor_speed_param <= 3000):
            # Allow it, but it might be outside effective range
            logger.warning(f"motor_speed_param {motor_speed_param} is outside typical 0-3000 range.")


        motor_rpm = float(motor_speed_param)

        motor_revs_per_second = motor_rpm / 60.0
        if self.gear_ratio == 0: # Avoid division by zero
            return 0.0
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        user_speed = output_revs_per_second * self.pitch
        return user_speed

    def get_parameters(self) -> dict:
        """Return the parameters of this kinematic model."""
        params = super().get_parameters()
        params.update(
            {
                "pitch": self.pitch,
                "units": self.units,
                "steps_per_user_unit": self.steps_per_user_unit,
            }
        )
        return params
    