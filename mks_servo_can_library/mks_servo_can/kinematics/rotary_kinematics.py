"""
Rotary kinematics: converts between angular displacement (degrees) and motor steps.
"""
import logging # Added for warning
from .base_kinematics import Kinematics

logger = logging.getLogger(__name__) # Added for warning

class RotaryKinematics(Kinematics):
    """
    Handles conversion for rotary motion systems.
    User units are typically degrees.
    """

    def __init__(
        self,
        steps_per_revolution: int,
        gear_ratio: float = 1.0,
        degrees_per_output_revolution: float = 360.0,
    ):
        """
        Initialize rotary kinematics.

        Args:
            steps_per_revolution: Motor steps for one full revolution of its shaft.
            gear_ratio: Gear ratio between motor and the final rotating output.
            degrees_per_output_revolution: How many degrees the final output shaft turns
                                           for one of its own revolutions. Typically 360.
        """
        super().__init__(steps_per_revolution, gear_ratio)
        self.degrees_per_output_revolution = degrees_per_output_revolution
        if self.degrees_per_output_revolution == 0:
            raise ValueError("degrees_per_output_revolution cannot be zero.")
        # Steps per degree of final output rotation
        self.steps_per_degree = (
            self.effective_steps_per_output_revolution
            / self.degrees_per_output_revolution
        )
        self.units = "deg" # Default units

    def user_to_steps(self, user_value: float) -> int:
        """
        Convert angular displacement (user_value in degrees) to motor steps.
        """
        return round(user_value * self.steps_per_degree)

    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert motor steps to angular displacement (in degrees).
        """
        if self.steps_per_degree == 0:
            # This should ideally be caught by degrees_per_output_revolution != 0
            raise ValueError("steps_per_degree is zero, cannot convert.")
        return steps_value / self.steps_per_degree

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert angular speed (user_speed in units/sec, e.g., degrees/sec)
        to MKS motor speed parameter (0-3000).

        Note:
            This conversion is an approximation. The MKS speed parameter (0-3000)
            is not perfectly linear with physical RPM across all work modes and
            microstepping settings. The MKS manual (V1.0.6, Part 6.1) states the
            speed parameter is calibrated based on 16/32/64 subdivisions.
            This implementation assumes a simplified proportional mapping, typically to
            `const.MAX_RPM_VFOC_MODE` (3000 RPM) when the parameter is 3000,
            most applicable in VFOC modes. For high precision, users might need to
            calibrate this mapping for their specific setup.

        Args:
            user_speed: Angular speed in user-defined units per second (e.g., degrees/sec).

        Returns:
            MKS motor speed parameter (0-3000), clamped.
        """
        if self.degrees_per_output_revolution == 0: # Avoid division by zero
             return 0
             
        # Revolutions of the output shaft per second
        output_revs_per_second = user_speed / self.degrees_per_output_revolution
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
        Convert MKS motor speed parameter (0-3000) to angular speed (user_units/sec).

        Note:
            This conversion shares the same approximations and assumptions as
            `user_speed_to_motor_speed`. It's most representative in VFOC modes where
            the 0-3000 parameter range often corresponds more directly to RPM.
            The conversion assumes `motor_speed_param` is roughly equivalent to motor RPM
            in VFOC mode or similar high-performance modes.

        Args:
            motor_speed_param: MKS motor speed parameter (0-3000).

        Returns:
            Equivalent angular speed in user-defined units per second (e.g., degrees/sec).
        """
        if not (0 <= motor_speed_param <= 3000):
            logger.warning(f"motor_speed_param {motor_speed_param} is outside typical 0-3000 range.")

        motor_rpm = float(motor_speed_param)

        motor_revs_per_second = motor_rpm / 60.0
        if self.gear_ratio == 0: # Avoid division by zero
            return 0.0
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        user_speed = output_revs_per_second * self.degrees_per_output_revolution
        return user_speed

    def get_parameters(self) -> dict:
        """Return the parameters of this kinematic model."""
        params = super().get_parameters()
        params.update(
            {
                "degrees_per_output_revolution": self.degrees_per_output_revolution,
                "steps_per_degree": self.steps_per_degree,
                "units": self.units, # Added units to params
            }
        )
        return params
    