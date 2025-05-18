"""
Eccentric kinematics: example for a non-linear relationship.
This is a placeholder and would need a specific mechanical model.
For example, converting an angle of a rotating input (motor) to a linear
displacement of an eccentrically driven output.
"""
import math
import logging # Added for warning

from mks_servo_can.exceptions import KinematicsError

from .base_kinematics import Kinematics

logger = logging.getLogger(__name__) # Added for warning

class EccentricKinematics(Kinematics):
    """
    Example for an eccentric drive or similar non-linear mechanism.
    This requires a specific model of the mechanism.
    Let's assume user wants to control the linear position of a follower
    driven by a cam with radius 'r' and eccentricity 'e', where the motor
    controls the cam angle 'theta'.
    Linear position y = e * cos(theta) + sqrt(r^2 - (e * sin(theta))^2) (simplified example)
    Or a simpler case: y = L_arm * sin(motor_angle) for a swinging arm.

    This implementation will be a simplified example:
    Motor rotates an arm of length L, and user controls the horizontal displacement 'x'.
    Motor angle (from vertical) = asin(x / L)
    Here, 'user_value' is 'x' (linear displacement, e.g., mm)
    'motor_value' is motor angle (degrees or radians) which then converts to steps.
    """

    def __init__(
        self,
        steps_per_revolution: int,
        arm_length: float,
        gear_ratio: float = 1.0,
        max_displacement: float = 0.0, # Keep for consistency if used
        units: str = "mm",
    ):
        """
        Initialize eccentric kinematics.

        Args:
            steps_per_revolution: Motor steps for one full revolution of its shaft.
            arm_length: Length of the arm in the eccentric mechanism (in self.units).
            gear_ratio: Gear ratio. Motor revs per one output mechanism revolution.
            max_displacement: Not directly used in core calcs here but kept for API consistency.
                              Could be used for input validation if needed.
            units: User-facing units.
        """
        super().__init__(steps_per_revolution, gear_ratio)
        if arm_length <= 0:
            raise KinematicsError("Arm length must be positive.")
        self.arm_length = arm_length
        self.units = units # Store units string
        self.max_displacement = ( # For informational purposes or potential validation
            max_displacement if max_displacement > 0 else arm_length
        )


    def _user_to_motor_angle_deg(self, user_value: float) -> float:
        """Converts user linear displacement 'x' to motor shaft angle in degrees."""
        if abs(user_value) > self.arm_length:
            user_value = math.copysign(self.arm_length, user_value)
            logger.warning(f"EccentricKinematics: User value {user_value} clamped to arm length {self.arm_length}.")


        if self.arm_length == 0:
            return 0.0
        ratio = user_value / self.arm_length
        # Clamp ratio to avoid math domain errors with asin due to floating point inaccuracies
        ratio = max(-1.0, min(1.0, ratio)) 

        output_angle_rad = math.asin(ratio)
        output_angle_deg = math.degrees(output_angle_rad)
        return output_angle_deg

    def _motor_angle_deg_to_user(self, motor_output_angle_deg: float) -> float:
        """Converts motor output shaft angle (degrees) to user linear displacement 'x'."""
        motor_output_angle_rad = math.radians(motor_output_angle_deg)
        return self.arm_length * math.sin(motor_output_angle_rad)

    def user_to_steps(self, user_value: float) -> int:
        """
        Convert user linear displacement to motor steps.
        This involves converting linear -> motor_output_angle -> steps.
        """
        motor_output_angle_deg = self._user_to_motor_angle_deg(user_value)
        if self.effective_steps_per_output_revolution == 0: # Should not happen with valid init
            return 0
        steps = round(
            motor_output_angle_deg
            * (self.effective_steps_per_output_revolution / 360.0)
        )
        return steps

    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert motor steps to user linear displacement.
        This involves steps -> motor_output_angle -> linear.
        """
        if self.effective_steps_per_output_revolution == 0: # Should not happen
            return 0.0
        motor_output_angle_deg = steps_value / (
            self.effective_steps_per_output_revolution / 360.0
        )
        return self._motor_angle_deg_to_user(motor_output_angle_deg)

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert linear speed (user_units/sec) to MKS motor speed parameter (0-3000)
        for an eccentric mechanism.

        Note:
            This is a significant simplification for non-linear kinematics.
            The true relationship `dx/dt = L*cos(theta)*d_theta/dt` means that the
            angular speed `d_theta/dt` required for a given linear speed `dx/dt`
            depends on the current angle `theta` (i.e., the current position).
            This implementation approximates this by assuming motion near the center
            of the mechanism's travel (where `theta` is small and `cos(theta) approx 1`),
            leading to `dx/dt approx L * d_theta/dt`.
            The resulting MKS speed parameter also inherits the general approximations
            of mapping RPM to the 0-3000 range, as described in Linear/RotaryKinematics.
            For accurate speed control in non-linear systems, a more sophisticated model
            considering the Jacobian of the mechanism and the current position is required.
            This method should be used with caution if precise speed control over the
            entire range of motion is critical.

        Args:
            user_speed: Linear speed in `self.units` per second.

        Returns:
            MKS motor speed parameter (0-3000), clamped.
        """
        if self.arm_length == 0: # Avoid division by zero
            return 0

        # Approximate angular speed of output shaft in radians/sec (assuming cos(theta) ~ 1)
        output_angular_speed_rad_per_sec = user_speed / self.arm_length
        output_angular_speed_deg_per_sec = math.degrees(
            output_angular_speed_rad_per_sec
        )

        output_revs_per_second = output_angular_speed_deg_per_sec / 360.0
        motor_revs_per_second = output_revs_per_second * self.gear_ratio
        motor_rpm = motor_revs_per_second * 60

        mks_speed_param = int(round(motor_rpm))
        mks_speed_param = max(0, min(mks_speed_param, 3000))
        return mks_speed_param

    def motor_speed_to_user_speed(self, motor_speed_param: int) -> float:
        """
        Convert MKS motor speed parameter to linear speed (user_units/sec)
        for an eccentric mechanism.

        Note:
            Uses similar simplifications as `user_speed_to_motor_speed`, assuming
            motion near the center (theta=0, where cos(theta)~1). This is an approximation.
            The conversion assumes `motor_speed_param` is roughly equivalent to motor RPM
            in VFOC mode or similar high-performance modes.

        Args:
            motor_speed_param: MKS motor speed parameter (0-3000).

        Returns:
            Equivalent linear speed in `self.units` per second.
        """
        if not (0 <= motor_speed_param <= 3000):
            logger.warning(f"EccentricKinematics: motor_speed_param {motor_speed_param} is outside typical 0-3000 range.")

        motor_rpm = float(motor_speed_param)
        motor_revs_per_second = motor_rpm / 60.0
        if self.gear_ratio == 0: # Avoid division by zero
            return 0.0
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        output_angular_speed_deg_per_sec = output_revs_per_second * 360.0
        output_angular_speed_rad_per_sec = math.radians(
            output_angular_speed_deg_per_sec
        )

        # Approximate linear speed dx/dt = L * d_theta/dt (at theta=0)
        user_speed = self.arm_length * output_angular_speed_rad_per_sec
        return user_speed

    def get_parameters(self) -> dict:
        """Return the parameters of this kinematic model."""
        params = super().get_parameters()
        params.update(
            {
                "arm_length": self.arm_length,
                "units": self.units,
                "max_displacement": self.max_displacement,
            }
        )
        return params
    