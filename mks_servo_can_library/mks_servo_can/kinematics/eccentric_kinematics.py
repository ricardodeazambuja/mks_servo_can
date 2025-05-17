# mks_servo_can/mks_servo_can_library/mks_servo_can/kinematics/eccentric_kinematics.py
"""
Eccentric kinematics: example for a non-linear relationship.
This is a placeholder and would need a specific mechanical model.
For example, converting an angle of a rotating input (motor) to a linear
displacement of an eccentrically driven output.
"""
import math

from mks_servo_can_library.mks_servo_can.exceptions import KinematicsError

from .base_kinematics import Kinematics


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
        max_displacement: float = 0.0,
        units: str = "mm",
    ):
        """
        Initialize eccentric kinematics.

        Args:
            steps_per_revolution: Motor steps for one full revolution of its shaft.
            arm_length: Length of the arm in the eccentric mechanism (in self.units).
            gear_ratio: Gear ratio.
            max_displacement: Maximum expected displacement. Used for speed calculations if provided.
            units: User-facing units.
        """
        super().__init__(steps_per_revolution, gear_ratio)
        if arm_length <= 0:
            raise KinematicsError("Arm length must be positive.")
        self.arm_length = arm_length
        self.units = units
        self.max_displacement = (
            max_displacement if max_displacement > 0 else arm_length
        )

        # Internal: steps per degree of the motor shaft itself (before output gearing)
        self._motor_steps_per_degree_motor_shaft = (
            self.steps_per_revolution / 360.0
        )

    def _user_to_motor_angle_deg(self, user_value: float) -> float:
        """Converts user linear displacement 'x' to motor shaft angle in degrees."""
        if abs(user_value) > self.arm_length:
            # Instead of error, clamp to max possible angle (90 deg for this simple model)
            # raise KinematicsError(f"User value {user_value} exceeds arm length {self.arm_length}.")
            user_value = math.copysign(self.arm_length, user_value)

        # Assuming user_value = arm_length * sin(motor_angle_at_output)
        # So, motor_angle_at_output_rad = asin(user_value / self.arm_length)
        # And motor_angle_at_motor_shaft_rad = motor_angle_at_output_rad * self.gear_ratio

        # Simpler: Let's assume the 'steps_per_revolution' is for the output shaft that drives the arm.
        # Then motor_angle_rad is the angle of this output shaft.
        if self.arm_length == 0:
            return 0.0  # Avoid division by zero
        ratio = user_value / self.arm_length
        if ratio > 1.0:
            ratio = 1.0
        if ratio < -1.0:
            ratio = -1.0

        output_angle_rad = math.asin(ratio)
        output_angle_deg = math.degrees(output_angle_rad)

        # This output_angle_deg is achieved by the output of the gearbox.
        # The motor itself turns gear_ratio times more/less.
        # The effective_steps_per_output_revolution already accounts for gear_ratio.
        # So, steps_per_degree for output is effective_steps_per_output_revolution / 360.
        return output_angle_deg  # This is the angle of the output shaft (arm pivot)

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
        # Now convert this output angle to steps.
        # effective_steps_per_output_revolution corresponds to 360 degrees of output shaft.
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
        motor_output_angle_deg = steps_value / (
            self.effective_steps_per_output_revolution / 360.0
        )
        return self._motor_angle_deg_to_user(motor_output_angle_deg)

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert linear speed (user_units/sec) to MKS motor speed parameter (0-3000).
        This is complex for non-linear kinematics as dx/dt = L*cos(theta)*d_theta/dt.
        So d_theta/dt (angular speed) depends on current theta (position).
        A simplification might be to calculate required angular speed at a nominal position (e.g., theta=0).
        At theta=0, dx/dt = L * d_theta/dt => d_theta/dt = (dx/dt) / L.
        """
        if self.arm_length == 0:
            return 0

        # Angular speed of output shaft in radians/sec
        # This simplification assumes cos(theta) is 1, i.e., near the center of motion (theta=0)
        output_angular_speed_rad_per_sec = user_speed / self.arm_length
        output_angular_speed_deg_per_sec = math.degrees(
            output_angular_speed_rad_per_sec
        )

        # Motor RPM (similar logic to RotaryKinematics)
        output_revs_per_second = output_angular_speed_deg_per_sec / 360.0
        motor_revs_per_second = output_revs_per_second * self.gear_ratio
        motor_rpm = motor_revs_per_second * 60

        mks_speed_param = int(round(motor_rpm))
        mks_speed_param = max(0, min(mks_speed_param, 3000))  # Clamp
        return mks_speed_param

    def motor_speed_to_user_speed(self, motor_speed_param: int) -> float:
        """
        Convert MKS motor speed parameter to linear speed (user_units/sec).
        Uses similar simplifications (assumes motion near center, theta=0).
        """
        motor_rpm = float(motor_speed_param)
        motor_revs_per_second = motor_rpm / 60.0
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        output_angular_speed_deg_per_sec = output_revs_per_second * 360.0
        output_angular_speed_rad_per_sec = math.radians(
            output_angular_speed_deg_per_sec
        )

        # Linear speed dx/dt = L * d_theta/dt (at theta=0)
        user_speed = self.arm_length * output_angular_speed_rad_per_sec
        return user_speed

    def get_parameters(self) -> dict:
        params = super().get_parameters()
        params.update(
            {
                "arm_length": self.arm_length,
                "units": self.units,
                "max_displacement": self.max_displacement,
            }
        )
        return params
