# mks_servo_can/mks_servo_can_library/mks_servo_can/kinematics/rotary_kinematics.py
"""
Rotary kinematics: converts between angular displacement (degrees) and motor steps.
"""
import math
from .base_kinematics import Kinematics
from mks_servo_can_library.mks_servo_can.constants import MAX_RPM_VFOC_MODE

class RotaryKinematics(Kinematics):
    """
    Handles conversion for rotary motion systems.
    User units are typically degrees.
    """
    def __init__(self, steps_per_revolution: int, gear_ratio: float = 1.0,
                 degrees_per_output_revolution: float = 360.0):
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
        # Steps per degree of final output rotation
        self.steps_per_degree = self.effective_steps_per_output_revolution / self.degrees_per_output_revolution

    def user_to_steps(self, user_value: float) -> int:
        """
        Convert angular displacement (user_value in degrees) to motor steps.
        """
        return round(user_value * self.steps_per_degree)

    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert motor steps to angular displacement (in degrees).
        """
        return steps_value / self.steps_per_degree

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert angular speed (degrees per second) to MKS motor speed parameter (0-3000).
        Similar assumptions as LinearKinematics regarding MKS speed parameter.
        """
        # Revolutions of the output shaft per second
        output_revs_per_second = user_speed / self.degrees_per_output_revolution
        # Revolutions of the motor per second
        motor_revs_per_second = output_revs_per_second * self.gear_ratio
        # Motor RPM
        motor_rpm = motor_revs_per_second * 60

        mks_speed_param = int(round(motor_rpm)) # Simplistic: assume param is roughly RPM in VFOC
        mks_speed_param = max(0, min(mks_speed_param, 3000))
        return mks_speed_param

    def motor_speed_to_user_speed(self, motor_speed_param: int) -> float:
        """
        Convert MKS motor speed parameter (0-3000) to angular speed (degrees per second).
        """
        motor_rpm = float(motor_speed_param) # Simplistic
        
        motor_revs_per_second = motor_rpm / 60
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        user_speed = output_revs_per_second * self.degrees_per_output_revolution
        return user_speed

    def get_parameters(self) -> dict:
        params = super().get_parameters()
        params.update({
            "degrees_per_output_revolution": self.degrees_per_output_revolution,
            "steps_per_degree": self.steps_per_degree
        })
        return params