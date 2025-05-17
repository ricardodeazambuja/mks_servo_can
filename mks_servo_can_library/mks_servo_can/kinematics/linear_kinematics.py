# mks_servo_can/mks_servo_can_library/mks_servo_can/kinematics/linear_kinematics.py
"""
Linear kinematics: converts between linear distance (e.g., mm) and motor steps.
"""
import math
from .base_kinematics import Kinematics
from mks_servo_can_library.mks_servo_can.exceptions import KinematicsError
from mks_servo_can_library.mks_servo_can.constants import MAX_RPM_VFOC_MODE # Default max RPM

class LinearKinematics(Kinematics):
    """
    Handles conversion for linear motion systems (e.g., lead screw, belt and pinion).
    User units are typically millimeters (mm).
    """
    def __init__(self, steps_per_revolution: int, pitch: float,
                 gear_ratio: float = 1.0, units: str = "mm"):
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
        self.steps_per_user_unit = self.effective_steps_per_output_revolution / self.pitch

    def user_to_steps(self, user_value: float) -> int:
        """
        Convert linear distance (user_value in self.units) to motor steps.
        """
        return round(user_value * self.steps_per_user_unit)

    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert motor steps to linear distance (in self.units).
        """
        return steps_value / self.steps_per_user_unit

    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert linear speed (user_value_per_second) to MKS motor speed parameter (0-3000).
        This is an approximation as the MKS speed parameter is not directly linear with RPM
        across all microstepping settings. It's calibrated for 16/32/64 microsteps according to manual.
        Assumes the 0-3000 range corresponds to a max RPM (e.g., 3000 RPM in VFOC mode).

        Args:
            user_speed: Linear speed in user_units / second.

        Returns:
            MKS motor speed parameter (0-3000).
        """
        # Revolutions of the output shaft per second
        output_revs_per_second = user_speed / self.pitch
        # Revolutions of the motor per second
        motor_revs_per_second = output_revs_per_second * self.gear_ratio
        # Motor RPM
        motor_rpm = motor_revs_per_second * 60

        # Convert motor_rpm to the MKS speed parameter (0-3000)
        # This mapping needs to be based on the MKS manual's definition of the speed parameter.
        # If the speed parameter directly maps to a certain RPM range:
        # Example: If 3000 parameter value = MAX_RPM_VFOC_MODE (3000 RPM)
        # This is a simplification; the actual relationship might be more complex.
        # The manual (6.1) states the speed parameter (0-3000) makes the motor rotate faster.
        # For SR_VFOC, max speed is 3000 RPM.
        # Let's assume the 0-3000 parameter maps somewhat proportionally to 0-MAX_RPM_VFOC_MODE
        # This is a major simplification.
        # A more accurate conversion would consider how the MKS controller firmware interprets this value.
        # Given the documentation in 6.1 regarding subdivisions effect on speed, this is tricky.
        # "speed value is calibrated based on 16/32/64 subdivisions"
        # For now, let's assume a direct scaling for the default microstepping assumption (16/32/64)
        
        # If we assume the parameter 0-3000 maps to 0-3000 RPM (in modes like VFOC)
        # then mks_speed_param = motor_rpm
        # However, this needs to be clamped and scaled by MAX_RPM for that parameter.
        # The MKS Speed parameter in commands (e.g. F6, FD) is 0-3000.
        # This parameter itself is not RPM directly, but a value that results in speed.
        # The manual (Part 6.1) is a bit vague on direct conversion to this parameter from physical units.
        # Let's assume for now:
        #   mks_speed_param = (motor_rpm / MAX_RPM_FOR_PARAM) * 3000
        #   If MAX_RPM_FOR_PARAM is also 3000 (like in VFOC mode), then mks_speed_param is approx motor_rpm.
        
        mks_speed_param = int(round(motor_rpm)) # Simplistic: assume param is roughly RPM in VFOC mode
        
        # Clamp to the 0-3000 range for the parameter itself
        mks_speed_param = max(0, min(mks_speed_param, 3000))
        return mks_speed_param

    def motor_speed_to_user_speed(self, motor_speed_param: int) -> float:
        """
        Convert MKS motor speed parameter (0-3000) to linear speed (user_units / second).
        Uses similar assumptions as user_speed_to_motor_speed.
        """
        if not (0 <= motor_speed_param <= 3000):
            # Allow it, but it might be outside effective range
            pass

        # Assume motor_speed_param is roughly motor_rpm in modes like VFOC
        motor_rpm = float(motor_speed_param) # Simplistic
        
        motor_revs_per_second = motor_rpm / 60
        output_revs_per_second = motor_revs_per_second / self.gear_ratio
        user_speed = output_revs_per_second * self.pitch
        return user_speed

    def get_parameters(self) -> dict:
        params = super().get_parameters()
        params.update({
            "pitch": self.pitch,
            "units": self.units,
            "steps_per_user_unit": self.steps_per_user_unit
        })
        return params