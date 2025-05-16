# mks_servo_can_project/mks_servo_can_library/mks_servo_can/kinematics/base_kinematics.py
"""
Base class for kinematic transformations.
"""
from abc import ABC, abstractmethod
from mks_servo_can_library.mks_servo_can.exceptions import KinematicsError

class Kinematics(ABC):
    """
    Abstract base class for defining kinematic relationships.
    It handles conversions between desired final movement units (e.g., mm, degrees)
    and motor encoder values or pulses.
    """
    def __init__(self, steps_per_revolution: int, gear_ratio: float = 1.0):
        """
        Initialize base kinematics.

        Args:
            steps_per_revolution: The number of steps (or microsteps, or encoder pulses)
                                  the motor makes for one full revolution of its output shaft.
                                  This should be the value corresponding to commands like
                                  CMD_RUN_POSITION_MODE_RELATIVE_PULSES.
                                  Often this is base_motor_steps * microstepping_setting for steppers,
                                  or encoder_resolution for closed-loop servos.
            gear_ratio: The gear ratio applied after the motor's output shaft.
                        Ratio > 1 means output turns slower than motor.
                        Ratio < 1 means output turns faster than motor.
                        Default is 1.0 (no gearing).
        """
        if steps_per_revolution <= 0:
            raise KinematicsError("steps_per_revolution must be positive.")
        if gear_ratio <= 0:
            raise KinematicsError("gear_ratio must be positive.")

        self.steps_per_revolution = steps_per_revolution
        self.gear_ratio = gear_ratio
        # Effective steps at the final output after gearing
        self.effective_steps_per_output_revolution = self.steps_per_revolution * self.gear_ratio

    @abstractmethod
    def user_to_steps(self, user_value: float) -> int:
        """
        Convert a value from user units (e.g., mm, degrees) to motor steps/pulses.

        Args:
            user_value: The value in user-defined units.

        Returns:
            The equivalent number of motor steps/pulses (integer).
        """
        pass

    @abstractmethod
    def steps_to_user(self, steps_value: int) -> float:
        """
        Convert a value from motor steps/pulses to user units (e.g., mm, degrees).

        Args:
            steps_value: The number of motor steps/pulses.

        Returns:
            The equivalent value in user-defined units (float).
        """
        pass

    @abstractmethod
    def user_speed_to_motor_speed(self, user_speed: float) -> int:
        """
        Convert a speed from user units per second to motor speed units (e.g., RPM, or
        the speed parameter used by commands like 0xF6, 0xFD which is 0-3000).
        Note: The MKS speed parameter (0-3000) is not directly RPM for all modes/subdivisions.
        This method might need more context or make assumptions about how 'motor speed' is defined.
        For simplicity, this might target the 0-3000 parameter if a clear mapping exists.

        Args:
            user_speed: Speed in user units per second.

        Returns:
            Equivalent motor speed parameter (integer).
        """
        pass

    @abstractmethod
    def motor_speed_to_user_speed(self, motor_speed: int) -> float:
        """
        Convert a motor speed parameter back to user units per second.

        Args:,
            motor_speed: Motor speed parameter (e.g., 0-3000 from servo commands).

        Returns:
            Equivalent speed in user units per second.
        """
        pass

    def get_parameters(self) -> dict:
        """Return the parameters of this kinematic model."""
        return {
            "type": self.__class__.__name__,
            "steps_per_revolution": self.steps_per_revolution,
            "gear_ratio": self.gear_ratio,
            "effective_steps_per_output_revolution": self.effective_steps_per_output_revolution
        }

    def __repr__(self) -> str:
        params = self.get_parameters()
        param_str = ", ".join(f"{k}={v}" for k, v in params.items())
        return f"{self.__class__.__name__}({param_str})"