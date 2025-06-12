"""
Utility functions for the Motor Digitizer system.

This module provides convenience functions for creating commonly used
axis configurations and other digitizer utilities.
"""

from .. import CANInterface, Axis, LinearKinematics, RotaryKinematics, const


def create_linear_axis(can_interface: CANInterface, motor_can_id: int, 
                      name: str, pitch_mm: float, gear_ratio: float = 1.0) -> Axis:
    """
    Create a linear axis with standard configuration.
    
    Args:
        can_interface: Connected CANInterface instance
        motor_can_id: CAN ID of the motor
        name: Name for the axis
        pitch_mm: Lead screw pitch in millimeters
        gear_ratio: Gear ratio (default 1.0)
        
    Returns:
        Configured Axis instance
    """
    kinematics = LinearKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        pitch=pitch_mm,
        gear_ratio=gear_ratio,
        units="mm"
    )
    return Axis(can_interface, motor_can_id=motor_can_id, name=name, kinematics=kinematics)


def create_rotary_axis(can_interface: CANInterface, motor_can_id: int, 
                      name: str, gear_ratio: float = 1.0) -> Axis:
    """
    Create a rotary axis with standard configuration.
    
    Args:
        can_interface: Connected CANInterface instance
        motor_can_id: CAN ID of the motor
        name: Name for the axis
        gear_ratio: Gear ratio (default 1.0)
        
    Returns:
        Configured Axis instance
    """
    kinematics = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        gear_ratio=gear_ratio
    )
    return Axis(can_interface, motor_can_id=motor_can_id, name=name, kinematics=kinematics)