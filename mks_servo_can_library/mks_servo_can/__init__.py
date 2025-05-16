# mks_servo_can_project/mks_servo_can_library/mks_servo_can/__init__.py
"""
MKS Servo CAN Communication Library
===================================

This library provides tools to control MKS SERVO42D and MKS SERVO57D motors
via a CAN interface. It includes low-level command APIs, high-level Axis
and MultiAxisController objects, and utilities for kinematics and simulation.
"""

# Import key classes and functions to make them available at the package level
from .constants import *
from .exceptions import *
from .crc import calculate_crc
from .can_interface import CANInterface
from .low_level_api import LowLevelAPI
from .kinematics import LinearKinematics, RotaryKinematics, EccentricKinematics
from .axis import Axis
from .multi_axis_controller import MultiAxisController

__version__ = "0.1.0"

__all__ = [
    # Constants (example, list actual constants from constants.py)
    "CAN_DEFAULT_BITRATE",
    # Exceptions
    "MKSServoError", "CANError", "CRCError", "CommandError",
    "ParameterError", "MotorError", "CommunicationError", "MultiAxisError",
    "SimulatorError", "KinematicsError",
    # CRC Util
    "calculate_crc",
    # Core classes
    "CANInterface",
    "LowLevelAPI",
    "Axis",
    "MultiAxisController",
    # Kinematics
    "LinearKinematics",
    "RotaryKinematics",
    "EccentricKinematics",
]

# Further initialization or checks can be added here if needed
print("MKS Servo CAN Library Initialized")