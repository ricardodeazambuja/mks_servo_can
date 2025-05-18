"""
MKS Servo CAN Communication Library
===================================

This library provides tools to control MKS SERVO42D and MKS SERVO57D motors
via a CAN interface. It includes low-level command APIs, high-level Axis
and MultiAxisController objects, and utilities for kinematics and simulation.
"""

# Import other key components
# Import the constants module and alias it as 'const' for patterned access (e.g., const.CAN_DEFAULT_BITRATE)
from . import constants as const
from . import crc  # Import the module itself
from . import exceptions  # Import the module itself
from .axis import Axis
from .can_interface import CANInterface

# Also make individual constants available directly from the package if desired,
# e.g., from mks_servo_can import CAN_DEFAULT_BITRATE.
# The __all__ list will control what 'from mks_servo_can import *' imports.
from .constants import *  # This makes them available directly in this namespace.
from .kinematics import EccentricKinematics
from .kinematics import Kinematics
from .kinematics import LinearKinematics
from .kinematics import RotaryKinematics
from .low_level_api import LowLevelAPI
from .multi_axis_controller import MultiAxisController

__version__ = "0.1.0"

__all__ = [
    # Export the 'const' alias for the constants module
    "const",
    # Export key classes
    "CANInterface",
    "LowLevelAPI",
    "Axis",
    "MultiAxisController",
    "Kinematics",
    "LinearKinematics",
    "RotaryKinematics",
    "EccentricKinematics",
    # Export utility functions or specific exceptions if needed directly
    "calculate_crc",  # from crc module (assuming crc.py has this function)
    # "verify_crc", # from crc module (add if needed in public API)
    # Export specific exceptions if they are commonly caught directly
    # (These should be defined in exceptions.py and imported above)
    "MKSServoError",
    "CANError",
    "CRCError",
    "CommandError",
    "ParameterError",
    "MotorError",
    "CommunicationError",
    "MultiAxisError",
    "SimulatorError",
    "KinematicsError",
    "ConfigurationError",
    "HomingError",
    "CalibrationError",
    "LimitError",
    "StallError",
    # Export all constants made available by 'from .constants import *'
    # This can be long. Alternatively, users can use 'const.CONSTANT_NAME'.
    # If you want `from mks_servo_can import *` to bring in all constants,
    # you'd need to list them or dynamically populate __all__ from constants.py.
    # For now, focusing on making `import const` work.
    # Example of exporting a few key constants directly (these are already covered by 'from .constants import *'):
    # "CAN_DEFAULT_BITRATE",
    # "MODE_SR_VFOC",
    # "ENCODER_PULSES_PER_REVOLUTION",
    # ... (add other frequently used constants here if desired for direct import via *)
]

# To make all constants from constants.py available via "from ... import *"
# and also have them listed in __all__ if you prefer explicit control:
#
# import inspect as _inspect
# from . import constants as _constants_module_for_all
#
# _constants_to_export = []
# for _name, _val in _inspect.getmembers(_constants_module_for_all):
#     if not _name.startswith("_") and isinstance(_val, (int, str, float, tuple, dict, bool)): # Check type
#         globals()[_name] = _val # Make it available in this module's scope
#         _constants_to_export.append(_name)
#
# __all__.extend(_constants_to_export) # Add all constants to __all__
#
# del _inspect, _constants_module_for_all, _constants_to_export, _name, _val # Clean up
#
# Note: The above dynamic __all__ population is more advanced.
# The current setup with 'from .constants import *' and manually listing key ones in __all__
# (or relying on users to use `const.X`) is simpler to start with.
# The `from .constants import *` makes them available, but `__all__` controls `import *`.

# Removed the print("MKS Servo CAN Library Initialized") to keep __init__.py cleaner.
# Use logging if initialization messages are needed for debugging.
