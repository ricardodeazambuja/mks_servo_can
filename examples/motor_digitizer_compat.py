"""
DEPRECATED: This module has moved to mks_servo_can.digitizer

This file provides backward compatibility. Please update your imports to:
    from mks_servo_can.digitizer import MotorDigitizer
    from mks_servo_can import MotorDigitizer  # or directly from main package
"""

import warnings
import sys
from pathlib import Path

# Add the library to the path
lib_path = Path(__file__).parent.parent / "mks_servo_can_library"
sys.path.insert(0, str(lib_path))

# Import everything from the new location
from mks_servo_can.digitizer import *

# Issue deprecation warning
warnings.warn(
    "motor_digitizer module is deprecated. "
    "Use 'from mks_servo_can.digitizer import MotorDigitizer' instead.",
    DeprecationWarning,
    stacklevel=2
)