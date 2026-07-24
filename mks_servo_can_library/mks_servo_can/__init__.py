"""
MKS Servo CAN Communication Library
===================================

This library provides tools to control MKS SERVO42D and MKS SERVO57D motors
via a CAN interface. It includes low-level command APIs, high-level Axis
and MultiAxisController objects, and utilities for kinematics and simulation.
"""

# Import the constants module and alias it as 'const' for patterned access
from . import constants as const
from . import motor_profile

# Import other key components from submodules
from .axis import Axis
from .can_interface import CANInterface

# Make all constants available directly in the package's namespace
# This allows users to do 'from mks_servo_can import CAN_DEFAULT_BITRATE'
from .constants import *

# Import CRC functions explicitly if they are to be exported
from .crc import calculate_crc, verify_crc

# Import digitizer classes
from .digitizer import (
    DigitizedPoint,
    DigitizedSequence,
    EnhancedHeightMapGenerator,
    MotorDigitizer,
    PlaybackStats,
    PrecisionAnalyzer,
    SurfaceMap,
    SurfacePoint,
    create_linear_axis,
    create_rotary_axis,
)

# Import all exception classes explicitly if they are to be exported
from .exceptions import (
    CalibrationError,
    CANError,
    CommandError,
    CommunicationError,
    ConfigurationError,
    CRCError,
    HomingError,
    KinematicsError,
    LimitError,
    MKSServoError,
    MotorError,
    MultiAxisError,
    ParameterError,
    SimulatorError,
    StallError,
)

# Import kinematics classes
from .kinematics import (
    EccentricKinematics,
    Kinematics,
    LinearKinematics,
    RotaryKinematics,
)
from .low_level_api import LowLevelAPI
from .multi_axis_controller import MultiAxisController
from .realtime import (
    AlphaBetaGammaTracker,
    AlphaBetaTracker,
    ServoStream,
    StreamAxis,
    StreamStats,
)

# Import robot kinematics classes
from .robot_kinematics import CartesianRobot, RobotModelBase, RRRArm, TwoLinkArmPlanar

__version__ = "0.3.0"

__all__ = [
    # Export the 'const' alias for the constants module
    "const",

    # Export key classes
    "CANInterface",
    "LowLevelAPI",
    "Axis",
    "MultiAxisController",

    # Motion parameter model (speed/accel parameters <-> engineering units)
    "motor_profile",

    # Fixed-rate streaming control, for tracking a moving reference
    "ServoStream",
    "StreamAxis",
    "StreamStats",
    "AlphaBetaTracker",
    "AlphaBetaGammaTracker",

    # Kinematics classes
    "Kinematics",
    "LinearKinematics",
    "RotaryKinematics",
    "EccentricKinematics",

    # Robot Kinematics classes
    "RobotModelBase",
    "TwoLinkArmPlanar",
    "CartesianRobot",
    "RRRArm",

    # Digitizer classes
    "MotorDigitizer",
    "DigitizedPoint",
    "DigitizedSequence",
    "PlaybackStats",
    "PrecisionAnalyzer",
    "SurfacePoint",
    "SurfaceMap",
    "EnhancedHeightMapGenerator",
    "create_linear_axis",
    "create_rotary_axis",

    # Export utility functions from crc module
    "calculate_crc",
    "verify_crc", # Added verify_crc as it's often useful

    # Export specific exceptions (already imported above)
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

    # --- Dynamically export all constants from .constants module ---
    # This section ensures that any constant defined in .constants
    # is also exported if a user does 'from mks_servo_can import *'
    # and makes them available for direct import like 'from mks_servo_can import SOME_CONSTANT'.
]

# Dynamically add all constants from the .constants module to __all__
# This ensures that `from mks_servo_can import *` also imports all constants.
# It also satisfies linters that check if __all__ entries are defined in the module.
# The `from .constants import *` above already brought them into the namespace.
_constants_to_export = [
    item for item in dir(const) if not item.startswith("_")
]
__all__.extend(_constants_to_export)
# Clean up to avoid polluting the module's namespace with loop variables
del _constants_to_export

# Note: The print statement was removed as per previous good practice.
# Use logging if initialization messages are needed for debugging.
