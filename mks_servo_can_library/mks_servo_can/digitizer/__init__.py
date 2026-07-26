"""
Motor Digitizer Module

Provides recording, playback, and precision testing capabilities for MKS servo motors.
"""

from .base_digitizer import MotorDigitizer
from .data_structures import DigitizedPoint, DigitizedSequence, PlaybackStats
from .precision_analyzer import PrecisionAnalyzer
from .surface_mapping import EnhancedHeightMapGenerator, SurfaceMap, SurfacePoint
from .utils import create_linear_axis, create_rotary_axis

__all__ = [
    "DigitizedPoint",
    "DigitizedSequence",
    "EnhancedHeightMapGenerator",
    "MotorDigitizer",
    "PlaybackStats",
    "PrecisionAnalyzer",
    "SurfaceMap",
    "SurfacePoint",
    "create_linear_axis",
    "create_rotary_axis"
]
