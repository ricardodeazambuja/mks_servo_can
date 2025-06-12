"""
Motor Digitizer Module

Provides recording, playback, and precision testing capabilities for MKS servo motors.
"""

from .base_digitizer import MotorDigitizer
from .data_structures import (
    DigitizedPoint,
    DigitizedSequence, 
    PlaybackStats
)
from .precision_analyzer import PrecisionAnalyzer
from .surface_mapping import (
    SurfacePoint,
    SurfaceMap, 
    EnhancedHeightMapGenerator
)
from .utils import create_linear_axis, create_rotary_axis

__all__ = [
    "MotorDigitizer",
    "DigitizedPoint", 
    "DigitizedSequence",
    "PlaybackStats",
    "PrecisionAnalyzer",
    "SurfacePoint",
    "SurfaceMap",
    "EnhancedHeightMapGenerator",
    "create_linear_axis",
    "create_rotary_axis"
]