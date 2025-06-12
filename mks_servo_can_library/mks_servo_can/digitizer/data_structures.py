"""
Data structures for the Motor Digitizer system.

This module defines the core data structures used for recording, storing,
and analyzing digitized motor sequences.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional


@dataclass
class DigitizedPoint:
    """Single recorded point with timestamps and motor positions"""
    timestamp: float
    positions: Dict[str, float]  # axis_name -> position_user
    velocities: Optional[Dict[str, float]] = None  # calculated velocities
    metadata: Optional[Dict[str, Any]] = None  # additional data


@dataclass
class DigitizedSequence:
    """Complete recorded sequence with metadata"""
    points: List[DigitizedPoint]
    axis_names: List[str]
    axis_configs: Dict[str, Dict[str, Any]]  # axis configuration data
    recording_date: str
    recording_duration: float
    sample_rate: float
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class PlaybackStats:
    """Statistics from playback execution"""
    planned_points: int
    executed_points: int
    average_position_error: Dict[str, float]  # per axis
    max_position_error: Dict[str, float]  # per axis
    average_timing_error: float
    max_timing_error: float
    total_duration: float