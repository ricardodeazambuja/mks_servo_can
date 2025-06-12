"""
Precision Analyzer for Motor Digitizer System

This module provides statistical analysis and assessment capabilities
for digitizer precision testing and system performance evaluation.
"""

import math
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass

from .data_structures import PlaybackStats, DigitizedSequence


@dataclass
class RepeatabilityStats:
    """Statistics from repeatability analysis"""
    mean_position_error: Dict[str, float]
    std_position_error: Dict[str, float]
    mean_timing_error: float
    std_timing_error: float
    run_count: int
    confidence_interval: float


class PrecisionAnalyzer:
    """
    Analyzes precision and repeatability of digitizer operations.
    
    Provides statistical analysis, performance assessment, and
    automated precision scoring for digitizer systems.
    """
    
    # Precision assessment thresholds
    PRECISION_THRESHOLDS = {
        "EXCELLENT": {"position": 0.1, "max_position": 0.5, "timing": 0.05},
        "GOOD": {"position": 0.5, "max_position": 2.0, "timing": 0.1},
        "FAIR": {"position": 1.0, "max_position": 5.0, "timing": 0.2},
        "POOR": {"position": float('inf'), "max_position": float('inf'), "timing": float('inf')}
    }
    
    @classmethod
    def assess_precision(cls, stats: PlaybackStats) -> str:
        """
        Assess overall precision performance.
        
        Args:
            stats: PlaybackStats from precision testing
            
        Returns:
            Assessment string: "EXCELLENT", "GOOD", "FAIR", or "POOR"
        """
        # Calculate overall position error (average across all axes)
        avg_position_errors = list(stats.average_position_error.values())
        max_position_errors = list(stats.max_position_error.values())
        
        if not avg_position_errors:
            return "POOR"
            
        overall_avg_error = sum(avg_position_errors) / len(avg_position_errors)
        overall_max_error = max(max_position_errors) if max_position_errors else float('inf')
        timing_error = stats.average_timing_error
        
        # Check thresholds in order of quality
        for assessment, thresholds in cls.PRECISION_THRESHOLDS.items():
            if (overall_avg_error <= thresholds["position"] and 
                overall_max_error <= thresholds["max_position"] and 
                timing_error <= thresholds["timing"]):
                return assessment
                
        return "POOR"
    
    @classmethod
    def calculate_path_complexity(cls, sequence: DigitizedSequence) -> float:
        """
        Calculate path complexity score based on movement characteristics.
        
        Args:
            sequence: DigitizedSequence to analyze
            
        Returns:
            Complexity score (0.0 = simple, 1.0 = very complex)
        """
        if len(sequence.points) < 2:
            return 0.0
            
        total_distance = 0.0
        direction_changes = 0
        speed_changes = 0
        
        for i in range(1, len(sequence.points)):
            prev_point = sequence.points[i-1]
            curr_point = sequence.points[i]
            
            # Calculate distance moved
            axis_distances = []
            for axis in sequence.axis_names:
                if axis in prev_point.positions and axis in curr_point.positions:
                    dist = abs(curr_point.positions[axis] - prev_point.positions[axis])
                    axis_distances.append(dist)
                    
            if axis_distances:
                segment_distance = math.sqrt(sum(d**2 for d in axis_distances))
                total_distance += segment_distance
                
                # Check for direction changes (simplified)
                if i > 1:
                    prev_prev_point = sequence.points[i-2]
                    # Calculate direction vectors and check for changes
                    # This is a simplified approach - could be more sophisticated
                    direction_changes += 1 if segment_distance > 0.1 else 0
                    
        # Normalize factors
        avg_distance_per_point = total_distance / len(sequence.points) if total_distance > 0 else 0
        direction_change_ratio = direction_changes / len(sequence.points)
        
        # Combine into complexity score (0-1)
        complexity = min(1.0, (avg_distance_per_point / 10.0) + 
                        (direction_change_ratio * 0.5))
        
        return complexity
    
    @classmethod
    def analyze_repeatability(cls, stats_list: List[PlaybackStats], 
                            confidence_level: float = 0.95) -> RepeatabilityStats:
        """
        Analyze repeatability across multiple test runs.
        
        Args:
            stats_list: List of PlaybackStats from multiple runs
            confidence_level: Confidence level for statistical analysis
            
        Returns:
            RepeatabilityStats with statistical analysis
        """
        if not stats_list:
            raise ValueError("No statistics provided for repeatability analysis")
            
        run_count = len(stats_list)
        
        # Collect position errors by axis
        axis_errors = {}
        timing_errors = []
        
        for stats in stats_list:
            timing_errors.append(stats.average_timing_error)
            
            for axis, error in stats.average_position_error.items():
                if axis not in axis_errors:
                    axis_errors[axis] = []
                axis_errors[axis].append(error)
        
        # Calculate statistics
        mean_position_error = {}
        std_position_error = {}
        
        for axis, errors in axis_errors.items():
            mean_position_error[axis] = sum(errors) / len(errors)
            if len(errors) > 1:
                variance = sum((e - mean_position_error[axis])**2 for e in errors) / (len(errors) - 1)
                std_position_error[axis] = math.sqrt(variance)
            else:
                std_position_error[axis] = 0.0
        
        mean_timing_error = sum(timing_errors) / len(timing_errors)
        if len(timing_errors) > 1:
            timing_variance = sum((e - mean_timing_error)**2 for e in timing_errors) / (len(timing_errors) - 1)
            std_timing_error = math.sqrt(timing_variance)
        else:
            std_timing_error = 0.0
        
        return RepeatabilityStats(
            mean_position_error=mean_position_error,
            std_position_error=std_position_error,
            mean_timing_error=mean_timing_error,
            std_timing_error=std_timing_error,
            run_count=run_count,
            confidence_interval=confidence_level
        )
    
    @classmethod
    def generate_performance_report(cls, stats: PlaybackStats, 
                                  sequence: DigitizedSequence,
                                  repeatability: Optional[RepeatabilityStats] = None) -> Dict[str, Any]:
        """
        Generate comprehensive performance report.
        
        Args:
            stats: PlaybackStats from precision testing
            sequence: DigitizedSequence that was tested
            repeatability: Optional repeatability statistics
            
        Returns:
            Comprehensive performance report dictionary
        """
        assessment = cls.assess_precision(stats)
        complexity = cls.calculate_path_complexity(sequence)
        
        report = {
            "overall_assessment": assessment,
            "path_complexity": complexity,
            "execution_success_rate": stats.executed_points / stats.planned_points,
            "timing_performance": {
                "average_error_ms": stats.average_timing_error * 1000,
                "max_error_ms": stats.max_timing_error * 1000,
                "assessment": cls._assess_timing(stats.average_timing_error)
            },
            "position_performance": {},
            "sequence_info": {
                "total_points": len(sequence.points),
                "duration_seconds": sequence.recording_duration,
                "sample_rate": sequence.sample_rate,
                "axes": sequence.axis_names
            }
        }
        
        # Per-axis position performance
        for axis in stats.average_position_error:
            avg_err = stats.average_position_error[axis]
            max_err = stats.max_position_error[axis]
            
            report["position_performance"][axis] = {
                "average_error": avg_err,
                "max_error": max_err,
                "assessment": cls._assess_position_error(avg_err, max_err)
            }
        
        # Add repeatability data if available
        if repeatability:
            report["repeatability"] = {
                "run_count": repeatability.run_count,
                "position_std_dev": repeatability.std_position_error,
                "timing_std_dev": repeatability.std_timing_error,
                "confidence_level": repeatability.confidence_interval
            }
        
        return report
    
    @classmethod
    def _assess_timing(cls, avg_timing_error: float) -> str:
        """Assess timing performance"""
        for assessment, thresholds in cls.PRECISION_THRESHOLDS.items():
            if avg_timing_error <= thresholds["timing"]:
                return assessment
        return "POOR"
    
    @classmethod
    def _assess_position_error(cls, avg_error: float, max_error: float) -> str:
        """Assess position error performance"""
        for assessment, thresholds in cls.PRECISION_THRESHOLDS.items():
            if (avg_error <= thresholds["position"] and 
                max_error <= thresholds["max_position"]):
                return assessment
        return "POOR"