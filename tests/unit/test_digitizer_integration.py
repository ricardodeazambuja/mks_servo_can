"""
Integration tests for the digitizer module.

These tests verify that the digitizer module integrates properly 
with the main library and can be imported correctly.
"""

import pytest
import sys
from pathlib import Path

# Add the library to the path for testing
lib_path = Path(__file__).parent.parent.parent / "mks_servo_can_library"
sys.path.insert(0, str(lib_path))


class TestDigitizerImports:
    """Test that digitizer components can be imported from the main library"""
    
    def test_import_from_main_library(self):
        """Test importing digitizer classes from main library"""
        from mks_servo_can import (
            MotorDigitizer,
            DigitizedPoint,
            DigitizedSequence,
            PlaybackStats,
            PrecisionAnalyzer,
            SurfacePoint,
            SurfaceMap,
            EnhancedHeightMapGenerator,
            create_linear_axis,
            create_rotary_axis
        )
        
        # Verify classes are imported correctly
        assert MotorDigitizer is not None
        assert DigitizedPoint is not None
        assert DigitizedSequence is not None
        assert PlaybackStats is not None
        assert PrecisionAnalyzer is not None
        assert SurfacePoint is not None
        assert SurfaceMap is not None
        assert EnhancedHeightMapGenerator is not None
        assert create_linear_axis is not None
        assert create_rotary_axis is not None
    
    def test_import_from_digitizer_module(self):
        """Test importing directly from digitizer module"""
        from mks_servo_can.digitizer import (
            MotorDigitizer,
            DigitizedPoint,
            DigitizedSequence,
            PlaybackStats
        )
        
        assert MotorDigitizer is not None
        assert DigitizedPoint is not None
        assert DigitizedSequence is not None
        assert PlaybackStats is not None
    
    def test_data_structures_creation(self):
        """Test creating basic data structures"""
        from mks_servo_can import DigitizedPoint, DigitizedSequence, PlaybackStats
        
        # Test DigitizedPoint creation
        point = DigitizedPoint(
            timestamp=1.0,
            positions={"X": 10.0, "Y": 5.0},
            velocities={"X": 2.0, "Y": 1.0}
        )
        assert point.timestamp == 1.0
        assert point.positions["X"] == 10.0
        
        # Test DigitizedSequence creation
        sequence = DigitizedSequence(
            points=[point],
            axis_names=["X", "Y"],
            axis_configs={"X": {"type": "linear"}, "Y": {"type": "linear"}},
            recording_date="2023-01-01",
            recording_duration=5.0,
            sample_rate=10.0
        )
        assert len(sequence.points) == 1
        assert sequence.sample_rate == 10.0
        
        # Test PlaybackStats creation
        stats = PlaybackStats(
            planned_points=100,
            executed_points=95,
            average_position_error={"X": 0.1, "Y": 0.2},
            max_position_error={"X": 0.5, "Y": 0.8},
            average_timing_error=0.01,
            max_timing_error=0.05,
            total_duration=10.0
        )
        assert stats.planned_points == 100
        assert stats.executed_points == 95
    
    def test_precision_analyzer_basic(self):
        """Test basic PrecisionAnalyzer functionality"""
        from mks_servo_can import PrecisionAnalyzer, PlaybackStats
        
        # Create test stats
        stats = PlaybackStats(
            planned_points=100,
            executed_points=100,
            average_position_error={"X": 0.05, "Y": 0.03},  # Excellent precision
            max_position_error={"X": 0.2, "Y": 0.15},
            average_timing_error=0.02,  # 20ms - good timing
            max_timing_error=0.08,
            total_duration=10.0
        )
        
        # Test precision assessment
        assessment = PrecisionAnalyzer.assess_precision(stats)
        assert assessment in ["EXCELLENT", "GOOD", "FAIR", "POOR"]
        
        # With these low errors, should be EXCELLENT
        assert assessment == "EXCELLENT"
    
    def test_utility_functions_mock(self):
        """Test utility functions can be called (mock test without real CAN)"""
        from mks_servo_can import create_linear_axis, create_rotary_axis
        
        # These functions require a real CANInterface, so we just test they exist
        # and have the right signatures
        import inspect
        
        # Check create_linear_axis signature
        sig = inspect.signature(create_linear_axis)
        assert 'can_interface' in sig.parameters
        assert 'motor_can_id' in sig.parameters
        assert 'name' in sig.parameters
        assert 'pitch_mm' in sig.parameters
        
        # Check create_rotary_axis signature  
        sig = inspect.signature(create_rotary_axis)
        assert 'can_interface' in sig.parameters
        assert 'motor_can_id' in sig.parameters
        assert 'name' in sig.parameters


class TestDigitizerModuleStructure:
    """Test the internal structure of the digitizer module"""
    
    def test_module_has_all_components(self):
        """Test that all expected components are in the digitizer module"""
        from mks_servo_can import digitizer
        
        # Check that all expected attributes exist
        expected_attrs = [
            'MotorDigitizer',
            'DigitizedPoint', 
            'DigitizedSequence',
            'PlaybackStats',
            'PrecisionAnalyzer',
            'SurfacePoint',
            'SurfaceMap',
            'EnhancedHeightMapGenerator',
            'create_linear_axis',
            'create_rotary_axis'
        ]
        
        for attr in expected_attrs:
            assert hasattr(digitizer, attr), f"Missing {attr} in digitizer module"
    
    def test_inheritance_structure(self):
        """Test that inheritance relationships are correct"""
        from mks_servo_can import MotorDigitizer, EnhancedHeightMapGenerator
        
        # Test that EnhancedHeightMapGenerator inherits from MotorDigitizer
        assert issubclass(EnhancedHeightMapGenerator, MotorDigitizer)


if __name__ == "__main__":
    pytest.main([__file__])