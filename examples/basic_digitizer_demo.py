#!/usr/bin/env python3
"""
Basic MotorDigitizer Usage Example

Demonstrates core recording and playback functionality using the
library's built-in MotorDigitizer.

This example shows how to use the MotorDigitizer that is now integrated
into the main mks_servo_can library.
"""

import asyncio
import sys
import math
from pathlib import Path

# Add the library to the path
lib_path = Path(__file__).parent.parent / "mks_servo_can_library"
sys.path.insert(0, str(lib_path))

from mks_servo_can import CANInterface, MotorDigitizer, create_linear_axis


async def basic_digitizer_demo():
    """Demonstrate basic digitizer functionality"""
    print("🤖 Basic MotorDigitizer Demo")
    print("=" * 50)
    
    # Setup CAN interface with simulator
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()
    print("✅ CAN interface connected")
    
    try:
        # Create digitizer
        digitizer = MotorDigitizer(can_if)
        print("✅ MotorDigitizer created")
        
        # Add axes using the utility functions from the library
        x_axis = create_linear_axis(can_if, 1, "X", 40.0)  # 40mm/rev
        y_axis = create_linear_axis(can_if, 2, "Y", 40.0)
        z_axis = create_linear_axis(can_if, 3, "Z", 8.0)   # 8mm/rev for Z
        
        await digitizer.add_axis(x_axis)
        await digitizer.add_axis(y_axis)
        await digitizer.add_axis(z_axis)
        print("✅ Added X, Y, Z axes")
        
        # Initialize axes
        await digitizer.initialize_axes()
        print("✅ Axes initialized")
        
        # Create a simple test sequence manually (simulating a recording)
        from mks_servo_can import DigitizedPoint, DigitizedSequence
        from datetime import datetime
        
        test_points = [
            DigitizedPoint(0.0, {"X": 0.0, "Y": 0.0, "Z": 0.0}),
            DigitizedPoint(1.0, {"X": 10.0, "Y": 5.0, "Z": 2.0}),
            DigitizedPoint(2.0, {"X": 20.0, "Y": 10.0, "Z": 1.0}),
            DigitizedPoint(3.0, {"X": 10.0, "Y": 15.0, "Z": 0.0}),
            DigitizedPoint(4.0, {"X": 0.0, "Y": 10.0, "Z": 0.0}),
        ]
        
        test_sequence = DigitizedSequence(
            points=test_points,
            axis_names=["X", "Y", "Z"],
            axis_configs=digitizer._get_axis_configs(),
            recording_date=datetime.now().isoformat(),
            recording_duration=4.0,
            sample_rate=1.0,
            metadata={"demo": "basic_test"}
        )
        
        print("✅ Created test sequence with 5 points")
        
        # Save the test sequence
        test_file = "/tmp/basic_test_sequence.json"
        digitizer.save_sequence(test_sequence, test_file)
        print(f"✅ Saved test sequence to {test_file}")
        
        # Load it back to verify
        loaded_sequence = digitizer.load_sequence(test_file)
        print(f"✅ Loaded sequence: {len(loaded_sequence.points)} points")
        
        # Test playback with precision measurement
        print("\n🎯 Starting precision test playback...")
        stats = await digitizer.playback_sequence(
            loaded_sequence, 
            speed_factor=2.0,  # 2x speed for faster demo
            precision_test=True
        )
        
        if stats:
            print("\n📊 Precision Test Results:")
            print(f"   Execution rate: {stats.executed_points}/{stats.planned_points}")
            print(f"   Timing accuracy: {stats.average_timing_error*1000:.1f}ms avg")
            
            # Test the precision analyzer
            from mks_servo_can import PrecisionAnalyzer
            assessment = PrecisionAnalyzer.assess_precision(stats)
            print(f"   Overall assessment: {assessment}")
            
        # Cleanup
        await digitizer.cleanup()
        print("✅ Digitizer cleanup complete")
        
    finally:
        await can_if.disconnect()
        print("✅ CAN interface disconnected")
    
    print("\n🎉 Demo completed successfully!")
    print("\nNext steps:")
    print("- Try manual recording with: await digitizer.start_recording()")
    print("- Test with real hardware by removing use_simulator=True")
    print("- Explore EnhancedHeightMapGenerator for surface mapping")


async def precision_analyzer_demo():
    """Demonstrate precision analyzer functionality"""
    print("\n🔬 PrecisionAnalyzer Demo")
    print("=" * 30)
    
    from mks_servo_can import PlaybackStats, PrecisionAnalyzer, DigitizedSequence, DigitizedPoint
    
    # Create sample stats for different precision levels
    excellent_stats = PlaybackStats(
        planned_points=100, executed_points=100,
        average_position_error={"X": 0.05, "Y": 0.03, "Z": 0.02},
        max_position_error={"X": 0.2, "Y": 0.15, "Z": 0.1},
        average_timing_error=0.02, max_timing_error=0.05,
        total_duration=10.0
    )
    
    good_stats = PlaybackStats(
        planned_points=100, executed_points=98,
        average_position_error={"X": 0.3, "Y": 0.4, "Z": 0.2},
        max_position_error={"X": 1.5, "Y": 1.8, "Z": 1.0},
        average_timing_error=0.08, max_timing_error=0.15,
        total_duration=10.0
    )
    
    poor_stats = PlaybackStats(
        planned_points=100, executed_points=85,
        average_position_error={"X": 2.0, "Y": 1.5, "Z": 3.0},
        max_position_error={"X": 8.0, "Y": 6.0, "Z": 12.0},
        average_timing_error=0.25, max_timing_error=0.8,
        total_duration=10.0
    )
    
    # Test assessments
    test_cases = [
        ("Excellent", excellent_stats),
        ("Good", good_stats), 
        ("Poor", poor_stats)
    ]
    
    for name, stats in test_cases:
        assessment = PrecisionAnalyzer.assess_precision(stats)
        print(f"{name:10s} system: {assessment:10s} assessment")
    
    # Test path complexity
    simple_sequence = DigitizedSequence(
        points=[
            DigitizedPoint(0.0, {"X": 0.0, "Y": 0.0}),
            DigitizedPoint(1.0, {"X": 10.0, "Y": 0.0}),
        ],
        axis_names=["X", "Y"], axis_configs={},
        recording_date="", recording_duration=1.0, sample_rate=1.0
    )
    
    complex_sequence = DigitizedSequence(
        points=[
            DigitizedPoint(i*0.1, {"X": 10*math.sin(i*0.5), "Y": 10*math.cos(i*0.5)})
            for i in range(20)
        ],
        axis_names=["X", "Y"], axis_configs={},
        recording_date="", recording_duration=2.0, sample_rate=10.0
    )
    
    simple_complexity = PrecisionAnalyzer.calculate_path_complexity(simple_sequence)
    complex_complexity = PrecisionAnalyzer.calculate_path_complexity(complex_sequence)
    
    print(f"\nPath Complexity Analysis:")
    print(f"Simple path:  {simple_complexity:.3f}")
    print(f"Complex path: {complex_complexity:.3f}")


if __name__ == "__main__":
    print("Starting MotorDigitizer Library Demo...")
    print("This demonstrates the migrated MotorDigitizer functionality\n")
    
    asyncio.run(basic_digitizer_demo())
    asyncio.run(precision_analyzer_demo())
    
    print("\n" + "="*60)
    print("Migration to library complete! 🎉")
    print("The MotorDigitizer is now part of mks_servo_can library")
    print("Import with: from mks_servo_can import MotorDigitizer")
    print("="*60)