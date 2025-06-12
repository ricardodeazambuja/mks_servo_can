"""
Motor Digitizer Precision Testing Example

This script demonstrates the precision testing capabilities of the MotorDigitizer
system by recording manual movements and playing them back to measure accuracy.

Test scenarios:
1. Basic precision test - simple XY movements
2. Complex path test - circular/curved movements  
3. Multi-axis coordination test - synchronized XYZ movements
4. Repeatability test - multiple runs of the same recording

Usage:
    python digitizer_precision_test.py --test basic
    python digitizer_precision_test.py --test complex --save-results
    python digitizer_precision_test.py --playback recording.json --runs 5
"""

import asyncio
import logging
import argparse
import json
import math
import statistics
from datetime import datetime
from typing import List, Dict, Any
from pathlib import Path

from mks_servo_can import CANInterface, const, exceptions
from motor_digitizer import (
    MotorDigitizer, 
    DigitizedSequence, 
    PlaybackStats,
    create_linear_axis,
    create_rotary_axis
)

logger = logging.getLogger("DigitizerPrecisionTest")

class PrecisionTestSuite:
    """Suite of precision tests for the motor digitizer system"""
    
    def __init__(self, can_interface: CANInterface):
        self.can_interface = can_interface
        self.digitizer = MotorDigitizer(can_interface)
        self.test_results = []
        
    async def setup_test_system(self, motor_ids: List[int] = None):
        """Setup a standard test system with 3 linear axes"""
        if motor_ids is None:
            motor_ids = [1, 2, 3]
            
        # Create test axes (all linear for simplicity)
        x_axis = create_linear_axis(self.can_interface, motor_ids[0], "X", pitch_mm=40.0)
        y_axis = create_linear_axis(self.can_interface, motor_ids[1], "Y", pitch_mm=40.0) 
        z_axis = create_linear_axis(self.can_interface, motor_ids[2], "Z", pitch_mm=8.0)  # Finer pitch for Z
        
        await self.digitizer.add_axis(x_axis)
        await self.digitizer.add_axis(y_axis)
        await self.digitizer.add_axis(z_axis)
        
        await self.digitizer.initialize_axes(enable_motors=True)
        
        logger.info("Test system setup complete: X, Y, Z linear axes")
        
    async def test_basic_precision(self, save_recording: str = None) -> Dict[str, Any]:
        """
        Basic precision test with simple linear movements.
        User manually moves axes, then system plays back for comparison.
        """
        print(f"\n" + "="*70)
        print(f"🎯 BASIC PRECISION TEST")
        print(f"="*70)
        print(f"This test records simple XY movements and measures playback precision.")
        print(f"Instructions:")
        print(f"  1. Move X and Y axes manually to create a simple path")
        print(f"  2. Suggested: Square pattern or simple back-and-forth")
        print(f"  3. Recording will run for 30 seconds")
        print(f"  4. System will then play back your movements")
        print(f"-"*70)
        
        input("Press Enter when ready to start recording...")
        
        # Record manual movements
        await self.digitizer.start_recording(sample_rate=10.0)
        
        if save_recording:
            self.digitizer.save_sequence(self.digitizer.current_sequence, save_recording)
            
        # Play back with precision testing
        print(f"\nStarting precision playback...")
        stats = await self.digitizer.playback_sequence(
            self.digitizer.current_sequence, 
            speed_factor=0.8,  # Slightly slower for better accuracy
            precision_test=True
        )
        
        # Analyze results
        test_result = {
            "test_name": "basic_precision",
            "timestamp": datetime.now().isoformat(),
            "recording_duration": self.digitizer.current_sequence.recording_duration,
            "points_recorded": len(self.digitizer.current_sequence.points),
            "playback_stats": stats.__dict__ if stats else None,
            "assessment": self._assess_precision(stats) if stats else "FAILED"
        }
        
        self.test_results.append(test_result)
        return test_result
        
    async def test_complex_path(self, save_recording: str = None) -> Dict[str, Any]:
        """
        Complex path test with curved movements and speed variations.
        """
        print(f"\n" + "="*70)
        print(f"🌀 COMPLEX PATH PRECISION TEST")
        print(f"="*70)
        print(f"This test records complex curved movements and measures precision.")
        print(f"Instructions:")
        print(f"  1. Create smooth, curved movements (circles, figure-8s, etc.)")
        print(f"  2. Vary your movement speed during recording")
        print(f"  3. Use both XY positioning and Z height changes")
        print(f"  4. Recording will run for 45 seconds")
        print(f"-"*70)
        
        input("Press Enter when ready to start recording...")
        
        # Record complex movements
        await self.digitizer.start_recording(sample_rate=15.0)  # Higher sample rate for curves
        
        if save_recording:
            self.digitizer.save_sequence(self.digitizer.current_sequence, save_recording)
            
        # Play back with precision testing
        print(f"\nStarting complex path playback...")
        stats = await self.digitizer.playback_sequence(
            self.digitizer.current_sequence,
            speed_factor=0.6,  # Slower for complex movements
            precision_test=True
        )
        
        # Additional analysis for complex paths
        complexity_score = self._calculate_path_complexity(self.digitizer.current_sequence)
        
        test_result = {
            "test_name": "complex_path",
            "timestamp": datetime.now().isoformat(),
            "recording_duration": self.digitizer.current_sequence.recording_duration,
            "points_recorded": len(self.digitizer.current_sequence.points),
            "path_complexity": complexity_score,
            "playback_stats": stats.__dict__ if stats else None,
            "assessment": self._assess_precision(stats) if stats else "FAILED"
        }
        
        self.test_results.append(test_result)
        return test_result
        
    async def test_repeatability(self, recording_path: str, num_runs: int = 5) -> Dict[str, Any]:
        """
        Test repeatability by running the same recording multiple times.
        """
        print(f"\n" + "="*70)
        print(f"🔄 REPEATABILITY TEST")
        print(f"="*70)
        print(f"Testing repeatability with {num_runs} runs of recorded sequence")
        print(f"Recording: {recording_path}")
        print(f"-"*70)
        
        # Load recording
        sequence = self.digitizer.load_sequence(recording_path)
        
        run_stats = []
        
        for run in range(num_runs):
            print(f"\n🏃 Run {run + 1}/{num_runs}")
            
            try:
                stats = await self.digitizer.playback_sequence(
                    sequence,
                    speed_factor=1.0,
                    precision_test=True
                )
                
                if stats:
                    run_stats.append(stats)
                    print(f"   ✅ Run {run + 1} complete")
                else:
                    print(f"   ❌ Run {run + 1} failed")
                    
            except Exception as e:
                logger.error(f"Run {run + 1} failed: {e}")
                
            # Small delay between runs
            await asyncio.sleep(2.0)
            
        # Analyze repeatability
        repeatability_analysis = self._analyze_repeatability(run_stats)
        
        test_result = {
            "test_name": "repeatability",
            "timestamp": datetime.now().isoformat(),
            "recording_file": recording_path,
            "num_runs": num_runs,
            "successful_runs": len(run_stats),
            "individual_runs": [stats.__dict__ for stats in run_stats],
            "repeatability_analysis": repeatability_analysis,
            "assessment": self._assess_repeatability(repeatability_analysis)
        }
        
        self.test_results.append(test_result)
        return test_result
        
    def _assess_precision(self, stats: PlaybackStats) -> str:
        """Assess precision performance"""
        if not stats:
            return "FAILED"
            
        # Calculate overall position error
        total_error = sum(stats.average_position_error.values())
        max_error = max(stats.max_position_error.values())
        
        # Timing assessment
        timing_error_ms = stats.average_timing_error * 1000
        
        # Scoring criteria
        if total_error < 0.1 and max_error < 0.5 and timing_error_ms < 50:
            return "EXCELLENT"
        elif total_error < 0.5 and max_error < 2.0 and timing_error_ms < 100:
            return "GOOD"
        elif total_error < 1.0 and max_error < 5.0 and timing_error_ms < 200:
            return "FAIR"
        else:
            return "POOR"
            
    def _calculate_path_complexity(self, sequence: DigitizedSequence) -> float:
        """Calculate a complexity score for the recorded path"""
        if len(sequence.points) < 3:
            return 0.0
            
        # Calculate path length and curvature
        total_distance = 0.0
        direction_changes = 0.0
        
        for i in range(1, len(sequence.points)):
            prev_point = sequence.points[i-1]
            curr_point = sequence.points[i]
            
            # Calculate distance between points
            dx = curr_point.positions.get("X", 0) - prev_point.positions.get("X", 0)
            dy = curr_point.positions.get("Y", 0) - prev_point.positions.get("Y", 0)
            dz = curr_point.positions.get("Z", 0) - prev_point.positions.get("Z", 0)
            
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += distance
            
            # Calculate direction change (simplified curvature measure)
            if i >= 2:
                prev_prev = sequence.points[i-2]
                
                # Vector from prev_prev to prev
                v1x = prev_point.positions.get("X", 0) - prev_prev.positions.get("X", 0)
                v1y = prev_point.positions.get("Y", 0) - prev_prev.positions.get("Y", 0)
                
                # Vector from prev to curr
                v2x = curr_point.positions.get("X", 0) - prev_point.positions.get("X", 0)
                v2y = curr_point.positions.get("Y", 0) - prev_point.positions.get("Y", 0)
                
                # Calculate angle between vectors
                dot_product = v1x*v2x + v1y*v2y
                mag1 = math.sqrt(v1x*v1x + v1y*v1y)
                mag2 = math.sqrt(v2x*v2x + v2y*v2y)
                
                if mag1 > 0 and mag2 > 0:
                    cos_angle = dot_product / (mag1 * mag2)
                    cos_angle = max(-1, min(1, cos_angle))  # Clamp to valid range
                    angle = math.acos(cos_angle)
                    direction_changes += angle
                    
        # Complexity score combines path length and curvature
        if len(sequence.points) > 1:
            complexity = (total_distance / 100.0) + (direction_changes / len(sequence.points))
        else:
            complexity = 0.0
            
        return complexity
        
    def _analyze_repeatability(self, run_stats: List[PlaybackStats]) -> Dict[str, Any]:
        """Analyze repeatability across multiple runs"""
        if not run_stats:
            return {"error": "No successful runs"}
            
        # Extract metrics from all runs
        position_errors = {axis: [] for axis in run_stats[0].average_position_error.keys()}
        timing_errors = [stats.average_timing_error for stats in run_stats]
        
        for stats in run_stats:
            for axis, error in stats.average_position_error.items():
                position_errors[axis].append(error)
                
        # Calculate statistics
        analysis = {
            "num_runs": len(run_stats),
            "timing_repeatability": {
                "mean": statistics.mean(timing_errors),
                "std_dev": statistics.stdev(timing_errors) if len(timing_errors) > 1 else 0.0,
                "min": min(timing_errors),
                "max": max(timing_errors)
            },
            "position_repeatability": {}
        }
        
        for axis, errors in position_errors.items():
            if errors:
                analysis["position_repeatability"][axis] = {
                    "mean": statistics.mean(errors),
                    "std_dev": statistics.stdev(errors) if len(errors) > 1 else 0.0,
                    "min": min(errors),
                    "max": max(errors)
                }
                
        return analysis
        
    def _assess_repeatability(self, analysis: Dict[str, Any]) -> str:
        """Assess repeatability performance"""
        if "error" in analysis:
            return "FAILED"
            
        # Check timing repeatability
        timing_std = analysis["timing_repeatability"]["std_dev"]
        
        # Check position repeatability
        position_stds = []
        for axis_data in analysis["position_repeatability"].values():
            position_stds.append(axis_data["std_dev"])
            
        avg_position_std = statistics.mean(position_stds) if position_stds else float('inf')
        
        # Assessment criteria
        if timing_std < 0.01 and avg_position_std < 0.05:
            return "EXCELLENT"
        elif timing_std < 0.05 and avg_position_std < 0.2:
            return "GOOD"
        elif timing_std < 0.1 and avg_position_std < 0.5:
            return "FAIR"
        else:
            return "POOR"
            
    def save_test_results(self, filepath: str):
        """Save all test results to JSON file"""
        results_data = {
            "test_session": {
                "timestamp": datetime.now().isoformat(),
                "total_tests": len(self.test_results)
            },
            "results": self.test_results
        }
        
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        with open(filepath, 'w') as f:
            json.dump(results_data, f, indent=2)
            
        logger.info(f"Test results saved to {filepath}")
        
    def print_summary(self):
        """Print a summary of all test results"""
        print(f"\n" + "="*70)
        print(f"📊 TEST RESULTS SUMMARY")
        print(f"="*70)
        
        if not self.test_results:
            print("No tests completed.")
            return
            
        for i, result in enumerate(self.test_results, 1):
            print(f"\n{i}. {result['test_name'].upper()}")
            print(f"   Assessment: {result['assessment']}")
            
            if result['test_name'] == 'repeatability':
                print(f"   Runs: {result['successful_runs']}/{result['num_runs']}")
            else:
                if result.get('playback_stats'):
                    stats = result['playback_stats']
                    avg_error = sum(stats['average_position_error'].values())
                    print(f"   Avg position error: {avg_error:.3f}")
                    print(f"   Avg timing error: {stats['average_timing_error']*1000:.1f}ms")
        
        # Overall assessment
        assessments = [r['assessment'] for r in self.test_results]
        excellent_count = assessments.count('EXCELLENT')
        good_count = assessments.count('GOOD')
        total_tests = len(assessments)
        
        print(f"\nOverall System Assessment:")
        if excellent_count >= total_tests * 0.8:
            print("🟢 EXCELLENT - System ready for high-precision work")
        elif (excellent_count + good_count) >= total_tests * 0.7:
            print("🟡 GOOD - System suitable for most applications") 
        else:
            print("🔴 NEEDS IMPROVEMENT - Consider recalibration")

async def main():
    """Main test application"""
    args = parse_arguments()
    
    # Setup logging
    log_level = getattr(logging, args.log_level.upper())
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    
    can_if = None
    test_suite = None
    
    try:
        # Setup CAN interface
        can_if = CANInterface()
        await can_if.connect()
        logger.info("CAN Interface connected")
        
        # Create test suite
        test_suite = PrecisionTestSuite(can_if)
        await test_suite.setup_test_system(args.motor_ids)
        
        # Run requested tests
        if args.test == 'basic':
            await test_suite.test_basic_precision(args.save_recording)
            
        elif args.test == 'complex':
            await test_suite.test_complex_path(args.save_recording)
            
        elif args.playback:
            await test_suite.test_repeatability(args.playback, args.runs)
            
        elif args.test == 'all':
            print("Running complete test suite...")
            await test_suite.test_basic_precision()
            await asyncio.sleep(2)
            await test_suite.test_complex_path()
            
        # Save results if requested
        if args.save_results:
            test_suite.save_test_results(args.save_results)
            
        # Print summary
        test_suite.print_summary()
        
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
    finally:
        if test_suite:
            await test_suite.digitizer.cleanup()
        if can_if and can_if.is_connected:
            await can_if.disconnect()

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Motor Digitizer Precision Testing")
    
    # Test selection
    test_group = parser.add_mutually_exclusive_group(required=True)
    test_group.add_argument('--test', choices=['basic', 'complex', 'all'],
                           help='Run specific test type')
    test_group.add_argument('--playback', type=str, metavar='RECORDING_FILE',
                           help='Test repeatability with existing recording')
    
    # Recording options
    parser.add_argument('--save-recording', type=str, metavar='FILE',
                       help='Save test recording to file')
    parser.add_argument('--save-results', type=str, metavar='FILE',
                       help='Save test results to file')
    
    # Repeatability test options
    parser.add_argument('--runs', type=int, default=5,
                       help='Number of repeatability test runs (default: 5)')
    
    # Hardware config
    parser.add_argument('--motor-ids', nargs=3, type=int, default=[1, 2, 3],
                       metavar=('X', 'Y', 'Z'), help='Motor CAN IDs for test axes')
    
    # System options
    parser.add_argument('--log-level', default='INFO',
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    
    return parser.parse_args()

if __name__ == "__main__":
    asyncio.run(main())