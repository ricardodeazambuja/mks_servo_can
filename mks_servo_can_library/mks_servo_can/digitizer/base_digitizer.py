"""
Base Motor Digitizer Class

This module provides the core MotorDigitizer class for recording motor positions
during manual movement and playing back recorded sequences with precision testing.
"""

import asyncio
import logging
import json
import time
import select
import sys
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple, Union
from dataclasses import asdict
from pathlib import Path

from .. import (
    CANInterface,
    Axis,
    MultiAxisController,
    exceptions,
)

from .data_structures import DigitizedPoint, DigitizedSequence, PlaybackStats

logger = logging.getLogger("MotorDigitizer")


class MotorDigitizer:
    """
    Base class for multi-motor digitizing operations.
    
    Supports recording motor positions during manual movement and
    playing back recorded sequences with precision testing capabilities.
    """
    
    def __init__(self, can_interface: CANInterface):
        """
        Initialize the digitizer with a CAN interface.
        
        Args:
            can_interface: Connected CANInterface instance
        """
        self.can_interface = can_interface
        self.controller: Optional[MultiAxisController] = None
        self.axes: Dict[str, Axis] = {}
        self.is_recording = False
        self.current_sequence: Optional[DigitizedSequence] = None
        self._recording_start_time = 0.0
        self._recording_points: List[DigitizedPoint] = []
        self._stop_recording = False
        
    async def add_axis(self, axis: Axis) -> None:
        """
        Add an axis to the digitizer.
        
        Args:
            axis: Configured Axis instance
        """
        if self.controller is None:
            self.controller = MultiAxisController(self.can_interface)
            
        self.controller.add_axis(axis)
        self.axes[axis.name] = axis
        logger.info(f"Added axis '{axis.name}' (CAN ID: {axis.can_id})")
        
    async def initialize_axes(self, enable_motors: bool = True) -> None:
        """
        Initialize all added axes.
        
        Args:
            enable_motors: Whether to enable motors after initialization
        """
        if not self.controller:
            raise ValueError("No axes added to digitizer")
            
        await self.controller.initialize_all_axes(calibrate=False, home=False)
        
        if enable_motors:
            await self.controller.enable_all_axes()
            
        # Set current positions as zero for all axes
        for axis in self.axes.values():
            await axis.set_current_position_as_zero()
            
        logger.info(f"Initialized {len(self.axes)} axes: {list(self.axes.keys())}")
        
    async def start_recording(self, sample_rate: float = 10.0, 
                            auto_velocity: bool = True) -> None:
        """
        Start recording motor positions.
        
        Args:
            sample_rate: Recording sample rate in Hz
            auto_velocity: Whether to automatically calculate velocities
        """
        if self.is_recording:
            raise RuntimeError("Recording already in progress")
            
        if not self.axes:
            raise ValueError("No axes configured for recording")
            
        # Disable all motors for manual movement
        await self.controller.disable_all_axes()
        
        self.is_recording = True
        self._recording_start_time = time.time()
        self._recording_points = []
        self._stop_recording = False
        
        print(f"\n" + "="*70)
        print(f"🎥 RECORDING STARTED")
        print(f"="*70)
        print(f"Sample rate: {sample_rate} Hz")
        print(f"Recording axes: {', '.join(self.axes.keys())}")
        print(f"Motors are DISABLED - move freely by hand")
        print(f"Commands:")
        print(f"  • Press Enter to STOP recording")
        print(f"  • Move motors manually to desired positions")
        print(f"  • Real-time position feedback shown below")
        print(f"-"*70)
        
        # Start recording task
        recording_task = asyncio.create_task(
            self._recording_loop(sample_rate, auto_velocity)
        )
        
        # Wait for user input to stop recording
        try:
            await asyncio.get_event_loop().run_in_executor(
                None, input, "Press Enter to stop recording..."
            )
            self._stop_recording = True
            await recording_task
            
        except KeyboardInterrupt:
            self._stop_recording = True
            await recording_task
            
        finally:
            self.is_recording = False
            await self.controller.enable_all_axes()
            
        # Create sequence object
        recording_duration = time.time() - self._recording_start_time
        
        self.current_sequence = DigitizedSequence(
            points=self._recording_points.copy(),
            axis_names=list(self.axes.keys()),
            axis_configs=self._get_axis_configs(),
            recording_date=datetime.now().isoformat(),
            recording_duration=recording_duration,
            sample_rate=sample_rate,
            metadata={
                "auto_velocity": auto_velocity,
                "total_points": len(self._recording_points)
            }
        )
        
        print(f"\n✅ RECORDING COMPLETE")
        print(f"   Duration: {recording_duration:.1f} seconds")
        print(f"   Points recorded: {len(self._recording_points)}")
        print(f"   Average sample rate: {len(self._recording_points)/recording_duration:.1f} Hz")
        
    async def _recording_loop(self, sample_rate: float, auto_velocity: bool) -> None:
        """Internal recording loop"""
        sample_interval = 1.0 / sample_rate
        last_positions = {}
        last_timestamp = 0.0
        update_counter = 0
        
        while not self._stop_recording:
            current_time = time.time()
            timestamp = current_time - self._recording_start_time
            
            # Read current positions from all axes
            positions = {}
            try:
                for axis_name, axis in self.axes.items():
                    positions[axis_name] = await axis.get_current_position_user()
            except Exception as e:
                logger.warning(f"Error reading positions: {e}")
                await asyncio.sleep(sample_interval)
                continue
                
            # Calculate velocities if requested
            velocities = None
            if auto_velocity and last_positions and last_timestamp > 0:
                dt = timestamp - last_timestamp
                if dt > 0:
                    velocities = {}
                    for axis_name in positions:
                        if axis_name in last_positions:
                            dp = positions[axis_name] - last_positions[axis_name]
                            velocities[axis_name] = dp / dt
                            
            # Create and store point
            point = DigitizedPoint(
                timestamp=timestamp,
                positions=positions.copy(),
                velocities=velocities
            )
            self._recording_points.append(point)
            
            # Update display periodically
            update_counter += 1
            if update_counter % max(1, int(sample_rate / 5)) == 0:  # Update 5 times per second max
                self._display_recording_status(timestamp, positions, velocities)
                
            # Store for next iteration
            last_positions = positions.copy()
            last_timestamp = timestamp
            
            # Wait for next sample
            await asyncio.sleep(sample_interval)
            
    def _display_recording_status(self, timestamp: float, positions: Dict[str, float], 
                                velocities: Optional[Dict[str, float]]) -> None:
        """Display current recording status"""
        status_lines = []
        status_lines.append(f"⏱️  Time: {timestamp:6.1f}s | Points: {len(self._recording_points):4d}")
        
        for axis_name, position in positions.items():
            vel_str = ""
            if velocities and axis_name in velocities:
                vel_str = f" | Vel: {velocities[axis_name]:7.1f}"
            status_lines.append(f"📐 {axis_name:8s}: {position:8.2f}{vel_str}")
            
        # Clear lines and redraw
        for i in range(len(status_lines)):
            if i > 0:
                print("\033[F\033[K", end="")  # Move up and clear line
        for line in status_lines:
            print(f"{line:<80}")
            
    async def playback_sequence(self, sequence: DigitizedSequence, 
                              speed_factor: float = 1.0,
                              precision_test: bool = False) -> Optional[PlaybackStats]:
        """
        Play back a recorded sequence.
        
        Args:
            sequence: DigitizedSequence to play back
            speed_factor: Playback speed multiplier (1.0 = original speed)
            precision_test: Whether to measure precision against original positions
            
        Returns:
            PlaybackStats if precision_test=True, None otherwise
        """
        if not self.axes:
            raise ValueError("No axes configured for playback")
            
        if self.is_recording:
            raise RuntimeError("Cannot playback during recording")
            
        # Validate sequence compatibility
        missing_axes = set(sequence.axis_names) - set(self.axes.keys())
        if missing_axes:
            raise ValueError(f"Sequence requires axes not available: {missing_axes}")
            
        print(f"\n" + "="*70)
        print(f"▶️  PLAYBACK STARTING")
        print(f"="*70)
        print(f"Sequence: {len(sequence.points)} points, {sequence.recording_duration:.1f}s")
        print(f"Speed factor: {speed_factor}x")
        print(f"Precision test: {'Enabled' if precision_test else 'Disabled'}")
        print(f"Axes: {', '.join(sequence.axis_names)}")
        print(f"-"*70)
        
        start_time = time.time()
        executed_points = 0
        position_errors = {axis: [] for axis in sequence.axis_names}
        timing_errors = []
        
        try:
            for i, point in enumerate(sequence.points):
                target_time = start_time + (point.timestamp / speed_factor)
                current_time = time.time()
                
                # Wait until target time
                sleep_time = target_time - current_time
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                    
                # Move to target positions
                positions_to_move = {
                    axis_name: position 
                    for axis_name, position in point.positions.items()
                    if axis_name in self.axes
                }
                
                if positions_to_move:
                    # Calculate speeds based on time to next point
                    speeds = {}
                    if i < len(sequence.points) - 1:
                        next_point = sequence.points[i + 1]
                        dt = (next_point.timestamp - point.timestamp) / speed_factor
                        for axis_name, next_pos in next_point.positions.items():
                            if axis_name in positions_to_move and dt > 0:
                                distance = abs(next_pos - point.positions[axis_name])
                                speeds[axis_name] = min(distance / dt, 100.0)  # Cap at reasonable speed
                    
                    # Execute movement (non-blocking for smoother playback)
                    await self.controller.move_all_to_positions_abs_user(
                        positions_user=positions_to_move,
                        speeds_user=speeds if speeds else None,
                        wait_for_all=False
                    )
                    
                executed_points += 1
                
                # Precision testing
                if precision_test:
                    # Small delay to let movement settle
                    await asyncio.sleep(0.05)
                    
                    # Read actual positions
                    actual_positions = await self.controller.get_all_positions_user()
                    actual_time = time.time()
                    
                    # Calculate errors
                    timing_error = abs(actual_time - target_time)
                    timing_errors.append(timing_error)
                    
                    for axis_name in sequence.axis_names:
                        if axis_name in actual_positions and axis_name in point.positions:
                            pos_error = abs(actual_positions[axis_name] - point.positions[axis_name])
                            position_errors[axis_name].append(pos_error)
                            
                # Progress display
                if i % max(1, len(sequence.points) // 20) == 0:  # Update 20 times max
                    progress = (i + 1) / len(sequence.points) * 100
                    print(f"▶️  Progress: {progress:5.1f}% ({i+1}/{len(sequence.points)} points)")
                    
        except KeyboardInterrupt:
            print(f"\n⚠️  Playback interrupted by user")
        except Exception as e:
            logger.error(f"Error during playback: {e}")
            
        finally:
            # Stop all movements
            try:
                await self.controller.stop_all_axes()
            except:
                pass
                
        total_duration = time.time() - start_time
        
        print(f"\n✅ PLAYBACK COMPLETE")
        print(f"   Duration: {total_duration:.1f}s")
        print(f"   Points executed: {executed_points}/{len(sequence.points)}")
        
        # Generate statistics if precision testing
        if precision_test and executed_points > 0:
            stats = PlaybackStats(
                planned_points=len(sequence.points),
                executed_points=executed_points,
                average_position_error={
                    axis: sum(errors) / len(errors) if errors else 0.0
                    for axis, errors in position_errors.items()
                },
                max_position_error={
                    axis: max(errors) if errors else 0.0
                    for axis, errors in position_errors.items()
                },
                average_timing_error=sum(timing_errors) / len(timing_errors) if timing_errors else 0.0,
                max_timing_error=max(timing_errors) if timing_errors else 0.0,
                total_duration=total_duration
            )
            
            self._display_precision_stats(stats)
            return stats
            
        return None
        
    def _display_precision_stats(self, stats: PlaybackStats) -> None:
        """Display precision testing statistics"""
        print(f"\n📊 PRECISION TEST RESULTS")
        print(f"="*70)
        print(f"Execution: {stats.executed_points}/{stats.planned_points} points "
              f"({stats.executed_points/stats.planned_points*100:.1f}%)")
        print(f"Duration: {stats.total_duration:.1f}s")
        print(f"\nTiming Accuracy:")
        print(f"  Average error: {stats.average_timing_error*1000:.1f}ms")
        print(f"  Maximum error: {stats.max_timing_error*1000:.1f}ms")
        print(f"\nPosition Accuracy:")
        for axis_name in stats.average_position_error:
            avg_err = stats.average_position_error[axis_name]
            max_err = stats.max_position_error[axis_name]
            print(f"  {axis_name:10s}: Avg {avg_err:.3f}, Max {max_err:.3f}")
            
    def _get_axis_configs(self) -> Dict[str, Dict[str, Any]]:
        """Get configuration data for all axes"""
        configs = {}
        for axis_name, axis in self.axes.items():
            configs[axis_name] = {
                "motor_can_id": axis.can_id,
                "kinematics_type": type(axis.kinematics).__name__,
                "kinematics_info": axis.kinematics.get_parameters(),
                "units": axis.kinematics.units
            }
        return configs
        
    def save_sequence(self, sequence: DigitizedSequence, filepath: str) -> None:
        """
        Save a digitized sequence to a JSON file.
        
        Args:
            sequence: DigitizedSequence to save
            filepath: Output file path
        """
        # Convert to serializable format
        data = {
            "points": [asdict(point) for point in sequence.points],
            "axis_names": sequence.axis_names,
            "axis_configs": sequence.axis_configs,
            "recording_date": sequence.recording_date,
            "recording_duration": sequence.recording_duration,
            "sample_rate": sequence.sample_rate,
            "metadata": sequence.metadata or {}
        }
        
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
            
        logger.info(f"Sequence saved to {filepath}")
        
    def load_sequence(self, filepath: str) -> DigitizedSequence:
        """
        Load a digitized sequence from a JSON file.
        
        Args:
            filepath: Input file path
            
        Returns:
            DigitizedSequence object
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        # Convert points back to dataclass objects
        points = []
        for point_data in data["points"]:
            points.append(DigitizedPoint(**point_data))
            
        sequence = DigitizedSequence(
            points=points,
            axis_names=data["axis_names"],
            axis_configs=data["axis_configs"],
            recording_date=data["recording_date"],
            recording_duration=data["recording_duration"],
            sample_rate=data["sample_rate"],
            metadata=data.get("metadata", {})
        )
        
        logger.info(f"Sequence loaded from {filepath}: {len(points)} points, "
                   f"{sequence.recording_duration:.1f}s")
        
        return sequence
        
    async def cleanup(self) -> None:
        """Clean up resources"""
        if self.is_recording:
            self._stop_recording = True
            await asyncio.sleep(0.5)  # Allow recording loop to finish
            
        if self.controller:
            try:
                await self.controller.disable_all_axes()
            except Exception as e:
                logger.warning(f"Error disabling axes during cleanup: {e}")