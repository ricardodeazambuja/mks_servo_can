# Motor Digitizer System

The Motor Digitizer system provides advanced recording and playback capabilities for MKS servo motors, enabling precision testing, manual teaching, and system calibration.

## Overview

The system consists of three main components:

1. **`motor_digitizer.py`** - Base MotorDigitizer class with recording/playback functionality
2. **`height_map_generator_v2.py`** - Enhanced height mapping that inherits from MotorDigitizer
3. **`digitizer_precision_test.py`** - Comprehensive precision testing suite

## Key Features

### 🎥 Recording Capabilities
- Record motor positions during manual movement (motors disabled)
- Configurable sample rates (1-50 Hz)
- Automatic velocity calculation
- Real-time position feedback during recording
- JSON export for data persistence

### ▶️ Playback Functionality
- High-precision playback of recorded sequences
- Variable speed playback (0.1x to 2.0x)
- Automatic timing synchronization
- Smooth trajectory following with velocity profiling

### 🎯 Precision Testing
- Compare recorded vs. playback positions
- Measure timing accuracy and position errors
- Repeatability testing with statistical analysis
- System performance assessment

### 🗺️ Enhanced Height Mapping
- Interactive manual height mapping with recording
- Automated grid-based mapping
- Precision testing of mapping accuracy
- Advanced surface analysis and statistics

## Quick Start

### Basic Recording and Playback

```python
import asyncio
from mks_servo_can import CANInterface
from motor_digitizer import MotorDigitizer, create_linear_axis

async def basic_example():
    # Setup CAN interface
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()
    
    # Create digitizer
    digitizer = MotorDigitizer(can_if)
    
    # Add axes
    x_axis = create_linear_axis(can_if, 1, "X", 40.0)  # 40mm/rev
    y_axis = create_linear_axis(can_if, 2, "Y", 40.0)
    
    await digitizer.add_axis(x_axis)
    await digitizer.add_axis(y_axis)
    await digitizer.initialize_axes()
    
    # Record manual movements (will disable motors for manual control)
    print("Move motors manually, then press Enter to stop recording...")
    await digitizer.start_recording(sample_rate=10.0)
    
    # Save recording
    digitizer.save_sequence(digitizer.current_sequence, "my_recording.json")
    
    # Play back with precision testing
    stats = await digitizer.playback_sequence(
        digitizer.current_sequence, 
        speed_factor=0.8, 
        precision_test=True
    )
    
    print(f"Average position error: {stats.average_position_error}")
    print(f"Average timing error: {stats.average_timing_error*1000:.1f}ms")
    
    await digitizer.cleanup()
    await can_if.disconnect()

asyncio.run(basic_example())
```

### Height Mapping with Recording

```python
from height_map_generator_v2 import EnhancedHeightMapGenerator

async def height_mapping_example():
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()
    
    # Create enhanced height mapper
    generator = EnhancedHeightMapGenerator(can_if)
    await generator.setup_standard_plotter([1, 2, 3])  # X, Y, Pen motors
    
    # Interactive height mapping with recording
    surface_map = await generator.interactive_height_mapping("height_session.json")
    
    # Save the surface map
    generator.save_surface_map(surface_map, "surface.json")
    
    # Test precision by replaying the recording
    stats = await generator.precision_test_from_recording("height_session.json")
    
    await generator.cleanup()
    await can_if.disconnect()
```

## Command Line Usage

### Precision Testing Suite

```bash
# Basic precision test
python digitizer_precision_test.py --test basic --save-recording basic_test.json

# Complex path precision test  
python digitizer_precision_test.py --test complex --save-results complex_results.json

# Repeatability testing
python digitizer_precision_test.py --playback recording.json --runs 5 --save-results repeatability.json

# Complete test suite
python digitizer_precision_test.py --test all --save-results complete_test.json
```

### Enhanced Height Mapping

```bash
# Interactive height mapping with recording
python height_map_generator_v2.py --interactive --save-recording session.json --save-map surface.json

# Automated grid mapping
python height_map_generator_v2.py --grid-map --spacing 10 --save-map grid_surface.json

# Precision test from recording
python height_map_generator_v2.py --playback session.json --speed-factor 0.5 --save-stats precision.json
```

## Data Formats

### DigitizedSequence JSON Structure

```json
{
  "points": [
    {
      "timestamp": 0.0,
      "positions": {"X": 0.0, "Y": 0.0, "Z": 0.0},
      "velocities": {"X": 0.0, "Y": 0.0, "Z": 0.0}
    }
  ],
  "axis_names": ["X", "Y", "Z"],
  "axis_configs": {
    "X": {
      "motor_can_id": 1,
      "kinematics_type": "LinearKinematics",
      "units": "mm"
    }
  },
  "recording_duration": 30.0,
  "sample_rate": 10.0,
  "metadata": {
    "auto_velocity": true,
    "total_points": 300
  }
}
```

### PlaybackStats Structure

```json
{
  "planned_points": 300,
  "executed_points": 298,
  "average_position_error": {
    "X": 0.05,
    "Y": 0.03,
    "Z": 0.02
  },
  "max_position_error": {
    "X": 0.25,
    "Y": 0.18,
    "Z": 0.12
  },
  "average_timing_error": 0.008,
  "max_timing_error": 0.025,
  "total_duration": 24.5
}
```

## Precision Assessment Criteria

The system automatically assesses precision performance:

| Assessment | Position Error | Max Error | Timing Error |
|------------|----------------|-----------|--------------|
| **EXCELLENT** | < 0.1mm | < 0.5mm | < 50ms |
| **GOOD** | < 0.5mm | < 2.0mm | < 100ms |
| **FAIR** | < 1.0mm | < 5.0mm | < 200ms |
| **POOR** | > 1.0mm | > 5.0mm | > 200ms |

## Use Cases

### 1. System Calibration and Validation
- Record reference movements and measure repeatability
- Test system precision under different conditions
- Validate mechanical accuracy vs. commanded positions

### 2. Manual Teaching and Path Generation
- Manually guide robot through desired paths
- Record complex trajectories that are difficult to program
- Create smooth motion profiles from manual input

### 3. Quality Assurance Testing
- Automated testing of system precision over time
- Regression testing after maintenance or modifications
- Performance benchmarking and comparison

### 4. Height Mapping and Surface Profiling
- Create detailed surface maps for compensation
- Test mapping accuracy and repeatability
- Generate calibration data for non-flat surfaces

## Architecture Details

### MotorDigitizer Base Class
- **Recording Engine**: Async recording loop with configurable sample rates
- **Playback Engine**: Precise timing control with velocity profiling
- **Precision Analysis**: Statistical analysis of position and timing errors
- **Data Management**: JSON serialization with compression support

### EnhancedHeightMapGenerator
- **Inheritance**: Extends MotorDigitizer for specialized height mapping
- **Interactive Mode**: Real-time user interface for manual mapping
- **Automated Mode**: Grid-based mapping with collision detection
- **Surface Analysis**: Statistical analysis and interpolation

### PrecisionTestSuite
- **Test Scenarios**: Basic, complex, and repeatability tests
- **Statistical Analysis**: Multi-run analysis with confidence intervals
- **Automated Assessment**: Performance scoring and recommendations

## Error Handling

The system includes comprehensive error handling:

- **Communication Errors**: CAN timeout and retry logic
- **Motor Errors**: State validation and recovery procedures
- **Recording Errors**: Data validation and corruption detection
- **Playback Errors**: Position error limits and emergency stops

## Performance Considerations

### Recording Performance
- Sample rates up to 50 Hz for smooth recordings
- Automatic adaptive sampling based on movement speed
- Memory-efficient data structures for long recordings

### Playback Performance
- Sub-millisecond timing accuracy on appropriate hardware
- Smooth velocity profiling to minimize mechanical stress
- Configurable speed factors for different accuracy requirements

## System Requirements

### Hardware
- MKS SERVO42D/57D CAN motors
- CAN interface (or simulator)
- Sufficient processing power for real-time control

### Software
- Python 3.8+
- mks-servo-can library
- numpy (for enhanced calculations)
- asyncio support

## Troubleshooting

### Common Issues

1. **High Position Errors**
   - Check mechanical backlash and play
   - Verify kinematics parameters
   - Reduce playback speed factor

2. **Timing Inconsistencies**
   - Reduce system load during recording/playback
   - Check CAN interface latency
   - Use dedicated hardware for critical applications

3. **Recording Failures**
   - Ensure motors are properly disabled during recording
   - Check CAN communication stability
   - Verify sufficient disk space for recordings

### Calibration Tips

1. **Use consistent environmental conditions** during recording and playback
2. **Allow mechanical systems to warm up** before precision testing
3. **Record multiple reference sequences** for statistical validation
4. **Test at different speeds** to characterize dynamic performance

## Future Enhancements

Planned improvements include:
- Force/torque sensing integration
- Advanced path smoothing algorithms
- Real-time collision detection
- Multi-robot coordination
- Machine learning-based error compensation

## Support

For questions, issues, or contributions related to the Motor Digitizer system, please refer to the main mks-servo-can project documentation and issue tracker.