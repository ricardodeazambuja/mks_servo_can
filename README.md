<p align="center">
  <img src="https://github.com/user-attachments/assets/4723e62f-edda-4f40-9573-5d56d7378ed7" width="400"/>
</p>


# MKS Servo CAN Control Project (WIP)
This project provides a Python library (`mks-servo-can`) for controlling MKS SERVO42D and MKS SERVO57D motors via a CAN bus interface, and a command-line simulator (`mks-servo-simulator`) for testing and development without physical hardware. The system is designed with `asyncio` for asynchronous operations, enabling efficient handling of I/O and motor communications.

**Key Reference:** The functionality and command implementations are primarily based on the "MKS SERVO42D/57D_CAN User Manual V1.0.6".

## Key Features

### Python Library (`mks-servo-can`)
* **Asynchronous Operations**: Built with `asyncio` for non-blocking communication and control.
* **Low-Level API**: Direct implementation of CAN commands as specified in the MKS user manual.
* **High-Level API**:
    * `Axis` class for intuitive control of individual motors.
    * `MultiAxisController` for managing and coordinating groups of motors.
* **Kinematics Engine**:
    * Convert between user-defined units (e.g., mm, degrees) and motor encoder/pulse values.
    * Includes `LinearKinematics`, `RotaryKinematics`, and a base for custom kinematics (e.g., `EccentricKinematics`).
    * **Robot Kinematics**: Includes `RobotModelBase` and implementations for common robot types like `TwoLinkArmPlanar`, `CartesianRobot`, and `RRRArm` found in `mks_servo_can.robot_kinematics`. These allow for controlling multi-axis robots in task space (e.g., Cartesian coordinates).
* **Motor Digitizer System**:
    * Record motor positions during manual movement (motors disabled).
    * High-precision playback with timing synchronization and statistical analysis.
    * Precision testing with automated assessment (EXCELLENT/GOOD/FAIR/POOR).
    * Surface mapping and height profiling capabilities for complex geometries.
    * Advanced plotting capabilities including SVG rendering and calligraphy.
* **Hardware & Simulator Support**: Can connect to real MKS servo motors via various `python-can` compatible interfaces or to the provided `mks-servo-simulator`.
* **Robust Error Handling**: Custom exceptions for clear diagnostics of communication, motor, and configuration issues.

### CLI Simulator (`mks-servo-simulator`)
* **Hardware-less Development**: Test the `mks-servo-can` library without physical motors.
* **Motor Behavior Modeling**: Simulates multiple MKS servo motors, responding to CAN commands by updating internal states (position, speed, etc.).
* **Virtual CAN Bus**: Emulates CAN bus interactions, managing communication with the library via a TCP socket.
* **Rich Interactive Dashboard**: Modern text-based interface with real-time motor status display.
    * Live motor position, speed, encoder values, and status indicators.
    * Interactive keyboard controls for motor selection and direct command injection.
    * Performance monitoring with latency tracking, throughput metrics, and connection health.
    * Color-coded status indicators and auto-refreshing displays.
* **Configuration Management**: 
    * Save/load configuration profiles for different simulator setups.
    * Motor templates for quick motor configuration (SERVO42D, SERVO57D, high-precision, high-speed).
    * Live parameter adjustment during runtime without restart.
    * Configuration file management with automatic persistence.
* **Advanced Debugging Tools**:
    * **LLM-Friendly Interface**: JSON output mode and HTTP API for Claude Code integration.
    * **Command Injection**: Direct command injection with raw hex commands or pre-defined templates.
    * **Test Scenarios**: Pre-built test sequences for validation and debugging.
    * **HTTP REST API**: Programmatic access to simulator state, command injection, and configuration management.
* **Performance Monitoring**:
    * Real-time latency tracking with percentiles (P95, P99) and histograms.
    * Throughput metrics, error rates, and success statistics.
    * Memory usage monitoring and connection health tracking.
    * Historical performance data and trend analysis.
* **Configurable Parameters**:
    * Number of simulated motors and their CAN IDs.
    * Simulated CAN bus latency to mimic real-world delays.
    * Motor type and basic characteristics (e.g., steps per revolution).
    * Individual motor current limits, speed limits, and position constraints.
* **TCP Socket Interface**: The library connects to the simulator via a TCP socket (default: `localhost:6789`).

### General
* **Determinism Focus**: Designed with considerations for analyzing and understanding timing behavior, aiding in applications with real-time constraints.
* **Comprehensive Test Strategy**: Includes unit tests, integration tests (simulator-based), and examples for Hardware-In-the-Loop (HIL) and determinism/timing benchmarks.

## Project Structure

The project is organized into two main Python packages and supporting directories:

```text
mks_servo_can/
├── mks_servo_can_library/           # The installable Python library (mks_servo_can)
│   ├── mks_servo_can/               # Source code for the library
│   │   ├── kinematics/              # Kinematic transformation modules
│   │   ├── digitizer/               # Motor digitizing and precision testing
│   │   │   ├── __init__.py
│   │   │   ├── base_digitizer.py    # Core MotorDigitizer class
│   │   │   ├── data_structures.py   # DigitizedPoint, DigitizedSequence, etc.
│   │   │   ├── precision_analyzer.py # Statistical analysis and assessment
│   │   │   ├── surface_mapping.py   # Enhanced height mapping capabilities
│   │   │   └── utils.py             # Utility functions (create_linear_axis, etc.)
│   │   ├── __init__.py
│   │   ├── can_interface.py         # Handles real CAN and simulator connection
│   │   ├── low_level_api.py         # Implements MKS CAN commands
│   │   ├── axis.py                  # High-level single motor control
│   │   ├── multi_axis_controller.py # High-level multi-motor control
│   │   ├── robot_kinematics.py      # High-level robot model kinematics
│   │   ├── constants.py
│   │   ├── crc.py
│   │   └── exceptions.py
│   └── setup.py                     # Packaging script for the library
├── mks_servo_simulator/             # The CLI simulator (mks_servo_simulator)
│   ├── mks_simulator/               # Source code for the simulator
│   │   ├── init.py
│   │   ├── cli.py                   # Command-line interface (using Click)
│   │   ├── motor_model.py           # Simulates individual motor behavior
│   │   ├── virtual_can_bus.py       # Manages simulated CAN traffic
│   │   ├── interface/               # User interface modules ✅ NEW
│   │   │   ├── __init__.py
│   │   │   ├── rich_dashboard.py    # Rich console dashboard with real-time display
│   │   │   ├── interactive_controls.py # Keyboard controls and user interaction
│   │   │   ├── config_manager.py    # Configuration profiles and live parameter adjustment
│   │   │   ├── debug_tools.py       # Command injection and testing framework
│   │   │   ├── performance_monitor.py # Performance tracking and monitoring
│   │   │   ├── llm_debug_interface.py # LLM-friendly debugging interface
│   │   │   └── http_debug_server.py # HTTP REST API for programmatic access
│   │   └── main.py                  # Entry point for the simulator CLI
│   └── setup.py                     # Packaging script for the simulator
├── tests/                           # Unit, integration, HIL, determinism tests
│   ├── unit/
│   ├── integration/
│   ├── hil/
│   └── determinism/
├── examples/                        # Example scripts demonstrating library usage
│   ├── single_axis_real_hw.py       # Basic single motor control with real hardware
│   ├── multi_axis_simulator.py      # Multi-motor control with simulator
│   ├── advanced_axis_control.py     # Advanced motor control techniques
│   ├── sync_mks_axis.py             # Example of a synchronous wrapper
│   ├── benchmark_command_latency.py # Performance benchmarking and timing analysis
│   ├── two_link_planar_arm.py       # Example using TwoLinkArmPlanar robot model
│   ├── cartesian_3dof_robot.py      # Example using CartesianRobot model
│   ├── three_link_arm.py            # Example using RRRArm model
│   ├── basic_digitizer_demo.py      # MotorDigitizer library integration demo
│   ├── motor_digitizer.py           # Advanced digitizing with recording/playback
│   ├── digitizer_precision_test.py  # Comprehensive precision testing suite
│   ├── height_map_generator.py      # Basic surface height mapping
│   ├── height_map_generator_v2.py   # Enhanced surface mapping with digitizer
│   ├── svg_plotter.py               # SVG file plotting capabilities
│   ├── enhanced_svg_plotter.py      # Advanced SVG plotting with optimization
│   ├── calligraphy_plotter.py       # Artistic calligraphy and text rendering
│   └── calligraphy_plotter_manual_interpolation.py # Manual interpolation techniques
├── docs/                            # Detailed documentation
│   └── README.md                    # Overview of documentation structure
│   └── (other .md files for specific sections)
├── README.md                        # Main project README (this file)
├── requirements.txt                 # Core Python dependencies
└── LICENSE.txt                      # Project license
```

## Getting Started

### Prerequisites

* Python 3.8 or higher.
* It is highly recommended to use a Python virtual environment.

**For real hardware interaction:**
* `python-can` library: `pip install python-can`
* A `python-can` compatible CAN adapter (e.g., CANable, Kvaser, PCAN, SocketCAN compatible device).
* Appropriate drivers and system configuration for your CAN adapter (e.g., `slcand` for serial-line CAN adapters, `gs_usb` kernel module for CANable).
* **Important Hardware Notes:**
    * Ensure correct CAN bus termination (typically 120 Ohms at each end of the bus).
    * Verify proper motor power supply.
    * Double-check CAN H and CAN L wiring.

**For the simulator CLI:**
* `click` library: `pip install click`

**For enhanced simulator features (✅ NEW):**
* `rich` library: `pip install rich` (for interactive dashboard and real-time displays)
* `fastapi` and `uvicorn`: `pip install fastapi uvicorn` (for HTTP debug API)
* `psutil`: `pip install psutil` (optional, for advanced performance monitoring)

**For enhanced digitizer features (optional):**
* `numpy` library: `pip install numpy` (for advanced surface calculations)
* `matplotlib` library: `pip install matplotlib` (for visualization capabilities)

You can install all core dependencies by running:
```bash
pip install -r requirements.txt
```

### Installation

**1. MKS Servo CAN Library (`mks-servo-can`)**

It's recommended to install the library in editable mode for development. From the project root directory:
```bash
cd mks_servo_can_library
pip install -e .
```
For a standard installation:
```bash
pip install .
```

**2. MKS Servo Simulator (`mks-servo-simulator`)**

Similarly, install the simulator in editable mode for development. From the project root directory:
```bash
cd mks_servo_simulator
pip install -e .
```
This makes the `mks-servo-simulator` command available in your environment. For a standard installation:
```bash
pip install .
```

**Development Note on `PYTHONPATH`:**
If you are developing and have not installed the library in a way that's discoverable by the simulator (e.g., not using `pip install -e .` for the library, or complex project structures), you might need to adjust your `PYTHONPATH`. For instance, when running the simulator from the project root, you might use:
`PYTHONPATH=$(pwd) mks-servo-simulator [options]`
Using editable installs (`pip install -e .`) for both packages within the same virtual environment is generally the smoothest approach for development.

## Usage

### 1. Running the Simulator

**Basic Simulator Usage:**
```bash
# Start basic simulator with two motors
mks-servo-simulator --num-motors 2 --start-can-id 1 --latency-ms 5
```

**Rich Interactive Dashboard (✅ NEW):**
```bash
# Start with interactive dashboard and real-time monitoring
mks-servo-simulator --dashboard --num-motors 3 --refresh-rate 200
```
Interactive controls include:
- **Arrow keys**: Select motors  
- **Space**: Pause/resume updates
- **h**: Show help  
- **i**: Command injection mode
- **p**: Configuration profiles
- **l**: Live parameter adjustment
- **+/-**: Adjust refresh rate

**Debug API with Configuration Management (✅ NEW):**
```bash
# Start with HTTP API for programmatic access
mks-servo-simulator --debug-api --dashboard --num-motors 2
# API available at: http://localhost:8765/docs
# Configuration endpoints: http://localhost:8765/config/*
```

**Configuration Profiles (✅ NEW):**
```bash
# Load saved configuration profile
mks-servo-simulator --config-profile my_setup

# Save current configuration as profile
mks-servo-simulator --save-config my_setup --num-motors 2

# Use custom config directory
mks-servo-simulator --config-dir ./my_configs --dashboard
```

**JSON Output for LLMs (✅ NEW):**
```bash
# JSON output mode for Claude Code integration
mks-servo-simulator --json-output --num-motors 2
```

**Combined Features:**
```bash
# Full-featured simulator with all capabilities
mks-servo-simulator \
  --dashboard \
  --debug-api \
  --num-motors 3 \
  --refresh-rate 150 \
  --latency-ms 2.5 \
  --config-profile production
```

The simulator will listen for connections from the library (default: `localhost:6789`). Use `mks-servo-simulator --help` for all options.

### 2. Using the Library

Refer to the scripts in the `examples/` directory for complete, runnable code:

**Basic Motor Control:**
* `examples/single_axis_real_hw.py`: Shows how to connect to and control a physical motor.
* `examples/multi_axis_simulator.py`: Demonstrates controlling multiple motors with the simulator.
* `examples/advanced_axis_control.py`: Advanced motor control techniques and patterns.

**Robot Kinematics:**
* `examples/two_link_planar_arm.py`: TwoLinkArmPlanar robot model example.
* `examples/cartesian_3dof_robot.py`: CartesianRobot model example.
* `examples/three_link_arm.py`: RRRArm robot model example.

**Motor Digitizer System:**
* `examples/basic_digitizer_demo.py`: Introduction to MotorDigitizer capabilities.
* `examples/motor_digitizer.py`: Advanced recording, playback, and precision testing.
* `examples/digitizer_precision_test.py`: Comprehensive system precision analysis.

**Surface Mapping & Plotting:**
* `examples/height_map_generator_v2.py`: Enhanced surface mapping with digitizer integration.
* `examples/enhanced_svg_plotter.py`: Advanced SVG plotting with path optimization.
* `examples/calligraphy_plotter.py`: Artistic text rendering and calligraphy.
* `examples/svg_plotter.py`: Basic SVG plotting.

https://github.com/user-attachments/assets/b7e87119-080f-4230-921d-b1fbb9b76aef

**Performance & Analysis:**
* `examples/benchmark_command_latency.py`: Command latency measurement and analysis.

**Conceptual Snippet (connecting to the simulator):**
```python
import asyncio
from mks_servo_can import (
    CANInterface, Axis, RotaryKinematics, const, exceptions
)

async def control_simulated_motor():
    # Connect to the simulator
    can_if = CANInterface(use_simulator=True, simulator_host='localhost', simulator_port=6789)
    await can_if.connect()

    # Setup an axis (assuming motor with CAN ID 1 is simulated)
    # Using default encoder pulses for MKS servos (16384 pulses/rev)
    kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
    motor1 = Axis(can_if, motor_can_id=1, name="SimMotor1", kinematics=kin)

    try:
        await motor1.initialize() # Basic communication check
        await motor1.enable_motor()

        initial_pos_deg = await motor1.get_current_position_user()
        print(f"Motor '{motor1.name}' initial position: {initial_pos_deg:.2f} degrees")

        # Move 90 degrees at 180 deg/s speed
        await motor1.move_to_position_abs_user(initial_pos_deg + 90.0, speed_user=180.0, wait=True)

        final_pos_deg = await motor1.get_current_position_user()
        print(f"Motor '{motor1.name}' final position: {final_pos_deg:.2f} degrees")

        await motor1.disable_motor()

    except exceptions.MKSServoError as e:
        print(f"Error: {e}")
    finally:
        await can_if.disconnect()

if __name__ == "__main__":
    asyncio.run(control_simulated_motor())
```

### 3. Using the Motor Digitizer System

The MotorDigitizer enables recording manual movements and playing them back with precision testing:

```python
import asyncio
from mks_servo_can import CANInterface, MotorDigitizer, create_linear_axis

async def digitizer_example():
    can_if = CANInterface(use_simulator=True)
    await can_if.connect()
    
    # Create digitizer and add axes
    digitizer = MotorDigitizer(can_if)
    x_axis = create_linear_axis(can_if, 1, "X", 40.0)  # 40mm/rev lead screw
    y_axis = create_linear_axis(can_if, 2, "Y", 40.0)
    
    await digitizer.add_axis(x_axis)
    await digitizer.add_axis(y_axis)
    await digitizer.initialize_axes()
    
    # Record manual movements (motors will be disabled for manual control)
    await digitizer.start_recording(sample_rate=10.0)
    
    # Play back with precision testing
    stats = await digitizer.playback_sequence(
        digitizer.current_sequence, 
        precision_test=True
    )
    
    # Analyze precision
    from mks_servo_can import PrecisionAnalyzer
    assessment = PrecisionAnalyzer.assess_precision(stats)
    print(f"System precision: {assessment}")
    
    await digitizer.cleanup()
    await can_if.disconnect()
```

### 4. Using the HTTP Debug API (✅ NEW)

The simulator provides a comprehensive REST API for programmatic access and LLM integration:

**Configuration Management:**
```bash
# List available configuration profiles
curl http://localhost:8765/config/profiles

# Load a configuration profile
curl -X POST http://localhost:8765/config/profiles/my_setup/load

# Get current configuration
curl http://localhost:8765/config

# Get available motor templates
curl http://localhost:8765/config/templates

# Apply motor template
curl -X POST http://localhost:8765/config/templates/servo42d/apply \
  -H "Content-Type: application/json" \
  -d '{"motor_id": 1}'
```

**Live Parameter Adjustment:**
```bash
# Get adjustable parameters
curl http://localhost:8765/config/parameters

# Update CAN bus latency
curl -X POST http://localhost:8765/config/parameters/latency_ms \
  -H "Content-Type: application/json" \
  -d '{"value": 3.5}'

# Update motor current limit
curl -X POST http://localhost:8765/config/parameters/motors.0.max_current \
  -H "Content-Type: application/json" \
  -d '{"value": 1200}'
```

**Command Injection:**
```bash
# Inject raw command
curl -X POST http://localhost:8765/inject \
  -H "Content-Type: application/json" \
  -d '{"motor_id": 1, "command_code": 246, "data_bytes": [1, 0, 100, 0]}'

# Use template command
curl -X POST http://localhost:8765/inject_template \
  -H "Content-Type: application/json" \
  -d '{"motor_id": 1, "template_name": "enable"}'
```

**Performance Monitoring:**
```bash
# Get current performance metrics
curl http://localhost:8765/performance

# Get performance history
curl http://localhost:8765/performance/history

# Get connection statistics
curl http://localhost:8765/performance/connections
```

## Documentation

Detailed documentation, including API references, user guides, and advanced topics, is planned for the `docs/` directory. The `docs/README.md` file provides an overview of the planned documentation structure.

For now, please refer to:
* **Docstrings** within the source code of the library.
* The **example scripts** in the `examples/` directory.
* The **"MKS SERVO42D/57D_CAN User Manual V1.0.6.pdf"** (which should be obtained separately) for specifics on CAN commands, motor parameters, and behavior.

## Development and Testing

### Dependencies for Development

Install development dependencies (for linting, testing, etc.) using the `[dev]` extra. From the project root:
```bash
pip install -e ./mks_servo_can_library[dev]
pip install -e ./mks_servo_simulator[dev]
```
Alternatively, if a combined `requirements-dev.txt` is created, use that.

### Running Tests

The project includes a `tests/` directory for unit, integration, HIL, and determinism tests. `pytest` is the recommended test runner.

To run all tests (excluding HIL, which requires specific hardware setup):
```bash
pytest
```
To run tests for a specific part:
```bash
# From project root, after installing dev dependencies
pytest tests/unit
pytest tests/integration # Requires the simulator to be running for some tests
```
(HIL tests require physical hardware and are typically run manually or in a dedicated CI environment.)

## Contributing

Contributions are welcome! Please follow these general guidelines:
1.  **Fork the repository.**
2.  **Create a new branch** for your feature or bug fix (e.g., `feature/my-new-feature` or `fix/issue-123`).
3.  **Write clean, well-commented code** adhering to PEP 8 guidelines. Use a linter like Flake8.
4.  **Include comprehensive docstrings** for all public modules, classes, and functions.
5.  **Add unit tests** for new functionality and bug fixes. Ensure good test coverage.
6.  **Ensure all tests pass** locally before submitting.
7.  **Submit a pull request** to the `main` (or `develop`) branch with a clear description of your changes and any relevant issue numbers.


## License

This project is licensed under the MIT License. See the `LICENSE.txt` file for details.

## AI Disclaimer

* This code may have been generated by an "AI". Side effects may include spontaneous bugs, existential dread, and coffee dependency.
* In case of failures, just blame the AI. But if it starts fixing its own bugs... run.


## Where to buy (affiliated links)
If you're planning to buy one or more motors and controllers to build something awesome with the MKS SERVO CAN library, you can support my work by using the affiliate links below:
* [USB 2 CAN Adapter (the same I have, works nicelly on linux)](https://amzn.to/3Ga00eQ)
* [MKS SERVO42D Closed Loop Stepper Motor Drive CAN](https://amzn.to/3TA9T8M)
* [NEMA 17 Stepper Motor (42x42x39mm)](https://amzn.to/4ndOwYy)
* [NEMA 17 Stepper Motor (42x42x60mm)](https://amzn.to/4ljbZpv)
* [NEMA 17 Stepper Motor (42x42x23mm - pancake style)](https://amzn.to/4edb4EJ)
