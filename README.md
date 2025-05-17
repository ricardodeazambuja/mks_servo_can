# MKS Servo CAN Control Project

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
* **Hardware & Simulator Support**: Can connect to real MKS servo motors via various `python-can` compatible interfaces or to the provided `mks-servo-simulator`.
* **Robust Error Handling**: Custom exceptions for clear diagnostics of communication, motor, and configuration issues.

### CLI Simulator (`mks-servo-simulator`)
* **Hardware-less Development**: Test the `mks-servo-can` library without physical motors.
* **Motor Behavior Modeling**: Simulates multiple MKS servo motors, responding to CAN commands by updating internal states (position, speed, etc.).
* **Virtual CAN Bus**: Emulates CAN bus interactions, managing communication with the library via a TCP socket.
* **Configurable Parameters**:
    * Number of simulated motors and their CAN IDs.
    * Simulated CAN bus latency to mimic real-world delays.
    * Motor type and basic characteristics (e.g., steps per revolution).
* **TCP Socket Interface**: The library connects to the simulator via a TCP socket (default: `localhost:6789`).

### General
* **Determinism Focus**: Designed with considerations for analyzing and understanding timing behavior, aiding in applications with real-time constraints.
* **Comprehensive Test Strategy**: Includes unit tests, integration tests (simulator-based), and examples for Hardware-In-the-Loop (HIL) and determinism/timing benchmarks.

## Project Structure

The project is organized into two main Python packages and supporting directories:

```
mks_servo_can/
├── mks_servo_can_library/        # The installable Python library (mks_servo_can)
│   ├── mks_servo_can/            # Source code for the library
│   │   ├── kinematics/           # Kinematic transformation modules
│   │   ├── __init__.py
│   │   ├── can_interface.py    # Handles real CAN and simulator connection
│   │   ├── low_level_api.py    # Implements MKS CAN commands
│   │   ├── axis.py               # High-level single motor control
│   │   ├── multi_axis_controller.py # High-level multi-motor control
│   │   ├── constants.py
│   │   ├── crc.py
│   │   └── exceptions.py
│   └── setup.py                  # Packaging script for the library
├── mks_servo_simulator/        # The CLI simulator (mks_servo_simulator)
│   ├── mks_simulator/            # Source code for the simulator
│   │   ├── __init__.py
│   │   ├── cli.py                # Command-line interface (using Click)
│   │   ├── motor_model.py        # Simulates individual motor behavior
│   │   ├── virtual_can_bus.py    # Manages simulated CAN traffic
│   │   └── main.py               # Entry point for the simulator CLI
│   └── setup.py                  # Packaging script for the simulator
├── tests/                        # Unit, integration, HIL, determinism tests
│   ├── unit/
│   ├── integration/
│   ├── hil/
│   └── determinism/
├── examples/                     # Example scripts demonstrating library usage
│   ├── single_axis_real_hw.py
│   ├── multi_axis_simulator.py
│   └── timing_benchmark.py
├── docs/                         # Detailed documentation
│   └── README.md                 # This file (overview of documentation)
│   └── (other .md files for specific sections)
├── README.md                     # Main project README
├── requirements.txt              # Core Python dependencies for running the library and simulator
└── LICENSE.txt                   # Project license (e.g., MIT License)
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

Open a terminal and start the simulator. For example, to simulate two motors with CAN IDs 1 and 2, and a 5ms round-trip bus latency:
```bash
mks-servo-simulator --num-motors 2 --start-can-id 1 --latency-ms 5
```
The simulator will listen for connections from the library (default: `localhost:6789`). Use `mks-servo-simulator --help` for all options.

### 2. Using the Library

Refer to the scripts in the `examples/` directory for complete, runnable code:
* `examples/multi_axis_simulator.py`: Demonstrates controlling multiple motors with the simulator.
* `examples/single_axis_real_hw.py`: Shows how to connect to and control a physical motor.
* `examples/timing_benchmark.py` (also known as `benchmark_command_latency.py`): Can be used to measure command latencies with either the simulator or real hardware.

**Conceptual Snippet (connecting to the simulator):**
```python
import asyncio
from mks_servo_can_library.mks_servo_can import (
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

**(Maintainers: Please update placeholder information like author, email, and project URLs in the `setup.py` files for both the library and simulator packages.)**

## License

This project is licensed under the MIT License. See the `LICENSE.txt` file for details.
