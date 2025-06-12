# Installation Guide

This guide covers the installation of both the `mks-servo-can` Python library and the `mks-servo-simulator` command-line tool. It's highly recommended to perform these installations within a Python virtual environment.

## Prerequisites

Before proceeding, please ensure you have met all the software and hardware prerequisites outlined in the [Prerequisites document](./prerequisites.md). This primarily includes:

* Python 3.8+
* `pip` (Python package installer)
* A virtual environment (recommended)

## Quick Installation (Recommended)

For most users, install both the library and simulator from the project root:

```bash
# Clone or download the project
git clone <repository-url>
cd mks_servo_can

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install library in editable mode
cd mks_servo_can_library
pip install -e .
cd ..

# Install simulator in editable mode  
cd mks_servo_simulator
pip install -e .
cd ..
```

This installs both packages in editable mode, making the `mks-servo-simulator` command available and allowing you to import `mks_servo_can` in your Python scripts.

## Verifying Installation

Test that everything is installed correctly:

### Test Library Import
```bash
python -c "import mks_servo_can; print('Library installed successfully!')"
```

### Test Simulator Command
```bash
mks-servo-simulator --help
```

You should see the simulator help text if installation was successful.

### Test with Examples
```bash
# Start the simulator (in one terminal)
mks-servo-simulator --num-motors 4 --start-can-id 1 --latency-ms 5

# Run an example (in another terminal)
cd examples
python benchmark_command_latency.py
```

## Detailed Installation Options

### Installing the `mks-servo-can` Library Only

If you only need the library (not the simulator):

```bash
cd mks_servo_can_library
pip install .  # Standard installation
# OR
pip install -e .  # Editable installation (for development)
```

### Installing the `mks-servo-simulator` Only

If you only need the simulator:

```bash
cd mks_servo_simulator  
pip install .  # Standard installation
# OR
pip install -e .  # Editable installation (for development)
```

## Hardware Dependencies (Optional)

For connecting to real hardware, install python-can:

```bash
pip install python-can
```

This is **not required** if you only plan to use the simulator.

## Development Setup

For contributing to the project:

```bash
# Install both packages in editable mode
cd mks_servo_can_library
pip install -e .
cd ../mks_servo_simulator
pip install -e .

# Install development dependencies (if available)
pip install -r requirements-dev.txt  # If this file exists
```

## Development Note on `PYTHONPATH`

When developing both packages, especially if not using editable installs for both within the same virtual environment, you might encounter import issues (e.g., the simulator trying to import constants from the library).

Using editable installs (`pip install -e .`) for both packages, ideally within the same Python virtual environment, is the generally recommended approach for a smooth development experience, as it correctly handles package discovery.

If you choose a different setup, you might need to manually adjust your `PYTHONPATH` environment variable to include the root directory of the project so that the `mks_servo_can` library can be found by the simulator or other scripts. For example, if running the simulator from the project root:
```bash
PYTHONPATH=$(pwd) mks-servo-simulator [options] # For Linux/macOS
# For Windows CMD (approximate, or use PowerShell equivalent):
# set PYTHONPATH=%cd%
# mks-servo-simulator [options]
```
