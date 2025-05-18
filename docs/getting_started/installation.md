# Installation Guide

This guide covers the installation of both the `mks-servo-can` Python library and the `mks-servo-simulator` command-line tool. It's highly recommended to perform these installations within a Python virtual environment.

## Prerequisites

Before proceeding, please ensure you have met all the software and hardware prerequisites outlined in the [Prerequisites document](./prerequisites.md). This primarily includes:

* Python 3.8+
* `pip` (Python package installer)
* A virtual environment (recommended)
* Core dependencies (`python-can` for hardware, `click` for simulator) which can be installed via `pip install -r requirements.txt` from the project root.

## Installing the `mks-servo-can` Library

The library allows you to control MKS servo motors from your Python scripts.

### For Users (Standard Installation)

If you intend to use the library as a dependency in your project:
1.  Navigate to the `mks_servo_can_library` directory within the project.
    ```bash
    cd path/to/mks_servo_can/mks_servo_can_library
    ```
2.  Install using pip:
    ```bash
    pip install .
    ```

### For Developers (Editable Installation)

If you plan to modify or contribute to the library, an editable installation is recommended. This links the installed package directly to your source code, so changes are reflected immediately.
1.  Navigate to the `mks_servo_can_library` directory within the project.
    ```bash
    cd path/to/mks_servo_can/mks_servo_can_library
    ```
2.  Install in editable mode:
    ```bash
    pip install -e .
    ```

## Installing the `mks-servo-simulator` CLI Tool

The simulator allows you to test your control logic without physical hardware.

### For Users (Standard Installation)

If you just want to use the simulator:
1.  Navigate to the `mks_servo_simulator` directory within the project.
    ```bash
    cd path/to/mks_servo_can/mks_servo_simulator
    ```
2.  Install using pip:
    ```bash
    pip install .
    ```
This will make the `mks-servo-simulator` command available in your shell path (if your virtual environment's script directory is in the PATH).

### For Developers (Editable Installation)

For developing or modifying the simulator:
1.  Navigate to the `mks_servo_simulator` directory within the project.
    ```bash
    cd path/to/mks_servo_can/mks_servo_simulator
    ```
2.  Install in editable mode:
    ```bash
    pip install -e .
    ```

This also makes the `mks-servo-simulator` command available and reflects source code changes immediately.

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
