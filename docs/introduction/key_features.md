# Key Features

This project offers a Python-based solution for controlling MKS SERVO42D and MKS SERVO57D motors, along with a simulator for development and testing. The primary features are categorized by their respective components:

## Python Library (`mks-servo-can`)

* **Asynchronous Operations**: Built from the ground up with `asyncio` to ensure non-blocking communication and motor control, suitable for responsive applications.
* **Low-Level API**: Provides direct access to CAN commands as detailed in the MKS user manual, allowing for fine-grained control and implementation of any motor feature.
* **High-Level API**: Offers abstracted and more intuitive interfaces:
    * `Axis` class: Simplifies the control of individual motors, managing state, and providing user-friendly methods for common operations (e.g., movement, enabling, status checks).
    * `MultiAxisController`: Facilitates the management and coordination of multiple `Axis` instances, useful for systems with several motors.
* **Kinematics Engine**:
    * Allows conversion between physical, user-defined units (like millimeters or degrees) and the motor's native encoder pulses or steps.
    * Comes with pre-built `LinearKinematics` and `RotaryKinematics` classes.
    * Includes a base `Kinematics` class to enable users to implement custom kinematic models (e.g., for SCARA arms, delta robots, or other non-linear mechanisms like the provided `EccentricKinematics` example).
* **Dual Mode Operation**: Seamlessly switch between controlling physical MKS servo motors (via `python-can` compatible adapters) and the `mks-servo-simulator` for development and testing.
* **Comprehensive Error Handling**: Features a set of custom exceptions for clear and specific reporting of issues related to CAN communication, motor responses, parameter errors, or configuration problems.

## CLI Simulator (`mks-servo-simulator`)

* **Hardware-Free Development**: Enables development and testing of the `mks-servo-can` library and user applications without requiring physical MKS motors or CAN hardware.
* **Motor Behavior Simulation**: Models the behavior of multiple MKS servo motors, including responses to CAN commands and updates to internal states like position and speed.
* **Virtual CAN Bus**: Emulates CAN bus activity and manages the communication interface (TCP socket) with the `mks-servo-can` library.
* **Configurable Simulation Parameters**: Allows users to specify:
    * The number of motors to simulate and their starting CAN IDs.
    * Simulated CAN bus latency to introduce realistic communication delays.
    * Basic motor characteristics such as type and steps per revolution.
* **TCP Socket Interface**: The control library connects to the simulator using a standard TCP socket (defaulting to `localhost:6789`), making it network-accessible if needed.

## General Project Features

* **Determinism Considerations**: The library is designed with an awareness of timing aspects, providing a foundation for applications where predictable command execution is important. Example benchmarks for latency are provided.
* **Structured Testing**: Includes a testing framework with unit tests for individual components, integration tests utilizing the simulator, and placeholders/examples for Hardware-In-the-Loop (HIL) and timing/determinism benchmarks.