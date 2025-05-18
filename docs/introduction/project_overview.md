# Project Overview

This project provides a Python library (`mks-servo-can`) for controlling MKS SERVO42D and MKS SERVO57D motors via a CAN bus interface, and a command-line simulator (`mks-servo-simulator`) for testing and development without physical hardware. The system is designed with `asyncio` for asynchronous operations, enabling efficient handling of I/O and motor communications.

**Key Reference:** The functionality and command implementations are primarily based on the "MKS SERVO42D/57D_CAN User Manual V1.0.6".

## Purpose

The primary goal of this project is to offer a robust, user-friendly, and asynchronous Python interface for controlling MKS servo motors commonly used in hobbyist and professional robotics, CNC machines, and automation projects. Additionally, the included simulator aims to facilitate development and testing workflows by removing the immediate need for physical hardware.

## Main Components

* **`mks-servo-can` (Python Library):** A comprehensive library featuring:
    * Asynchronous communication via `asyncio`.
    * Both low-level access to raw CAN commands and a high-level API (`Axis`, `MultiAxisController`).
    * A kinematics engine for unit conversions (e.g., degrees/mm to motor steps).
    * Support for both real hardware (via `python-can`) and the provided simulator.
* **`mks-servo-simulator` (CLI Tool):** A command-line application that:
    * Simulates the behavior of multiple MKS servo motors.
    * Listens for CAN commands from the `mks-servo-can` library over a TCP socket.
    * Allows for testing and debugging the control library without physical motors.

*(For more details, you can refer to the [main project README.md](../../README.md).)*