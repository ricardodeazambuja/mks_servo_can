# Prerequisites

Before you begin working with the `mks-servo-can` library and `mks-servo-simulator`, ensure you have the following prerequisites met:

## Software Requirements

* **Python:** Version 3.8 or higher is required. You can download Python from [python.org](https://www.python.org/).
* **pip:** The Python package installer. It usually comes with Python installations.
* **Virtual Environment (Highly Recommended):** To avoid conflicts with other Python projects and system-wide packages, it's strongly advised to use a Python virtual environment.
    * To create a virtual environment:
        ```bash
        python -m venv .venv
        ```
    * To activate it:
        * On Windows:
            ```bash
            .venv\Scripts\activate
            ```
        * On macOS and Linux:
            ```bash
            source .venv/bin/activate
            ```
* **Core Dependencies:** These can be installed via the `requirements.txt` file located in the project root.
    ```bash
    pip install -r requirements.txt
    ```
    This will typically install:
    * `python-can>=3.0.0,<5.0.0` (for real hardware interaction)
    * `click>=7.0` (for the `mks-servo-simulator` CLI)

## Hardware Requirements (for Physical Motor Control)

If you plan to control physical MKS SERVO42D or SERVO57D motors, you will need:

* **MKS Servo Motor(s):** SERVO42D, SERVO57D, or compatible MKS CAN-bus enabled servo motors.
* **CAN Adapter:** A `python-can` compatible CAN interface. Common examples include:
    * **CANable:** (Often using `slcan` firmware or `gs_usb` kernel module)
    * **Kvaser:** (e.g., Kvaser Leaf Light)
    * **PCAN:** (e.g., PEAK-System PCAN-USB)
    * **SocketCAN compatible devices:** (e.g., Raspberry Pi with a CAN MCP2515 controller, or virtual CAN `vcan`)
    * **USB2CAN adapters**
    * Other interfaces supported by `python-can` (refer to [python-can documentation](https://python-can.readthedocs.io/en/stable/interfaces.html) for a full list).
* **Appropriate Drivers:** Install any necessary drivers for your specific CAN adapter on your host computer. This might involve kernel modules (like `gs_usb` or `socketcan` drivers on Linux) or manufacturer-provided drivers.
* **CAN Bus Wiring:**
    * Twisted-pair cable for CAN_H and CAN_L lines.
    * Appropriate connectors if your motors/adapter use them.
* **Termination Resistors:** Two 120 Ohm termination resistors. These are critical and must be placed at the two physical ends of the CAN bus. Many MKS motors and some CAN adapters have built-in, selectable termination.
* **Motor Power Supply:** A DC power supply appropriate for your MKS servo motors (e.g., 24V DC), with sufficient current capacity for all connected motors.
* **Host Computer:** A computer to run your Python scripts and connect to the CAN adapter.

## For Using the Simulator Only

If you only intend to use the `mks-servo-simulator`, the hardware requirements above are not necessary. You will only need the software requirements, primarily Python and the `click` library (which is included in `requirements.txt`).

## Operating System

* **Linux:** Generally well-supported, especially for SocketCAN interfaces.
* **Windows:** Supported, specific CAN adapter drivers may be required.
* **macOS:** Supported, specific CAN adapter drivers may be required.

Ensure your operating system is configured correctly for the chosen CAN adapter (e.g., setting up `slcand` for serial-line CAN adapters on Linux, or ensuring kernel modules like `gs_usb` are loaded for CANable devices).

Refer to the [Installation Guide](installation.md) for steps on installing the library and simulator.
Refer to the [Basic Hardware Setup](hardware_setup.md) for more details on connecting physical motors.