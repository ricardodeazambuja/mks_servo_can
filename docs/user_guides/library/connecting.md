# Connecting to Motors (Hardware & Simulator)

The `CANInterface` class in the `mks-servo-can` library is responsible for establishing and managing the communication link to your MKS servo motors. It can connect to either physical hardware via a `python-can` compatible adapter or to the `mks-servo-simulator`.

## Importing `CANInterface`

First, import the necessary class and other relevant modules:
```python
import asyncio
from mks_servo_can import CANInterface, exceptions, const
```

## Connecting to the Simulator
To connect to the mks-servo-simulator (ensure it's running first, see Running the Simulator):
```python
async def connect_to_simulator_example():
    can_if_sim = CANInterface(
        use_simulator=True,
        simulator_host="localhost",  # Default host
        simulator_port=6789         # Default port
    )
    try:
        print("Attempting to connect to the MKS Servo Simulator...")
        await can_if_sim.connect()
        print("Successfully connected to the MKS Servo Simulator!")
        # At this point, the can_if_sim object can be used to initialize Axis objects
        # Example: Check if connected
        if can_if_sim.is_connected:
            print("CANInterface reports connected.")
    except exceptions.SimulatorError as e:
        print(f"Error connecting to simulator: {e}")
        print("Please ensure the simulator is running and accessible.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if can_if_sim.is_connected:
            print("Disconnecting from the simulator...")
            await can_if_sim.disconnect()
            print("Disconnected.")

# To run this example:
# asyncio.run(connect_to_simulator_example())
```

## Key Parameters for Simulator Connection:
* `use_simulator=True`: This is essential to tell CANInterface to operate in simulator mode.
* `simulator_host`: The hostname or IP address where the simulator is running. Defaults to "localhost".
* `simulator_port`: The TCP port the simulator is listening on. Defaults to 6789.

## Connecting to Real Hardware
To connect to physical MKS servo motors:

```python
async def connect_to_hardware_example():
    # --- IMPORTANT: Modify these for your specific CAN adapter and setup ---
    can_interface_type = "canable"  # Examples: 'socketcan', 'kvaser', 'pcan', 'usb2can', 'serial'
    can_channel = "/dev/ttyACM0"    # Examples: 'slcan0', 'can0', 'PCAN_USBBUS1', 'COM3'
    can_bitrate = const.CAN_DEFAULT_BITRATE # Default is 500000, ensure it matches your motor's setting

    can_if_hw = CANInterface(
        interface_type=can_interface_type,
        channel=can_channel,
        bitrate=can_bitrate,
        use_simulator=False # Explicitly set to False or omit (default is False)
    )
    try:
        print(f"Attempting to connect to CAN hardware ({can_interface_type} on {can_channel} at {can_bitrate} bps)...")
        await can_if_hw.connect()
        print(f"Successfully connected to CAN hardware!")
        # At this point, the can_if_hw object can be used to initialize Axis objects
        if can_if_hw.is_connected:
            print("CANInterface reports connected.")
    except exceptions.CANError as e:
        print(f"Error connecting to CAN hardware: {e}")
        print("Troubleshooting tips:")
        print("- Verify your CAN adapter is properly connected to the computer and the CAN bus.")
        print("- Ensure the necessary drivers for your CAN adapter are installed and configured.")
        print("- Check that the 'interface_type' and 'channel' parameters are correct for your adapter and OS.")
        print("- Confirm the 'bitrate' matches the setting on your MKS servo motors.")
        print("- Ensure the CAN bus is correctly wired (CAN_H, CAN_L) and terminated (120 Ohm resistors at both ends).")
        print("- Make sure the MKS servo motors are powered on.")
    except exceptions.ConfigurationError as e:
        print(f"Configuration error: {e}")
        print("This might happen if 'python-can' is not installed or if required parameters are missing.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if can_if_hw.is_connected:
            print("Disconnecting from CAN hardware...")
            await can_if_hw.disconnect()
            print("Disconnected.")

# To run this example:
# asyncio.run(connect_to_hardware_example())
```

## Key Parameters for Hardware Connection:
* `use_simulator=False`: Ensures CANInterface attempts to use a real CAN bus. This is the default if use_simulator is not specified
* `interface_type`: The type of python-can interface you are using (e.g., "socketcan", "canable", "kvaser", "pcan").
* `channel`: The specific channel or port for your CAN adapter (e.g., "can0" for SocketCAN, "/dev/ttyUSB0" or "COM3" for serial-based adapters like CANable using slcan). This parameter is crucial and depends heavily on your OS and adapter.
* `bitrate`: The CAN bus bitrate. This must match the bitrate configured on your MKS servo motors (e.g., 500000). Default is 500000 (from `const.CAN_DEFAULT_BITRATE`). 

Refer to the python-can documentation for more details on configuring various interface types and channels 
Also, review the Basic Hardware Setup guide for wiring and termination.

## `asyncio` Event Loop
The `CANInterface` can accept an `asyncio.AbstractEventLoop` instance via the loop parameter in its constructor. If no loop is provided, it will use `asyncio.get_event_loop()`.