# Connecting to Motors (Hardware & Simulator)

The `CANInterface` class in the `mks-servo-can` library is responsible for establishing and managing the communication link to your MKS servo motors. It can connect to either physical hardware via a `python-can` compatible adapter or to the `mks-servo-simulator`.

## Importing `CANInterface`

First, import the necessary class and other relevant modules:
```python
import asyncio
from mks_servo_can import CANInterface, exceptions
```

## Connecting to the Simulator

To connect to the mks-servo-simulator, you can either let the library auto-detect a running simulator or explicitly configure it. First, ensure the simulator is running:

```bash
# Start the simulator with 4 motors (CAN IDs 1-4)
mks-servo-simulator --num-motors 4 --start-can-id 1 --latency-ms 5
```

### Auto-detection (Recommended)
The simplest approach - the library will automatically detect and connect to a running simulator:
```python
async def connect_to_simulator_auto():
    # CANInterface will auto-detect the simulator if running
    can_interface = CANInterface()
    
    try:
        print("Attempting to connect to CAN interface...")
        await can_interface.connect()
        print("Successfully connected! Using simulator mode.")
        
        # Verify connection
        if can_interface.is_connected:
            print("CANInterface reports connected.")
            return can_interface
            
    except exceptions.CANError as e:
        print(f"Error connecting: {e}")
        print("Please ensure the simulator is running with: mks-servo-simulator --num-motors 4")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

# To run this example:
# can_if = asyncio.run(connect_to_simulator_auto())
```

### Explicit Simulator Configuration
For more control over simulator connection:
```python
async def connect_to_simulator_explicit():
    can_interface = CANInterface(
        interface_type="simulator",
        simulator_host="localhost",  # Default host
        simulator_port=6789         # Default port
    )
    
    try:
        print("Attempting to connect to the MKS Servo Simulator...")
        await can_interface.connect()
        print("Successfully connected to the MKS Servo Simulator!")
        
        if can_interface.is_connected:
            print("CANInterface reports connected.")
            return can_interface
            
    except exceptions.CANError as e:
        print(f"Error connecting to simulator: {e}")
        print("Please ensure the simulator is running and accessible.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None
    finally:
        # Note: Don't disconnect here if you plan to use the interface
        pass

# To run this example:
# can_if = asyncio.run(connect_to_simulator_explicit())
```

## Key Parameters for Simulator Connection:
* `interface_type="simulator"`: Explicitly tells CANInterface to use simulator mode.
* `simulator_host`: The hostname or IP address where the simulator is running. Defaults to "localhost".
* `simulator_port`: The TCP port the simulator is listening on. Defaults to 6789.

## Connecting to Real Hardware
To connect to physical MKS servo motors:

```python
async def connect_to_hardware_example():
    # --- IMPORTANT: Modify these for your specific CAN adapter and setup ---
    can_interface = CANInterface(
        interface_type="socketcan",  # Examples: 'socketcan', 'kvaser', 'pcan', 'canable'
        channel="can0",              # Examples: 'can0', 'PCAN_USBBUS1', '/dev/ttyACM0', 'COM3'
        bitrate=500000              # Default is 500000, ensure it matches your motor's setting
    )
    
    try:
        print(f"Attempting to connect to CAN hardware...")
        await can_interface.connect()
        print(f"Successfully connected to CAN hardware!")
        
        if can_interface.is_connected:
            print("CANInterface reports connected.")
            return can_interface
            
    except exceptions.CANError as e:
        print(f"Error connecting to CAN hardware: {e}")
        print("Troubleshooting tips:")
        print("- Verify your CAN adapter is properly connected to the computer and the CAN bus.")
        print("- Ensure the necessary drivers for your CAN adapter are installed and configured.")
        print("- Check that the 'interface_type' and 'channel' parameters are correct for your adapter and OS.")
        print("- Confirm the 'bitrate' matches the setting on your MKS servo motors.")
        print("- Ensure the CAN bus is correctly wired (CAN_H, CAN_L) and terminated (120 Ohm resistors at both ends).")
        print("- Make sure the MKS servo motors are powered on.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

# To run this example:
# can_if = asyncio.run(connect_to_hardware_example())
```

## Key Parameters for Hardware Connection:
* `interface_type`: The type of python-can interface you are using (e.g., "socketcan", "canable", "kvaser", "pcan").
* `channel`: The specific channel or port for your CAN adapter (e.g., "can0" for SocketCAN, "/dev/ttyUSB0" or "COM3" for serial-based adapters like CANable using slcan). This parameter is crucial and depends heavily on your OS and adapter.
* `bitrate`: The CAN bus bitrate. This must match the bitrate configured on your MKS servo motors (e.g., 500000). Default is 500000.

Refer to the python-can documentation for more details on configuring various interface types and channels.
Also, review the Basic Hardware Setup guide for wiring and termination.

## Disconnecting
Always properly disconnect when done:
```python
async def cleanup_connection(can_interface):
    if can_interface and can_interface.is_connected:
        print("Disconnecting from CAN interface...")
        await can_interface.disconnect()
        print("Disconnected.")

# In your main function:
# try:
#     can_if = await connect_to_simulator_auto()  # or connect_to_hardware_example()
#     # ... use can_if ...
# finally:
#     await cleanup_connection(can_if)
```