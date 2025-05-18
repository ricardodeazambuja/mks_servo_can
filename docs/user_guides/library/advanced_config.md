# Advanced Configuration

## Prerequisites

* Familiarity with `CANInterface` and `Axis` class initialization and basic usage.
* Understanding of reading motor parameters as covered in [Reading Motor Status & Parameters](./reading_status.md).
* Awareness of the potential risks when modifying persistent motor parameters.

## 1. Timeout Management

### a. Default Timeout in `Axis`

When an `Axis` object is initialized, it uses a default timeout for all its operations. This can be set during instantiation:

```python
from mks_servo_can import Axis, CANInterface, const, exceptions
from mks_servo_can.kinematics import RotaryKinematics
import asyncio

# Assume can_if is a connected CANInterface
# can_if = CANInterface(use_simulator=True) 
# await can_if.connect()

# Set a default timeout of 1 second (1000 ms) for this axis
custom_timeout_axis = Axis(
    can_interface=can_if, 
    can_id=1, 
    default_timeout_ms=1000 # Default is const.DEFAULT_CAN_TIMEOUT_MS (e.g., 500ms)
)
# print(f"Axis {custom_timeout_axis.name} initialized with default timeout: {custom_timeout_axis.default_timeout_ms} ms")
```

If `default_timeout_ms` is not provided, it defaults to `const.DEFAULT_CAN_TIMEOUT_MS`. 

### b. Per-Command Timeout in Axis Methods

Most methods in the Axis class that involve communication with the motor (e.g., move_absolute, read_parameter, enable) accept an optional timeout_ms parameter. This allows you to override the axis's default timeout for that specific command:

```python
async def per_command_timeout_example(axis: Axis):
    try:
        # Use a longer timeout (2 seconds) for this specific read operation
        firmware_version = await axis.read_parameter(
            const.PARAM_FIRMWARE_VERSION, 
            timeout_ms=2000
        )
        print(f"{axis.name} Firmware (with custom timeout): {firmware_version}")

        # This next command will use the axis's default_timeout_ms
        await axis.ping() 
        print(f"{axis.name} pinged successfully using default timeout.")

    except exceptions.MotorTimeoutError as e:
        print(f"A command timed out for {axis.name}: {e}")

# await per_command_timeout_example(custom_timeout_axis)
```

### c. CANInterface Response Timeout

The CANInterface itself has a response_timeout_ms parameter (defaulting to const.DEFAULT_CAN_TIMEOUT_MS). This is the underlying timeout used when waiting for any message response from the CAN bus that matches an expected response ID. The Axis level timeouts are typically built upon this. While you can set it during CANInterface initialization, it's generally recommended to manage timeouts at the Axis level or per-command unless you have a specific reason to change the global interface timeout:

```python
can_if_custom_timeout = CANInterface(
    use_simulator=True, 
    response_timeout_ms=750 # Sets the base timeout for the interface
)
```

## 2. Writing Motor Parameters (write_parameter)

The MKS servo motors have various configurable parameters stored internally (e.g., CAN ID, maximum speed, PID settings). The Axis class provides a `write_parameter` method to modify these:

- **Caution:** Writing parameters, especially critical ones like CAN ID or bitrate, while the system is operational can be risky. Incorrect values may cause the motor to behave 
unexpectedly. Always consult the MKS motor manual for details on each parameter, its valid range, and whether a power cycle is needed for changes to take effect.

- **Example:** Change the motor's 'Max Acceleration' (PARAM_MAX_ACCELERATION - 0x2009)

**Important:** The actual value and its meaning depend on the motor firmware. This is a hypothetical example. Consult the MKS manual for correct values and units.

- **Note:** The simulator by default starts motors with ID 1, 2, ...
- **Note:** Ensure the simulator (or real hardware) is set to an ID you expect.

## 3. CRC Checking Configuration

The CANInterface can be configured to perform CRC (Cyclic Redundancy Check) validation on received messages. This helps ensure data integrity:

```python
can_if_crc_enabled = CANInterface(
    use_simulator=True, # Or hardware parameters
    enable_crc=True 
)
```

**Enable CRC checking:** If enable_crc=True (default), and a received message's CRC does not match the calculated CRC, a `CRCError` will be raised.

**Disable CRC checking:** If enable_crc=False, CRC checks are skipped.

**Note:** This may slightly reduce CPU load but increases the risk of acting on corrupted data in noisy environments.

## 4. Low-Level API for Unexposed Features

While the Axis class aims to provide a high-level interface for common operations, there might be motor features or parameters not directly exposed through Axis methods. For these, you 
can use the LowLevelAPI associated with the CANInterface:

```python
can_interface = CANInterface(...) # Ensure can_interface is defined and connected
# await can_interface.connect()
# low_level_api = can_interface.low_level_api
# motor_can_id = 1 # Example motor ID

# Example: Sending a raw command (use with extreme caution)
frame_id = 0x0140 + motor_can_id # Constructing a command frame ID
data = [0x23, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x00, 0x00] # Example: Raw command to read microstep (0x200B)
response_future = await low_level_api.send_command_and_wait_response(
    can_id_motor=motor_can_id, 
    command_id_hex=frame_id, # This is not the command_id but the full frame_id
    data=data,
    response_id_hex=0x0240 + motor_can_id # Expected response frame ID
)

print(f"Raw response: {response_message}")
```

Using the LowLevelAPI directly requires a thorough understanding of the MKS CAN communication protocol as detailed in their user manual. It bypasses the abstractions and safety checks of 
the Axis class.

## Conclusion
These advanced configuration options provide flexibility for tailoring the library's behavior to specific application needs and for interacting with a wider range of motor settings.