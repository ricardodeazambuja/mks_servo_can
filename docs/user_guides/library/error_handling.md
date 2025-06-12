# Error Handling & Exceptions in `mks-servo-can`

Robust applications require proper error handling. The `mks-servo-can` library defines a hierarchy of custom exceptions to help you identify and manage issues that can arise during communication with MKS servo motors or the simulator.

## Base Exception: `MKSServoCANError`

All custom exceptions raised by this library inherit from `MKSServoCANError`. This allows you to catch any library-specific error with a single `except` block if needed.

```python
from mks_servo_can import exceptions
from mks_servo_can import CANInterface # For context in examples
from mks_servo_can import Axis # For context in examples
from mks_servo_can import const # For context in examples
import asyncio # For context in examples


# Placeholder for an Axis object for example snippets
# async def setup_axis():
#     can_if = CANInterface(use_simulator=True)
#     await can_if.connect()
#     axis = Axis(can_if, 1)
#     return axis, can_if
#
# async def main_example():
#     axis, can_if = await setup_axis()
#     try:
#         # Some library operation that might fail
#         await axis.move_absolute(100, 10) # Example operation
#         print("Operation successful.")
#     except exceptions.MKSServoCANError as e:
#         print(f"A general mks-servo-can library error occurred: {e}")
#     except Exception as e:
#         print(f"An unrelated or unexpected error occurred: {e}")
#     finally:
#         if can_if.is_connected:
#             await can_if.disconnect()
#
# # asyncio.run(main_example())
```

## Specific Exception Types

Here's a breakdown of the more specific exception types:

### 1. ConfigurationError

**Description**: Raised when there's an issue with the setup or configuration of the library or its dependencies.

**Common Causes**:
- python-can library not installed when trying to connect to real hardware (use_simulator=False).
- Missing essential parameters for CANInterface initialization (e.g., channel for hardware mode).
- Invalid CAN interface type specified.

**Example**:
```python
# try:
#     can_if = CANInterface(interface_type="non_existent_type", channel="can0", use_simulator=False)
# except exceptions.ConfigurationError as e:
#     print(f"Configuration Error: {e}")
```

### 2. CANError

**Description**: Related to problems occurring at the CAN bus communication level when interacting with physical hardware. This often wraps errors from the underlying python-can library.

**Common Causes**:
- CAN adapter not connected or not detected by the system.
- Incorrect CAN interface type or channel specified for python-can.
- Driver issues for the CAN adapter.
- Physical CAN bus problems (e.g., wiring faults, missing termination, bus-off state).
- python-can bus exceptions during send/receive operations.

**Example**:
```python
# can_if_hw = CANInterface(interface_type="socketcan", channel="can0", use_simulator=False)
# try:
#     await can_if_hw.connect() 
# except exceptions.CANError as e:
#     print(f"CAN Bus Error: {e}")
#     print("Check CAN adapter, wiring, termination, and motor power.")
# finally:
#     if can_if_hw.is_connected:
#         await can_if_hw.disconnect()
```

### 3. SimulatorError

**Description**: Raised for issues specific to communication with the mks-servo-simulator.

**Common Causes**:
- Simulator is not running.
- Incorrect simulator host or port specified in CANInterface.
- Network issues preventing connection to the simulator.
- Simulator sends an unexpected or malformed response.

**Example**:
```python
# can_if_sim = CANInterface(use_simulator=True, simulator_host="wrong_host", simulator_port=12345)
# try:
#     await can_if_sim.connect()
# except exceptions.SimulatorError as e:
#     print(f"Simulator Connection Error: {e}")
#     print("Ensure the simulator is running and the host/port are correct.")
# finally:
#     if can_if_sim.is_connected:
#         await can_if_sim.disconnect()
```

### 4. MotorCommError

**Description**: A general communication error with a specific motor after a connection has been established. This indicates that a command was sent, but a valid or expected response was not received, or the response indicated a communication fault.

**Common Causes**:
- Motor powered off or disconnected from the CAN bus after initial connection.
- Incorrect CAN ID specified for an Axis object (motor doesn't exist at that ID).
- Severe noise on the CAN bus corrupting messages.
- Motor firmware issue.
- This is often a superclass for MotorTimeoutError.

**Example**:
```python
# async def motor_comm_example(axis: Axis): # Assuming axis is an initialized Axis object
#     try:
#         await axis.ping() # Or any other command to a specific motor
#         print(f"Communication with {axis.name} successful.")
#     except exceptions.MotorCommError as e: # Catches MotorTimeoutError too
#         print(f"Motor Communication Error with {axis.name}: {e}")
```

### 5. MotorTimeoutError

**Description**: A subclass of MotorCommError. Raised when a command is sent to a motor, but no response is received within the expected timeout period.

**Common Causes**:
- Motor is not responding (powered off, disconnected, wrong CAN ID).
- CAN bus issues preventing messages from reaching the motor or responses from returning.
- The motor is busy or in an error state where it cannot respond.
- Timeout value is too short for the operation or bus latency.

**Example**:
```python
# async def motor_timeout_example(axis: Axis): # Assuming axis is an initialized Axis object
#     try:
#         await axis.get_current_position()
#         print(f"Position read from {axis.name} successfully.")
#     except exceptions.MotorTimeoutError:
#         print(f"Timeout: Motor {axis.name} did not respond in time.")
#     except exceptions.MotorCommError as e: # Catch other comm errors if not a timeout
#         print(f"Communication Error with {axis.name}: {e}")
```

### 6. MotorOperationError

**Description**: Raised when a motor responds to a command, but the response indicates that the requested operation could not be performed successfully or the motor is in a state that prevents the operation.

**Common Causes**:
- Trying to move a motor that is not enabled.
- Command parameters are out of the motor's acceptable range.
- Motor is in an error state (e.g., over-voltage, stall) that prevents the operation.
- The command itself is valid, but the motor's internal logic or current state prohibits it.

**Example**:
```python
# async def motor_op_error_example(axis: Axis): # Assuming axis is initialized but not enabled
#     try:
#         await axis.move_relative(10, 10) 
#     except exceptions.MotorOperationError as e:
#         print(f"Motor Operation Error for {axis.name}: {e}")
#         print("Check if the motor is enabled and not in an error state.")
#         # You might want to check axis.error_code here:
#         # error_code = await axis.read_parameter(const.PARAM_SYSTEM_ERROR_CODE)
#         # print(f"Motor error code: {error_code} - {axis.get_error_description(error_code)}")
#     except exceptions.MotorTimeoutError:
#         print(f"Timeout: Motor {axis.name} did not respond.")
```

### 7. CRCError

**Description**: Raised if a received CAN message from a motor fails its CRC (Cyclic Redundancy Check) validation. This indicates data corruption during transmission.

**Common Causes**:
- Noise on the CAN bus.
- Faulty wiring or connectors.
- Issues with the CAN transceivers on the motor or adapter.

**Note**: CRC checking is optional and can be enabled/disabled in CANInterface. If disabled, this error won't be raised for CRC issues.

**Example**:
```python
# async def crc_error_example(axis: Axis): # Assuming CRC checking is enabled in CANInterface
#     try:
#         await axis.read_parameter(const.PARAM_FIRMWARE_VERSION)
#         print(f"Firmware read from {axis.name} successfully.")
#     except exceptions.CRCError as e:
#         print(f"CRC Error from {axis.name}: {e}. Data may be corrupted.")
```

## General Error Handling Strategies

### Use try...except...finally Blocks:
- Wrap CAN communication and motor control logic in try blocks.
- Catch specific exceptions first, then more general ones.
- Use a finally block to ensure resources are cleaned up (e.g., await can_interface.disconnect()).

### Check Motor Status:
- Before critical operations, or after an error, consider pinging the motor (axis.ping()) or updating its status (axis.update_status()).
- Check axis.error_code (after an update_status() or relevant read_parameter()) and axis.get_error_description() to understand if the motor itself has reported an internal fault (e.g., stall, over-voltage).

### Retry Logic:
- For transient issues like timeouts (MotorTimeoutError) or CRC errors, you might implement a limited retry mechanism with a short delay.
- Be cautious with retrying operations that modify state, as this could lead to unintended behavior if the first attempt partially succeeded.

### User Feedback:
- Provide clear feedback to the user about the nature of the error and potential troubleshooting steps.

### Logging:
- Implement logging to record errors, which can be invaluable for diagnosing problems, especially in deployed systems. The library itself uses Python's logging module.

## Example of a More Complete Error Handling Structure:

```python
async def robust_motor_operation(axis: Axis): # Assuming axis is an initialized Axis object
    try:
        print(f"Attempting operation on {axis.name}...")
        # It's good practice to ensure the motor is in a known state
        await axis.update_status() 
        if axis.error_code != 0:
            print(f"Motor {axis.name} has an existing error: {axis.get_error_description()}. Attempting to clear...")
            # await axis.clear_errors() # Assuming such a method exists or is implemented via write_parameter
            # await axis.update_status() # Re-check status
            # if axis.error_code != 0:
            #     print(f"Failed to clear errors on {axis.name}. Aborting operation.")
            #     return

        if not axis.is_enabled():
            print(f"Enabling {axis.name} first...")
            await axis.enable_motor()
        
        await axis.move_relative(distance=90, speed=30)
        print(f"{axis.name} move command sent successfully.")
        # Further operations...

    except exceptions.MotorTimeoutError as e:
        print(f"Timeout error with {axis.name}: {e}. Motor might be unresponsive.")
        # Consider trying to ping or update status to confirm
    except exceptions.MotorOperationError as e:
        # Update status to get the latest error code if the operation failed
        try:
            await axis.update_status()
            error_desc = axis.get_error_description()
        except exceptions.MKSServoCANError: # If update_status also fails
            error_desc = "Could not retrieve error description."
        print(f"Operation error with {axis.name}: {e}. Motor state: error_code={axis.error_code} ({error_desc})")
        # Attempt to clear error if applicable, or guide user.
    except exceptions.MotorCommError as e: # Catches other communication issues
        print(f"General communication error with {axis.name}: {e}")
    except exceptions.MKSServoCANError as e: # Catch any other library-specific error
        print(f"A mks-servo-can library error occurred: {e}")
    except Exception as e: # Catch any other unexpected error
        print(f"An unexpected non-library error occurred: {e}")
    finally:
        print(f"Finished operation attempt for {axis.name}.")
        # Don't disconnect CANInterface here if it's shared or will be reused.
        # Disconnection is usually handled at a higher application
```