# Reading Motor Status & Parameters

Understanding the current state of your MKS servo motor is crucial for robust control and debugging. The `Axis` class provides several methods to read various status indicators and system parameters.

## Prerequisites

* An `Axis` instance that is initialized and connected. See [Basic Motor Control](./basic_control.md).
* The motor should ideally be powered on and responsive (e.g., successfully pinged).

## Core Concepts: Polling and Cached Status

The `Axis` object maintains an internal cache of the motor's status (e.g., position, speed, enabled state, error codes). This cache is updated when:
1. Specific read commands are issued (e.g., `get_current_position()`).
2. A general status update is performed using `update_status()`.
3. Some commands that inherently return status (like `enable()` or `disable()`) might also update parts of the cache.

Relying solely on the cache without periodic updates can lead to stale data. Therefore, it's often necessary to explicitly request fresh information from the motor.

## 1. Getting Current Position (`get_current_position`)

Retrieves the motor's current position.

```python
async def get_position_example(axis: Axis):
    try:
        # Get position using kinematics (e.g., degrees, mm)
        current_pos_kin = await axis.get_current_position(use_kinematics=True)
        print(f"{axis.name} current position ({axis.kinematics.units}): {current_pos_kin:.2f}")

        # Get position in raw motor steps
        current_pos_steps = await axis.get_current_position(use_kinematics=False)
        print(f"{axis.name} current position (raw steps): {current_pos_steps}")

        # The Axis object also stores the last known position
        # This might be slightly older than a direct call if no recent poll occurred
        print(f"{axis.name} cached position (kin): {axis.current_position:.2f}")
        print(f"{axis.name} cached position (steps): {axis.current_position_steps}")

    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to position query.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error reading position for {axis.name}: {e}")

# --- Example Usage (assuming 'motor1' is an initialized Axis object) ---
# await get_position_example(motor1)
```

`use_kinematics` (boolean, default True): Determines if the returned position is converted using the axis's kinematics settings or returned as raw encoder steps.

This method actively polls the motor for its current position. The result also updates `axis.current_position` and `axis.current_position_steps`.

## 2. Getting Current Speed (`get_current_speed`)

Retrieves the motor's current speed.

```python
async def get_speed_example(axis: Axis):
    try:
        # Get speed using kinematics (e.g., degrees/s, mm/s)
        current_speed_kin = await axis.get_current_speed(use_kinematics=True)
        print(f"{axis.name} current speed ({axis.kinematics.units}/s): {current_speed_kin:.2f}")

        # Get speed in raw motor steps/s
        current_speed_steps = await axis.get_current_speed(use_kinematics=False)
        print(f"{axis.name} current speed (raw steps/s): {current_speed_steps}")
        
        # Cached speed
        print(f"{axis.name} cached speed (kin): {axis.current_speed:.2f}")
        print(f"{axis.name} cached speed (steps/s): {axis.current_speed_steps_per_sec}")


    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to speed query.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error reading speed for {axis.name}: {e}")

# --- Example Usage ---
# await motor1.set_speed_mode(20) # Start motor moving
# await asyncio.sleep(0.5)
# await get_speed_example(motor1)
# await motor1.stop_movement()
```

`use_kinematics` (boolean, default True): Similar to `get_current_position`.

This method actively polls the motor. The result updates `axis.current_speed` and `axis.current_speed_steps_per_sec`.

## 3. Checking Enabled State (`is_enabled`)

Returns `True` if the motor is currently enabled (servo on), `False` otherwise. This method typically returns the cached state. To get the latest state, call `await axis.update_status()` or a command like `await axis.ping()` first.

```python
async def check_enabled_state_example(axis: Axis):
    # It's good practice to update status before checking if you need the absolute latest
    # await axis.update_status() 
    # or await axis.ping() as ping also updates the enabled flag.
    
    # For critical checks, directly read the enable status parameter
    try:
        is_actually_enabled = await axis.read_parameter(const.PARAM_SYSTEM_ENABLE_STATUS) == 1
        print(f"{axis.name} direct read - Enabled: {is_actually_enabled}")
    except exceptions.MKSServoCANError as e:
        print(f"Error reading direct enable status: {e}")

    # Check cached status
    if axis.is_enabled():
        print(f"{axis.name} (cached) is Enabled.")
    else:
        print(f"{axis.name} (cached) is Disabled.")


# --- Example Usage ---
# await motor1.enable()
# await check_enabled_state_example(motor1)
# await motor1.disable()
# await check_enabled_state_example(motor1)
```

The `axis.is_enabled_flag` attribute holds the cached boolean value.

## 4. Checking Movement Status (`is_moving`, `is_target_reached`)

These properties provide information about the motor's movement state based on the last known status.

- `axis.is_moving`: Returns `True` if the motor is believed to be currently in motion.
- `axis.is_target_reached`: Returns `True` if the motor believes it has reached its last commanded target position.

```python
async def check_movement_status_example(axis: Axis):
    # Update status to get fresh values
    await axis.update_status() 
    
    if axis.is_moving():
        print(f"{axis.name} is currently moving.")
    else:
        print(f"{axis.name} is not moving.")

    if axis.is_target_reached():
        print(f"{axis.name} has reached its target.")
    else:
        print(f"{axis.name} has not reached its target (or no target was set).")

# --- Example Usage ---
# await motor1.enable()
# move_future = motor1.move_relative(180, 30, wait_for_completion=False)
# await asyncio.sleep(0.5) # Give it time to start moving
# await check_movement_status_example(motor1)
# await move_future # Wait for move to complete
# await check_movement_status_example(motor1)
# await motor1.disable()
```

These flags are typically updated by `update_status()` or after movement commands that poll status.

## 5. Comprehensive Status Update (`update_status`)

This method polls the motor for a block of common status parameters, including position, speed, enabled state, error codes, and movement flags. It updates the Axis object's internal cache.

```python
async def update_and_show_status_example(axis: Axis):
    try:
        print(f"Updating full status for {axis.name}...")
        await axis.update_status()
        print(f"Status for {axis.name}:")
        print(f"  Enabled: {axis.is_enabled()}")
        print(f"  Moving: {axis.is_moving()}")
        print(f"  Target Reached: {axis.is_target_reached()}")
        print(f"  Position ({axis.kinematics.units}): {axis.current_position:.2f}")
        print(f"  Speed ({axis.kinematics.units}/s): {axis.current_speed:.2f}")
        print(f"  Error Code: {axis.error_code} (Description: {axis.get_error_description()})")
        print(f"  System Voltage: {axis.system_voltage:.2f}V") # If available
        print(f"  System Temperature: {axis.system_temperature}°C") # If available

    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to status update query.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error updating status for {axis.name}: {e}")

# --- Example Usage ---
# await update_and_show_status_example(motor1)
```

Calling `update_status()` is a good way to refresh multiple status attributes at once.

## 6. Reading Specific Motor Parameters (`read_parameter`)

MKS servo motors expose many internal parameters via CANopen-like object dictionary entries (though not strictly CANopen). The `read_parameter(param_id)` method allows you to read any of these if you know its parameter ID. Parameter IDs are defined in `mks_servo_can.constants`.

```python
from mks_servo_can import const # For PARAM_* constants

async def read_specific_parameters_example(axis: Axis):
    try:
        # Read Firmware Version (Parameter ID 0x1008 in MKS manual, mapped to const.PARAM_FIRMWARE_VERSION)
        firmware_version_raw = await axis.read_parameter(const.PARAM_FIRMWARE_VERSION)
        # Firmware version is often returned as an integer like V106 for V1.0.6
        print(f"{axis.name} Firmware Version (raw): {firmware_version_raw}") 
        # You might need to format this value for display, e.g. V{fw//100}.{(fw%100)//10}.{fw%10}

        # Read CAN ID (Parameter ID 0x2000, const.PARAM_CAN_ID)
        can_id_read = await axis.read_parameter(const.PARAM_CAN_ID)
        print(f"{axis.name} CAN ID (read from motor): {can_id_read}")

        # Read System Error Code (Parameter ID 0x2003, const.PARAM_SYSTEM_ERROR_CODE)
        error_code = await axis.read_parameter(const.PARAM_SYSTEM_ERROR_CODE)
        print(f"{axis.name} System Error Code: {error_code} - {axis.get_error_description(error_code)}")
        # This also updates axis.error_code

    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to parameter read.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error reading parameter for {axis.name}: {e}")

# --- Example Usage ---
# await read_specific_parameters_example(motor1)
```

Refer to `mks_servo_can/constants.py` and the MKS Servo Motor User Manual for a list of available parameter IDs and their meanings.

## 7. Accessing Cached Status Attributes

Many status values, once read or updated, are available as direct attributes of the Axis object:

- `axis.current_position`: Last known position (kinematic units).
- `axis.current_position_steps`: Last known position (raw steps).
- `axis.current_speed`: Last known speed (kinematic units/s).
- `axis.current_speed_steps_per_sec`: Last known speed (raw steps/s).
- `axis.is_enabled_flag`: Cached boolean for enabled state.
- `axis.is_moving_flag`: Cached boolean for moving state.
- `axis.is_target_reached_flag`: Cached boolean for target reached state.
- `axis.error_code`: Last read error code.
- `axis.system_voltage`: Last read system voltage (if update_status polled it).
- `axis.system_temperature`: Last read system temperature (if update_status polled it).

Accessing these attributes is fast as it doesn't involve CAN communication, but remember the data might be stale if not recently updated.

## 8. Understanding Error Codes (`get_error_description`)

When an error code is read (e.g., via `update_status()` or `read_parameter(const.PARAM_SYSTEM_ERROR_CODE)`), you can get a human-readable description using `axis.get_error_description()`:

```python
# Assuming axis.error_code has been populated
error_desc = axis.get_error_description() 
# OR provide a code directly: error_desc = axis.get_error_description(some_error_code)
print(f"Error description for code {axis.error_code}: {error_desc}")
```

The descriptions are based on `mks_servo_can.constants.ERROR_DESCRIPTIONS`.