# Movement Commands

The `Axis` class provides several methods for commanding motor movement, catering to different use cases such as moving to a specific position, moving by a certain amount, or running at a continuous speed. This guide details these movement commands.

## Prerequisites

* An `Axis` instance that is initialized and connected. See [Basic Motor Control](./basic_control.md).
* The motor must be **enabled** using `await axis.enable_motor()` before most movement commands will be accepted.
* Understanding of how `Kinematics` are used if you intend to work with physical units (degrees, mm) rather than raw motor steps. See [Using Kinematics](./kinematics.md). For simplicity, some examples here will use raw steps, but using kinematics for unit conversion is highly recommended for most applications.

## Common Parameters for Movement

Many movement commands share common parameters:

* `speed`: The desired speed for the movement. The units depend on the active `Kinematics` object (e.g., degrees/sec, mm/sec) or steps/sec if no specific kinematics are used or if `use_kinematics=False`.
* `acceleration` (optional): The desired acceleration for the movement. Units also depend on kinematics (e.g., degrees/sec^2, mm/sec^2) or steps/sec^2. If not provided, the motor's pre-configured acceleration or a library default might be used.
* `deceleration` (optional): The desired deceleration. Similar to acceleration. Often, if not specified, it defaults to the acceleration value.
* `use_kinematics` (optional, boolean): Defaults to `True`. If `True`, position, speed, and acceleration values are interpreted according to the `Axis` object's `kinematics` settings (e.g., degrees, mm). If `False`, values are interpreted as raw motor encoder steps. [cite: mks_servo_can_library/mks_servo_can/axis.py]
* `wait_for_completion` (optional, boolean): Defaults to `True`. If `True`, the method will wait until the motor reports that the movement is complete (or a timeout occurs). If `False`, the command is sent, and the method returns immediately, allowing other operations to proceed while the motor moves. [cite: mks_servo_can_library/mks_servo_can/axis.py]
* `timeout_ms` (optional, int): Specific timeout in milliseconds for this command. Overrides the `Axis` default timeout.

## 1. Absolute Movement (`move_absolute`)

Moves the motor to a specific target position.

```python
async def move_absolute_example(axis: Axis, target_position: float, speed: float):
    try:
        print(f"Moving {axis.name} to absolute position {target_position} at speed {speed} (units via kinematics).")
        # Assuming axis.kinematics is set, e.g., to RotaryKinematics for degrees
        await axis.move_absolute(
            target_position=target_position,
            speed=speed,
            # acceleration=accel, # Optional
            wait_for_completion=True
        )
        current_pos = await axis.get_current_position()
        print(f"{axis.name} move complete. Current position: {current_pos:.2f}")
    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not complete absolute move.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error during absolute move for {axis.name}: {e}")

# --- Example Usage (assuming 'motor1' is an enabled Axis object) ---
# await motor1.set_kinematics(RotaryKinematics(steps_per_revolution=16384)) # Example
# await move_absolute_example(motor1, target_position=90.0, speed=30.0) # Move to 90 degrees at 30 deg/s
# await move_absolute_example(motor1, target_position=0.0, speed=30.0)   # Move back to 0 degrees
```

If `use_kinematics=False`, `target_position` would be in raw motor steps.

## 2. Relative Movement (move_relative)

Moves the motor by a specified amount from its current position.
```python
async def move_relative_example(axis: Axis, distance: float, speed: float):
    try:
        print(f"Moving {axis.name} by relative distance {distance} at speed {speed} (units via kinematics).")
        initial_pos = await axis.get_current_position(use_kinematics=True)
        print(f"Initial position: {initial_pos:.2f}")

        await axis.move_relative(
            distance=distance,
            speed=speed,
            wait_for_completion=True
        )
        final_pos = await axis.get_current_position(use_kinematics=True)
        print(f"{axis.name} relative move complete. Final position: {final_pos:.2f}")
        print(f"Actual distance moved: {final_pos - initial_pos:.2f}")

    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not complete relative move.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error during relative move for {axis.name}: {e}")

# --- Example Usage ---
# await motor1.set_kinematics(RotaryKinematics(steps_per_revolution=16384)) # Example
# await move_relative_example(motor1, distance=45.0, speed=20.0)  # Move 45 degrees forward
# await asyncio.sleep(1)
# await move_relative_example(motor1, distance=-45.0, speed=20.0) # Move 45 degrees backward
```
A positive distance moves in one direction, and a negative distance moves in the opposite. If `use_kinematics=False`, distance is in raw motor steps.

## 3. Speed Control Mode (`set_speed_mode`)
Commands the motor to run continuously at a specified speed until explicitly stopped or another movement command is issued. This is also known as velocity mode or jog mode.
```python
async def speed_mode_example(axis: Axis, target_speed: float):
    try:
        print(f"Setting {axis.name} to speed mode with target speed {target_speed} (units via kinematics).")
        await axis.set_speed_mode(
            speed=target_speed
            # acceleration=accel # Optional
        )
        print(f"{axis.name} is now running in speed mode at {target_speed}.")
        
        # Let it run for a few seconds
        await asyncio.sleep(3)
        
        print(f"Stopping {axis.name} from speed mode...")
        await axis.stop_movement() # or axis.set_speed_mode(speed=0)
        print(f"{axis.name} stopped.")
        
    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to speed mode command.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error during speed mode for {axis.name}: {e}")

# --- Example Usage ---
# await motor1.set_kinematics(RotaryKinematics(steps_per_revolution=16384)) # Example
# await speed_mode_example(motor1, target_speed=50.0)  # Run at 50 degrees/sec
# await asyncio.sleep(1)
# await speed_mode_example(motor1, target_speed=-25.0) # Run at -25 degrees/sec (reverse)
```
To stop a motor running in speed mode, you can call `axis.set_speed_mode(speed=0)` or `axis.stop_movement()`.

## 4. Stopping Movement (`stop_movement`)
This command attempts to halt any ongoing motor movement immediately. It's useful as an emergency stop or to interrupt a long move.
```python
async def stop_movement_example(axis: Axis):
    try:
        print(f"Attempting to stop any movement on {axis.name}.")
        await axis.stop_movement()
        print(f"{axis.name} stop command sent.")
        # Note: After a stop, the motor might be in an intermediate position.
        # You might want to query its current position.
        current_pos = await axis.get_current_position()
        print(f"Position after stop: {current_pos:.2f}")
    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to stop command.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error stopping {axis.name}: {e}")

# --- Example Usage (e.g., after a non-waiting move or speed mode) ---
# await motor1.move_absolute(target_position=1000, speed=10, wait_for_completion=False)
# await asyncio.sleep(0.5) # Let it move a bit
# await stop_movement_example(motor1)
```
The `stop_movement()` command typically uses the motor's configured deceleration (or a default) to ramp down the speed.

## `wait_for_completion` and Asynchronous Movement
When `wait_for_completion=False` is used with `move_absolute` or `move_relative`, the method returns an `asyncio.Future`. You can await this future later if you need to ensure the move has finished. This allows for concurrent operations:
```python
async def concurrent_move_example(axis1: Axis, axis2: Axis):
    print("Starting concurrent moves...")
    
    # Start moves without waiting
    future1 = axis1.move_relative(distance=90, speed=30, wait_for_completion=False)
    future2 = axis2.move_relative(distance=-90, speed=30, wait_for_completion=False)
    
    print("Move commands sent. Doing other work...")
    await asyncio.sleep(1) # Simulate other operations
    print("Other work done. Now waiting for moves to complete.")
    
    try:
        await future1 # Wait for axis1 to finish
        print(f"{axis1.name} completed its move.")
        await future2 # Wait for axis2 to finish
        print(f"{axis2.name} completed its move.")
    except exceptions.MotorTimeoutError:
        print("A motor timed out during concurrent move.")
    except exceptions.MotorOperationError as e:
        print(f"Operation Error during concurrent move: {e}")

# --- Example (requires two Axis objects, motor1 and motor2) ---
# await concurrent_move_example(motor1, motor2)
```

## Important Considerations
* Motor Limits: This library layer does not inherently manage software limits. It's the application's responsibility to ensure commanded positions are within safe mechanical ranges.
* Clearing Errors: If a motor encounters an error (e.g., stall), it might need to be explicitly cleared of errors and re-enabled before accepting new movement commands. See `Reading Motor Status` & `Parameters and Error Handling & Exceptions`.
* Tuning: Movement smoothness and accuracy can depend on the motor's PID tuning and configured acceleration/deceleration parameters. These are typically set using MKS configuration software or potentially via low-level CAN commands if supported.

This guide provides the foundation for controlling motor movement. For more advanced scenarios, such as coordinating multiple axes or implementing custom unit conversions, refer to the subsequent guides