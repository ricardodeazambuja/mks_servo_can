# Movement Commands

The `Axis` class provides several methods for commanding motor movement. This guide covers the different types of movements available: absolute positioning, relative movements, and continuous speed control.

## Prerequisites

* An `Axis` instance that is initialized and connected. See [Basic Motor Control](./basic_control.md).
* The motor must be **enabled** using `await axis.enable_motor()` before movement commands will be accepted.
* Understanding of [Kinematics](./kinematics.md) for unit conversions is recommended but not required.

## Movement Method Overview

The `Axis` class provides these main movement methods:
- `move_to_position_abs_user()` - Move to absolute position in user units
- `move_to_position_abs_pulses()` - Move to absolute position in motor steps
- `move_relative_user()` - Move relative distance in user units
- `move_relative_pulses()` - Move relative distance in motor steps

## Common Parameters

Most movement methods share these parameters:
* `speed_user` or `speed_pulses_per_sec`: Movement speed in user units/sec or pulses/sec
* `accel_user` or `accel_pulses_per_sec2` (optional): Acceleration in user units/sec² or pulses/sec²
* `wait` (optional, default `True`): If `True`, waits for movement completion; if `False`, returns immediately

## 1. Absolute Movement (User Units)

Move to a specific position using user-friendly units (degrees, mm, etc.):

```python
async def absolute_movement_example(axis: Axis):
    try:
        # Get current position first
        current_pos = await axis.get_current_position_user()
        print(f"Current position: {current_pos:.2f}°")
        
        # Move to 90 degrees at 30 degrees/second
        print("Moving to 90°...")
        await axis.move_to_position_abs_user(
            target_position_user=90.0,
            speed_user=30.0,
            wait=True  # Wait for completion
        )
        
        # Check final position
        final_pos = await axis.get_current_position_user()
        print(f"Movement complete. Final position: {final_pos:.2f}°")
        
    except exceptions.MotorError as e:
        print(f"Motor error during movement: {e}")
    except exceptions.CommunicationError as e:
        print(f"Communication error: {e}")

# Example usage:
# await absolute_movement_example(axis)
```

## 2. Relative Movement (User Units)

Move by a specific distance from the current position:

```python
async def relative_movement_example(axis: Axis):
    try:
        # Get starting position
        start_pos = await axis.get_current_position_user()
        print(f"Starting position: {start_pos:.2f}°")
        
        # Move 45 degrees clockwise (positive direction)
        print("Moving +45° relative...")
        await axis.move_relative_user(
            distance_user=45.0,
            speed_user=20.0,
            wait=True
        )
        
        # Check new position
        new_pos = await axis.get_current_position_user()
        print(f"After +45°: {new_pos:.2f}°")
        
        # Move 90 degrees counter-clockwise (negative direction)
        print("Moving -90° relative...")
        await axis.move_relative_user(
            distance_user=-90.0,
            speed_user=20.0,
            wait=True
        )
        
        # Check final position
        final_pos = await axis.get_current_position_user()
        print(f"After -90°: {final_pos:.2f}°")
        
    except exceptions.MotorError as e:
        print(f"Motor error during relative movement: {e}")

# Example usage:
# await relative_movement_example(axis)
```

## 3. Raw Pulse Movement

For precise control or when working without kinematics, you can command movements in raw motor pulses:

```python
async def pulse_movement_example(axis: Axis):
    try:
        # MKS servos typically have 16384 pulses per revolution
        # So 4096 pulses = 90 degrees
        target_pulses = 4096  # 90 degrees
        speed_pulses_per_sec = 1000  # Moderate speed
        
        print(f"Moving to {target_pulses} pulses...")
        await axis.move_to_position_abs_pulses(
            target_position_pulses=target_pulses,
            speed_pulses_per_sec=speed_pulses_per_sec,
            wait=True
        )
        
        # Check position in both pulses and user units
        pos_pulses = await axis.get_current_position_steps()
        pos_user = await axis.get_current_position_user()
        print(f"Final position: {pos_pulses} pulses ({pos_user:.2f}°)")
        
    except exceptions.MotorError as e:
        print(f"Motor error during pulse movement: {e}")

# Example usage:
# await pulse_movement_example(axis)
```

## 4. Non-blocking Movement

Start a movement and do other work while the motor moves:

```python
async def non_blocking_movement_example(axis: Axis):
    try:
        print("Starting non-blocking movement...")
        
        # Start movement but don't wait for completion
        await axis.move_to_position_abs_user(
            target_position_user=180.0,
            speed_user=15.0,  # Slower speed for longer movement
            wait=False  # Don't wait - return immediately
        )
        
        print("Movement started, doing other work...")
        
        # Do other work while motor moves
        for i in range(10):
            await asyncio.sleep(0.5)
            
            # Check if movement is complete
            status = await axis.get_current_status()
            position = await axis.get_current_position_user()
            
            print(f"  Step {i+1}: Position = {position:.1f}°")
            
            # Check if motor has reached target
            if abs(position - 180.0) < 1.0:  # Within 1 degree
                print("Movement completed!")
                break
        
    except exceptions.MotorError as e:
        print(f"Motor error during non-blocking movement: {e}")

# Example usage:
# await non_blocking_movement_example(axis)
```

## 5. Movement with Custom Acceleration

Control acceleration and deceleration for smooth movements:

```python
async def custom_acceleration_example(axis: Axis):
    try:
        print("Movement with custom acceleration...")
        
        await axis.move_to_position_abs_user(
            target_position_user=360.0,  # Full rotation
            speed_user=45.0,             # 45 degrees/second
            accel_user=90.0,             # 90 degrees/second²
            wait=True
        )
        
        print("Smooth movement complete!")
        
    except exceptions.MotorError as e:
        print(f"Motor error during accelerated movement: {e}")

# Example usage:
# await custom_acceleration_example(axis)
```

## 6. Emergency Stop

Stop the motor immediately:

```python
async def emergency_stop_example(axis: Axis):
    try:
        # Start a long movement
        print("Starting long movement...")
        await axis.move_to_position_abs_user(
            target_position_user=720.0,  # Two full rotations
            speed_user=10.0,              # Slow speed for long movement
            wait=False  # Don't wait
        )
        
        # Let it run for a bit
        await asyncio.sleep(2.0)
        
        # Emergency stop
        print("Emergency stop!")
        await axis.emergency_stop()
        
        # Check where it stopped
        final_pos = await axis.get_current_position_user()
        print(f"Stopped at position: {final_pos:.2f}°")
        
    except exceptions.MotorError as e:
        print(f"Motor error during emergency stop: {e}")

# Example usage:
# await emergency_stop_example(axis)
```

## 7. Speed Limits and Safety

Always consider speed limits based on your mechanical system:

```python
async def safe_movement_example(axis: Axis):
    # Define safe limits for your system
    MAX_SPEED_USER = 60.0      # degrees/second
    MAX_ACCEL_USER = 120.0     # degrees/second²
    
    def clamp_speed(speed):
        return min(abs(speed), MAX_SPEED_USER) * (1 if speed >= 0 else -1)
    
    def clamp_accel(accel):
        return min(abs(accel), MAX_ACCEL_USER)
    
    try:
        # User requests fast movement
        requested_speed = 100.0    # Too fast!
        requested_accel = 200.0    # Too fast!
        
        # Apply safety limits
        safe_speed = clamp_speed(requested_speed)
        safe_accel = clamp_accel(requested_accel)
        
        print(f"Requested: {requested_speed}°/s, {requested_accel}°/s²")
        print(f"Using safe: {safe_speed}°/s, {safe_accel}°/s²")
        
        await axis.move_to_position_abs_user(
            target_position_user=90.0,
            speed_user=safe_speed,
            accel_user=safe_accel,
            wait=True
        )
        
        print("Safe movement complete!")
        
    except exceptions.MotorError as e:
        print(f"Motor error: {e}")

# Example usage:
# await safe_movement_example(axis)
```

## Complete Movement Example

Here's a comprehensive example showing different movement types:

```python
async def complete_movement_demo():
    # Setup (assuming simulator is running)
    can_interface = CANInterface()
    await can_interface.connect()
    
    # Create axis with rotary kinematics (degrees)
    kinematics = RotaryKinematics(steps_per_revolution=16384)
    axis = Axis(can_interface, motor_can_id=1, kinematics=kinematics)
    
    try:
        # Initialize and enable
        await axis.initialize()
        await axis.enable_motor()
        
        print("=== Movement Demo ===")
        
        # 1. Absolute movements
        print("1. Absolute movements...")
        await axis.move_to_position_abs_user(90.0, speed_user=30.0)
        await asyncio.sleep(1)
        await axis.move_to_position_abs_user(180.0, speed_user=30.0)
        await asyncio.sleep(1)
        
        # 2. Relative movements
        print("2. Relative movements...")
        await axis.move_relative_user(90.0, speed_user=20.0)   # +90°
        await asyncio.sleep(1)
        await axis.move_relative_user(-180.0, speed_user=20.0) # -180°
        await asyncio.sleep(1)
        
        # 3. Return to zero
        print("3. Returning to zero...")
        await axis.move_to_position_abs_user(0.0, speed_user=45.0)
        
        print("Movement demo complete!")
        
    except Exception as e:
        print(f"Error during demo: {e}")
    finally:
        # Cleanup
        try:
            await axis.disable_motor()
        except:
            pass
        await can_interface.disconnect()

# Run the demo:
# asyncio.run(complete_movement_demo())
```

## Best Practices

1. **Always check motor status** before commanding movements
2. **Use appropriate speeds** for your mechanical system
3. **Handle timeouts gracefully** - motors may not reach target due to mechanical issues
4. **Use emergency stop** when needed for safety
5. **Disable motors** when finished to save power and allow manual movement
6. **Consider using user units** (degrees, mm) instead of raw pulses for readability

## Next Steps

* Learn about [Multi-Axis Control](./multi_axis.md) for coordinating multiple motors
* Explore [Robot Kinematics](./robot_control.md) for higher-level robot control
* Check out [Error Handling](./error_handling.md) for robust error management