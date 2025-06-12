# Basic Motor Control (`Axis` class)

Once you have a connected `CANInterface` instance, the next step is to interact with individual motors. The `Axis` class provides a high-level abstraction for controlling a single MKS servo motor. This guide covers the fundamental operations: creating an `Axis` instance, initializing it, and controlling the motor.

## Prerequisites

* A successfully connected `CANInterface` instance (either to the simulator or real hardware). See [Connecting to Motors (Hardware & Simulator)](./connecting.md).
* Knowledge of the CAN ID of the motor you wish to control.

## Importing Required Classes

First, ensure you import the necessary components:

```python
import asyncio
from mks_servo_can import CANInterface, Axis, exceptions
from mks_servo_can.kinematics import RotaryKinematics, LinearKinematics
```

## Creating an Axis Instance

To control a motor, you need to create an `Axis` object. It requires a connected `CANInterface`, the `can_id` of the target motor, and optionally a kinematics object for unit conversions.

```python
async def create_axis_example(can_interface: CANInterface, motor_can_id: int):
    # Create kinematics for unit conversion (degrees for rotary motion)
    kinematics = RotaryKinematics(steps_per_revolution=16384)  # MKS servo default
    
    # Create the axis instance
    axis = Axis(
        can_interface=can_interface,
        motor_can_id=motor_can_id,
        name=f"Motor_{motor_can_id}",  # Optional, for easier identification
        kinematics=kinematics
    )
    
    print(f"Axis '{axis.name}' created for CAN ID {axis.motor_can_id}")
    return axis

# Example usage:
# axis = await create_axis_example(can_interface, 1)
```

## Key Parameters for Axis Creation:
* `can_interface`: The active and connected CANInterface instance.
* `motor_can_id`: The unique CAN identifier of the motor (integer, typically 1-254).
* `name` (optional): A string name for the axis. Defaults to `f"Axis_{motor_can_id}"`.
* `kinematics` (optional): A Kinematics object for unit conversions. If not provided, a default RotaryKinematics instance is used.

## Initializing the Axis

Before using the motor, you should initialize the axis. This performs communication tests and optionally calibrates the encoder and homes the axis:

```python
async def initialize_axis_example(axis: Axis):
    try:
        print(f"Initializing {axis.name}...")
        
        # Basic initialization (communication test only)
        await axis.initialize(calibrate=False, home=False)
        print(f"{axis.name} initialized successfully.")
        
        return True
        
    except exceptions.CommunicationError as e:
        print(f"Communication error with {axis.name}: {e}")
        print("Check CAN wiring and motor power.")
        return False
    except exceptions.MotorError as e:
        print(f"Motor error with {axis.name}: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error initializing {axis.name}: {e}")
        return False

# Example usage:
# if await initialize_axis_example(axis):
#     print(f"{axis.name} is ready for use.")
```

## Advanced Initialization

For more thorough setup, you can include encoder calibration and homing:

```python
async def initialize_axis_advanced(axis: Axis):
    try:
        print(f"Performing advanced initialization of {axis.name}...")
        
        # Full initialization: communication test + calibration + homing
        await axis.initialize(calibrate=True, home=True)
        print(f"{axis.name} fully initialized (calibrated and homed).")
        
        return True
        
    except exceptions.CalibrationError as e:
        print(f"Calibration failed for {axis.name}: {e}")
        return False
    except exceptions.HomingError as e:
        print(f"Homing failed for {axis.name}: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error during advanced initialization: {e}")
        return False
```

## Enabling and Disabling the Motor

After initialization, you need to enable the motor before it will accept movement commands:

```python
async def motor_enable_disable_example(axis: Axis):
    try:
        # Check current status
        current_status = await axis.get_current_status()
        print(f"{axis.name} enabled status: {current_status.get('enabled', 'unknown')}")
        
        # Enable the motor
        if not await axis.is_enabled():
            print(f"Enabling {axis.name}...")
            await axis.enable_motor()
            print(f"{axis.name} enabled successfully.")
        else:
            print(f"{axis.name} is already enabled.")
        
        # Do some work with the motor here...
        await asyncio.sleep(1)
        
        # Disable the motor when done
        print(f"Disabling {axis.name}...")
        await axis.disable_motor()
        print(f"{axis.name} disabled successfully.")
        
        return True
        
    except exceptions.MotorError as e:
        print(f"Motor operation error: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False
```

## Checking Motor Status

You can query various motor parameters and status:

```python
async def check_motor_status(axis: Axis):
    try:
        # Get current position in user units (degrees for rotary)
        position_user = await axis.get_current_position_user()
        print(f"{axis.name} current position: {position_user:.2f}°")
        
        # Get current position in raw steps
        position_steps = await axis.get_current_position_steps()
        print(f"{axis.name} current position: {position_steps} steps")
        
        # Check if motor is enabled
        enabled = await axis.is_enabled()
        print(f"{axis.name} enabled: {enabled}")
        
        # Get comprehensive status
        status = await axis.get_current_status()
        print(f"{axis.name} status: {status}")
        
    except exceptions.CommunicationError as e:
        print(f"Communication error: {e}")
    except Exception as e:
        print(f"Error checking status: {e}")
```

## Complete Example

Here's a complete example demonstrating the typical sequence:

```python
async def complete_basic_control_example():
    # 1. Connect to CAN interface (simulator or hardware)
    can_interface = CANInterface()  # Auto-detect simulator or use hardware
    
    try:
        await can_interface.connect()
        print("CAN Interface connected.")
        
        # 2. Create and initialize axis
        kinematics = RotaryKinematics(steps_per_revolution=16384)
        axis = Axis(can_interface, motor_can_id=1, name="TestMotor", kinematics=kinematics)
        
        if not await initialize_axis_example(axis):
            print("Failed to initialize axis. Aborting.")
            return
        
        # 3. Enable the motor
        await axis.enable_motor()
        print(f"{axis.name} is now enabled and ready.")
        
        # 4. Check current status
        await check_motor_status(axis)
        
        # 5. Disable motor when done
        await axis.disable_motor()
        print(f"{axis.name} has been disabled.")
        
    except exceptions.CANError as e:
        print(f"CAN interface error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if can_interface.is_connected:
            await can_interface.disconnect()

# Run the example
# asyncio.run(complete_basic_control_example())
```

## Next Steps

Once you have basic motor control working:
* Learn about [Movement Commands](./movements.md) to actually move the motor
* Explore [Kinematics](./kinematics.md) for different unit systems
* Check out [Multi-Axis Control](./multi_axis.md) for coordinating multiple motors

## Common Patterns

### Simple Setup for Quick Testing
```python
# Quick setup for testing with simulator
async def quick_axis_setup(motor_id=1):
    can_if = CANInterface()
    await can_if.connect()
    
    axis = Axis(can_if, motor_id, kinematics=RotaryKinematics())
    await axis.initialize()
    await axis.enable_motor()
    
    return can_if, axis

# Usage:
# can_if, axis = await quick_axis_setup(1)
# # ... use axis ...
# await can_if.disconnect()
```

### Error Handling Best Practices
```python
async def robust_axis_operation(axis: Axis):
    try:
        await axis.enable_motor()
        # ... do motor operations ...
        
    except exceptions.MotorError as e:
        print(f"Motor error: {e}")
        # Handle motor-specific errors
    except exceptions.CommunicationError as e:
        print(f"Communication error: {e}")
        # Handle communication timeouts/failures
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Always try to disable motor safely
        try:
            await axis.disable_motor()
        except:
            pass  # Motor may already be disabled or disconnected
```