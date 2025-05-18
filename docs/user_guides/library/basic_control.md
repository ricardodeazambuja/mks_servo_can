# Basic Motor Control (`Axis` class)

Once you have a connected `CANInterface` instance, the next step is to interact with individual motors. The `Axis` class provides a high-level abstraction for controlling a single MKS servo motor. This guide covers the fundamental operations: initializing an `Axis` instance, enabling/disabling the motor, and checking its responsiveness (pinging).

## Prerequisites

* A successfully connected `CANInterface` instance (either to the simulator or real hardware). See [Connecting to Motors (Hardware & Simulator)](./connecting.md).
* Knowledge of the CAN ID of the motor you wish to control.

## Importing `Axis`

First, ensure you import the `Axis` class along with other necessary components:

```python
import asyncio
from mks_servo_can import CANInterface, Axis, exceptions, const
# Assuming you might also use Kinematics later
from mks_servo_can.kinematics import RotaryKinematics
```

## Initializing an Axis Instance
To control a motor, you need to create an Axis object. It requires a connected `CANInterface` and the `can_id` of the target motor. You can optionally provide a name for easier identification and a kinematics object for unit conversions (covered in a later guide).

```python
async def initialize_axis_example(can_interface: CANInterface, motor_can_id: int):
    # For this basic example, we'll use default RotaryKinematics
    # You might use LinearKinematics or a custom one depending on your setup
    default_kinematics = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)

    motor_axis = Axis(
        can_interface=can_interface,
        can_id=motor_can_id,
        name=f"Motor_{motor_can_id}",
        kinematics=default_kinematics  # Optional, but good practice
    )
    print(f"Axis object '{motor_axis.name}' initialized for CAN ID {motor_axis.can_id}.")
    return motor_axis

# --- Example Usage (within an async function) ---
# async def main():
#     # Assume can_if is a connected CANInterface instance
#     # can_if = CANInterface(...)
#     # await can_if.connect()
#
#     try:
#         motor1 = await initialize_axis_example(can_if, 1)
#         # ... further operations with motor1 ...
#     finally:
#         if can_if.is_connected:
#             await can_if.disconnect()
#
# # asyncio.run(main())
```

## Key Parameters for Axis Initialization:
* `can_interface`: The active and connected CANInterface instance.
* `can_id`: The unique CAN identifier of the motor (integer, typically 1-254).
* `name` (optional): A string name for the axis. Defaults to f"Axis_{can_id}".
* `kinematics` (optional): A Kinematics object for unit conversions. If not provided, a default RotaryKinematics instance with ENCODER_PULSES_PER_REVOLUTION is used. 
* `default_timeout_ms` (optional): Default timeout in milliseconds for motor commands. Defaults to `const.DEFAULT_CAN_TIMEOUT_MS`. 

## Pinging the Motor
Before sending control commands, it's often useful to check if the motor is present and responding on the CAN bus. The `ping()` method serves this purpose.
```python
async def ping_motor_example(axis: Axis):
    try:
        print(f"Pinging {axis.name} (CAN ID {axis.can_id})...")
        response_time_ms = await axis.ping()
        if response_time_ms is not None:
            print(f"{axis.name} responded in {response_time_ms:.2f} ms.")
            return True
        else:
            # This case should ideally be caught by TimeoutError or CommError
            print(f"{axis.name} did not respond as expected (ping returned None).")
            return False
    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to ping within the timeout period.")
        return False
    except exceptions.MotorCommError as e:
        print(f"Communication Error with {axis.name}: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during ping for {axis.name}: {e}")
        return False

# --- Example Usage (within an async function with an initialized 'motor1' Axis object) ---
# if await ping_motor_example(motor1):
#     print(f"{motor1.name} is responsive.")
# else:
#     print(f"{motor1.name} is not responsive. Check connections and power.")
```
The `ping()` method sends a command to read a simple parameter (like the firmware version) and waits for a response. It returns the response time in milliseconds if successful. If the motor doesn't reply within the timeout, a `MotorTimeoutError` is raised.

## Enabling the Motor
MKS servo motors typically need to be explicitly enabled before they will accept movement commands or actively hold their position. The `enable()` method activates the motor's servo loop.
```python
async def enable_motor_example(axis: Axis):
    try:
        if axis.is_enabled():
            print(f"{axis.name} is already enabled.")
            return True

        print(f"Enabling {axis.name}...")
        await axis.enable()
        if axis.is_enabled():
            print(f"{axis.name} enabled successfully.")
            return True
        else:
            # This state might occur if the command was sent but confirmation wasn't processed correctly
            # or if an error occurred that didn't raise an exception but prevented enabling.
            print(f"Attempted to enable {axis.name}, but it's still reported as disabled.")
            # It's good practice to re-query the status or attempt again if necessary.
            # For instance, by calling await axis.update_status()
            return False
            
    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to enable command.")
        return False
    except exceptions.MotorOperationError as e:
        print(f"Operation Error enabling {axis.name}: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while enabling {axis.name}: {e}")
        return False

# --- Example Usage ---
# if await enable_motor_example(motor1):
#     print(f"{motor1.name} is ready for commands.")
```
The `enable()` method sets the motor to servo mode. You can check the enabled state using `axis.is_enabled()`, which returns `True` or `False` based on the last known status (polled or received via commands).

## Disabling the Motor
Disabling the motor releases it, meaning it will no longer actively try to hold its position or respond to movement commands (other than re-enabling). This is often done when the application finishes or if manual movement of the axis is required.
```python
async def disable_motor_example(axis: Axis):
    try:
        if not axis.is_enabled():
            print(f"{axis.name} is already disabled.")
            return True

        print(f"Disabling {axis.name}...")
        await axis.disable()
        if not axis.is_enabled():
            print(f"{axis.name} disabled successfully.")
            return True
        else:
            print(f"Attempted to disable {axis.name}, but it's still reported as enabled.")
            return False

    except exceptions.MotorTimeoutError:
        print(f"Timeout: {axis.name} did not respond to disable command.")
        return False
    except exceptions.MotorOperationError as e:
        print(f"Operation Error disabling {axis.name}: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while disabling {axis.name}: {e}")
        return False

# --- Example Usage ---
# await disable_motor_example(motor1)
```

The `disable()` method sets the motor to idle/release mode. Similar to `enable()`, you can check the state with `axis.is_enabled()`.
## Full Example Flow
Here's a combined example demonstrating the typical sequence:
```python
async def basic_motor_operations_flow():
    # 1. Setup CANInterface (replace with your actual hardware/simulator setup)
    can_if = CANInterface(use_simulator=True) # Assuming simulator is running
    motor_id_to_test = 1

    try:
        await can_if.connect()
        print("CAN Interface connected.")

        # 2. Initialize Axis
        # Using default RotaryKinematics for simplicity here
        motor = Axis(can_if, motor_id_to_test, kinematics=RotaryKinematics())
        print(f"Axis for motor {motor_id_to_test} initialized.")

        # 3. Ping the motor
        if not await ping_motor_example(motor):
            print(f"Motor {motor.can_id} not responding. Aborting.")
            return

        # 4. Enable the motor
        if not await enable_motor_example(motor):
            print(f"Failed to enable motor {motor.can_id}. Aborting.")
            return
        
        print(f"{motor.name} is now enabled and ready.")
        # ... (Movement commands would go here - covered in the next guide) ...
        await asyncio.sleep(1) # Placeholder for operations

        # 5. Disable the motor
        await disable_motor_example(motor)
        print(f"{motor.name} has been disabled.")

    except exceptions.MKSServoCANError as e:
        print(f"A library-specific error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if can_if.is_connected

```
