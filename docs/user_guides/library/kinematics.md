# Using Kinematics for Unit Conversion

The `mks-servo-can` library includes a kinematics system to simplify working with physical units (e.g., degrees, millimeters, inches) rather than raw motor encoder steps. This is crucial for writing intuitive and maintainable control code. The `Axis` class uses a kinematics object to perform these conversions automatically for position, speed, and acceleration values in its movement commands.

## Prerequisites

* An understanding of how to initialize and use the `Axis` class. See [Basic Motor Control](./basic_control.md).
* Familiarity with the movement commands. See [Movement Commands](./movement.md).

## Core Concept

Each `Axis` object can have an associated kinematics object (an instance of a class derived from `BaseKinematics`). When you provide a position, speed, or acceleration to methods like `move_absolute()`, `move_relative()`, or `set_speed_mode()`, these values are processed by the kinematics object if `use_kinematics=True` (which is the default).

* **Physical Units to Steps:** When sending commands, the kinematics object converts your human-readable units (e.g., 90 degrees) into the motor's required encoder steps.
* **Steps to Physical Units:** When reading data from the motor (e.g., current position), the kinematics object converts the raw step values back into your defined physical units.

If no kinematics object is explicitly assigned to an `Axis`, it defaults to `RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)`, assuming a direct rotary output with the motor's native high resolution. [cite: mks_servo_can_library/mks_servo_can/axis.py]

## Importing Kinematics Classes

The available kinematics classes are in the `mks_servo_can.kinematics` module:

```python
import asyncio
from mks_servo_can import CANInterface, Axis, const, exceptions
from mks_servo_can.kinematics import RotaryKinematics, LinearKinematics, BaseKinematics, EccentricKinematics
```
## 1. Rotary Kinematics (RotaryKinematics)
Used for axes where the motor's rotation directly translates to an angular output (e.g., a motor shaft, a simple rotary stage).
### Initialization:
`RotaryKinematics(steps_per_revolution: int, gear_ratio: float = 1.0, units: str = "degrees")`
* `steps_per_revolution`: The number of motor encoder pulses that correspond to one full revolution of the motor shaft itself. For MKS SERVO42D/57D, this is typically `const.ENCODER_PULSES_PER_REVOLUTION` (16384). 
* `gear_ratio` (optional): The gear ratio between the motor and the final output. For example, if a 5:1 gearbox is used, meaning 5 motor revolutions result in 1 output revolution, the gear_ratio would be 5.0. Defaults to 1.0 (direct drive). 
* `units` (optional): A string describing the physical units, primarily for display/logging purposes (e.g., "degrees", "radians"). Defaults to "degrees". 
### Example:
```python
async def rotary_kinematics_example(axis: Axis):
    # Motor with 16384 steps/rev, connected to a 10:1 gearbox
    # We want to control the output shaft in degrees.
    rotary_kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, # Motor's native steps
        gear_ratio=10.0,
        units="degrees"
    )
    axis.set_kinematics(rotary_kin)
    print(f"{axis.name} kinematics set to Rotary: {axis.kinematics.get_info()}")

    # Now, movement commands use degrees for the output shaft
    try:
        await axis.enable()
        # Move the output shaft to 36 degrees
        await axis.move_absolute(target_position=36.0, speed=10.0) # 36 degrees, 10 deg/s
        current_pos_deg = await axis.get_current_position()
        print(f"Moved to {current_pos_deg:.2f} {axis.kinematics.units}") # Should be approx 36.0 degrees

        # Internally, 36 degrees on the output shaft is 1 full motor revolution (360 degrees)
        # due to the 10:1 gearbox, which is 16384 steps.
        # 36 output_deg * (10 motor_rev / 1 output_rev) * (16384 steps / 1 motor_rev) / (360 deg / 1 motor_rev)
        # = 36 * 10 * 16384 / 360 = 16384 steps.
```