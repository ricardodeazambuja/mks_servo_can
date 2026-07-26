# Using Kinematics for Unit Conversion

`mks-servo-can` includes a kinematics system so you can work in physical units —
degrees, millimetres — instead of raw encoder steps. The `Axis` class uses a
kinematics object to convert automatically in both directions.

## Prerequisites

* An understanding of how to initialize and use the `Axis` class. See
  [Basic Motor Control](./basic_control.md).
* Familiarity with the movement commands. See [Movement Commands](./movements.md).

## Core concept

Every `Axis` has a kinematics object. Methods whose names end in `_user` —
`move_to_position_abs_user()`, `move_relative_user()`, `set_speed_user()`,
`get_current_position_user()`, `get_current_speed_user()` — go through it. The
`_axis` and `_pulses` variants bypass it and work in the motor's own units.

* **User units to steps** when commanding: 90 degrees becomes an encoder count.
* **Steps to user units** when reading: an encoder count becomes 90 degrees.

The kinematics object is passed at construction and is not changed afterwards.
If you omit it, the axis defaults to
`RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)` — a
direct-drive rotary output at the motor's native resolution.

Note what is *not* converted: `default_speed_param` and `default_accel_param` are
MKS parameters in the motor's own units. A speed you pass explicitly is in user
units and is converted; an omitted one falls back to the parameter directly.

## Importing

```python
import asyncio

from mks_servo_can import Axis, CANInterface, const, exceptions
from mks_servo_can.kinematics import (
    EccentricKinematics,
    Kinematics,
    LinearKinematics,
    RotaryKinematics,
)
```

## 1. `RotaryKinematics`

For axes whose output is angular: a bare motor shaft, a rotary stage, a geared
joint.

`RotaryKinematics(steps_per_revolution, gear_ratio=1.0, degrees_per_output_revolution=360.0)`

* `steps_per_revolution` — encoder pulses per revolution of the *motor* shaft.
  For the SERVO42D/57D this is `const.ENCODER_PULSES_PER_REVOLUTION` (16384).
* `gear_ratio` — motor revolutions per output revolution. A 10:1 gearbox is
  `10.0`. Defaults to direct drive.
* `degrees_per_output_revolution` — how many user units one output revolution
  spans. Leave at 360.0 to work in degrees; set it to 400.0 to work in gradians.

The units string is always `"deg"`.

```python
def build_geared_rotary_axis(can_if: CANInterface) -> Axis:
    # A motor behind a 10:1 gearbox, commanded in degrees of the output shaft.
    rotary_kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        gear_ratio=10.0,
    )
    return Axis(
        can_interface_manager=can_if,
        motor_can_id=1,
        name="joint",
        kinematics=rotary_kin,
    )


async def rotary_kinematics_example(axis: Axis):
    try:
        await axis.enable_motor()
        await axis.move_to_position_abs_user(36.0, speed_user=10.0)
        position = await axis.get_current_position_user()
        print(f"Output shaft at {position:.2f} {axis.kinematics.units}")
    except exceptions.MKSServoError as e:
        print(f"Move failed: {e}")

    # 36 degrees of output is one full motor revolution through a 10:1 gearbox:
    # 36 x 10 x 16384 / 360 = 16384 encoder steps.
    print(axis.kinematics.user_to_steps(36.0))
```

## 2. `LinearKinematics`

For axes that turn rotation into travel: a leadscrew, a belt, a rack and pinion.

`LinearKinematics(steps_per_revolution, pitch, gear_ratio=1.0, units="mm")`

* `pitch` — travel per revolution of the *output*, in `units`. A 10 mm leadscrew
  is `10.0`; for a belt it is the pulley circumference.
* `units` — the label carried through to `axis.kinematics.units`.

```python
def build_linear_axis(can_if: CANInterface) -> Axis:
    linear_kin = LinearKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        pitch=10.0,   # 10 mm of travel per output revolution
        units="mm",
    )
    return Axis(
        can_interface_manager=can_if,
        motor_can_id=2,
        name="x",
        kinematics=linear_kin,
    )


async def linear_kinematics_example(axis: Axis):
    await axis.enable_motor()
    await axis.move_to_position_abs_user(25.0, speed_user=5.0)  # 25 mm at 5 mm/s
    print(f"Carriage at {await axis.get_current_position_user():.3f} mm")
```

## 3. `EccentricKinematics`

For a crank or cam, where the relationship between angle and displacement is not
linear.

`EccentricKinematics(steps_per_revolution, arm_length, gear_ratio=1.0, max_displacement=0.0, units="mm")`

Speed conversion for this model is an approximation — displacement per unit of
angle varies through the rotation — so treat commanded speeds as nominal.

## 4. Converting without an axis

Every kinematics object is usable on its own, which is handy for planning:

```python
def conversion_example():
    kin = LinearKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0
    )
    print(kin.user_to_steps(25.0))            # mm -> encoder steps
    print(kin.steps_to_user(40960))           # encoder steps -> mm
    print(kin.user_speed_to_motor_speed(5.0)) # mm/s -> MKS speed parameter
    print(kin.motor_speed_to_user_speed(300)) # MKS speed parameter -> mm/s
    print(kin.get_parameters())               # a dict describing the model
```

## 5. Writing your own

Subclass `Kinematics` and implement the four conversions. See
[Custom Kinematics](../../tutorials/custom_kinematics.md) for a worked example.
