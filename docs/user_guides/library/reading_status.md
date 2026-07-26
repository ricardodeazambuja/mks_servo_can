# Reading Motor Status & Parameters

Understanding the current state of your MKS servo motor is crucial for robust
control and debugging. The `Axis` class provides methods to read position,
speed, enable state and a combined status snapshot.

## Prerequisites

* An `Axis` instance that is initialized and connected. See
  [Basic Motor Control](./basic_control.md).
* The motor should be powered on and responsive (e.g. successfully pinged).

## Core concepts: what is polled and what is cached

Reads that talk to the motor are `async` and cost a CAN round trip:
`get_current_position_steps()`, `get_current_position_user()`,
`get_current_speed_rpm()`, `get_current_speed_user()`, `read_en_status()`,
`get_motor_status_code()` and `get_status_dict()`.

Everything else is a cheap accessor over what the axis already knows:
`is_enabled()`, `is_homed()`, `is_calibrated()` and `is_move_complete()` return
immediately and do not communicate. They can be stale; call the matching `async`
read when you need certainty.

One consequence is worth knowing about: an absolute move returns as soon as the
motor reports completion, a moment before the axis refreshes its cached
position. The library handles this internally — a move is never skipped because
of a stale cache — but a `get_current_position_user()` immediately after a move
is what gives you the settled value.

## 1. Position

```python
import asyncio

from mks_servo_can import Axis, exceptions


async def get_position_example(axis: Axis):
    try:
        # Position in user units, via the axis's kinematics (degrees, mm, ...)
        current_pos_user = await axis.get_current_position_user()
        print(f"{axis.name} position ({axis.kinematics.units}): {current_pos_user:.2f}")

        # Position in raw encoder steps (16384 per motor shaft revolution)
        current_pos_steps = await axis.get_current_position_steps()
        print(f"{axis.name} position (encoder steps): {current_pos_steps}")
    except exceptions.CommunicationError:
        print(f"Timeout: {axis.name} did not respond to the position query.")
    except exceptions.MotorError as e:
        print(f"Motor error reading position for {axis.name}: {e}")
```

`get_current_position_user()` reads the encoder and converts with the axis's
kinematics; `get_current_position_steps()` returns the raw count. Both refresh
the axis's cached position.

## 2. Speed

```python
from mks_servo_can import Axis, exceptions


async def get_speed_example(axis: Axis):
    try:
        # Speed in user units per second (deg/s, mm/s, ...)
        current_speed_user = await axis.get_current_speed_user()
        print(f"{axis.name} speed ({axis.kinematics.units}/s): {current_speed_user:.2f}")

        # Speed as the motor reports it, in RPM
        current_speed_rpm = await axis.get_current_speed_rpm()
        print(f"{axis.name} speed (RPM): {current_speed_rpm}")
    except exceptions.CommunicationError:
        print(f"Timeout: {axis.name} did not respond to the speed query.")
    except exceptions.MotorError as e:
        print(f"Motor error reading speed for {axis.name}: {e}")
```

## 3. Enable state

`is_enabled()` returns the cached flag and does not communicate.
`read_en_status()` asks the motor and updates that flag.

```python
from mks_servo_can import Axis, exceptions


async def check_enabled_state_example(axis: Axis):
    # Cached - free, possibly stale.
    print(f"{axis.name} (cached) enabled: {axis.is_enabled()}")

    # Authoritative - one CAN round trip.
    try:
        actually_enabled = await axis.read_en_status()
        print(f"{axis.name} (from motor) enabled: {actually_enabled}")
    except exceptions.MKSServoError as e:
        print(f"Error reading the enable status: {e}")
```

## 4. Movement state

`is_move_complete()` reports whether the axis has an outstanding move. It is
synchronous and reflects the library's own view of the move it dispatched.
To block until the move finishes, await `wait_for_move_completion()`.

```python
import asyncio

from mks_servo_can import Axis


async def check_movement_status_example(axis: Axis):
    await axis.move_relative_user(180.0, speed_user=90.0, wait=False)
    await asyncio.sleep(0.5)

    if axis.is_move_complete():
        print(f"{axis.name} has finished its move.")
    else:
        print(f"{axis.name} is still moving.")

    await axis.wait_for_move_completion(timeout=10.0)
    print(f"{axis.name} settled at {await axis.get_current_position_user():.2f}")
```

## 5. A full status snapshot (`get_status_dict`)

`get_status_dict()` polls position, enable state and the motor's status code,
and combines them with what the axis already knows.

```python
from mks_servo_can import Axis, exceptions


async def show_status_example(axis: Axis):
    try:
        status = await axis.get_status_dict()
        print(f"Status for {status['name']} (CAN ID {status['can_id']}):")
        print(f"  Position: {status['position_user']:.2f} {status['position_units']}")
        print(f"  Position (steps): {status['position_steps']}")
        print(f"  Enabled: {status['is_enabled']}")
        print(f"  Homed: {status['is_homed']}")
        print(f"  Calibrated: {status['is_calibrated']}")
        print(f"  Motor status: {status['motor_status_str']} ({status['motor_status_code']})")
        print(f"  Move in progress: {status['active_move_in_progress']}")
        print(f"  Last error: {status['error_state']}")
    except exceptions.CommunicationError:
        print(f"Timeout: {axis.name} did not respond to the status query.")
    except exceptions.MotorError as e:
        print(f"Motor error reading status for {axis.name}: {e}")
```

If the underlying reads fail, the dictionary comes back partial with an
`error_during_status_fetch` key rather than raising, so check for it if you poll
this in a loop.

## 6. The motor's status code

`get_motor_status_code()` returns the raw code; `constants.MOTOR_STATUS_MAP`
turns it into a description. `get_status_dict()` does both for you as
`motor_status_code` and `motor_status_str`.

```python
from mks_servo_can import Axis, const


async def read_motor_status_example(axis: Axis):
    code = await axis.get_motor_status_code()
    print(f"{axis.name}: {const.MOTOR_STATUS_MAP.get(code, 'unknown')} (code {code})")
```

## 7. Reading arbitrary system parameters

`Axis` deliberately exposes only the status a controller needs. Anything else —
firmware version, board parameters, the shaft protection state — is read through
the low-level API, which mirrors the manual's command set one method per
command.

```python
from mks_servo_can import CANInterface, LowLevelAPI


async def read_low_level_example(can_interface: CANInterface, can_id: int):
    api = LowLevelAPI(can_interface)
    carry, value = await api.read_encoder_value_carry(can_id)
    print(f"Encoder carry={carry} value={value}")

    io_status = await api.read_io_status(can_id)
    print(f"IO port status: {io_status}")

    shaft_error = await api.read_shaft_angle_error(can_id)
    print(f"Shaft angle error: {shaft_error}")
```

See `mks_servo_can/low_level_api.py` for the full list, and the MKS Servo Motor
User Manual for what each command returns.

## 8. Errors

Failed reads raise, they do not return a code: `CommunicationError` for a
timeout or a malformed reply, `MotorError` for something the motor reported,
`CRCError` for a corrupted frame. All derive from `MKSServoError`, so a single
`except exceptions.MKSServoError` catches the lot. The most recent one is kept
on the axis and appears in `get_status_dict()` as `error_state`.

See [Error Handling](./error_handling.md) for the full hierarchy.
