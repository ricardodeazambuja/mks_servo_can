# Error Handling & Exceptions in `mks-servo-can`

Robust applications require proper error handling. `mks-servo-can` defines a
hierarchy of exceptions so you can catch as narrowly or as broadly as you need.

## Base exception: `MKSServoError`

Every exception the library raises inherits from `MKSServoError`, so one
`except` clause catches all of them. It carries two useful attributes:
`error_code` (whatever the motor reported, when applicable) and `can_id` (which
motor it came from).

```python
from mks_servo_can import Axis, CANInterface, exceptions


async def move_with_error_handling(axis: Axis, can_if: CANInterface):
    try:
        await axis.move_to_position_abs_user(100.0, speed_user=30.0)
        print("Operation successful.")
    except exceptions.MKSServoError as e:
        print(f"Library error on CAN ID {e.can_id}: {e}")
    finally:
        if can_if.is_connected:
            await can_if.disconnect()
```

## The hierarchy

```
MKSServoError
├── CANError             bus-level failure (adapter, wiring, python-can)
├── CRCError             a received frame failed its checksum
├── CommandError         a reply was malformed, or echoed the wrong command
├── CommunicationError   no reply within the timeout, or the link is down
├── ConfigurationError   the library or interface is set up wrongly
├── KinematicsError      a unit conversion could not be performed
├── MultiAxisError       one or more axes failed; see `individual_errors`
├── ParameterError       an argument is out of the range the motor accepts
├── SimulatorError       the simulator is unreachable or misbehaving
└── MotorError           the motor reported a failure
    ├── CalibrationError   encoder calibration failed
    ├── HomingError        the homing sequence failed
    ├── LimitError         a limit switch or soft limit stopped the move
    └── StallError         the motor reported a stall
```

Catch `MotorError` for "the motor said no" and `CommunicationError` for "the
motor said nothing".

## The ones you will actually see

### `CommunicationError` — a timeout

By far the most common. The command went out and nothing came back within
`const.CAN_TIMEOUT_SECONDS`.

```python
from mks_servo_can import Axis, exceptions


async def read_with_timeout_handling(axis: Axis):
    try:
        position = await axis.get_current_position_user()
        print(f"{axis.name} is at {position:.2f}")
    except exceptions.CommunicationError as e:
        print(f"Timeout: {axis.name} did not respond ({e}).")
```

Usual causes: the motor is powered off or off the bus, the `Axis` was built with
a CAN ID no motor answers on, the bus is missing termination, or the simulator
is not running.

### `MotorError` — the motor refused

The motor answered and reported failure: a move commanded while disabled, or a
command its current state does not allow.

```python
from mks_servo_can import Axis, exceptions


async def move_with_motor_error_handling(axis: Axis):
    try:
        await axis.move_relative_user(10.0, speed_user=10.0)
    except exceptions.LimitError as e:
        print(f"{axis.name} stopped on a limit: {e}")
    except exceptions.MotorError as e:
        print(f"{axis.name} refused the move (status {e.error_code}): {e}")
        if not axis.is_enabled():
            print("The motor is not enabled.")
```

Catch `LimitError` and `StallError` before `MotorError` if you want to treat
them separately — they are subclasses, so a bare `except MotorError` swallows
them.

### `ParameterError` — caught before anything is sent

Raised locally when an argument is outside the range the protocol allows, such
as an MKS speed parameter above 3000 or an acceleration above 255. No frame is
sent.

### `ConfigurationError`, `CANError` and `SimulatorError` — at connect time

```python
from mks_servo_can import CANInterface, exceptions


async def connect_with_handling():
    can_if = CANInterface(interface_type="socketcan", channel="can0")
    try:
        await can_if.connect()
    except exceptions.ConfigurationError as e:
        print(f"Configuration problem: {e}")
    except exceptions.CANError as e:
        print(f"Bus did not come up: {e}. Check the adapter, wiring and power.")
    except exceptions.SimulatorError as e:
        print(f"Simulator unreachable: {e}. Is mks-servo-simulator running?")
```

### `MultiAxisError` — one axis of several

`MultiAxisController` gathers per-axis failures rather than stopping at the
first. The individual exceptions are on `individual_errors`, keyed by axis name.

```python
from mks_servo_can import MultiAxisController, exceptions


async def move_all_with_handling(controller: MultiAxisController):
    try:
        await controller.move_all_to_positions_abs_user({"x": 10.0, "y": 20.0})
    except exceptions.MultiAxisError as e:
        for axis_name, error in (e.individual_errors or {}).items():
            print(f"  {axis_name}: {error}")
```

## Strategies

### Catch specific first, general last

Subclasses must come before their parents, or the parent clause wins.

### Check state rather than guessing

`axis.is_enabled()` and `axis.is_move_complete()` are free and synchronous.
`await axis.read_en_status()`, `await axis.ping()` and
`await axis.get_status_dict()` cost a round trip and tell you what the motor
currently thinks. After an error, `get_status_dict()` is the quickest way to see
where things stand — it returns a partial dictionary with an
`error_during_status_fetch` key rather than raising again.

### Retry transient failures only

A `CommunicationError` or `CRCError` is worth one or two retries with a short
delay. Retrying a *move* is different: the first attempt may have partially
succeeded, so re-issue an absolute target rather than repeating a relative one.

### Log rather than print

The library logs through the standard `logging` module under the `mks_servo_can`
hierarchy. Per-frame records are at `DEBUG` and are lazily formatted, so leaving
`INFO` on in production costs nothing.

## A fuller example

```python
import asyncio

from mks_servo_can import Axis, exceptions


async def robust_motor_operation(axis: Axis):
    try:
        status = await axis.get_status_dict()
        if "error_during_status_fetch" in status:
            print(f"{axis.name} is not answering cleanly: {status['error_during_status_fetch']}")
            return

        if not status["is_enabled"]:
            print(f"Enabling {axis.name} first...")
            await axis.enable_motor()

        await axis.move_relative_user(90.0, speed_user=30.0)
        print(f"{axis.name} reached {await axis.get_current_position_user():.2f}")

    except exceptions.CommunicationError as e:
        print(f"Timeout with {axis.name}: {e}. The motor may be unresponsive.")
    except exceptions.LimitError as e:
        print(f"{axis.name} hit a limit: {e}")
    except exceptions.MotorError as e:
        print(f"{axis.name} reported a failure (code {e.error_code}): {e}")
    except exceptions.MKSServoError as e:
        print(f"Library error: {e}")
    finally:
        print(f"Finished the operation attempt for {axis.name}.")
```

Disconnecting the `CANInterface` belongs at application shutdown, not in a
per-operation `finally` — it is usually shared between axes.
