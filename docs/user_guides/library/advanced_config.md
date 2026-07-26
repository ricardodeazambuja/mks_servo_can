# Advanced Configuration

## Prerequisites

* Familiarity with `CANInterface` and `Axis` initialization and basic usage.
* Understanding of reading motor status, as covered in
  [Reading Motor Status & Parameters](./reading_status.md).
* Awareness of the risk in modifying persistent motor parameters. Several of the
  commands below survive a power cycle, and one of them changes the CAN ID you
  are talking to.

## 1. Configuring an `Axis`

Everything about an axis is fixed at construction:

```python
from mks_servo_can import Axis, CANInterface, RotaryKinematics, const


def build_axis(can_if: CANInterface) -> Axis:
    return Axis(
        can_interface_manager=can_if,
        motor_can_id=1,
        name="pan",
        kinematics=RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
            degrees_per_output_revolution=360.0,
        ),
        mstep_value=16,               # the motor's microstep setting
        base_motor_steps_per_rev=200, # 1.8 deg/step motor
        default_speed_param=500,      # MKS speed parameter, not user units
        default_accel_param=100,      # MKS acceleration parameter (0-255)
    )
```

`default_speed_param` and `default_accel_param` are **MKS parameters**, in the
motor's own units — not degrees per second. Parameter 500 means roughly 500 RPM.
When you pass `speed_user=` to a move, that value *is* in user units and is
converted for you; when you omit it, the default parameter is used as-is.
`motor_profile.speed_param_to_rpm()` converts between the two if you need to
reason about them together.

## 2. Timeouts

There is one interface-level timeout, `const.CAN_TIMEOUT_SECONDS` (1.0 s),
applied to every request/response exchange. Individual commands do not take a
timeout argument; the two places you can override waiting are:

```python
from mks_servo_can import Axis


async def timeout_examples(axis: Axis):
    # How long to wait for a move that is already under way.
    await axis.move_to_position_abs_user(90.0, wait=False)
    await axis.wait_for_move_completion(timeout=15.0)

    # How long to wait for a liveness check.
    alive = await axis.ping(timeout=0.5)
    print(f"{axis.name} reachable: {alive}")
```

Move completion timeouts are computed by the axis from the distance, the speed
parameter and the motor's work mode, with a floor of five CAN timeouts — a long
slow move is not cut short. If you need a different exchange timeout globally,
use the low-level API, whose send helpers accept one directly.

## 3. Changing motor parameters

The `Axis` class exposes the settings a controller changes at runtime:

```python
from mks_servo_can import Axis, const


async def axis_level_settings(axis: Axis):
    await axis.set_work_mode(const.MODE_SR_VFOC)  # serial FOC, the CAN default
    await axis.set_motor_subdivision(16)          # microsteps
    await axis.set_current_position_as_zero()
```

Everything else lives on `LowLevelAPI`, one method per command in the manual:

```python
from mks_servo_can import CANInterface, LowLevelAPI, const


async def motor_level_settings(can_if: CANInterface):
    api = LowLevelAPI(can_if)
    can_id = 1

    await api.set_working_current(can_id, 1600)  # mA
    # Holding current is a code, not a percentage: 0x00 is 10% and 0x08 is 90%.
    await api.set_holding_current_percentage(can_id, 0x04)  # 50%
    await api.set_stall_protection(can_id, enable=True)
    await api.set_motor_direction(can_id, const.DIR_CW)
    await api.set_subdivision_interpolation(can_id, enable=True)
```

Some of these persist across power cycles. Two deserve particular care:

* `set_can_id()` changes the address the motor answers on. The `Axis` talking to
  it stops working at the moment it succeeds.
* `set_can_bitrate()` changes the bus speed for that motor alone, which takes it
  off the bus until the adapter is reconfigured to match.

`restore_default_parameters()` puts everything back, and `restart_motor()`
applies changes that need a power cycle.

## 4. CRC checking

CRC is not optional in this library. Every frame it sends carries a valid CRC and
every frame it receives is checked; a mismatch raises `CRCError` rather than
being acted on. Real motors drop frames with a bad CRC, so a configuration that
skipped the check would let a CRC bug pass unnoticed — the simulator deliberately
mirrors the hardware here and drops them too.

## 5. Dropping to the low-level API

`Axis` covers common motion control. For anything it does not expose, construct a
`LowLevelAPI` over the same interface — they share the transport, so you can mix
the two freely:

```python
from mks_servo_can import Axis, CANInterface, LowLevelAPI


async def mixed_usage(can_if: CANInterface, axis: Axis):
    api = LowLevelAPI(can_if)

    protection_state = await api.read_motor_protection_state(axis.can_id)
    print(f"Shaft protection triggered: {protection_state}")

    # Fire-and-forget variants for fixed-rate control loops: no reply is
    # awaited, so they cost roughly a seventh of a normal command.
    await api.run_position_mode_absolute_axis_no_wait(
        axis.can_id, speed=600, acceleration=250, absolute_axis=4096
    )
```

The `_no_wait` variants are what `mks_servo_can.realtime.ServoStream` is built
on; see [Streaming control](../../advanced_topics/asynchronous_control_with_asyncio.md)
before using them directly, because with motor responses disabled a failure is
silent by construction.

Using `LowLevelAPI` requires knowing the protocol as described in the MKS user
manual — it bypasses the unit conversion and state tracking `Axis` provides.

## Conclusion

The knobs that exist are: how an `Axis` is constructed, how long you wait for a
move, and the motor's own persistent settings through the low-level API.
Anything else in this library is deliberately not configurable.
