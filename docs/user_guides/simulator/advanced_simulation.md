# Simulating Latency and Multiple Motors

The defaults are chosen to be convenient, not realistic: one motor and 2 ms of
bus latency. This page covers making the simulation resemble the bench closely
enough that the difference stops hiding bugs.

## Multiple motors

```bash
mks-servo-simulator --num-motors 3 --start-can-id 1
```

gives motors on CAN IDs 1, 2 and 3, all of the same type, sharing one virtual
bus. That is the arrangement a `MultiAxisController` expects:

```python
import asyncio

from mks_servo_can import Axis, CANInterface, MultiAxisController, RotaryKinematics


async def main():
    can_if = CANInterface(use_simulator=True, simulator_port=6789)
    await can_if.connect()

    controller = MultiAxisController(can_interface_manager=can_if)
    for name, can_id in (("x", 1), ("y", 2), ("z", 3)):
        controller.add_axis(Axis(can_if, motor_can_id=can_id, name=name,
                                 kinematics=RotaryKinematics(steps_per_revolution=16384)))

    await controller.initialize_all_axes()
    await controller.enable_all_axes()
    print(await controller.get_all_positions_user())

    await can_if.disconnect()


asyncio.run(main())
```

Two properties of the virtual bus are worth knowing, because they are what make
multi-axis tests meaningful:

* **The bus is shared and serialised, like a real one.** Commands to three
  motors do not travel in parallel for free; the latency below applies per
  exchange. If your coordinated move looks slower than the arithmetic suggests,
  that is the bus, and it is the same effect you will get on hardware.
* **An unknown CAN ID is answered by silence, not by an error.** Address motor
  99 on a three-motor simulator and the frame goes out and nothing comes back.
  This is deliberate and it is the cheapest way to test your timeout and retry
  handling — no stubbing, no mocks, no patched transport. The project's own
  compliance suite provokes every failure path this way.

Motors are otherwise independent: each has its own position, enable state,
work mode and limits.

## Latency

```bash
mks-servo-simulator --latency-ms 5
```

`--latency-ms` is the **round-trip** figure. The bus applies half of it before
the motor sees the command and half before the response comes back, so 5 ms
means roughly 2.5 ms each way. `0` disables the delay entirely.

Which value to use:

| value | what it models |
|---|---|
| `0` | no bus at all — fastest tests, least realistic |
| `1` | the project's own unit and compliance fixtures; fast, still ordered |
| `2` (default) | a comfortable working default |
| `5` | the performance fixture — a loaded bus, and where ordering bugs surface |
| `20`–`100` | a deliberately awful bus, for testing timeouts and retries |

Turning latency **up** is the most useful debugging move the simulator offers.
Races between a command and the state read that follows it are invisible at 0 ms
and obvious at 50 ms. If a test is flaky in CI and solid on your machine, run it
at high latency locally before you assume CI is at fault.

Latency can also be changed on a running simulator, without restarting:

```bash
curl -X POST http://localhost:8765/config/parameters/latency_ms \
  -H "Content-Type: application/json" \
  -d '{"value": 25.0}'
```

Accepted range is 0.1 to 100.0 ms. A failed update returns `{"success": false}`
with HTTP 200 rather than an error status, so check the body, not just the code.

## Other live parameters

`GET /config/parameters` lists what can be changed on a running simulator:

| parameter | range |
|---|---|
| `latency_ms` | 0.1 – 100.0 |
| `refresh_rate` | 50 – 2000 ms |
| `motors.{index}.max_current` | 100 – 3000 mA |
| `motors.{index}.max_speed` | 100 – 10000 steps/s |

Note that the motor parameters are indexed by **position in the configuration**,
not by CAN ID — `motors.0.max_current` is the first motor.

## Reproducible setups

Once you have a configuration worth keeping, save it:

```bash
mks-servo-simulator --num-motors 4 --latency-ms 5 --debug-api --save-config bench
mks-servo-simulator --config-profile bench
```

A profile also carries per-motor detail the command line cannot express —
initial position, position limits in degrees, per-motor current, and whether the
motor starts enabled. Remember that a profile **overrides** the flags beside it;
see [Command-Line Options](cli_options.md#configuration-profiles).

Profiles can be listed, loaded and saved over the debug API, so a test can
switch the whole machine configuration between cases without restarting the
process:

```bash
curl http://localhost:8765/config/profiles
curl -X POST http://localhost:8765/config/profiles/bench/load
```

## Injecting commands directly

For protocol-level work you can bypass the library entirely and put a raw frame
on the bus:

```bash
curl -X POST http://localhost:8765/inject \
  -H "Content-Type: application/json" \
  -d '{"motor_id": 1, "command_code": 246, "data_bytes": [1, 0, 100, 0]}'
```

This is how you test what the motor does with a frame the library would never
send — a malformed length, a reserved command, a value out of range. `/templates`
lists the pre-built ones and `/inject_template` sends them by name.

## What the simulator cannot tell you

It is worth being precise about this, because it bounds what a green test means.

The simulator was written from the same reading of the MKS manual as the
library. Where that reading is wrong, both are wrong in the same direction and
every test here agrees with itself. Specifically, the sign convention, the exact
abort-frame behaviour when a move is re-targeted mid-flight, and whether
`CanRSP` suppresses replies to reads as well as to run commands are all
inferences, not observations — the `errata` block in
`mks_servo_can/data/manual_commands_v106.json` records where the manual is
ambiguous or self-contradictory.

The simulator also does not model the physics that eventually matter: no motor
heating, no lost steps, no stall, no supply sag, no EMI on the bus.

Use it to prove your logic, your error handling and your protocol encoding. Use
`tests/hil/` and a real motor to prove the reading of the manual underneath
them.

## See also

* [Command-Line Options](cli_options.md)
* [Interpreting Simulator Logs](logs.md)
* [HTTP Debug API Guide](../http_debug_api.md)
* [Working with Multiple Axes](../library/multi_axis.md)
