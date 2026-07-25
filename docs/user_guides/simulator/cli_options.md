# Simulator Command-Line Options

`mks-servo-simulator` emulates one or more MKS SERVO42D/57D motors on a virtual
CAN bus carried over TCP, so the library can be driven with no hardware
attached. The authoritative list of options is always:

```bash
mks-servo-simulator --help
```

This page explains what they are for. `mks_servo_simulator` (underscored) is a
synonym for the same entry point, and `python -m mks_simulator.cli` works from a
checkout.

## Bus and motors

| option | default | what it does |
|---|---|---|
| `--host TEXT` | `localhost` | Address the simulator listens on. |
| `--port INTEGER` | `6789` | TCP port the library connects to. |
| `--num-motors INTEGER` | `1` | How many motors to create. |
| `--start-can-id INTEGER` | `1` | CAN ID of the first motor; the rest count up from there. |
| `--motor-type [mks servo42d\|mks servo57d\|generic]` | `generic` | Which motor model to emulate. |
| `--steps-per-rev INTEGER` | `16384` | Encoder steps per revolution. |
| `--latency-ms FLOAT` | `2.0` | Simulated round-trip bus latency in milliseconds. |

`--num-motors 3 --start-can-id 1` gives you motors on CAN IDs 1, 2 and 3. Any
other ID answers nothing — which is a useful way to exercise your timeout
handling without stubbing anything.

To connect the library to a running simulator, use the simulator interface type
rather than a hardware channel:

```python
from mks_servo_can import CANInterface

can_if = CANInterface(use_simulator=True, simulator_host="localhost", simulator_port=6789)
```

## Watching what it is doing

| option | default | what it does |
|---|---|---|
| `--debug-api` | off | Starts the HTTP debug server. |
| `--debug-api-port INTEGER` | `8765` | Port for it. |
| `--step` | off | Simulated time advances only when `POST /step` says so. Implies `--debug-api`. See [Deterministic time](advanced_simulation.md#deterministic-time---step). |
| `--json-output` | off | Emits machine-readable JSON state on stdout, periodically. |
| `--textual-dashboard` | off | Legacy terminal TUI. |
| `--refresh-rate INTEGER` | `200` | Dashboard refresh interval in milliseconds. |
| `--log-level [debug\|info\|warning\|error]` | `info` | Console verbosity. |
| `--no-color` | off | Plain output, for terminals or pipes that mangle escapes. |

There are two first-class ways to see what the simulator is doing, and they
share one payload so they cannot disagree:

* **For a human:** `--debug-api`, then open `http://127.0.0.1:8765/dashboard`.
* **For a program or an agent:** the same `--debug-api`, then poll
  `http://127.0.0.1:8765/status`, which returns the JSON the dashboard renders.
  `http://127.0.0.1:8765/docs` is the generated OpenAPI page.

`--json-output` is the alternative for a caller that wants state pushed at it on
stdout rather than fetched. `--textual-dashboard` is legacy and kept working for
existing users; prefer `--debug-api`.

See the [HTTP Debug API Guide](../http_debug_api.md) for the endpoints and
[Interpreting Simulator Logs](logs.md) for the console output.

## Configuration profiles

| option | default | what it does |
|---|---|---|
| `--config-profile TEXT` | — | Start from a named saved profile. |
| `--save-config TEXT` | — | Save the options you just passed as a named profile. |
| `--config-dir TEXT` | `~/.mks_simulator_config` | Where profiles are stored. |

```bash
# Save a setup once...
mks-servo-simulator --num-motors 3 --latency-ms 5 --debug-api --save-config bench

# ...and reuse it.
mks-servo-simulator --config-profile bench
```

A profile carries the host, port, motor list, latency, refresh rate and the
output modes.

**A profile wins over the flags on the same command line.** `--config-profile`
is applied *after* the options are parsed and overwrites `--host`, `--port`,
`--latency-ms`, `--refresh-rate`, `--no-color`, `--json-output`, `--debug-api`
and `--textual-dashboard`. The motors are built entirely from the profile's own
list, so `--num-motors`, `--start-can-id`, `--motor-type` and `--steps-per-rev`
are ignored outright. So

```bash
mks-servo-simulator --config-profile bench --num-motors 6
```

does **not** give you six motors — it gives you however many `bench` was saved
with, silently. If you need a variation, save a second profile rather than
trying to adjust one from the command line.

Profiles can also be listed and loaded over the debug API at `/config/profiles`,
which is how you switch setups without restarting.

## Common invocations

```bash
# Simplest thing that works: one motor on CAN ID 1, port 6789.
mks-servo-simulator

# A three-axis machine with the browser dashboard.
mks-servo-simulator --num-motors 3 --debug-api

# Something closer to a real bus, for latency-sensitive work.
mks-servo-simulator --num-motors 4 --latency-ms 5

# Two simulators at once, for a test that needs isolation.
mks-servo-simulator --port 6789 --debug-api --debug-api-port 8765
mks-servo-simulator --port 6790 --debug-api --debug-api-port 8766

# Chasing a protocol problem.
mks-servo-simulator --log-level debug
```

If you run more than one, give each its own `--port` *and* `--debug-api-port`.
A second simulator that fails to bind will not stop the first from answering,
and a client talking to the wrong instance is a genuinely confusing thing to
debug — the project's own test suite reserves ports 6789, 6790 and 6791 for
exactly this reason.
