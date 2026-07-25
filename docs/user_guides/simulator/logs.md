# Interpreting Simulator Logs

## The format

Console output is standard `logging` with the format

```text
%(asctime)s - %(name)s - %(levelname)s - %(message)s
```

and timestamps as `YYYY-MM-DD HH:MM:SS`, so a line looks like:

```text
2026-07-24 11:02:13 - VirtualCANBus - INFO - Client connected from 127.0.0.1:51422
```

The `name` field is the useful one, because it tells you which layer is talking:

| logger | what it reports |
|---|---|
| `MKSSimulatorCLI` | startup, configuration, shutdown |
| `VirtualCANBus` | the socket server, client connections, frames in and out |
| `SimulatedMotor` | per-motor behaviour — commands accepted, moves started and finished, limits hit |
| `mks_simulator.interface.*` | the debug API, config manager and LLM debug interface |

## Verbosity

`--log-level` takes `debug`, `info` (the default), `warning` or `error`. It sets
the root logger and, explicitly, the `VirtualCANBus` and `SimulatedMotor`
loggers — so `--log-level debug` really does give you the frame-level detail,
not just more startup chatter.

```bash
mks-servo-simulator --log-level debug
```

Use `debug` when you are chasing a protocol problem — which command byte
arrived, what the motor made of it, what went back. Use `warning` when you are
running something long and only care about what went wrong; the fixture that
drives the performance tests in this repository does exactly that, to keep the
noise out of timing measurements.

## Where the output goes

**Normally:** the console.

**With `--textual-dashboard`:** the TUI owns the terminal, so logging is
redirected. A file handler is installed writing to `simulator.log` in the
current working directory (opened with `mode="w"`, so it is truncated on each
start), and the console handlers are raised to `WARNING` so that only real
problems break through the dashboard. If you started the dashboard and the logs
seem to have vanished, they are in `simulator.log`.

**With `--json-output`:** stdout carries a periodic machine-readable state
document instead. Human log lines still go to stderr, so redirect the two
separately if you are parsing:

```bash
mks-servo-simulator --json-output 2>simulator.err | your-consumer
```

## What to look for

**On startup**, confirm the simulator came up the way you meant:

```text
... - MKSSimulatorCLI - INFO - Starting MKS Servo CAN Simulator...
... - MKSSimulatorCLI - INFO - Debug API server starting on http://127.0.0.1:8765
... - MKSSimulatorCLI - INFO - Dashboard (for humans): http://127.0.0.1:8765/dashboard
... - MKSSimulatorCLI - INFO - Status JSON (for agents): http://127.0.0.1:8765/status
```

If you asked for `--debug-api` and instead see

```text
... - MKSSimulatorCLI - ERROR - Failed to start debug API server: ...
... - MKSSimulatorCLI - ERROR - Install FastAPI and uvicorn: pip install fastapi uvicorn
```

the simulator keeps running without it — the CAN side is unaffected, but nothing
will answer on port 8765.

**A motor that never answers** is usually a CAN ID mismatch rather than a
protocol fault. The simulator creates IDs `--start-can-id` through
`--start-can-id + --num-motors - 1`; a request to any other ID is simply not
answered, and at `INFO` there is nothing to see. Drop to `--log-level debug` and
you will see the frame arrive and go nowhere.

**A profile that did not do what you asked** announces itself:

```text
... - MKSSimulatorCLI - INFO - Loaded configuration profile: bench
... - MKSSimulatorCLI - INFO - Using 3 motors from profile configuration
```

That second line is worth reading. A profile overrides the flags next to it on
the command line — see [Command-Line Options](cli_options.md#configuration-profiles).

## Logs are not the primary interface

The log is a narrative; it is not a good way to ask "what is the motor doing
right now". For that, use the debug API — `/status` returns the complete state
as JSON, and `/dashboard` renders that same payload for a human. Both are
described in the [HTTP Debug API Guide](../http_debug_api.md).

This matters particularly if you are driving the simulator from a program or an
agent: parsing log lines to infer state means writing a parser against text that
is not a contract, and inferring a move has finished from the absence of further
output is exactly the kind of guess that produces flaky results. Poll `/status`
instead.
