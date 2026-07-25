# Installation Guide

There is one distribution, `mks-servo-can`. The simulator is an extra of it, not
a separate package. Install into a virtual environment.

## Prerequisites

See the [Prerequisites document](./prerequisites.md). In short:

* **Python 3.9 or newer.** 3.9 is the floor and it is tested; older versions are
  not supported, whatever older packaging metadata may have claimed.
* `pip`
* a virtual environment (recommended)

No CAN hardware is needed for anything except talking to a real motor.

## Install

```bash
git clone https://github.com/ricardodeazambuja/mks_servo_can.git
cd mks_servo_can

python -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate

pip install .[simulator]
```

(Not yet on PyPI — installing means cloning first. See
[the roadmap](../development/roadmap.md) for what remains.)

### Which extra

| command | what you get |
|---|---|
| `pip install .` | the library alone — enough to drive real motors |
| `pip install .[simulator]` | the above plus the `mks-servo-simulator` command, its browser dashboard and its HTTP debug API |
| `pip install .[dashboard]` | adds `textual`, needed *only* for the legacy `--textual-dashboard` TUI |
| `pip install .[monitoring]` | adds `psutil`, for the advanced performance-monitoring panels |
| `pip install -e .[dev]` | everything above plus pytest and ruff — for working on the project |

Extras combine: `pip install .[simulator,monitoring]`.

`python-can` is a hard dependency of the library and comes in with every one of
these, since it is what talks to real hardware.

## Verify

```bash
python -c "import mks_servo_can; print(mks_servo_can.__version__)"
mks-servo-simulator --help
```

Then start a simulator and drive it:

```bash
# In one terminal
mks-servo-simulator --num-motors 4 --start-can-id 1 --latency-ms 5 --debug-api

# In another
python examples/benchmark_command_latency.py
```

With `--debug-api` you can also open <http://127.0.0.1:8765/dashboard> in a
browser, or poll <http://127.0.0.1:8765/status> for the same state as JSON.

## For development

```bash
pip install -e .[dev]
pytest -q
ruff check .
```

See [Setting up a Development Environment](../development/setup.md) and
[Running Tests](../development/running_tests.md).

## A note on the old layout

Until recently this was **two** distributions, installed from
`mks_servo_can_library/` and `mks_servo_simulator/` with a `setup.py` each. If
you have instructions or a script that does that, it will no longer work: those
`setup.py` files are gone and the packaging lives in the root `pyproject.toml`.

The merge also removed the `PYTHONPATH` advice that used to be needed here. The
simulator hard-depends on the library — it shares its constants, CRC and motion
model rather than reimplementing them — so a single install now puts both
packages in the same environment by construction, and there is nothing left to
mismatch.
