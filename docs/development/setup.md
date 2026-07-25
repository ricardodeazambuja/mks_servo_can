# Setting up a Development Environment

This page covers getting a working checkout. If you only want to *use* the
library, [the installation guide](../getting_started/installation.md) is shorter
and enough.

## What you need

* **Python 3.9 or newer.** 3.9 is the floor and it is enforced: the CI matrix
  runs 3.9 through 3.13, and `tests/unit/test_regressions.py` asserts the
  package still imports on the oldest one. That is why the code uses
  `typing.List` rather than PEP 585 builtin generics, and why the `UP006`,
  `UP007` and `UP035` lint rules are switched off in `pyproject.toml`.
* **Git.**
* **No CAN hardware.** The simulator emulates the protocol over TCP, so the
  entire library, the examples and all but the `tests/hil/` suite run on a
  laptop with nothing plugged in.

## Clone and install

The repository holds two distributions in subdirectories — the library in
`mks_servo_can_library/` and the simulator in `mks_servo_simulator/`. Both are
installed editable into the same virtual environment:

```bash
git clone https://github.com/ricardodeazambuja/mks_servo_can.git
cd mks_servo_can

python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate

pip install -e ./mks_servo_can_library[dev]
pip install -e ./mks_servo_simulator
```

Install the library **first**. The simulator declares a hard dependency on it —
it shares the library's constants, CRC and motion model rather than
reimplementing them — and installing the library from the checkout first stops
pip reaching out to PyPI for it.

The simulator also needs `click`, `rich`, `fastapi` and `uvicorn`; they come in
via its own `install_requires`. `psutil` is optional and only enables the
advanced performance-monitoring panels.

## Check it works

Start the simulator in one terminal:

```bash
mks-servo-simulator --num-motors 2 --debug-api
```

and in another, confirm the test suite is green:

```bash
pytest -q
```

The suite starts and stops its own simulator instances on their own ports, so it
does not need the one you just started — but if it cannot start one, that is the
first thing to investigate. See [Running Tests](running_tests.md).

## Editor and tooling

Linting and formatting are configured once, project-wide, in `pyproject.toml`
under `[tool.ruff]`. There is no per-package configuration and no separate
`.flake8`, `setup.cfg` or `.pylintrc` — those existed once and none of them was
ever enforced. Point your editor at `ruff` and it will agree with CI. See
[Coding Standards](coding_standards.md).

## A note on `PYTHONPATH`

If the simulator cannot find `mks_servo_can`, the usual cause is that the two
packages were installed into different environments, or that the library was
never installed at all and you are relying on the current working directory.
Editable installs of both into one virtualenv is the arrangement everything else
here assumes; reach for `PYTHONPATH` only if you have a reason not to.

## Where things live

| path | what it is |
|---|---|
| `mks_servo_can_library/mks_servo_can/` | the library |
| `mks_servo_can_library/mks_servo_can/data/` | the packaged transcription of the MKS manual, read at runtime by both the simulator and the conformance tests |
| `mks_servo_simulator/mks_simulator/` | the simulator |
| `tests/` | the suite — `unit/`, `integration/`, `simulator_compliance/`, `hil/`, `determinism/` |
| `examples/` | runnable scripts, most of which work against the simulator |
| `docs/` | this documentation |
| `docs/development/roadmap.md` | what is planned next, in priority order |
