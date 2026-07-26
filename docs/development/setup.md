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

One distribution, one command:

```bash
git clone https://github.com/ricardodeazambuja/mks_servo_can.git
cd mks_servo_can

python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate

pip install -e .[dev]
```

The two packages still live in subdirectories — the library in
`mks_servo_can_library/mks_servo_can/` and the simulator in
`mks_servo_simulator/mks_simulator/` — but they are built and installed as one
distribution, `mks-servo-can`, declared in the root `pyproject.toml`. There is
no `setup.py` in either subdirectory any more, and installing from one of them
will not work.

`[dev]` is the extra to use while working on the project: it pulls in every
other extra plus the test tooling, so the whole suite is runnable from that one
install. The extras exist so that a *user* need not take what they do not use:

| extra | pulls in | needed for |
|---|---|---|
| `simulator` | click, rich, fastapi, uvicorn | the `mks-servo-simulator` command, its browser dashboard and its debug API |
| `dashboard` | textual | only the legacy `--textual-dashboard` TUI |
| `monitoring` | psutil | only the advanced performance-monitoring panels |
| `dev` | all of the above + pytest, ruff | the test suite |

`dashboard` and `monitoring` are genuinely optional and
`tests/unit/test_optional_dependencies.py` keeps them that way — it blocks each
module at import time and asserts the simulator still starts. That test exists
because `textual` was once imported at module scope and declared nowhere, which
made a clean install of the simulator fail before it parsed an argument.

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

## Where things live

| path | what it is |
|---|---|
| `pyproject.toml` | packaging, extras, entry points, ruff and coverage config — all of it |
| `mks_servo_can_library/mks_servo_can/` | the library |
| `mks_servo_can_library/mks_servo_can/data/` | the packaged transcription of the MKS manual, read at runtime by both the simulator and the conformance tests |
| `mks_servo_simulator/mks_simulator/` | the simulator |
| `tests/` | the suite — `unit/`, `integration/`, `simulator_compliance/`, `hil/`, `determinism/` |
| `examples/` | runnable scripts, most of which work against the simulator |
| `docs/` | this documentation |
| `docs/development/roadmap.md` | what is planned next, in priority order |
