"""
The Textual dashboard renders the snapshot, and stays off the motor loop.

Two loose ends from the simulator work:

1. It was the only reporting surface still reading motor attributes directly,
   through `getattr(motor, 'name', default)` chains. It happened to read the
   right ones, but that is precisely the arrangement that produced the drift
   removed from the other three surfaces - where `/status` reported zeros for a
   moving motor because the attributes it asked for had never existed.
2. It ran on the simulator's own event loop, so the TUI's rendering and input
   handling competed with the 10 ms motor integration tick. Watching the
   simulation slowed the simulation down.

The rendering is tested against a real `SimulatedMotor` rather than a mock, and
the two arrangements are gated at the source level, in the same style as the
hot-path logging gate in tests/unit/test_regressions.py.
"""
import ast
import asyncio
import pathlib
import re

import pytest

from mks_simulator.interface.textual_dashboard import motor_details, motor_row
from mks_simulator.motor_model import SimulatedMotor

SIM_ROOT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "mks_servo_simulator"
    / "mks_simulator"
)
DASHBOARD_SOURCE = SIM_ROOT / "interface" / "textual_dashboard.py"
CLI_SOURCE = SIM_ROOT / "cli.py"


@pytest.fixture
def motor(event_loop=None):
    """A real simulated motor, parked at zero."""
    return SimulatedMotor(1, asyncio.get_event_loop_policy().new_event_loop())


def test_row_tracks_a_real_motor(motor):
    """The row must show where the motor actually is."""
    before = motor_row(motor.status_snapshot())
    assert before[0] == "1"
    assert before[8] == "No", "a fresh motor is not enabled"

    motor.is_enabled = True
    motor.position_steps = motor.steps_per_rev_encoder / 4.0  # a quarter turn

    after = motor_row(motor.status_snapshot())
    assert after[8] == "Yes", "the row did not follow the enable state"
    assert after[2] == "90.0 deg", f"expected 90 degrees, row showed {after[2]}"
    assert after != before


def test_row_has_one_cell_per_column():
    """A row that does not match the table's columns raises at render time."""
    columns = 9  # ID, Status, Position, Speed, Target, Velocity, WorkMode, MStep, Enabled
    loop = asyncio.get_event_loop_policy().new_event_loop()
    row = motor_row(SimulatedMotor(3, loop).status_snapshot())
    assert len(row) == columns


def test_details_track_a_real_motor(motor):
    """The detail pane must follow the same source of truth."""
    motor.is_enabled = True
    motor.position_steps = motor.steps_per_rev_encoder / 2.0
    lines = motor_details(motor.status_snapshot())
    joined = "\n".join(lines)

    assert "Motor ID 1" in joined
    assert "Enabled: Yes" in joined
    assert "180.0 deg" in joined, f"detail pane did not show the position:\n{joined}"


def test_dashboard_reads_no_motor_attributes_directly():
    """
    Every value shown must come from `status_snapshot()`.

    Reading attributes off the motor is the pattern that let three other
    surfaces drift into reporting fiction.
    """
    source = DASHBOARD_SOURCE.read_text(encoding="utf-8")
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"getattr\(\s*motor\b", line)
    ]
    assert not offenders, (
        "the Textual dashboard reads motor attributes directly instead of "
        "rendering MotorSnapshot:\n  " + "\n  ".join(offenders)
    )


def test_dashboard_is_not_scheduled_on_the_simulator_loop():
    """
    The TUI must not be started with `loop.create_task(...)`.

    Sharing the simulator's loop puts terminal rendering in competition with
    the motor integration tick.
    """
    tree = ast.parse(CLI_SOURCE.read_text(encoding="utf-8"))
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and func.attr == "create_task"
            and isinstance(func.value, ast.Name)
            and func.value.id == "loop"
        ):
            called = ast.unparse(node.args[0]) if node.args else ""
            if "textual" in called.lower():
                offenders.append(f"line {node.lineno}: {called}")
    assert not offenders, (
        "the Textual dashboard is scheduled on the simulator's event loop:\n  "
        + "\n  ".join(offenders)
    )


def test_dashboard_is_started_on_its_own_thread():
    """The positive half: it must actually be given a thread."""
    source = CLI_SOURCE.read_text(encoding="utf-8")
    assert "threading.Thread(" in source and "textual" in source, (
        "the CLI no longer starts the Textual dashboard on its own thread"
    )
