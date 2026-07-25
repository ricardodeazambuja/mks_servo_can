"""
The simulator's own entry points must start.

Each mode here shipped broken, and none of the breakages were unit-testable:
they lived in the wiring between the CLI, the configuration manager and the
motor model, which no test ever executed.

- `--json-output` raised AttributeError on the first status emission and died
  before serving anything, because the debug interface read `motor.name`.
- `--config-profile` raised TypeError constructing the first motor, because the
  CLI passed three keyword arguments `SimulatedMotor` has never accepted.
- The package printed a banner to stdout on import, so the first line of the
  machine-readable JSON stream was not JSON.

So these tests run the real console script in a subprocess and check it comes
up. They are slower than a unit test and they are the only kind that would have
caught any of the above.
"""
import json
import pathlib
import shutil
import socket
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.integration

SCRIPT_NAME = "mks-servo-simulator"


def _simulator_command() -> str:
    """
    Returns the console script belonging to the interpreter running the tests.

    Deliberately *not* a bare PATH lookup. `shutil.which` finds whichever
    `mks-servo-simulator` happens to come first on PATH, which need not belong
    to the environment the test imported `mks_simulator` from - so a machine
    with an old install elsewhere would exercise that one and report on code
    nobody is looking at. That is not hypothetical: it is what happened the
    first time this suite was run against an installed wheel in a fresh
    virtualenv, and the failure looked like a packaging defect rather than a
    test looking in the wrong place.

    Returns:
        Path to the script beside `sys.executable` if it exists, else whatever
        is on PATH, else None.
    """
    beside_interpreter = pathlib.Path(sys.executable).parent / SCRIPT_NAME
    if beside_interpreter.exists():
        return str(beside_interpreter)
    return shutil.which(SCRIPT_NAME)


SIMULATOR_CMD = _simulator_command()


def _free_port() -> int:
    """Returns a TCP port that is free right now."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(autouse=True)
def _require_simulator():
    """Skips these tests where the console script is not installed."""
    if SIMULATOR_CMD is None:
        pytest.skip(f"{SCRIPT_NAME} is not installed for {sys.executable}")


def _run_briefly(args, seconds=6.0):
    """
    Starts the simulator, lets it settle, and returns its output.

    Args:
        args: Extra command-line arguments.
        seconds: How long to let it run before terminating.

    Returns:
        A tuple of (stdout, stderr, returncode).
    """
    proc = subprocess.Popen(
        [SIMULATOR_CMD, *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(seconds)
    finally:
        proc.terminate()
        try:
            out, err = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, err = proc.communicate(timeout=10)
    return out, err, proc.returncode


class TestJSONOutputMode:
    """The mode an agent consumes must emit nothing but JSON, and must survive."""

    def test_emits_only_parseable_json_lines(self):
        out, err, _ = _run_briefly(
            ["--port", str(_free_port()), "--json-output", "--num-motors", "2"]
        )
        lines = [line for line in out.splitlines() if line.strip()]
        assert lines, f"no stdout produced. stderr:\n{err[-2000:]}"

        for i, line in enumerate(lines):
            try:
                json.loads(line)
            except json.JSONDecodeError as exc:
                pytest.fail(
                    f"stdout line {i} is not JSON ({exc}): {line!r}\n"
                    "Anything written to stdout in this mode corrupts the "
                    "event stream."
                )

    def test_does_not_crash_on_startup(self):
        _, err, _ = _run_briefly(
            ["--port", str(_free_port()), "--json-output", "--num-motors", "1"]
        )
        assert "Traceback" not in err, f"simulator crashed:\n{err[-3000:]}"
        assert "AttributeError" not in err

    def test_status_updates_describe_the_motors(self):
        out, _, _ = _run_briefly(
            ["--port", str(_free_port()), "--json-output", "--num-motors", "2"]
        )
        events = [json.loads(line) for line in out.splitlines() if line.strip()]
        updates = [e for e in events if e.get("event") == "status_update"]
        assert updates, f"no status_update emitted. events: {[e.get('event') for e in events]}"

        motors = updates[-1]["motors"]
        assert set(motors) == {"1", "2"}
        for motor in motors.values():
            # Fields an agent needs in order to reason about the motor at all.
            for field in (
                "position_steps", "position_degrees", "enabled", "moving",
                "current_rpm", "status_text", "work_mode_name",
            ):
                assert field in motor, f"status_update motor is missing '{field}'"

    def test_command_reference_is_populated(self):
        """
        The spec an agent consults must load.

        Its path was off by one directory, so this silently reported zero.
        """
        out, _, _ = _run_briefly(
            ["--port", str(_free_port()), "--json-output", "--num-motors", "1"]
        )
        updates = [
            json.loads(line)
            for line in out.splitlines()
            if line.strip() and json.loads(line).get("event") == "status_update"
        ]
        assert updates[-1]["available_commands"] > 0


class TestConfigProfileMode:
    """Loading a saved profile must build motors rather than raise."""

    def test_profile_round_trip_starts_the_simulator(self, tmp_path):
        config_dir = tmp_path / "config"
        port = _free_port()

        # Write a profile through the real configuration manager, so the file
        # on disk is exactly what the CLI will be asked to read back.
        from mks_simulator.interface.config_manager import ConfigurationManager

        manager = ConfigurationManager(str(config_dir))
        config = manager.create_default_config(2, 1)
        config.port = port
        assert manager.save_config(config, "pytest_profile")

        _, err, _ = _run_briefly(
            [
                "--port", str(port),
                "--config-dir", str(config_dir),
                "--config-profile", "pytest_profile",
            ]
        )
        assert "Traceback" not in err, f"--config-profile crashed:\n{err[-3000:]}"
        assert "TypeError" not in err
        assert "Failed to load configuration profile" not in err


class TestPackageImportIsSilent:
    """Importing the package must not write to stdout."""

    def test_import_prints_nothing(self):
        result = subprocess.run(
            [sys.executable, "-c", "import mks_simulator"],
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert result.stdout == "", (
            f"importing the package wrote to stdout: {result.stdout!r}. "
            "In --json-output mode stdout is the event stream."
        )
