"""
The manual's command specification must ship inside the library (defect L7).

It used to be read out of `tests/fixtures/` by path arithmetic. An installed
wheel has no `tests/` directory, so anyone who installed rather than cloned got
an empty command reference from the simulator's `/commands` endpoint and an
`available_commands` count of zero, with nothing to explain it.

These tests pin the two properties that matter: the data is inside the package,
and it is reached through `importlib.resources` rather than by walking up from
`__file__` - so it survives being installed, zipped, or imported from a
different working directory.
"""
import json
import pathlib
import subprocess
import sys

import pytest

import mks_servo_can
from mks_servo_can import get_manual_commands, get_manual_errata, load_manual_spec

LIB_ROOT = pathlib.Path(mks_servo_can.__file__).resolve().parent


def test_specification_lives_inside_the_package():
    """The JSON must be package data, not a test fixture."""
    spec_path = LIB_ROOT / "data" / "manual_commands_v106.json"
    assert spec_path.is_file(), (
        f"the manual specification is not in the package at {spec_path}"
    )
    payload = json.loads(spec_path.read_text())
    assert payload["commands"], "the packaged specification has no commands"


def test_no_module_reads_the_specification_by_path():
    """
    Nothing may reach for the file by walking the filesystem.

    Path arithmetic is what broke this twice: once by being off a level, and
    once by pointing into a directory that installs do not have.
    """
    offenders = []
    roots = [
        LIB_ROOT,
        pathlib.Path(__file__).resolve().parents[2] / "mks_servo_simulator",
    ]
    for root in roots:
        for module in root.rglob("*.py"):
            # manual_spec.py is the one place allowed to name the file: it is
            # the loader everything else goes through.
            if module.name == "manual_spec.py":
                continue
            if "manual_commands_v106.json" in module.read_text(encoding="utf-8"):
                offenders.append(str(module))
    assert not offenders, (
        "these modules name the specification file directly instead of using "
        "mks_servo_can.manual_spec:\n  " + "\n  ".join(offenders)
    )


def test_specification_loads_from_an_unrelated_working_directory(tmp_path):
    """
    Loading must not depend on where the process was started.

    Run in a subprocess from an empty directory, which is the closest thing to
    "installed somewhere else" that a source checkout can offer.
    """
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from mks_servo_can import get_manual_commands; "
            "print(len(get_manual_commands()))",
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"loading the specification from {tmp_path} failed:\n{result.stderr}"
    )
    assert int(result.stdout.strip()) > 0, "no commands were loaded"


def test_commands_are_keyed_by_hex_code():
    """The command table is a mapping, not a list.

    `debug_tools` assumed a list of dicts and so silently fell back to a
    hard-coded table with several wrong names.
    """
    commands = get_manual_commands()
    assert isinstance(commands, dict)
    for code, spec in commands.items():
        assert code.startswith("0x"), f"command key {code!r} is not a hex code"
        int(code, 16)
        assert "name" in spec, f"{code} has no name"


def test_errata_are_carried_with_the_specification():
    """The manual contradicts itself; the record of that must travel with it."""
    errata = get_manual_errata()
    assert "sign_convention" in errata, (
        "the sign-convention erratum is missing - it is the reason the worked "
        "examples rather than the prose are taken as authoritative"
    )


def test_repeated_loads_return_the_same_object():
    """The specification is read once; callers must not pay for it repeatedly."""
    assert load_manual_spec() is load_manual_spec()


@pytest.mark.parametrize("command_code", ["0xF5", "0xF4", "0x31"])
def test_commands_the_streaming_api_depends_on_are_present(command_code):
    """A truncated specification must not pass unnoticed."""
    assert command_code in get_manual_commands()
