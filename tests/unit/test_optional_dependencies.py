"""
The optional extras must actually be optional (defect L10).

`pip install mks-servo-can[simulator]` promises a working simulator. It did not
deliver one: `cli.py` imported the legacy Textual dashboard at module scope, so
`textual` was a hard requirement of the entire simulator - including
`--debug-api` and `--json-output`, which have nothing to do with a TUI - while
being declared in no install requirement anywhere. The `mks-servo-simulator`
command died with `ModuleNotFoundError: No module named 'textual'` before
parsing a single argument.

Nothing caught it because every environment that has ever run this suite had
`textual` installed for `tests/test_textual_dashboard.py`. That is the shape to
watch for: a dependency that is present for a *different* reason in every
environment that tests it.

These tests block the optional module at import time in a subprocess, which is
the only way to ask "would this work on a machine that does not have it?"
without uninstalling anything.
"""
import subprocess
import sys
import textwrap

import pytest

# Modules that are declared in an extra other than `simulator`, and so may be
# absent on a machine that installed `mks-servo-can[simulator]`.
OPTIONAL_MODULES = [
    pytest.param("textual", id="textual (the [dashboard] extra)"),
    pytest.param("psutil", id="psutil (the [monitoring] extra)"),
]


def _run_without(module_name, body):
    """
    Runs `body` in a subprocess where importing `module_name` fails.

    Args:
        module_name: Top-level module to make unimportable.
        body: Python source to run once the block is in place.

    Returns:
        The finished CompletedProcess.
    """
    program = textwrap.dedent(
        f"""
        import sys

        class _Blocked:
            def find_module(self, name, path=None):
                return self.find_spec(name, path)

            def find_spec(self, name, path=None, target=None):
                if name == {module_name!r} or name.startswith({module_name!r} + "."):
                    raise ImportError("blocked for test: " + name)
                return None

        sys.meta_path.insert(0, _Blocked())
        # Anything already imported would defeat the block.
        for loaded in list(sys.modules):
            if loaded == {module_name!r} or loaded.startswith({module_name!r} + "."):
                del sys.modules[loaded]
        """
    ) + textwrap.dedent(body)
    return subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_the_block_actually_blocks():
    """
    Guard against the harness silently doing nothing.

    If the import block stopped working, every other test in this file would
    pass vacuously - which is precisely the failure mode they exist to catch.
    """
    result = _run_without("textual", "import textual\n")
    assert result.returncode != 0, (
        "the import block did not block anything; the rest of this file proves "
        "nothing"
    )
    assert "blocked for test" in result.stderr


@pytest.mark.parametrize("module_name", OPTIONAL_MODULES)
def test_the_simulator_cli_imports_without(module_name):
    """The CLI must not drag an optional extra in at module scope."""
    result = _run_without(
        module_name,
        """
        import mks_simulator.cli
        print("imported")
        """,
    )
    assert result.returncode == 0, (
        f"mks_simulator.cli cannot be imported without {module_name}, so a "
        f"`pip install mks-servo-can[simulator]` that omits it is broken:\n"
        f"{result.stderr}"
    )


@pytest.mark.parametrize("module_name", OPTIONAL_MODULES)
def test_the_simulator_command_runs_without(module_name):
    """
    `mks-servo-simulator --help` must work.

    This is the entry point a user reaches first, and the one that failed: the
    console script imports `mks_simulator.main`, which imports the package,
    which imported the dashboard.
    """
    result = _run_without(
        module_name,
        """
        from click.testing import CliRunner

        from mks_simulator.cli import main

        outcome = CliRunner().invoke(main, ["--help"])
        assert outcome.exit_code == 0, outcome.output
        assert "--debug-api" in outcome.output
        print("ok")
        """,
    )
    assert result.returncode == 0, (
        f"the simulator command does not run without {module_name}:\n"
        f"{result.stderr}"
    )


def test_asking_for_the_textual_dashboard_without_textual_says_what_to_install():
    """
    The one flag that genuinely needs the extra must fail comprehensibly.

    Silence, or a bare ModuleNotFoundError from three frames down, is what sent
    this defect unnoticed in the first place. The message has to name the extra.
    """
    result = _run_without(
        "textual",
        """
        try:
            from mks_simulator.interface.textual_dashboard import TextualDashboard  # noqa: F401
        except ImportError:
            pass
        else:
            raise AssertionError("the dashboard imported without textual")
        print("ok")
        """,
    )
    assert result.returncode == 0, result.stderr

    # And the CLI branch turns that into advice rather than a traceback.
    source = (
        __import__("pathlib")
        .Path(__import__("mks_simulator").__file__)
        .parent.joinpath("cli.py")
        .read_text(encoding="utf-8")
    )
    assert "mks-servo-can[dashboard]" in source, (
        "the --textual-dashboard failure path must name the extra to install"
    )
