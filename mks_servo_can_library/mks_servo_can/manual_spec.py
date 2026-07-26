"""
The machine-readable transcription of the MKS SERVO42D/57D CAN manual.

The specification describes every command the motors accept: its code, name,
payload layout, expected reply and DLC. It ships *inside the package*, next to
the constants it describes, so an installed wheel carries it too. It used to be
read out of `tests/fixtures/`, which meant anyone who installed rather than
cloned got an empty command reference and no explanation.

Both the simulator (for its command reference and its command log) and the
conformance tests read it from here, so there is exactly one copy to correct
when the manual is found to be wrong - and it has been wrong; see the `errata`
block for the sign convention.
"""
import json
from functools import lru_cache
from typing import Any, Dict

try:  # Python 3.9+
    from importlib.resources import files as _resource_files
except ImportError:  # pragma: no cover - Python 3.8 fallback
    _resource_files = None
    import importlib_resources  # type: ignore

_DATA_PACKAGE = "mks_servo_can.data"
_SPEC_FILENAME = "manual_commands_v106.json"


@lru_cache(maxsize=1)
def load_manual_spec() -> Dict[str, Any]:
    """
    Loads the full manual specification.

    Read through `importlib.resources` rather than by path arithmetic, so it
    works from a wheel, a zip import and a source checkout alike.

    Returns:
        The parsed specification: `commands`, `categories`, `protocol`,
        `errata`, `firmware_history` and `source`.

    Raises:
        FileNotFoundError: If the package data was not installed.
        ValueError: If the specification is not valid JSON.
    """
    if _resource_files is not None:
        resource = _resource_files(_DATA_PACKAGE).joinpath(_SPEC_FILENAME)
        text = resource.read_text(encoding="utf-8")
    else:  # pragma: no cover - Python 3.8 fallback
        text = importlib_resources.files(_DATA_PACKAGE).joinpath(
            _SPEC_FILENAME
        ).read_text(encoding="utf-8")
    return json.loads(text)


def get_manual_commands() -> Dict[str, Any]:
    """
    Returns the command table, keyed by two-digit uppercase hex code.

    Returns:
        Mapping of command code (e.g. `"F5"`) to its specification.
    """
    return load_manual_spec()["commands"]


def get_firmware_history() -> Dict[str, Any]:
    """
    Returns which firmware release introduced each command.

    Taken from the revision table at the front of the manual. Used to turn
    "this board answers command X" into a lower bound on its firmware version,
    since the boards expose no version command of their own.

    Returns:
        Mapping of dotted release (e.g. `"1.0.6"`) to the list of command codes
        it added, as `"0x35"`-style strings.
    """
    return load_manual_spec().get("firmware_history", {})


def get_firmware_probes() -> Dict[str, str]:
    """
    Returns the commands that are safe to send when identifying a board.

    Deliberately a curated subset of `get_firmware_history()`: only commands
    that *read*. Probing with a setter is unsafe because several accept a frame
    shorter than their documented payload, answer with a success status, and
    write whatever they found.

    Returns:
        Mapping of command code (e.g. `"0x35"`) to the release that added it.
        Keys beginning with an underscore are commentary, not probes.
    """
    return load_manual_spec().get("firmware_probes", {})


def get_firmware_fixes() -> Dict[str, Any]:
    """
    Returns behavioural fixes that added no new command.

    These cannot be probed directly - there is nothing to ask the board - so
    their presence can only be inferred from a version floor established by the
    probes, and otherwise has to be reported as unknown.

    Returns:
        Mapping of dotted release to a list of human-readable descriptions.
    """
    return load_manual_spec().get("firmware_fixes", {})


def get_manual_errata() -> Dict[str, Any]:
    """
    Returns the recorded contradictions in the published manual.

    The manual contradicts itself on the sign convention: the prose above the
    0x31/0x35 worked examples says clockwise is positive, while the examples
    themselves and the 0x32 note say counter-clockwise is. The examples are
    taken as authoritative here.

    Returns:
        The errata block, or an empty mapping if the specification carries none.
    """
    return load_manual_spec().get("errata", {})
