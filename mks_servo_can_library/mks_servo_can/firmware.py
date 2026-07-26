"""Identifying what a connected MKS SERVO board's firmware can actually do.

The boards expose **no firmware version command**. The version appears on the
OLED at boot and nowhere on the CAN bus, so a library cannot ask "which version
are you?". What it can do is ask "do you answer this command?", and the manual's
revision history maps commands to the release that introduced them. Answering a
command therefore establishes a *floor* on the version.

Two consequences shape everything here.

**Probes must only read.** Several MKS setters accept a frame shorter than
their documented payload, reply with a success status, and write whatever they
found - a bare ``0x84`` silently corrupts the subdivision, which shows up much
later as relative moves running at the wrong speed. So the probe set is drawn
from ``firmware_probes`` in the manual spec, which contains read commands only.

**The result is a floor, never an exact version.** Everything V1.0.4 and V1.0.5
added is a setter or an action (``0xF7``, ``0xFE``, ``0x94``, ``0x41``), none of
which can be probed safely. A board that fails the V1.0.6 probes could be
anything from V1.0.0 to V1.0.5, so behaviour that depends on a V1.0.5 fix is
reported as *unknown*, not as absent. Claiming otherwise would be guessing.
"""
import logging
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from . import constants as const
from .exceptions import MKSServoError
from .manual_spec import get_firmware_fixes, get_firmware_probes

logger = logging.getLogger(__name__)

# Parameter read through 0x00 to test whether the command exists at all. The
# work mode is a good choice: every board has one, and knowing it materially
# improves move timeout estimates.
_PROBE_PARAMETER_CODE = const.CMD_SET_WORK_MODE

# A board that implements a read command answers it in single-digit
# milliseconds, so a probe only has to outlast bus latency, not a real
# operation. Every unsupported command costs this much wall clock during
# initialisation, once per axis.
DEFAULT_PROBE_TIMEOUT = 0.25

# The transport logs an unanswered command at WARNING, which is right for a
# command the caller expected to work and wrong for a probe, where silence is
# the measurement. Probing an older board would otherwise print a wall of
# warnings on every startup.
# Both layers log an unanswered command: the transport reports the timeout and
# the API layer re-reports it as a communication error.
_TRANSPORT_LOGGERS = (
    "mks_servo_can.can_interface",
    "mks_servo_can.low_level_api",
)


@contextmanager
def _expected_silence():
    """Suppresses transport warnings for commands a board may legitimately lack."""
    loggers = [logging.getLogger(name) for name in _TRANSPORT_LOGGERS]
    previous = [each.level for each in loggers]
    for each in loggers:
        each.setLevel(logging.ERROR)
    try:
        yield
    finally:
        for each, level in zip(loggers, previous):
            each.setLevel(level)


def _version_tuple(text: str):
    """Parses a dotted version string into a comparable tuple of ints."""
    try:
        return tuple(int(part) for part in text.split("."))
    except (AttributeError, ValueError):
        return (0,)


@dataclass(frozen=True)
class FirmwareCapabilities:
    """What a specific board was observed to support.

    Attributes:
        supported_commands: Probe command codes the board answered.
        unsupported_commands: Probe command codes the board ignored.
        minimum_version: Highest firmware release whose probes all passed, or
            None when even the oldest probe failed. This is a lower bound.
        work_mode: The work mode read back, or None when unreadable. Drives the
            RPM ceiling used to size move timeouts.
        microsteps: The subdivision read back, or None when unreadable.
        probed: False when detection was skipped, in which case every other
            field is a default rather than an observation.
    """

    supported_commands: List[int] = field(default_factory=list)
    unsupported_commands: List[int] = field(default_factory=list)
    minimum_version: Optional[str] = None
    work_mode: Optional[int] = None
    microsteps: Optional[int] = None
    probed: bool = False

    def supports(self, command_code: int) -> bool:
        """Whether a probed command is known to work on this board.

        Args:
            command_code: The MKS command code to ask about.

        Returns:
            True only when the command was probed and answered. A command that
            was never probed returns False, because the honest answer is "not
            known to work".
        """
        return command_code in self.supported_commands

    @property
    def can_read_system_parameters(self) -> bool:
        """Whether ``0x00`` works, which also means the board is V1.0.6+."""
        return self.supports(const.CMD_READ_SYSTEM_PARAMETER_PREFIX)

    @property
    def can_read_raw_encoder(self) -> bool:
        """Whether ``0x35`` works."""
        return self.supports(const.CMD_READ_RAW_ENCODER_ADDITION)

    def has_fix(self, version: str) -> Optional[bool]:
        """Whether a behavioural fix from `version` is present.

        Args:
            version: Dotted release that introduced the fix, e.g. "1.0.5".

        Returns:
            True when the established floor is at or above `version`. None when
            the floor is lower, because the fixes added by V1.0.4 and V1.0.5
            introduced no probeable command and so cannot be ruled out.
        """
        if self.minimum_version is None:
            return None
        if _version_tuple(self.minimum_version) >= _version_tuple(version):
            return True
        return None

    def describe(self) -> str:
        """Renders a short human-readable summary for logs and status output."""
        if not self.probed:
            return "firmware not probed"
        floor = f"v{self.minimum_version}+" if self.minimum_version else "pre-v1.0.3"
        bits = [floor]
        if self.work_mode is not None:
            bits.append(f"work mode {self.work_mode}")
        else:
            bits.append("work mode unreadable")
        if self.microsteps is not None:
            bits.append(f"{self.microsteps} microsteps")
        return ", ".join(bits)


async def probe_firmware(
    low_level_api, can_id: int, timeout: float = DEFAULT_PROBE_TIMEOUT
) -> FirmwareCapabilities:
    """Identifies a board's capabilities using read-only commands.

    Sends each command in the manual spec's ``firmware_probes`` table and
    records which are answered. Silence is treated as "not supported": these
    are all read commands, so a board that implements one always replies.

    Args:
        low_level_api: The `LowLevelAPI` to send probes through.
        can_id: CAN ID of the board to identify.
        timeout: Seconds to wait for each probe. Kept short because a
            non-response is the expected outcome for older boards, and the
            probes run during initialisation.

    Returns:
        A `FirmwareCapabilities` describing what was observed. Never raises for
        an unsupported command - that is the thing being measured.
    """
    probes: Dict[str, str] = get_firmware_probes()
    supported: List[int] = []
    unsupported: List[int] = []
    passed_versions: List[str] = []

    with _expected_silence():
        for code_text, introduced_in in sorted(probes.items()):
            if code_text.startswith("_"):
                continue
            code = int(code_text, 16)
            if await _probe_one(low_level_api, can_id, code, timeout):
                supported.append(code)
                passed_versions.append(introduced_in)
            else:
                unsupported.append(code)

    minimum_version = max(passed_versions, key=_version_tuple) if passed_versions else None

    work_mode: Optional[int] = None
    microsteps: Optional[int] = None
    if const.CMD_READ_SYSTEM_PARAMETER_PREFIX in supported:
        with _expected_silence():
            work_mode = await _read_parameter_byte(
                low_level_api, can_id, const.CMD_SET_WORK_MODE, timeout
            )
            microsteps = await _read_parameter_byte(
                low_level_api, can_id, const.CMD_SET_SUBDIVISION, timeout
            )

    caps = FirmwareCapabilities(
        supported_commands=supported,
        unsupported_commands=unsupported,
        minimum_version=minimum_version,
        work_mode=work_mode,
        microsteps=microsteps,
        probed=True,
    )
    logger.info("CAN ID %03X: %s", can_id, caps.describe())
    _log_unprovable_fixes(can_id, caps)
    return caps


async def _probe_one(low_level_api, can_id: int, code: int, timeout: float) -> bool:
    """Sends one read-only probe and reports whether the board answered.

    Args:
        low_level_api: The API to send through.
        can_id: Target CAN ID.
        code: Probe command code.
        timeout: Seconds to wait for the reply.

    Returns:
        True when a well-formed reply arrived.
    """
    try:
        if code == const.CMD_READ_SYSTEM_PARAMETER_PREFIX:
            await low_level_api.read_system_parameter(
                can_id, _PROBE_PARAMETER_CODE, timeout=timeout
            )
        elif code == const.CMD_READ_RAW_ENCODER_ADDITION:
            await low_level_api.read_raw_encoder_value_addition(can_id, timeout=timeout)
        elif code == const.CMD_READ_IO_STATUS:
            await low_level_api.read_io_status(can_id, timeout=timeout)
        else:
            logger.debug("No reader wired for probe %02X; treating as unsupported", code)
            return False
        return True
    except (MKSServoError, ValueError, TypeError, IndexError) as exc:
        # A malformed or absent reply both mean "cannot rely on this command".
        logger.debug("CAN ID %03X: probe %02X unanswered (%s)", can_id, code, exc)
        return False


async def _read_parameter_byte(
    low_level_api, can_id: int, parameter_code: int, timeout: float
) -> Optional[int]:
    """Reads a single-byte system parameter, returning None if unavailable.

    Args:
        low_level_api: The API to read through.
        can_id: Target CAN ID.
        parameter_code: The setter's command code, per manual section 5.9.
        timeout: Seconds to wait for the reply.

    Returns:
        The parameter's first data byte, or None when it could not be read.
    """
    try:
        _, data = await low_level_api.read_system_parameter(
            can_id, parameter_code, timeout=timeout
        )
        return data[0] if data else None
    except (MKSServoError, ValueError, TypeError, IndexError) as exc:
        logger.debug(
            "CAN ID %03X: parameter %02X unreadable (%s)", can_id, parameter_code, exc
        )
        return None


def _log_unprovable_fixes(can_id: int, caps: FirmwareCapabilities) -> None:
    """Warns about behavioural fixes whose presence the probes cannot settle.

    Args:
        can_id: The board being described, for the log line.
        caps: The capabilities just established.
    """
    unconfirmed = [
        f"v{version} ({'; '.join(descriptions)})"
        for version, descriptions in sorted(
            get_firmware_fixes().items(), key=lambda kv: _version_tuple(kv[0])
        )
        if not version.startswith("_") and caps.has_fix(version) is None
    ]
    if unconfirmed:
        # One line per axis, not one per fix: a multi-axis machine would
        # otherwise print the same paragraph on every startup.
        logger.warning(
            "CAN ID %03X: firmware floor is %s, so these fixes can be neither "
            "confirmed nor ruled out (they added no command a read-only probe "
            "could test): %s",
            can_id,
            f"v{caps.minimum_version}+" if caps.minimum_version else "unknown",
            ", ".join(unconfirmed),
        )
