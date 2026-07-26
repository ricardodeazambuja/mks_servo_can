"""Firmware capability detection.

The boards have no version command, so the library probes which commands they
answer and turns that into a lower bound on the firmware release. These tests
pin the parts that are easy to get wrong: that probing never writes, that an
unestablished version reports "unknown" rather than "absent", and that an
unknown work mode makes move timeouts more generous rather than less.
"""
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from mks_servo_can import constants as const
from mks_servo_can.axis import Axis
from mks_servo_can.can_interface import CANInterface
from mks_servo_can.exceptions import CommunicationError
from mks_servo_can.firmware import FirmwareCapabilities, probe_firmware
from mks_servo_can.kinematics import RotaryKinematics
from mks_servo_can.low_level_api import LowLevelAPI
from mks_servo_can.manual_spec import (
    get_firmware_fixes,
    get_firmware_history,
    get_firmware_probes,
    get_manual_commands,
)

# Every command that mutates state. Probing with any of these is unsafe: a
# short frame can be accepted and written, which is how a bare 0x84 silently
# corrupted a motor's subdivision during hardware bring-up.
WRITE_CATEGORIES = {"configuration", "motion", "control", "homing"}


@pytest.fixture
def api():
    """A LowLevelAPI mock whose probe commands all fail, i.e. an old board."""
    mock = AsyncMock(spec=LowLevelAPI)
    for name in ("read_system_parameter", "read_raw_encoder_value_addition",
                 "read_io_status"):
        setattr(mock, name, AsyncMock(side_effect=CommunicationError("silent")))
    return mock


def test_probes_are_read_only():
    """Nothing in the probe set may be a command that writes.

    This is the invariant that keeps detection from damaging a motor, so it is
    checked against the manual's own categorisation rather than a hand list.
    """
    commands = get_manual_commands()
    offenders = []
    for code in get_firmware_probes():
        if code.startswith("_"):
            continue
        spec = commands.get(code) or commands.get(code.upper())
        assert spec is not None, f"probe {code} is not in the manual spec"
        if spec.get("category") in WRITE_CATEGORIES:
            offenders.append(f"{code} ({spec.get('name')}, {spec.get('category')})")
    assert not offenders, "probe set contains state-changing commands: " + ", ".join(offenders)


def test_probe_versions_agree_with_the_firmware_history():
    """Each probe must claim the release the manual says introduced it."""
    history = get_firmware_history()
    for code, version in get_firmware_probes().items():
        if code.startswith("_"):
            continue
        assert code in history.get(version, []), (
            f"probe {code} claims v{version}, but firmware_history puts it in "
            f"{[v for v, codes in history.items() if code in codes] or 'no release'}"
        )


@pytest.mark.asyncio
async def test_silent_board_reports_no_version_floor(api):
    """A board answering nothing must not be credited with any release."""
    caps = await probe_firmware(api, can_id=1, timeout=0.01)

    assert caps.probed
    assert caps.minimum_version is None
    assert not caps.can_read_system_parameters
    assert not caps.can_read_raw_encoder
    assert caps.work_mode is None


@pytest.mark.asyncio
async def test_v106_board_is_detected_and_its_work_mode_read(api):
    """Answering 0x00 establishes V1.0.6 and yields the work mode."""
    api.read_system_parameter = AsyncMock(
        side_effect=lambda can_id, code, timeout=None: (
            (code, bytes([const.MODE_SR_OPEN])) if code == const.CMD_SET_WORK_MODE
            else (code, bytes([16]))
        )
    )
    api.read_raw_encoder_value_addition = AsyncMock(return_value=0)
    api.read_io_status = AsyncMock(return_value={"raw_byte": 0})

    caps = await probe_firmware(api, can_id=1, timeout=0.01)

    assert caps.minimum_version == "1.0.6"
    assert caps.can_read_system_parameters
    assert caps.work_mode == const.MODE_SR_OPEN
    assert caps.microsteps == 16


@pytest.mark.asyncio
async def test_old_board_still_credited_for_the_commands_it_does_answer(api):
    """A board answering only 0x34 is V1.0.3+, not V1.0.6."""
    api.read_io_status = AsyncMock(return_value={"raw_byte": 0})

    caps = await probe_firmware(api, can_id=1, timeout=0.01)

    assert caps.minimum_version == "1.0.3"
    assert not caps.can_read_system_parameters


def test_unprovable_fix_reports_unknown_not_absent():
    """The F4/F5 fix cannot be ruled out below its version floor.

    V1.0.5 added the fix but no probeable command, so a board that fails the
    V1.0.6 probes might still have it. Reporting False here would be a guess,
    and it is the guess that produced a wrong claim during bring-up.
    """
    assert "1.0.5" in get_firmware_fixes(), "the F4/F5 fix must be recorded"

    old = FirmwareCapabilities(minimum_version="1.0.3", probed=True)
    assert old.has_fix("1.0.5") is None, "must be unknown, never a definite answer"

    new = FirmwareCapabilities(minimum_version="1.0.6", probed=True)
    assert new.has_fix("1.0.5") is True


def test_unprobed_capabilities_claim_nothing():
    """A default instance must not imply support for anything."""
    caps = FirmwareCapabilities()
    assert not caps.probed
    assert caps.minimum_version is None
    assert not caps.can_read_system_parameters
    assert caps.has_fix("1.0.5") is None
    assert caps.supports(const.CMD_READ_IO_STATUS) is False


# --- how detection feeds the move timeout -------------------------------


@pytest.fixture
def axis():
    """An Axis on mocks, enabled, with nothing detected yet."""
    can_if = AsyncMock(spec=CANInterface)
    can_if.create_response_future = MagicMock(
        side_effect=lambda *a, **k: asyncio.get_event_loop().create_future()
    )
    can_if.expect_stale_notification = MagicMock()
    with patch("mks_servo_can.axis.LowLevelAPI", return_value=AsyncMock(spec=LowLevelAPI)):
        ax = Axis(
            can_interface_manager=can_if, motor_can_id=1, name="FwAxis",
            kinematics=RotaryKinematics(
                steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION),
        )
    ax._is_enabled = True
    return ax


def test_unknown_work_mode_is_more_generous_than_any_known_one(axis):
    """An undetected work mode must never shorten a timeout.

    The ceiling only ever caps the speed estimate, so guessing high (the old
    vFOC default) shortens the budget and fails moves that a 400 RPM open-mode
    board completes normally. Unknown must therefore behave like the slowest
    mode, not the fastest.
    """
    distance = const.ENCODER_PULSES_PER_REVOLUTION * 200
    speed_param = 1000

    def timeout_for(mode):
        axis._work_mode = mode
        return axis._calculate_move_timeout(
            const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, distance, speed_param, 0
        )

    unknown = timeout_for(None)
    known = [timeout_for(mode) for mode in const.MAX_RPM_BY_WORK_MODE]

    assert unknown >= max(known), (
        f"unknown work mode gave {unknown:.2f}s, less than the "
        f"{max(known):.2f}s of the most conservative known mode"
    )


@pytest.mark.asyncio
async def test_initialize_adopts_a_readable_work_mode(axis):
    """A board that reports its work mode must stop being guessed at."""
    axis._low_level_api.read_encoder_value_addition = AsyncMock(return_value=0)
    axis._low_level_api.read_en_pin_status = AsyncMock(return_value=True)

    detected = FirmwareCapabilities(
        supported_commands=[const.CMD_READ_SYSTEM_PARAMETER_PREFIX],
        minimum_version="1.0.6", work_mode=const.MODE_SR_CLOSE, probed=True,
    )
    with patch("mks_servo_can.axis.probe_firmware", AsyncMock(return_value=detected)):
        await axis.initialize(calibrate=False, home=False)

    assert axis._work_mode == const.MODE_SR_CLOSE
    assert axis.firmware.minimum_version == "1.0.6"


@pytest.mark.asyncio
async def test_detection_can_be_skipped(axis):
    """detect_firmware=False must issue no probes and leave the mode unknown."""
    axis._low_level_api.read_encoder_value_addition = AsyncMock(return_value=0)
    axis._low_level_api.read_en_pin_status = AsyncMock(return_value=True)

    with patch("mks_servo_can.axis.probe_firmware", AsyncMock()) as probe:
        await axis.initialize(calibrate=False, home=False, detect_firmware=False)

    probe.assert_not_called()
    assert axis._work_mode is None
    assert not axis.firmware.probed
