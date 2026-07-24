"""Wire-level fidelity tests: does the simulator behave like a real motor?

The rest of the compliance suite exercises the library's Python API and mostly
asserts that return values have plausible types. That cannot catch a simulator
that speaks a subtly different protocol from the hardware, because the library
and the simulator would be wrong together.

These tests instead assert on the raw CAN frames: the exact DLC, the CRC over
the exact bytes, the command echo, the integer widths and byte order, and the
sign conventions. The reference is tests/fixtures/manual_commands_v106.json,
which encodes the frame layouts from the MKS SERVO42D/57D_CAN user manual.

What these tests can and cannot prove
-------------------------------------
They prove the simulator is self-consistent with the documented protocol. They
do NOT prove it matches physical hardware: both the simulator and this fixture
were derived from the same manual, so a shared misreading stays invisible. The
only cure is a captured candump from a real motor, which belongs in tests/hil/.
Nothing here should be read as hardware validation.
"""
import json
import pathlib

import pytest

from mks_servo_can import constants as const
from mks_servo_can.crc import calculate_crc, verify_crc

try:
    from can import Message as CanMessage
except ImportError:  # pragma: no cover - python-can is a hard dependency in CI
    CanMessage = None

FIXTURE = json.loads(
    (
        pathlib.Path(__file__).resolve().parents[1]
        / "fixtures"
        / "manual_commands_v106.json"
    ).read_text()
)
MANUAL_COMMANDS = FIXTURE["commands"]

pytestmark = pytest.mark.compliance


async def exchange(can_if, can_id, command_code, data=None, timeout=2.0):
    """
    Sends one raw command frame and returns the raw response frame.

    Deliberately bypasses LowLevelAPI so that the response bytes are examined
    exactly as they came off the wire, rather than after the library has parsed
    and possibly normalised them.

    Args:
        can_if: A connected CANInterface.
        can_id: Target motor CAN ID.
        command_code: MKS command byte.
        data: Optional payload bytes following the command byte.
        timeout: Seconds to wait for the response.

    Returns:
        The raw `can.Message` response.
    """
    payload = [command_code] + list(data or [])
    payload.append(calculate_crc(can_id, payload))
    msg = CanMessage(
        arbitration_id=can_id, data=bytes(payload), is_extended_id=False
    )
    return await can_if.send_and_wait_for_response(
        msg, can_id, command_code, timeout=timeout
    )


def decode_signed_be(raw: bytes) -> int:
    """
    Decodes a big-endian two's-complement integer of arbitrary width.

    The MKS protocol uses 16-, 24- and 48-bit signed fields, none of which
    struct handles directly.

    Args:
        raw: The bytes of the field, most significant first.

    Returns:
        The signed value.
    """
    return int.from_bytes(raw, byteorder="big", signed=True)


class TestResponseFraming:
    """Every response must be a well-formed frame per the manual."""

    # (command code, payload) pairs that need no motor state to issue.
    READ_COMMANDS = [
        (const.CMD_READ_ENCODER_CARRY, None),
        (const.CMD_READ_ENCODER_ADDITION, None),
        (const.CMD_READ_MOTOR_SPEED_RPM, None),
        (const.CMD_READ_PULSES_RECEIVED, None),
        (const.CMD_READ_IO_STATUS, None),
        (const.CMD_READ_EN_PIN_STATUS, None),
        (const.CMD_READ_SHAFT_ANGLE_ERROR, None),
        (const.CMD_READ_POWER_ON_ZERO_STATUS, None),
        (const.CMD_READ_MOTOR_PROTECTION_STATE, None),
        (const.CMD_QUERY_MOTOR_STATUS, None),
    ]

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "command_code,data", READ_COMMANDS, ids=lambda v: f"{v:#04x}" if isinstance(v, int) else ""
    )
    async def test_response_crc_is_valid(
        self, compliance_can_interface, command_code, data
    ):
        """CRC = (ID + all preceding bytes) & 0xFF over the real response."""
        response = await exchange(compliance_can_interface, 1, command_code, data)
        assert verify_crc(response.arbitration_id, list(response.data)), (
            f"bad CRC on response to {command_code:#04x}: {response.data.hex()}"
        )

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "command_code,data", READ_COMMANDS, ids=lambda v: f"{v:#04x}" if isinstance(v, int) else ""
    )
    async def test_response_echoes_command_byte(
        self, compliance_can_interface, command_code, data
    ):
        """byte0 of every uplink frame is the command being answered."""
        response = await exchange(compliance_can_interface, 1, command_code, data)
        assert response.data[0] == command_code

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "command_code,data", READ_COMMANDS, ids=lambda v: f"{v:#04x}" if isinstance(v, int) else ""
    )
    async def test_response_dlc_matches_the_manual(
        self, compliance_can_interface, command_code, data
    ):
        """A wrong DLC is invisible to the library but fatal on real hardware."""
        spec = MANUAL_COMMANDS.get(f"{command_code:#04X}".replace("0X", "0x"))
        if spec is None or "dlc" not in spec.get("response", {}):
            pytest.skip(f"no documented DLC for {command_code:#04x}")
        response = await exchange(compliance_can_interface, 1, command_code, data)
        assert response.dlc == spec["response"]["dlc"], (
            f"{command_code:#04x}: manual says DLC {spec['response']['dlc']}, "
            f"simulator sent {response.dlc} ({response.data.hex()})"
        )

    @pytest.mark.asyncio
    async def test_responses_never_exceed_eight_bytes(
        self, compliance_can_interface
    ):
        """Classic CAN 2.0A caps the payload at 8 bytes."""
        for command_code, data in self.READ_COMMANDS:
            response = await exchange(
                compliance_can_interface, 1, command_code, data
            )
            assert 0 < response.dlc <= 8, (
                f"{command_code:#04x} returned DLC {response.dlc}"
            )
            assert len(response.data) == response.dlc


class TestFieldEncoding:
    """Integer widths, byte order and ranges of the data-bearing responses."""

    @pytest.mark.asyncio
    async def test_encoder_carry_layout(self, compliance_can_interface):
        """0x30: carry(int32) then value(uint16), big-endian, DLC 8."""
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_ENCODER_CARRY
        )
        assert response.dlc == 8
        value = int.from_bytes(response.data[5:7], "big", signed=False)
        assert 0 <= value <= 0x3FFF, (
            f"manual: encoder value field is 0..0x3FFF, got {value:#06x}"
        )

    @pytest.mark.asyncio
    async def test_encoder_addition_is_int48(self, compliance_can_interface):
        """0x31: a single signed 48-bit accumulator, big-endian, DLC 8."""
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_ENCODER_ADDITION
        )
        assert response.dlc == 8
        value = decode_signed_be(response.data[1:7])
        assert -(2**47) <= value < 2**47

    @pytest.mark.asyncio
    async def test_speed_is_int16(self, compliance_can_interface):
        """0x32: signed 16-bit RPM, big-endian, DLC 4."""
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_MOTOR_SPEED_RPM
        )
        assert response.dlc == 4
        speed = decode_signed_be(response.data[1:3])
        assert -32768 <= speed <= 32767

    @pytest.mark.asyncio
    async def test_enable_status_is_boolean_byte(self, compliance_can_interface):
        """0x3A: a single status byte, DLC 3."""
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_EN_PIN_STATUS
        )
        assert response.dlc == 3
        assert response.data[1] in (0, 1)

    @pytest.mark.asyncio
    async def test_motor_status_is_a_documented_code(
        self, compliance_can_interface
    ):
        """0xF1 must return one of the seven documented states."""
        response = await exchange(
            compliance_can_interface, 1, const.CMD_QUERY_MOTOR_STATUS
        )
        assert response.data[1] in const.MOTOR_STATUS_MAP, (
            f"undocumented motor status {response.data[1]:#04x}"
        )


class TestSignConventions:
    """
    Counter-clockwise is positive, for both position and speed.

    Manual V1.0.6 is self-contradictory on this: the prose above the 0x31 and
    0x35 worked examples says clockwise is positive, but the examples themselves
    ('current value 0x3FF0, after one turn CCW the value(+0x4000) is 0x7FF0')
    and the note on 0x32 ('if it run CCW, the speed > 0') both say the opposite.
    The examples win. The fixture used to encode the wrong version.
    """

    @pytest.mark.asyncio
    async def test_fixture_records_ccw_as_positive(self):
        """Guard the fixture itself - it is the reference for HIL testing."""
        for code in ("0x31", "0x35"):
            notes = " ".join(MANUAL_COMMANDS[code]["notes"]).lower()
            assert "after one turn ccw, the value += 0x4000".lower() in notes
        speed_notes = " ".join(MANUAL_COMMANDS["0x32"]["notes"]).lower()
        assert "counter-clockwise" in speed_notes
        assert "positive values indicate counter-clockwise" in speed_notes

    @pytest.mark.asyncio
    async def test_ccw_move_increases_the_accumulator(
        self, compliance_can_interface
    ):
        """A CCW revolution must add 0x4000 counts, not subtract them."""
        can_if = compliance_can_interface
        await exchange(can_if, 1, const.CMD_ENABLE_MOTOR, [0x01])
        before = decode_signed_be(
            (await exchange(can_if, 1, const.CMD_READ_ENCODER_ADDITION)).data[1:7]
        )

        # 0xF4: relative move by axis. Positive delta is CCW.
        one_rev = const.ENCODER_PULSES_PER_REVOLUTION
        payload = [0x02, 0x58, 0xF0] + list(one_rev.to_bytes(3, "big"))
        await exchange(can_if, 1, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, payload)

        import asyncio

        for _ in range(60):
            await asyncio.sleep(0.05)
            after = decode_signed_be(
                (
                    await exchange(can_if, 1, const.CMD_READ_ENCODER_ADDITION)
                ).data[1:7]
            )
            if abs(after - before) > one_rev * 0.9:
                break

        delta = after - before
        assert delta > 0, (
            f"a CCW revolution changed the accumulator by {delta}; the manual's "
            "worked example requires +0x4000 for CCW"
        )
        assert abs(delta - one_rev) < one_rev * 0.2, (
            f"expected roughly +{one_rev} counts for one revolution, got {delta}"
        )


class TestErrorHandling:
    """The simulator must reject malformed input the way hardware would."""

    @pytest.mark.asyncio
    async def test_bad_crc_is_not_answered(self, compliance_can_interface):
        """A frame with a corrupt checksum must be dropped, not acted on."""
        import asyncio

        payload = [const.CMD_READ_EN_PIN_STATUS, 0x00]  # deliberately wrong CRC
        msg = CanMessage(
            arbitration_id=1, data=bytes(payload), is_extended_id=False
        )
        with pytest.raises((asyncio.TimeoutError, Exception)):
            await asyncio.wait_for(
                compliance_can_interface.send_and_wait_for_response(
                    msg, 1, const.CMD_READ_EN_PIN_STATUS, timeout=0.5
                ),
                timeout=1.0,
            )

    @pytest.mark.asyncio
    async def test_unknown_command_does_not_wedge_the_bus(
        self, compliance_can_interface
    ):
        """An unrecognised command must leave the motor answering normally."""
        import asyncio

        try:
            await exchange(compliance_can_interface, 1, 0x7A, timeout=0.5)
        except (asyncio.TimeoutError, Exception):
            pass
        # The next legitimate command must still work.
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_EN_PIN_STATUS
        )
        assert response.data[0] == const.CMD_READ_EN_PIN_STATUS
