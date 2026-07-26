"""Wire-level fidelity tests: does the simulator behave like a real motor?

The rest of the compliance suite exercises the library's Python API and mostly
asserts that return values have plausible types. That cannot catch a simulator
that speaks a subtly different protocol from the hardware, because the library
and the simulator would be wrong together.

These tests instead assert on the raw CAN frames: the exact DLC, the CRC over
the exact bytes, the command echo, the integer widths and byte order, and the
sign conventions. The reference is mks_servo_can/data/manual_commands_v106.json,
which encodes the frame layouts from the MKS SERVO42D/57D_CAN user manual.

What these tests can and cannot prove
-------------------------------------
They prove the simulator is self-consistent with the documented protocol. They
do NOT prove it matches physical hardware: both the simulator and this fixture
were derived from the same manual, so a shared misreading stays invisible. The
only cure is a captured candump from a real motor, which belongs in tests/hil/.
Nothing here should be read as hardware validation.
"""

import pytest

from mks_servo_can import constants as const
from mks_servo_can import load_manual_spec
from mks_servo_can.crc import calculate_crc, verify_crc

from .test_protocol_compliance import command_calls

try:
    from can import Message as CanMessage
except ImportError:  # pragma: no cover - python-can is a hard dependency in CI
    CanMessage = None

# Read from the library package, which ships the manual's transcription as
# package data. Reading it out of tests/ meant an installed wheel had no copy.
FIXTURE = load_manual_spec()
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
        # 0x35 was missing from this list, so nothing checked its framing. The
        # transcription had its response at DLC 4 carrying a uint16 where the
        # manual gives DLC 8 carrying an int48.
        (const.CMD_READ_RAW_ENCODER_ADDITION, None),
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
    async def test_raw_encoder_addition_is_int48(self, compliance_can_interface):
        """
        0x35: the same layout as 0x31, over the encoder's uncorrected count.

        The transcription had this at DLC 4 carrying a uint16, which the entry's
        own notes contradicted - a value that moves by 0x4000 a revolution is an
        accumulator, not a raw single-turn reading.
        """
        response = await exchange(
            compliance_can_interface, 1, const.CMD_READ_RAW_ENCODER_ADDITION
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


class TestRequestFraming:
    """
    The frames the *library* emits, checked against the manual.

    Everything above examines what the simulator answers. Nothing examined what
    went out, and for thirty-one of the forty-nine commands nothing could: they
    were absent from the transcription, so there was no layout to check them
    against. A command whose request frame is the wrong length is accepted by
    the simulator - it reads the payload it expects and ignores the rest - and
    rejected by hardware.
    """

    @pytest.mark.asyncio
    async def test_every_command_goes_out_with_the_manuals_layout(
        self, compliance_api, compliance_can_interface
    ):
        """Every transcribed command, driven through the library, one frame at a time."""
        sent = []
        received = []
        original_send = compliance_can_interface.send_message
        original_receive = compliance_can_interface._process_received_message

        async def recording_send(msg, *args, **kwargs):
            """Records the outgoing frame, then sends it unchanged."""
            sent.append(msg)
            return await original_send(msg, *args, **kwargs)

        async def recording_receive(msg, *args, **kwargs):
            """Records the incoming frame, then hands it on untouched."""
            received.append(msg)
            return await original_receive(msg, *args, **kwargs)

        compliance_can_interface.send_message = recording_send
        compliance_can_interface._process_received_message = recording_receive
        wrong = []
        try:
            for code, call in command_calls(compliance_api).items():
                sent.clear()
                received.clear()
                try:
                    await call()
                except Exception as exc:  # framing is the subject, not the outcome
                    if not sent:
                        wrong.append(f"{code}: nothing was sent ({exc})")
                        continue
                if not sent:
                    wrong.append(f"{code}: nothing was sent")
                    continue

                expected_dlc = MANUAL_COMMANDS[code]["request"]["dlc"]
                frame = sent[0]
                if frame.data[0] != int(code, 16):
                    wrong.append(
                        f"{code}: frame carries command byte "
                        f"0x{frame.data[0]:02X}"
                    )
                elif len(frame.data) != expected_dlc:
                    wrong.append(
                        f"{code}: manual says DLC {expected_dlc}, library sent "
                        f"{len(frame.data)} ({frame.data.hex()})"
                    )
                elif not verify_crc(frame.arbitration_id, list(frame.data)):
                    wrong.append(f"{code}: bad CRC on {frame.data.hex()}")

                wrong.extend(self._response_complaints(code, received))
        finally:
            compliance_can_interface.send_message = original_send
            compliance_can_interface._process_received_message = original_receive

        assert not wrong, "frames disagree with the manual:\n" + "\n".join(wrong)

    @staticmethod
    def _response_complaints(code, received):
        """
        Checks the replies to one command against the manual's uplink layout.

        Args:
            code: The command's `"0xNN"` key in the transcription.
            received: Every frame that arrived while the command was in flight.

        Returns:
            A list of complaints, empty when the replies are well formed.
        """
        response_spec = MANUAL_COMMANDS[code]["response"]
        if response_spec.get("variable_dlc"):
            # 0x00's reply echoes the parameter's code and is as long as that
            # parameter needs, so there is no fixed layout to check.
            return []

        echoes = [f for f in received if f.data and f.data[0] == int(code, 16)]
        if not echoes:
            return [f"{code}: the motor sent no frame echoing the command"]

        complaints = []
        for frame in echoes:
            if len(frame.data) != response_spec["dlc"]:
                complaints.append(
                    f"{code}: manual says response DLC {response_spec['dlc']}, "
                    f"motor sent {len(frame.data)} ({frame.data.hex()})"
                )
            if not verify_crc(frame.arbitration_id, list(frame.data)):
                complaints.append(f"{code}: bad CRC on reply {frame.data.hex()}")
        return complaints


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
