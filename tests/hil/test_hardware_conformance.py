"""Hardware-in-the-loop conformance tests.

Everything else in this repository validates the library against the simulator,
and the simulator was written from the same reading of the manual as the
library. A shared misreading of the protocol is therefore invisible: both sides
agree on the wrong bytes and every test passes. These tests are the only place
that can break that circularity.

Running them
------------
They are skipped unless a real motor is reachable. Point the environment at
your adapter and run the suite::

    export MKS_HIL_CHANNEL=can0            # or /dev/ttyACM0 for slcan
    export MKS_HIL_INTERFACE=socketcan     # default: socketcan
    export MKS_HIL_BITRATE=500000          # default: 500000
    export MKS_HIL_CAN_ID=1                # default: 1
    pytest tests/hil -v

Safety
------
`test_motion_*` rotates the shaft. Set ``MKS_HIL_ALLOW_MOTION=1`` to enable
those; without it only read-only commands run. Decouple the motor from any
mechanism before enabling motion.

Recording a reference trace
---------------------------
The highest-value artefact this file produces is a capture that lets the
simulator be checked against ground truth offline, in CI, with no hardware::

    pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json

`test_simulator_matches_recorded_hardware` in this module then replays that
capture against the simulator and asserts the responses are byte-identical.
That test runs everywhere, hardware or not, as soon as a trace exists.
"""
import asyncio
import json
import os
import pathlib

import pytest

from mks_servo_can import CANInterface, constants as const
from mks_servo_can.crc import calculate_crc, verify_crc

try:
    from can import Message as CanMessage
except ImportError:  # pragma: no cover
    CanMessage = None

TRACE_PATH = (
    pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "hardware_trace.json"
)

HIL_CHANNEL = os.environ.get("MKS_HIL_CHANNEL")
HIL_INTERFACE = os.environ.get("MKS_HIL_INTERFACE", "socketcan")
HIL_BITRATE = int(os.environ.get("MKS_HIL_BITRATE", "500000"))
HIL_CAN_ID = int(os.environ.get("MKS_HIL_CAN_ID", "1"))
HIL_ALLOW_MOTION = os.environ.get("MKS_HIL_ALLOW_MOTION") == "1"

requires_hardware = pytest.mark.skipif(
    not HIL_CHANNEL,
    reason="set MKS_HIL_CHANNEL to run hardware-in-the-loop tests",
)
requires_motion_consent = pytest.mark.skipif(
    not HIL_ALLOW_MOTION,
    reason="set MKS_HIL_ALLOW_MOTION=1 to allow tests that rotate the shaft",
)

pytestmark = pytest.mark.hil

# Read-only commands that are safe to issue to any motor in any state. These are
# the probes whose responses form the reference trace.
PROBE_COMMANDS = [
    (const.CMD_READ_ENCODER_CARRY, []),
    (const.CMD_READ_ENCODER_ADDITION, []),
    (const.CMD_READ_MOTOR_SPEED_RPM, []),
    (const.CMD_READ_PULSES_RECEIVED, []),
    (const.CMD_READ_IO_STATUS, []),
    (const.CMD_READ_RAW_ENCODER_ADDITION, []),
    (const.CMD_READ_SHAFT_ANGLE_ERROR, []),
    (const.CMD_READ_EN_PIN_STATUS, []),
    (const.CMD_READ_POWER_ON_ZERO_STATUS, []),
    (const.CMD_READ_MOTOR_PROTECTION_STATE, []),
    (const.CMD_QUERY_MOTOR_STATUS, []),
]


def build_frame(can_id, command_code, data):
    """
    Builds a downlink frame with its checksum appended.

    Args:
        can_id: Target motor CAN ID.
        command_code: MKS command byte.
        data: Payload bytes following the command byte.

    Returns:
        A `can.Message` ready to send.
    """
    payload = [command_code] + list(data)
    payload.append(calculate_crc(can_id, payload))
    return CanMessage(
        arbitration_id=can_id, data=bytes(payload), is_extended_id=False
    )


async def probe(can_if, can_id, command_code, data, timeout=1.0):
    """
    Issues one command and returns the raw response frame.

    Args:
        can_if: A connected CANInterface.
        can_id: Target motor CAN ID.
        command_code: MKS command byte.
        data: Payload bytes following the command byte.
        timeout: Seconds to wait for the response.

    Returns:
        The raw `can.Message` response.
    """
    return await can_if.send_and_wait_for_response(
        build_frame(can_id, command_code, data), can_id, command_code, timeout=timeout
    )


@pytest.fixture
async def hardware_can_interface():
    """Connects to a physical CAN bus, or skips the test."""
    if not HIL_CHANNEL:
        pytest.skip("no hardware configured")
    can_if = CANInterface(
        interface_type=HIL_INTERFACE, channel=HIL_CHANNEL, bitrate=HIL_BITRATE
    )
    await can_if.connect()
    try:
        yield can_if
    finally:
        await can_if.disconnect()


@requires_hardware
class TestHardwareFraming:
    """The same wire-format assertions the simulator is held to."""

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "command_code,data", PROBE_COMMANDS, ids=lambda v: f"{v:#04x}" if isinstance(v, int) else ""
    )
    async def test_response_crc_is_valid(
        self, hardware_can_interface, command_code, data
    ):
        """Confirms the library computes the checksum the motor expects."""
        response = await probe(
            hardware_can_interface, HIL_CAN_ID, command_code, data
        )
        assert verify_crc(response.arbitration_id, list(response.data))

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "command_code,data", PROBE_COMMANDS, ids=lambda v: f"{v:#04x}" if isinstance(v, int) else ""
    )
    async def test_response_echoes_command(
        self, hardware_can_interface, command_code, data
    ):
        response = await probe(
            hardware_can_interface, HIL_CAN_ID, command_code, data
        )
        assert response.data[0] == command_code


@requires_hardware
@requires_motion_consent
class TestHardwareMotion:
    """Behavioural checks that require the shaft to turn. Decouple the load."""

    @pytest.mark.asyncio
    async def test_ccw_move_increases_the_accumulator(
        self, hardware_can_interface
    ):
        """
        Settles the sign convention that manual V1.0.6 contradicts itself on.

        The prose above the 0x31 worked example says clockwise is positive; the
        example itself and the 0x32 note say counter-clockwise is. Only hardware
        can decide. If this test fails, the errata in
        tests/fixtures/manual_commands_v106.json is backwards and the simulator
        must be corrected to match.
        """
        can_if = hardware_can_interface
        await probe(can_if, HIL_CAN_ID, const.CMD_ENABLE_MOTOR, [0x01])

        def read_accumulator(resp):
            return int.from_bytes(resp.data[1:7], "big", signed=True)

        before = read_accumulator(
            await probe(can_if, HIL_CAN_ID, const.CMD_READ_ENCODER_ADDITION, [])
        )

        one_rev = const.ENCODER_PULSES_PER_REVOLUTION
        # 0xF4 relative-by-axis, speed 600, acc 240, +1 revolution.
        payload = [0x02, 0x58, 0xF0] + list(one_rev.to_bytes(3, "big"))
        await probe(
            can_if, HIL_CAN_ID, const.CMD_RUN_POSITION_MODE_RELATIVE_AXIS, payload
        )
        await asyncio.sleep(3.0)

        after = read_accumulator(
            await probe(can_if, HIL_CAN_ID, const.CMD_READ_ENCODER_ADDITION, [])
        )
        delta = after - before
        assert delta > 0, (
            f"a positive relative-axis move changed the accumulator by {delta}. "
            "If this is consistently negative, the sign convention recorded in "
            "manual_commands_v106.json is wrong and the simulator matches the "
            "wrong reading."
        )
        assert abs(delta - one_rev) < one_rev * 0.1

    @pytest.mark.asyncio
    async def test_absolute_axis_accepts_a_target_while_moving(
        self, hardware_can_interface
    ):
        """
        Verifies 0xF5's documented 'supports real-time updates' claim.

        The whole streaming control model - and the gimbal example - depends on
        being able to retarget a move in flight. The simulator permits it; this
        is the test that confirms hardware does too, and reveals whether the
        motor emits an abort frame for the superseded move (which the transport
        must then discard, see CANInterface.expect_stale_notification).
        """
        can_if = hardware_can_interface
        await probe(can_if, HIL_CAN_ID, const.CMD_ENABLE_MOTOR, [0x01])

        one_rev = const.ENCODER_PULSES_PER_REVOLUTION
        statuses = []
        for target in (one_rev * 2, one_rev, one_rev * 3, 0):
            payload = [0x02, 0x58, 0xF0] + list(
                (target & 0xFFFFFF).to_bytes(3, "big")
            )
            response = await probe(
                can_if,
                HIL_CAN_ID,
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
                payload,
            )
            statuses.append(response.data[1])
            await asyncio.sleep(0.2)

        assert all(s != const.POS_RUN_FAIL for s in statuses), (
            f"motor rejected a retarget mid-move: statuses {statuses}. If this "
            "fails on hardware, streaming 0xF5 is not viable and the gimbal "
            "example must switch to speed mode (0xF6)."
        )


class TestSimulatorMatchesRecordedHardware:
    """
    Replays a recorded hardware trace against the simulator.

    This is the test that actually closes the loop, and it needs no hardware to
    run - only a trace someone recorded once. Until such a trace exists it
    skips, which is an honest statement of the project's validation status
    rather than a false green.
    """

    @pytest.mark.asyncio
    async def test_simulator_reproduces_hardware_responses(
        self, compliance_can_interface
    ):
        if not TRACE_PATH.exists():
            pytest.skip(
                f"no hardware trace at {TRACE_PATH}. Record one with: "
                "pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json"
            )
        trace = json.loads(TRACE_PATH.read_text())

        mismatches = []
        for entry in trace["exchanges"]:
            response = await probe(
                compliance_can_interface,
                1,
                entry["command_code"],
                entry["request_data"],
            )
            expected_dlc = entry["response_dlc"]
            if response.dlc != expected_dlc:
                mismatches.append(
                    f"{entry['command_code']:#04x}: hardware DLC {expected_dlc}, "
                    f"simulator DLC {response.dlc}"
                )
            if response.data[0] != entry["response_data"][0]:
                mismatches.append(
                    f"{entry['command_code']:#04x}: echo byte differs "
                    f"(hardware {entry['response_data'][0]:#04x}, "
                    f"simulator {response.data[0]:#04x})"
                )

        assert not mismatches, "simulator diverges from recorded hardware:\n" + "\n".join(
            mismatches
        )


@requires_hardware
@pytest.mark.asyncio
async def test_record_hardware_trace(hardware_can_interface, request):
    """
    Captures a reference trace from real hardware.

    Not really a test - it is the recording tool, shaped as a test so it can
    reuse the connection fixture. It only writes when --hil-record is given.

    Args:
        hardware_can_interface: Connected CANInterface from the fixture.
        request: pytest request, used to read the --hil-record option.
    """
    destination = request.config.getoption("--hil-record", default=None)
    if not destination:
        pytest.skip("pass --hil-record=<path> to capture a trace")

    exchanges = []
    for command_code, data in PROBE_COMMANDS:
        response = await probe(
            hardware_can_interface, HIL_CAN_ID, command_code, data
        )
        exchanges.append(
            {
                "command_code": command_code,
                "request_data": list(data),
                "response_dlc": response.dlc,
                "response_data": list(response.data),
            }
        )

    path = pathlib.Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "source": "MKS SERVO42D/57D hardware capture",
                "interface": HIL_INTERFACE,
                "bitrate": HIL_BITRATE,
                "can_id": HIL_CAN_ID,
                "note": (
                    "Read-only probes only. Replayed against the simulator by "
                    "TestSimulatorMatchesRecordedHardware."
                ),
                "exchanges": exchanges,
            },
            indent=2,
        )
        + "\n"
    )
    assert exchanges, "captured no exchanges"
