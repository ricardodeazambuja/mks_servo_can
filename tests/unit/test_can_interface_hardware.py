"""
The hardware branch of `CANInterface`, which no test had touched.

`can_interface.py` sat at 56%, and the untested half was the half that talks to
a real bus: `can.interface.Bus`, `can.Notifier`, `AsyncioCanListener`, and the
send and receive paths that go through them. "Needs hardware" was the reason,
and it is not true - `python-can` ships a `virtual` interface, an in-process bus
that two `CANInterface` instances can join and exchange real `can.Message`
frames over. Everything below the driver is exercised for real; only the driver
is different.

That leaves genuinely nothing here to mock. The one exception is the
"`python-can` is not installed" path, which is a module-level flag.
"""
import asyncio

import pytest

from mks_servo_can import CANInterface, exceptions
from mks_servo_can import can_interface as can_interface_module

can = pytest.importorskip("can")

# Two interfaces on the same virtual channel see each other's traffic.
VIRTUAL_CHANNEL = "mks-tests"


async def _connected_pair(channel: str):
    """
    Builds two connected interfaces on one virtual bus.

    Args:
        channel: The virtual channel name; distinct per test so that frames from
            one test cannot reach another.

    Returns:
        A `(sender, receiver)` pair, both connected.
    """
    sender = CANInterface(interface_type="virtual", channel=channel)
    receiver = CANInterface(interface_type="virtual", channel=channel)
    await sender.connect()
    await receiver.connect()
    return sender, receiver


def _frame(can_id: int, payload: bytes) -> "can.Message":
    """Builds a standard-frame CAN message."""
    return can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)


class TestConstruction:
    """What is decided before anything is opened."""

    def test_hardware_settings_are_kept_as_given(self):
        """A bitrate quietly replaced by a default is a bus that never talks."""
        interface = CANInterface(
            interface_type="socketcan", channel="can0", bitrate=1_000_000
        )

        assert interface.interface_type == "socketcan"
        assert interface.channel == "can0"
        assert interface.bitrate == 1_000_000
        assert interface.use_simulator is False

    def test_the_simulator_settings_are_kept_as_given(self):
        """The simulator branch keeps its own three."""
        interface = CANInterface(
            use_simulator=True, simulator_host="10.0.0.4", simulator_port=7000
        )

        assert interface.use_simulator is True
        assert interface.simulator_host == "10.0.0.4"
        assert interface.simulator_port == 7000

    def test_hardware_without_python_can_is_refused_at_construction(self, monkeypatch):
        """
        The one path with nothing real behind it.

        `python-can` is a hard dependency of the library, so it is always
        installed here; the flag it sets is what the guard reads.
        """
        monkeypatch.setattr(can_interface_module, "CAN_AVAILABLE", False)

        with pytest.raises(exceptions.ConfigurationError, match="python-can"):
            CANInterface(interface_type="socketcan", channel="can0")

    def test_the_simulator_needs_no_python_can(self, monkeypatch):
        """The simulator path must stay usable without the hardware dependency."""
        monkeypatch.setattr(can_interface_module, "CAN_AVAILABLE", False)

        interface = CANInterface(use_simulator=True)

        assert interface.use_simulator is True

    def test_an_interface_can_be_built_with_no_loop_in_the_thread(self):
        """
        Constructing must not require an event loop to already exist.

        The constructor called `asyncio.get_event_loop()`, which raises
        `RuntimeError: There is no current event loop` the moment anything in
        the thread has called `set_event_loop(None)` — which every asyncio test
        framework does at teardown, and which any program that finishes one
        `asyncio.run()` before building an interface for the next hits too.
        Whether construction worked therefore depended on what had run before
        it; the failure surfaced only when the suite was run against an
        installed wheel, where the order differed.
        """
        asyncio.set_event_loop(None)
        try:
            simulator = CANInterface(use_simulator=True)
            hardware = CANInterface(interface_type="virtual", channel="later")

            assert simulator.use_simulator is True
            assert hardware.channel == "later"
        finally:
            asyncio.set_event_loop(asyncio.new_event_loop())

    @pytest.mark.asyncio
    async def test_an_explicit_loop_is_used_when_one_is_given(self):
        """A caller who passes a loop must get that loop, not the running one."""
        given = asyncio.new_event_loop()
        try:
            interface = CANInterface(use_simulator=True, loop=given)
            assert interface._loop is given
        finally:
            given.close()


class TestConnectionErrors:
    """Failures that happen on the way to a bus."""

    @pytest.mark.asyncio
    async def test_a_missing_channel_is_refused(self):
        """Every interface but `virtual` needs to be told which one."""
        interface = CANInterface(interface_type="socketcan", channel=None)

        with pytest.raises(exceptions.ConfigurationError, match="Channel"):
            await interface.connect()

    @pytest.mark.asyncio
    async def test_the_virtual_interface_needs_no_channel(self):
        """It is the documented exception to the rule above."""
        interface = CANInterface(interface_type="virtual", channel=None)

        await interface.connect()
        try:
            assert interface.is_connected
        finally:
            await interface.disconnect()

    @pytest.mark.asyncio
    async def test_an_unknown_interface_type_raises_a_can_error(self):
        """
        `python-can` raises on an interface it does not know.

        Whatever it raises has to come back as this library's `CANError`, not as
        a driver exception the caller has no reason to expect.
        """
        interface = CANInterface(
            interface_type="definitely-not-a-real-interface", channel="x"
        )

        with pytest.raises(exceptions.CANError):
            await interface.connect()

    @pytest.mark.asyncio
    async def test_sending_before_connecting_raises(self):
        """A frame that goes nowhere must not be reported as sent."""
        interface = CANInterface(interface_type="virtual", channel=VIRTUAL_CHANNEL)

        with pytest.raises(exceptions.CANError, match="not connected"):
            await interface.send_message(_frame(1, b"\x30"))

    @pytest.mark.asyncio
    async def test_a_refused_simulator_connection_says_so(self):
        """Port 1 is privileged and unbound; nothing will be listening."""
        interface = CANInterface(
            use_simulator=True, simulator_host="127.0.0.1", simulator_port=1
        )

        with pytest.raises(exceptions.SimulatorError):
            await interface.connect()


class TestTrafficOverARealBus:
    """
    Frames on a `python-can` bus, through the notifier and the listener.

    This is the path the library takes on real hardware: `bus.send`, the
    `Notifier` thread, `AsyncioCanListener.on_message_received` handing the
    frame to the event loop, and the listening task dispatching it.
    """

    @pytest.mark.asyncio
    async def test_a_frame_reaches_a_registered_handler(self):
        """The whole receive chain, end to end."""
        sender, receiver = await _connected_pair("mks-tests-handler")
        seen = []
        try:
            receiver.add_message_handler(0x01, seen.append)
            await sender.send_message(_frame(0x01, b"\x30\x31"))
            await asyncio.sleep(0.3)
        finally:
            await sender.disconnect()
            await receiver.disconnect()

        assert [message.data.hex() for message in seen] == ["3031"]

    @pytest.mark.asyncio
    async def test_a_removed_handler_stops_receiving(self):
        """Removal has to actually detach it, or a stale callback keeps firing."""
        sender, receiver = await _connected_pair("mks-tests-removal")
        seen = []
        try:
            receiver.add_message_handler(0x02, seen.append)
            await sender.send_message(_frame(0x02, b"\x30"))
            await asyncio.sleep(0.3)
            assert len(seen) == 1

            receiver.remove_message_handler(0x02, seen.append)
            await sender.send_message(_frame(0x02, b"\x31"))
            await asyncio.sleep(0.3)
        finally:
            await sender.disconnect()
            await receiver.disconnect()

        assert len(seen) == 1, "the handler kept receiving after being removed"

    @pytest.mark.asyncio
    async def test_a_handler_registered_for_another_id_is_not_called(self):
        """Dispatch is per CAN ID; a broadcast to every handler would be worse than none."""
        sender, receiver = await _connected_pair("mks-tests-routing")
        wrong_id = []
        right_id = []
        try:
            receiver.add_message_handler(0x05, wrong_id.append)
            receiver.add_message_handler(0x06, right_id.append)
            await sender.send_message(_frame(0x06, b"\x30"))
            await asyncio.sleep(0.3)
        finally:
            await sender.disconnect()
            await receiver.disconnect()

        assert len(right_id) == 1
        assert wrong_id == []

    @pytest.mark.asyncio
    async def test_a_reply_resolves_the_waiting_request(self):
        """
        `send_and_wait_for_response` over a real bus.

        The responder is a handler on the other interface that answers with the
        echoed command byte, exactly as a motor does.
        """
        requester, responder = await _connected_pair("mks-tests-reply")
        loop = asyncio.get_running_loop()
        # Held so the reply task cannot be garbage-collected mid-send.
        replies = []

        def answer(message):
            """Replies to any frame with the command byte echoed back."""
            replies.append(
                loop.create_task(
                    responder.send_message(
                        _frame(0x07, bytes([message.data[0], 0x01, 0x00]))
                    )
                )
            )

        try:
            responder.add_message_handler(0x07, answer)
            reply = await requester.send_and_wait_for_response(
                _frame(0x07, b"\x30\x31"), 0x07, 0x30, timeout=2.0
            )
        finally:
            await requester.disconnect()
            await responder.disconnect()

        assert reply.data[0] == 0x30
        assert reply.data[1] == 0x01

    @pytest.mark.asyncio
    async def test_an_unanswered_request_times_out_rather_than_hanging(self):
        """Nothing is listening on this ID, and the wait has to end by itself."""
        interface = CANInterface(interface_type="virtual", channel="mks-tests-silent")
        await interface.connect()
        try:
            with pytest.raises(exceptions.CommunicationError, match="Timeout"):
                await interface.send_and_wait_for_response(
                    _frame(0x09, b"\x30"), 0x09, 0x30, timeout=0.3
                )
        finally:
            await interface.disconnect()


class TestConnectionLifecycle:
    """Connecting, reconnecting and shutting down."""

    @pytest.mark.asyncio
    async def test_connecting_twice_is_harmless(self):
        """The second call must not replace a working bus with another one."""
        interface = CANInterface(interface_type="virtual", channel="mks-tests-twice")
        await interface.connect()
        try:
            bus = interface.bus
            await interface.connect()
            assert interface.bus is bus
            assert interface.is_connected
        finally:
            await interface.disconnect()

    @pytest.mark.asyncio
    async def test_disconnecting_stops_the_listener(self):
        """A listener left running holds the loop open after the bus is gone."""
        interface = CANInterface(interface_type="virtual", channel="mks-tests-stop")
        await interface.connect()
        assert interface.is_connected

        await interface.disconnect()

        assert not interface.is_connected
        assert interface._listener_task is None

    @pytest.mark.asyncio
    async def test_disconnecting_twice_is_harmless(self):
        """Teardown paths call this more than once; it must not raise."""
        interface = CANInterface(interface_type="virtual", channel="mks-tests-twice-off")
        await interface.connect()

        await interface.disconnect()
        await interface.disconnect()

        assert not interface.is_connected
