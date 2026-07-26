"""
The receive path must survive one bad frame (defect L4).

An exception raised while processing a single message escaped the per-message
`try` in both listeners and was caught by the outer `except Exception`, which
ends the task. One malformed frame silently stopped all reception for the rest
of the session: every later command then failed with a timeout, far from the
cause.

This is reachable, not theoretical. A response predicate that reads `data[1]` -
`Axis.home_axis` registers one - raises `IndexError` on a one-byte frame.

These tests drive the real listeners over a real `asyncio.StreamReader` and a
real `asyncio.Queue`, with nothing stubbed: a poisoned frame goes in, and a
perfectly ordinary one has to still be delivered afterwards.
"""
import asyncio

import pytest

from mks_servo_can import CANInterface
from mks_servo_can import constants as const
from mks_servo_can.low_level_api import CanMessage

CAN_ID = 0x001

# A predicate of the shape the library actually registers. On a frame carrying
# only the command byte it raises IndexError.
def _predicate_that_raises_on_short_frames(msg) -> bool:
    return msg.data[1] == const.HOME_SUCCESS


async def _drain(task: asyncio.Task, iface: CANInterface) -> None:
    """Stops a listener task and waits for it, so nothing leaks between tests."""
    iface._is_listening = False
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


@pytest.mark.asyncio
async def test_sim_listener_survives_a_frame_that_raises():
    """A frame that trips a predicate must not stop the listener."""
    iface = CANInterface(use_simulator=True)
    reader = asyncio.StreamReader()
    iface._sim_reader = reader
    iface._is_listening = True
    task = asyncio.create_task(iface._listen_for_messages_sim())
    try:
        poisoned = iface.create_response_future(
            CAN_ID, const.CMD_GO_HOME,
            response_predicate=_predicate_that_raises_on_short_frames,
        )
        reader.feed_data(b"SIM_CAN_RECV 001 1 91\n")
        await asyncio.sleep(0.05)
        assert not poisoned.done()

        survivor = iface.create_response_future(
            CAN_ID, const.CMD_READ_ENCODER_ADDITION
        )
        reader.feed_data(b"SIM_CAN_RECV 001 2 3100\n")
        await asyncio.wait_for(survivor, timeout=1.0)
        assert survivor.result().data[0] == const.CMD_READ_ENCODER_ADDITION
        assert not task.done(), "the listener task ended on a bad frame"
    finally:
        await _drain(task, iface)


@pytest.mark.asyncio
async def test_sim_listener_survives_undecodable_bytes():
    """The line the simulator sent need not be valid UTF-8."""
    iface = CANInterface(use_simulator=True)
    reader = asyncio.StreamReader()
    iface._sim_reader = reader
    iface._is_listening = True
    task = asyncio.create_task(iface._listen_for_messages_sim())
    try:
        reader.feed_data(b"\xff\xfe not utf-8 at all\n")
        await asyncio.sleep(0.05)

        survivor = iface.create_response_future(
            CAN_ID, const.CMD_READ_ENCODER_ADDITION
        )
        reader.feed_data(b"SIM_CAN_RECV 001 2 3100\n")
        await asyncio.wait_for(survivor, timeout=1.0)
        assert not task.done(), "the listener task ended on undecodable bytes"
    finally:
        await _drain(task, iface)


@pytest.mark.asyncio
async def test_hardware_listener_survives_a_frame_that_raises():
    """The hardware listener has the same structure and the same duty."""
    iface = CANInterface(use_simulator=True)  # only the queue is exercised
    queue: asyncio.Queue = asyncio.Queue()
    iface._message_queue = queue
    iface._is_listening = True
    task = asyncio.create_task(iface._listen_for_messages_hw())
    try:
        poisoned = iface.create_response_future(
            CAN_ID, const.CMD_GO_HOME,
            response_predicate=_predicate_that_raises_on_short_frames,
        )
        await queue.put(
            CanMessage(
                arbitration_id=CAN_ID, data=bytes([const.CMD_GO_HOME]), dlc=1
            )
        )
        await asyncio.sleep(0.05)
        assert not poisoned.done()

        survivor = iface.create_response_future(
            CAN_ID, const.CMD_READ_ENCODER_ADDITION
        )
        await queue.put(
            CanMessage(
                arbitration_id=CAN_ID,
                data=bytes([const.CMD_READ_ENCODER_ADDITION, 0x00]),
                dlc=2,
            )
        )
        await asyncio.wait_for(survivor, timeout=1.0)
        assert not task.done(), "the listener task ended on a bad frame"
    finally:
        await _drain(task, iface)
