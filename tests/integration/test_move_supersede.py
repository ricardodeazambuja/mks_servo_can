"""
Re-targeting a move that is still in flight (defect L1).

The motor reuses one command byte for the acknowledgement of a command and for
the asynchronous completion/abort notification of the move that command
superseded. These tests pin the two ways of getting that wrong:

1. Against the simulator, driving a real `Axis`: a second move issued while the
   first is still running must be accepted and must run to its target.
2. Against a real `CANInterface` with hand-built frames: a motor that emits *no*
   abort frame must not have the credit registered for that abort swallow a
   later, legitimate reply.

Both are integration-shaped on purpose. The pre-existing "coverage" for this
logic mocked `expect_stale_notification` and asserted only that a credit was
*requested*, which is why a 100% reproducible bug shipped.
"""
import asyncio

import pytest
import pytest_asyncio

from mks_servo_can import Axis, CANInterface
from mks_servo_can import constants as const
from mks_servo_can.low_level_api import CanMessage

# One revolution of the motor's raw encoder coordinate system.
STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION

# Slow enough that a move of half a revolution is comfortably still running when
# the second move is dispatched, fast enough not to dominate the suite runtime.
SLOW_SPEED_PARAM = 60


@pytest_asyncio.fixture
async def retarget_axis(basic_can_interface: CANInterface):
    """
    An enabled Axis on the basic simulator, parked at zero.

    The simulator is module-scoped, so the motor is explicitly stopped on the
    way in and on the way out. A test that leaves it moving otherwise sends
    late abort frames into the *next* test, which shifts the frame ordering
    these tests are about and can mask the very defect they pin.
    """
    axis = Axis(
        can_interface_manager=basic_can_interface,
        motor_can_id=1,
        name="RetargetAxis",
    )
    await axis.initialize()
    await axis.enable_motor()
    await axis.stop_motor()
    await asyncio.sleep(0.2)
    await axis.set_current_position_as_zero()

    yield axis

    try:
        await axis.stop_motor()
        await asyncio.sleep(0.2)
    except Exception:  # pylint: disable=broad-except
        pass  # teardown must not mask the test's own failure


@pytest.mark.integration
@pytest.mark.asyncio
async def test_retarget_mid_move_is_accepted(retarget_axis: Axis):
    """
    A move dispatched while another is running must not raise.

    Before the fix this raised
    `MotorError: ... failed to start (status 0x00)` every single time: the
    credit registered for the superseded move's abort frame consumed the new
    command's acknowledgement (status 0x01) instead, and the abort (status 0x00)
    that arrived next resolved the acknowledgement's future.
    """
    axis = retarget_axis

    await axis.move_to_position_abs_axis(
        STEPS_PER_REV // 2, speed_param=SLOW_SPEED_PARAM, wait=False
    )
    await asyncio.sleep(0.3)
    assert not axis.is_move_complete(), "first move finished before it was retargeted"

    # The line under test: this must be accepted, not rejected as a failed start.
    await axis.move_to_position_abs_axis(
        STEPS_PER_REV // 4, speed_param=SLOW_SPEED_PARAM, wait=False
    )

    await axis.wait_for_move_completion(timeout=20.0)

    final_steps = await axis.get_current_position_steps()
    assert abs(final_steps - STEPS_PER_REV // 4) < STEPS_PER_REV // 100, (
        f"axis settled at {final_steps}, expected ~{STEPS_PER_REV // 4}"
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_repeated_retargets_all_succeed(retarget_axis: Axis):
    """
    Retargeting repeatedly is the case the streaming design rests on.

    One success could be luck of the arrival order; this asserts the property
    holds over a sequence.
    """
    axis = retarget_axis
    targets = [STEPS_PER_REV // 2, STEPS_PER_REV // 3, STEPS_PER_REV // 4, 0]

    for target in targets:
        await axis.move_to_position_abs_axis(
            target, speed_param=SLOW_SPEED_PARAM, wait=False
        )
        await asyncio.sleep(0.15)

    await axis.wait_for_move_completion(timeout=20.0)

    final_steps = await axis.get_current_position_steps()
    assert abs(final_steps - targets[-1]) < STEPS_PER_REV // 100, (
        f"axis settled at {final_steps}, expected ~{targets[-1]}"
    )


def _frame(can_id: int, command: int, status: int) -> CanMessage:
    """Builds a two-byte motor reply frame (command echo plus status)."""
    return CanMessage(
        arbitration_id=can_id, data=bytearray([command, status]), dlc=2
    )


@pytest.mark.asyncio
async def test_credit_does_not_consume_the_acknowledgement():
    """
    The credit must match the abort's status byte, not merely arrival order.

    The acknowledgement (POS_RUN_STARTING) arrives *before* the superseded
    move's abort, so an order-based credit eats the wrong frame.
    """
    iface = CANInterface(use_simulator=True)
    can_id = 1
    cmd = const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS

    iface.expect_stale_notification(can_id, cmd)
    ack_future = iface.create_response_future(can_id, cmd)

    await iface._process_received_message(_frame(can_id, cmd, const.POS_RUN_STARTING))
    assert ack_future.done(), "the acknowledgement was discarded as stale"
    assert ack_future.result().data[1] == const.POS_RUN_STARTING

    # The abort arrives next and must be the frame that is dropped.
    later_future = iface.create_response_future(can_id, cmd)
    await iface._process_received_message(_frame(can_id, cmd, const.POS_RUN_FAIL))
    assert not later_future.done(), "the superseded move's abort was not discarded"


@pytest.mark.asyncio
async def test_credit_expires_when_no_abort_arrives():
    """
    A credit for an abort that never comes must not poison a later reply.

    If real hardware turns out not to emit an abort frame at all, an immortal
    credit swallows the next legitimate response instead - the mirror image of
    the bug above, and just as fatal.
    """
    iface = CANInterface(use_simulator=True)
    can_id = 2
    cmd = const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS

    iface.expect_stale_notification(can_id, cmd, ttl_seconds=0.05)
    await asyncio.sleep(0.1)

    completion_future = iface.create_response_future(can_id, cmd)
    await iface._process_received_message(_frame(can_id, cmd, const.POS_RUN_COMPLETE))
    assert completion_future.done(), (
        "an expired credit swallowed a legitimate completion frame"
    )
