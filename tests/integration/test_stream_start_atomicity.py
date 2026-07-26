"""
A failed `ServoStream.start()` must leave no motor silent (defect L5).

`start()` disables motor responses axis by axis. If it failed partway, the axes
already done kept responses disabled: `__aenter__` never returns, so `__aexit__`
never runs, and calling `stop()` by hand returns immediately because `_running`
is still False. The motor is then mute for the rest of the session and every
later command times out.

The failure is provoked without stubbing anything: one of the axes is given a
CAN ID no motor answers on, so disabling responses on it times out for real.

"Still answering" has to be checked with a command that CanRSP actually
suppresses - the run commands of manual sections 6.4-6.8. An encoder read is
answered either way, so it proves nothing here.
"""
import asyncio

import pytest

from mks_servo_can import ServoStream, StreamAxis
from mks_servo_can.exceptions import MKSServoError
from mks_servo_can.low_level_api import LowLevelAPI

pytestmark = pytest.mark.integration

# The compliance simulator serves CAN IDs 1-3; nothing answers on this one.
ABSENT_CAN_ID = 99


@pytest.mark.asyncio
async def test_failed_start_restores_responses_on_the_axes_it_disabled(
    compliance_can_interface,
):
    """After a start that fails, the healthy motor must still answer."""
    api = LowLevelAPI(compliance_can_interface)
    await api.enable_motor(1, True)
    here = await api.read_encoder_value_addition(1)

    # Establish that the motor answers a suppressible command before the failed
    # start, so a timeout afterwards can only be the doing of ServoStream.
    await api.run_position_mode_absolute_axis(1, 100, 250, here)

    stream = ServoStream(
        compliance_can_interface,
        [
            StreamAxis("healthy", 1, accel_param=250),
            StreamAxis("absent", ABSENT_CAN_ID, accel_param=250),
        ],
        rate_hz=50,
        watchdog_timeout=None,
    )

    with pytest.raises(MKSServoError):
        await stream.start()

    # The axis that was disabled first has to have been restored, so the same
    # suppressible command must still be answered.
    status = await asyncio.wait_for(
        api.run_position_mode_absolute_axis(1, 100, 250, here), timeout=3.0
    )
    assert status is not None, "motor 1 was left with its responses disabled"


@pytest.mark.asyncio
async def test_failed_start_does_not_leave_the_loop_running(
    compliance_can_interface,
):
    """A start that raised must not leave background tasks behind."""
    stream = ServoStream(
        compliance_can_interface,
        [
            StreamAxis("healthy", 1, accel_param=250),
            StreamAxis("absent", ABSENT_CAN_ID, accel_param=250),
        ],
        rate_hz=50,
        watchdog_timeout=None,
    )

    with pytest.raises(MKSServoError):
        await stream.start()

    assert stream._loop_task is None
    assert stream._feedback_task is None
    # stop() must remain safe to call, and must not raise.
    await stream.stop()
