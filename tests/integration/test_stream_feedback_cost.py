"""
Feedback polling costs one round trip per axis, not three (defect L5).

`_run_feedback` used to re-enable motor responses before every encoder read and
disable them again afterwards. By this repository's reading of the manual -
`SUPPRESSIBLE_RESPONSE_COMMANDS` in the simulator's motor model - CanRSP only
suppresses the run commands of sections 6.4-6.8, so 0x31 is answered either way
and the toggling bought nothing while tripling the traffic.

The count is taken by subclassing `LowLevelAPI` rather than mocking it: the real
methods still run against the real simulator, and only the tally is added.
"""
import asyncio

import pytest

from mks_servo_can import ServoStream, StreamAxis
from mks_servo_can.low_level_api import LowLevelAPI

pytestmark = pytest.mark.integration

FEEDBACK_HZ = 40.0
POLL_SECONDS = 0.5


class CountingAPI(LowLevelAPI):
    """A real LowLevelAPI that also counts CanRSP commands."""

    def __init__(self, can_interface):
        super().__init__(can_interface)
        self.respond_calls = 0

    async def set_slave_respond_active(self, can_id, respond_enabled, active_enabled):
        self.respond_calls += 1
        return await super().set_slave_respond_active(
            can_id, respond_enabled, active_enabled
        )


@pytest.mark.asyncio
async def test_feedback_does_not_toggle_can_rsp_per_read(compliance_can_interface):
    """Only the entry and exit transitions may send 0x8C."""
    axes = [StreamAxis("pan", 1, accel_param=250)]
    stream = ServoStream(
        compliance_can_interface,
        axes,
        rate_hz=50,
        feedback_rate_hz=FEEDBACK_HZ,
        watchdog_timeout=None,
    )
    counting = CountingAPI(compliance_can_interface)
    stream._api = counting

    async with stream:
        stream.set_target("pan", 0.0)
        await asyncio.sleep(POLL_SECONDS)
        measured = stream.get_measured_positions()["pan"]
        during_run = counting.respond_calls

    assert measured is not None, "feedback never produced a reading"
    # One call to disable on entry. Anything beyond that is per-read toggling:
    # at 40 Hz over half a second it would be tens of calls.
    assert during_run <= 1, (
        f"feedback sent {during_run} CanRSP commands while running; it should "
        "send none beyond the one that disables responses at entry"
    )
