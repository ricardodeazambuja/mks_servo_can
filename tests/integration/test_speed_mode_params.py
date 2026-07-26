"""
Saving and clearing speed-mode parameters, command 0xFF (defect L2).

The motor echoes the command byte, 0xFF, and puts the status in the second byte.
The library expected it to echo the *sub-command* (0xC8 save / 0xCA clean)
instead, so every call timed out after waiting for a frame that never comes.
The command had no test coverage at all, which is how that survived.
"""
import pytest

from mks_servo_can import LowLevelAPI


@pytest.mark.integration
@pytest.mark.asyncio
async def test_save_speed_mode_params_completes(basic_api: LowLevelAPI):
    """Saving must return, not time out waiting for an echo of 0xC8."""
    await basic_api.save_or_clean_speed_mode_params(can_id=1, save=True)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_clean_speed_mode_params_completes(basic_api: LowLevelAPI):
    """Clearing takes the other sub-command and must behave the same way."""
    await basic_api.save_or_clean_speed_mode_params(can_id=1, save=False)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_save_then_clean_round_trip(basic_api: LowLevelAPI):
    """
    The pair is what a caller actually uses: park a speed, then undo it.

    Asserting each call returns is the whole point - a timeout here is the
    defect, and the motor reports success in the status byte the library reads.
    """
    await basic_api.save_or_clean_speed_mode_params(can_id=1, save=True)
    await basic_api.save_or_clean_speed_mode_params(can_id=1, save=False)
