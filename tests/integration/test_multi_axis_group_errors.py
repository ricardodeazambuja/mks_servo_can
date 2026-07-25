"""
A partial failure of a group operation must be reported (defect L11).

`MultiAxisController` runs an operation across every axis and gathers the
per-axis failures into `MultiAxisError.individual_errors`. That contract is the
only thing standing between "two of my three axes moved" and "the move
succeeded", and nothing tested it against real motors - the existing unit tests
build `MagicMock(spec=Axis)` and then assign the attributes they want, which
teaches the mock an API and proves nothing about the real one.

These tests drive the compliance simulator, which serves CAN IDs 1-3. Nothing
answers on ID 99, which is how a communication failure is provoked without
stubbing anything out.
"""
import asyncio

import pytest
import pytest_asyncio

from mks_servo_can import Axis, CANInterface, MultiAxisController
from mks_servo_can import constants as const
from mks_servo_can.exceptions import MultiAxisError

pytestmark = pytest.mark.integration

STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION

# Nothing is configured on this CAN ID, so every command to it times out.
DEAD_CAN_ID = 99

# Slow enough that a move of several revolutions is unambiguously still running
# while a short move on another axis has finished.
SLOW_SPEED_PARAM = 60


def _axis(interface, can_id, name, speed_param=None):
    """
    Builds an Axis on the given interface.

    Args:
        interface: The connected CANInterface.
        can_id: Motor CAN ID.
        name: Axis name, as the controller will know it.
        speed_param: Optional default speed parameter override.

    Returns:
        The Axis.
    """
    kwargs = {}
    if speed_param is not None:
        kwargs["default_speed_param"] = speed_param
    return Axis(
        can_interface_manager=interface,
        motor_can_id=can_id,
        name=name,
        **kwargs,
    )


@pytest_asyncio.fixture
async def live_controller(compliance_can_interface: CANInterface):
    """
    A controller over the three real simulated motors, enabled and zeroed.

    The simulator is module-scoped, so each axis is stopped and re-zeroed on the
    way in and stopped again on the way out: a motor left running sends late
    completion frames into the next test.
    """
    controller = MultiAxisController(can_interface_manager=compliance_can_interface)
    for can_id in (1, 2, 3):
        controller.add_axis(_axis(compliance_can_interface, can_id, f"ax{can_id}"))

    await controller.initialize_all_axes()
    await controller.enable_all_axes()
    await controller.stop_all_axes()
    await asyncio.sleep(0.2)
    for axis in controller.axes.values():
        await axis.set_current_position_as_zero()

    yield controller

    try:
        await controller.stop_all_axes()
        await asyncio.sleep(0.2)
    except Exception:  # pragma: no cover - best-effort teardown
        pass


@pytest_asyncio.fixture
async def controller_with_a_dead_axis(compliance_can_interface: CANInterface):
    """
    Two real motors and one that does not exist, in that order.

    The dead axis is added *last* on purpose: several of the failure paths here
    index positionally, and a failure at the end of the list is the case that
    goes missing.
    """
    controller = MultiAxisController(can_interface_manager=compliance_can_interface)
    controller.add_axis(_axis(compliance_can_interface, 1, "ax1"))
    controller.add_axis(_axis(compliance_can_interface, 2, "ax2"))
    controller.add_axis(_axis(compliance_can_interface, DEAD_CAN_ID, "dead"))

    yield controller

    for name in ("ax1", "ax2"):
        try:
            await controller.axes[name].stop_motor()
        except Exception:  # pragma: no cover - best-effort teardown
            pass
    await asyncio.sleep(0.2)


class TestPartialFailureIsReported:
    """One dead axis among live ones must not pass for success."""

    @pytest.mark.asyncio
    async def test_enable_all_axes_names_the_axis_that_failed(
        self, controller_with_a_dead_axis
    ):
        with pytest.raises(MultiAxisError) as excinfo:
            await controller_with_a_dead_axis.enable_all_axes()

        errors = excinfo.value.individual_errors
        assert errors is not None, "the failure was reported with no detail at all"
        assert set(errors) == {"dead"}, (
            f"expected only the dead axis to fail, got {sorted(errors)}"
        )

    @pytest.mark.asyncio
    async def test_get_all_positions_reports_rather_than_returns_a_short_dict(
        self, controller_with_a_dead_axis
    ):
        """
        The dangerous alternative is returning the two positions it *could* get.

        A caller that received `{"ax1": ..., "ax2": ...}` with no error would
        have no way to tell a three-axis machine from a two-axis one.
        """
        await controller_with_a_dead_axis.axes["ax1"].initialize()
        await controller_with_a_dead_axis.axes["ax2"].initialize()

        with pytest.raises(MultiAxisError) as excinfo:
            await controller_with_a_dead_axis.get_all_positions_user()

        assert set(excinfo.value.individual_errors) == {"dead"}

    @pytest.mark.asyncio
    async def test_the_live_axes_were_still_commanded(
        self, controller_with_a_dead_axis
    ):
        """
        A group failure must not mean the whole group was skipped.

        The operation is attempted on every axis; the error collects what went
        wrong. Asserting the effect here - that the live motors really are
        enabled afterwards - is what distinguishes that from "gave up on the
        first failure".
        """
        with pytest.raises(MultiAxisError):
            await controller_with_a_dead_axis.enable_all_axes()

        for name in ("ax1", "ax2"):
            assert controller_with_a_dead_axis.axes[name].is_enabled(), (
                f"axis '{name}' was never enabled; the group operation stopped "
                "at the failure instead of covering every axis"
            )


class TestWaitForAllMovesToComplete:
    """
    The failure that goes missing when it is not the first one.

    `wait_for_all_moves_to_complete` builds its task list from the axes that are
    still moving, awaits them, and then rebuilds that list to map results back
    to axis names. The rebuild happens *after* the wait, by which time every
    axis that finished - successfully or not - reports `is_move_complete()` as
    True and drops out. The two lists are therefore different lengths, and the
    positional index into the second one is wrong.
    """

    @pytest.mark.asyncio
    async def test_a_timeout_on_the_last_axis_is_not_silently_dropped(
        self, live_controller
    ):
        """
        Short moves on ax1 and ax2, a long one on ax3, then a wait too short
        for ax3.

        Results come back as [None, None, TimeoutError]. When the axis list is
        rebuilt after the wait it holds only ax3 - the one still pending - so
        the error at index 2 has no entry to map onto, and the bounds check
        that was meant to be defensive discards it. The method then returns
        normally, telling the caller that every move completed.
        """
        ax1, ax2, ax3 = (live_controller.axes[n] for n in ("ax1", "ax2", "ax3"))

        # A fraction of a revolution finishes quickly; several revolutions at a
        # slow speed does not. The gap is wide enough not to be a race.
        await ax1.move_to_position_abs_user(30.0, wait=False)
        await ax2.move_to_position_abs_user(30.0, wait=False)
        await ax3.move_to_position_abs_user(
            3600.0, speed_user=20.0, wait=False
        )

        with pytest.raises(MultiAxisError) as excinfo:
            await live_controller.wait_for_all_moves_to_complete(
                timeout_per_axis=2.0
            )

        errors = excinfo.value.individual_errors
        assert errors is not None, "a move timed out and was reported with no detail"
        assert "ax3" in errors, (
            f"ax3 timed out but was not named; individual_errors held {sorted(errors)}"
        )

    @pytest.mark.asyncio
    async def test_an_error_is_attributed_to_the_axis_that_raised_it(
        self, live_controller
    ):
        """
        Mapping by position must not mis-name the failure.

        Reporting the wrong axis is worse than reporting none: it sends whoever
        reads the error to the wrong motor.
        """
        ax1, ax2, ax3 = (live_controller.axes[n] for n in ("ax1", "ax2", "ax3"))

        await ax1.move_to_position_abs_user(3600.0, speed_user=20.0, wait=False)
        await ax2.move_to_position_abs_user(30.0, wait=False)
        await ax3.move_to_position_abs_user(30.0, wait=False)

        with pytest.raises(MultiAxisError) as excinfo:
            await live_controller.wait_for_all_moves_to_complete(
                timeout_per_axis=2.0
            )

        errors = excinfo.value.individual_errors
        assert set(errors) == {"ax1"}, (
            f"only ax1 was still moving, but the error named {sorted(errors)}"
        )

    @pytest.mark.asyncio
    async def test_all_moves_completing_raises_nothing(self, live_controller):
        """The happy path must stay quiet, or the guard above is useless."""
        for name in ("ax1", "ax2", "ax3"):
            await live_controller.axes[name].move_to_position_abs_user(
                20.0, wait=False
            )

        await live_controller.wait_for_all_moves_to_complete(timeout_per_axis=10.0)

        assert live_controller.are_all_moves_complete()
