"""
The simulator's command injector, which had never injected a command.

`/inject`, `/inject_template` and `/templates` only became reachable when L15
was fixed, and the first thing that could be seen through them was that nothing
behind them worked:

- every command specification was built from the manual's DLC, which counts the
  command code and the CRC as well as the payload, so `validate_command`
  demanded two data bytes for commands that take none and rejected all
  eighteen;
- `inject_command` `await`ed `process_command`, which is synchronous and
  returns a tuple, and handed it a one-argument completion callback where the
  motor calls a two-argument one;
- eight of the eleven templates named one command and sent another - `enable`
  sent 0x80, which is encoder calibration.

The tests here drive a real `VirtualCANBus` and a real `SimulatedMotor`; the
point is what arrives at the motor, not that a method was called.
"""
import asyncio

import pytest

from mks_servo_can import constants as const
from mks_simulator.clock import SteppedClock
from mks_simulator.interface.debug_tools import CommandInjector
from mks_simulator.motor_model import SimulatedMotor
from mks_simulator.virtual_can_bus import VirtualCANBus

# Loops created for the synchronous tests below, kept alive so they are not
# collected while a bus holds a reference. Nothing is ever scheduled on them.
_IDLE_LOOPS = []


def _loop_for_tests():
    """
    Returns the running loop, or a fresh idle one for a synchronous test.

    `asyncio.get_event_loop()` is not usable here: it is deprecated outside a
    running loop on Python 3.12+, and it raises today once anything in the
    thread has called `set_event_loop(None)` - which pytest-asyncio does at the
    end of every async test, so whether a synchronous test worked depended on
    what had run before it.
    """
    try:
        return asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        _IDLE_LOOPS.append(loop)
        return loop


def _make_injector(clock=None, can_id=1):
    """
    Builds an injector over a real bus carrying one real motor.

    Args:
        clock: Optional clock for the motor; the default is wall-clock.
        can_id: CAN ID for the motor.

    Returns:
        A `(injector, motor)` pair.
    """
    loop = _loop_for_tests()
    bus = VirtualCANBus(loop)
    motor = SimulatedMotor(can_id=can_id, loop=loop, clock=clock)
    motor.is_enabled = True
    bus.add_motor(motor)
    return CommandInjector(bus), motor


class TestTemplates:
    """The pre-defined templates must send the command they are named after."""

    def test_enable_and_disable_send_the_enable_command(self):
        """
        `enable` sent 0x80 - encoder calibration - and `disable` sent the same
        code with a different data byte, which 0x80 does not carry at all.
        """
        injector, _ = _make_injector()
        templates = injector.get_available_templates()

        assert templates["enable"]["code"] == const.CMD_ENABLE_MOTOR
        assert templates["enable"]["data"] == [0x01]
        assert templates["disable"]["code"] == const.CMD_ENABLE_MOTOR
        assert templates["disable"]["data"] == [0x00]

    def test_the_positioning_templates_use_the_absolute_command(self):
        """
        Both are described as moving *to* a position, and both sent 0xFD, which
        moves by a relative number of pulses.
        """
        injector, _ = _make_injector()
        templates = injector.get_available_templates()

        for name in ("move_home", "move_90deg"):
            assert (
                templates[name]["code"]
                == const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES
            ), f"{name} does not send the absolute-position command"

    def test_every_template_validates(self):
        """
        A template that cannot survive validation can never be injected, and
        eleven of eleven could not: five carried the wrong number of bytes and
        five named a command the packaged manual specification did not cover.
        """
        injector, _ = _make_injector()

        for name, template in injector.get_available_templates().items():
            assert injector.get_command_spec(template["code"]) is not None, (
                f"template {name!r} sends 0x{template['code']:02X}, which the "
                "packaged manual specification does not describe"
            )
            is_valid, message = injector.validate_command(
                template["code"], template["data"]
            )
            assert is_valid, f"template {name!r} is rejected: {message}"


class TestCommandSpecifications:
    """Lengths in a `CommandSpec` count payload bytes, not whole frames."""

    def test_a_command_with_no_arguments_expects_no_data_bytes(self):
        """
        0x30 is `CAN_ID | 2 | 0x30 | CRC`: DLC 2, but zero payload bytes. Read
        as a payload length it made every argument-free command unusable.
        """
        injector, _ = _make_injector()

        spec = injector.get_command_spec(const.CMD_READ_ENCODER_CARRY)
        assert spec is not None
        assert spec.data_length == 0

        is_valid, message = injector.validate_command(
            const.CMD_READ_ENCODER_CARRY, []
        )
        assert is_valid, message

    def test_a_command_with_arguments_counts_only_the_arguments(self):
        """
        0xFE carries speed, acceleration and a 24-bit position: six bytes
        inside a frame of DLC 8.
        """
        injector, _ = _make_injector()

        spec = injector.get_command_spec(
            const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES
        )
        assert spec is not None
        assert spec.data_length == 6

    def test_the_fallback_table_does_not_call_calibration_an_enable(self):
        """
        The hard-coded fallback repeated the same misnaming, so the two ways of
        loading specifications agreed with each other and disagreed with the
        manual.
        """
        injector, _ = _make_injector()
        injector.command_specs = {}
        injector._setup_basic_command_specs()

        assert injector.command_specs[const.CMD_ENABLE_MOTOR].name == "Enable Motor"
        assert "alibrat" in injector.command_specs[const.CMD_CALIBRATE_ENCODER].name


class TestInjection:
    """What actually reaches the motor."""

    @pytest.mark.asyncio
    async def test_a_read_comes_back_with_the_motors_answer(self):
        """
        The reply is the frame the motor would have put on the bus: the echoed
        command code, the encoder value, and a CRC.
        """
        injector, motor = _make_injector()
        motor.position_steps = 0.0

        record = await injector.inject_command(
            motor_id=1, command_code=const.CMD_READ_ENCODER_CARRY, data_bytes=[]
        )

        assert record.success, record.error_message
        assert record.response_data is not None
        assert record.response_data[0] == const.CMD_READ_ENCODER_CARRY
        assert len(record.response_data) == 8

    @pytest.mark.asyncio
    async def test_an_injected_move_moves_the_motor(self):
        """
        The effect, not the return value. Starting away from zero separates the
        absolute command the template now sends from the relative one it used
        to send: a quarter turn *to* 4096 is not a quarter turn *from* 1000.
        """
        clock = SteppedClock()
        injector, motor = _make_injector(clock=clock)
        await motor.start()
        await asyncio.sleep(0)
        try:
            motor.position_steps = 1000.0

            record = await injector.inject_template_command(1, "move_90deg")
            # The motor answers immediately and does the move in a task; let it
            # reach the point where it has taken the target on board.
            await asyncio.sleep(0)

            assert record.success, record.error_message
            quarter_turn = const.ENCODER_PULSES_PER_REVOLUTION // 4
            assert motor.target_position_steps == quarter_turn
        finally:
            if motor._current_move_task and not motor._current_move_task.done():
                motor._current_move_task.cancel()
            await motor.stop_simulation()
            await asyncio.sleep(0)

    @pytest.mark.asyncio
    async def test_injecting_into_an_absent_motor_reports_failure(self):
        """The motor is not there; nothing may claim otherwise."""
        injector, _ = _make_injector()

        record = await injector.inject_command(
            motor_id=99, command_code=const.CMD_READ_ENCODER_CARRY, data_bytes=[]
        )

        assert not record.success
        assert "99" in record.error_message

    @pytest.mark.asyncio
    async def test_injection_leaves_the_buss_completion_callback_alone(self):
        """
        A motor keeps whichever completion callback it was handed last, so an
        injection would otherwise redirect a connected client's move-completion
        frames to the injector for the rest of the session.
        """
        injector, motor = _make_injector()

        async def bus_callback(_can_id, _payload):
            """Stands in for the callback `VirtualCANBus` installs."""

        motor._send_completion_callback = bus_callback

        await injector.inject_command(
            motor_id=1, command_code=const.CMD_READ_ENCODER_CARRY, data_bytes=[]
        )

        assert motor._send_completion_callback is bus_callback

    @pytest.mark.asyncio
    async def test_history_records_what_was_injected(self):
        """The injector is a debugging tool; its log has to be true."""
        injector, _ = _make_injector()

        await injector.inject_command(
            motor_id=1, command_code=const.CMD_READ_MOTOR_SPEED_RPM, data_bytes=[]
        )

        history = injector.get_command_history()
        assert len(history) == 1
        assert history[0].command_code == const.CMD_READ_MOTOR_SPEED_RPM
        assert history[0].success
