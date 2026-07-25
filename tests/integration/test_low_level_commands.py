"""
The commands of `low_level_api.py` that nothing had ever sent.

The module sat at 49%: whole commands with no coverage at all, on a class where
every method is a frame layout. L2 was exactly this shape - 0xFF always timed
out, and the one unit test that existed asserted the wrong expectation, so the
defect was covered *and* invisible.

The compliance suite drives all 46 commands and checks their framing. That
proves a frame goes out and a well-formed one comes back; it does not prove the
motor did what the command said. These tests assert the effect: a setting is
written and then read back out of the motor with 0x00, a read is taken against
motor state the test arranged, and an out-of-range argument is refused before
anything reaches the bus.
"""
import asyncio
import struct

import pytest

from mks_servo_can import LowLevelAPI, exceptions
from mks_servo_can import constants as const

# The compliance simulator serves CAN IDs 1-3; nothing answers on this one.
ABSENT_CAN_ID = 99
MOTOR = 1

pytestmark = pytest.mark.asyncio


async def _settle(api: LowLevelAPI, seconds: float = 0.6) -> None:
    """
    Waits for a dispatched move to finish.

    The simulator runs on a real clock here, so a move takes real time; the
    deterministic alternative is `tests/determinism/`, which drives the motor
    model directly rather than over a socket.

    Args:
        api: Unused; present so callers read as one sequence of awaits.
        seconds: How long to wait.
    """
    await asyncio.sleep(seconds)


async def read_parameter(api: LowLevelAPI, code: int) -> bytes:
    """
    Reads a stored parameter back out of the motor with command 0x00.

    Args:
        api: A connected `LowLevelAPI`.
        code: The command code of the setting to read, e.g. 0x83.

    Returns:
        The parameter's bytes, big-endian, without the echoed code or the CRC.
    """
    echoed, payload = await api.read_system_parameter(MOTOR, code)
    assert echoed == code, f"0x00 echoed 0x{echoed:02X} for a read of 0x{code:02X}"
    return payload


class TestSettingsRoundTrip:
    """
    Each setter, then the same value read back out of the motor.

    Asserting that a setter returned without raising proves only that the motor
    answered `status = 1`. These read the value back, which is the difference
    between "the frame was accepted" and "the setting took".
    """

    async def test_working_current(self, compliance_api):
        """0x83 carries milliamps as a big-endian uint16."""
        await compliance_api.set_working_current(MOTOR, 1600)
        assert struct.unpack(">H", await read_parameter(compliance_api, 0x83))[0] == 1600

        await compliance_api.set_working_current(MOTOR, 800)
        assert struct.unpack(">H", await read_parameter(compliance_api, 0x83))[0] == 800

    async def test_work_mode(self, compliance_api):
        """0x82 selects one of the six control modes."""
        await compliance_api.set_work_mode(MOTOR, const.MODE_SR_CLOSE)
        assert (await read_parameter(compliance_api, 0x82))[0] == const.MODE_SR_CLOSE

        await compliance_api.set_work_mode(MOTOR, const.MODE_SR_VFOC)
        assert (await read_parameter(compliance_api, 0x82))[0] == const.MODE_SR_VFOC

    async def test_subdivision(self, compliance_api):
        """0x84 sets microstepping, which scales every pulse-mode command."""
        await compliance_api.set_subdivision(MOTOR, 32)
        assert (await read_parameter(compliance_api, 0x84))[0] == 32

        await compliance_api.set_subdivision(MOTOR, 16)
        assert (await read_parameter(compliance_api, 0x84))[0] == 16

    async def test_en_pin_active_level(self, compliance_api):
        """0x85: 0 active low, 1 active high, 2 always active."""
        await compliance_api.set_en_pin_active_level(MOTOR, const.EN_ACTIVE_HIGH)
        assert (await read_parameter(compliance_api, 0x85))[0] == const.EN_ACTIVE_HIGH

    async def test_motor_direction(self, compliance_api):
        """0x86: 0 clockwise, 1 counter-clockwise."""
        await compliance_api.set_motor_direction(MOTOR, const.DIR_CCW)
        assert (await read_parameter(compliance_api, 0x86))[0] == const.DIR_CCW

        await compliance_api.set_motor_direction(MOTOR, const.DIR_CW)
        assert (await read_parameter(compliance_api, 0x86))[0] == const.DIR_CW

    async def test_holding_current_percentage(self, compliance_api):
        """0x9B: code 0 is 10%, rising in tens to code 8 for 90%."""
        await compliance_api.set_holding_current_percentage(MOTOR, 0x04)
        assert (await read_parameter(compliance_api, 0x9B))[0] == 0x04

    async def test_can_bitrate(self, compliance_api):
        """0x8A: 0 is 125K through 3 for 1M."""
        await compliance_api.set_can_bitrate(MOTOR, const.CAN_BITRATE_1M)
        assert (await read_parameter(compliance_api, 0x8A))[0] == const.CAN_BITRATE_1M

    async def test_group_id(self, compliance_api):
        """0x8D: a second address the motor also answers to, as a uint16."""
        await compliance_api.set_group_id(MOTOR, 0x50)
        assert struct.unpack(">H", await read_parameter(compliance_api, 0x8D))[0] == 0x50

    async def test_the_boolean_settings(self, compliance_api):
        """
        Four switches that share one shape.

        Each is set both ways: a setter that ignores its argument passes a test
        that only ever turns something on.
        """
        cases = (
            (compliance_api.set_auto_screen_off, 0x87),
            (compliance_api.set_stall_protection, 0x88),
            (compliance_api.set_subdivision_interpolation, 0x89),
        )
        for setter, code in cases:
            await setter(MOTOR, True)
            assert (await read_parameter(compliance_api, code))[0] == 1, hex(code)
            await setter(MOTOR, False)
            assert (await read_parameter(compliance_api, code))[0] == 0, hex(code)

        await compliance_api.set_key_lock(MOTOR, True)
        assert (await read_parameter(compliance_api, 0x8F))[0] == 1
        await compliance_api.set_key_lock(MOTOR, False)
        assert (await read_parameter(compliance_api, 0x8F))[0] == 0

    async def test_slave_respond_and_active(self, compliance_api):
        """
        0x8C carries two independent switches, and both must land.

        A single byte written where two are expected leaves the second at its
        default, which is exactly the failure this reads back to exclude. The
        pair is restored before the test ends: with responses off, nothing else
        in the session would get an answer.
        """
        try:
            await compliance_api.set_slave_respond_active(
                MOTOR, respond_enabled=True, active_enabled=False
            )
            assert list(await read_parameter(compliance_api, 0x8C)) == [1, 0]
        finally:
            await compliance_api.set_slave_respond_active(
                MOTOR, respond_enabled=True, active_enabled=True
            )
        assert list(await read_parameter(compliance_api, 0x8C)) == [1, 1]


class TestReadsReflectMotorState:
    """Reads taken against state the test arranged, rather than against nothing."""

    async def test_the_en_pin_status_follows_enabling_the_motor(self, compliance_api):
        """0x3A must change when 0xF3 changes it."""
        await compliance_api.enable_motor(MOTOR, True)
        assert await compliance_api.read_en_pin_status(MOTOR) is True

        await compliance_api.enable_motor(MOTOR, False)
        assert await compliance_api.read_en_pin_status(MOTOR) is False

        await compliance_api.enable_motor(MOTOR, True)

    async def test_the_motor_status_says_it_is_stopped_when_it_is(self, compliance_api):
        """0xF1 is the only way to follow a motor whose responses are switched off."""
        await compliance_api.enable_motor(MOTOR, True)
        await compliance_api.emergency_stop(MOTOR)

        assert await compliance_api.query_motor_status(MOTOR) == const.MOTOR_STATUS_STOPPED

    async def test_the_encoder_reads_agree_with_each_other(self, compliance_api):
        """
        0x30 and 0x31 are two views of one position.

        `carry * 0x4000 + value` is the accumulated count 0x31 reports, so a
        sign or width error in either shows up as a disagreement.
        """
        await compliance_api.set_current_axis_to_zero(MOTOR)

        carry, value = await compliance_api.read_encoder_value_carry(MOTOR)
        addition = await compliance_api.read_encoder_value_addition(MOTOR)

        assert carry * 0x4000 + value == pytest.approx(addition, abs=64)

    async def test_the_shaft_angle_error_is_in_range(self, compliance_api):
        """0x39 is a signed 32-bit count where 51200 is a full turn."""
        error = await compliance_api.read_shaft_angle_error(MOTOR)

        assert isinstance(error, int)
        assert -(2**31) <= error < 2**31

    async def test_the_io_status_decodes_to_the_four_documented_pins(self, compliance_api):
        """0x34's byte carries IN_1, IN_2, OUT_1 and OUT_2 in its low nibble."""
        status = await compliance_api.read_io_status(MOTOR)

        assert {"IN_1", "IN_2", "OUT_1", "OUT_2"} <= set(status)
        assert all(status[pin] in (0, 1) for pin in ("IN_1", "IN_2", "OUT_1", "OUT_2"))
        assert status["raw_byte"] == (
            (status["OUT_2"] << 3) | (status["OUT_1"] << 2)
            | (status["IN_2"] << 1) | status["IN_1"]
        )

    async def test_writing_an_output_port_changes_what_the_read_reports(
        self, compliance_api
    ):
        """0x36 writes; 0x34 is how anyone finds out whether it took."""
        await compliance_api.write_io_port(MOTOR, out1_value=1, out1_mask_action=1)
        assert (await compliance_api.read_io_status(MOTOR))["OUT_1"] == 1

        await compliance_api.write_io_port(MOTOR, out1_value=0, out1_mask_action=1)
        assert (await compliance_api.read_io_status(MOTOR))["OUT_1"] == 0


class TestSpeedModeParameters:
    """0xFF, which L2 found always timed out."""

    async def test_saving_and_clearing_both_answer(self, compliance_api):
        """
        Both argument values, because they are different bytes on the wire.

        L2's unit test asserted the wrong expectation, so the command being
        broken and the test passing were the same event.
        """
        await compliance_api.save_or_clean_speed_mode_params(MOTOR, save=True)
        await compliance_api.save_or_clean_speed_mode_params(MOTOR, save=False)

    async def test_speed_mode_runs_and_stops(self, compliance_api):
        """0xF6 in both of its roles: a run, then a stop that is a run at zero."""
        await compliance_api.enable_motor(MOTOR, True)

        started = await compliance_api.run_speed_mode(
            MOTOR, ccw_direction=False, speed=200, acceleration=2
        )
        assert started in (const.POS_RUN_STARTING, const.POS_RUN_COMPLETE)

        stopped = await compliance_api.stop_speed_mode(MOTOR, acceleration=0)
        assert stopped in (const.POS_RUN_STARTING, const.POS_RUN_COMPLETE)


class TestArgumentsAreCheckedBeforeTheBus:
    """
    An out-of-range argument must be refused, not encoded.

    Every one of these would otherwise be truncated into a byte and sent, and
    the motor would answer `status = 1` to a value nobody meant.
    """

    @pytest.mark.parametrize(
        "call,argument",
        [
            ("set_work_mode", 6),
            ("set_work_mode", -1),
            ("set_subdivision", 256),
            ("set_can_bitrate", 4),
            ("set_holding_current_percentage", 9),
            ("set_group_id", 0),
            ("set_group_id", 0x800),
        ],
    )
    async def test_out_of_range_settings_are_refused(
        self, compliance_api, call, argument
    ):
        """Each is one past a boundary the manual states."""
        with pytest.raises(exceptions.ParameterError):
            await getattr(compliance_api, call)(MOTOR, argument)

    async def test_an_out_of_range_can_id_is_refused(self, compliance_api):
        """0x8B carries 11 bits; 0x800 does not fit in a standard frame."""
        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_can_id(MOTOR, 0x800)

    async def test_an_impossible_working_current_is_refused(self, compliance_api):
        """No MKS servo takes six amps."""
        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_working_current(MOTOR, 6001)

    @pytest.mark.parametrize("speed", [-1, const.MAX_SPEED_PARAM + 1])
    async def test_an_out_of_range_speed_is_refused(self, compliance_api, speed):
        """The speed parameter is 0-3000 whichever motion command carries it."""
        with pytest.raises(exceptions.ParameterError):
            await compliance_api.run_speed_mode(
                MOTOR, ccw_direction=False, speed=speed, acceleration=2
            )


class TestMotionCommandsMoveTheMotor:
    """
    Each of the four motion commands, checked by where the motor ended up.

    The compliance suite proves the frames are well formed and the simulator
    answers `run starting`. What it cannot say is whether the motor went where
    the command said, which is the whole point of a position command - and the
    difference between the relative and absolute pairs is exactly what L18's
    templates got wrong one layer up.
    """

    async def test_an_absolute_axis_move_lands_on_the_commanded_count(
        self, compliance_api
    ):
        """0xF5 takes an absolute encoder count, whatever the motor's position."""
        await compliance_api.enable_motor(MOTOR, True)
        target = const.ENCODER_PULSES_PER_REVOLUTION // 4

        await compliance_api.run_position_mode_absolute_axis(
            MOTOR, speed=600, acceleration=0, absolute_axis=target
        )
        await _settle(compliance_api)

        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            target, abs=64
        )

    async def test_a_relative_axis_move_adds_to_where_the_motor_already_is(
        self, compliance_api
    ):
        """
        0xF4 adds to the current count.

        Commanding it from a non-zero position is what separates it from 0xF5:
        from zero the two are indistinguishable.
        """
        await compliance_api.enable_motor(MOTOR, True)
        start = const.ENCODER_PULSES_PER_REVOLUTION // 4
        step = const.ENCODER_PULSES_PER_REVOLUTION // 8

        await compliance_api.run_position_mode_absolute_axis(
            MOTOR, speed=600, acceleration=0, absolute_axis=start
        )
        await _settle(compliance_api)
        await compliance_api.run_position_mode_relative_axis(
            MOTOR, speed=600, acceleration=0, relative_axis=step
        )
        await _settle(compliance_api)

        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            start + step, abs=64
        )

    async def test_a_negative_relative_move_goes_the_other_way(self, compliance_api):
        """The sign lives in the value for 0xF4, not in a direction flag."""
        await compliance_api.enable_motor(MOTOR, True)
        start = const.ENCODER_PULSES_PER_REVOLUTION // 2
        step = const.ENCODER_PULSES_PER_REVOLUTION // 8

        await compliance_api.run_position_mode_absolute_axis(
            MOTOR, speed=600, acceleration=0, absolute_axis=start
        )
        await _settle(compliance_api)
        await compliance_api.run_position_mode_relative_axis(
            MOTOR, speed=600, acceleration=0, relative_axis=-step
        )
        await _settle(compliance_api)

        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            start - step, abs=64
        )

    async def test_a_pulse_move_counts_in_command_microsteps(self, compliance_api):
        """
        0xFE is quoted in command pulses, not raw encoder counts.

        At 16 microsteps on 200 full steps that is 3200 a revolution against the
        encoder's 16384, so the two units differ by more than a factor of five -
        a confusion between them is a fivefold overshoot, not a rounding error.
        """
        await compliance_api.enable_motor(MOTOR, True)
        await compliance_api.set_subdivision(MOTOR, 16)
        await compliance_api.set_current_axis_to_zero(MOTOR)

        pulses_per_rev = 200 * 16
        await compliance_api.run_position_mode_absolute_pulses(
            MOTOR, speed=600, acceleration=0, absolute_pulses=pulses_per_rev // 4
        )
        await _settle(compliance_api)

        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            const.ENCODER_PULSES_PER_REVOLUTION // 4, abs=64
        )


class TestStoppingAMoveInFlight:
    """
    The stop form of each motion command: the same code, at speed zero.

    Nothing had sent any of them. A stop that is accepted and does not stop is
    the worst version of this repository's recurring defect, because the caller
    has already decided the machine is safe to approach.
    """

    @pytest.mark.parametrize(
        "start,stop",
        [
            ("run_position_mode_relative_axis", "stop_position_mode_relative_axis"),
            ("run_position_mode_absolute_axis", "stop_position_mode_absolute_axis"),
        ],
    )
    async def test_a_long_move_can_be_stopped(self, compliance_api, start, stop):
        """Command a move of several turns, stop it, and check it stayed stopped."""
        await compliance_api.enable_motor(MOTOR, True)
        await compliance_api.set_current_axis_to_zero(MOTOR)

        far = const.ENCODER_PULSES_PER_REVOLUTION * 4
        argument = (
            "relative_axis" if "relative" in start else "absolute_axis"
        )
        await getattr(compliance_api, start)(
            MOTOR, speed=100, acceleration=0, **{argument: far}
        )
        await asyncio.sleep(0.2)

        await getattr(compliance_api, stop)(MOTOR, acceleration=0)
        await asyncio.sleep(0.3)

        first = await compliance_api.read_encoder_value_addition(MOTOR)
        await asyncio.sleep(0.3)
        second = await compliance_api.read_encoder_value_addition(MOTOR)

        assert second == pytest.approx(first, abs=64), "the motor kept moving after a stop"
        assert abs(second) < far, "the move ran to completion despite the stop"

    @pytest.mark.parametrize(
        "call",
        [
            "stop_position_mode_relative_axis",
            "stop_position_mode_absolute_axis",
            "stop_speed_mode",
        ],
    )
    async def test_an_out_of_range_deceleration_is_refused(self, compliance_api, call):
        """The acceleration byte is 0-255 in every stop command that carries one."""
        with pytest.raises(exceptions.ParameterError):
            await getattr(compliance_api, call)(MOTOR, acceleration=256)


class TestFireAndForgetVariants:
    """
    The `_no_wait` commands the streaming API is built on.

    They send and return without waiting for an acknowledgement, so the only
    thing that can show they worked is the motor moving.
    """

    async def test_an_unacknowledged_absolute_move_still_moves(self, compliance_api):
        """`run_position_mode_absolute_axis_no_wait` returns before the motor answers."""
        await compliance_api.enable_motor(MOTOR, True)
        await compliance_api.set_current_axis_to_zero(MOTOR)
        target = const.ENCODER_PULSES_PER_REVOLUTION // 4

        await compliance_api.run_position_mode_absolute_axis_no_wait(
            MOTOR, speed=600, acceleration=0, absolute_axis=target
        )
        await _settle(compliance_api)

        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            target, abs=64
        )

    async def test_an_unacknowledged_stop_halts_a_move_in_flight(self, compliance_api):
        """A stop that is never acknowledged still has to stop the motor."""
        await compliance_api.enable_motor(MOTOR, True)
        await compliance_api.set_current_axis_to_zero(MOTOR)

        await compliance_api.run_position_mode_absolute_axis_no_wait(
            MOTOR,
            speed=100,
            acceleration=0,
            absolute_axis=const.ENCODER_PULSES_PER_REVOLUTION * 4,
        )
        await asyncio.sleep(0.2)
        await compliance_api.stop_position_mode_absolute_axis_no_wait(
            MOTOR, acceleration=0
        )
        await asyncio.sleep(0.3)

        first = await compliance_api.read_encoder_value_addition(MOTOR)
        await asyncio.sleep(0.3)
        assert await compliance_api.read_encoder_value_addition(MOTOR) == pytest.approx(
            first, abs=64
        ), "the motor was still moving after an unacknowledged stop"

    @pytest.mark.parametrize(
        "call,arguments",
        [
            ("run_speed_mode_no_wait", {"ccw_direction": False, "speed": 3001, "acceleration": 2}),
            ("run_speed_mode_no_wait", {"ccw_direction": False, "speed": 100, "acceleration": 256}),
            (
                "run_position_mode_absolute_axis_no_wait",
                {"speed": 3001, "acceleration": 2, "absolute_axis": 0},
            ),
            (
                "run_position_mode_absolute_axis_no_wait",
                {"speed": 100, "acceleration": 2, "absolute_axis": 8388608},
            ),
            ("stop_position_mode_absolute_axis_no_wait", {"acceleration": 256}),
        ],
    )
    async def test_the_unacknowledged_commands_still_check_their_arguments(
        self, compliance_api, call, arguments
    ):
        """
        These send without waiting, so nothing downstream can reject a bad value.

        A frame that leaves with a truncated speed is simply obeyed, and there
        is no acknowledgement to notice it in.
        """
        with pytest.raises(exceptions.ParameterError):
            await getattr(compliance_api, call)(MOTOR, **arguments)

    async def test_an_unacknowledged_speed_command_still_turns_the_motor(
        self, compliance_api
    ):
        """`run_speed_mode_no_wait` is the velocity-streaming path."""
        await compliance_api.enable_motor(MOTOR, True)
        before = await compliance_api.read_encoder_value_addition(MOTOR)

        await compliance_api.run_speed_mode_no_wait(
            MOTOR, ccw_direction=False, speed=300, acceleration=0
        )
        await asyncio.sleep(0.4)
        after = await compliance_api.read_encoder_value_addition(MOTOR)
        await compliance_api.emergency_stop(MOTOR)

        assert after != before, "the motor did not turn"


class TestConfigurationWithNoReadBack:
    """
    Setters the motor stores but will not read back.

    There is nothing to assert but the acknowledgement, so these check the two
    things that are still checkable: that a legal call is accepted, and that an
    illegal one never reaches the bus.
    """

    async def test_the_homing_parameters_are_accepted(self, compliance_api):
        """0x90 carries five fields in eight bytes."""
        await compliance_api.set_home_parameters(
            MOTOR,
            home_trig_level=0,
            home_dir=const.DIR_CW,
            home_speed_rpm=60,
            end_limit_enabled=False,
            home_mode=0,
        )

    @pytest.mark.parametrize(
        "field,value",
        [
            ("home_trig_level", 2),
            ("home_dir", 2),
            ("home_speed_rpm", 3001),
            ("home_mode", 2),
        ],
    )
    async def test_bad_homing_parameters_are_refused(
        self, compliance_api, field, value
    ):
        """Each field one past its documented range."""
        arguments = {
            "home_trig_level": 0,
            "home_dir": const.DIR_CW,
            "home_speed_rpm": 60,
            "end_limit_enabled": False,
            "home_mode": 0,
        }
        arguments[field] = value

        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_home_parameters(MOTOR, **arguments)

    async def test_the_no_limit_homing_parameters_are_accepted(self, compliance_api):
        """0x94: a uint32 return angle and a uint16 current."""
        await compliance_api.set_nolimit_home_params(
            MOTOR, reverse_angle_pulses=0x2000, home_current_ma=800
        )

        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_nolimit_home_params(
                MOTOR, reverse_angle_pulses=0x1_0000_0000, home_current_ma=800
            )

    async def test_the_zero_mode_parameters_are_accepted(self, compliance_api):
        """0x9A: mode, action, speed code and direction."""
        await compliance_api.set_zero_mode_parameters(
            MOTOR, mode=0, set_zero_action=2, speed_code=0, direction_code=const.DIR_CW
        )

        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_zero_mode_parameters(
                MOTOR, mode=0, set_zero_action=2, speed_code=5, direction_code=0
            )

    async def test_the_error_protection_parameters_are_accepted(self, compliance_api):
        """0x9D: two flag bits and two uint16 thresholds."""
        await compliance_api.set_en_trigger_and_pos_error_protection(
            MOTOR,
            enable_en_trigger_zero=False,
            enable_pos_error_protection=True,
            error_detection_time_ms_units=100,
            error_threshold_pulses=28000,
        )

        with pytest.raises(exceptions.ParameterError):
            await compliance_api.set_en_trigger_and_pos_error_protection(
                MOTOR,
                enable_en_trigger_zero=False,
                enable_pos_error_protection=True,
                error_detection_time_ms_units=0x1_0000,
                error_threshold_pulses=28000,
            )

    async def test_the_limit_port_remap_is_accepted_both_ways(self, compliance_api):
        """0x9E, set on and off again."""
        await compliance_api.set_limit_port_remap(MOTOR, enable_remap=True)
        await compliance_api.set_limit_port_remap(MOTOR, enable_remap=False)


class TestAbsentMotor:
    """
    A command to an address nothing answers on must fail, not return.

    The simulator serves CAN IDs 1 to 3, so 99 is a silence no stub had to be
    written to produce.
    """

    @pytest.mark.parametrize(
        "call",
        [
            "read_encoder_value_carry",
            "read_pulses_received",
            "read_shaft_angle_error",
            "read_en_pin_status",
            "query_motor_status",
            "restore_default_parameters",
        ],
    )
    async def test_reads_from_an_absent_motor_time_out(self, compliance_api, call):
        """Each must raise rather than hand back a default."""
        with pytest.raises(exceptions.CommunicationError):
            await getattr(compliance_api, call)(ABSENT_CAN_ID)

    async def test_a_setting_written_to_an_absent_motor_raises(self, compliance_api):
        """Silently accepting a write nobody received is the house defect."""
        with pytest.raises(exceptions.CommunicationError):
            await compliance_api.set_working_current(ABSENT_CAN_ID, 1000)
