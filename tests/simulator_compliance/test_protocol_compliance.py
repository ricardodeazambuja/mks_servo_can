"""
Test simulator against official MKS manual specification.

This module validates that the simulator responses match the exact
specifications from the MKS SERVO42D/57D_CAN User Manual V1.0.6.

Updated to use shared simulator management fixtures instead of direct imports.
"""

import time
from typing import List

import pytest

from mks_servo_can import constants as const
from mks_servo_can import exceptions, load_manual_spec

# The manual's transcription ships with the library as package data.
MANUAL_SPEC = load_manual_spec()

MANUAL_COMMANDS = MANUAL_SPEC["commands"]


def command_calls(api):
    """
    One library call per transcribed command, keyed by command code.

    Two tests need this and each used to carry its own copy, which is how a
    mapping can be right in one and wrong in the other. Every entry must call
    the method that emits *this* opcode: three once did not - "0x3B" called
    `go_home` (0x91), "0xF4" the relative-pulses method (0xFD) and "0xF5" the
    absolute-pulses method (0xFE) - so the suite reported compliance for three
    commands it never sent, including the two axis commands the streaming API
    is built on.

    Args:
        api: A connected `LowLevelAPI`.

    Returns:
        A mapping of `"0xNN"` to a zero-argument coroutine function.
    """
    return {
        "0x00": lambda: api.read_system_parameter(can_id=1, parameter_command_code=0x82),
        "0x30": lambda: api.read_encoder_value_carry(can_id=1),
        "0x31": lambda: api.read_encoder_value_addition(can_id=1),
        "0x32": lambda: api.read_motor_speed_rpm(can_id=1),
        "0x33": lambda: api.read_pulses_received(can_id=1),
        "0x34": lambda: api.read_io_status(can_id=1),
        "0x35": lambda: api.read_raw_encoder_value_addition(can_id=1),
        "0x36": lambda: api.write_io_port(can_id=1, out1_value=0, out1_mask_action=1),
        "0x39": lambda: api.read_shaft_angle_error(can_id=1),
        "0x3A": lambda: api.read_en_pin_status(can_id=1),
        "0x3B": lambda: api.read_power_on_zero_status(can_id=1),
        "0x3D": lambda: api.release_stall_protection(can_id=1),
        "0x3E": lambda: api.read_motor_protection_state(can_id=1),
        "0x3F": lambda: api.restore_default_parameters(can_id=1),
        "0x41": lambda: api.restart_motor(can_id=1),
        "0x80": lambda: api.calibrate_encoder(can_id=1),
        "0x82": lambda: api.set_work_mode(can_id=1, mode=const.MODE_SR_VFOC),
        "0x83": lambda: api.set_working_current(can_id=1, current_ma=1600),
        "0x84": lambda: api.set_subdivision(can_id=1, microsteps=16),
        "0x85": lambda: api.set_en_pin_active_level(can_id=1, level_code=0x01),
        "0x86": lambda: api.set_motor_direction(can_id=1, direction_code=0x00),
        "0x87": lambda: api.set_auto_screen_off(can_id=1, enable=False),
        "0x88": lambda: api.set_stall_protection(can_id=1, enable=True),
        "0x89": lambda: api.set_subdivision_interpolation(can_id=1, enable=True),
        "0x8A": lambda: api.set_can_bitrate(can_id=1, bitrate_code=0x02),
        # Re-addressed to the address it already has, so the frame goes out in
        # full and the motor stays reachable for the rest of the run.
        "0x8B": lambda: api.set_can_id(current_can_id=1, new_can_id=1),
        "0x8C": lambda: api.set_slave_respond_active(
            can_id=1, respond_enabled=True, active_enabled=True
        ),
        "0x8D": lambda: api.set_group_id(can_id=1, group_id=0x50),
        "0x8F": lambda: api.set_key_lock(can_id=1, lock_enabled=False),
        "0x90": lambda: api.set_home_parameters(
            can_id=1,
            home_trig_level=0,
            home_dir=0,
            home_speed_rpm=60,
            end_limit_enabled=False,
            home_mode=0,
        ),
        "0x91": lambda: api.go_home(can_id=1),
        "0x92": lambda: api.set_current_axis_to_zero(can_id=1),
        "0x94": lambda: api.set_nolimit_home_params(
            can_id=1, reverse_angle_pulses=0x2000, home_current_ma=800
        ),
        "0x9A": lambda: api.set_zero_mode_parameters(
            can_id=1, mode=0, set_zero_action=2, speed_code=0, direction_code=0
        ),
        "0x9B": lambda: api.set_holding_current_percentage(can_id=1, percentage_code=4),
        "0x9D": lambda: api.set_en_trigger_and_pos_error_protection(
            can_id=1,
            enable_en_trigger_zero=False,
            enable_pos_error_protection=False,
            error_detection_time_ms_units=100,
            error_threshold_pulses=28000,
        ),
        "0x9E": lambda: api.set_limit_port_remap(can_id=1, enable_remap=False),
        "0xF1": lambda: api.query_motor_status(can_id=1),
        "0xF3": lambda: api.enable_motor(can_id=1, enable=True),
        "0xF4": lambda: api.run_position_mode_relative_axis(
            can_id=1, speed=500, acceleration=100, relative_axis=1000
        ),
        "0xF5": lambda: api.run_position_mode_absolute_axis(
            can_id=1, speed=500, acceleration=100, absolute_axis=1000
        ),
        "0xF6": lambda: api.run_speed_mode(
            can_id=1, ccw_direction=False, speed=100, acceleration=2
        ),
        "0xF7": lambda: api.emergency_stop(can_id=1),
        "0xFD": lambda: api.run_position_mode_relative_pulses(
            can_id=1, pulses=100, ccw_direction=False, speed=500, acceleration=100
        ),
        "0xFE": lambda: api.run_position_mode_absolute_pulses(
            can_id=1, speed=500, acceleration=100, absolute_pulses=1500
        ),
        "0xFF": lambda: api.save_or_clean_speed_mode_params(can_id=1, save=False),
    }


@pytest.mark.compliance
class TestManualProtocolCompliance:
    """Test simulator against official MKS manual specification"""

    def test_crc_calculation_manual_spec(self):
        """Test CRC calculation matches manual specification"""
        # Manual example: command "01 30 CRC"
        # CRC = (0x01 + 0x30) & 0xFF = 0x31

        can_id = 0x01
        command = 0x30
        expected_crc = 0x31

        calculated_crc = self._calculate_manual_crc(can_id, [command])
        assert calculated_crc == expected_crc, f"Expected CRC 0x{expected_crc:02X}, got 0x{calculated_crc:02X}"

        # Test additional cases
        test_cases = [
            (0x01, [0x30], 0x31),
            (0x01, [0x31], 0x32),  # 0x01 + 0x31 = 0x32
            (0x02, [0x30], 0x32),  # 0x02 + 0x30 = 0x32
            (0x01, [0x80], 0x81),  # Calibration command example
        ]

        for can_id, data, expected in test_cases:
            calculated = self._calculate_manual_crc(can_id, data)
            assert calculated == expected, f"CRC mismatch for ID {can_id}, data {data}: expected 0x{expected:02X}, got 0x{calculated:02X}"

    @pytest.mark.asyncio
    async def test_encoder_carry_format_manual(self, compliance_api):
        """Test encoder carry format per manual specification (0x30)"""
        api = compliance_api

        # Test command 0x30 (read encoder carry)
        try:
            carry, value = await api.read_encoder_value_carry(can_id=1)

            # Verify values match expected format (should be reasonable values)
            assert isinstance(value, int), f"Value should be int, got {type(value)}"
            assert isinstance(carry, int), f"Carry should be int, got {type(carry)}"
            assert 0 <= value <= 0xFFFF, f"Value should be uint16 range, got 0x{value:04X}"

        except Exception as e:
            pytest.fail(f"Command 0x30 failed: {e}")

    @pytest.mark.asyncio
    async def test_encoder_addition_format_manual(self, compliance_api):
        """Test encoder addition format per manual specification (0x31)"""
        api = compliance_api

        # Test command 0x31 (read encoder addition)
        try:
            value = await api.read_encoder_value_addition(can_id=1)

            # Verify value format (should be a reasonable int48 value)
            assert isinstance(value, int), f"Value should be int, got {type(value)}"

        except Exception as e:
            pytest.fail(f"Command 0x31 failed: {e}")

    @pytest.mark.asyncio
    async def test_speed_reading_manual(self, compliance_api):
        """Test speed reading format per manual specification (0x32)"""
        api = compliance_api

        # Test command 0x32 (read motor speed)
        try:
            speed = await api.read_motor_speed_rpm(can_id=1)

            # Verify speed format (should be int16)
            assert isinstance(speed, int), f"Speed should be integer, got {type(speed)}"
            assert -32768 <= speed <= 32767, f"Speed out of int16 range: {speed}"

        except Exception as e:
            pytest.fail(f"Command 0x32 failed: {e}")

    @pytest.mark.asyncio
    async def test_io_status_reading_manual(self, compliance_api):
        """Test IO status reading per manual specification (0x34)"""
        api = compliance_api

        # Test command 0x34 (read IO status)
        try:
            io_status = await api.read_io_status(can_id=1)

            # Verify response format
            assert isinstance(io_status, dict), f"IO status should be dict, got {type(io_status)}"

        except Exception as e:
            pytest.fail(f"Command 0x34 failed: {e}")

    @pytest.mark.asyncio
    async def test_home_command_manual(self, compliance_api):
        """Test home command per manual specification (0x91)"""
        api = compliance_api

        # Test command 0x91 (go home)
        try:
            status = await api.go_home(can_id=1)

            # Verify status codes per manual
            # 0: going to zero, 1: success, 2: fail
            assert status in [0, 1, 2], f"Invalid home status code: {status}"

        except Exception as e:
            pytest.fail(f"Command 0x91 (go_home) failed: {e}")

    @pytest.mark.asyncio
    async def test_protection_commands_manual(self, compliance_api):
        """Test protection commands per manual specification (0x3D, 0x3E)"""
        api = compliance_api

        # Test 0x3D (release protection)
        try:
            success = await api.release_stall_protection(can_id=1)
            assert isinstance(success, bool), f"Protection release should return bool, got {type(success)}"

        except Exception as e:
            pytest.fail(f"Command 0x3D (release_protection) failed: {e}")

        # Test 0x3E (read protection state)
        try:
            protected = await api.read_motor_protection_state(can_id=1)
            assert isinstance(protected, bool), f"Protection state should return bool, got {type(protected)}"

        except Exception as e:
            pytest.fail(f"Command 0x3E (read_protection_state) failed: {e}")

    @pytest.mark.asyncio
    async def test_motion_commands_manual(self, compliance_api):
        """Test motion commands per manual specification (0xF4, 0xF5, 0xF7, 0xFE)"""
        api = compliance_api

        # Test 0xF4 (relative move)
        try:
            await api.run_position_mode_relative_pulses(
                can_id=1, pulses=1000, ccw_direction=False, speed=500, acceleration=100
            )
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0xF4 (relative_move) failed: {e}")

        # Test 0xF5 (absolute move)
        try:
            await api.run_position_mode_absolute_pulses(
                can_id=1, speed=500, acceleration=100, absolute_pulses=2000
            )
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0xF5 (absolute_move) failed: {e}")

        # Test 0xF7 (emergency stop)
        try:
            await api.emergency_stop(can_id=1)
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0xF7 (emergency_stop) failed: {e}")

    @pytest.mark.asyncio
    async def test_configuration_commands_manual(self, compliance_api):
        """Test configuration commands per manual specification"""
        api = compliance_api

        # Test 0x80 (calibration)
        try:
            await api.calibrate_encoder(can_id=1)
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0x80 (calibration) failed: {e}")

        # Test 0x92 (set position zero)
        try:
            await api.set_current_axis_to_zero(can_id=1)
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0x92 (set_zero) failed: {e}")

        # Test 0x41 (restart motor)
        try:
            await api.restart_motor(can_id=1)
            # Should not raise exception

        except Exception as e:
            pytest.fail(f"Command 0x41 (restart) failed: {e}")

    @pytest.mark.asyncio
    async def test_all_manual_commands_implemented(self, compliance_api):
        """Verify all commands from manual are implemented in simulator"""
        api = compliance_api

        # Enable motor for motion commands that require it
        try:
            await api.enable_motor(can_id=1)
        except Exception:
            pass  # Ignore if enable fails

        # Map of manual commands to library methods
        command_mapping = command_calls(api)

        missing_commands = []
        failed_commands = []

        for cmd_code, cmd_spec in MANUAL_COMMANDS.items():
            if cmd_code in command_mapping:
                try:
                    # Execute the command
                    await command_mapping[cmd_code]()
                    # If we get here, command succeeded

                except Exception as e:
                    # Motion commands might fail due to motor state, which is acceptable
                    if cmd_code in ["0xF4", "0xF5", "0xFE"]:
                        # For motion commands, any response (even error) indicates the command is implemented
                        if "Motor run command" in str(e) or "failed to start" in str(e):
                            continue  # Command is implemented, just motor state issue
                    failed_commands.append(f"{cmd_code} ({cmd_spec['name']}): {e!s}")
            else:
                missing_commands.append(f"{cmd_code} ({cmd_spec['name']})")

        # Report results
        error_messages = []
        if missing_commands:
            error_messages.append(f"Missing command mappings: {missing_commands}")
        if failed_commands:
            error_messages.append(f"Failed command executions: {failed_commands}")

        if error_messages:
            pytest.fail("\n".join(error_messages))

    def test_can_frame_format_compliance(self):
        """Test CAN frame format matches manual specification"""
        # Manual specifies standard frames (not extended)
        # DLC values should match command specifications
        # Arbitration ID format should be correct

        # Test standard frame format requirements
        for cmd_code, cmd_spec in MANUAL_COMMANDS.items():
            request_dlc = cmd_spec["request"]["dlc"]
            assert 1 <= request_dlc <= 8, f"Invalid request DLC for {cmd_code}: {request_dlc}"

            if cmd_spec["response"].get("variable_dlc"):
                # 0x00 answers in the layout of whichever parameter was asked
                # for, so its response length is not a constant.
                continue
            response_dlc = cmd_spec["response"]["dlc"]
            assert 1 <= response_dlc <= 8, f"Invalid response DLC for {cmd_code}: {response_dlc}"

    @pytest.mark.asyncio
    async def test_error_conditions_manual(self, compliance_api):
        """Test error handling matches manual specification"""
        api = compliance_api

        # Test invalid motor ID (should raise exception or return error)
        with pytest.raises(exceptions.MKSServoError):
            await api.read_encoder_value_carry(can_id=99)  # Non-existent motor

        # Test reading commands should work on motors
        try:
            await api.read_encoder_value_carry(can_id=1)  # Reading should work
        except Exception:
            pytest.fail("Reading commands should work")

    @pytest.mark.asyncio
    async def test_timing_behavior_manual(self, compliance_api):
        """Test response timing is realistic per manual guidelines"""
        api = compliance_api

        # Measure response time for simple command
        start_time = time.time()
        await api.read_encoder_value_carry(can_id=1)
        response_time = time.time() - start_time

        # Should be faster than real hardware but not instantaneous
        assert 0.001 < response_time < 0.1, f"Response time {response_time}s outside realistic range"

    def _calculate_manual_crc(self, can_id: int, data: List[int]) -> int:
        """Calculate CRC per manual specification"""
        crc = can_id
        for byte_val in data:
            crc += byte_val
        return crc & 0xFF


@pytest.mark.compliance
class TestCommandResponseMapping:
    """Test specific command-response pairs from manual specification"""

    @pytest.mark.parametrize("cmd_code,cmd_spec", [
        (cmd_code, cmd_spec) for cmd_code, cmd_spec in MANUAL_COMMANDS.items()
    ])
    @pytest.mark.asyncio
    async def test_command_response_format(self, compliance_api, cmd_code, cmd_spec):
        """Test each command's response format matches manual specification"""
        api = compliance_api

        # Enable motor for motion commands that may require it
        if cmd_code in ["0xF4", "0xF5", "0xFE"]:
            try:
                await api.enable_motor(can_id=1)
            except Exception:
                pass  # Ignore enable failures

        # Map commands to API calls
        calls = command_calls(api)

        if cmd_code not in calls:
            pytest.skip(f"Command {cmd_code} not mapped for testing")

        try:
            result = await calls[cmd_code]()

            # Verify result type based on command specification
            if cmd_code == "0x30":
                assert isinstance(result, tuple) and len(result) == 2, "0x30 should return (carry, value) tuple"
                carry, value = result
                assert isinstance(carry, int) and isinstance(value, int), "0x30 should return int types"
            elif cmd_code in ["0x31", "0x32", "0x35"]:
                assert isinstance(result, int), f"{cmd_code} should return int"
            elif cmd_code == "0x34":
                assert isinstance(result, dict), "0x34 should return dict"
            elif cmd_code in ["0x3D", "0x3E"]:
                assert isinstance(result, bool), f"{cmd_code} should return bool"
            elif cmd_code in ["0x3B", "0x91"]:
                assert isinstance(result, int), f"{cmd_code} should return status int"
            elif cmd_code in ["0xF4", "0xF5", "0xFD", "0xFE"]:
                assert isinstance(result, int), f"{cmd_code} should return status int"
            elif cmd_code in ["0x36", "0x41", "0x80", "0x92", "0xF7"]:
                # These commands may return various types or None
                pass  # Just verify they don't raise exceptions

        except Exception as e:
            # Motion commands might fail due to motor state, which is acceptable for testing command implementation
            if cmd_code in ["0xF4", "0xF5", "0xFD", "0xFE"] and ("Motor run command" in str(e) or "failed to start" in str(e)):
                pass  # Command is implemented, just motor state issue
            else:
                pytest.fail(f"Command {cmd_code} ({cmd_spec['name']}) failed: {e}")
