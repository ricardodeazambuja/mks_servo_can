"""
Test simulator against official MKS manual specification.

This module validates that the simulator responses match the exact
specifications from the MKS SERVO42D/57D_CAN User Manual V1.0.6.
"""

import pytest
import asyncio
import sys
import json
import struct
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional

# Add library and simulator to path
test_dir = Path(__file__).parent.parent.parent
lib_path = test_dir / "mks_servo_can_library"
sim_path = test_dir / "mks_servo_simulator"
sys.path.extend([str(lib_path), str(sim_path)])

from mks_servo_can import CANInterface, LowLevelAPI, const


# Load manual command specifications
FIXTURES_DIR = Path(__file__).parent.parent / "fixtures"
with open(FIXTURES_DIR / "manual_commands_v106.json") as f:
    MANUAL_SPEC = json.load(f)

MANUAL_COMMANDS = MANUAL_SPEC["commands"]


class TestManualProtocolCompliance:
    """Test simulator against official MKS manual specification"""
    
    @pytest.fixture
    async def simulator_setup(self):
        """Setup simulator with known motor configuration"""
        # Import simulator components
        try:
            from mks_simulator.virtual_can_bus import VirtualCANBus
            from mks_simulator.motor_model import MotorModel
        except ImportError:
            pytest.skip("Simulator not available for testing")
        
        # Create motors per standard test configuration
        motors = {
            1: MotorModel(can_id=1),
            2: MotorModel(can_id=2),
            3: MotorModel(can_id=3)
        }
        
        # Configure simulator
        can_bus = VirtualCANBus(motors, latency_ms=1)
        
        # Start simulator in background
        sim_task = asyncio.create_task(can_bus.start())
        await asyncio.sleep(0.1)  # Let simulator start
        
        # Connect library
        can_if = CANInterface(use_simulator=True)
        await can_if.connect()
        api = LowLevelAPI(can_if)
        
        yield api, motors, can_bus
        
        # Cleanup
        await can_if.disconnect()
        can_bus.stop()
        try:
            sim_task.cancel()
            await sim_task
        except asyncio.CancelledError:
            pass
    
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
    
    async def test_encoder_carry_format_manual(self, simulator_setup):
        """Test encoder carry format per manual specification (0x30)"""
        api, motors, can_bus = simulator_setup
        
        # Set specific encoder state per manual example
        motor = motors[1]
        motor.encoder_position = 0x3FF0  # Example from manual
        if hasattr(motor, 'encoder_carry'):
            motor.encoder_carry = 0
        
        # Test command 0x30 (read encoder carry)
        try:
            carry, value = await api.read_encoder_value_carry(can_id=1)
            
            # Verify values match expected format
            assert value == 0x3FF0, f"Expected value 0x3FF0, got 0x{value:04X}"
            assert carry == 0, f"Expected carry 0, got {carry}"
            
        except Exception as e:
            pytest.fail(f"Command 0x30 failed: {e}")
    
    async def test_encoder_addition_format_manual(self, simulator_setup):
        """Test encoder addition format per manual specification (0x31)"""
        api, motors, can_bus = simulator_setup
        
        motor = motors[1]
        motor.encoder_position = 0x3FF0
        
        # Test command 0x31 (read encoder addition)
        try:
            value = await api.read_encoder_value_addition(can_id=1)
            
            # Verify value matches expected
            assert value == 0x3FF0, f"Expected value 0x3FF0, got 0x{value:04X}"
            
        except Exception as e:
            pytest.fail(f"Command 0x31 failed: {e}")
    
    async def test_speed_reading_manual(self, simulator_setup):
        """Test speed reading format per manual specification (0x32)"""
        api, motors, can_bus = simulator_setup
        
        motor = motors[1]
        motor.current_speed = 120  # 120 RPM
        
        # Test command 0x32 (read motor speed)
        try:
            speed = await api.read_motor_speed_rpm(can_id=1)
            
            # Verify speed format (should be int16)
            assert isinstance(speed, int), f"Speed should be integer, got {type(speed)}"
            assert -32768 <= speed <= 32767, f"Speed out of int16 range: {speed}"
            assert speed == 120, f"Expected speed 120 RPM, got {speed}"
            
        except Exception as e:
            pytest.fail(f"Command 0x32 failed: {e}")
    
    async def test_io_status_reading_manual(self, simulator_setup):
        """Test IO status reading per manual specification (0x34)"""
        api, motors, can_bus = simulator_setup
        
        # Test command 0x34 (read IO status)
        try:
            io_status = await api.read_io_status(can_id=1)
            
            # Verify response format
            assert isinstance(io_status, dict), f"IO status should be dict, got {type(io_status)}"
            
        except Exception as e:
            pytest.fail(f"Command 0x34 failed: {e}")
    
    async def test_home_command_manual(self, simulator_setup):
        """Test home command per manual specification (0x3B)"""
        api, motors, can_bus = simulator_setup
        
        # Test command 0x3B (go home)
        try:
            status = await api.go_home(can_id=1)
            
            # Verify status codes per manual
            # 0: going to zero, 1: success, 2: fail
            assert status in [0, 1, 2], f"Invalid home status code: {status}"
            
        except Exception as e:
            pytest.fail(f"Command 0x3B (go_home) failed: {e}")
    
    async def test_protection_commands_manual(self, simulator_setup):
        """Test protection commands per manual specification (0x3D, 0x3E)"""
        api, motors, can_bus = simulator_setup
        
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
    
    async def test_motion_commands_manual(self, simulator_setup):
        """Test motion commands per manual specification (0xF4, 0xF5, 0xF7, 0xFE)"""
        api, motors, can_bus = simulator_setup
        
        motor = motors[1]
        motor.enabled = True
        
        # Test 0xF4 (relative move)
        try:
            await api.run_position_mode_relative_pulses(
                can_id=1, pulses=1000, speed=500
            )
            # Should not raise exception
            
        except Exception as e:
            pytest.fail(f"Command 0xF4 (relative_move) failed: {e}")
        
        # Test 0xF5 (absolute move) 
        try:
            await api.run_position_mode_absolute_pulses(
                can_id=1, position=2000, speed=500
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
    
    async def test_configuration_commands_manual(self, simulator_setup):
        """Test configuration commands per manual specification"""
        api, motors, can_bus = simulator_setup
        
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
    
    async def test_all_manual_commands_implemented(self, simulator_setup):
        """Verify all commands from manual are implemented in simulator"""
        api, motors, can_bus = simulator_setup
        
        # Map of manual commands to library methods
        command_mapping = {
            "0x30": lambda: api.read_encoder_value_carry(can_id=1),
            "0x31": lambda: api.read_encoder_value_addition(can_id=1),
            "0x32": lambda: api.read_motor_speed_rpm(can_id=1),
            "0x34": lambda: api.read_io_status(can_id=1),
            "0x35": lambda: api.read_raw_encoder_value_addition(can_id=1),
            "0x36": lambda: api.write_io_port(can_id=1, port_selection=1, port_value=0),
            "0x3B": lambda: api.go_home(can_id=1),
            "0x3D": lambda: api.release_stall_protection(can_id=1),
            "0x3E": lambda: api.read_motor_protection_state(can_id=1),
            "0x41": lambda: api.restart_motor(can_id=1),
            "0x80": lambda: api.calibrate_encoder(can_id=1),
            "0x92": lambda: api.set_current_axis_to_zero(can_id=1),
            "0xF4": lambda: api.run_position_mode_relative_pulses(can_id=1, pulses=100, speed=500),
            "0xF5": lambda: api.run_position_mode_absolute_pulses(can_id=1, position=1000, speed=500),
            "0xF7": lambda: api.emergency_stop(can_id=1),
            "0xFE": lambda: api.run_position_mode_absolute_pulses(can_id=1, position=1500, speed=500),
        }
        
        missing_commands = []
        failed_commands = []
        
        for cmd_code, cmd_spec in MANUAL_COMMANDS.items():
            if cmd_code in command_mapping:
                try:
                    # Execute the command
                    await command_mapping[cmd_code]()
                    # If we get here, command succeeded
                    
                except Exception as e:
                    failed_commands.append(f"{cmd_code} ({cmd_spec['name']}): {str(e)}")
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
            response_dlc = cmd_spec["response"]["dlc"]
            
            # Verify DLC values are valid (1-8)
            assert 1 <= request_dlc <= 8, f"Invalid request DLC for {cmd_code}: {request_dlc}"
            assert 1 <= response_dlc <= 8, f"Invalid response DLC for {cmd_code}: {response_dlc}"
    
    async def test_error_conditions_manual(self, simulator_setup):
        """Test error handling matches manual specification"""
        api, motors, can_bus = simulator_setup
        
        # Test invalid motor ID (should raise exception or return error)
        with pytest.raises(Exception):
            await api.read_encoder_value_carry(can_id=99)  # Non-existent motor
        
        # Test command to disabled motor  
        motor = motors[1]
        motor.enabled = False
        
        # Some commands should handle disabled motors gracefully
        try:
            await api.read_encoder_value_carry(can_id=1)  # Reading should work
        except Exception:
            pytest.fail("Reading commands should work on disabled motors")
    
    async def test_timing_behavior_manual(self, simulator_setup):
        """Test response timing is realistic per manual guidelines"""
        api, motors, can_bus = simulator_setup
        
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


class TestCommandResponseMapping:
    """Test specific command-response pairs from manual specification"""
    
    @pytest.fixture
    async def simulator_setup(self):
        """Reuse simulator setup"""
        # Same setup as above
        try:
            from mks_simulator.virtual_can_bus import VirtualCANBus
            from mks_simulator.motor_model import MotorModel
        except ImportError:
            pytest.skip("Simulator not available for testing")
        
        motors = {1: MotorModel(can_id=1), 2: MotorModel(can_id=2)}
        can_bus = VirtualCANBus(motors, latency_ms=1)
        
        sim_task = asyncio.create_task(can_bus.start())
        await asyncio.sleep(0.1)
        
        can_if = CANInterface(use_simulator=True)
        await can_if.connect()
        api = LowLevelAPI(can_if)
        
        yield api, motors, can_bus
        
        await can_if.disconnect()
        can_bus.stop()
        try:
            sim_task.cancel()
            await sim_task
        except asyncio.CancelledError:
            pass
    
    @pytest.mark.parametrize("cmd_code,cmd_spec", [
        (cmd_code, cmd_spec) for cmd_code, cmd_spec in MANUAL_COMMANDS.items()
    ])
    async def test_command_response_format(self, simulator_setup, cmd_code, cmd_spec):
        """Test each command's response format matches manual specification"""
        api, motors, can_bus = simulator_setup
        
        # Skip commands that require specific parameters for now
        if cmd_code in ["0x36", "0xF4", "0xF5", "0xFE"]:
            pytest.skip(f"Command {cmd_code} requires specific parameter testing")
        
        # Map commands to API calls
        command_calls = {
            "0x30": lambda: api.read_encoder_value_carry(can_id=1),
            "0x31": lambda: api.read_encoder_value_addition(can_id=1),
            "0x32": lambda: api.read_motor_speed_rpm(can_id=1),
            "0x34": lambda: api.read_io_status(can_id=1),
            "0x35": lambda: api.read_raw_encoder_value_addition(can_id=1),
            "0x3B": lambda: api.go_home(can_id=1),
            "0x3D": lambda: api.release_stall_protection(can_id=1),
            "0x3E": lambda: api.read_motor_protection_state(can_id=1),
            "0x41": lambda: api.restart_motor(can_id=1),
            "0x80": lambda: api.calibrate_encoder(can_id=1),
            "0x92": lambda: api.set_current_axis_to_zero(can_id=1),
            "0xF7": lambda: api.emergency_stop(can_id=1),
        }
        
        if cmd_code not in command_calls:
            pytest.skip(f"Command {cmd_code} not mapped for testing")
        
        try:
            result = await command_calls[cmd_code]()
            
            # Verify result type based on command specification
            if cmd_code == "0x30":
                assert isinstance(result, tuple) and len(result) == 2, f"0x30 should return (carry, value) tuple"
                carry, value = result
                assert isinstance(carry, int) and isinstance(value, int), f"0x30 should return int types"
            elif cmd_code in ["0x31", "0x32", "0x35"]:
                assert isinstance(result, int), f"{cmd_code} should return int"
            elif cmd_code == "0x34":
                assert isinstance(result, dict), f"0x34 should return dict"
            elif cmd_code in ["0x3D", "0x3E"]:
                assert isinstance(result, bool), f"{cmd_code} should return bool"
            elif cmd_code in ["0x3B"]:
                assert isinstance(result, int), f"{cmd_code} should return status int"
            
        except Exception as e:
            pytest.fail(f"Command {cmd_code} ({cmd_spec['name']}) failed: {e}")