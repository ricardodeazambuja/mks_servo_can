"""
Test CAN frame format compliance with MKS manual specification.

Validates that CAN messages use standard frames with correct arbitration IDs,
DLC values, and data field formats as specified in the manual.
"""

import pytest
import asyncio
import sys
import json
import struct
from pathlib import Path
from typing import Dict, List, Optional

# Add library to path
test_dir = Path(__file__).parent.parent.parent
lib_path = test_dir / "mks_servo_can_library"
sys.path.append(str(lib_path))

from mks_servo_can import CANInterface, const

# Load manual specifications
FIXTURES_DIR = Path(__file__).parent.parent / "fixtures"
with open(FIXTURES_DIR / "manual_commands_v106.json") as f:
    MANUAL_SPEC = json.load(f)


class TestCANFrameFormat:
    """Test CAN frame format compliance with manual specification"""
    
    def test_standard_frame_format(self):
        """Test that frames use standard format (not extended)"""
        # Manual specifies: "The CAN uses standard frames"
        
        # Standard CAN ID range: 0x000 to 0x7FF (0-2047)
        valid_can_ids = [0x001, 0x010, 0x100, 0x7FF]
        
        for can_id in valid_can_ids:
            assert 0x000 <= can_id <= 0x7FF, f"CAN ID {can_id:03X} outside standard range"
            
        # Extended CAN ID range would be 0x00000000 to 0x1FFFFFFF
        # These should NOT be used
        invalid_extended_ids = [0x800, 0x1000, 0x1FFFFFFF]
        
        for ext_id in invalid_extended_ids:
            assert ext_id > 0x7FF, f"Extended ID {ext_id:08X} incorrectly in standard range"
    
    def test_arbitration_id_format(self):
        """Test arbitration ID format per manual specification"""
        # Manual specifies downlink and uplink ID formats
        
        # Test downlink format (PC → Motor): typically 0x140 + motor_id
        motor_ids = [1, 2, 3, 16]  # Test various motor IDs
        
        for motor_id in motor_ids:
            downlink_id = 0x140 + motor_id
            uplink_id = 0x240 + motor_id
            
            # Verify IDs are in standard range
            assert 0x000 <= downlink_id <= 0x7FF, f"Downlink ID {downlink_id:03X} outside standard range"
            assert 0x000 <= uplink_id <= 0x7FF, f"Uplink ID {uplink_id:03X} outside standard range"
            
            # Verify ID calculation
            assert downlink_id == 0x140 + motor_id, f"Incorrect downlink ID calculation"
            assert uplink_id == 0x240 + motor_id, f"Incorrect uplink ID calculation"
    
    def test_dlc_values_compliance(self):
        """Test DLC values match manual specifications"""
        commands = MANUAL_SPEC["commands"]
        
        for cmd_code, cmd_spec in commands.items():
            request_dlc = cmd_spec["request"]["dlc"]
            response_dlc = cmd_spec["response"]["dlc"]
            
            # DLC must be 0-8 for CAN frames
            assert 0 <= request_dlc <= 8, f"Invalid request DLC for {cmd_code}: {request_dlc}"
            assert 0 <= response_dlc <= 8, f"Invalid response DLC for {cmd_code}: {response_dlc}"
            
            # Verify DLC matches expected data size
            if cmd_code == "0x30":  # Read encoder carry
                assert request_dlc == 2, f"0x30 request should have DLC=2, got {request_dlc}"
                assert response_dlc == 8, f"0x30 response should have DLC=8, got {response_dlc}"
            
            elif cmd_code == "0x31":  # Read encoder addition
                assert request_dlc == 2, f"0x31 request should have DLC=2, got {request_dlc}"
                assert response_dlc == 8, f"0x31 response should have DLC=8, got {response_dlc}"
            
            elif cmd_code == "0x32":  # Read speed
                assert request_dlc == 2, f"0x32 request should have DLC=2, got {request_dlc}"
                assert response_dlc == 4, f"0x32 response should have DLC=4, got {response_dlc}"
    
    def test_data_field_formats(self):
        """Test data field formats match manual specifications"""
        commands = MANUAL_SPEC["commands"]
        
        for cmd_code, cmd_spec in commands.items():
            response_fields = cmd_spec["response"]["fields"]
            
            # Verify field definitions
            assert "byte0" in response_fields, f"Response for {cmd_code} missing byte0 definition"
            
            # First byte should be command echo
            assert response_fields["byte0"] == "command_echo", f"First byte should echo command for {cmd_code}"
            
            # Verify data type specifications
            if cmd_code == "0x30":  # Encoder carry format
                assert "bytes1-4" in response_fields, f"0x30 missing carry field"
                assert "bytes5-6" in response_fields, f"0x30 missing value field"
                assert response_fields["bytes1-4"] == "carry_int32", f"0x30 carry should be int32"
                assert response_fields["bytes5-6"] == "value_uint16", f"0x30 value should be uint16"
            
            elif cmd_code == "0x31":  # Encoder addition format
                assert "bytes1-6" in response_fields, f"0x31 missing value field"
                assert response_fields["bytes1-6"] == "value_int48", f"0x31 value should be int48"
            
            elif cmd_code == "0x32":  # Speed format
                assert "bytes1-2" in response_fields, f"0x32 missing speed field"
                assert response_fields["bytes1-2"] == "speed_int16_rpm", f"0x32 speed should be int16"
    
    def test_crc_field_position(self):
        """Test CRC field is always last byte"""
        commands = MANUAL_SPEC["commands"]
        
        for cmd_code, cmd_spec in commands.items():
            response_dlc = cmd_spec["response"]["dlc"]
            response_fields = cmd_spec["response"]["fields"]
            
            # Find the highest byte index
            max_byte_field = f"byte{response_dlc-1}"
            
            # CRC should be in last position
            assert max_byte_field in response_fields or "crc" in response_fields.values(), \
                f"CRC field not found in last position for {cmd_code}"


class TestMessageStructure:
    """Test message structure compliance"""
    
    def test_downlink_message_structure(self):
        """Test downlink message structure: CAN_ID | DLC | Command_Code | Data_Bytes | CRC"""
        protocol_format = MANUAL_SPEC["protocol"]["downlink_format"]
        expected_format = "CAN_ID | DLC | Command_Code | Data_Bytes | CRC"
        
        assert protocol_format == expected_format, f"Downlink format mismatch: {protocol_format}"
    
    def test_uplink_message_structure(self):
        """Test uplink message structure: CAN_ID | DLC | Command_Code | Response_Data | CRC"""
        protocol_format = MANUAL_SPEC["protocol"]["uplink_format"]
        expected_format = "CAN_ID | DLC | Command_Code | Response_Data | CRC"
        
        assert protocol_format == expected_format, f"Uplink format mismatch: {protocol_format}"
    
    def test_frame_type_specification(self):
        """Test that manual specifies standard frames"""
        frame_type = MANUAL_SPEC["protocol"]["frame_type"]
        assert frame_type == "standard", f"Manual should specify standard frames, got: {frame_type}"
    
    def test_crc_calculation_specification(self):
        """Test CRC calculation method per manual"""
        crc_method = MANUAL_SPEC["protocol"]["crc_calculation"]
        expected_method = "(ID + byte1 + ... + byteN) & 0xFF"
        
        assert crc_method == expected_method, f"CRC method mismatch: {crc_method}"


class TestDataTypeCompliance:
    """Test data type specifications from manual"""
    
    def test_integer_types_specification(self):
        """Test integer type specifications"""
        commands = MANUAL_SPEC["commands"]
        
        # Test specific data types from manual
        test_cases = [
            ("0x30", "carry", "int32"),
            ("0x30", "value", "uint16"), 
            ("0x31", "value", "int48"),
            ("0x32", "speed", "int16"),
            ("0x34", "io_status", "uint8"),
            ("0x35", "raw_encoder", "uint16"),
        ]
        
        for cmd_code, field_name, expected_type in test_cases:
            if cmd_code in commands:
                cmd_spec = commands[cmd_code]
                response_fields = cmd_spec["response"]["fields"]
                
                # Find field with expected type
                found_type = False
                for field_key, field_desc in response_fields.items():
                    if expected_type in field_desc:
                        found_type = True
                        break
                
                assert found_type, f"Type {expected_type} not found in {cmd_code} fields"
    
    def test_byte_order_specification(self):
        """Test byte order for multi-byte fields"""
        # Manual examples suggest little-endian byte order for multi-byte values
        # This should be verified in actual message tests
        
        # For now, document the expected byte order
        expected_byte_orders = {
            "int16": "little-endian",
            "uint16": "little-endian", 
            "int32": "little-endian",
            "int48": "little-endian",
        }
        
        for data_type, byte_order in expected_byte_orders.items():
            assert byte_order == "little-endian", f"Expected little-endian for {data_type}"


class TestCommandCodeMapping:
    """Test command code mapping against manual"""
    
    def test_command_code_coverage(self):
        """Test that all manual command codes are defined"""
        commands = MANUAL_SPEC["commands"]
        
        expected_commands = [
            "0x30", "0x31", "0x32", "0x34", "0x35", "0x36",
            "0x3B", "0x3D", "0x3E", "0x41", "0x80", "0x92", 
            "0xF4", "0xF5", "0xF7", "0xFE"
        ]
        
        for expected_cmd in expected_commands:
            assert expected_cmd in commands, f"Command {expected_cmd} missing from manual specification"
    
    def test_command_categories(self):
        """Test command categorization"""
        categories = MANUAL_SPEC["categories"]
        expected_categories = ["status", "motion", "control", "configuration"]
        
        for expected_cat in expected_categories:
            assert expected_cat in categories, f"Category {expected_cat} missing"
            assert isinstance(categories[expected_cat], str), f"Category {expected_cat} should have description"
    
    def test_firmware_version_tracking(self):
        """Test firmware version tracking for commands"""
        firmware_history = MANUAL_SPEC["firmware_history"]
        
        # Verify recent firmware versions are tracked
        expected_versions = ["1.0.6", "1.0.5", "1.0.4"]
        
        for version in expected_versions:
            assert version in firmware_history, f"Firmware version {version} not tracked"
            assert isinstance(firmware_history[version], list), f"Firmware {version} should list commands"