"""
Test CRC calculation compliance with MKS manual specification.

Validates that CRC calculation matches the exact algorithm specified
in the manual: CRC = (ID + byte1 + ... + byteN) & 0xFF

Updated to use shared simulator management fixtures.
"""

import pytest
from pathlib import Path
from typing import List

from mks_servo_can import const
from mks_servo_can.crc import calculate_crc


@pytest.mark.compliance
class TestCRCCalculation:
    """Test CRC calculation against manual specification"""
    
    def test_manual_crc_algorithm(self):
        """Test CRC algorithm matches manual specification"""
        # Manual specification: CRC = (ID + byte1 + ... + byteN) & 0xFF
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            """Calculate CRC per manual specification"""
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # Test cases from manual
        test_cases = [
            # Manual example: command "01 30 CRC"
            # CRC = (0x01 + 0x30) & 0xFF = 0x31
            (0x01, [0x30], 0x31),
            
            # Additional test cases
            (0x01, [0x31], 0x32),  # 0x01 + 0x31 = 0x32
            (0x02, [0x30], 0x32),  # 0x02 + 0x30 = 0x32
            (0x01, [0x80], 0x81),  # Calibration command
            (0x01, [0xF4, 0x00], 0xF5),  # Multi-byte command
            
            # Test overflow wrapping
            (0xFF, [0xFF], 0xFE),  # (255 + 255) & 0xFF = 254
            (0x80, [0x80], 0x00),  # (128 + 128) & 0xFF = 0
        ]
        
        for can_id, data, expected_crc in test_cases:
            calculated_crc = manual_crc(can_id, data)
            assert calculated_crc == expected_crc, \
                f"CRC mismatch for ID {can_id:02X}, data {[hex(x) for x in data]}: " \
                f"expected 0x{expected_crc:02X}, got 0x{calculated_crc:02X}"
    
    def test_library_crc_vs_manual(self):
        """Test that library CRC calculation matches manual specification"""
        # Test library's calculate_crc function against manual algorithm
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            """Manual CRC calculation"""
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        test_cases = [
            (0x01, [0x30]),
            (0x01, [0x31]), 
            (0x02, [0x32]),
            (0x01, [0xF4, 0x10, 0x00, 0x00, 0x00, 0x64, 0x00]),  # Relative move command
            (0x01, [0xF5, 0xE8, 0x03, 0x00, 0x00, 0x64, 0x00]),  # Absolute move command
        ]
        
        for can_id, data in test_cases:
            manual_result = manual_crc(can_id, data)
            
            # Test library function
            data_bytes = bytes(data)
            library_result = calculate_crc(can_id, data_bytes)
            
            assert library_result == manual_result, \
                f"Library CRC mismatch for ID {can_id:02X}, data {[hex(x) for x in data]}: " \
                f"manual=0x{manual_result:02X}, library=0x{library_result:02X}"
    
    def test_crc_edge_cases(self):
        """Test CRC calculation edge cases"""
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # Edge cases
        edge_cases = [
            # Minimum values
            (0x00, [0x00], 0x00),
            
            # Maximum values  
            (0xFF, [0xFF], 0xFE),
            
            # Empty data
            (0x01, [], 0x01),
            
            # Single byte
            (0x01, [0x01], 0x02),
            
            # Maximum length data (8 bytes)
            (0x01, [0x01] * 8, 0x09),
            
            # Mixed values
            (0x10, [0x20, 0x30, 0x40], 0xA0),
        ]
        
        for can_id, data, expected in edge_cases:
            result = manual_crc(can_id, data)
            assert result == expected, \
                f"Edge case failed: ID {can_id:02X}, data {[hex(x) for x in data]}, " \
                f"expected 0x{expected:02X}, got 0x{result:02X}"
    
    def test_crc_overflow_behavior(self):
        """Test CRC calculation overflow behavior"""
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # Test cases that cause overflow
        overflow_cases = [
            # Simple overflow
            (0x80, [0x80], 0x00),  # 128 + 128 = 256, & 0xFF = 0
            (0x90, [0x90], 0x20),  # 144 + 144 = 288, & 0xFF = 32
            
            # Multiple byte overflow
            (0x50, [0x50, 0x50, 0x50], 0x40),  # 80 + 80*3 = 320, & 0xFF = 64
            
            # Maximum overflow
            (0xFF, [0xFF, 0xFF, 0xFF, 0xFF], 0xFB),  # 255 + 255*4 = 1275, & 0xFF = 251
        ]
        
        for can_id, data, expected in overflow_cases:
            result = manual_crc(can_id, data)
            assert result == expected, \
                f"Overflow case failed: ID {can_id:02X}, data {[hex(x) for x in data]}, " \
                f"expected 0x{expected:02X}, got 0x{result:02X}"
    
    def test_real_command_crc_examples(self):
        """Test CRC calculation for real command examples"""
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # Real command examples with known CRC values
        real_commands = [
            # Read encoder carry: 01 30 CRC
            (0x01, [0x30], 0x31),
            
            # Read encoder addition: 01 31 CRC  
            (0x01, [0x31], 0x32),
            
            # Read speed: 01 32 CRC
            (0x01, [0x32], 0x33),
            
            # Calibrate encoder: 01 80 CRC
            (0x01, [0x80], 0x81),
            
            # Go home: 01 3B CRC
            (0x01, [0x3B], 0x3C),
            
            # Emergency stop: 01 F7 CRC
            (0x01, [0xF7], 0xF8),
        ]
        
        for can_id, data, expected_crc in real_commands:
            calculated_crc = manual_crc(can_id, data)
            assert calculated_crc == expected_crc, \
                f"Real command CRC mismatch: {[hex(x) for x in [can_id] + data]}, " \
                f"expected 0x{expected_crc:02X}, got 0x{calculated_crc:02X}"
    
    def test_crc_consistency_across_motors(self):
        """Test CRC calculation consistency across different motor IDs"""
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # Same command to different motors should have different CRCs
        command = [0x30]  # Read encoder carry
        motor_ids = [0x01, 0x02, 0x03, 0x10]
        
        crc_values = []
        for motor_id in motor_ids:
            crc = manual_crc(motor_id, command)
            crc_values.append(crc)
            
            # Verify CRC calculation
            expected = (motor_id + 0x30) & 0xFF
            assert crc == expected, f"CRC mismatch for motor {motor_id:02X}: expected 0x{expected:02X}, got 0x{crc:02X}"
        
        # All CRC values should be different (since motor IDs are different)
        assert len(set(crc_values)) == len(crc_values), "CRC values should be unique for different motor IDs"
    
    def test_crc_byte_order_independence(self):
        """Test that CRC calculation doesn't depend on byte order of individual bytes"""
        
        def manual_crc(can_id: int, data: List[int]) -> int:
            crc = can_id
            for byte_val in data:
                crc += byte_val
            return crc & 0xFF
        
        # CRC should be sum-based, so order matters but individual byte representation doesn't
        test_data = [0x10, 0x20, 0x30]
        can_id = 0x01
        
        # Calculate CRC normally
        normal_crc = manual_crc(can_id, test_data)
        
        # Same data should give same result
        same_data_crc = manual_crc(can_id, [0x10, 0x20, 0x30])
        assert normal_crc == same_data_crc
        
        # Different order should give different result
        reordered_data = [0x30, 0x20, 0x10]
        reordered_crc = manual_crc(can_id, reordered_data)
        
        # Sum should be same since addition is commutative, so CRC should be same
        assert normal_crc == reordered_crc, "CRC should be same for reordered data (addition is commutative)"