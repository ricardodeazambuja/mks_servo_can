# Unit tests are designed to test individual components or functions in isolation.

import pytest
# Imports the pytest framework, which is used for writing and running these tests.

from mks_servo_can.crc import calculate_crc
# Imports the 'calculate_crc' function from the crc module within the mks_servo_can library.
# This function is responsible for generating the CRC checksum for a given CAN ID and data payload.

from mks_servo_can.crc import verify_crc
# Imports the 'verify_crc' function from the crc module.
# This function is used to check if a received CRC matches the calculated CRC for a message.

from mks_servo_can.exceptions import ParameterError
# Imports the 'ParameterError' exception from the library's exceptions module.
# This custom exception is expected to be raised when invalid parameters are passed to the CRC functions.

# Test cases from MKS SERVO42&57D_CAN User Manual V1.0.6.pdf
# This comment indicates that the following test cases are derived or inspired by examples
# found in the official MKS servo motor user manual, specifically version 1.0.6.
# This is good practice as it ensures the CRC implementation aligns with the hardware documentation.

# Page 14: Example: command "01 30 CRC" -> CRC = (0x01 + 0x30) & 0xFF = 0x31
# This is a specific example cited from page 14 of the manual.
# For a CAN ID of 0x01 and data byte 0x30, the CRC should be 0x31.

# Page 54 (cangaroo example for 0x30):
# This refers to an example, possibly from a CAN tool named "cangaroo", on page 54 of the manual,
# related to command 0x30 (Read Encoder Value).
#   Downlink: ID=01, DLC=2, Data=[30, 31(CRC)]
#   A message sent from the host (PC) to the motor (SERVO).
#   CAN ID is 0x01, Data Length Code is 2, and the data payload is [0x30 (command), 0x31 (CRC)].
#   Uplink:   ID=01, DLC=8, Data=[30, 00,00,00,00, 01,29, EF, 4A(CRC)]
#   A message sent from the motor back to the host.
#   CAN ID is 0x01, DLC is 8. The data payload contains the echoed command (0x30), response data, and the CRC (0x4A).
#     CRC for uplink: (0x01 + 0x30 + 0x00 + 0x00 + 0x00 + 0x00 + 0x01 + 0x29 + 0xEF) & 0xFF
#     This shows the manual calculation for the uplink CRC.
#     = (1 + 48 + 0 + 0 + 0 + 0 + 1 + 41 + 239) & 0xFF
#     The byte values are summed with the CAN ID.
#     = (330) & 0xFF = 0x4A (Correct)
#     The sum (330 decimal, which is 0x14A hex) is bitwise ANDed with 0xFF, resulting in 0x4A. This confirms the manual's example.

# Page 55 (cangaroo example for 0xF6):
# Another example from page 55 for command 0xF6 (Run Speed Mode).
#   Downlink: ID=01, DLC=5, Data=[F6, 01, 40, 02, 3A(CRC)]
#   Host to motor message for command 0xF6.
#     CRC for downlink: (0x01 + 0xF6 + 0x01 + 0x40 + 0x02) & 0xFF
#     Manual calculation for this downlink CRC.
#     = (1 + 246 + 1 + 64 + 2) & 0xFF
#     = (314) & 0xFF = 0x3A (Correct)
#     The sum (314 decimal, 0x13A hex) ANDed with 0xFF gives 0x3A.
#   Uplink: ID=01, DLC=3, Data=[F6, 01, F8(CRC)]
#   Motor to host response for command 0xF6.
#     CRC for uplink: (0x01 + 0xF6 + 0x01) & 0xFF
#     Manual calculation for this uplink CRC.
#     = (1 + 246 + 1) & 0xFF
#     = (248) & 0xFF = 0xF8 (Correct)
#     The sum (248 decimal, 0xF8 hex) ANDed with 0xFF gives 0xF8.

class TestCRC:
    # Defines a test class 'TestCRC' to group CRC-related test methods.
    # Pytest will discover and run methods within this class that start with 'test_'.

    def test_calculate_crc_simple(self):
        # This test method checks basic CRC calculations with simple inputs.
        """Test basic CRC calculation."""
        # Test case 1: CAN ID 0x01, data byte 0x30. Expected CRC is 0x31.
        assert calculate_crc(can_id=0x01, data_bytes=[0x30]) == 0x31
        # Test case 2: CAN ID 0x01, data bytes [0x30, 0x31].
        # (0x01 + 0x30 + 0x31) & 0xFF = (1 + 48 + 49) & 0xFF = 98 & 0xFF = 0x62.
        assert calculate_crc(can_id=0x01, data_bytes=[0x30, 0x31]) == 0x62

    def test_calculate_crc_manual_examples(self):
        # This test method verifies CRC calculations against the examples taken directly from the MKS user manual.
        # This is crucial for ensuring the library's CRC logic matches the device's expected behavior.
        """Test CRC calculations based on manual examples."""
        # Example 1 (Downlink for command 0x30): CAN ID 0x01, data [0x30]. Expected CRC 0x31.
        assert calculate_crc(can_id=0x01, data_bytes=[0x30]) == 0x31
        # Data payload for the uplink response to command 0x30.
        data_uplink_30 = [0x30, 0x00, 0x00, 0x00, 0x00, 0x01, 0x29, 0xEF]
        # Example 2 (Uplink for command 0x30): CAN ID 0x01, data 'data_uplink_30'. Expected CRC 0x4A.
        assert calculate_crc(can_id=0x01, data_bytes=data_uplink_30) == 0x4A
        # Data payload for the downlink command 0xF6.
        data_downlink_f6 = [0xF6, 0x01, 0x40, 0x02]
        # Example 3 (Downlink for command 0xF6): CAN ID 0x01, data 'data_downlink_f6'. Expected CRC 0x3A.
        assert calculate_crc(can_id=0x01, data_bytes=data_downlink_f6) == 0x3A
        # Data payload for the uplink response to command 0xF6.
        data_uplink_f6 = [0xF6, 0x01]
        # Example 4 (Uplink for command 0xF6): CAN ID 0x01, data 'data_uplink_f6'. Expected CRC 0xF8.
        assert calculate_crc(can_id=0x01, data_bytes=data_uplink_f6) == 0xF8

    def test_calculate_crc_with_zero_id_and_data(self):
        # This test checks CRC calculation with zero values for CAN ID and data bytes.
        # The MKS CRC formula is a simple sum, so these should result in 0.
        """Test CRC with zero values."""
        # Test case 1: CAN ID 0x00, data [0x00]. (0x00 + 0x00) & 0xFF = 0x00.
        assert calculate_crc(can_id=0x00, data_bytes=[0x00]) == 0x00
        # Test case 2: CAN ID 0x00, data [0x00, 0x00, 0x00]. (0x00 + 0x00 + 0x00 + 0x00) & 0xFF = 0x00.
        assert calculate_crc(can_id=0x00, data_bytes=[0x00, 0x00, 0x00]) == 0x00

    def test_calculate_crc_max_values(self):
        # This test checks CRC calculation with maximum byte values for data to see if overflow is handled correctly by the '& 0xFF' operation.
        """Test CRC with maximum byte values."""
        # (0x01 + 0xFF + 0xFF) & 0xFF = (1 + 255 + 255) & 0xFF = 511 & 0xFF
        # 511 in decimal is 0x1FF.
        # 0x1FF & 0xFF = 0xFF.
        # The original comment had a calculation error; the code's expected result (0xFF) is correct.
        # Original comment: "The previous comment was (0x1FD & 0xFF = 0xFD). 0x1FD is 409+253 = 509." This seems unrelated or mistaken.
        # 1 + 255 + 255 = 511 = 0x1FF. So 0x1FF & 0xFF = 0xFF.
        assert (
            calculate_crc(can_id=0x01, data_bytes=[0xFF, 0xFF]) == 0xFF
        )  # Corrected from 0xFD (original code was correct)
        # This assertion re-verifies the calculation inline, confirming the logic.
        assert (
            calculate_crc(can_id=0x01, data_bytes=[0xFF, 0xFF])
            == (0x01 + 0xFF + 0xFF) & 0xFF
        )

    def test_calculate_crc_broadcast_id(self):
        # Tests CRC calculation when the CAN ID is the broadcast address (0x00).
        # The CAN ID is part of the CRC sum, so this is a valid test case.
        """Test CRC calculation when CAN ID is broadcast (0x00)."""
        # (0x00 + 0x82 + 0x05) & 0xFF = (0 + 130 + 5) & 0xFF = 135 & 0xFF = 0x87.
        assert calculate_crc(can_id=0x00, data_bytes=[0x82, 0x05]) == 0x87

    def test_calculate_crc_invalid_can_id(self):
        # This test ensures that the 'calculate_crc' function raises a 'ParameterError'
        # if the provided CAN ID is outside the valid 11-bit range (0-2047, or 0x000-0x7FF).
        """Test CRC calculation with an invalid CAN ID."""
        # Test with CAN ID 0x800 (2048), which is out of range.
        with pytest.raises(
            ParameterError, match="CAN ID 2048 is out of valid range 0-2047."
        ):
            calculate_crc(can_id=0x800, data_bytes=[0x30])
        # Test with a negative CAN ID, also out of range.
        with pytest.raises(
            ParameterError, match="CAN ID -1 is out of valid range 0-2047."
        ):
            calculate_crc(can_id=-1, data_bytes=[0x30])

    def test_calculate_crc_empty_data_bytes(self):
        # This test ensures that 'calculate_crc' raises a 'ParameterError' if the 'data_bytes' list is empty,
        # as the MKS protocol implies at least a command byte is always present before the CRC.
        """Test CRC calculation with empty data_bytes list."""
        with pytest.raises(
            ParameterError, match="Data bytes list cannot be empty"
        ):
            calculate_crc(can_id=0x01, data_bytes=[])

    def test_calculate_crc_invalid_data_byte_value(self):
        # This test ensures that 'calculate_crc' raises a 'ParameterError' if any byte in 'data_bytes'
        # is not within the valid range for a byte (0-255).
        """Test CRC calculation with data_bytes containing out-of-range values."""
        # Test with a data byte value of 0x100 (256), which is too large.
        with pytest.raises(
            ParameterError, match="All data bytes must be in range 0-255."
        ):
            calculate_crc(can_id=0x01, data_bytes=[0x00, 0x100])
        # Test with a negative data byte value.
        with pytest.raises(
            ParameterError, match="All data bytes must be in range 0-255."
        ):
            calculate_crc(can_id=0x01, data_bytes=[-1])

    def test_verify_crc_valid(self):
        # This test method checks that 'verify_crc' correctly identifies valid CRCs
        # using data from the manual examples.
        """Test successful CRC verification."""
        # Data payload (including command, data, and received CRC) for uplink of command 0x30.
        received_bytes_30 = [
            0x30, # Echoed command
            0x00, 0x00, 0x00, 0x00, # Data
            0x01, 0x29, 0xEF, # Data
            0x4A, # Received CRC
        ]
        # Verify this message with CAN ID 0x01. It should be True.
        assert verify_crc(can_id=0x01, received_bytes=received_bytes_30) is True
        # Data payload for uplink of command 0xF6.
        received_bytes_f6 = [0xF6, 0x01, 0xF8] # Echoed command, data, received CRC
        # Verify this message with CAN ID 0x01. It should be True.
        assert verify_crc(can_id=0x01, received_bytes=received_bytes_f6) is True

    def test_verify_crc_invalid(self):
        # This test ensures that 'verify_crc' correctly identifies an invalid CRC.
        """Test failed CRC verification."""
        # This payload is similar to 'received_bytes_30' but with an incorrect CRC (0x4B instead of 0x4A).
        received_bytes_invalid_crc = [
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0x29,
            0xEF,
            0x4B, # Incorrect CRC
        ]
        # Verification should return False.
        assert (
            verify_crc(can_id=0x01, received_bytes=received_bytes_invalid_crc)
            is False
        )

    def test_verify_crc_short_payload(self):
        # This test ensures 'verify_crc' raises a 'ParameterError' if the received byte list
        # is too short to contain at least a command byte and a CRC byte.
        """Test CRC verification with a payload that's too short."""
        # Payload with only one byte (not enough for command + CRC).
        with pytest.raises(
            ParameterError, match="Received bytes list is too short"
        ):
            verify_crc(can_id=0x01, received_bytes=[0x30])
        # Empty payload.
        with pytest.raises(
            ParameterError, match="Received bytes list is too short"
        ):
            verify_crc(can_id=0x01, received_bytes=[])

    def test_verify_crc_invalid_can_id_for_calc(self):
        # This test ensures that 'verify_crc' correctly propagates 'ParameterError'
        # if the CAN ID provided for verification would be invalid for the internal CRC calculation.
        """Test CRC verification where CAN ID for calculation would be invalid."""
        # A valid-looking payload.
        received_bytes = [0x30, 0x31]
        # Provide an invalid CAN ID (0x800) for verification.
        # The 'calculate_crc' function called internally by 'verify_crc' should raise the error.
        with pytest.raises(
            ParameterError, match="CAN ID 2048 is out of valid range 0-2047."
        ):
            verify_crc(can_id=0x800, received_bytes=received_bytes)
            