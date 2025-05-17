# mks_servo_can_project/tests/unit/test_crc.py
import pytest

from mks_servo_can.crc import calculate_crc
from mks_servo_can.crc import verify_crc
from mks_servo_can.exceptions import ParameterError

# Test cases from MKS SERVO42&57D_CAN User Manual V1.0.6.pdf
# Page 14: Example: command "01 30 CRC" -> CRC = (0x01 + 0x30) & 0xFF = 0x31
# Page 54 (cangaroo example for 0x30):
#   Downlink: ID=01, DLC=2, Data=[30, 31(CRC)]
#   Uplink:   ID=01, DLC=8, Data=[30, 00,00,00,00, 01,29, EF, 4A(CRC)]
#     CRC for uplink: (0x01 + 0x30 + 0x00 + 0x00 + 0x00 + 0x00 + 0x01 + 0x29 + 0xEF) & 0xFF
#     = (1 + 48 + 0 + 0 + 0 + 0 + 1 + 41 + 239) & 0xFF
#     = (330) & 0xFF = 0x4A (Correct)

# Page 55 (cangaroo example for 0xF6):
#   Downlink: ID=01, DLC=5, Data=[F6, 01, 40, 02, 3A(CRC)]
#     CRC for downlink: (0x01 + 0xF6 + 0x01 + 0x40 + 0x02) & 0xFF
#     = (1 + 246 + 1 + 64 + 2) & 0xFF
#     = (314) & 0xFF = 0x3A (Correct)
#   Uplink: ID=01, DLC=3, Data=[F6, 01, F8(CRC)]
#     CRC for uplink: (0x01 + 0xF6 + 0x01) & 0xFF
#     = (1 + 246 + 1) & 0xFF
#     = (248) & 0xFF = 0xF8 (Correct)


class TestCRC:
    def test_calculate_crc_simple(self):
        """Test basic CRC calculation."""
        assert calculate_crc(can_id=0x01, data_bytes=[0x30]) == 0x31
        assert calculate_crc(can_id=0x01, data_bytes=[0x30, 0x31]) == 0x62

    def test_calculate_crc_manual_examples(self):
        """Test CRC calculations based on manual examples."""
        assert calculate_crc(can_id=0x01, data_bytes=[0x30]) == 0x31
        data_uplink_30 = [0x30, 0x00, 0x00, 0x00, 0x00, 0x01, 0x29, 0xEF]
        assert calculate_crc(can_id=0x01, data_bytes=data_uplink_30) == 0x4A
        data_downlink_f6 = [0xF6, 0x01, 0x40, 0x02]
        assert calculate_crc(can_id=0x01, data_bytes=data_downlink_f6) == 0x3A
        data_uplink_f6 = [0xF6, 0x01]
        assert calculate_crc(can_id=0x01, data_bytes=data_uplink_f6) == 0xF8

    def test_calculate_crc_with_zero_id_and_data(self):
        """Test CRC with zero values."""
        assert calculate_crc(can_id=0x00, data_bytes=[0x00]) == 0x00
        assert calculate_crc(can_id=0x00, data_bytes=[0x00, 0x00, 0x00]) == 0x00

    def test_calculate_crc_max_values(self):
        """Test CRC with maximum byte values."""
        # (0x01 + 0xFF + 0xFF) & 0xFF = (1 + 255 + 255) & 0xFF = 511 & 0xFF
        # 511 in binary is 1 1111 1111. 0xFF is 00000000 1111 1111.
        # 511 & 255 = 255 (0xFF).
        # The previous comment was (0x1FD & 0xFF = 0xFD). 0x1FD is 409+253 = 509.
        # 1 + 255 + 255 = 511 = 0x1FF. So 0x1FF & 0xFF = 0xFF.
        assert (
            calculate_crc(can_id=0x01, data_bytes=[0xFF, 0xFF]) == 0xFF
        )  # Corrected from 0xFD
        # Verify the first part of the original assertion which should be true
        assert (
            calculate_crc(can_id=0x01, data_bytes=[0xFF, 0xFF])
            == (0x01 + 0xFF + 0xFF) & 0xFF
        )

    def test_calculate_crc_broadcast_id(self):
        """Test CRC calculation when CAN ID is broadcast (0x00)."""
        assert calculate_crc(can_id=0x00, data_bytes=[0x82, 0x05]) == 0x87

    def test_calculate_crc_invalid_can_id(self):
        """Test CRC calculation with an invalid CAN ID."""
        with pytest.raises(
            ParameterError, match="CAN ID 2048 is out of valid range 0-2047."
        ):
            calculate_crc(can_id=0x800, data_bytes=[0x30])
        with pytest.raises(
            ParameterError, match="CAN ID -1 is out of valid range 0-2047."
        ):
            calculate_crc(can_id=-1, data_bytes=[0x30])

    def test_calculate_crc_empty_data_bytes(self):
        """Test CRC calculation with empty data_bytes list."""
        with pytest.raises(
            ParameterError, match="Data bytes list cannot be empty"
        ):
            calculate_crc(can_id=0x01, data_bytes=[])

    def test_calculate_crc_invalid_data_byte_value(self):
        """Test CRC calculation with data_bytes containing out-of-range values."""
        with pytest.raises(
            ParameterError, match="All data bytes must be in range 0-255."
        ):
            calculate_crc(can_id=0x01, data_bytes=[0x00, 0x100])
        with pytest.raises(
            ParameterError, match="All data bytes must be in range 0-255."
        ):
            calculate_crc(can_id=0x01, data_bytes=[-1])

    def test_verify_crc_valid(self):
        """Test successful CRC verification."""
        received_bytes_30 = [
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0x29,
            0xEF,
            0x4A,
        ]
        assert verify_crc(can_id=0x01, received_bytes=received_bytes_30) is True
        received_bytes_f6 = [0xF6, 0x01, 0xF8]
        assert verify_crc(can_id=0x01, received_bytes=received_bytes_f6) is True

    def test_verify_crc_invalid(self):
        """Test failed CRC verification."""
        received_bytes_invalid_crc = [
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0x29,
            0xEF,
            0x4B,
        ]
        assert (
            verify_crc(can_id=0x01, received_bytes=received_bytes_invalid_crc)
            is False
        )

    def test_verify_crc_short_payload(self):
        """Test CRC verification with a payload that's too short."""
        with pytest.raises(
            ParameterError, match="Received bytes list is too short"
        ):
            verify_crc(can_id=0x01, received_bytes=[0x30])
        with pytest.raises(
            ParameterError, match="Received bytes list is too short"
        ):
            verify_crc(can_id=0x01, received_bytes=[])

    def test_verify_crc_invalid_can_id_for_calc(self):
        """Test CRC verification where CAN ID for calculation would be invalid."""
        received_bytes = [0x30, 0x31]
        with pytest.raises(
            ParameterError, match="CAN ID 2048 is out of valid range 0-2047."
        ):
            verify_crc(can_id=0x800, received_bytes=received_bytes)
