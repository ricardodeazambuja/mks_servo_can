import pytest
from mks_servo_can_library.mks_servo_can.crc import calculate_crc, verify_crc
from mks_servo_can_library.mks_servo_can.exceptions import ParameterError

def test_calculate_crc_valid():
    assert calculate_crc(0x01, [0x30]) == 0x31
    assert calculate_crc(0x01, [0xF6, 0x01, 0x40, 0x02]) == 0x3A # Example from manual PDF page 55

def test_verify_crc_valid():
    assert verify_crc(0x01, [0x30, 0x00, 0x00, 0x00, 0x01, 0x29, 0xEF, 0x4A]) # PDF page 54 (rx)
                                                                              # Data: 30 00 00 00 01 29 EF, CRC: 4A
                                                                              # (0x01 + 0x30 + 0x00 ... + 0xEF) & 0xFF = 0x4A

def test_calculate_crc_invalid_id():
    with pytest.raises(ParameterError):
        calculate_crc(0x800, [0x30])

def test_calculate_crc_empty_data():
    with pytest.raises(ParameterError):
        calculate_crc(0x01, [])