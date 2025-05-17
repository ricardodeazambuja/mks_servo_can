# mks_servo_can/mks_servo_can_library/mks_servo_can/crc.py
"""
CRC calculation utility for MKS Servo CAN commands.
"""
from .exceptions import ParameterError

def calculate_crc(can_id: int, data_bytes: list[int]) -> int:
    """
    Calculates the 8-bit checksum (CRC) for an MKS Servo CAN command.
    The formula is CRC = (ID + byte1 + ... + byte(n)) & 0xFF. [cite: 90]

    Args:
        can_id: The CAN ID of the motor (0-2047).
        data_bytes: A list of data bytes for the command (command code + data).
                    The list should not include the CRC byte itself.

    Returns:
        The calculated 8-bit CRC value.

    Raises:
        ParameterError: if can_id or data_bytes are invalid.
    """
    if not (0 <= can_id <= 0x7FF): # Max CAN ID for standard frame is 11-bit (0-2047)
        raise ParameterError(f"CAN ID {can_id} is out of valid range 0-2047.")
    if not data_bytes:
        raise ParameterError("Data bytes list cannot be empty for CRC calculation.")
    if not all(0 <= b <= 0xFF for b in data_bytes):
        raise ParameterError("All data bytes must be in range 0-255.")

    checksum = can_id
    for byte_val in data_bytes:
        checksum += byte_val

    return checksum & 0xFF

def verify_crc(can_id: int, received_bytes: list[int]) -> bool:
    """
    Verifies the CRC of a received MKS Servo CAN message.

    Args:
        can_id: The CAN ID from the received message.
        received_bytes: A list of all data bytes from the received CAN message,
                        INCLUDING the received CRC byte as the last element.

    Returns:
        True if the CRC is valid, False otherwise.

    Raises:
        ParameterError: if inputs are invalid.
    """
    if not received_bytes or len(received_bytes) < 2: # Must have at least command code and CRC
        raise ParameterError("Received bytes list is too short to verify CRC (must include data and CRC).")

    data_payload = received_bytes[:-1]
    received_crc = received_bytes[-1]

    calculated_crc = calculate_crc(can_id, data_payload)
    return calculated_crc == received_crc