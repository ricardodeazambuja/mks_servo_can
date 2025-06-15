# CAN Protocol Details (MKS Specifics)

This document outlines the CAN communication protocol used by MKS SERVO42D and SERVO57D motors, as implemented and interpreted by the mks-servo-can library. The primary reference for this information is the "MKS SERVO42D/57D_CAN User Manual V1.0.6".

## CAN Frame Format

MKS servo motors use the Standard CAN frame format (11-bit identifier). Extended CAN frames (29-bit identifier) are not typically used for basic command communication with these motors.

A standard CAN frame consists of:

- **Arbitration ID (CAN ID)**: An 11-bit identifier. For MKS servos, this ID typically represents the specific motor being addressed.
  - The CAN ID can range from 0 to 2047 (0x000 to 0x7FF).
  - A CAN ID of 0x00 is typically reserved as a broadcast address.
  - The default CAN ID for a new motor is often 0x01.
- **Data Length Code (DLC)**: Indicates the number of data bytes in the payload (0 to 8 bytes).
- **Data Bytes (Payload)**: Up to 8 bytes of data, which for MKS servos includes:
  - **Command Code (Byte 1)**: The first byte of the data payload always represents the command being sent or the echoed command in a response.
  - **Command-Specific Data (Bytes 2 to n-1)**: Subsequent bytes carry parameters or data associated with the command. The number and meaning of these bytes vary per command.
  - **CRC Checksum (Byte n)**: The last byte of the data payload is an 8-bit checksum for validating data integrity.
- **Other CAN fields**: Such as RTR (Remote Transmission Request), IDE (Identifier Extension), CRC field (CAN-level CRC, distinct from the data payload CRC), ACK slot, EOF (End of Frame). These are handled by the CAN controller and python-can library.

Reference from MKS SERVO57D_CAN User Manual V1.0.0 (Source 1.5):

- **Downlink package (PC → SERVO42D/57D)**: `CANID DLC byte1 byte2 ... byte(n-1) Byte(n)` where byte1 is the command code, and Byte(n) is the CRC.
- **Uplink frame (PC ← SERVO42D/57D)**: `CANID DLC byte1 byte2 ... byte(n-1) Byte(n)` where byte1 is the echoed command code, and Byte(n) is the CRC.

## Command Structure

A typical command sent from the host (e.g., this library) to an MKS servo motor via CAN will have the following structure within the CAN frame's data payload:

```
[Command_Code, Optional_Data_Byte_1, ..., Optional_Data_Byte_k, CRC_Checksum]
```

- **Command_Code**: A single byte identifying the operation to be performed (e.g., read position, set speed, enable motor). These are defined in mks_servo_can.constants.
- **Optional_Data_Bytes**: Zero or more bytes that provide parameters for the command. For example, when setting a work mode, a data byte will specify the desired mode. For movement commands, data bytes will specify speed, acceleration, and target position/pulses.
- **CRC_Checksum**: The final byte of the data payload is an 8-bit checksum calculated over the Command_Code and any Optional_Data_Bytes, plus the CAN ID itself.

## Response Structure

When a motor responds to a command, the response frame's data payload generally mirrors the command structure:

```
[Echoed_Command_Code, Optional_Response_Data_1, ..., Optional_Response_Data_m, CRC_Checksum]
```

- **Echoed_Command_Code**: The first byte is typically an echo of the original Command_Code that was sent, or a related command code if the operation is to read a specific parameter (e.g., 0x00 for reading system parameters, where the actual parameter's command code is echoed in the response).
- **Optional_Response_Data**: Zero or more bytes containing the data requested or status information. For example, a "read position" command will result in response data bytes containing the position value. A "set parameter" command might respond with a status byte indicating success or failure.
- **CRC_Checksum**: The final byte, calculated over the Echoed_Command_Code, Optional_Response_Data, and the CAN ID of the responding motor.

## CRC Checksum Calculation

The MKS servo motors use a simple 8-bit checksum for the data payload. The formula is:

CRC = (CAN_ID + Byte₁ + Byte₂ + ... + Byteₖ) (mod 256)

Where:
- CAN_ID is the 11-bit CAN identifier of the motor.
- Byte_1 is the Command Code.
- Byte_2 to Byte_k are the optional data bytes excluding the CRC byte itself.
- The sum is performed, and then an bitwise AND with 0xFF (or modulo 256) is applied to get the 8-bit CRC.

The `mks_servo_can.crc.calculate_crc()` function implements this logic. The `mks_servo_can.crc.verify_crc()` function is used to validate the CRC of received messages.

**Example from MKS Manual (Source 1.5)**:
For a command "01 30 CRC" (Read Encoder Value, target CAN ID 01, command code 0x30):
CRC = (0x01 + 0x30) (mod 256) = 0x31 (mod 256) = 0x31.
The CAN frame data payload would be [0x30, 0x31].

## Key Command Codes

The `mks_servo_can.constants` module contains an extensive list of command codes. These are derived from the MKS User Manual. Some fundamental categories include:

### Read Status Parameters (e.g., 0x30 - 0x3E):
- `CMD_READ_ENCODER_CARRY` (0x30)
- `CMD_READ_ENCODER_ADDITION` (0x31): Reads the accumulated encoder value.
- `CMD_READ_MOTOR_SPEED_RPM` (0x32)
- `CMD_READ_IO_STATUS` (0x34)
- `CMD_READ_EN_PIN_STATUS` (0x3A)

### Set System Parameters (e.g., 0x80 - 0x9B):
- `CMD_CALIBRATE_ENCODER` (0x80)
- `CMD_SET_WORK_MODE` (0x82)
- `CMD_SET_WORKING_CURRENT` (0x83)
- `CMD_SET_SUBDIVISION` (0x84)
- `CMD_SET_CAN_ID` (0x8B)

### Set Home Commands (e.g., 0x90 - 0x9E):
- `CMD_GO_HOME` (0x91)
- `CMD_SET_CURRENT_AXIS_TO_ZERO` (0x92)

### Read System Parameter (0x00 prefix):
- `CMD_READ_SYSTEM_PARAMETER_PREFIX` (0x00): This command is special. The host sends [0x00, Parameter_Command_Code_To_Read, CRC]. The motor responds with [Parameter_Command_Code_To_Read, Value_Byte_1, ..., Value_Byte_m, CRC].

### Motor Control Commands (e.g., 0xF1 - 0xFE):
- `CMD_QUERY_MOTOR_STATUS` (0xF1)
- `CMD_ENABLE_MOTOR` (0xF3): Data byte 0x01 to enable, 0x00 to disable.
- `CMD_EMERGENCY_STOP` (0xF7)
- `CMD_RUN_SPEED_MODE` (0xF6)
- `CMD_RUN_POSITION_MODE_RELATIVE_PULSES` (0xFD)
- `CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES` (0xFE)

The interpretation of data bytes (e.g., how pulses, speed, or current are encoded into multiple bytes, including byte order - typically little-endian for MKS) is handled by the methods in low_level_api.py, often using Python's `struct` module for packing and unpacking.

## Data Interpretation

- **Multi-byte values**: Parameters like position, speed, or current are often sent or received as multi-byte values. These are typically encoded in big-endian format, as handled by the `struct` module in `low_level_api.py`. The `struct` module in Python is used within `low_level_api.py` to pack and unpack these values.
- **Signed vs. Unsigned**: Care must be taken to interpret whether a value is signed (e.g., motor speed RPM can be negative for reverse direction) or unsigned (e.g., pulse counts for relative moves are magnitudes, direction is a separate bit). The `low_level_api.py` handles these interpretations based on the MKS manual. For instance, `read_encoder_value_addition` explicitly handles sign extension for its 6-byte value.

## Active Response Mode (CanRSP Active)

MKS motors can be configured for "CanRSP Active" mode (Slave Respond Active). If enabled (usually via parameter 0x8C), the motor may send unsolicited status messages or multi-stage responses to certain commands (e.g., a "move complete" notification after a move command has been acknowledged). This library's `CANInterface` and `Axis` classes are designed to handle expected direct responses and use futures for asynchronous completion of commands, which can accommodate some of these active responses if they echo the original command code. More complex unsolicited message handling would require custom handlers to be registered with `CANInterface`.

This summary provides a high-level overview. For the exact data byte sequence and meaning for each specific command, the MKS SERVO42D/57D_CAN User Manual V1.0.6 and the implementation details in `mks_servo_can_library/mks_servo_can/low_level_api.py` and `mks_servo_can_library/mks_servo_can/constants.py` are the definitive sources.