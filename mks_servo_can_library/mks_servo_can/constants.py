# mks_servo_can/mks_servo_can_library/mks_servo_can/constants.py
"""
Constants for the MKS Servo CAN library.
Includes CAN command codes, default values, and other relevant constants
derived from the MKS SERVO42D/57D_CAN User Manual.
"""

# Default CAN Interface Parameters [cite: 450]
CAN_DEFAULT_BITRATE = 500000  # 500K bps
CAN_TIMEOUT_SECONDS = 1.0     # Default timeout for CAN operations

# CAN IDs [cite: 87, 88]
BROADCAST_ADDRESS = 0x00
DEFAULT_CAN_ID = 0x01

# Work Modes (from manual Part 3, section 2 and Part 5.2, command 0x82) [cite: 35, 36, 37, 151]
MODE_CR_OPEN = 0
MODE_CR_CLOSE = 1
MODE_CR_VFOC = 2
MODE_SR_OPEN = 3
MODE_SR_CLOSE = 4
MODE_SR_VFOC = 5 # Default for serial according to some examples [cite: 398]

WORK_MODES = {
    MODE_CR_OPEN: "CR_OPEN",    # Pulse interface, Open loop
    MODE_CR_CLOSE: "CR_CLOSE",  # Pulse interface, Close loop
    MODE_CR_VFOC: "CR_VFOC",    # Pulse interface, FOC (Default overall) [cite: 14]
    MODE_SR_OPEN: "SR_OPEN",    # Serial interface, Open loop
    MODE_SR_CLOSE: "SR_CLOSE",  # Serial interface, Close loop
    MODE_SR_VFOC: "SR_VFOC",    # Serial interface, FOC
}

# Command Codes (from manual Part 5 & 6)
# Read Status Parameters (Part 5.1) [cite: 93, 102, 108, 112, 115, 120, 127, 132, 136, 140, 143]
CMD_READ_ENCODER_CARRY = 0x30
CMD_READ_ENCODER_ADDITION = 0x31
CMD_READ_MOTOR_SPEED_RPM = 0x32
CMD_READ_PULSES_RECEIVED = 0x33
CMD_READ_IO_STATUS = 0x34
CMD_READ_RAW_ENCODER_ADDITION = 0x35 # V1.0.6 [cite: 2]
CMD_READ_SHAFT_ANGLE_ERROR = 0x39
CMD_READ_EN_PIN_STATUS = 0x3A
CMD_READ_POWER_ON_ZERO_STATUS = 0x3B
CMD_RELEASE_STALL_PROTECTION = 0x3D
CMD_READ_MOTOR_PROTECTION_STATE = 0x3E

# Set System Parameters (Part 5.2) [cite: 147, 151, 156, 160, 166, 170, 173, 178, 183, 190, 195, 199, 204, 210, 214]
CMD_CALIBRATE_ENCODER = 0x80
CMD_SET_WORK_MODE = 0x82
CMD_SET_WORKING_CURRENT = 0x83
CMD_SET_SUBDIVISION = 0x84
CMD_SET_EN_PIN_ACTIVE_LEVEL = 0x85
CMD_SET_MOTOR_DIRECTION = 0x86
CMD_SET_AUTO_SCREEN_OFF = 0x87
CMD_SET_STALL_PROTECTION = 0x88
CMD_SET_SUBDIVISION_INTERPOLATION = 0x89
CMD_SET_CAN_BITRATE = 0x8A
CMD_SET_CAN_ID = 0x8B
CMD_SET_SLAVE_RESPOND_ACTIVE = 0x8C
CMD_SET_GROUP_ID = 0x8D
CMD_SET_KEY_LOCK = 0x8F
CMD_SET_HOLDING_CURRENT_PERCENTAGE = 0x9B # V1.0.4 [cite: 2]

# Write IO Port (Part 5.3) [cite: 221]
CMD_WRITE_IO_PORT = 0x36 # V1.0.6 [cite: 2]

# Set Home Command (Part 5.4) [cite: 224, 232, 237, 241, 246]
CMD_SET_HOME_PARAMETERS = 0x90
CMD_GO_HOME = 0x91
CMD_SET_CURRENT_AXIS_TO_ZERO = 0x92
CMD_SET_NOLIMIT_HOME_PARAMS = 0x94 # V1.0.5 [cite: 2]
CMD_SET_LIMIT_PORT_REMAP = 0x9E # V1.0.4 [cite: 2]

# Set 0_Mode Command (Part 5.5) [cite: 254]
CMD_SET_ZERO_MODE_PARAMETERS = 0x9A

# Restore Default Parameters (Part 5.6) [cite: 260]
CMD_RESTORE_DEFAULT_PARAMETERS = 0x3F

# Restart Motor (Part 5.7) [cite: 265]
CMD_RESTART_MOTOR = 0x41 # V1.0.5 [cite: 2]

# En Triggers and Position Error Protection (Part 5.8) [cite: 268]
CMD_SET_EN_TRIGGER_POS_ERROR_PROTECTION = 0x9D # V1.0.6 [cite: 2]

# Read System Parameter (Part 5.9) [cite: 280]
# This command uses 0x00 as the first byte, then the actual parameter's command code.
CMD_READ_SYSTEM_PARAMETER_PREFIX = 0x00

# Run Motor Commands (Part 6)
# Query/Enable Motor (Part 6.2) [cite: 302, 305]
CMD_QUERY_MOTOR_STATUS = 0xF1
CMD_ENABLE_MOTOR = 0xF3

# Emergency Stop (Part 6.3) [cite: 310]
CMD_EMERGENCY_STOP = 0xF7 # V1.0.4 [cite: 2]

# Speed Mode (Part 6.4) [cite: 315, 322, 329]
CMD_RUN_SPEED_MODE = 0xF6
CMD_SAVE_CLEAN_SPEED_MODE_PARAMS = 0xFF # Note: Overlaps with 0xFF for error in read response [cite: 286]

# Position Mode 1: Relative motion by pulses (Part 6.5) [cite: 335, 343]
CMD_RUN_POSITION_MODE_RELATIVE_PULSES = 0xFD

# Position Mode 2: Absolute motion by pulses (Part 6.6) [cite: 350, 358]
CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES = 0xFE # V1.0.4 [cite: 2]

# Position Mode 3: Relative motion by axis (Part 6.7) [cite: 365, 375]
CMD_RUN_POSITION_MODE_RELATIVE_AXIS = 0xF4 # V1.0.5 (Fixed bug) [cite: 2]

# Position Mode 4: Absolute motion by axis (Part 6.8) [cite: 382, 390]
CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS = 0xF5 # V1.0.5 (Supports real-time update) [cite: 2]


# Motor Types (for high-level API)
MOTOR_TYPE_SERVO42D = "MKS SERVO42D"
MOTOR_TYPE_SERVO57D = "MKS SERVO57D"

# Max currents (mA) [cite: 40, 158]
MAX_CURRENT_SERVO42D = 3000
MAX_CURRENT_SERVO57D = 5200
MAX_CURRENT_SERVO28D = 3000 # (Mentioned in manual for Ma settings)
MAX_CURRENT_SERVO35D = 3000 # (Mentioned in manual for Ma settings)

# Default RPMs and Speeds (from manual Part 1.4, Part 6.1) [cite: 13, 291, 292]
MAX_RPM_OPEN_MODE = 400
MAX_RPM_CLOSE_MODE = 1500
MAX_RPM_VFOC_MODE = 3000

# Kinematics
DEFAULT_STEPS_PER_REVOLUTION = 200 * 16 # Assuming 200 base steps and default 16 microsteps [cite: 43]
# Encoder resolution (from manual Part 5.1, command 0x30 example, one turn = 0x4000) [cite: 97]
ENCODER_PULSES_PER_REVOLUTION = 0x4000 # 16384

# Response Status Codes (examples, expand as needed)
STATUS_SUCCESS = 0x01
STATUS_FAILURE = 0x00
STATUS_CALIBRATING = 0x00
STATUS_CALIBRATED_SUCCESS = 0x01
STATUS_CALIBRATING_FAIL = 0x02

# For CMD_SLAVE_RESPOND_ACTIVE (0x8C) [cite: 205, 206]
RESPOND_DISABLED = 0x00
RESPOND_ENABLED = 0x01
ACTIVE_DISABLED = 0x00
ACTIVE_ENABLED = 0x01

# For motor status query (0xF1) [cite: 305]
MOTOR_STATUS_QUERY_FAIL = 0x00
MOTOR_STATUS_STOPPED = 0x01
MOTOR_STATUS_SPEED_UP = 0x02
MOTOR_STATUS_SPEED_DOWN = 0x03
MOTOR_STATUS_FULL_SPEED = 0x04
MOTOR_STATUS_HOMING = 0x05
MOTOR_STATUS_CALIBRATING = 0x06

# For position mode run status (e.g. 0xFD, 0xFE, 0xF4, 0xF5) [cite: 341, 356, 373, 388]
POS_RUN_FAIL = 0x00
POS_RUN_STARTING = 0x01
POS_RUN_COMPLETE = 0x02
POS_RUN_END_LIMIT_STOPPED = 0x03

# Home status (0x91) [cite: 236]
HOME_FAIL = 0x00
HOME_START = 0x01
HOME_SUCCESS = 0x02

# O_Mode settings (0x9A) [cite: 257]
OMODE_DISABLE = 0
OMODE_DIR_MODE = 1
OMODE_NEAR_MODE = 2
OMODE_SET_ZERO_CLEAN = 0
OMODE_SET_ZERO_SET = 1
OMODE_SET_ZERO_NO_MODIFY = 2
OMODE_DIR_CW = 0
OMODE_DIR_CCW = 1

# EN Pin active levels (0x85) [cite: 45, 46, 170]
EN_ACTIVE_LOW = 0x00
EN_ACTIVE_HIGH = 0x01
EN_ACTIVE_ALWAYS = 0x02 # "Hold" in menu

# Motor Direction (0x86) [cite: 47, 174, 175]
DIR_CW = 0x00
DIR_CCW = 0x01

# Bit rates for CAN_SET_BITRATE (0x8A) [cite: 197]
CAN_BITRATE_125K = 0x00
CAN_BITRATE_250K = 0x01
CAN_BITRATE_500K = 0x02
CAN_BITRATE_1M = 0x03

# Simulator specific constants
SIM_DEFAULT_LATENCY_MS = 5 # ms
SIM_DEFAULT_MOTOR_RESPONSE_SCALE = 1.0