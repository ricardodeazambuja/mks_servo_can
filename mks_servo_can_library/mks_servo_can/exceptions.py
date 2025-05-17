# mks_servo_can/mks_servo_can_library/mks_servo_can/exceptions.py
"""
Custom exceptions for the MKS Servo CAN library.
"""

class MKSServoError(Exception):
    """Base exception class for all MKS Servo library errors."""
    pass

class CANError(MKSServoError):
    """Exception related to CAN communication issues."""
    pass

class CRCError(MKSServoError):
    """Exception for CRC calculation or mismatch errors."""
    pass

class CommandError(MKSServoError):
    """Exception for errors related to sending or parsing commands."""
    pass

class ParameterError(MKSServoError):
    """Exception for invalid parameters provided to API functions."""
    pass

class MotorError(MKSServoError):
    """Exception for errors reported by the motor controller itself."""
    def __init__(self, message, error_code=None, can_id=None):
        super().__init__(message)
        self.error_code = error_code
        self.can_id = can_id

    def __str__(self):
        parts = [super().__str__()]
        if self.can_id is not None:
            parts.append(f"CAN ID: {self.can_id}")
        if self.error_code is not None:
            parts.append(f"Error Code: {self.error_code}")
        return " - ".join(parts)


class CommunicationError(MKSServoError):
    """General communication errors, e.g., timeouts."""
    pass

class MultiAxisError(MKSServoError):
    """Errors specific to the MultiAxisController operations."""
    def __init__(self, message, individual_errors=None):
        super().__init__(message)
        self.individual_errors = individual_errors or {}

    def __str__(self):
        base_msg = super().__str__()
        if self.individual_errors:
            err_details = "; ".join(f"Axis {axis_id}: {err}" for axis_id, err in self.individual_errors.items())
            return f"{base_msg} - Individual Errors: [{err_details}]"
        return base_msg

class SimulatorError(MKSServoError):
    """Errors related to the simulator's operation or communication with it."""
    pass

class KinematicsError(MKSServoError):
    """Errors related to kinematic calculations."""
    pass

class ConfigurationError(MKSServoError):
    """Errors related to library or hardware configuration."""
    pass

class HomingError(MotorError):
    """Errors specific to the homing process."""
    pass

class CalibrationError(MotorError):
    """Errors specific to the calibration process."""
    pass

class LimitError(MotorError):
    """Errors related to hitting a limit switch or software limits."""
    pass

class StallError(MotorError):
    """Errors related to motor stall detection."""
    pass