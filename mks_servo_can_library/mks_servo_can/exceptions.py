"""
Custom exceptions for the MKS Servo CAN library.
"""


class MKSServoError(Exception):
    """Base exception class for all MKS Servo library errors."""
    def __init__(self, message, *args, error_code=None, can_id=None, **kwargs):
        """
        Initializes the base MKS Servo library error.

        Args:
            message (str): The primary error message.
            *args: Additional positional arguments for the base Exception.
            error_code (Optional[int]): An optional error code associated with the error.
            can_id (Optional[int]): The CAN ID of the motor involved, if applicable.
            **kwargs: Additional keyword arguments.
        """
        super().__init__(message, *args)  # Pass message and any other positional args to parent Exception
        self.message = message  # Store message for direct access if needed
        self.error_code = error_code
        self.can_id = can_id
        # You could store other kwargs if necessary:
        # self.additional_kwargs = kwargs

    def __str__(self):
        """
        Returns a string representation of the error, including CAN ID and error code if available.
        """
        # Get the base message from Exception's __str__
        base_message = super().__str__()

        details = []
        if self.can_id is not None:
            details.append(f"CAN ID: {self.can_id}")
        if self.error_code is not None:
            details.append(f"Error Code: {self.error_code}")
        # Example for additional kwargs:
        # if hasattr(self, 'additional_kwargs') and self.additional_kwargs:
        #    details.extend([f"{k}: {v}" for k,v in self.additional_kwargs.items()])

        if details:
            return f"{base_message} ({', '.join(details)})"
        return base_message



class CANError(MKSServoError):
    """Exception related to CAN communication issues."""



class CRCError(MKSServoError):
    """Exception for CRC calculation or mismatch errors."""



class CommandError(MKSServoError):
    """Exception for errors related to sending or parsing commands."""



class ParameterError(MKSServoError):
    """Exception for invalid parameters provided to API functions."""



class MotorError(MKSServoError):
    """Exception for errors reported by the motor controller itself."""

    def __init__(self, message, error_code=None, can_id=None):
        """
        Initializes a motor-specific error.

        Args:
            message (str): The primary error message.
            error_code (Optional[int]): The error code reported by the motor, if any.
            can_id (Optional[int]): The CAN ID of the motor that reported the error.
        """
        # Call the enhanced MKSServoError constructor which now handles these keywords
        super().__init__(message, error_code=error_code, can_id=can_id)
        # self.message, self.error_code, and self.can_id are now set by the parent.

    def __str__(self):
        """
        Returns a string representation of the motor error, including CAN ID and error code.
        """
        # To maintain MotorError's specific " - " formatting:
        parts = [self.message]  # Access self.message set by MKSServoError
        if self.can_id is not None: # Access self.can_id set by MKSServoError
            parts.append(f"CAN ID: {self.can_id}")
        if self.error_code is not None: # Access self.error_code set by MKSServoError
            parts.append(f"Error Code: {self.error_code}")

        # Only join if there are details beyond the base message
        if len(parts) > 1 and any(p != self.message for p in parts[1:]):
            return " - ".join(parts)
        return self.message # Return just the message if no additional details


class CommunicationError(MKSServoError):
    """General communication errors, e.g., timeouts."""



class MultiAxisError(MKSServoError):
    """Errors specific to the MultiAxisController operations."""

    def __init__(self, message, individual_errors=None):
        """
        Initializes an error related to multi-axis operations.

        Args:
            message (str): The primary error message.
            individual_errors (Optional[dict]): A dictionary where keys are axis identifiers
                                                 (e.g., names or CAN IDs) and values are
                                                 the specific errors related to that axis.
        """
        super().__init__(message)
        self.individual_errors = individual_errors or {}

    def __str__(self):
        """
        Returns a string representation of the multi-axis error,
        including details of individual axis errors if present.
        """
        base_msg = super().__str__()
        if self.individual_errors:
            err_details = "; ".join(
                f"Axis {axis_id}: {err}"
                for axis_id, err in self.individual_errors.items()
            )
            return f"{base_msg} - Individual Errors: [{err_details}]"
        return base_msg


class SimulatorError(MKSServoError):
    """Errors related to the simulator's operation or communication with it."""



class KinematicsError(MKSServoError):
    """Errors related to kinematic calculations."""



class ConfigurationError(MKSServoError):
    """Errors related to library or hardware configuration."""



class HomingError(MotorError):
    """Errors specific to the homing process."""



class CalibrationError(MotorError):
    """Errors specific to the calibration process."""



class LimitError(MotorError):
    """Errors related to hitting a limit switch or software limits."""



class StallError(MotorError):
    """Errors related to motor stall detection."""
    