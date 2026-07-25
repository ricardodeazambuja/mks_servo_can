"""
MKS Servo CAN Simulator Package.

This package contains the components for a CLI-based simulator that mimics
the behavior of MKS SERVO42D/57D motors on a CAN bus. It allows testing
the mks-servo-can library without physical hardware.
"""

from mks_servo_can import __version__ as _library_version

from .cli import main as run_simulator_cli
from .motor_model import SimulatedMotor
from .virtual_can_bus import VirtualCANBus

# One version number for the whole distribution. The simulator ships as an extra
# of `mks-servo-can` and cannot be installed apart from it, so a version of its
# own could only ever disagree with the library's - which is exactly what the
# two setup.py files used to allow (0.1.0 against the library's 0.3.0).
__version__ = _library_version

__all__ = [
"SimulatedMotor",
"VirtualCANBus",
"run_simulator_cli",
]

# Nothing is printed on import. A banner here went to stdout, which in
# --json-output mode is the machine-readable event stream: any consumer doing
# json.loads() per line hit a decode error on the very first one. Importing a
# library should be silent regardless; here it is also a correctness matter.
