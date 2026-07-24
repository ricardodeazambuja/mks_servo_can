"""
MKS Servo CAN Simulator Package.

This package contains the components for a CLI-based simulator that mimics
the behavior of MKS SERVO42D/57D motors on a CAN bus. It allows testing
the mks-servo-can library without physical hardware.
"""

from .cli import main as run_simulator_cli
from .motor_model import SimulatedMotor
from .virtual_can_bus import VirtualCANBus

__version__ = "0.1.0"

__all__ = [
"SimulatedMotor",
"VirtualCANBus",
"run_simulator_cli",
]

# Nothing is printed on import. A banner here went to stdout, which in
# --json-output mode is the machine-readable event stream: any consumer doing
# json.loads() per line hit a decode error on the very first one. Importing a
# library should be silent regardless; here it is also a correctness matter.
