# mks_servo_can/mks_servo_simulator/mks_simulator/main.py
"""
Main entry point for running the MKS Servo CAN Simulator.
This simply calls the CLI's main function.
"""
from .cli import main as cli_main


def main():
    """Runs the command-line interface for the simulator."""
    cli_main()


if __name__ == "__main__":
    main()
