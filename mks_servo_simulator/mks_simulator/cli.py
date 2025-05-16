# mks_servo_can_project/mks_servo_simulator/mks_simulator/cli.py
"""
Command-Line Interface for the MKS Servo CAN Simulator.
Uses 'click' for CLI argument parsing and command structure.
"""
import asyncio
import logging
import signal

import click # Ensure 'click' is in your requirements for the simulator

from .virtual_can_bus import VirtualCANBus
from .motor_model import SimulatedMotor
# Assuming the main library's constants are accessible for defaults
try:
    from mks_servo_can_library.mks_servo_can import constants as lib_const
except ImportError:
    # Minimal fallback if library not in path
    class lib_const: # type: ignore
        ENCODER_PULSES_PER_REVOLUTION = 16384 # Default from MKS manual
        MOTOR_TYPE_SERVO42D = "SERVO42D"
        MOTOR_TYPE_SERVO57D = "SERVO57D"


# Basic logging setup for the simulator
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("MKSSimulatorCLI")


async def shutdown(sig, loop, server_task, bus):
    """Graceful shutdown for the simulator."""
    logger.info(f"Received exit signal {sig.name}...")
    logger.info("Shutting down simulated motors...")
    if bus:
        await bus.stop_all_motors() # Ensure motors stop their tasks

    if server_task and not server_task.done():
        logger.info("Cancelling server task...")
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            logger.info("Server task cancelled successfully.")
        except Exception as e:
            logger.error(f"Error during server task shutdown: {e}")

    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    if tasks:
        logger.info(f"Cancelling {len(tasks)} outstanding tasks...")
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        logger.info("All outstanding tasks cancelled.")

    logger.info("Flushing writers and stopping loop...")
    # Ensure all writers are flushed if possible (tricky to get all here)
    # loop.call_soon_threadsafe(loop.stop) # If run from different thread
    if loop.is_running():
        loop.stop()
    logger.info("Simulator shutdown complete.")


@click.command()
@click.option('--host', default='localhost', help='Host for the simulator server.', show_default=True)
@click.option('--port', default=6789, type=int, help='Port for the simulator server.', show_default=True)
@click.option('--num-motors', default=1, type=int, help='Number of simulated motors to create.', show_default=True)
@click.option('--start-can-id', default=1, type=int, help='Starting CAN ID for simulated motors.', show_default=True)
@click.option('--motor-type',
              type=click.Choice([lib_const.MOTOR_TYPE_SERVO42D, lib_const.MOTOR_TYPE_SERVO57D, "GENERIC"], case_sensitive=False),
              default="GENERIC", help='Type of motor to simulate.', show_default=True)
@click.option('--latency-ms', default=2.0, type=float, help='Simulated CAN bus latency in milliseconds (round trip).', show_default=True)
@click.option('--log-level', default='INFO', type=click.Choice(['DEBUG', 'INFO', 'WARNING', 'ERROR'], case_sensitive=False),
              help='Logging level for the simulator.', show_default=True)
@click.option('--steps-per-rev', default=lib_const.ENCODER_PULSES_PER_REVOLUTION, type=int,
              help="Encoder steps per revolution for simulated motors.", show_default=True)
def main(host: str, port: int, num_motors: int, start_can_id: int, motor_type: str,
         latency_ms: float, log_level: str, steps_per_rev: int):
    """
    MKS Servo CAN Simulator.

    This tool simulates one or more MKS servo motors on a virtual CAN bus,
    allowing the mks-servo-can library to connect and interact with them
    for testing and development without physical hardware.
    """
    numeric_log_level = getattr(logging, log_level.upper(), logging.INFO)
    # Update root logger level if CLI provides one, or specific simulator loggers
    logging.getLogger().setLevel(numeric_log_level) # Set root logger for all modules
    logger.setLevel(numeric_log_level) # Set CLI logger
    logging.getLogger("VirtualCANBus").setLevel(numeric_log_level)
    logging.getLogger("SimulatedMotor").setLevel(numeric_log_level)


    logger.info("Starting MKS Servo CAN Simulator...")
    logger.info(f"Config: Host={host}, Port={port}, NumMotors={num_motors}, StartID={start_can_id}, Type={motor_type}, Latency={latency_ms}ms")

    if num_motors < 1:
        logger.error("Number of motors must be at least 1.")
        return

    loop = asyncio.get_event_loop()
    bus = VirtualCANBus(loop)
    bus.set_latency(latency_ms) # Set global latency for the bus

    for i in range(num_motors):
        current_can_id = start_can_id + i
        if not (0 < current_can_id <= 0x7FF):
            logger.error(f"Calculated CAN ID {current_can_id} is out of valid range (1-2047). Skipping.")
            continue

        # Determine motor type for simulation details
        sim_motor_type_str = motor_type
        if motor_type.upper() == "GENERIC":
            sim_motor_type_str = lib_const.MOTOR_TYPE_SERVO42D # Default generic to 42D behavior

        motor = SimulatedMotor(
            can_id=current_can_id,
            loop=loop,
            motor_type=sim_motor_type_str,
            steps_per_rev_encoder=steps_per_rev
            # Add options for initial pos, limits if needed from CLI
        )
        bus.add_motor(motor)

    server_task = loop.create_task(bus.start_server(host, port))

    # Setup signal handlers for graceful shutdown
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        loop.add_signal_handler(
            s, lambda s=s: asyncio.create_task(shutdown(s, loop, server_task, bus))
        )

    try:
        logger.info("Simulator server running. Press Ctrl+C to stop.")
        loop.run_forever() # Will be stopped by shutdown()
    except KeyboardInterrupt: # Should be caught by signal handler mostly
        logger.info("KeyboardInterrupt received directly by CLI.")
    finally:
        logger.info("CLI main loop finalizing...")
        if not server_task.done(): # If loop.stop() was called before server_task cancelled
            server_task.cancel()
            # Await cancellation if loop is still running (it might not be)
            if loop.is_running():
                try:
                    loop.run_until_complete(server_task)
                except asyncio.CancelledError:
                    pass # Expected
        
        # Final cleanup for motors if shutdown wasn't fully completed by signal
        if loop.is_running(): # Ensure cleanup tasks run if loop was prematurely stopped
            loop.run_until_complete(bus.stop_all_motors())

        # Close the loop if it wasn't closed by shutdown
        if loop.is_running():
            loop.close() # Close the loop
        logger.info("Simulator CLI finished.")


if __name__ == '__main__':
    main()