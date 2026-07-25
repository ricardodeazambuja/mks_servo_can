"""
Command-Line Interface for the MKS Servo CAN Simulator.
Uses 'click' for CLI argument parsing and command structure.
"""
import asyncio
import logging
import signal
import threading
from typing import TYPE_CHECKING, Optional

import click  # Ensure 'click' is in your requirements for the simulator

from .interface.config_manager import ConfigurationManager, LiveConfigurationInterface
from .interface.http_debug_server import DebugHTTPServer, JSONOutputHandler
from .interface.llm_debug_interface import LLMDebugInterface
from .motor_model import SimulatedMotor
from .virtual_can_bus import VirtualCANBus

# The Textual dashboard is imported inside the --textual-dashboard branch, not
# here. It is the legacy surface - the supported ones are the browser dashboard
# under --debug-api and --json-output - and `textual` is a heavy dependency that
# nothing else needs. Importing it at module scope made it a hard requirement of
# the whole simulator: a clean `pip install mks-servo-can[simulator]` produced a
# `mks-servo-simulator` command that died with ModuleNotFoundError before
# parsing a single argument, because `textual` was declared in no install
# requirement anywhere. Nothing caught it, because every environment that ran
# the tests had textual installed for tests/test_textual_dashboard.py.
if TYPE_CHECKING:
    from .interface.textual_dashboard import TextualDashboard

# Strong references to background tasks, so they are not garbage collected.
_keepalive_tasks: set = set()

# Basic logging setup for the simulator
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("MKSSimulatorCLI")

# Assuming the main library's constants are accessible for defaults
try:
    from mks_servo_can import constants as lib_const
except ImportError as exc:
    # Minimal fallback if library not in path
    logger.warning(f"Exception: {exc}")
    logger.warning("Bypassing the import...")
    class lib_const:  # type: ignore
        ENCODER_PULSES_PER_REVOLUTION = 16384  # Default from MKS manual
        MOTOR_TYPE_SERVO42D = "SERVO42D"
        MOTOR_TYPE_SERVO57D = "SERVO57D"


def _degrees_to_steps(degrees: Optional[float], steps_per_rev: int) -> Optional[int]:
    """
    Converts a configuration profile's angle into encoder counts.

    Profiles state positions and limits in degrees; `SimulatedMotor` works in
    encoder counts. Keeping the conversion in one named place stops the two
    from being wired together directly, which would read a +/-360 degree limit
    as +/-360 counts - under eight degrees.

    Args:
        degrees: Angle in degrees, or None.
        steps_per_rev: The motor's encoder counts per revolution.

    Returns:
        The equivalent count, or None if `degrees` was None.
    """
    if degrees is None:
        return None
    return int(round(degrees * steps_per_rev / 360.0))


async def shutdown(sig, loop, server_task, bus, debug_server_task=None, json_handler=None, textual_app=None, performance_monitor=None):
    """Graceful shutdown for the simulator."""
    logger.info(f"Received exit signal {sig.name}...")
    logger.info("Shutting down simulated motors...")
    if bus:
        await bus.stop_all_motors()  # Ensure motors stop their tasks

    if server_task and not server_task.done():
        logger.info("Cancelling server task...")
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            logger.info("Server task cancelled successfully.")
        except Exception as e:
            logger.error(f"Error during server task shutdown: {e}")

    if debug_server_task and not debug_server_task.done():
        logger.info("Cancelling debug server task...")
        debug_server_task.cancel()
        try:
            await debug_server_task
        except asyncio.CancelledError:
            logger.info("Debug server task cancelled successfully.")
        except Exception as e:
            logger.error(f"Error during debug server shutdown: {e}")

    if textual_app is not None:
        # The dashboard owns a loop on another thread, so it has to be asked to
        # exit from that thread rather than cancelled from this one.
        logger.info("Asking the Textual dashboard to exit...")
        try:
            textual_app.call_from_thread(textual_app.exit)
        except Exception as e:
            logger.debug("Textual dashboard did not exit cleanly: %s", e)

    if performance_monitor:
        logger.info("Stopping performance monitor...")
        performance_monitor.stop_monitoring()
        logger.info("Performance monitor stopped.")

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
@click.option(
    "--host",
    default="localhost",
    help="Host for the simulator server.",
    show_default=True,
)
@click.option(
    "--port",
    default=6789,
    type=int,
    help="Port for the simulator server.",
    show_default=True,
)
@click.option(
    "--num-motors",
    default=1,
    type=int,
    help="Number of simulated motors to create.",
    show_default=True,
)
@click.option(
    "--start-can-id",
    default=1,
    type=int,
    help="Starting CAN ID for simulated motors.",
    show_default=True,
)
@click.option(
    "--motor-type",
    type=click.Choice(
        [
            lib_const.MOTOR_TYPE_SERVO42D,
            lib_const.MOTOR_TYPE_SERVO57D,
            "GENERIC",
        ],
        case_sensitive=False,
    ),
    default="GENERIC",
    help="Type of motor to simulate.",
    show_default=True,
)
@click.option(
    "--latency-ms",
    default=2.0,
    type=float,
    help="Simulated CAN bus latency in milliseconds (round trip).",
    show_default=True,
)
@click.option(
    "--log-level",
    default="INFO",
    type=click.Choice(
        ["DEBUG", "INFO", "WARNING", "ERROR"], case_sensitive=False
    ),
    help="Logging level for the simulator.",
    show_default=True,
)
@click.option(
    "--steps-per-rev",
    default=lib_const.ENCODER_PULSES_PER_REVOLUTION,
    type=int,
    help="Encoder steps per revolution for simulated motors.",
    show_default=True,
)
@click.option(
    "--json-output",
    is_flag=True,
    help="Enable JSON output mode for LLM consumption.",
)
@click.option(
    "--debug-api",
    is_flag=True,
    help=(
        "Enable the HTTP debug API. Serves the browser dashboard at "
        "/dashboard and the same state as JSON at /status."
    ),
)
@click.option(
    "--debug-api-port",
    default=8765,
    type=int,
    help="Port for HTTP debug API server.",
    show_default=True,
)
@click.option(
    "--textual-dashboard",
    is_flag=True,
    help=(
        "Enable the Textual TUI dashboard (legacy; prefer --debug-api, which "
        "serves the browser dashboard at /dashboard)."
    ),
)
@click.option(
    "--refresh-rate",
    default=200,
    type=int,
    help="Dashboard refresh rate in milliseconds.",
    show_default=True,
)
@click.option(
    "--no-color",
    is_flag=True,
    help="Disable color output for compatibility.",
)
@click.option(
    "--config-profile",
    type=str,
    help="Load configuration from named profile.",
)
@click.option(
    "--save-config",
    type=str,
    help="Save current configuration as named profile.",
)
@click.option(
    "--config-dir",
    type=str,
    help="Directory for configuration files (default: ~/.mks_simulator_config).",
)
def main(
    host: str,
    port: int,
    num_motors: int,
    start_can_id: int,
    motor_type: str,
    latency_ms: float,
    log_level: str,
    steps_per_rev: int,
    json_output: bool,
    debug_api: bool,
    debug_api_port: int,
    textual_dashboard: bool,
    refresh_rate: int,
    no_color: bool,
    config_profile: Optional[str],
    save_config: Optional[str],
    config_dir: Optional[str],
):
    """
    MKS Servo CAN Simulator.

    This tool simulates one or more MKS servo motors on a virtual CAN bus,
    allowing the mks-servo-can library to connect and interact with them
    for testing and development without physical hardware.
    """
    numeric_log_level = getattr(logging, log_level.upper(), logging.INFO)
    # Update root logger level if CLI provides one, or specific simulator loggers
    logging.getLogger().setLevel(
        numeric_log_level
    )  # Set root logger for all modules
    logger.setLevel(numeric_log_level)  # Set CLI logger
    logging.getLogger("VirtualCANBus").setLevel(numeric_log_level)
    logging.getLogger("SimulatedMotor").setLevel(numeric_log_level)

    # logger.info calls below this point will be affected by the redirection if textual_dashboard is true.

    logger.info("Starting MKS Servo CAN Simulator...")
    logger.info(
        f"Config: Host={host}, Port={port}, NumMotors={num_motors}, StartID={start_can_id}, Type={motor_type}, Latency={latency_ms}ms"
    )

    if num_motors < 1:
        logger.error("Number of motors must be at least 1.")
        return

    # Initialize configuration management
    config_manager = ConfigurationManager(config_dir)
    live_config_interface: Optional[LiveConfigurationInterface] = None

    # Load configuration profile if specified
    if config_profile:
        loaded_config = config_manager.load_config(config_profile)
        if loaded_config:
            config_manager.current_config = loaded_config
            logger.info(f"Loaded configuration profile: {config_profile}")

            # Override CLI parameters with profile settings
            host = loaded_config.host
            port = loaded_config.port
            latency_ms = loaded_config.latency_ms
            refresh_rate = loaded_config.refresh_rate
            no_color = loaded_config.no_color
            json_output = loaded_config.json_output
            debug_api = loaded_config.debug_api
            textual_dashboard = getattr(loaded_config, 'textual_dashboard', False)

            # Use motors from profile
            num_motors = len(loaded_config.motors)
            logger.info(f"Using {num_motors} motors from profile configuration")
        else:
            logger.error(f"Failed to load configuration profile: {config_profile}")
            return
    else:
        # Create default configuration from CLI parameters
        current_config = config_manager.create_default_config(num_motors, start_can_id)
        current_config.host = host
        current_config.port = port
        current_config.latency_ms = latency_ms
        current_config.refresh_rate = refresh_rate
        current_config.no_color = no_color
        current_config.json_output = json_output
        current_config.debug_api = debug_api
        current_config.textual_dashboard = textual_dashboard
        config_manager.current_config = current_config

    # Final check and setup for Textual dashboard logging after all config is resolved
    # The 'textual_dashboard' variable now holds its definitive value.
    if textual_dashboard:
        # Check if a file handler for simulator.log has already been added to prevent duplication.
        # This is important if this main() function could somehow be re-entered or the CLI options re-parsed,
        # though standard Click usage makes this unlikely for a single run.
        root_logger = logging.getLogger()
        has_sim_log_handler = any(
            isinstance(h, logging.FileHandler) and "simulator.log" in getattr(h, 'baseFilename', '')
            for h in root_logger.handlers
        )

        if not has_sim_log_handler:
            file_handler = logging.FileHandler("simulator.log", mode="w")
            formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
            file_handler.setFormatter(formatter)
            # Set file handler to INFO to capture INFO and DEBUG (if root logger is set to DEBUG elsewhere)
            # Or set to DEBUG explicitly if you always want DEBUG in file regardless of global log_level for console.
            # For now, respecting the global numeric_log_level for the file handler's max level.
            file_handler.setLevel(min(numeric_log_level, logging.INFO)) # Ensure it captures at least INFO
            if numeric_log_level == logging.DEBUG: # If global is debug, file is also debug
                file_handler.setLevel(logging.DEBUG)

            root_logger.addHandler(file_handler)

            # Adjust existing console StreamHandlers
            for handler in root_logger.handlers:
                if isinstance(handler, logging.StreamHandler) and handler is not file_handler:
                    # Only elevate level of console handlers if they are currently set to INFO or DEBUG
                    if handler.level < logging.WARNING:
                        handler.setLevel(logging.WARNING) # Restrict console to WARNING and above

            logger.info("Textual dashboard active. Logging detailed messages to simulator.log. Console will show WARNINGs and above.")
        else:
            logger.info("Textual dashboard active and file logger already configured.")


    loop = asyncio.get_event_loop()
    bus = VirtualCANBus(loop)
    bus.set_latency(latency_ms)  # Set global latency for the bus

    # Create live configuration interface
    live_config_interface = LiveConfigurationInterface(config_manager, bus)

    # Initialize debug interface and optional components
    debug_interface: Optional[LLMDebugInterface] = None
    debug_server: Optional[DebugHTTPServer] = None
    debug_server_task: Optional[asyncio.Task] = None
    json_handler: Optional[JSONOutputHandler] = None
    textual_app: Optional[TextualDashboard] = None

    # Create motors based on configuration
    if config_profile and config_manager.current_config:
        # Create motors from configuration profile
        for motor_config in config_manager.current_config.motors:
            if not (0 < motor_config.can_id <= 0x7FF):
                logger.error(
                    f"Motor CAN ID {motor_config.can_id} is out of valid range (1-2047). Skipping."
                )
                continue

            # Determine motor type for simulation details
            sim_motor_type_str = motor_config.motor_type
            if motor_config.motor_type.upper() == "GENERIC":
                sim_motor_type_str = lib_const.MOTOR_TYPE_SERVO42D

            # Only the fields SimulatedMotor actually models are passed here.
            # This branch previously forwarded max_current, max_speed and
            # initial_position as constructor arguments; none of the three
            # exist on SimulatedMotor, so --config-profile raised TypeError on
            # every invocation. max_speed has no equivalent at all - a real
            # motor's speed ceiling comes from its work mode, not from a
            # per-motor limit - so it is deliberately not mapped.
            #
            # MotorConfig expresses positions in degrees (its limits default to
            # +/-360) while SimulatedMotor takes encoder counts, so the two
            # cannot be connected directly: -360 passed through unconverted
            # would mean a limit of 7.9 degrees.
            steps_per_rev = motor_config.steps_per_rev
            position_limits = motor_config.position_limits or {}
            motor = SimulatedMotor(
                can_id=motor_config.can_id,
                loop=loop,
                motor_type=sim_motor_type_str,
                initial_pos_steps=(
                    _degrees_to_steps(motor_config.initial_position, steps_per_rev) or 0
                ),
                steps_per_rev_encoder=steps_per_rev,
                min_pos_limit_steps=_degrees_to_steps(
                    position_limits.get("min"), steps_per_rev
                ),
                max_pos_limit_steps=_degrees_to_steps(
                    position_limits.get("max"), steps_per_rev
                ),
            )
            motor.working_current_ma = motor_config.max_current

            if motor_config.enable_on_start:
                motor.is_enabled = True

            bus.add_motor(motor)
    else:
        # Create motors from CLI parameters (original logic)
        for i in range(num_motors):
            current_can_id = start_can_id + i
            if not (0 < current_can_id <= 0x7FF):
                logger.error(
                    f"Calculated CAN ID {current_can_id} is out of valid range (1-2047). Skipping."
                )
                continue

            # Determine motor type for simulation details
            sim_motor_type_str = motor_type
            if motor_type.upper() == "GENERIC":
                sim_motor_type_str = (
                    lib_const.MOTOR_TYPE_SERVO42D
                )  # Default generic to 42D behavior

            motor = SimulatedMotor(
                can_id=current_can_id,
                loop=loop,
                motor_type=sim_motor_type_str,
                steps_per_rev_encoder=steps_per_rev,
                # Add options for initial pos, limits if needed from CLI
            )
            bus.add_motor(motor)

    server_task = loop.create_task(bus.start_server(host, port))

    # Initialize LLM debug interface if needed
    if json_output or debug_api or textual_dashboard:
        debug_interface = LLMDebugInterface(bus.simulated_motors, bus)

        # Set up debug interface in the bus for command tracking
        bus.debug_interface = debug_interface

        # Initialize performance monitoring (will be started after loop is available)
        from .interface.performance_monitor import PerformanceMonitor
        performance_monitor = PerformanceMonitor(bus, debug_interface)
        bus.performance_monitor = performance_monitor
        # Note: performance_monitor.start_monitoring() will be called after loop setup

        logger.info("Performance monitoring initialized")

        if json_output:
            json_handler = JSONOutputHandler(debug_interface)
            config = {
                "host": host,
                "port": port,
                "num_motors": num_motors,
                "motor_type": motor_type,
                "latency_ms": latency_ms
            }
            json_handler.emit_startup(config)

            # Start periodic updates
            # Keep a reference: a bare create_task() may be garbage collected
            # mid-flight, which silently stops the periodic updates.
            _periodic_update_task = loop.create_task(
                json_handler.run_periodic_updates()
            )
            _keepalive_tasks.add(_periodic_update_task)
            _periodic_update_task.add_done_callback(_keepalive_tasks.discard)

        if debug_api:
            try:
                debug_server = DebugHTTPServer(
                    debug_interface,
                    debug_api_port,
                    "127.0.0.1",
                    config_manager=config_manager,
                    live_config=live_config_interface
                )
                debug_server_task = loop.create_task(debug_server.start_server())
                logger.info(f"Debug API server starting on http://127.0.0.1:{debug_api_port}")
                logger.info(
                    "Dashboard (for humans): http://127.0.0.1:%d/dashboard",
                    debug_api_port,
                )
                logger.info(
                    "Status JSON (for agents): http://127.0.0.1:%d/status",
                    debug_api_port,
                )
                logger.info(f"API documentation available at http://127.0.0.1:{debug_api_port}/docs")
                logger.info("Configuration management endpoints available at /config/*")
            except ImportError as e:
                logger.error(f"Failed to start debug API server: {e}")
                logger.error("Install FastAPI and uvicorn: pip install fastapi uvicorn")

        # Textual dashboard. Legacy: the browser dashboard at /dashboard is the
        # supported human surface, and --json-output the machine one.
        if textual_dashboard:
            try:
                logger.info("Starting Textual dashboard...")
                try:
                    # Aliased so it does not shadow the TYPE_CHECKING import of
                    # the same name, which the annotation above refers to.
                    from .interface.textual_dashboard import (
                        TextualDashboard as _TextualDashboard,
                    )
                except ImportError as exc:
                    # Say what to install and carry on. The CAN side and the
                    # debug API are unaffected by the TUI being unavailable, and
                    # exiting here would take them down with it.
                    raise ImportError(
                        "the Textual dashboard needs the 'dashboard' extra: "
                        "pip install mks-servo-can[dashboard]"
                    ) from exc
                textual_app = _TextualDashboard(bus)

                # On its own thread, with its own event loop. Sharing the
                # simulator's loop put the TUI's render and input handling in
                # direct competition with the 10 ms motor integration tick, so
                # the thing being measured was slowed down by the act of
                # watching it.
                textual_thread = threading.Thread(
                    target=textual_app.run,
                    name="textual-dashboard",
                    daemon=True,
                )
                textual_thread.start()

                logger.info("Textual dashboard started on its own thread")

            except Exception as e:
                logger.error(f"Failed to start textual dashboard: {e}")

    # Start performance monitoring if initialized
    if 'performance_monitor' in locals() and performance_monitor:
        performance_monitor.start_monitoring(loop)
        logger.info("Performance monitoring started")

    # Setup signal handlers for graceful shutdown
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        loop.add_signal_handler(
            s,
            lambda s=s: asyncio.create_task(
                shutdown(s, loop, server_task, bus, debug_server_task, json_handler, textual_app, performance_monitor if 'performance_monitor' in locals() else None)
            ),
        )

    try:
        if json_output:
            logger.info("Simulator running in JSON output mode. Press Ctrl+C to stop.")
        elif debug_api:
            logger.info(f"Simulator running with debug API on port {debug_api_port}. Press Ctrl+C to stop.")
        else:
            logger.info("Simulator server running. Press Ctrl+C to stop.")
        loop.run_forever()  # Will be stopped by shutdown()
    except KeyboardInterrupt:  # Should be caught by signal handler mostly
        logger.info("KeyboardInterrupt received directly by CLI.")
    finally:
        logger.info("CLI main loop finalizing...")
        if (
            not server_task.done()
        ):  # If loop.stop() was called before server_task cancelled
            server_task.cancel()
            # Await cancellation if loop is still running (it might not be)
            if loop.is_running():
                try:
                    loop.run_until_complete(server_task)
                except asyncio.CancelledError:
                    pass  # Expected

        # Clean up debug server if running
        if debug_server_task and not debug_server_task.done():
            debug_server_task.cancel()
            if loop.is_running():
                try:
                    loop.run_until_complete(debug_server_task)
                except asyncio.CancelledError:
                    pass  # Expected

        # Clean up the textual dashboard if running. Its thread is a daemon, so
        # it cannot hold the process open; this just gives it the chance to
        # restore the terminal.
        if textual_app is not None:
            try:
                textual_app.call_from_thread(textual_app.exit)
            except Exception:
                pass

        # Final cleanup for motors if shutdown wasn't fully completed by signal
        if (
            loop.is_running()
        ):  # Ensure cleanup tasks run if loop was prematurely stopped
            loop.run_until_complete(bus.stop_all_motors())

        # Close the loop if it wasn't closed by shutdown
        if loop.is_running():
            loop.close()  # Close the loop
        logger.info("Simulator CLI finished.")

    # Save configuration if requested
    if save_config and config_manager.current_config:
        success = config_manager.save_config(config_manager.current_config, save_config)
        if success:
            logger.info(f"Configuration saved as profile: {save_config}")
        else:
            logger.error(f"Failed to save configuration profile: {save_config}")


if __name__ == "__main__":
    main()
