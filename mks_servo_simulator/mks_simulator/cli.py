"""
Command-Line Interface for the MKS Servo CAN Simulator.
Uses 'click' for CLI argument parsing and command structure.
"""
import asyncio
import click  # Ensure 'click' is in your requirements for the simulator
import logging
import signal
import json
from typing import Optional

from .motor_model import SimulatedMotor
from .virtual_can_bus import VirtualCANBus
from .interface.llm_debug_interface import LLMDebugInterface
from .interface.http_debug_server import DebugHTTPServer, JSONOutputHandler
from .interface.textual_dashboard import TextualDashboard
from .interface.config_manager import ConfigurationManager, LiveConfigurationInterface

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
    logger.warning(f"Bypassing the import...")
    class lib_const:  # type: ignore
        ENCODER_PULSES_PER_REVOLUTION = 16384  # Default from MKS manual
        MOTOR_TYPE_SERVO42D = "SERVO42D"
        MOTOR_TYPE_SERVO57D = "SERVO57D"


async def shutdown(sig, loop, server_task, bus, debug_server_task=None, json_handler=None, textual_dashboard_task=None, performance_monitor=None):
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
    
    if textual_dashboard_task and not textual_dashboard_task.done():
        logger.info("Cancelling Textual dashboard task...")
        textual_dashboard_task.cancel()
        try:
            await textual_dashboard_task
        except asyncio.CancelledError:
            logger.info("Textual dashboard task cancelled successfully.")
        except Exception as e:
            logger.error(f"Error during Textual dashboard task shutdown: {e}")
    
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
    help="Enable HTTP debug API server for programmatic access.",
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
    help="Enable Textual TUI dashboard (experimental).",
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
    textual_dashboard_task: Optional[asyncio.Task] = None # Initialize textual_dashboard_task

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

            motor = SimulatedMotor(
                can_id=motor_config.can_id,
                loop=loop,
                motor_type=sim_motor_type_str,
                steps_per_rev_encoder=motor_config.steps_per_rev,
                max_current=motor_config.max_current,
                max_speed=motor_config.max_speed,
                initial_position=motor_config.initial_position,
            )
            
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
            loop.create_task(json_handler.run_periodic_updates())
        
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
                logger.info(f"API documentation available at http://127.0.0.1:{debug_api_port}/docs")
                logger.info("Configuration management endpoints available at /config/*")
            except ImportError as e:
                logger.error(f"Failed to start debug API server: {e}")
                logger.error("Install FastAPI and uvicorn: pip install fastapi uvicorn")
        
        # Textual Dashboard (Phase 2 test)  
        if textual_dashboard:
            try:
                logger.info("Starting Textual dashboard...")
                textual_app = TextualDashboard(bus)
                
                # Run textual asynchronously
                textual_dashboard_task = loop.create_task(textual_app.run_async())
                
                logger.info("Textual dashboard task created")
                
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
                shutdown(s, loop, server_task, bus, debug_server_task, json_handler, textual_dashboard_task if 'textual_dashboard_task' in locals() else None, performance_monitor if 'performance_monitor' in locals() else None)
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
        
        # Clean up textual dashboard if running
        if 'textual_dashboard_task' in locals() and textual_dashboard_task and not textual_dashboard_task.done():
            textual_dashboard_task.cancel()
            if loop.is_running():
                try:
                    loop.run_until_complete(textual_dashboard_task)
                except asyncio.CancelledError:
                    pass # Expected

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
