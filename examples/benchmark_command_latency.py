# It's designed to measure the round-trip time (latency) of commands sent to MKS servo motors.
# This is useful for understanding system responsiveness and can be part of determinism testing.

"""
Example: Benchmarking command round-trip times with MKS Servo.
Can be used with either the simulator or real hardware.
This script helps in understanding the latency involved in sending a command
and receiving a response from an MKS servo motor.
"""
# This is a module-level docstring explaining the script's purpose:
# - It benchmarks command round-trip times (latency).
# - It can be used with both the simulator and real hardware.
# - It helps quantify the delay between sending a command and getting a response.

import argparse  # For command-line arguments
# Imports the 'argparse' module, which is used to create a command-line interface
# for the script, allowing users to specify parameters like whether to use the simulator,
# CAN settings, motor ID, and number of iterations.

import statistics  # For mean, median, stdev
# Imports the 'statistics' module, used to calculate statistical measures
# (mean, median, standard deviation) from the collected latency data.

import asyncio
# Imports the 'asyncio' library, as the 'mks_servo_can' library is asynchronous.

import logging
# Imports the 'logging' module for displaying informational messages and results.

import time
# Imports the 'time' module, specifically 'time.perf_counter()' for precise timing
# of command execution to measure latency.

# Assuming the library is installed or in PYTHONPATH
# This comment indicates that the 'mks_servo_can' library components need to be importable.
from mks_servo_can import Axis
# Imports the high-level 'Axis' class for motor control.

from mks_servo_can import CANInterface
# Imports 'CANInterface' for managing the CAN connection.

from mks_servo_can import const
# Imports library constants (e.g., default values, command codes).

from mks_servo_can import exceptions
# Imports custom exceptions for error handling.

from mks_servo_can import RotaryKinematics
# Imports 'RotaryKinematics' for setting up a basic Axis instance.

from mks_servo_can import LowLevelAPI
# Imports 'LowLevelAPI' to allow benchmarking direct low-level command calls
# in addition to higher-level Axis method calls.

# --- Configuration ---
# This section defines default values for various settings.
# These can be overridden by command-line arguments if the script is run from a terminal.
DEFAULT_USE_SIMULATOR = True
# By default, the benchmark will use the simulator.

DEFAULT_SIMULATOR_HOST = "localhost"
# Default hostname for the simulator.

DEFAULT_SIMULATOR_PORT = 6789
# Default TCP port for the simulator.

DEFAULT_SIMULATOR_LATENCY_MS = 2.0  # Default simulated bus latency for info
# This is an informational default, representing an expected latency if the simulator
# itself is configured with a 2ms latency. The script doesn't set the simulator's latency.

DEFAULT_CAN_INTERFACE_TYPE = (
    "canable"  # e.g., 'socketcan', 'kvaser', 'pcan', 'usb2can'
)
# Default CAN adapter type if using real hardware.

DEFAULT_CAN_CHANNEL = "/dev/ttyACM0"  # e.g., 'slcan0', 'can0', 'PCAN_USBBUS1'
# Default CAN adapter channel if using real hardware.

DEFAULT_CAN_BITRATE = 500000
# Default CAN bus bitrate.

DEFAULT_MOTOR_CAN_ID = 1
# Default CAN ID of the motor to be benchmarked.

DEFAULT_NUM_ITERATIONS = 100
# Default number of times each benchmarked command will be executed.

DEFAULT_LOG_LEVEL = "INFO"
# Default logging level for the script's output.

# Configure basic logging
logger = logging.getLogger("BenchmarkScript")
# Creates a logger instance named "BenchmarkScript".
# The actual logging level and format will be set later based on command-line arguments.

async def benchmark_read_position_low_level(
    low_level_api: "LowLevelAPI", can_id: int, iterations: int
) -> list[float]:
    # Defines an asynchronous function to benchmark reading the motor's position
    # using the 'LowLevelAPI' directly. This bypasses the 'Axis' class overhead.
    # It returns a list of latencies in milliseconds.
    """Benchmarks reading position using the low-level API directly."""
    # List to store latency values for each iteration.
    latencies = []
    logger.info(
        "Benchmarking Low-Level read_encoder_value_addition "
        "(%d iterations)...",
        iterations,
    )
    # Logs the start of this specific benchmark.

    # Ensure motor is enabled for responsiveness if that affects timing,
    # though read usually works regardless
    # This comment suggests that enabling the motor might be considered for consistent timing,
    # but reading position typically works even if the motor is disabled.
    # await low_level_api.enable_motor(can_id, True) # Might be needed

    for i in range(iterations):
        # Loop for the specified number of iterations.
        start_time = time.perf_counter()
        # Records the time just before sending the command. 'perf_counter' provides high-resolution timing.
        try:
            await low_level_api.read_encoder_value_addition(can_id)
            # Calls the low-level API method to read the encoder's accumulated value.
        except exceptions.MKSServoError as e:
            # Catches any library-specific errors during the read operation.
            logger.warning("Iteration %d: Low-level read error: %s", i + 1, e)
            # Logs a warning if an error occurs for an iteration.
            continue # Skips this iteration and proceeds to the next.
        end_time = time.perf_counter()
        # Records the time just after the command completes.
        latencies.append((end_time - start_time) * 1000)  # Convert to ms
        # Calculates the duration (latency) in milliseconds and adds it to the list.
        if (i + 1) % (iterations // 10 or 1) == 0:
            # Provides progress updates every 10% of iterations (or every iteration if less than 10 total).
            logger.debug(
                "Low-level read iteration %d/%d completed.", i + 1, iterations
            )
            # Logs progress at DEBUG level.
        await asyncio.sleep(0.001)  # Small delay between commands
        # Introduces a very short pause (1ms) between commands. This can help avoid
        # overwhelming the CAN bus or the motor, and gives a slightly more realistic
        # measure than back-to-back commands with zero delay.
    return latencies
    # Returns the list of collected latencies.

async def benchmark_axis_get_position(
    axis: "Axis", iterations: int
) -> list[float]:
    # Defines an asynchronous function to benchmark reading the motor's position
    # using the high-level 'Axis.get_current_position_steps()' method.
    # This includes any overhead from the Axis class itself.
    """Benchmarks reading position via Axis.get_current_position_steps()."""
    latencies = []
    # List to store latencies.
    logger.info(
        "Benchmarking Axis.get_current_position_steps (%d iterations)...",
        iterations,
    )
    # Logs the start of this benchmark.

    for i in range(iterations):
        # Loop for iterations.
        start_time = time.perf_counter()
        # Start timing.
        try:
            await axis.get_current_position_steps()
            # Calls the Axis method to get position in raw steps.
        except exceptions.MKSServoError as e:
            # Handles potential errors.
            logger.warning(
                "Iteration %d: Axis get_position error: %s", i + 1, e
            )
            continue # Skip to next iteration.
        end_time = time.perf_counter()
        # End timing.
        latencies.append((end_time - start_time) * 1000)  # ms
        # Store latency.
        if (i + 1) % (iterations // 10 or 1) == 0:
            # Log progress.
            logger.debug(
                "Axis get_position iteration %d/%d completed.",
                i + 1,
                iterations,
            )
        await asyncio.sleep(0.001) # Small delay.
    return latencies
    # Return latencies.

async def benchmark_axis_enable_disable(
    axis: "Axis", iterations: int
) -> list[float]:
    # Defines an asynchronous function to benchmark the time taken to enable
    # and then immediately disable the motor using Axis methods.
    # This measures the latency of two sequential state-changing commands.
    """Benchmarks enabling and then disabling the motor using Axis methods."""
    latencies = []
    logger.info(
        "Benchmarking Axis enable/disable cycle (%d iterations)...", iterations
    )
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.enable_motor() # Command to enable the motor.
            await axis.disable_motor() # Command to disable the motor.
        except exceptions.MKSServoError as e:
            logger.warning(
                "Iteration %d: Axis enable/disable error: %s", i + 1, e
            )
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                "Axis enable/disable iteration %d/%d completed.",
                i + 1,
                iterations,
            )
        await asyncio.sleep(0.005)  # Slightly longer pause after a cycle
        # A slightly longer delay to allow the motor state to settle if necessary.
    return latencies

async def benchmark_short_relative_move(
    axis: "Axis", iterations: int, move_pulses: int = 100
) -> list[float]:
    # Defines an asynchronous function to benchmark a short relative move.
    # It commands the motor to move by a small number of pulses and waits for completion.
    # This measures the latency of a complete motion command cycle.
    """Benchmarks a short relative move and waits for completion."""
    latencies = []
    logger.info(
        "Benchmarking Axis short relative move "
        "(%d pulses, %d iterations)...",
        move_pulses,
        iterations,
    )
    await axis.enable_motor() # Motor must be enabled to move.

    for i in range(iterations):
        try:
            start_time = time.perf_counter()
            # Commands a relative move. 'wait=True' ensures the call blocks until the move is reported complete.
            # Speed and acceleration parameters are provided.
            await axis.move_relative_pulses(
                move_pulses, speed_param=1000, accel_param=150, wait=True
            )
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms
            if (i + 1) % (iterations // 10 or 1) == 0:
                logger.debug(
                    "Axis short move iteration %d/%d completed.",
                    i + 1,
                    iterations,
                )
            await asyncio.sleep(0.01)  # Pause between move cycles
            # Allows a brief pause for the system/motor before the next move.
        except exceptions.MKSServoError as e:
            # Handles errors during the move.
            logger.warning("Iteration %d: Axis move error: %s", i + 1, e)
            await asyncio.sleep(0.1) # Longer pause if an error occurred.
            try:
                await axis.enable_motor() # Attempt to re-enable if an error disabled it.
            except Exception:  # pylint: disable=bare-except
                # Catches any error during re-enable attempt and ignores it to continue benchmark.
                pass
            continue # Skip this iteration.
        except asyncio.CancelledError:
            # Handles if the move task is cancelled externally (e.g., script shutdown).
            logger.warning("Iteration %d: Move cancelled.", i + 1)
            break # Exit the loop if cancelled.
    return latencies

def _parse_args() -> argparse.Namespace:
    # Defines a function to parse command-line arguments using 'argparse'.
    # This allows users to customize benchmark parameters without modifying the script.
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(
        description="MKS Servo CAN Command Latency Benchmarker."
    )
    # --- Hardware Mode Selection ---
    # Default behavior is to use the simulator. Use --hardware flag to enable real hardware.
    parser.add_argument(
        '--hardware', 
        action='store_true',
        help='Use real hardware instead of the simulator (default is to use simulator).'
    )
    # Simulator-specific arguments.
    parser.add_argument(
        "--simulator-host",
        default=DEFAULT_SIMULATOR_HOST,
        help="Simulator host.",
    )
    parser.add_argument(
        "--simulator-port",
        type=int,
        default=DEFAULT_SIMULATOR_PORT,
        help="Simulator port.",
    )
    parser.add_argument(
        "--simulator-latency-ms",
        type=float,
        default=DEFAULT_SIMULATOR_LATENCY_MS,
        help="Expected simulator round-trip latency (for info).",
    )
    # Real hardware CAN arguments.
    parser.add_argument(
        "--can-interface-type",
        default=DEFAULT_CAN_INTERFACE_TYPE,
        help="CAN interface type for python-can.",
    )
    parser.add_argument(
        "--can-channel", default=DEFAULT_CAN_CHANNEL,
        help="CAN interface channel."
    )
    parser.add_argument(
        "--can-bitrate",
        type=int,
        default=DEFAULT_CAN_BITRATE,
        help="CAN bus bitrate.",
    )
    # General benchmark arguments.
    parser.add_argument(
        "--motor-can-id",
        type=int,
        default=DEFAULT_MOTOR_CAN_ID,
        help="CAN ID of the target motor.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_NUM_ITERATIONS,
        help="Number of iterations for each benchmark.",
    )
    parser.add_argument(
        "--log-level",
        default=DEFAULT_LOG_LEVEL,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"], # Restricts input to these choices.
        help="Logging level.",
    )
    parser.add_argument(
        "--benchmarks",
        nargs="+", # Allows specifying one or more benchmark names.
        default=["read_low_level", "read_axis", "enable_disable", "short_move"], # Default set of benchmarks to run.
        choices=["read_low_level", "read_axis", "enable_disable", "short_move"], # Available benchmark choices.
        help="Specify which benchmarks to run.",
    )
    return parser.parse_args()
    # Returns the parsed arguments as a Namespace object.

async def _setup_can_interface(args: argparse.Namespace) -> CANInterface:
    # Asynchronous helper function to set up and connect the CANInterface
    # based on the parsed command-line arguments.
    """Sets up and connects the CAN interface based on arguments."""
    if not args.hardware:
        # If using the simulator, configure CANInterface for simulator mode.
        can_if = CANInterface(
            use_simulator=True,
            simulator_host=args.simulator_host,
            simulator_port=args.simulator_port,
        )
        # The C0301 (self.logger.info) Pylint error was for line 226 in the original file,
        # which is now 'logger.info'. This is fine.
        logger.info(
            "Using Simulator. Ensure it's running (e.g., with "
            "latency ~%.1fms).",
            args.simulator_latency_ms, # Logs the expected simulator latency for user awareness.
        )
    else:
        # If using real hardware, configure CANInterface with hardware parameters.
        can_if = CANInterface(
            interface_type=args.can_interface_type,
            channel=args.can_channel,
            bitrate=args.can_bitrate,
            use_simulator=False,
        )
        logger.info(
            "Using Real Hardware: %s on %s @ %s bps.",
            args.can_interface_type,
            args.can_channel,
            args.can_bitrate,
        )
    try:
        await can_if.connect() # Attempt to connect.
        logger.info("CAN Interface connected.")
        return can_if # Return the connected interface.
    except exceptions.MKSServoError as e:
        # If connection fails, log the error and re-raise to halt the script.
        logger.error("Failed to connect to CAN interface: %s", e)
        raise

async def _initialize_axis(
    can_if: CANInterface, args: argparse.Namespace
) -> Axis:
    # Asynchronous helper function to initialize the Axis instance.
    """Initializes the Axis instance."""
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION # Standard kinematics.
    )
    axis = Axis(
        can_interface_manager=can_if, # Uses the previously connected CANInterface.
        motor_can_id=args.motor_can_id, # Uses the specified motor CAN ID.
        name="BenchmarkAxis", # Assigns a name.
        kinematics=kin,
    )
    try:
        logger.info("Initializing Axis (CAN ID: %d)...", args.motor_can_id)
        await axis.initialize(calibrate=False, home=False) # Basic initialization.
        logger.info("Axis initialized.")
        return axis # Returns the initialized axis.
    except exceptions.MKSServoError as e:
        # If axis initialization fails, log and re-raise.
        logger.error(
            "Failed to initialize axis %d: %s", args.motor_can_id, e
        )
        raise

def _print_results(
    all_results: dict[str, list[float]], args: argparse.Namespace
):
    # Helper function to print the benchmark results in a formatted way.
    """Prints the benchmark results."""
    logger.info("\n--- BENCHMARK RESULTS (latencies in ms) ---")
    # Iterates through the results dictionary (benchmark_name -> list_of_latencies).
    for test_name, latencies in all_results.items():
        if not latencies:
            # If no data was collected for a benchmark (e.g., all iterations failed).
            logger.info(
                "%s: No data (all iterations failed or benchmark not run).",
                test_name,
            )
            continue

        # Calculate statistics for the collected latencies.
        count = len(latencies)
        mean_lat = statistics.mean(latencies)
        median_lat = statistics.median(latencies)
        min_lat = min(latencies)
        max_lat = max(latencies)
        std_dev = statistics.stdev(latencies) if count > 1 else 0.0 # Stddev requires at least 2 data points.

        # Print the formatted results.
        logger.info("\nResults for: %s", test_name)
        logger.info("  Iterations: %d/%d", count, args.iterations) # Shows successful vs. total iterations.
        logger.info("  Min     : %.3f ms", min_lat)
        logger.info("  Max     : %.3f ms", max_lat)
        logger.info("  Mean    : %.3f ms", mean_lat)
        logger.info("  Median  : %.3f ms", median_lat)
        logger.info("  Std.Dev.: %.3f ms", std_dev)
        if args.use_simulator:
            # If using the simulator, add a note about its configured latency.
            logger.info(
                "  (Simulator Note: Bus latency is configured within the "
                "simulator instance, e.g., --latency-ms %.1f for round trip)",
                args.simulator_latency_ms,
            )

async def _execute_selected_benchmarks(
    axis: Axis, low_level_api: LowLevelAPI, args: argparse.Namespace
) -> dict[str, list[float]]:
    # Asynchronous helper function to execute the benchmarks selected by the user
    # via command-line arguments.
    """Executes the selected benchmark functions."""
    all_results: dict[str, list[float]] = {} # Dictionary to store results from all run benchmarks.
    # Checks which benchmarks were specified in 'args.benchmarks' and runs them.
    if "read_low_level" in args.benchmarks:
        all_results["Low-Level Read Position"] = (
            await benchmark_read_position_low_level(
                low_level_api, args.motor_can_id, args.iterations
            )
        )
    if "read_axis" in args.benchmarks:
        all_results["Axis Read Position (get_current_position_steps)"] = (
            await benchmark_axis_get_position(axis, args.iterations)
        )
    if "enable_disable" in args.benchmarks:
        all_results["Axis Enable-Disable Cycle"] = (
            await benchmark_axis_enable_disable(axis, args.iterations)
        )
    if "short_move" in args.benchmarks:
        all_results["Axis Short Relative Move (100 pulses)"] = (
            await benchmark_short_relative_move(
                axis, args.iterations, move_pulses=100
            )
        )
    return all_results
    # Returns the dictionary of results.

async def run_benchmarks_main(args: argparse.Namespace):
    # The main asynchronous function that orchestrates the entire benchmark process.
    """Main function to set up and run the benchmarks."""
    log_level_numeric = getattr(logging, args.log_level.upper(), logging.INFO)
    # Sets the logging level based on command-line arguments.
    logging.basicConfig(
        level=log_level_numeric,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.setLevel(log_level_numeric) # Applies the level to the script's logger.

    can_if = None # Initialize for the finally block.
    axis = None   # Initialize for the finally block.
    try:
        # Setup CAN interface and Axis.
        can_if = await _setup_can_interface(args)
        axis = await _initialize_axis(can_if, args)

        # pylint: disable=protected-access
        # This comment acknowledges that accessing a "protected" member (_low_level_api)
        # is being done intentionally for the purpose of comparing its performance
        # directly against the higher-level Axis API.
        # Accessing _low_level_api for direct benchmarking is intentional here
        # to compare overheads.
        low_level_api = axis._low_level_api # Gets the LowLevelAPI instance from the Axis object.

        # Execute the selected benchmarks.
        all_results = await _execute_selected_benchmarks(
            axis, low_level_api, args
        )
        # Print the collected results.
        _print_results(all_results, args)

    except KeyboardInterrupt:
        # Handles graceful exit if the user interrupts the script (Ctrl+C).
        logger.info("Benchmark run interrupted by user.")
    except exceptions.MKSServoError:
        # Catches library-specific errors during setup or execution.
        logger.error("Exiting due to MKS Servo library error.")
    except Exception as e:  # pylint: disable=broad-except
        # Catches any other unexpected errors.
        logger.error(
            "An unexpected error occurred during benchmarking: %s",
            e,
            exc_info=True, # Includes traceback in the log.
        )
    finally:
        # This block ensures cleanup (disabling motor, disconnecting CAN) happens
        # regardless of whether the benchmarks completed successfully or an error occurred.
        logger.info("Cleaning up and disconnecting CAN interface...")
        if axis and axis.is_enabled(): # If axis exists and is enabled.
            try:
                await axis.disable_motor() # Attempt to disable the motor.
            except Exception as e:  # pylint: disable=broad-except
                # Log if disabling fails but don't let it prevent further cleanup.
                logger.warning(
                    "Could not disable motor during cleanup: %s", e
                )
        if can_if: # If CAN interface exists.
            await can_if.disconnect() # Disconnect from CAN.
        logger.info("Benchmarking finished.")

def main_cli():
    # This function is the entry point when the script is run from the command line.
    """Parses arguments and runs the benchmark asynchronous function."""
    args = _parse_args() # Parses command-line arguments.
    asyncio.run(run_benchmarks_main(args)) # Runs the main async benchmark orchestrator.

if __name__ == "__main__":
    # Standard Python idiom: if the script is executed directly (not imported),
    # call main_cli().
    main_cli()
    