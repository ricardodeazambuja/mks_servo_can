# mks_servo_can/tests/determinism/benchmark_command_latency.py
# (Also aliased or intended to be examples/timing_benchmark.py)
"""
Example: Benchmarking command round-trip times with MKS Servo.
Can be used with either the simulator or real hardware.
This script helps in understanding the latency involved in sending a command
and receiving a response from an MKS servo motor.
"""
import argparse  # For command-line arguments
import statistics  # For mean, median, stdev

import asyncio
import logging
import time

# Assuming the library is installed or in PYTHONPATH
from mks_servo_can import Axis
from mks_servo_can import CANInterface
from mks_servo_can import const
from mks_servo_can import exceptions
from mks_servo_can import RotaryKinematics
from mks_servo_can import LowLevelAPI

# --- Configuration ---
# These can be overridden by command-line arguments
DEFAULT_USE_SIMULATOR = True
DEFAULT_SIMULATOR_HOST = "localhost"
DEFAULT_SIMULATOR_PORT = 6789
DEFAULT_SIMULATOR_LATENCY_MS = 2.0  # Default simulated bus latency for info

DEFAULT_CAN_INTERFACE_TYPE = (
    "canable"  # e.g., 'socketcan', 'kvaser', 'pcan', 'usb2can'
)
DEFAULT_CAN_CHANNEL = "/dev/ttyACM0"  # e.g., 'slcan0', 'can0', 'PCAN_USBBUS1'
DEFAULT_CAN_BITRATE = 500000

DEFAULT_MOTOR_CAN_ID = 1
DEFAULT_NUM_ITERATIONS = 100
DEFAULT_LOG_LEVEL = "INFO"

# Configure basic logging
logger = logging.getLogger("BenchmarkScript")


async def benchmark_read_position_low_level(
    low_level_api: "LowLevelAPI", can_id: int, iterations: int
) -> list[float]:
    """Benchmarks reading position using the low-level API directly."""
    latencies = []
    logger.info(
        "Benchmarking Low-Level read_encoder_value_addition "
        "(%d iterations)...",
        iterations,
    )
    # Ensure motor is enabled for responsiveness if that affects timing,
    # though read usually works regardless
    # await low_level_api.enable_motor(can_id, True) # Might be needed

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await low_level_api.read_encoder_value_addition(can_id)
        except exceptions.MKSServoError as e:
            logger.warning("Iteration %d: Low-level read error: %s", i + 1, e)
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                "Low-level read iteration %d/%d completed.", i + 1, iterations
            )
        await asyncio.sleep(0.001)  # Small delay between commands
    return latencies


async def benchmark_axis_get_position(
    axis: "Axis", iterations: int
) -> list[float]:
    """Benchmarks reading position via Axis.get_current_position_steps()."""
    latencies = []
    logger.info(
        "Benchmarking Axis.get_current_position_steps (%d iterations)...",
        iterations,
    )

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.get_current_position_steps()
        except exceptions.MKSServoError as e:
            logger.warning(
                "Iteration %d: Axis get_position error: %s", i + 1, e
            )
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                "Axis get_position iteration %d/%d completed.",
                i + 1,
                iterations,
            )
        await asyncio.sleep(0.001)
    return latencies


async def benchmark_axis_enable_disable(
    axis: "Axis", iterations: int
) -> list[float]:
    """Benchmarks enabling and then disabling the motor using Axis methods."""
    latencies = []
    logger.info(
        "Benchmarking Axis enable/disable cycle (%d iterations)...", iterations
    )
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.enable_motor()
            await axis.disable_motor()
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
    return latencies


async def benchmark_short_relative_move(
    axis: "Axis", iterations: int, move_pulses: int = 100
) -> list[float]:
    """Benchmarks a short relative move and waits for completion."""
    latencies = []
    logger.info(
        "Benchmarking Axis short relative move "
        "(%d pulses, %d iterations)...",
        move_pulses,
        iterations,
    )
    await axis.enable_motor()

    for i in range(iterations):
        try:
            start_time = time.perf_counter()
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
        except exceptions.MKSServoError as e:
            logger.warning("Iteration %d: Axis move error: %s", i + 1, e)
            await asyncio.sleep(0.1)
            try:
                await axis.enable_motor()
            except Exception:  # pylint: disable=bare-except
                pass
            continue
        except asyncio.CancelledError:
            logger.warning("Iteration %d: Move cancelled.", i + 1)
            break
    return latencies


def _parse_args() -> argparse.Namespace:
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(
        description="MKS Servo CAN Command Latency Benchmarker."
    )
    parser.add_argument(
        "--use-simulator",
        action="store_true",
        default=DEFAULT_USE_SIMULATOR,
        help="Use the simulator instead of real hardware.",
    )
    parser.add_argument(
        "--no-simulator",
        action="store_false",
        dest="use_simulator",
        help="Use real hardware.",
    )
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
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level.",
    )
    parser.add_argument(
        "--benchmarks",
        nargs="+",
        default=["read_low_level", "read_axis", "enable_disable", "short_move"],
        choices=["read_low_level", "read_axis", "enable_disable", "short_move"],
        help="Specify which benchmarks to run.",
    )
    return parser.parse_args()


async def _setup_can_interface(args: argparse.Namespace) -> CANInterface:
    """Sets up and connects the CAN interface based on arguments."""
    if args.use_simulator:
        can_if = CANInterface(
            use_simulator=True,
            simulator_host=args.simulator_host,
            simulator_port=args.simulator_port,
        )
        # The C0301 on line 226 was here.
        logger.info(
            "Using Simulator. Ensure it's running (e.g., with "
            "latency ~%.1fms).",
            args.simulator_latency_ms,
        )
    else:
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
        await can_if.connect()
        logger.info("CAN Interface connected.")
        return can_if
    except exceptions.MKSServoError as e:
        logger.error("Failed to connect to CAN interface: %s", e)
        raise


async def _initialize_axis(
    can_if: CANInterface, args: argparse.Namespace
) -> Axis:
    """Initializes the Axis instance."""
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )
    axis = Axis(
        can_interface_manager=can_if,
        motor_can_id=args.motor_can_id,
        name="BenchmarkAxis",
        kinematics=kin,
    )
    try:
        logger.info("Initializing Axis (CAN ID: %d)...", args.motor_can_id)
        await axis.initialize(calibrate=False, home=False)
        logger.info("Axis initialized.")
        return axis
    except exceptions.MKSServoError as e:
        logger.error(
            "Failed to initialize axis %d: %s", args.motor_can_id, e
        )
        raise


def _print_results(
    all_results: dict[str, list[float]], args: argparse.Namespace
):
    """Prints the benchmark results."""
    logger.info("\n--- BENCHMARK RESULTS (latencies in ms) ---")
    for test_name, latencies in all_results.items():
        if not latencies:
            logger.info(
                "%s: No data (all iterations failed or benchmark not run).",
                test_name,
            )
            continue

        count = len(latencies)
        mean_lat = statistics.mean(latencies)
        median_lat = statistics.median(latencies)
        min_lat = min(latencies)
        max_lat = max(latencies)
        std_dev = statistics.stdev(latencies) if count > 1 else 0.0

        logger.info("\nResults for: %s", test_name)
        logger.info("  Iterations: %d/%d", count, args.iterations)
        logger.info("  Min     : %.3f ms", min_lat)
        logger.info("  Max     : %.3f ms", max_lat)
        logger.info("  Mean    : %.3f ms", mean_lat)
        logger.info("  Median  : %.3f ms", median_lat)
        logger.info("  Std.Dev.: %.3f ms", std_dev)
        if args.use_simulator:
            logger.info(
                "  (Simulator Note: Bus latency is configured within the "
                "simulator instance, e.g., --latency-ms %.1f for round trip)",
                args.simulator_latency_ms,
            )


async def _execute_selected_benchmarks(
    axis: Axis, low_level_api: LowLevelAPI, args: argparse.Namespace
) -> dict[str, list[float]]:
    """Executes the selected benchmark functions."""
    all_results: dict[str, list[float]] = {}
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


async def run_benchmarks_main(args: argparse.Namespace):
    """Main function to set up and run the benchmarks."""
    log_level_numeric = getattr(logging, args.log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=log_level_numeric,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.setLevel(log_level_numeric)

    can_if = None
    axis = None
    try:
        can_if = await _setup_can_interface(args)
        axis = await _initialize_axis(can_if, args)

        # pylint: disable=protected-access
        # Accessing _low_level_api for direct benchmarking is intentional here
        # to compare overheads.
        low_level_api = axis._low_level_api

        all_results = await _execute_selected_benchmarks(
            axis, low_level_api, args
        )
        _print_results(all_results, args)

    except KeyboardInterrupt:
        logger.info("Benchmark run interrupted by user.")
    except exceptions.MKSServoError:
        logger.error("Exiting due to MKS Servo library error.")
    except Exception as e:  # pylint: disable=broad-except
        logger.error(
            "An unexpected error occurred during benchmarking: %s",
            e,
            exc_info=True,
        )
    finally:
        logger.info("Cleaning up and disconnecting CAN interface...")
        if axis and axis.is_enabled():
            try:
                await axis.disable_motor()
            except Exception as e:  # pylint: disable=broad-except
                logger.warning(
                    "Could not disable motor during cleanup: %s", e
                )
        if can_if:
            await can_if.disconnect()
        logger.info("Benchmarking finished.")


def main_cli():
    """Parses arguments and runs the benchmark asynchronous function."""
    args = _parse_args()
    asyncio.run(run_benchmarks_main(args))


if __name__ == "__main__":
    main_cli()
    