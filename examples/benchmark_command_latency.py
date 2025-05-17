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
from mks_servo_can_library.mks_servo_can import Axis
from mks_servo_can_library.mks_servo_can import CANInterface
from mks_servo_can_library.mks_servo_can import const
from mks_servo_can_library.mks_servo_can import exceptions
from mks_servo_can_library.mks_servo_can import RotaryKinematics

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
        f"Benchmarking Low-Level read_encoder_value_addition ({iterations} iterations)..."
    )
    # Ensure motor is enabled for responsiveness if that affects timing, though read usually works regardless
    # await low_level_api.enable_motor(can_id, True) # Might be needed if motor sleeps

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await low_level_api.read_encoder_value_addition(can_id)
        except exceptions.MKSServoError as e:
            logger.warning(f"Iteration {i+1}: Low-level read error: {e}")
            # Optionally skip or record failure
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                f"Low-level read iteration {i+1}/{iterations} completed."
            )
        await asyncio.sleep(0.001)  # Small delay between commands
    return latencies


async def benchmark_axis_get_position(
    axis: "Axis", iterations: int
) -> list[float]:
    """Benchmarks reading position using the high-level Axis.get_current_position_steps()."""
    latencies = []
    logger.info(
        f"Benchmarking Axis.get_current_position_steps ({iterations} iterations)..."
    )
    # await axis.enable_motor() # Axis methods often handle enabling if needed

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.get_current_position_steps()
        except exceptions.MKSServoError as e:
            logger.warning(f"Iteration {i+1}: Axis get_position error: {e}")
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                f"Axis get_position iteration {i+1}/{iterations} completed."
            )
        await asyncio.sleep(0.001)
    return latencies


async def benchmark_axis_enable_disable(
    axis: "Axis", iterations: int
) -> list[float]:
    """Benchmarks enabling and then disabling the motor using Axis methods."""
    latencies = []
    logger.info(
        f"Benchmarking Axis enable/disable cycle ({iterations} iterations)..."
    )
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.enable_motor()
            # Add a tiny pause if the motor needs time to report enabled status before disable is effective
            # await asyncio.sleep(0.001)
            await axis.disable_motor()
        except exceptions.MKSServoError as e:
            logger.warning(f"Iteration {i+1}: Axis enable/disable error: {e}")
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)  # ms
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.debug(
                f"Axis enable/disable iteration {i+1}/{iterations} completed."
            )
        await asyncio.sleep(0.005)  # Slightly longer pause after a cycle
    return latencies


async def benchmark_short_relative_move(
    axis: "Axis", iterations: int, move_pulses: int = 100
) -> list[float]:
    """Benchmarks a short relative move and waits for completion."""
    latencies = []
    logger.info(
        f"Benchmarking Axis short relative move ({move_pulses} pulses, {iterations} iterations)..."
    )
    await axis.enable_motor()
    # Ensure axis is in a known state, e.g., set work mode if necessary
    # await axis.set_work_mode(const.MODE_SR_VFOC) # Or other suitable serial mode

    for i in range(iterations):
        # Move back and forth to avoid hitting limits and to have a consistent start
        try:
            start_time = time.perf_counter()
            await axis.move_relative_pulses(
                move_pulses, speed_param=1000, accel_param=150, wait=True
            )
            # await axis.move_relative_pulses(-move_pulses, speed_param=1000, accel_param=150, wait=True) # Move back
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms
            if (i + 1) % (iterations // 10 or 1) == 0:
                logger.debug(
                    f"Axis short move iteration {i+1}/{iterations} completed."
                )
            await asyncio.sleep(0.01)  # Pause between move cycles
        except exceptions.MKSServoError as e:
            logger.warning(f"Iteration {i+1}: Axis move error: {e}")
            await asyncio.sleep(0.1)  # Longer pause on error before retrying
            # Attempt to re-enable if error might have disabled it
            try:
                await axis.enable_motor()
            except:
                pass  # Ignore if re-enable fails
            continue
        except asyncio.CancelledError:
            logger.warning(f"Iteration {i+1}: Move cancelled.")
            break

    return latencies


async def run_benchmarks(args: argparse.Namespace):
    """Main function to set up and run the benchmarks."""
    log_level_numeric = getattr(logging, args.log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=log_level_numeric,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.setLevel(log_level_numeric)  # Set our specific logger level

    if args.use_simulator:
        can_if = CANInterface(
            use_simulator=True,
            simulator_host=args.simulator_host,
            simulator_port=args.simulator_port,
        )
        logger.info(
            f"Using Simulator. Ensure it's running (e.g., with latency ~{args.simulator_latency_ms}ms)."
        )
        # Note: The CANInterface itself doesn't know the simulator's configured latency.
        # The simulator's internal latency setting is what matters.
    else:
        can_if = CANInterface(
            interface_type=args.can_interface_type,
            channel=args.can_channel,
            bitrate=args.can_bitrate,
            use_simulator=False,
        )
        logger.info(
            f"Using Real Hardware: {args.can_interface_type} on {args.can_channel} @ {args.can_bitrate}bps."
        )

    try:
        await can_if.connect()
        logger.info("CAN Interface connected.")
    except exceptions.MKSServoError as e:
        logger.error(f"Failed to connect to CAN interface: {e}")
        return

    # Use default RotaryKinematics for the axis, steps_per_revolution won't affect raw pulse moves or reads
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
        logger.info(f"Initializing Axis (CAN ID: {args.motor_can_id})...")
        await axis.initialize(
            calibrate=False, home=False
        )  # Basic communication check
        logger.info("Axis initialized.")
    except exceptions.MKSServoError as e:
        logger.error(f"Failed to initialize axis {args.motor_can_id}: {e}")
        await can_if.disconnect()
        return

    all_results: dict[str, list[float]] = {}
    low_level_api = axis._low_level_api  # For direct low-level benchmarks

    try:
        # --- Run selected benchmarks ---
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

        # --- Analysis and Output ---
        logger.info("\n--- BENCHMARK RESULTS (latencies in ms) ---")
        for test_name, latencies in all_results.items():
            if not latencies:
                logger.info(
                    f"{test_name}: No data (all iterations failed or benchmark not run)."
                )
                continue

            count = len(latencies)
            mean_lat = statistics.mean(latencies)
            median_lat = statistics.median(latencies)
            min_lat = min(latencies)
            max_lat = max(latencies)
            std_dev = statistics.stdev(latencies) if count > 1 else 0.0

            logger.info(f"\nResults for: {test_name}")
            logger.info(f"  Iterations: {count}/{args.iterations}")
            logger.info(f"  Min     : {min_lat:.3f} ms")
            logger.info(f"  Max     : {max_lat:.3f} ms")
            logger.info(f"  Mean    : {mean_lat:.3f} ms")
            logger.info(f"  Median  : {median_lat:.3f} ms")
            logger.info(f"  Std.Dev.: {std_dev:.3f} ms")
            if args.use_simulator:
                logger.info(
                    f"  (Simulator Note: Bus latency is configured within the simulator instance, e.g., --latency-ms {args.simulator_latency_ms} for round trip)"
                )

    except KeyboardInterrupt:
        logger.info("Benchmark run interrupted by user.")
    except Exception as e:
        logger.error(
            f"An unexpected error occurred during benchmarking: {e}",
            exc_info=True,
        )
    finally:
        logger.info("Cleaning up and disconnecting CAN interface...")
        if (
            axis.is_enabled()
        ):  # Attempt to disable the motor if it was left enabled
            try:
                await axis.disable_motor()
            except Exception as e:
                logger.warning(f"Could not disable motor during cleanup: {e}")
        await can_if.disconnect()
        logger.info("Benchmarking finished.")


def main_cli():
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
        help="Expected simulator round-trip latency (for informational display).",
    )

    parser.add_argument(
        "--can-interface-type",
        default=DEFAULT_CAN_INTERFACE_TYPE,
        help="CAN interface type for python-can.",
    )
    parser.add_argument(
        "--can-channel",
        default=DEFAULT_CAN_CHANNEL,
        help="CAN interface channel.",
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

    args = parser.parse_args()

    # Forcing use_simulator to True if no real hardware args are changed from defaults
    # This is a bit heuristic, user should explicitly use --no-simulator for hardware.
    if (
        args.can_interface_type == DEFAULT_CAN_INTERFACE_TYPE
        and args.can_channel == DEFAULT_CAN_CHANNEL
        and not hasattr(args, "no_simulator_explicitly_set")
    ):  # A way to check if --no-simulator was used
        if not args.use_simulator and not (
            args.can_interface_type != DEFAULT_CAN_INTERFACE_TYPE
            or args.can_channel != DEFAULT_CAN_CHANNEL
        ):
            # If still default hardware and --no-simulator wasn't explicitly used, but --use-simulator is false (default state)
            # This logic is tricky. Better to make --use-simulator explicit or --hardware explicit.
            # For now, if --no-simulator is not used, and hardware params are default, assume sim.
            # Let's simplify: if --no-simulator is present, use_simulator becomes False. Otherwise, it's its default/set value.
            pass

    asyncio.run(run_benchmarks(args))


if __name__ == "__main__":
    main_cli()
