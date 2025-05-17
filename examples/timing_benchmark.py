# mks_servo_can/examples/timing_benchmark.py
"""
Example: Benchmarking command round-trip times with MKS Servo.
Can be used with either the simulator or real hardware.
"""
import statistics

import asyncio
import logging
import time

from mks_servo_can import Axis
from mks_servo_can import CANInterface
from mks_servo_can import exceptions
from mks_servo_can import const
# Correct import for RotaryKinematics:
from mks_servo_can import RotaryKinematics

# --- Configuration ---
USE_SIMULATOR = True  # Set to False for real hardware
SIMULATOR_HOST = "localhost"
SIMULATOR_PORT = 6789

# Real Hardware (if USE_SIMULATOR = False)
CAN_INTERFACE_TYPE = "canable"
CAN_CHANNEL = "/dev/ttyACM0"  # Adjust for your setup
CAN_BITRATE = 500000

MOTOR_CAN_ID = 1
NUM_ITERATIONS = 100

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def benchmark_read_position(axis: Axis, iterations: int) -> list[float]:
    """Benchmarks the time taken to read motor position."""
    latencies = []
    logger.info(
        "Benchmarking read_encoder_value_addition (%d iterations)...",
        iterations
    )
    await axis.enable_motor()

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.get_current_position_steps()
        except exceptions.MKSServoError as e:
            logger.warning("Iteration %d: Error reading position: %s", i + 1, e)
            continue
        end_time = time.perf_counter()
        latencies.append(
            (end_time - start_time) * 1000
        )
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.info("Iteration %d/%d completed.", i + 1, iterations)
        await asyncio.sleep(
            0.001
        )
    return latencies


async def benchmark_enable_disable(axis: Axis, iterations: int) -> list[float]:
    """Benchmarks enabling and then disabling the motor."""
    latencies = []
    logger.info("Benchmarking enable/disable (%d iterations)...", iterations)
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.enable_motor()
            await axis.disable_motor()
        except exceptions.MKSServoError as e:
            logger.warning(
                "Iteration %d: Error during enable/disable: %s", i + 1, e
            )
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)
        if (i + 1) % (iterations // 10 or 1) == 0:
            logger.info("Iteration %d/%d completed.", i + 1, iterations)
        await asyncio.sleep(0.005)
    return latencies


async def _setup_can_interface_for_benchmark() -> CANInterface:
    """Sets up and connects the CAN interface for the benchmark."""
    if USE_SIMULATOR:
        can_if = CANInterface(
            use_simulator=True,
            simulator_host=SIMULATOR_HOST,
            simulator_port=SIMULATOR_PORT,
        )
        sim_latency_ms = 2.0
        logger.info(
            "Using Simulator. Ensure it's running and configured (e.g., "
            "with latency %.1fms).",
            sim_latency_ms,
        )
    else:
        can_if = CANInterface(
            interface_type=CAN_INTERFACE_TYPE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE,
            use_simulator=False,
        )
        logger.info(
            "Using Real Hardware: %s on %s @ %s bps.",
            CAN_INTERFACE_TYPE,
            CAN_CHANNEL,
            CAN_BITRATE,
        )
    try:
        await can_if.connect()
        return can_if
    except exceptions.MKSServoError as e:
        logger.error("Failed to connect: %s", e)
        raise


async def _setup_axis_for_benchmark(can_if: CANInterface) -> Axis:
    """Sets up the Axis instance for the benchmark."""
    # Correct instantiation of RotaryKinematics
    kin = RotaryKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
    )
    axis = Axis(can_if, MOTOR_CAN_ID, "BenchmarkAxis", kinematics=kin)
    try:
        await axis.initialize(calibrate=False, home=False)
        return axis
    except exceptions.MKSServoError as e:
        logger.error("Failed to initialize axis %d: %s", MOTOR_CAN_ID, e)
        raise


def _print_benchmark_results(
    all_results: dict[str, list[float]], sim_used: bool
):
    """Prints the formatted benchmark results."""
    logger.info("\n--- Benchmark Results (ms) ---")
    for test_name, latencies in all_results.items():
        if not latencies:
            logger.info(
                "%s: No data collected (all iterations failed or skipped).",
                test_name,
            )
            continue
        count = len(latencies)
        mean_lat = statistics.mean(latencies)
        median_lat = statistics.median(latencies)
        min_lat = min(latencies)
        max_lat = max(latencies)
        std_dev = statistics.stdev(latencies) if count > 1 else 0.0

        title = test_name.replace("_", " ").title()
        logger.info("%s:", title)
        logger.info("  Iterations: %d", count)
        logger.info("  Min     : %.3f ms", min_lat)
        logger.info("  Max     : %.3f ms", max_lat)
        logger.info("  Mean    : %.3f ms", mean_lat)
        logger.info("  Median  : %.3f ms", median_lat)
        logger.info("  Std Dev : %.3f ms", std_dev)
        if sim_used:
            logger.info(
                "  (Note: Simulator introduces its own configured bus latency.)"
            )


async def run_all_benchmarks(axis: Axis):
    """Runs all defined benchmarks and returns the results."""
    all_results: dict[str, list[float]] = {}
    all_results["read_position"] = await benchmark_read_position(
        axis, NUM_ITERATIONS
    )
    all_results["enable_disable"] = await benchmark_enable_disable(
        axis, NUM_ITERATIONS
    )
    return all_results


async def main():
    """Main function to run the timing benchmarks."""
    can_if = None
    axis = None
    try:
        can_if = await _setup_can_interface_for_benchmark()
        axis = await _setup_axis_for_benchmark(can_if)
        all_results = await run_all_benchmarks(axis)
        _print_benchmark_results(all_results, USE_SIMULATOR)

    except KeyboardInterrupt:
        logger.info("Benchmark interrupted by user.")
    except exceptions.MKSServoError:
        logger.info("Exiting due to MKS Servo library error during setup.")
    except Exception as e:  # pylint: disable=broad-except
        logger.error(
            "An error occurred during benchmarking: %s", e, exc_info=True
        )
    finally:
        logger.info("Disconnecting...")
        if axis and axis.is_enabled():
            try:
                await axis.disable_motor()
            except Exception:  # pylint: disable=bare-except
                pass
        if can_if:
            await can_if.disconnect()
        logger.info("Benchmark finished.")


if __name__ == "__main__":
    asyncio.run(main())
    