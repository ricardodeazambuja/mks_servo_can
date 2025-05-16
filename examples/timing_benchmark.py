# mks_servo_can_project/examples/timing_benchmark.py
"""
Example: Benchmarking command round-trip times with MKS Servo.
Can be used with either the simulator or real hardware.
"""
import asyncio
import time
import logging
import statistics

from mks_servo_can_library.mks_servo_can import (
    CANInterface, Axis, const, exceptions
)

# --- Configuration ---
USE_SIMULATOR = True # Set to False for real hardware
SIMULATOR_HOST = 'localhost'
SIMULATOR_PORT = 6789

# Real Hardware (if USE_SIMULATOR = False)
CAN_INTERFACE_TYPE = 'canable'
CAN_CHANNEL = '/dev/ttyACM0' # Adjust for your setup
CAN_BITRATE = 500000

MOTOR_CAN_ID = 1
NUM_ITERATIONS = 100

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

async def benchmark_read_position(axis: Axis, iterations: int) -> list[float]:
    """Benchmarks the time taken to read motor position."""
    latencies = []
    logging.info(f"Benchmarking read_encoder_value_addition ({iterations} iterations)...")
    await axis.enable_motor() # Ensure motor is responsive

    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            # Use a low-level command directly for minimal overhead if Axis method adds too much
            # await axis._low_level_api.read_encoder_value_addition(axis.can_id)
            await axis.get_current_position_steps() # Axis method
        except exceptions.MKSServoError as e:
            logging.warning(f"Iteration {i+1}: Error reading position: {e}")
            continue # Skip this iteration on error
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000) # Convert to milliseconds
        if (i + 1) % (iterations // 10 or 1) == 0:
            logging.info(f"Iteration {i+1}/{iterations} completed.")
        await asyncio.sleep(0.001) # Small delay to prevent overwhelming bus/sim
    return latencies

async def benchmark_enable_disable(axis: Axis, iterations: int) -> list[float]:
    """Benchmarks enabling and then disabling the motor."""
    latencies = []
    logging.info(f"Benchmarking enable/disable ({iterations} iterations)...")
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            await axis.enable_motor()
            await axis.disable_motor()
        except exceptions.MKSServoError as e:
            logging.warning(f"Iteration {i+1}: Error during enable/disable: {e}")
            continue
        end_time = time.perf_counter()
        latencies.append((end_time - start_time) * 1000)
        if (i + 1) % (iterations // 10 or 1) == 0:
            logging.info(f"Iteration {i+1}/{iterations} completed.")
        await asyncio.sleep(0.005)
    return latencies


async def main():
    if USE_SIMULATOR:
        can_if = CANInterface(use_simulator=True, simulator_host=SIMULATOR_HOST, simulator_port=SIMULATOR_PORT)
        sim_latency_ms = 2.0 # Example latency to configure in simulator CLI
        logging.info(f"Using Simulator. Ensure it's running and configured (e.g., with latency {sim_latency_ms}ms).")
    else:
        can_if = CANInterface(interface_type=CAN_INTERFACE_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE, use_simulator=False)
        logging.info(f"Using Real Hardware: {CAN_INTERFACE_TYPE} on {CAN_CHANNEL} @ {CAN_BITRATE}bps.")

    try:
        await can_if.connect()
    except exceptions.MKSServoError as e:
        logging.error(f"Failed to connect: {e}")
        return

    axis = Axis(can_if, MOTOR_CAN_ID, "BenchmarkAxis")
    try:
        await axis.initialize(calibrate=False, home=False) # Basic check
    except exceptions.MKSServoError as e:
        logging.error(f"Failed to initialize axis {MOTOR_CAN_ID}: {e}")
        await can_if.disconnect()
        return

    all_results: dict[str, list[float]] = {}

    try:
        all_results["read_position"] = await benchmark_read_position(axis, NUM_ITERATIONS)
        all_results["enable_disable"] = await benchmark_enable_disable(axis, NUM_ITERATIONS)

        # --- Analysis ---
        logging.info("\n--- Benchmark Results (ms) ---")
        for test_name, latencies in all_results.items():
            if not latencies:
                logging.info(f"{test_name}: No data collected (all iterations failed or skipped).")
                continue
            count = len(latencies)
            mean_lat = statistics.mean(latencies)
            median_lat = statistics.median(latencies)
            min_lat = min(latencies)
            max_lat = max(latencies)
            std_dev = statistics.stdev(latencies) if count > 1 else 0

            logging.info(f"{test_name.replace('_', ' ').title()}:")
            logging.info(f"  Iterations: {count}")
            logging.info(f"  Min     : {min_lat:.3f} ms")
            logging.info(f"  Max     : {max_lat:.3f} ms")
            logging.info(f"  Mean    : {mean_lat:.3f} ms")
            logging.info(f"  Median  : {median_lat:.3f} ms")
            logging.info(f"  Std Dev : {std_dev:.3f} ms")
            if USE_SIMULATOR:
                logging.info(f"  (Note: Simulator introduces its own configured latency of approx. {can_if.simulator_host if hasattr(can_if, 'simulator_host') else 'N/A'} bus latency.)") # TODO: Get actual sim latency if possible

    except KeyboardInterrupt:
        logging.info("Benchmark interrupted by user.")
    except Exception as e:
        logging.error(f"An error occurred during benchmarking: {e}", exc_info=True)
    finally:
        logging.info("Disconnecting...")
        if axis.is_enabled(): # Try to disable if left enabled
            try:
                await axis.disable_motor()
            except: pass
        await can_if.disconnect()
        logging.info("Benchmark finished.")

if __name__ == "__main__":
    asyncio.run(main())