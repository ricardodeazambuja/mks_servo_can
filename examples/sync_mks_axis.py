import asyncio
import threading
import time # For sleep in examples
import logging
from typing import Any, Dict, Optional # Added for type hints

# Assuming mks_servo_can library is installed and importable
from mks_servo_can import (
    CANInterface,
    Axis,
    RotaryKinematics, # Or your preferred kinematics
    LinearKinematics, # Adding for variety in kinematics
    const,
    exceptions
)
# Import Kinematics base class for type hinting
from mks_servo_can.kinematics import Kinematics # Corrected import for base class

# Configure basic logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("SyncMKSAxisWrapper")

class SyncMKSAxis:
    def __init__(self, motor_can_id: int,
                 can_interface_type: str = "canable", # For real hardware
                 can_channel: str = "/dev/ttyACM0",   # For real hardware
                 can_bitrate: int = const.CAN_DEFAULT_BITRATE,
                 use_simulator: bool = False,
                 simulator_host: str = "localhost",
                 simulator_port: int = 6789,
                 kinematics: Optional[Kinematics] = None, # Use Optional and base class
                 axis_name: str = "SyncAxis"):
        """
        Initializes the synchronous wrapper for an MKS Servo Axis.

        Args:
            motor_can_id: The CAN ID of the motor.
            can_interface_type: Type of CAN interface if not using simulator.
            can_channel: Channel for the CAN interface if not using simulator.
            can_bitrate: Bitrate for the CAN interface if not using simulator.
            use_simulator: True to use the simulator, False for real hardware.
            simulator_host: Hostname for the simulator.
            simulator_port: Port for the simulator.
            kinematics: Kinematics object for the axis. Defaults to Rotary.
            axis_name: A name for this axis.
        """
        self.motor_can_id = motor_can_id
        self.axis_name = axis_name

        # Parameters for CANInterface
        self._can_params = {
            "use_simulator": use_simulator,
            "simulator_host": simulator_host,
            "simulator_port": simulator_port,
            "interface_type": can_interface_type,
            "channel": can_channel,
            "bitrate": can_bitrate,
        }
        self._kinematics = kinematics if kinematics is not None else RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        )

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._can_if: Optional[CANInterface] = None
        self._axis: Optional[Axis] = None
        self._is_connected_flag = False
        self._shutdown_event = threading.Event()

    def _thread_target(self):
        """The function run by the dedicated asyncio thread."""
        logger.info(f"Asyncio thread started for {self.axis_name}.")
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            # Initialize CANInterface and Axis within this new loop
            # Pass the loop to CANInterface and Axis if their constructors accept it
            # (Checking the library, CANInterface and Axis can take a loop)
            self._can_if = CANInterface(loop=self._loop, **self._can_params)
            self._axis = Axis(
                can_interface_manager=self._can_if,
                motor_can_id=self.motor_can_id,
                name=self.axis_name,
                kinematics=self._kinematics,
                loop=self._loop
            )
            self._loop.run_forever()
        except Exception as e:
            logger.error(f"Exception in asyncio thread for {self.axis_name}: {e}", exc_info=True)
        finally:
            # Cleanly stop the loop if it's still running (e.g., due to an unhandled exception)
            if self._loop and self._loop.is_running():
                logger.info(f"Stopping loop from within _thread_target for {self.axis_name}")
                self._loop.call_soon_threadsafe(self._loop.stop)
            # Loop close is usually handled by the caller of run_forever after it stops
            # but we can call close once the loop is fully stopped.
            # Let's ensure all tasks are cancelled before closing.
            if self._loop:
                try:
                    # Gather all remaining tasks and cancel them
                    remaining_tasks = asyncio.all_tasks(self._loop)
                    if remaining_tasks:
                        logger.info(f"Cancelling {len(remaining_tasks)} outstanding tasks for {self.axis_name}.")
                        for task in remaining_tasks:
                            task.cancel()
                        # Wait for tasks to cancel
                        self._loop.run_until_complete(asyncio.gather(*remaining_tasks, return_exceptions=True))
                except Exception as e_cancel:
                     logger.error(f"Error cancelling tasks for {self.axis_name}: {e_cancel}")
                finally:
                    if not self._loop.is_closed():
                        logger.info(f"Closing asyncio loop for {self.axis_name}.")
                        self._loop.close()
            logger.info(f"Asyncio thread for {self.axis_name} finished.")


    def _submit_coro(self, coro, timeout_sec: Optional[float] = 5.0): # timeout_sec can be None
        """
        Submits a coroutine to the asyncio event loop and waits for its result.
        Raises exceptions encountered in the coroutine.
        """
        if not self._thread or not self._thread.is_alive():
             raise exceptions.MKSServoError(f"Asyncio thread for {self.axis_name} is not running. Call connect() first.")
        if not self._loop or not self._axis : # Check self._axis as well
            # This condition might be hit if connect() failed to initialize these
            raise exceptions.MKSServoError(f"Asyncio loop or Axis object for {self.axis_name} not initialized. Call connect() first.")
        if not self._is_connected_flag and hasattr(coro, '__name__') and coro.__name__ not in ['_async_connect', '_async_initialize_axis', '_async_disconnect']:
             raise exceptions.MKSServoError(f"{self.axis_name} is not connected. Call connect() first.")


        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            return future.result(timeout=timeout_sec) # Pass timeout directly
        except asyncio.TimeoutError as e:
            # It's good practice to try and cancel the underlying asyncio task if the sync call times out
            if not future.done(): # Check if future is not already done (e.g. cancelled by other means)
                logger.warning(f"Synchronous call for {self.axis_name} timed out. Attempting to cancel underlying async task.")
                self._loop.call_soon_threadsafe(future.cancel)
            raise TimeoutError(f"Operation timed out after {timeout_sec}s for {self.axis_name}") from e
        except Exception as e: # Catch other exceptions from the coroutine
            # The exception 'e' will be the one raised inside the coroutine
            raise e # Re-raise the original exception

    # --- Public Synchronous Methods ---

    async def _async_connect(self):
        """Internal async method to connect CAN and initialize Axis."""
        if not self._can_if:
            raise exceptions.ConfigurationError(f"CANInterface not initialized in async thread for {self.axis_name}")
        if not self._axis:
            raise exceptions.ConfigurationError(f"Axis not initialized in async thread for {self.axis_name}")

        if not self._can_if.is_connected:
            await self._can_if.connect()
        await self._axis.initialize(calibrate=False, home=False)

    def connect(self, timeout_sec: float = 10.0):
        """
        Starts the asyncio thread, connects the CAN interface, and initializes the axis.
        This is a blocking call.
        """
        if self._is_connected_flag:
            logger.info(f"{self.axis_name} is already connected.")
            return

        self._shutdown_event.clear()
        self._thread = threading.Thread(target=self._thread_target, name=f"AsyncioThread-{self.axis_name}", daemon=True)
        self._thread.start()

        # Wait for the loop and core objects to be initialized by the new thread
        # Using an event for synchronization is more reliable than time.sleep()
        # However, for simplicity in this context, we'll keep polling with a timeout
        # In a production wrapper, use threading.Event or similar.
        start_time = time.monotonic()
        while time.monotonic() - start_time < 5.0: # Max wait 5 seconds for loop init
            if self._loop and self._loop.is_running() and self._can_if and self._axis:
                break
            time.sleep(0.05)
        else:
            if self._thread and self._thread.is_alive(): # If thread is alive but objects not set, something is wrong
                if self._loop and self._loop.is_running():
                    self._loop.call_soon_threadsafe(self._loop.stop)
                self._thread.join(timeout=2.0)
            raise exceptions.MKSServoError(f"Failed to initialize asyncio components in thread for {self.axis_name}.")


        logger.info(f"Attempting to connect {self.axis_name} via asyncio thread...")
        try:
            self._submit_coro(self._async_connect(), timeout_sec=timeout_sec)
            self._is_connected_flag = True
            logger.info(f"{self.axis_name} connected successfully.")
        except Exception as e:
            logger.error(f"Connection attempt for {self.axis_name} failed: {e}")
            self.disconnect(timeout_sec=2.0) # Attempt graceful shutdown if connect failed
            raise exceptions.MKSServoError(f"Failed to connect {self.axis_name}: {e}") from e


    async def _async_disconnect(self):
        """Internal async method to disconnect CAN."""
        if self._axis and self._axis.is_enabled(): # Check if axis object exists and is enabled
             logger.info(f"Disabling {self.axis_name} before disconnect...")
             await self._axis.disable_motor() # Ensure motor is disabled
        if self._can_if and self._can_if.is_connected:
            await self._can_if.disconnect()

    def disconnect(self, timeout_sec: float = 5.0):
        """
        Disconnects the CAN interface, stops the asyncio loop, and joins the thread.
        This is a blocking call.
        """
        if not self._thread or not self._thread.is_alive():
            logger.info(f"{self.axis_name} thread is not running. No disconnect action needed beyond flag.")
            self._is_connected_flag = False # Ensure flag is false
            return

        logger.info(f"Disconnecting {self.axis_name}...")
        if self._loop and self._can_if and self._is_connected_flag: # Only run async_disconnect if was connected
            try:
                # Submit disconnect coroutine with a shorter timeout than thread join
                self._submit_coro(self._async_disconnect(), timeout_sec=max(1.0, timeout_sec -1.0 ))
            except Exception as e: # Catch all exceptions including TimeoutError from _submit_coro
                logger.warning(f"Error during async disconnect operations for {self.axis_name}: {e}")

        self._is_connected_flag = False # Set flag false before stopping loop

        if self._loop and self._loop.is_running():
            logger.debug(f"Requesting asyncio loop for {self.axis_name} to stop.")
            self._loop.call_soon_threadsafe(self._loop.stop)

        if self._thread and self._thread.is_alive():
            logger.debug(f"Joining asyncio thread for {self.axis_name}...")
            self._thread.join(timeout=timeout_sec)
            if self._thread.is_alive():
                logger.warning(f"Asyncio thread for {self.axis_name} did not terminate gracefully.")
        
        self._loop = None
        self._thread = None
        logger.info(f"{self.axis_name} disconnected sequence complete.")

    def enable_motor(self, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.enable_motor(), timeout_sec=timeout_sec)

    def disable_motor(self, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.disable_motor(), timeout_sec=timeout_sec)

    def is_enabled(self, timeout_sec: float = 1.0) -> bool:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.read_en_status(), timeout_sec=timeout_sec)

    def get_current_position_user(self, timeout_sec: float = 1.0) -> float:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_position_user(), timeout_sec=timeout_sec)

    def get_current_position_steps(self, timeout_sec: float = 1.0) -> int:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_position_steps(), timeout_sec=timeout_sec)

    def move_absolute_user(self, target_pos_user: float, speed_user: Optional[float] = None, wait_for_completion: bool = True, move_timeout_sec: float = 30.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_to_position_abs_user(
            target_pos_user=target_pos_user,
            speed_user=speed_user, # Axis method will use default if None
            wait=wait_for_completion
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_relative_user(self, relative_dist_user: float, speed_user: Optional[float] = None, wait_for_completion: bool = True, move_timeout_sec: float = 30.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_relative_user(
            relative_dist_user=relative_dist_user,
            speed_user=speed_user, # Axis method will use default if None
            wait=wait_for_completion
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_absolute_pulses(self, target_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True, move_timeout_sec: float = 30.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_to_position_abs_pulses(
            target_pulses=target_pulses,
            speed_param=speed_param,
            accel_param=accel_param,
            wait=wait
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_relative_pulses(self, relative_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True, move_timeout_sec: float = 30.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_relative_pulses(
            relative_pulses=relative_pulses,
            speed_param=speed_param,
            accel_param=accel_param,
            wait=wait
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def set_current_position_as_zero(self, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.set_current_position_as_zero(), timeout_sec=timeout_sec)

    def home_axis(self, wait_for_completion: bool = True, home_timeout_sec: float = 60.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The axis.home_axis itself has a timeout parameter for its internal async waiting logic.
        # This home_timeout_sec is for the blocking .result() call.
        internal_async_timeout = home_timeout_sec - 2.0 if home_timeout_sec > 3 else home_timeout_sec / 2
        coro = self._axis.home_axis(wait_for_completion=wait_for_completion, timeout=max(1.0, internal_async_timeout))
        return self._submit_coro(coro, timeout_sec=home_timeout_sec)

    def set_speed_user(self, speed_user: float, accel_user: Optional[float] = None, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.set_speed_user(speed_user=speed_user, accel_user=accel_user)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def stop_motor(self, deceleration_param: Optional[int] = None, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.stop_motor(deceleration_param=deceleration_param)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def emergency_stop(self, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.emergency_stop(), timeout_sec=timeout_sec)

    def get_current_speed_rpm(self, timeout_sec: float = 1.0) -> int:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_speed_rpm(), timeout_sec=timeout_sec)

    def get_current_speed_user(self, timeout_sec: float = 1.0) -> float:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_speed_user(), timeout_sec=timeout_sec)

    def get_motor_status_code(self, timeout_sec: float = 1.0) -> int:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_motor_status_code(), timeout_sec=timeout_sec)

    def get_status_dict(self, timeout_sec: float = 1.5) -> Dict[str, Any]:
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # This involves multiple reads if not cached, so a slightly longer timeout might be good.
        return self._submit_coro(self._axis.get_status_dict(), timeout_sec=timeout_sec)

    def set_work_mode(self, mode_const: int, timeout_sec: float = 2.0):
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.set_work_mode(mode_const), timeout_sec=timeout_sec)

    def is_homed(self) -> bool:
        """Returns the cached homed status."""
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # This directly returns the flag set by async operations, no CAN call here by default
        return self._axis.is_homed()

    def is_calibrated(self) -> bool:
        """Returns the cached calibrated status."""
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._axis.is_calibrated()

    def is_move_complete(self) -> bool:
        """Checks if the last commanded move is complete (based on cached future)."""
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._axis.is_move_complete()

    def wait_for_move_completion(self, timeout_sec: Optional[float] = None):
        """Blocks until the current move is complete or timeout occurs."""
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The timeout_sec for _submit_coro should be slightly larger than the
        # internal timeout for self._axis.wait_for_move_completion if that one also has a timeout.
        # Or, if self._axis.wait_for_move_completion can run indefinitely (None timeout),
        # then the timeout_sec here will be the effective timeout.
        internal_async_timeout = timeout_sec - 1.0 if timeout_sec and timeout_sec > 1 else timeout_sec
        coro = self._axis.wait_for_move_completion(timeout=internal_async_timeout)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def ping(self, timeout_sec: float = 1.0) -> Optional[float]:
        """Pings the motor and returns response time in ms, or None on failure."""
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The Axis.ping() method in the library already returns response_time_ms or raises an error.
        # So, we expect a float if successful.
        try:
            response_time = self._submit_coro(self._axis.ping(), timeout_sec=timeout_sec)
            return response_time # Should be float if successful
        except exceptions.CommunicationError:
            logger.warning(f"Ping timed out for {self.axis_name}")
            return None
        except exceptions.MKSServoError as e:
            logger.warning(f"Ping failed for {self.axis_name}: {e}")
            return None

    def __enter__(self):
        """Context management: connect on entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context management: disconnect on exit."""
        self.disconnect()


if __name__ == "__main__":
    logger.info("Starting synchronous MKS Servo control example.")

    # --- Configuration ---
    # For Simulator:
    USE_SIM = True
    MOTOR_ID = 1
    # Ensure simulator is running: mks-servo-simulator --num-motors 1 --start-can-id 1

    # For Real Hardware (example):
    # USE_SIM = False
    # MOTOR_ID = 1 # Your motor's CAN ID
    # CAN_TYPE = "canable"
    # CAN_CHANNEL = "/dev/ttyACM0" # Linux example, use "COMx" for Windows if applicable
    # CAN_BITRATE_HW = 500000

    if USE_SIM:
        motor = SyncMKSAxis(motor_can_id=MOTOR_ID, use_simulator=True)
    else:
        # motor = SyncMKSAxis(
        #     motor_can_id=MOTOR_ID,
        #     use_simulator=False,
        #     can_interface_type=CAN_TYPE,
        #     can_channel=CAN_CHANNEL,
        #     can_bitrate=CAN_BITRATE_HW
        # )
        logger.error("Real hardware part of example is commented out. Set USE_SIM=False and configure above.")
        exit()


    try:
        logger.info("Connecting to motor...")
        motor.connect() # This is a blocking call
        logger.info("Motor connected.")

        logger.info("Enabling motor...")
        motor.enable_motor() # Blocking
        if motor.is_enabled(): # Blocking
            logger.info("Motor is enabled.")
        else:
            logger.error("Failed to enable motor.")
            motor.disconnect()
            exit()

        initial_pos = motor.get_current_position_user() # Blocking
        logger.info(f"Initial position: {initial_pos:.2f} {motor._kinematics.units}") # Accessing kinematics units for example

        logger.info("Moving to 90.0 degrees...")
        motor.move_absolute_user(target_pos_user=90.0, speed_user=180.0, wait_for_completion=True) # Blocking
        logger.info(f"Position after move: {motor.get_current_position_user():.2f} {motor._kinematics.units}")

        time.sleep(1)

        logger.info("Moving relatively by -30.0 degrees...")
        motor.move_relative_user(relative_dist_user=-30.0, speed_user=90.0, wait_for_completion=True) # Blocking
        logger.info(f"Position after relative move: {motor.get_current_position_user():.2f} {motor._kinematics.units}")

        time.sleep(1)

        logger.info("Setting current position as zero...")
        motor.set_current_position_as_zero() # Blocking
        logger.info(f"New current position: {motor.get_current_position_user():.2f} {motor._kinematics.units}")


        logger.info("Disabling motor...")
        motor.disable_motor() # Blocking
        logger.info("Motor disabled.")

    except TimeoutError as e:
        logger.error(f"Operation timed out: {e}")
    except exceptions.MKSServoError as e:
        logger.error(f"MKS Servo library error: {e}", exc_info=True)
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        logger.info("Shutting down motor connection...")
        if motor: # Check if motor object was created
            motor.disconnect() # Blocking
        logger.info("Synchronous example finished.")