"""
Example script demonstrating a synchronous wrapper for the mks-servo-can library.

This script provides a `SyncMKSAxis` class that allows controlling an MKS servo motor
using blocking (synchronous) calls, which can be easier to integrate into
traditional synchronous Python applications. It manages an asyncio event loop
in a separate thread to interact with the underlying asynchronous library.
"""
import asyncio
# Imports the asyncio library, which is the foundation for the mks_servo_can library's asynchronous operations.
# Even though this example creates a synchronous wrapper, asyncio is still needed to run the underlying async library.

import threading
# Imports the threading module, used here to run the asyncio event loop in a separate thread.
# This allows the synchronous part of the wrapper to block and wait for results from the async operations
# without freezing the main thread (if the main application were also GUI-based, for example).

import time # For sleep in examples
# Imports the time module, primarily used in the example usage part for pausing execution (time.sleep).

import logging
# Imports the logging module for outputting informational and error messages.

from typing import Any, Dict, Optional # Added for type hints
# Imports typing utilities for better code clarity and static analysis.
# 'Any' can represent an unconstrained type.
# 'Dict' is for dictionary types.
# 'Optional' indicates a type that can be 'None'.

# Assuming mks_servo_can library is installed and importable
# This comment indicates that the mks_servo_can library needs to be accessible in the Python environment.
from mks_servo_can import (
    CANInterface,
    Axis,
    RotaryKinematics, # Or your preferred kinematics
    LinearKinematics, # Adding for variety in kinematics
    const,
    exceptions
)
# Imports core components from the mks_servo_can library:
# - CANInterface: For managing the CAN bus connection.
# - Axis: The high-level class for controlling a single motor.
# - RotaryKinematics, LinearKinematics: Specific kinematics implementations.
# - const: The module containing library constants (command codes, default values).
# - exceptions: The module containing custom library exceptions.

# Import Kinematics base class for type hinting
from mks_servo_can.kinematics import Kinematics # Corrected import for base class
# Imports the base 'Kinematics' class, primarily for type hinting the 'kinematics' parameter.

# Configure basic logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
# Sets up basic logging configuration for the script.
# - level=logging.INFO: Logs messages of INFO level and above (WARNING, ERROR, CRITICAL).
# - format: Defines the log message format.
logger = logging.getLogger("SyncMKSAxisWrapper")
# Creates a logger instance specific to this wrapper class.

class SyncMKSAxis:
    """
    Synchronous wrapper for an MKS Servo Axis.

    This class provides a blocking interface to control an MKS servo motor,
    managing an internal asyncio event loop in a separate thread to interact
    with the asynchronous `mks_servo_can.Axis` object.
    """
    # Defines a class 'SyncMKSAxis' which acts as a synchronous wrapper around an asynchronous 'Axis' object.
    # This allows users to call methods in a blocking, synchronous way.
    def __init__(self, motor_can_id: int,
                 can_interface_type: str = "canable", # For real hardware
                 can_channel: str = "/dev/ttyACM0",   # For real hardware
                 can_bitrate: int = const.CAN_DEFAULT_BITRATE,
                 use_simulator: bool = False,
                 simulator_host: str = "localhost",
                 simulator_port: int = 6789,
                 kinematics: Optional[Kinematics] = None, # Use Optional and base class
                 axis_name: str = "SyncAxis"):
        # The constructor for the SyncMKSAxis wrapper.
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
        # Stores the motor's CAN ID.
        self.motor_can_id = motor_can_id
        # Stores the name for this axis instance.
        self.axis_name = axis_name

        # Stores parameters needed to initialize the CANInterface in the separate thread.
        self._can_params = {
            "use_simulator": use_simulator,
            "simulator_host": simulator_host,
            "simulator_port": simulator_port,
            "interface_type": can_interface_type,
            "channel": can_channel,
            "bitrate": can_bitrate,
        }
        # Sets up the kinematics. If None is provided, it defaults to RotaryKinematics
        # using the standard encoder pulse count.
        self._kinematics = kinematics if kinematics is not None else RotaryKinematics(
            steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
        )

        # Internal attributes for managing the asyncio loop and thread.
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Will hold the asyncio event loop instance.
        self._thread: Optional[threading.Thread] = None
        # Will hold the dedicated thread object for the asyncio loop.
        self._can_if: Optional[CANInterface] = None
        # Will hold the CANInterface instance created in the asyncio thread.
        self._axis: Optional[Axis] = None
        # Will hold the Axis instance created in the asyncio thread.
        self._is_connected_flag = False
        # A flag to track the connection status.
        self._shutdown_event = threading.Event()
        # A threading.Event used to signal shutdown to the asyncio thread (though not explicitly used for this in current code, good for future).

    def _thread_target(self):
        # This method is the target function that will be run in the dedicated asyncio thread.
        """The function run by the dedicated asyncio thread."""
        logger.info(f"Asyncio thread started for {self.axis_name}.")
        # Logs the start of the thread.
        try:
            # Creates a new asyncio event loop for this thread.
            self._loop = asyncio.new_event_loop()
            # Sets this new loop as the current event loop for this thread.
            asyncio.set_event_loop(self._loop)
            # Initialize CANInterface and Axis within this new loop
            # These objects must be created in the thread where their asyncio loop is running.
            # Pass the loop to CANInterface and Axis if their constructors accept it
            # (Checking the library, CANInterface and Axis can take a loop)
            # The comment confirms that the library's classes can accept an event loop.
            self._can_if = CANInterface(loop=self._loop, **self._can_params)
            # Creates the CANInterface instance, passing the loop and CAN parameters.
            self._axis = Axis(
                can_interface_manager=self._can_if,
                motor_can_id=self.motor_can_id,
                name=self.axis_name,
                kinematics=self._kinematics,
                loop=self._loop # Passes the loop to the Axis instance.
            )
            # Creates the Axis instance.
            self._loop.run_forever()
            # Starts the asyncio event loop, which will run until loop.stop() is called.
        except Exception as e:
            # Catches any exceptions that occur within the asyncio thread.
            logger.error(f"Exception in asyncio thread for {self.axis_name}: {e}", exc_info=True)
            # Logs the error with traceback.
        finally:
            # This block ensures cleanup even if an error occurs in the thread.
            # Cleanly stop the loop if it's still running (e.g., due to an unhandled exception)
            if self._loop and self._loop.is_running():
                # If the loop is still running (e.g., run_forever exited due to an error, not stop()),
                # schedule stop() to be called.
                logger.info(f"Stopping loop from within _thread_target for {self.axis_name}")
                self._loop.call_soon_threadsafe(self._loop.stop)
            # Loop close is usually handled by the caller of run_forever after it stops
            # but we can call close once the loop is fully stopped.
            # Let's ensure all tasks are cancelled before closing.
            # This further cleanup ensures all asyncio tasks are properly handled before closing the loop.
            if self._loop:
                try:
                    # Gather all remaining tasks associated with this loop.
                    remaining_tasks = asyncio.all_tasks(self._loop)
                    if remaining_tasks:
                        logger.info(f"Cancelling {len(remaining_tasks)} outstanding tasks for {self.axis_name}.")
                        # Cancel each remaining task.
                        for task in remaining_tasks:
                            task.cancel()
                        # Wait for all cancellation tasks to complete.
                        self._loop.run_until_complete(asyncio.gather(*remaining_tasks, return_exceptions=True))
                except Exception as e_cancel:
                     logger.error(f"Error cancelling tasks for {self.axis_name}: {e_cancel}")
                finally:
                    # If the loop is not already closed, close it.
                    if not self._loop.is_closed():
                        logger.info(f"Closing asyncio loop for {self.axis_name}.")
                        self._loop.close()
            logger.info(f"Asyncio thread for {self.axis_name} finished.")
            # Logs the end of the thread execution.

    def _submit_coro(self, coro, timeout_sec: Optional[float] = 5.0): # timeout_sec can be None
        # This private helper method submits a coroutine (an async function call)
        # to the asyncio event loop running in the separate thread and waits for its result synchronously.
        """
        Submits a coroutine to the asyncio event loop and waits for its result.
        Raises exceptions encountered in the coroutine.
        """
        # Checks if the asyncio thread is alive and essential objects are initialized.
        if not self._thread or not self._thread.is_alive():
             raise exceptions.MKSServoError(f"Asyncio thread for {self.axis_name} is not running. Call connect() first.")
        if not self._loop or not self._axis : # Check self._axis as well
            # This condition might be hit if connect() failed to initialize these
            raise exceptions.MKSServoError(f"Asyncio loop or Axis object for {self.axis_name} not initialized. Call connect() first.")
        # Prevents operations (except connect/disconnect) if not connected.
        if not self._is_connected_flag and hasattr(coro, '__name__') and coro.__name__ not in ['_async_connect', '_async_initialize_axis', '_async_disconnect']:
             raise exceptions.MKSServoError(f"{self.axis_name} is not connected. Call connect() first.")

        # Submits the coroutine to the event loop in the other thread.
        # 'run_coroutine_threadsafe' returns a concurrent.futures.Future.
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            # Waits for the result of the future (and thus the coroutine) with a timeout.
            return future.result(timeout=timeout_sec) # Pass timeout directly
        except asyncio.TimeoutError as e:
            # If 'future.result()' times out.
            # It's good practice to try and cancel the underlying asyncio task if the sync call times out
            if not future.done(): # Check if future is not already done (e.g. cancelled by other means)
                logger.warning(f"Synchronous call for {self.axis_name} timed out. Attempting to cancel underlying async task.")
                # Schedules cancellation of the asyncio task in the event loop's thread.
                self._loop.call_soon_threadsafe(future.cancel)
            # Raises a standard Python TimeoutError for the synchronous caller.
            raise TimeoutError(f"Operation timed out after {timeout_sec}s for {self.axis_name}") from e
        except Exception as e: # Catch other exceptions from the coroutine
            # If the coroutine itself raised an exception, it will be re-raised here.
            # The exception 'e' will be the one raised inside the coroutine
            raise e # Re-raise the original exception

    # --- Public Synchronous Methods ---
    # The following methods are the public synchronous API of the SyncMKSAxis wrapper.
    # They internally call their asynchronous counterparts using '_submit_coro'.

    async def _async_connect(self):
        """Internal async method to connect CAN and initialize Axis."""
        # An internal asynchronous method to handle the connection logic.
        # This will be run in the asyncio thread via _submit_coro.
        if not self._can_if:
            # Should not happen if _thread_target initialized correctly.
            raise exceptions.ConfigurationError(f"CANInterface not initialized in async thread for {self.axis_name}")
        if not self._axis:
            # Should not happen if _thread_target initialized correctly.
            raise exceptions.ConfigurationError(f"Axis not initialized in async thread for {self.axis_name}")

        # Connects the CANInterface if not already connected.
        if not self._can_if.is_connected:
            await self._can_if.connect()
        # Initializes the Axis (e.g., pings motor, reads initial status).
        await self._axis.initialize(calibrate=False, home=False)

    def connect(self, timeout_sec: float = 10.0):
        # The public synchronous method to connect to the motor.
        """
        Starts the asyncio thread, connects the CAN interface, and initializes the axis.
        This is a blocking call.

        Args:
            timeout_sec: Maximum time to wait for connection and initialization.
        """
        # If already connected, do nothing.
        if self._is_connected_flag:
            logger.info(f"{self.axis_name} is already connected.")
            return

        self._shutdown_event.clear() # Not currently used for active signaling in _thread_target.
        # Creates and starts the dedicated asyncio thread.
        self._thread = threading.Thread(target=self._thread_target, name=f"AsyncioThread-{self.axis_name}", daemon=True)
        self._thread.start()

        # Wait for the loop and core objects to be initialized by the new thread
        # This loop waits for the _thread_target to set up _loop, _can_if, and _axis.
        # Using an event for synchronization is more reliable than time.sleep()
        # However, for simplicity in this context, we'll keep polling with a timeout
        # In a production wrapper, use threading.Event or similar.
        # The comment acknowledges a more robust synchronization method (threading.Event).
        start_time = time.monotonic()
        while time.monotonic() - start_time < 5.0: # Max wait 5 seconds for loop init
            if self._loop and self._loop.is_running() and self._can_if and self._axis:
                # Break if all necessary components are initialized.
                break
            time.sleep(0.05) # Brief pause before re-checking.
        else:
            # If the loop times out, it means components weren't initialized.
            if self._thread and self._thread.is_alive(): # If thread is alive but objects not set, something is wrong
                # Attempt to stop the loop and join the thread if initialization failed.
                if self._loop and self._loop.is_running():
                    self._loop.call_soon_threadsafe(self._loop.stop)
                self._thread.join(timeout=2.0)
            raise exceptions.MKSServoError(f"Failed to initialize asyncio components in thread for {self.axis_name}.")

        logger.info(f"Attempting to connect {self.axis_name} via asyncio thread...")
        try:
            # Submits the _async_connect coroutine to the asyncio thread.
            self._submit_coro(self._async_connect(), timeout_sec=timeout_sec)
            # Sets the connection flag to True on success.
            self._is_connected_flag = True
            logger.info(f"{self.axis_name} connected successfully.")
        except Exception as e:
            # If connection fails.
            logger.error(f"Connection attempt for {self.axis_name} failed: {e}")
            # Attempt to clean up by calling disconnect.
            self.disconnect(timeout_sec=2.0) # Attempt graceful shutdown if connect failed
            raise exceptions.MKSServoError(f"Failed to connect {self.axis_name}: {e}") from e

    async def _async_disconnect(self):
        # Internal asynchronous method for disconnection logic.
        """Internal async method to disconnect CAN."""
        # If the axis object exists and is enabled, disable it first.
        if self._axis and self._axis.is_enabled(): # Check if axis object exists and is enabled
             logger.info(f"Disabling {self.axis_name} before disconnect...")
             await self._axis.disable_motor() # Ensure motor is disabled
        # If the CAN interface exists and is connected, disconnect it.
        if self._can_if and self._can_if.is_connected:
            await self._can_if.disconnect()

    def disconnect(self, timeout_sec: float = 5.0):
        # Public synchronous method to disconnect from the motor and clean up the thread.
        """
        Disconnects the CAN interface, stops the asyncio loop, and joins the thread.
        This is a blocking call.

        Args:
            timeout_sec: Maximum time to wait for disconnection.
        """
        # If the thread isn't running, there's nothing to disconnect in terms of thread/loop.
        if not self._thread or not self._thread.is_alive():
            logger.info(f"{self.axis_name} thread is not running. No disconnect action needed beyond flag.")
            self._is_connected_flag = False # Ensure flag is false
            return

        logger.info(f"Disconnecting {self.axis_name}...")
        # If the loop and CAN interface exist and the connection flag was true, attempt async disconnect operations.
        if self._loop and self._can_if and self._is_connected_flag: # Only run async_disconnect if was connected
            try:
                # Submit the _async_disconnect coroutine.
                # Use a timeout slightly less than the thread join timeout to ensure async operations complete or timeout first.
                self._submit_coro(self._async_disconnect(), timeout_sec=max(1.0, timeout_sec -1.0 ))
            except Exception as e: # Catch all exceptions including TimeoutError from _submit_coro
                logger.warning(f"Error during async disconnect operations for {self.axis_name}: {e}")

        self._is_connected_flag = False # Set flag false before stopping loop

        # If the asyncio loop is running, request it to stop.
        if self._loop and self._loop.is_running():
            logger.debug(f"Requesting asyncio loop for {self.axis_name} to stop.")
            self._loop.call_soon_threadsafe(self._loop.stop) # Schedules loop.stop() in the asyncio thread.

        # If the thread object exists and is alive, wait for it to join (terminate).
        if self._thread and self._thread.is_alive():
            logger.debug(f"Joining asyncio thread for {self.axis_name}...")
            self._thread.join(timeout=timeout_sec) # Waits for the thread to finish.
            if self._thread.is_alive():
                # If the thread doesn't terminate within the timeout.
                logger.warning(f"Asyncio thread for {self.axis_name} did not terminate gracefully.")

        # Clear references to the loop and thread.
        self._loop = None
        self._thread = None
        logger.info(f"{self.axis_name} disconnected sequence complete.")

    def enable_motor(self, timeout_sec: float = 2.0):
        """Enables the motor. Blocks until the command is acknowledged or timeout."""
        # Synchronous wrapper for Axis.enable_motor().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.enable_motor(), timeout_sec=timeout_sec)

    def disable_motor(self, timeout_sec: float = 2.0):
        """Disables the motor. Blocks until the command is acknowledged or timeout."""
        # Synchronous wrapper for Axis.disable_motor().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.disable_motor(), timeout_sec=timeout_sec)

    def is_enabled(self, timeout_sec: float = 1.0) -> bool:
        """Checks if the motor is enabled by querying its status. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.read_en_status() (as Axis.is_enabled() is usually cached).
        # This actively queries the motor for its enable status.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.read_en_status(), timeout_sec=timeout_sec)

    def get_current_position_user(self, timeout_sec: float = 1.0) -> float:
        """Gets the current motor position in user units. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_current_position_user().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_position_user(), timeout_sec=timeout_sec)

    def get_current_position_steps(self, timeout_sec: float = 1.0) -> int:
        """Gets the current motor position in raw encoder steps. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_current_position_steps().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_position_steps(), timeout_sec=timeout_sec)

    def move_absolute_user(self, target_pos_user: float, speed_user: Optional[float] = None, wait_for_completion: bool = True, move_timeout_sec: float = 30.0):
        """Moves the motor to an absolute position in user units."""
        # Synchronous wrapper for Axis.move_to_position_abs_user().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_to_position_abs_user(
            target_pos_user=target_pos_user,
            speed_user=speed_user, # Axis method will use default if None
            wait=wait_for_completion # The async method's 'wait' parameter controls its internal awaiting.
        )
        # 'move_timeout_sec' is for the synchronous _submit_coro call.
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_relative_user(self, relative_dist_user: float, speed_user: Optional[float] = None, wait_for_completion: bool = True, move_timeout_sec: float = 30.0):
        """Moves the motor by a relative distance in user units."""
        # Synchronous wrapper for Axis.move_relative_user().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_relative_user(
            relative_dist_user=relative_dist_user,
            speed_user=speed_user, # Axis method will use default if None
            wait=wait_for_completion
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_absolute_pulses(self, target_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True, move_timeout_sec: float = 30.0):
        """Moves the motor to an absolute position in raw command pulses."""
        # Synchronous wrapper for Axis.move_to_position_abs_pulses().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_to_position_abs_pulses(
            target_pulses=target_pulses,
            speed_param=speed_param,
            accel_param=accel_param,
            wait=wait
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def move_relative_pulses(self, relative_pulses: int, speed_param: Optional[int] = None, accel_param: Optional[int] = None, wait: bool = True, move_timeout_sec: float = 30.0):
        """Moves the motor by a relative distance in raw command pulses."""
        # Synchronous wrapper for Axis.move_relative_pulses().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.move_relative_pulses(
            relative_pulses=relative_pulses,
            speed_param=speed_param,
            accel_param=accel_param,
            wait=wait
        )
        return self._submit_coro(coro, timeout_sec=move_timeout_sec)

    def set_current_position_as_zero(self, timeout_sec: float = 2.0):
        """Sets the motor's current position as the zero reference. Blocks until acknowledged."""
        # Synchronous wrapper for Axis.set_current_position_as_zero().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.set_current_position_as_zero(), timeout_sec=timeout_sec)

    def home_axis(self, wait_for_completion: bool = True, home_timeout_sec: float = 60.0):
        """Commands the motor to perform its homing sequence."""
        # Synchronous wrapper for Axis.home_axis().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The axis.home_axis itself has a timeout parameter for its internal async waiting logic.
        # This home_timeout_sec is for the blocking .result() call.
        # Ensure internal async timeout is less than the sync timeout to allow the sync timeout to catch it if necessary.
        internal_async_timeout = home_timeout_sec - 2.0 if home_timeout_sec > 3 else home_timeout_sec / 2
        coro = self._axis.home_axis(wait_for_completion=wait_for_completion, timeout=max(1.0, internal_async_timeout))
        return self._submit_coro(coro, timeout_sec=home_timeout_sec)

    def set_speed_user(self, speed_user: float, accel_user: Optional[float] = None, timeout_sec: float = 2.0):
        """Sets the motor to run continuously at a specified speed in user units."""
        # Synchronous wrapper for Axis.set_speed_user().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.set_speed_user(speed_user=speed_user, accel_user=accel_user)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def stop_motor(self, deceleration_param: Optional[int] = None, timeout_sec: float = 2.0):
        """Commands the motor to stop its current movement with controlled deceleration."""
        # Synchronous wrapper for Axis.stop_motor().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        coro = self._axis.stop_motor(deceleration_param=deceleration_param)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def emergency_stop(self, timeout_sec: float = 2.0):
        """Commands an immediate emergency stop of the motor."""
        # Synchronous wrapper for Axis.emergency_stop().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.emergency_stop(), timeout_sec=timeout_sec)

    def get_current_speed_rpm(self, timeout_sec: float = 1.0) -> int:
        """Gets the current motor speed in RPM. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_current_speed_rpm().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_speed_rpm(), timeout_sec=timeout_sec)

    def get_current_speed_user(self, timeout_sec: float = 1.0) -> float:
        """Gets the current motor speed in user units. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_current_speed_user().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_current_speed_user(), timeout_sec=timeout_sec)

    def get_motor_status_code(self, timeout_sec: float = 1.0) -> int:
        """Queries and returns the raw motor status code. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_motor_status_code().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.get_motor_status_code(), timeout_sec=timeout_sec)

    def get_status_dict(self, timeout_sec: float = 1.5) -> Dict[str, Any]:
        """Retrieves a comprehensive status dictionary for the axis. Blocks until response or timeout."""
        # Synchronous wrapper for Axis.get_status_dict().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # This involves multiple reads if not cached, so a slightly longer timeout might be good.
        return self._submit_coro(self._axis.get_status_dict(), timeout_sec=timeout_sec)

    def set_work_mode(self, mode_const: int, timeout_sec: float = 2.0):
        """Sets the working mode of the MKS servo motor. Blocks until acknowledged."""
        # Synchronous wrapper for Axis.set_work_mode().
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._submit_coro(self._axis.set_work_mode(mode_const), timeout_sec=timeout_sec)

    def is_homed(self) -> bool:
        """Returns the cached homed status."""
        # Returns the cached homed status from the underlying Axis object.
        # This does not involve a new CAN call unless the underlying Axis.is_homed() itself triggers one.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # This directly returns the flag set by async operations, no CAN call here by default
        return self._axis.is_homed()

    def is_calibrated(self) -> bool:
        """Returns the cached calibrated status."""
        # Returns the cached calibrated status.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._axis.is_calibrated()

    def is_move_complete(self) -> bool:
        """Checks if the last commanded move is complete (based on cached future)."""
        # Checks if the last commanded move on the underlying Axis object is complete.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        return self._axis.is_move_complete()

    def wait_for_move_completion(self, timeout_sec: Optional[float] = None):
        """Blocks until the current move is complete or timeout occurs."""
        # Synchronously waits for the current move on the underlying Axis object to complete.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The timeout_sec for _submit_coro should be slightly larger than the
        # internal timeout for self._axis.wait_for_move_completion if that one also has a timeout.
        # Or, if self._axis.wait_for_move_completion can run indefinitely (None timeout),
        # then the timeout_sec here will be the effective timeout.
        # This ensures the synchronous call has its own timeout respecting the async operation's potential timeout.
        internal_async_timeout = timeout_sec - 1.0 if timeout_sec and timeout_sec > 1 else timeout_sec
        coro = self._axis.wait_for_move_completion(timeout=internal_async_timeout)
        return self._submit_coro(coro, timeout_sec=timeout_sec)

    def ping(self, timeout_sec: float = 1.0) -> Optional[float]:
        """Pings the motor and returns response time in ms, or None on failure."""
        # Synchronously pings the motor.
        if not self._axis: raise RuntimeError(f"Axis not initialized for {self.axis_name}")
        # The Axis.ping() method in the library already returns response_time_ms or raises an error.
        # So, we expect a float if successful.
        try:
            # Calls the async ping method of the underlying Axis object via _submit_coro.
            response_time = self._submit_coro(self._axis.ping(), timeout_sec=timeout_sec)
            return response_time # Should be float if successful
        except exceptions.CommunicationError: # Catches timeouts or other comms issues from ping.
            logger.warning(f"Ping timed out for {self.axis_name}")
            return None
        except exceptions.MKSServoError as e: # Catches other library-specific errors.
            logger.warning(f"Ping failed for {self.axis_name}: {e}")
            return None

    def __enter__(self):
        # Implements the context manager protocol: connect when entering 'with' block.
        """Context management: connect on entry."""
        self.connect()
        return self # Returns the instance to be used as the target of 'as'.

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Implements the context manager protocol: disconnect when exiting 'with' block.
        # This ensures resources are cleaned up.
        """Context management: disconnect on exit."""
        self.disconnect()

if __name__ == "__main__":
    # This block executes if the script is run directly.
    # It provides an example of how to use the SyncMKSAxis wrapper.
    logger.info("Starting synchronous MKS Servo control example.")

    # --- Configuration ---
    # Default configuration is to use the simulator.
    # For Simulator:
    USE_SIM = True
    # If using the simulator, ensure it's started, e.g.,:
    # mks-servo-simulator --num-motors 1 --start-can-id 1
    MOTOR_ID = 1

    # For Real Hardware (example settings, commented out):
    # Users would uncomment and modify these for their physical setup.
    # USE_SIM = False
    # MOTOR_ID = 1 # Your motor's CAN ID
    # CAN_TYPE = "canable"
    # CAN_CHANNEL = "/dev/ttyACM0" # Linux example, use "COMx" for Windows if applicable
    # CAN_BITRATE_HW = 500000

    # Creates an instance of SyncMKSAxis based on the USE_SIM flag.
    if USE_SIM:
        motor = SyncMKSAxis(motor_can_id=MOTOR_ID, use_simulator=True)
    else:
        # This part for real hardware is commented out to prevent accidental execution
        # without proper hardware configuration.
        # motor = SyncMKSAxis(
        #     motor_can_id=MOTOR_ID,
        #     use_simulator=False,
        #     can_interface_type=CAN_TYPE,
        #     can_channel=CAN_CHANNEL,
        #     can_bitrate=CAN_BITRATE_HW
        # )
        logger.error("Real hardware part of example is commented out. Set USE_SIM=False and configure above.")
        exit() # Exits if real hardware is selected but not configured.

    try:
        # Demonstrates the typical sequence of operations using the synchronous wrapper.
        logger.info("Connecting to motor...")
        motor.connect() # This is a blocking call due to the wrapper.
        logger.info("Motor connected.")

        logger.info("Enabling motor...")
        motor.enable_motor() # Blocking call.
        if motor.is_enabled(): # Blocking call.
            logger.info("Motor is enabled.")
        else:
            logger.error("Failed to enable motor.")
            motor.disconnect() # Clean up.
            exit() # Exit if enable fails.

        # Gets the initial position in user units (degrees by default).
        initial_pos = motor.get_current_position_user() # Blocking.
        logger.info(f"Initial position: {initial_pos:.2f} {motor._kinematics.units}") # Accessing kinematics units for example.

        logger.info("Moving to 90.0 degrees...")
        # Commands an absolute move to 90.0 degrees at 180 deg/s.
        # 'wait_for_completion=True' means this call will block until the move is done.
        motor.move_absolute_user(target_pos_user=90.0, speed_user=180.0, wait_for_completion=True) # Blocking.
        logger.info(f"Position after move: {motor.get_current_position_user():.2f} {motor._kinematics.units}")

        time.sleep(1) # Pauses for 1 second.

        logger.info("Moving relatively by -30.0 degrees...")
        # Commands a relative move by -30.0 degrees.
        motor.move_relative_user(relative_dist_user=-30.0, speed_user=90.0, wait_for_completion=True) # Blocking.
        logger.info(f"Position after relative move: {motor.get_current_position_user():.2f} {motor._kinematics.units}")

        time.sleep(1) # Pauses again.

        logger.info("Setting current position as zero...")
        # Sets the current motor position as the new zero reference.
        motor.set_current_position_as_zero() # Blocking.
        logger.info(f"New current position: {motor.get_current_position_user():.2f} {motor._kinematics.units}")

        logger.info("Disabling motor...")
        motor.disable_motor() # Blocking.
        logger.info("Motor disabled.")

    except TimeoutError as e:
        # Catches timeout errors specifically raised by the wrapper.
        logger.error(f"Operation timed out: {e}")
    except exceptions.MKSServoError as e:
        # Catches any library-specific errors.
        logger.error(f"MKS Servo library error: {e}", exc_info=True)
    except Exception as e:
        # Catches any other unexpected errors.
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # Ensures disconnection happens even if errors occur.
        logger.info("Shutting down motor connection...")
        if motor: # Check if motor object was successfully created.
            motor.disconnect() # Blocking disconnect call.
        logger.info("Synchronous example finished.")
        