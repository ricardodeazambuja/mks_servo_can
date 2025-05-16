# mks_servo_can_project/mks_servo_can_library/mks_servo_can/multi_axis_controller.py
"""
MultiAxisController for managing and coordinating multiple MKS Servo axes.
"""
import asyncio
import logging
from typing import List, Dict, Optional, Any, Tuple

from .axis import Axis
from .can_interface import CANInterface
from .exceptions import ConfigurationError, MultiAxisError, MKSServoError
from .kinematics import Kinematics # For type hinting if needed

logger = logging.getLogger(__name__)

class MultiAxisController:
    """
    Manages a collection of Axis objects for coordinated control and group operations.
    """
    def __init__(self, can_interface_manager: CANInterface, loop: Optional[asyncio.AbstractEventLoop] = None):
        """
        Initialize the MultiAxisController.

        Args:
            can_interface_manager: The shared CANInterface instance.
            loop: The asyncio event loop.
        """
        self.can_if = can_interface_manager
        self.axes: Dict[str, Axis] = {} # Store axes by name
        self._loop = loop if loop else asyncio.get_event_loop()

    def add_axis(self, axis: Axis) -> None:
        """Adds an axis to the controller."""
        if axis.name in self.axes:
            raise ConfigurationError(f"Axis with name '{axis.name}' already exists in controller.")
        if axis._can_if is not self.can_if:
            raise ConfigurationError(f"Axis '{axis.name}' is using a different CAN interface instance.")
        
        # Check for CAN ID conflicts
        for existing_axis in self.axes.values():
            if existing_axis.can_id == axis.can_id:
                raise ConfigurationError(f"CAN ID {axis.can_id} for axis '{axis.name}' conflicts with existing axis '{existing_axis.name}'.")
        
        self.axes[axis.name] = axis
        logger.info(f"Added axis '{axis.name}' (CAN ID: {axis.can_id:03X}) to MultiAxisController.")

    def get_axis(self, name: str) -> Optional[Axis]:
        """Retrieves an axis by its name."""
        return self.axes.get(name)

    @property
    def axis_names(self) -> List[str]:
        """Returns a list of names of all managed axes."""
        return list(self.axes.keys())

    async def initialize_all_axes(self, calibrate: bool = False, home: bool = False, concurrent: bool = True) -> None:
        """
        Initializes all managed axes.

        Args:
            calibrate: If True, calibrate each axis.
            home: If True, home each axis after calibration (if performed).
            concurrent: If True, initialize axes concurrently. Otherwise, sequentially.
        """
        logger.info("Initializing all axes...")
        tasks = [axis.initialize(calibrate=calibrate, home=home) for axis in self.axes.values()]
        results = {}
        
        if concurrent:
            # gather will run them concurrently and collect results or exceptions
            task_results = await asyncio.gather(*tasks, return_exceptions=True)
            for i, axis_name in enumerate(self.axis_names):
                results[axis_name] = task_results[i]
        else:
            for axis_name, axis in self.axes.items():
                try:
                    await axis.initialize(calibrate=calibrate, home=home)
                    results[axis_name] = "Success"
                except MKSServoError as e:
                    results[axis_name] = e
        
        # Report outcomes
        all_successful = True
        for axis_name, result in results.items():
            if isinstance(result, Exception):
                logger.error(f"Error initializing axis '{axis_name}': {result}")
                all_successful = False
            else:
                logger.info(f"Axis '{axis_name}' initialized successfully.")
        
        if not all_successful:
            # Filter out non-exception results to build individual_errors for MultiAxisError
            errors_dict = {name: res for name, res in results.items() if isinstance(res, Exception)}
            raise MultiAxisError("One or more axes failed to initialize.", individual_errors=errors_dict)
        logger.info("All axes initialized.")


    # --- Group Operations ---
    async def _execute_on_axes(self, method_name: str, *args, concurrent: bool = True, **kwargs) -> Dict[str, Any]:
        """Helper to execute a method on multiple axes, optionally concurrently."""
        results: Dict[str, Any] = {}
        tasks = []

        for axis_name, axis in self.axes.items():
            method_to_call = getattr(axis, method_name, None)
            if not callable(method_to_call):
                results[axis_name] = ConfigurationError(f"Method '{method_name}' not found on axis '{axis_name}'.")
                continue
            
            if concurrent:
                tasks.append(self._loop.create_task(method_to_call(*args, **kwargs), name=f"{axis_name}_{method_name}"))
            else: # Sequential execution
                try:
                    results[axis_name] = await method_to_call(*args, **kwargs)
                except MKSServoError as e:
                    results[axis_name] = e # Store the exception
                except Exception as e:
                    results[axis_name] = MKSServoError(f"Unexpected error on axis {axis_name}: {e}")


        if concurrent and tasks:
            # Wait for all concurrent tasks to complete
            done, pending = await asyncio.wait(tasks, return_when=asyncio.ALL_COMPLETED)
            for task in done:
                axis_name_from_task = task.get_name().split('_')[0] # Extract axis name
                try:
                    results[axis_name_from_task] = task.result()
                except Exception as e: # Catch exceptions from tasks
                    results[axis_name_from_task] = e
        
        # Check for any errors and potentially raise a MultiAxisError
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            # If the method was expected to return something, other results might be valid.
            # For void methods, just raise if any error.
            # This behavior can be refined based on specific method needs.
            logger.warning(f"Errors occurred during group operation '{method_name}': {errors_found}")
            # raise MultiAxisError(f"Errors during group operation '{method_name}'.", individual_errors=errors_found)
        return results


    async def home_all_axes(self, wait_for_completion: bool = True, concurrent: bool = True, timeout_per_axis: float = 30.0) -> None:
        """Homes all managed axes."""
        logger.info(f"Homing all axes (concurrent={concurrent})...")
        # The 'home_axis' method has its own wait_for_completion and timeout.
        # The timeout here in _execute_on_axes would be for the task itself if concurrent.
        results = await self._execute_on_axes(
            "home_axis", 
            wait_for_completion=wait_for_completion, 
            timeout=timeout_per_axis, # This timeout is passed to axis.home_axis
            concurrent=concurrent
        )
        
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            raise MultiAxisError("One or more axes failed during homing.", individual_errors=errors_found)
        logger.info("All axes homing process initiated/completed.")


    async def enable_all_axes(self, concurrent: bool = True) -> None:
        """Enables all managed axes."""
        logger.info("Enabling all axes...")
        await self._execute_on_axes("enable_motor", concurrent=concurrent)
        logger.info("Enable command sent to all axes.")

    async def disable_all_axes(self, concurrent: bool = True) -> None:
        """Disables all managed axes."""
        logger.info("Disabling all axes...")
        await self._execute_on_axes("disable_motor", concurrent=concurrent)
        logger.info("Disable command sent to all axes.")

    async def emergency_stop_all_axes(self, concurrent: bool = True) -> None:
        """Emergency stops all managed axes."""
        logger.warning("EMERGENCY STOPPING ALL AXES.")
        await self._execute_on_axes("emergency_stop", concurrent=concurrent)
        logger.warning("Emergency stop command sent to all axes.")
        
    async def stop_all_axes(self, deceleration_param: Optional[int] = None, concurrent: bool = True) -> None:
        """Stops all axes using their default or specified deceleration."""
        logger.info("Stopping all axes...")
        await self._execute_on_axes("stop_motor", deceleration_param=deceleration_param, concurrent=concurrent)
        logger.info("Stop command sent to all axes.")


    # --- Coordinated Motion (Conceptual - MKS CAN doesn't directly support synchronized multi-axis moves) ---
    # True coordinated motion (like linear interpolation) requires the controller (this Python code)
    # to manage the timing and potentially profile the moves.
    # The MKS servos execute commands independently.

    async def move_all_to_positions_abs_user(
        self,
        positions_user: Dict[str, float], # axis_name -> target_pos_user
        speeds_user: Optional[Dict[str, float]] = None, # axis_name -> speed_user
        wait_for_all: bool = True
    ) -> None:
        """
        Moves multiple axes to specified absolute positions in user units.
        Axes will start moving concurrently but are not guaranteed to finish simultaneously
        unless profiled and speeds are adjusted (advanced).

        Args:
            positions_user: Dict mapping axis name to target user position.
            speeds_user: Optional dict mapping axis name to desired user speed.
                         If None, axis default speed is used.
            wait_for_all: If True, waits for all moves to complete.
        """
        logger.info(f"Moving multiple axes to absolute user positions: {positions_user}")
        if not positions_user:
            return

        tasks = []
        active_futures = {}

        for axis_name, target_pos in positions_user.items():
            axis = self.axes.get(axis_name)
            if not axis:
                logger.warning(f"Axis '{axis_name}' not found in controller for multi-move.")
                continue

            axis_speed_user = None
            if speeds_user and axis_name in speeds_user:
                axis_speed_user = speeds_user[axis_name]
            
            # axis.move_to_position_abs_user does not return a future directly in its signature for awaiting.
            # It uses an internal _active_move_future.
            # We need to call it and then await its internal future.
            
            # Create task to start the move
            # The move method itself uses an internal future for completion
            tasks.append(
                self._loop.create_task(
                    axis.move_to_position_abs_user(target_pos, speed_user=axis_speed_user, wait=False), # wait=False to not block here
                    name=f"{axis_name}_move_abs"
                )
            )
            active_futures[axis_name] = axis._active_move_future # Get the future set by the move command

        # Wait for all move initiation tasks to complete (i.e., command sent)
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True) # Check for errors during initiation
        
        # If waiting for completion, wait for all active move futures
        if wait_for_all and active_futures:
            logger.info("Waiting for all initiated moves to complete...")
            completion_tasks = [fut for fut in active_futures.values() if fut] # Filter out None futures
            if completion_tasks:
                await asyncio.gather(*completion_tasks, return_exceptions=True) # Check for errors during move execution
        
        logger.info("Multi-axis absolute move command sequence finished.")
        # Further error aggregation from futures can be done here.


    async def move_all_relative_user(
        self,
        distances_user: Dict[str, float], # axis_name -> relative_dist_user
        speeds_user: Optional[Dict[str, float]] = None,
        wait_for_all: bool = True
    ) -> None:
        """Moves multiple axes by specified relative distances in user units."""
        logger.info(f"Moving multiple axes by relative user distances: {distances_user}")
        if not distances_user:
            return

        tasks = []
        active_futures = {}

        for axis_name, rel_dist in distances_user.items():
            axis = self.axes.get(axis_name)
            if not axis:
                logger.warning(f"Axis '{axis_name}' not found for multi-relative move.")
                continue
            
            axis_speed_user = None
            if speeds_user and axis_name in speeds_user:
                axis_speed_user = speeds_user[axis_name]

            tasks.append(
                self._loop.create_task(
                    axis.move_relative_user(rel_dist, speed_user=axis_speed_user, wait=False),
                    name=f"{axis_name}_move_rel"
                )
            )
            active_futures[axis_name] = axis._active_move_future

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        
        if wait_for_all and active_futures:
            logger.info("Waiting for all initiated relative moves to complete...")
            completion_tasks = [fut for fut in active_futures.values() if fut]
            if completion_tasks:
                 await asyncio.gather(*completion_tasks, return_exceptions=True)
        
        logger.info("Multi-axis relative move command sequence finished.")


    # --- Aggregated Feedback ---
    async def get_all_positions_user(self) -> Dict[str, float]:
        """Gets current positions of all axes in user units."""
        results = await self._execute_on_axes("get_current_position_user", concurrent=True)
        # Filter out exceptions if any, or handle them
        valid_results = {name: pos for name, pos in results.items() if not isinstance(pos, Exception)}
        return valid_results

    async def get_all_statuses(self) -> Dict[str, Dict[str, Any]]:
        """Gets detailed status dictionaries for all axes."""
        results = await self._execute_on_axes("get_status_dict", concurrent=True)
        valid_results = {name: stat for name, stat in results.items() if not isinstance(stat, Exception)}
        return valid_results

    def are_all_moves_complete(self) -> bool:
        """Checks if all axes have completed their last initiated moves."""
        if not self.axes: return True # No axes, so no moves
        return all(axis.is_move_complete() for axis in self.axes.values())

    async def wait_for_all_moves_to_complete(self, timeout_per_axis: Optional[float] = None) -> None:
        """Waits for all currently active moves on all axes to complete."""
        logger.info("Waiting for all axes to complete their moves...")
        tasks = [
            axis.wait_for_move_completion(timeout=timeout_per_axis)
            for axis in self.axes.values() if not axis.is_move_complete()
        ]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True) # Handle exceptions if needed
        logger.info("All active moves are complete.")

    # Consider adding methods for kinematic chains / robot models if needed (advanced)