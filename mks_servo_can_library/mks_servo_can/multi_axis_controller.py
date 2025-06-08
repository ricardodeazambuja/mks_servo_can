"""
MultiAxisController for managing and coordinating multiple MKS Servo axes.

This module provides the `MultiAxisController` class, which facilitates
the control of multiple `Axis` objects simultaneously. It allows for
group operations such as enabling all axes, homing all axes, or commanding
coordinated movements (to the extent supported by the underlying MKS CAN protocol,
which typically means concurrent command dispatch rather than true interpolated
multi-axis motion).
"""
import math

from typing import Any, Dict, List, Optional

import asyncio
import logging

from .axis import Axis
from .can_interface import CANInterface
from .exceptions import ConfigurationError
from .exceptions import MKSServoError
from .exceptions import MultiAxisError
from .kinematics import Kinematics  # For type hinting if needed

logger = logging.getLogger(__name__)


class MultiAxisController:
    """
    Manages a collection of Axis objects for coordinated control and group operations.

    The `MultiAxisController` simplifies tasks involving multiple MKS servo motors
    by providing methods to perform actions (e.g., initialization, enabling,
    movement commands) on all managed axes concurrently or sequentially.
    It requires a shared `CANInterface` instance for all its axes.
    """

    def __init__(
        self,
        can_interface_manager: CANInterface,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ):
        """
        Initialize the MultiAxisController.

        Args:
            can_interface_manager: The shared `CANInterface` instance that all axes
                                   managed by this controller will use for communication.
            loop: The asyncio event loop to use. If None, `asyncio.get_event_loop()`
                  will be used.
        """
        self.can_if = can_interface_manager
        self.axes: Dict[str, Axis] = {}  # Store axes by name
        self._loop = loop if loop else asyncio.get_event_loop()

    def add_axis(self, axis: Axis) -> None:
        """
        Adds an already initialized Axis object to the controller.

        The provided Axis object must use the same `CANInterface` instance
        as the `MultiAxisController`. The axis is stored and can be accessed
        by its `name` attribute.

        Args:
            axis: The `Axis` object to add to the controller.

        Raises:
            ConfigurationError: If an axis with the same name already exists,
                                if the axis uses a different `CANInterface` instance,
                                or if the axis's CAN ID conflicts with an already
                                added axis.
        """
        if axis.name in self.axes:
            raise ConfigurationError(
                f"Axis with name '{axis.name}' already exists in controller."
            )
        if axis._can_if is not self.can_if: # Accessing protected member _can_if for this check
            raise ConfigurationError(
                f"Axis '{axis.name}' is using a different CAN interface instance. "
                "All axes in a MultiAxisController must share the same CANInterface."
            )

        # Check for CAN ID conflicts
        for existing_axis in self.axes.values():
            if existing_axis.can_id == axis.can_id:
                raise ConfigurationError(
                    f"CAN ID {axis.can_id} for axis '{axis.name}' conflicts with "
                    f"existing axis '{existing_axis.name}'."
                )

        self.axes[axis.name] = axis
        logger.info(
            f"Added axis '{axis.name}' (CAN ID: {axis.can_id:03X}) to MultiAxisController."
        )

    def get_axis(self, name: str) -> Optional[Axis]:
        """
        Retrieves an axis managed by this controller by its unique name.

        Args:
            name: The name of the axis to retrieve.

        Returns:
            The `Axis` object if found, otherwise None.
        """
        return self.axes.get(name)

    @property
    def axis_names(self) -> List[str]:
        """
        Returns a list of names of all axes currently managed by this controller.

        Returns:
            A list of strings, where each string is the name of an axis.
        """
        return list(self.axes.keys())

    async def initialize_all_axes(
        self,
        calibrate: bool = False,
        home: bool = False,
        concurrent: bool = True,
    ) -> None:
        """
        Initializes all managed axes, optionally performing calibration and homing.

        This method calls the `initialize()` method on each `Axis` object
        managed by the controller.

        Args:
            calibrate: If True, each axis will attempt to calibrate its encoder
                       during initialization.
            home: If True, each axis will attempt to perform its homing sequence
                  after successful initialization (and calibration, if performed).
            concurrent: If True (default), initialization of all axes is attempted
                        concurrently using `asyncio.gather`. If False, axes are
                        initialized sequentially one by one.

        Raises:
            MultiAxisError: If one or more axes fail to initialize. The `individual_errors`
                            attribute of the exception will contain details about which
                            axes failed and the specific error for each.
        """
        logger.info("Initializing all axes...")
        tasks = [
            axis.initialize(calibrate=calibrate, home=home)
            for axis in self.axes.values()
        ]
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
            errors_dict = {
                name: res
                for name, res in results.items()
                if isinstance(res, Exception)
            }
            raise MultiAxisError(
                "One or more axes failed to initialize.",
                individual_errors=errors_dict,
            )
        logger.info("All axes initialized.")

    async def _execute_on_axes(
        self, method_name: str, *args, concurrent: bool = True, **kwargs
    ) -> Dict[str, Any]:
        """
        Helper method to execute a given method on all managed axes.

        This internal method provides a common pattern for applying an operation
        (specified by `method_name`) to all axes, either concurrently or sequentially.
        It collects results or exceptions from each axis.

        Args:
            method_name: The name of the method to call on each `Axis` object.
            *args: Positional arguments to pass to the method.
            concurrent: If True (default), execute the method on all axes concurrently.
                        If False, execute sequentially.
            **kwargs: Keyword arguments to pass to the method.

        Returns:
            A dictionary where keys are axis names and values are the results
            of the method call for that axis, or an Exception object if an error
            occurred for that axis.

        Note:
            If `concurrent` is True, this method uses `asyncio.wait` for task completion,
            which might behave differently from `asyncio.gather` in terms of error
            propagation if not handled carefully. The current implementation catches
            exceptions from tasks.
        """
        results: Dict[str, Any] = {}
        tasks = []

        for axis_name, axis in self.axes.items():
            method_to_call = getattr(axis, method_name, None)
            if not callable(method_to_call):
                results[axis_name] = ConfigurationError(
                    f"Method '{method_name}' not found on axis '{axis_name}'."
                )
                continue

            if concurrent:
                tasks.append(
                    self._loop.create_task(
                        method_to_call(*args, **kwargs),
                        name=f"{axis_name}_{method_name}",
                    )
                )
            else:  # Sequential execution
                try:
                    results[axis_name] = await method_to_call(*args, **kwargs)
                except MKSServoError as e:
                    results[axis_name] = e  # Store the exception
                except Exception as e:
                    results[axis_name] = MKSServoError(
                        f"Unexpected error on axis {axis_name}: {e}"
                    )

        if concurrent and tasks:
            # Wait for all concurrent tasks to complete
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.ALL_COMPLETED
            )
            # Process results from completed tasks
            for task in done:
                # Extract axis name from task name (assuming format "AxisName_methodName")
                try:
                    axis_name_from_task = task.get_name().split(f"_{method_name}")[0]
                except Exception: # pylint: disable=broad-except
                    # Fallback if task name doesn't match expected format
                    axis_name_from_task = f"UnknownTask_{task.get_name()}"

                try:
                    results[axis_name_from_task] = task.result()
                except Exception as e:  # Catch exceptions from tasks
                    results[axis_name_from_task] = e
            # Handle any tasks that might be in the pending set, though with ALL_COMPLETED they should be empty
            for task in pending:
                logger.error(f"Task {task.get_name()} was still pending after asyncio.wait with ALL_COMPLETED.")
                task.cancel() # Attempt to cancel pending tasks

        # Check for any errors and potentially raise a MultiAxisError
        errors_found = {
            name: res
            for name, res in results.items()
            if isinstance(res, Exception)
        }
        if errors_found:
            # If the method was expected to return something, other results might be valid.
            # For void methods, just raise if any error.
            # This behavior can be refined based on specific method needs.
            logger.warning(
                f"Errors occurred during group operation '{method_name}': {errors_found}"
            )
            # raise MultiAxisError(f"Errors during group operation '{method_name}'.", individual_errors=errors_found)
        return results

    async def home_all_axes(
        self,
        wait_for_completion: bool = True,
        concurrent: bool = True,
        timeout_per_axis: float = 30.0,
    ) -> None:
        """
        Commands all managed axes to perform their homing sequence.

        Args:
            wait_for_completion: If True (default), waits for each axis's homing
                                 process to complete.
            concurrent: If True (default), homing commands are initiated concurrently
                        for all axes. If False, axes are homed sequentially.
            timeout_per_axis: Maximum time in seconds to wait for each individual
                              axis's homing completion if `wait_for_completion` is True.
                              This timeout is passed to each axis's `home_axis` method.

        Raises:
            MultiAxisError: If one or more axes fail during the homing process.
                            The `individual_errors` attribute will contain details.
        """
        logger.info(f"Homing all axes (concurrent={concurrent})...")
        # The 'home_axis' method has its own wait_for_completion and timeout.
        # The timeout here in _execute_on_axes would be for the task itself if concurrent.
        results = await self._execute_on_axes(
            "home_axis",
            wait_for_completion=wait_for_completion,
            timeout=timeout_per_axis,  # This timeout is passed to axis.home_axis
            concurrent=concurrent,
        )

        errors_found = {
            name: res
            for name, res in results.items()
            if isinstance(res, Exception)
        }
        if errors_found:
            raise MultiAxisError(
                "One or more axes failed during homing.",
                individual_errors=errors_found,
            )
        logger.info("All axes homing process initiated/completed.")

    async def enable_all_axes(self, concurrent: bool = True) -> None:
        """
        Enables the servo loop for all managed axes.

        Args:
            concurrent: If True (default), enable commands are sent concurrently.
                        If False, axes are enabled sequentially.

        Raises:
            MultiAxisError: If enabling one or more axes fails. `individual_errors`
                            will contain details for each failed axis.
        """
        logger.info("Enabling all axes...")
        results = await self._execute_on_axes("enable_motor", concurrent=concurrent)
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            raise MultiAxisError("Failed to enable one or more axes.", individual_errors=errors_found)
        logger.info("Enable command sent to all axes successfully (or errors raised).")


    async def disable_all_axes(self, concurrent: bool = True) -> None:
        """
        Disables the servo loop for all managed axes.

        Args:
            concurrent: If True (default), disable commands are sent concurrently.
                        If False, axes are disabled sequentially.

        Raises:
            MultiAxisError: If disabling one or more axes fails. `individual_errors`
                            will contain details for each failed axis.
        """
        logger.info("Disabling all axes...")
        results = await self._execute_on_axes("disable_motor", concurrent=concurrent)
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            raise MultiAxisError("Failed to disable one or more axes.", individual_errors=errors_found)
        logger.info("Disable command sent to all axes successfully (or errors raised).")


    async def emergency_stop_all_axes(self, concurrent: bool = True) -> None:
        """
        Sends an emergency stop command to all managed axes.

        Args:
            concurrent: If True (default), E-stop commands are sent concurrently.
                        If False, E-stop commands are sent sequentially.
        Raises:
            MultiAxisError: If sending E-stop to one or more axes fails.
                           `individual_errors` will contain details.
        """
        logger.warning("EMERGENCY STOPPING ALL AXES.")
        results = await self._execute_on_axes("emergency_stop", concurrent=concurrent)
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            raise MultiAxisError("Failed to emergency stop one or more axes.", individual_errors=errors_found)
        logger.warning("Emergency stop command sent to all axes successfully (or errors raised).")


    async def stop_all_axes(
        self, deceleration_param: Optional[int] = None, concurrent: bool = True
    ) -> None:
        """
        Stops any ongoing movement on all managed axes using their default or specified deceleration.

        This typically uses the motor's standard stop command (e.g., speed mode stop).

        Args:
            deceleration_param (Optional[int]): The MKS deceleration parameter (0-255)
                                                to use for stopping. If None, each axis
                                                might use its default.
            concurrent (bool): If True (default), stop commands are sent concurrently.
                               If False, axes are stopped sequentially.
        Raises:
            MultiAxisError: If stopping one or more axes fails.
                           `individual_errors` will contain details.
        """
        logger.info("Stopping all axes...")
        results = await self._execute_on_axes(
            "stop_motor",
            deceleration_param=deceleration_param,
            concurrent=concurrent,
        )
        errors_found = {name: res for name, res in results.items() if isinstance(res, Exception)}
        if errors_found:
            raise MultiAxisError("Failed to stop one or more axes.", individual_errors=errors_found)
        logger.info("Stop command sent to all axes successfully (or errors raised).")


    async def move_all_to_positions_abs_user(
        self,
        positions_user: Dict[str, float],  # axis_name -> target_pos_user
        speeds_user: Optional[
            Dict[str, float]
        ] = None,  # axis_name -> speed_user
        wait_for_all: bool = True,
    ) -> None:
        """
        Moves multiple axes to specified absolute positions in their user-defined units.

        Commands are dispatched to all specified axes concurrently. If `wait_for_all`
        is True, this method will wait until all participating axes report that their
        move is complete or has failed.

        Note on Synchronization:
        The MKS servo motors execute commands independently once received. This method
        ensures commands *start* nearly simultaneously. True path-synchronized
        motion (e.g., linear interpolation in a 2D/3D space) typically requires
        a higher-level motion planner to calculate and stream small, coordinated
        move segments to each axis. This method provides "start-together, finish-when-all-done"
        coordination.

        Args:
            positions_user: A dictionary mapping axis names (str) to their target
                            absolute positions in user units (float).
            speeds_user: An optional dictionary mapping axis names (str) to their
                         desired speeds in user units per second (float). If an axis
                         is not included or if this argument is None, the axis's
                         default speed (or MKS default if not set on axis) will be used.
            wait_for_all: If True (default), this method will not return until all
                          commanded moves have completed or an error occurs on one of the axes.
                          If False, commands are dispatched, and the method returns immediately.
                          Completion can then be tracked using `are_all_moves_complete()`
                          and `wait_for_all_moves_to_complete()`.

        Raises:
            MultiAxisError: If initiating or executing the move fails for one or more axes.
                            The `individual_errors` attribute will contain details.
            KeyError: If an axis name in `positions_user` or `speeds_user` is not
                      found in the controller.
        """
        logger.info(
            f"Moving multiple axes to absolute user positions: {positions_user}"
        )
        if not positions_user:
            return

        tasks = []
        active_futures_map: Dict[str, Optional[asyncio.Future]] = {} # Store future for each axis

        initiation_errors: Dict[str, Exception] = {}

        for axis_name, target_pos in positions_user.items():
            axis = self.axes.get(axis_name)
            if not axis:
                msg = f"Axis '{axis_name}' not found in controller for multi-move."
                logger.warning(msg)
                initiation_errors[axis_name] = ConfigurationError(msg)
                continue

            axis_speed_user = None
            if speeds_user and axis_name in speeds_user:
                axis_speed_user = speeds_user[axis_name]

            # Create a task to initiate the move on the axis.
            # The axis.move_to_position_abs_user itself handles setting _active_move_future.
            async def _initiate_move(ax: Axis, pos: float, speed: Optional[float]):
                try:
                    await ax.move_to_position_abs_user(pos, speed_user=speed, wait=False)
                    active_futures_map[ax.name] = ax._active_move_future # pylint: disable=protected-access
                except Exception as e: # pylint: disable=broad-except
                    initiation_errors[ax.name] = e
                    active_futures_map[ax.name] = None # Ensure no stale future

            tasks.append(
                self._loop.create_task(
                    _initiate_move(axis, target_pos, axis_speed_user),
                    name=f"{axis_name}_move_abs_init",
                )
            )

        # Wait for all move initiation tasks to complete
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

        if initiation_errors:
            raise MultiAxisError("Error(s) initiating multi-axis absolute move.", individual_errors=initiation_errors)

        # If waiting for completion, wait for all valid active move futures
        if wait_for_all:
            futures_to_wait = [f for f in active_futures_map.values() if f and not f.done()]
            if futures_to_wait:
                logger.info("Waiting for all initiated absolute moves to complete...")
                completion_results = await asyncio.gather(*futures_to_wait, return_exceptions=True)
                
                # Check for errors during move execution
                move_execution_errors: Dict[str, Exception] = {}
                for axis_name, fut in active_futures_map.items():
                    if fut and fut.done() and fut.exception():
                        move_execution_errors[axis_name] = fut.exception() # type: ignore

                if move_execution_errors:
                    raise MultiAxisError("Error(s) during multi-axis absolute move execution.", individual_errors=move_execution_errors)

        logger.info("Multi-axis absolute move command sequence finished.")


    async def move_all_relative_user(
        self,
        distances_user: Dict[str, float],  # axis_name -> relative_dist_user
        speeds_user: Optional[Dict[str, float]] = None,
        wait_for_all: bool = True,
    ) -> None:
        """
        Moves multiple axes by specified relative distances in their user-defined units.

        Similar to `move_all_to_positions_abs_user`, commands are dispatched
        concurrently. If `wait_for_all` is True, the method waits for all
        moves to complete.

        Args:
            distances_user: A dictionary mapping axis names (str) to their desired
                            relative move distances in user units (float).
            speeds_user: An optional dictionary mapping axis names (str) to speeds
                         in user units per second.
            wait_for_all: If True (default), waits for all moves to complete.

        Raises:
            MultiAxisError: If initiating or executing the move fails for one or more axes.
            KeyError: If an axis name is not found.
        """
        logger.info(
            f"Moving multiple axes by relative user distances: {distances_user}"
        )
        if not distances_user:
            return

        tasks = []
        active_futures_map: Dict[str, Optional[asyncio.Future]] = {}
        initiation_errors: Dict[str, Exception] = {}

        for axis_name, rel_dist in distances_user.items():
            axis = self.axes.get(axis_name)
            if not axis:
                msg = f"Axis '{axis_name}' not found for multi-relative move."
                logger.warning(msg)
                initiation_errors[axis_name] = ConfigurationError(msg)
                continue

            axis_speed_user = None
            if speeds_user and axis_name in speeds_user:
                axis_speed_user = speeds_user[axis_name]

            async def _initiate_move(ax: Axis, dist: float, speed: Optional[float]):
                try:
                    await ax.move_relative_user(dist, speed_user=speed, wait=False)
                    active_futures_map[ax.name] = ax._active_move_future # pylint: disable=protected-access
                except Exception as e: # pylint: disable=broad-except
                    initiation_errors[ax.name] = e
                    active_futures_map[ax.name] = None

            tasks.append(
                self._loop.create_task(
                    _initiate_move(axis, rel_dist, axis_speed_user),
                    name=f"{axis_name}_move_rel_init",
                )
            )

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        
        if initiation_errors:
            raise MultiAxisError("Error(s) initiating multi-axis relative move.", individual_errors=initiation_errors)

        if wait_for_all:
            futures_to_wait = [f for f in active_futures_map.values() if f and not f.done()]
            if futures_to_wait:
                logger.info(
                    "Waiting for all initiated relative moves to complete..."
                )
                completion_results = await asyncio.gather(*futures_to_wait, return_exceptions=True)
                move_execution_errors: Dict[str, Exception] = {}
                for axis_name, fut in active_futures_map.items():
                    if fut and fut.done() and fut.exception():
                        move_execution_errors[axis_name] = fut.exception() # type: ignore
                if move_execution_errors:
                    raise MultiAxisError("Error(s) during multi-axis relative move execution.", individual_errors=move_execution_errors)


        logger.info("Multi-axis relative move command sequence finished.")


    async def get_all_positions_user(self) -> Dict[str, float]:
        """
        Gets the current positions of all managed axes in their user-defined units.

        This method calls `get_current_position_user()` on each axis concurrently.

        Returns:
            A dictionary mapping axis names (str) to their current positions
            in user units (float). Axes that failed to report their position
            (e.g., due to a communication error) will be excluded, and errors
            will be logged.

        Raises:
            MultiAxisError: If fetching positions fails for one or more axes,
                            this exception is raised containing details of individual errors.
                            If all axes succeed, no exception is raised.
        """
        results = await self._execute_on_axes(
            "get_current_position_user", concurrent=True
        )
        
        valid_results: Dict[str, float] = {}
        errors_found: Dict[str, Exception] = {}

        for name, pos_or_err in results.items():
            if isinstance(pos_or_err, Exception):
                errors_found[name] = pos_or_err
                logger.error(f"Failed to get position for axis '{name}': {pos_or_err}")
            elif isinstance(pos_or_err, float): # Ensure it's a float
                valid_results[name] = pos_or_err
            else: # Should not happen if Axis.get_current_position_user returns float or raises
                 errors_found[name] = TypeError(f"Unexpected return type for axis '{name}': {type(pos_or_err)}")
                 logger.error(f"Unexpected return type for axis '{name}' getting position: {type(pos_or_err)}")


        if errors_found:
            raise MultiAxisError("Failed to get positions for one or more axes.", individual_errors=errors_found)
            
        return valid_results

    async def get_all_statuses(self) -> Dict[str, Dict[str, Any]]:
        """
        Gets detailed status dictionaries for all managed axes concurrently.

        This method calls `get_status_dict()` on each axis.

        Returns:
            A dictionary mapping axis names (str) to their status dictionaries.
            Axes that failed to report their status will be excluded, and errors
            will be logged.

        Raises:
            MultiAxisError: If fetching status fails for one or more axes,
                            this exception is raised containing details of individual errors.
        """
        results = await self._execute_on_axes(
            "get_status_dict", concurrent=True
        )
        valid_results: Dict[str, Dict[str, Any]] = {}
        errors_found: Dict[str, Exception] = {}

        for name, stat_or_err in results.items():
            if isinstance(stat_or_err, Exception):
                errors_found[name] = stat_or_err
                logger.error(f"Failed to get status for axis '{name}': {stat_or_err}")
            elif isinstance(stat_or_err, dict):
                 valid_results[name] = stat_or_err
            else:
                 errors_found[name] = TypeError(f"Unexpected return type for axis '{name}': {type(stat_or_err)}")
                 logger.error(f"Unexpected return type for axis '{name}' getting status: {type(stat_or_err)}")

        if errors_found:
            raise MultiAxisError("Failed to get status for one or more axes.", individual_errors=errors_found)

        return valid_results

    def are_all_moves_complete(self) -> bool:
        """
        Checks if all managed axes have completed their last initiated moves.

        This iterates through all axes and calls their `is_move_complete()` method.

        Returns:
            True if all axes have completed their moves (or no moves are active),
            False otherwise. Returns True if no axes are managed.
        """
        if not self.axes:
            return True  # No axes, so no moves
        return all(axis.is_move_complete() for axis in self.axes.values())

    async def wait_for_all_moves_to_complete(
        self, timeout_per_axis: Optional[float] = None
    ) -> None:
        """
        Asynchronously waits until all currently active moves on all managed axes
        are complete, or a timeout occurs for any of them.

        This method gathers the `wait_for_move_completion(timeout_per_axis)`
        coroutines for all axes that are not already reporting their move as complete.

        Args:
            timeout_per_axis (Optional[float]): The maximum time in seconds to wait
                                                for each individual axis's move to complete.
                                                If None, each axis might wait indefinitely
                                                or based on its internal move future's timeout.

        Raises:
            MultiAxisError: If one or more axes fail to complete their move
                            (e.g., due to timeout passed to the axis, or other motor error).
                            The `individual_errors` attribute will detail these failures.
        """
        logger.info("Waiting for all axes to complete their moves...")
        tasks = [
            axis.wait_for_move_completion(timeout=timeout_per_axis)
            for axis in self.axes.values()
            if not axis.is_move_complete()
        ]
        if tasks:
            results = await asyncio.gather(
                *tasks, return_exceptions=True
            )
            # Check for exceptions in results
            errors_found: Dict[str, Exception] = {}
            axis_list = [axis for axis in self.axes.values() if not axis.is_move_complete()] # Re-create list of axes that had pending moves
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    if i < len(axis_list): # Ensure index is valid
                        axis_name = axis_list[i].name
                        errors_found[axis_name] = result
                        logger.error(f"Error waiting for move completion on axis '{axis_name}': {result}")
                    else: # Should not happen if tasks and axis_list align
                        logger.error(f"Mismatch in results and axis list during wait_for_all_moves_to_complete: {result}")
            
            if errors_found:
                raise MultiAxisError("One or more axes failed to complete their move.", individual_errors=errors_found)

        logger.info("All active moves are complete.")

    # Consider adding methods for kinematic chains / robot models if needed (advanced)

    async def move_linearly_to(
        self,
        target_positions: Dict[str, float],
        tool_speed_user: float,
        wait_for_all: bool = True,
    ) -> None:
        """
        Moves multiple axes to a target position in a coordinated straight line.

        This method implements linear interpolation by calculating the required
        speed for each axis to ensure the tool moves at a constant velocity
        along the path from the current point to the target point.

        Args:
            target_positions: A dictionary mapping axis names to their target
                              absolute positions in user units.
            tool_speed_user: The desired speed of the end-effector (tool)
                             along the path, in user units per second.
            wait_for_all: If True (default), waits for all axes to complete
                          their moves before returning.

        Raises:
            MultiAxisError: If fetching current positions or executing the
                            move fails for one or more axes.
            ValueError: If tool_speed_user is not a positive number.
        """
        if tool_speed_user <= 0:
            raise ValueError("Tool speed must be a positive number.")

        logger.info(
            f"Calculating linear move to {target_positions} "
            f"with tool speed {tool_speed_user:.2f} units/s."
        )

        # 1. Get the current positions of the axes involved in the move.
        # This ensures we calculate the path from the true current state.
        axes_to_move = list(target_positions.keys())
        current_positions = await self.get_all_positions_user()
        
        deltas = {}
        for axis_name in axes_to_move:
            current_pos = current_positions.get(axis_name)
            if current_pos is None:
                 raise MultiAxisError(f"Could not get current position for axis '{axis_name}'.")
            deltas[axis_name] = target_positions[axis_name] - current_pos

        # 2. Calculate the total Euclidean distance of the move.
        # This is the length of the straight-line path in N-dimensional space.
        distance = math.sqrt(sum(delta**2 for delta in deltas.values()))

        if distance < 1e-6: # A negligible distance
            logger.info("Target position is same as current. No move needed.")
            return

        # 3. Calculate the total duration the move should take.
        duration_seconds = distance / tool_speed_user

        # 4. Calculate the required speed for each individual axis.
        # speed = distance / time
        speeds_user = {
            axis_name: abs(delta / duration_seconds)
            for axis_name, delta in deltas.items()
        }
        
        logger.info(f"Calculated move duration: {duration_seconds:.2f}s. "
                    f"Calculated axis speeds: {speeds_user}")

        # 5. Use the existing multi-axis move method with the calculated speeds.
        await self.move_all_to_positions_abs_user(
            positions_user=target_positions,
            speeds_user=speeds_user,
            wait_for_all=wait_for_all,
        )
