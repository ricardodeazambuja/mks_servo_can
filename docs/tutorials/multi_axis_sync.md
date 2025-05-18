# Tutorial: Synchronizing Multiple Axes

This tutorial explains how to command multiple MKS servo motors to start and complete movements in a coordinated fashion using the `MultiAxisController` from the `mks-servo-can` library.

**Important Note on Synchronization:**
True synchronized multi-axis *interpolation* (where multiple motors follow a precise, time-coordinated path to ensure, for example, a perfectly straight line in a 2D or 3D space) is a complex task typically handled by a higher-level motion controller or motion planning software. The MKS servo motors themselves execute individual commands (like "move to position X at speed Y") independently once they receive them.

The `mks-servo-can` library, through the `MultiAxisController`, provides tools to:
1.  Dispatch movement commands to multiple axes concurrently (i.e., send the "start move" commands to all involved motors at nearly the same time).
2.  Wait for all these concurrently initiated moves to report completion.

This allows for a level of coordination where multiple axes start their movements together and the application can proceed once all have finished. However, it does not guarantee that they will all follow a perfectly synchronized path relative to each other throughout their entire travel, especially if the distances, speeds, or loads differ significantly.

## Prerequisites

* You have set up your `CANInterface` and it's connected (either to real hardware or the simulator). See [Connecting to Motors](../user_guides/library/connecting.md).
* You understand how to use the `Axis` class for basic motor control. See [Basic Motor Control](../user_guides/library/basic_control.md).
* You are familiar with the `MultiAxisController`. See [Working with Multiple Axes](../user_guides/library/multi_axis.md).
* The MKS servo motors you intend to control are powered on and have unique CAN IDs.

## Steps for Coordinated Movement

### 1. Initialize `CANInterface` and `MultiAxisController`

First, set up your communication interface and the controller that will manage your axes.

```python
import asyncio
import logging
from mks_servo_can import (
    CANInterface,
    Axis,
    MultiAxisController,
    const,
    exceptions,
    LinearKinematics, # Example, choose kinematics appropriate for your setup
    RotaryKinematics
)

# Configure logging (optional, but helpful)
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

async def main():
    # --- Step 1: Setup CANInterface ---
    # Replace with your actual connection parameters if not using simulator
    can_if = CANInterface(
        use_simulator=True, # Set to False for real hardware
        simulator_host="localhost",
        simulator_port=6789
        # For real hardware:
        # interface_type="canable",
        # channel="/dev/ttyACM0", # Your CAN adapter channel
        # bitrate=500000
    )

    try:
        await can_if.connect()
        logger.info("CAN Interface connected.")

        # --- Step 2: Setup MultiAxisController ---
        multi_controller = MultiAxisController(can_interface_manager=can_if)
        logger.info("MultiAxisController initialized.")

        # --- Step 3: Add Axes to the Controller ---
        # Define configurations for your axes. Adjust kinematics as needed.
        # Ensure motor CAN IDs match your hardware/simulator setup.
        # If using the simulator, start it with the appropriate number of motors.
        # e.g., mks-servo-simulator --num-motors 2 --start-can-id 1
        axis_configs = [
            {"can_id": 1, "name": "AxisX", "kinematics": LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0, units="mm")},
            {"can_id": 2, "name": "AxisY", "kinematics": LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0, units="mm")},
            # Add more axes if needed
        ]

        for config in axis_configs:
            multi_controller.add_axis(
                axis=Axis(
                    can_interface_manager=can_if, # Pass the CANInterface object
                    motor_can_id=config["can_id"],
                    name=config["name"],
                    kinematics=config["kinematics"]
                )
            )
        logger.info(f"Added {len(multi_controller.axes)} axes to the controller.")

        # --- Step 4: Initialize and Enable All Axes ---
        logger.info("Initializing all axes...")
        await multi_controller.initialize_all_axes(calibrate=False, home=False, concurrent=True)
        logger.info("Enabling all axes...")
        await multi_controller.enable_all_axes(concurrent=True)

        # Check if all axes are enabled
        all_enabled_check = True
        for axis_name, axis_obj in multi_controller.axes.items():
            if not axis_obj.is_enabled(): # is_enabled() is synchronous, relies on cached state
                # For a fresh check: await axis_obj.read_en_status()
                logger.warning(f"Axis {axis_name} did not confirm enabled status.")
                all_enabled_check = False
        if not all_enabled_check:
            logger.error("One or more axes failed to enable. Aborting synchronized move.")
            return

        # --- Step 5: Define Target Positions and Speeds ---
        # Positions are in user units defined by their kinematics (e.g., mm)
        target_positions = {
            "AxisX": 100.0,  # Target 100 mm for AxisX
            "AxisY": 50.0   # Target 50 mm for AxisY
        }
        # Speeds are also in user units per second (e.g., mm/s)
        # If speeds are not provided, the axis's default speed will be used.
        target_speeds = {
            "AxisX": 20.0, # 20 mm/s for AxisX
            "AxisY": 10.0  # 10 mm/s for AxisY
        }

        logger.info(f"Target positions: {target_positions}")
        logger.info(f"Target speeds: {target_speeds}")

        # Get initial positions for reference
        initial_positions = await multi_controller.get_all_positions_user()
        logger.info(f"Initial positions: {initial_positions}")

        # --- Step 6: Execute Coordinated Absolute Move ---
        # The `move_all_to_positions_abs_user` method sends commands to all specified axes
        # to start their moves concurrently.
        # `wait_for_all=True` means this call will block until all motors report completion.
        logger.info("Starting coordinated absolute move...")
        await multi_controller.move_all_to_positions_abs_user(
            positions_user=target_positions,
            speeds_user=target_speeds,
            wait_for_all=True
        )
        logger.info("Coordinated absolute move reported complete by all axes.")

        # Verify final positions
        final_positions = await multi_controller.get_all_positions_user()
        logger.info(f"Final positions after absolute move: {final_positions}")

        # --- Example: Coordinated Relative Move ---
        # Now, let's perform a relative move.
        relative_distances = {
            "AxisX": -20.0, # Move AxisX back by 20 mm
            "AxisY": 10.0   # Move AxisY forward by 10 mm
        }
        # Speeds can be different for relative moves too
        relative_speeds = {
            "AxisX": 15.0,
            "AxisY": 5.0
        }
        logger.info(f"Starting coordinated relative move: {relative_distances}")
        await multi_controller.move_all_relative_user(
            distances_user=relative_distances,
            speeds_user=relative_speeds,
            wait_for_all=True
        )
        logger.info("Coordinated relative move reported complete.")

        final_positions_after_relative = await multi_controller.get_all_positions_user()
        logger.info(f"Final positions after relative move: {final_positions_after_relative}")


        # --- Step 7: Non-Waiting (Asynchronous) Coordinated Move ---
        # If you need to perform other operations while motors are moving.
        logger.info("Starting non-waiting coordinated absolute move back to near zero...")
        target_positions_async = {"AxisX": 5.0, "AxisY": 2.0}
        target_speeds_async = {"AxisX": 25.0, "AxisY": 15.0}

        # Call with wait_for_all=False. This dispatches the commands and returns.
        # The actual movement completion needs to be awaited later if required.
        await multi_controller.move_all_to_positions_abs_user(
            positions_user=target_positions_async,
            speeds_user=target_speeds_async,
            wait_for_all=False # Key change here
        )
        logger.info("Non-waiting move commands dispatched. Motors should be moving.")

        # Do other work while motors are moving
        logger.info("Simulating other work for 2 seconds...")
        await asyncio.sleep(2)
        logger.info("Other work finished.")

        # Now, wait for all previously started moves to complete
        if not multi_controller.are_all_moves_complete():
            logger.info("Waiting for non-waiting moves to finish...")
            await multi_controller.wait_for_all_moves_to_complete(timeout_per_axis=30.0) # Optional timeout
            logger.info("Non-waiting moves are now complete.")
        else:
            logger.info("Non-waiting moves seem to have completed already.")

        final_positions_async = await multi_controller.get_all_positions_user()
        logger.info(f"Final positions after non-waiting move: {final_positions_async}")


    except exceptions.MKSServoCANError as e:
        logger.error(f"A mks-servo-can library error occurred: {e}")
        if isinstance(e, exceptions.MultiAxisError) and e.individual_errors:
            for axis_name, err_detail in e.individual_errors.items():
                logger.error(f"  Error for axis '{axis_name}': {err_detail}")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        if multi_controller is not None and multi_controller.axes: # Check if controller and axes exist
            logger.info("Disabling all axes...")
            try:
                await multi_controller.disable_all_axes(concurrent=True)
            except Exception as e_disable:
                logger.error(f"Error disabling axes: {e_disable}")

        if can_if and can_if.is_connected:
            logger.info("Disconnecting CAN Interface.")
            await can_if.disconnect()
        logger.info("Program finished.")

if __name__ == "__main__":
    # Ensure the simulator is running with at least 2 motors if use_simulator=True
    # e.g., mks-servo-simulator --num-motors 2 --start-can-id 1
    asyncio.run(main())
```

## Explanation

### 1. Setup
* We initialize the `CANInterface` and `MultiAxisController`.
* We add multiple `Axis` objects to the controller, each with its own name, CAN ID, and kinematics configuration.
* All axes are initialized and enabled using the controller's group methods (`initialize_all_axes`, `enable_all_axes`).

### 2. Defining Coordinated Moves
* A dictionary `target_positions` maps axis names (strings) to their desired absolute target positions in user units (e.g., millimeters or degrees, depending on their individual kinematics).
* Similarly, `target_speeds` maps axis names to their desired speeds in user units per second.

### 3. Executing Moves with `move_all_to_positions_abs_user()`
* The `multi_controller.move_all_to_positions_abs_user()` method is called.
* Internally, this method iterates through the provided dictionaries and calls the `move_to_position_abs_user(..., wait=False)` method on each respective `Axis` object. This dispatches the move commands to all specified motors nearly simultaneously.
* If `wait_for_all=True` (the default), the `move_all_to_positions_abs_user` method then waits for all the internal `asyncio.Future` objects (one for each move, stored in `axis._active_move_future`) to complete.
* This ensures that your program flow pauses until all motors in the group have signaled completion of their moves.

### 4. Executing Relative Moves with `move_all_relative_user()`
* This works similarly to absolute moves, but you provide `distances_user` (a dictionary mapping axis names to relative distances).
* The controller commands each axis to move by its specified relative amount.

### 5. Non-Waiting Moves (`wait_for_all=False`)
* If you pass `wait_for_all=False` to `move_all_to_positions_abs_user` or `move_all_relative_user`, the methods will dispatch the move commands to all axes and return immediately. This allows your program to perform other tasks while the motors are in motion.
* You can later check if all moves are complete using `multi_controller.are_all_moves_complete()`.
* To explicitly wait for these non-blocking moves to finish at a later point in your code, use `await multi_controller.wait_for_all_moves_to_complete()`.

## Important Considerations for Synchronization

* **Start Time:** The commands to start the moves are sent out nearly concurrently by `asyncio.gather()`. The actual start of physical motion on each motor depends on CAN bus latency, motor processing time, and any configured delays.

* **End Time & Path:** Since each motor executes its move independently based on its commanded distance and speed (and internal acceleration profiles), axes moving shorter distances or at higher speeds will finish earlier. The `wait_for_all=True` ensures the *slowest* or longest move in the group has completed before proceeding.

* **No Path Interpolation:** This method does *not* perform path interpolation. If AxisX needs to move 100mm and AxisY needs to move 50mm, and they are to trace a straight line, their speeds must be carefully profiled such that $SpeedX / SpeedY = DistanceX / DistanceY$. The `move_all_..._user` methods allow specifying individual speeds, which can help achieve this *if calculated correctly by the user's application*.

* **External Motion Planner:** For true, complex synchronized multi-axis motion (e.g., circular interpolation, complex curves, maintaining constant feed rates along a path), a dedicated motion planning system is usually required. Such a system would break down the desired path into small, frequent position/velocity commands for each axis, which could then be sent using this library.

* **Error Handling:** The `move_all_...` methods will raise a `MultiAxisError` if one or more axes encounter an error during the move initiation or execution (if `wait_for_all=True`). The `MultiAxisError` object can contain a dictionary of individual errors per axis.

This tutorial provides the tools for basic "start-together, finish-when-all-done" coordination. For more advanced synchronization, your application will need to implement higher-level path planning and velocity profiling logic.