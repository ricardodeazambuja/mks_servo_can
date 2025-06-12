# Working with Multiple Axes (`MultiAxisController`)

The `MultiAxisController` class is a powerful tool for managing and coordinating multiple `Axis` objects simultaneously. It simplifies tasks that involve group operations, such as enabling all motors, initializing them together, or commanding concurrent movements.

## Prerequisites

- A successfully connected `CANInterface` instance. See [Connecting to Motors (Hardware & Simulator)](./connecting.md).
- Understanding of how to initialize and use individual `Axis` objects. See [Basic Motor Control](./basic_control.md) and subsequent guides.
- Knowledge of the CAN IDs for all motors you intend to control.

## Importing `MultiAxisController`

First, ensure you import the `MultiAxisController` class along with other necessary components:

```python
import asyncio
from mks_servo_can import CANInterface, Axis, MultiAxisController, exceptions, const
from mks_servo_can.kinematics import RotaryKinematics, LinearKinematics
```

## Initializing MultiAxisController

The controller is initialized with a `CANInterface` instance that will be shared by all axes it manages.

```python
import asyncio
from mks_servo_can import CANInterface, MultiAxisController

async def initialize_multi_axis_controller_example():
    # Assume can_if is a connected CANInterface instance
    can_if = CANInterface(use_simulator=True) # Example: using simulator
    await can_if.connect()
    print("CAN Interface connected.")

    # Correct initialization using the 'can_interface_manager' parameter
    multi_controller = MultiAxisController(can_interface_manager=can_if)
    print("MultiAxisController initialized.")

    # The controller is now ready to have Axis objects added to it.
    return multi_controller, can_if
```

## Adding Axes to the Controller

A key concept of the `MultiAxisController` is that it manages pre-initialized `Axis` objects. You must create each `Axis` object first and then add it to the controller.

```python
# --- Continuing from the previous example ---
# async def add_axes_to_controller_example(controller: MultiAxisController, can_if: CANInterface):
    # Define configurations for your axes
    axis_configs = [
        {"can_id": 1, "name": "Axis_X", "kinematics": LinearKinematics(pitch=10.0, steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)},
        {"can_id": 2, "name": "Axis_Y", "kinematics": LinearKinematics(pitch=10.0, steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)},
    ]

    for config in axis_configs:
        # 1. Create the Axis instance
        axis_to_add = Axis(
            can_interface_manager=can_if,
            motor_can_id=config["can_id"],
            name=config["name"],
            kinematics=config["kinematics"]
        )
        # 2. Add the created instance to the controller
        controller.add_axis(axis_to_add)
        print(f"Added {axis_to_add.name} (CAN ID: {axis_to_add.can_id}) to the controller.")

    print(f"Controller now manages {len(controller.axes)} axes: {controller.axis_names}")
```

## Collective Operations

The controller provides methods to operate on all managed axes concurrently using `asyncio.gather()`.

### Initializing and Enabling All Axes

```python
# async def initialize_and_enable_all_example(controller: MultiAxisController):
    print("Initializing all axes concurrently...")
    try:
        await controller.initialize_all_axes(calibrate=False, home=False, concurrent=True)
        print("All axes initialized successfully.")
        
        print("Enabling all axes concurrently...")
        await controller.enable_all_axes(concurrent=True)
        print("All axes enabled successfully.")
        
    except exceptions.MultiAxisError as e:
        print(f"A multi-axis error occurred: {e}")
        # The exception contains details about which specific axes failed
        if e.individual_errors:
            for axis_name, error in e.individual_errors.items():
                print(f"  - Error for {axis_name}: {error}")
```

### Disabling and Stopping All Axes

```python
# async def disable_and_stop_all_example(controller: MultiAxisController):
    # Stop any motion
    print("Stopping all axes...")
    await controller.stop_all_axes()

    # Disable all motors
    print("Disabling all axes...")
    await controller.disable_all_axes()
```

## Coordinated Movement

While the MKS motors do not support true path interpolation via a single command, the `MultiAxisController` can dispatch move commands to all axes nearly simultaneously. This is useful for "start-together" movements.

### Moving Multiple Axes

The primary method for coordinated movement is `move_all_to_positions_abs_user()`.

```python
# async def coordinated_move_example(controller: MultiAxisController):
    # Define target positions for each axis by name
    target_positions = {
        "Axis_X": 100.0, # Target 100 mm for AxisX
        "Axis_Y": 50.0   # Target 50 mm for AxisY
    }
    # Speeds are also specified by axis name
    target_speeds = {
        "Axis_X": 25.0,  # 25 mm/s
        "Axis_Y": 15.0   # 15 mm/s
    }

    print(f"Starting coordinated move to: {target_positions}")
    
    # This call dispatches commands to both motors and waits for them all to finish.
    await controller.move_all_to_positions_abs_user(
        positions_user=target_positions,
        speeds_user=target_speeds,
        wait_for_all=True  # This makes the call block until all moves are done
    )
    
    print("Coordinated move complete.")
    
    final_positions = await controller.get_all_positions_user()
    print(f"Final positions: {final_positions}")
```

### Non-Blocking Moves and Waiting

You can also initiate a move and continue with other tasks by setting `wait_for_all=False`.

```python
# async def non_blocking_move_example(controller: MultiAxisController):
    target_positions = {"Axis_X": 10.0, "Axis_Y": 5.0}

    # This call returns immediately after sending the move commands
    await controller.move_all_to_positions_abs_user(
        positions_user=target_positions,
        wait_for_all=False
    )
    print("Move commands dispatched. Motors are moving in the background.")

    # Do other work...
    print("Performing other tasks...")
    await asyncio.sleep(1.0)
    print("Other tasks finished.")

    # Now, wait for the moves to complete
    if not controller.are_all_moves_complete():
        print("Waiting for moves to finish...")
        await controller.wait_for_all_moves_to_complete(timeout_per_axis=30.0)
        print("All moves are now complete.")
```

## Error Handling

When a collective operation fails for one or more axes, a `MultiAxisError` is raised. This exception contains an `individual_errors` dictionary that maps the names of the failed axes to the specific exception that occurred for each one, allowing for granular error handling.