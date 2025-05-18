# Working with Multiple Axes (`MultiAxisController`)

## Prerequisites

- A successfully connected `CANInterface` instance. See [Connecting to Motors (Hardware & Simulator)](./connecting.md).
- Understanding of how to initialize and use individual `Axis` objects. See [Basic Motor Control](./basic_control.md) and subsequent guides on movement and kinematics.
- Knowledge of the CAN IDs for all motors you intend to control.

## Importing `MultiAxisController`

First, ensure you import the `MultiAxisController` class along with other necessary components:

```python
import asyncio
from mks_servo_can import CANInterface, Axis, MultiAxisController, exceptions, const
from mks_servo_can.kinematics import RotaryKinematics # Or other kinematics as needed
```

## Initializing MultiAxisController

```python
import asyncio
from mks_servo_can import CANInterface, Axis, MultiAxisController, exceptions, const
from mks_servo_can.kinematics import RotaryKinematics # Or other kinematics as needed

async def initialize_multi_axis_controller_example():
    # Assume can_if is a connected CANInterface instance
    can_if = CANInterface(use_simulator=True) # Example: using simulator
    await can_if.connect()
    print("CAN Interface connected.")

    multi_controller = MultiAxisController(can_interface=can_if)
    print("MultiAxisController initialized.")

    # Important: The controller is initialized, but it doesn't have any axes yet.
    # You need to add Axis objects to it.

    return multi_controller, can_if # Return can_if for later disconnection

# --- Example Usage (within an async function) ---
# async def main():
#     m_controller, c_interface = await initialize_multi_axis_controller_example()
#     # ... add axes and use m_controller ...
#     if c_interface.is_connected:
#         await c_interface.disconnect()
#         print("CAN Interface disconnected.")

# --- Example Usage (assuming 'm_controller' is an initialized MultiAxisController) ---
# await add_axes_to_controller_example(m_controller)
```

## Adding Axes to the Controller

```python
async def add_axes_to_controller_example(controller: MultiAxisController):
    # Define configurations for your axes
    axis_configs = [
        {"can_id": 1, "name": "Axis_X", "kinematics": RotaryKinematics()},
        {"can_id": 2, "name": "Axis_Y", "kinematics": RotaryKinematics(gear_ratio=2.0)},
        {"can_id": 3, "name": "Axis_Z", "kinematics": LinearKinematics(pitch=5.0)},
    ]

    for config in axis_configs:
        axis = controller.add_axis(
            can_id=config["can_id"],
            name=config["name"],
            kinematics=config["kinematics"]
        )
        print(f"Added {axis.name} (CAN ID: {axis.can_id}) to the controller.")
        # The add_axis method creates and returns the Axis instance.
        # It's also stored within the controller.

    print(f"Controller now manages {len(controller.axes)} axes: {[ax.name for ax in controller.axes.values()]}")

# --- Example Usage (assuming 'm_controller' is an initialized MultiAxisController) ---
# await add_axes_to_controller_example(m_controller)
```

## Collective Operations

The MultiAxisController provides methods to operate on all managed axes simultaneously. These operations are performed concurrently using `asyncio.gather()`. The following methods are 
available:

### Pinging All Axes

```python
async def ping_all_axes_example(controller: MultiAxisController):
    print("Pinging all axes...")
    # ping_all returns a dictionary: {can_id: response_time_ms or None if timeout/error}
    results = await controller.ping_all()
    for can_id, rtt_ms in results.items():
        axis_name = controller.get_axis(can_id).name if controller.get_axis(can_id) else f"ID_{can_id}"
        if rtt_ms is not None:
            print(f"{axis_name} responded in {rtt_ms:.2f} ms.")
        else:
            print(f"{axis_name} did not respond or an error occurred.")
    return results
```

### Enabling All Axes

```python
async def enable_all_axes_example(controller: MultiAxisController):
    print("Enabling all axes...")
    # enable_all returns a dictionary: {can_id: True if successful, False otherwise}
    results = await controller.enable_all()
    all_enabled = True
    for can_id, success in results.items():
        axis_name = controller.get_axis(can_id).name if controller.get_axis(can_id) else f"ID_{can_id}"
        print(f"{axis_name}: Enable {'succeeded' if success else 'failed'}.")
        if not success:
            all_enabled = False
    if all_enabled:
        print("All axes enabled successfully.")
    else:
        print("One or more axes failed to enable.")
    return results
```

### Disabling All Axes

```python
async def disable_all_axes_example(controller: MultiAxisController):
    print("Disabling all axes...")
    # disable_all returns a dictionary: {can_id: True if successful, False otherwise}
    results = await controller.disable_all()
    all_disabled = True
    for can_id, success in results.items():
        axis_name = controller.get_axis(can_id).name if controller.get_axis(can_id) else f"ID_{can_id}"
        print(f"{axis_name}: Disable {'succeeded' if success else 'failed'}.")
        if not success:
            all_disabled = False
    if all_disabled:
        print("All axes disabled successfully.")
    else:
        print("One or more axes failed to disable.")
    return results
```

### Synchronized Multi-Axis Interpolation

```python
async def synchronized_like_move_example(m_controller: MultiAxisController):
    # Ensure required axes are enabled
    if not m_controller.is_enabled(can_ids=[m_controller.add_axis(1, "Axis_X", LinearKinematics(pitch=1.0))]])
        print("Failed to enable required axes for synchronized move.")
        return

    # Define target positions and speeds
    target_x = 100.0 # e.g., mm
    speed_x = 20.0   # e.g., mm/s
    target_y = 50.0  # e.g., mm
    speed_y = 10.0   # e.g., mm/s

    # Start moves without waiting for completion
    future_x = m_controller.move_absolute(target_x, speed_x, wait_for_completion=False)
    future_y = m_controller.move_absolute(target_y, speed_y, wait_for_completion=False)
    print("Movement commands dispatched concurrently.")

    # Wait for both moves to complete
    print("Waiting for moves to finish...")
    await asyncio.gather(future_x, future_y)
    print("All moves completed.")

    pos_x = await m_controller.get_current_position()
    pos_y = await m_controller.get_current_position()
    print(f"Final positions: X={pos_x:.2f}, Y={pos_y:.2f}")

# --- Example Setup ---
# async def main():
#     can_if = CANInterface(use_simulator=True)
#     await can_if.connect()
#     m_controller = MultiAxisController(can_if)
#     m_controller.add_axis(1, "Axis_X", LinearKinematics(pitch=1.0)) # Example kinematics
#     m_controller.add_axis(2, "Axis_Y", LinearKinematics(pitch=1.0)) # Example kinematics
#     
#     await synchronized_like_move_example(m_controller)
#     
#     await can_if.disconnect()
# asyncio.run(main())
```
For true synchronized multi-axis interpolation, a higher-level motion controller or planner that calculates velocities and step profiles for each axis based on a desired path and overall feedrate is typically required. The `mks-servo-can` library provides the building blocks for individual axis control that such a planner would use.

## Error Handling

The following methods return dictionaries indicating the success or result for each axis:

- `enable_all()`
- `ping_all()`
- `disable_all()`

Your application should check these results to handle cases where some axes might fail an operation while others succeed. Individual Axis methods can still raise specific exceptions like `MotorTimeoutError` or `MotorOperationError`, which might indicate issues with the axis itself.
