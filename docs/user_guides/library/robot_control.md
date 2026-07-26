# Controlling Robot Models

`MultiAxisController` moves joints. `robot_kinematics.py` adds a layer above it
that moves an *end-effector*: you give it a Cartesian pose, it runs inverse
kinematics to work out the joint targets, and hands those to the controller.

This guide is about wiring one up and what to expect from it. For the full
signatures see the
[`robot_kinematics.py` API reference](../../api_reference/library/robot_kinematics.md).

## The three layers

```text
CartesianRobot / TwoLinkArmPlanar / RRRArm    "put the tool at (x, y, z)"
        |
MultiAxisController                            "move axis 'x' to 40 mm"
        |
Axis  x3                                       "move motor 1 to 65536 steps"
        |
CANInterface                                   frames on the bus
```

Each layer is usable on its own. Dropping to `MultiAxisController` for a
homing sequence and coming back up to the robot model for the actual work is
normal and nothing carries hidden state that makes it awkward.

## Building one

Every model takes a `MultiAxisController` and the names of the axes that form
its joints, **in kinematic order**. The order is the whole contract — the model
has no other way to know which motor is the elbow.

```python
import asyncio

from mks_servo_can import Axis, CANInterface, LinearKinematics, MultiAxisController
from mks_servo_can.robot_kinematics import CartesianRobot


async def main():
    can_if = CANInterface(use_simulator=True, simulator_port=6789)
    await can_if.connect()

    controller = MultiAxisController(can_interface_manager=can_if)
    for name, can_id in (("x", 1), ("y", 2), ("z", 3)):
        controller.add_axis(
            Axis(
                can_if,
                motor_can_id=can_id,
                name=name,
                kinematics=LinearKinematics(steps_per_revolution=16384, pitch=40.0),
            )
        )

    await controller.initialize_all_axes()
    await controller.enable_all_axes()

    robot = CartesianRobot(
        multi_axis_controller=controller,
        x_axis_name="x",
        y_axis_name="y",
        z_axis_name="z",
    )

    await robot.move_to_cartesian_pose({"x": 100.0, "y": 50.0, "z": 20.0})
    print(await robot.get_current_pose())

    await can_if.disconnect()


asyncio.run(main())
```

If a name you pass is not in the controller, or you pass the wrong number of
names for the model, the constructor raises `ConfigurationError` immediately.
That check is worth having: the failure mode it prevents is a robot that runs
and puts the tool somewhere else.

## The models

### `CartesianRobot` — 3 DOF, XYZ

Orthogonal axes, each motor driving one Cartesian dimension. Kinematics are a
translation by `origin_offset` and nothing more, so it never fails to find a
solution — but it also does no reachability checking. Use `LinearKinematics` on
the underlying axes so that joint units are already millimetres.

Takes the three axis names as separate arguments (`x_axis_name`, `y_axis_name`,
`z_axis_name`), not a list.

### `TwoLinkArmPlanar` — 2 DOF, XY

An RR arm in a plane: base joint then elbow, `link1_length` and `link2_length`
in the same units as your target coordinates.

```python
from mks_servo_can.robot_kinematics import TwoLinkArmPlanar

arm = TwoLinkArmPlanar(
    multi_axis_controller=controller,
    axis_names=["base_joint", "elbow_joint"],
    link1_length=100.0,
    link2_length=80.0,
)
```

Inverse kinematics picks the **elbow-up** solution. The elbow-down mirror image
is equally valid geometrically and the class will not give it to you, so if your
machine can only reach a point the other way round, that point is unreachable as
far as this model is concerned.

Joint angles are in degrees, matching what `RotaryKinematics` expects.

### `RRRArm` — 3 DOF, XYZ

Base rotating about Z, then shoulder and elbow rotating about parallel
horizontal axes. `axis_names` is `[base, shoulder, elbow]`; `link1_length` is
shoulder-to-elbow and `link2_length` is elbow-to-end-effector.

The IK is written for that specific geometry. If your arm differs — an offset
wrist, a non-zero shoulder offset, a different joint-2 axis — the numbers will
come out plausible and wrong. Check the forward kinematics against a few known
joint positions before trusting it.

## What you get

All models share these, from `RobotModelBase`:

| method | what it does |
|---|---|
| `await robot.forward_kinematics(joint_states)` | joints → pose. Accepts an ordered list or a `{axis_name: value}` dict. |
| `await robot.inverse_kinematics(target_pose)` | pose → `{axis_name: value}`. Does not move anything. |
| `await robot.get_current_joint_states()` | reads the motors, returns `{axis_name: value}` |
| `await robot.get_current_pose()` | reads the motors, then forward kinematics |
| `await robot.move_to_cartesian_pose(pose, speeds_user=None, wait_for_all=True)` | inverse kinematics, then commands every axis |

A pose is a plain dictionary — `{"x": 100.0, "y": 50.0}` for planar models,
with `"z"` added for the 3-DOF ones. There is no pose class to import.

## Speed, and what "coordinated" does not mean

`speeds_user` maps axis names to speeds in that axis's user units per second.
Leave it out and each axis uses its own default.

**This is not Cartesian path control.** The model computes the joint targets for
the endpoint and starts every axis moving toward its own target at its own
speed. The joints therefore arrive at different times and the tool traces
whatever curve falls out of that — a straight line only in the Cartesian case,
and only when the axes happen to finish together.

If you need the tool to follow a path, interpolate it yourself: break it into
closely spaced waypoints and call `move_to_cartesian_pose` for each with
`wait_for_all=True`. `examples/calligraphy_plotter_manual_interpolation.py`
does exactly this and is the honest demonstration of the cost.

## Errors

* `ConfigurationError` — at construction: wrong number of axis names, a name the
  controller does not have, a non-positive link length.
* `KinematicsError` — the target is out of reach, the pose dictionary is missing
  a key the model needs, or the joint states you passed do not match the DOF.
* `MultiAxisError` — the geometry was fine but the motors were not. Its
  `individual_errors` attribute maps axis name to the exception that axis
  raised, so a partial failure tells you *which* joint failed rather than just
  that something did. Read it; a three-axis move where one axis never moved is a
  different problem from one where the bus dropped.

`move_to_cartesian_pose` lets `KinematicsError` and `MultiAxisError` through
unchanged and wraps anything else in `MKSServoError`.

## Working examples

These run against the simulator and are exercised, unlike prose:

* `examples/cartesian_3dof_robot.py` — `CartesianRobot`
* `examples/two_link_planar_arm.py` — `TwoLinkArmPlanar`
* `examples/three_link_arm.py` — `RRRArm`
* `examples/calligraphy_plotter_manual_interpolation.py` — path interpolation

Start a simulator with as many motors as the example needs, then run it:

```bash
mks-servo-simulator --num-motors 3 &
python examples/cartesian_3dof_robot.py
```

## See also

* [Working with Multiple Axes](multi_axis.md)
* [Using Kinematics](kinematics.md) — per-axis units, the layer below this one
* [Implementing Custom Kinematics](../../tutorials/custom_kinematics.md)
* [`robot_kinematics.py` API reference](../../api_reference/library/robot_kinematics.md)
