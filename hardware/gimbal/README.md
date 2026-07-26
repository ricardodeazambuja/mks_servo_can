# Two-axis camera gimbal

A pan/tilt camera gimbal built from two NEMA17 32 mm motors, each carrying an
MKS SERVO42D_CAN driver on its back face. Two printed parts are required; a
third is optional but you will probably want it.

| Part | Module | Joins |
| --- | --- | --- |
| **A** | `pan_yoke()` | pan motor's D-shaft → tilt motor's face |
| **B** | `camera_cradle()` | tilt motor's D-shaft → camera |
| C (optional) | `base_plate()` | pan motor → tripod or bench |

Part C exists because a motor with a driver on its back **cannot stand on a flat
surface**. The plate bolts to the pan motor's front face and hangs the motor
underneath, so the driver and its connectors sit in free air. If you already
have a bracket that does this, skip it.

## What the driver dictates

The SERVO42D covers the whole back face and puts hardware on all four edges
(manual page 2):

| Edge | What is there |
| --- | --- |
| top | 4-way screw terminal, motor phases, wires exit upward |
| left | 5-way terminal — EVCC, EGND, IN_1, CAN_H, CAN_L |
| right | 6-way terminal — V+, GND, COM, EN, STP, DIR |
| **bottom** | **OLED display and the Next / Enter / Menu buttons** |

The bottom edge is the binding constraint. On firmware below V1.0.6 the work
mode, `Ma` (working current) and `HoldMa` cannot be read back over CAN at all —
`Axis.initialize()` will report `work mode unreadable` — so the OLED and its
three buttons are the *only* way to see or change them. A bracket that covers
them makes the motor unconfigurable.

So neither part touches the driver end. Both locate on the motor's front face
(the 31 mm bolt square and the 22 mm boss) and on the D-shaft, leaving the PCB,
all three terminal blocks and the entire user interface exposed.

## Printing

PLA or PETG, 0.2 mm layers, 4 perimeters, ≥40% infill in the clamp hubs. No
supports needed in the orientations below, which are also how the STLs are
exported. Each is oriented so the clamp screw tightens *across* layer lines
rather than trying to peel them apart — a hub printed the other way up splits
the first time you torque it.

| Part | Orientation | Notes |
| --- | --- | --- |
| A pan yoke | hub's bottom face on the bed | the beam's overhang is self-bridging |
| B camera cradle | platform underside on the bed | ribs print upward |
| C base plate | plate flat, legs up | countersinks face the bed |

Hardware needed: 8 × M3×8 (motor faces), 2 × M3×20 + 2 × M3 nuts (shaft
clamps), 1 × ¼"-20 (camera), 4 × M3×10 (base plate to motor).

## Building it

```
openscad -D 'part="A"' --export-format binstl -o build/pan_yoke.stl      gimbal_parts.scad
openscad -D 'part="B"' --export-format binstl -o build/camera_cradle.stl gimbal_parts.scad
openscad -D 'part="C"' --export-format binstl -o build/base_plate.stl    gimbal_parts.scad
```

Pre-built STLs are in `build/`. To preview the whole thing:

```
openscad -D 'part="assembly"' -D pan=25 -D tilt=-20 gimbal_parts.scad
```

## Verifying a change

`gimbal_parts.scad` is parametric, and a plausible-looking edit can quietly move
the camera into the yoke. OpenSCAD will not tell you — it renders
interpenetrating solids without complaint, and a collision only appears at
particular angles.

```
python check_clearances.py          # exits non-zero on a clash
python check_clearances.py --verbose
```

It sweeps the full soft-limit range, reads its dimensions straight out of the
`.scad`, and includes a representative camera envelope. That last part matters:
the parts cleared each other comfortably at `tilt_axis_h = 62`, but the *camera*
came within 2.8 mm of the yoke beam at −39° of tilt. The current 70 mm leaves
10.8 mm. The payload is the largest thing that moves, so it belongs in the check.

Current worst-case clearances:

| Against | Clearance | At tilt |
| --- | --- | --- |
| yoke beam | 10.8 mm | −39° |
| yoke column | 4.0 mm | −45° |
| tilt motor + driver | 10.0 mm | −45° |
| base plate | 23.8 mm | −39° |

The yoke-column figure is a fixed assembly gap along the shaft, not a swept
clearance — set it when you slide the cradle onto the tilt shaft, and it does
not vary with angle.

## Viewer

`gimbal_viewer.html` is a self-contained page with pan/tilt sliders, pose
presets and an orbit view. It is generated, not hand-maintained, so its geometry
cannot drift from the printed parts:

```
python make_visualizer.py
```

It reads every dimension out of `gimbal_parts.scad` and runs the real clearance
sweep to fill in the figure it quotes.

## Driving it

The gimbal example runs this two-axis build directly:

```
# against the simulator
mks-servo-simulator --num-motors 2 --start-can-id 2 --latency-ms 0
python examples/camera_gimbal_tracker.py --two-axis --can-ids "pan=2,tilt=3"

# against the hardware
python examples/camera_gimbal_tracker.py --hardware --channel can0 \
    --two-axis --can-ids "pan=2,tilt=3"
```

`--two-axis` drops the roll stage; roll changes how the frame is oriented about
the optical axis, not where the camera points, so a two-motor build is a
perfectly good tracking gimbal.

Soft limits assumed by both the example and the clearance check: pan ±170°,
tilt −45°…+90°. Pan is limited rather than continuous because the camera cabling
has to come back down through the yoke.

## Balance, and why it matters more here than usual

`plat_drop` is deliberately small (12 mm) so the camera body straddles the tilt
axis instead of hanging off it. A stepper holding a static gravity torque burns
current continuously — and in the `OPEN` and `CLOSE` work modes these drivers
hold **full `Ma` regardless of load**, so an unbalanced axis is not merely wasted
torque, it is a motor that runs hot doing nothing. Slide the camera along the
three ¼"-20 positions until the tilt axis balances, and re-check after any lens
change.
