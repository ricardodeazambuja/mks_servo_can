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
exported — **measured, not assumed**, see `check_printability.py`. Each part is
oriented so the clamp screw tightens *across* layer lines rather than trying to
peel them apart; a hub printed the other way up splits the first time you torque
it.

| Part | Orientation | Mass | Notes |
| --- | --- | --- | --- |
| A pan yoke | beam's underside on the bed, plate pointing up | 32 g | column tapers into the plate at ~10° from vertical, so the wings are never an overhang |
| B camera cradle | standing on the hub's end face, platform vertical | 20 g | small footprint for a 54 mm part — use a brim |
| C base plate | plate flat, legs up | 61 g | countersinks face the bed |

`python check_printability.py` reads the exported STLs and measures this rather
than taking it on trust. It reports down-facing patches by the *span* each has
to bridge, not by area, because area is misleading here: every bolt hole through
a vertical plate has a down-facing ceiling, so the yoke accumulates 129 mm² of
them and still needs no support. What matters is that each one only crosses the
plate it passes through.

| Part | Bed footprint | Down-facing area | Widest bridge |
| --- | --- | --- | --- |
| pan yoke | 1139 mm² | 129 mm² over 7 patches | **4.0 mm** |
| camera cradle | 668 mm² | 94 mm² over 5 patches | **4.7 mm** |
| base plate | 4487 mm² | none | — |

Every patch is a hole ceiling spanning a 4–5 mm plate, which any FDM printer
bridges. The shallowest down-facing wall on either part is 48° from horizontal,
comfortably inside the 45° rule.

Both clamp hubs print with their **bore vertical**. That is deliberate: the
layers then run around the bore, and the clamp screw pulls the slit closed
within a layer instead of trying to peel layers apart. A hub printed on its
side splits the first time you torque it.

Hardware needed: 8 × M3×8 (motor faces), 2 × M3×20 + 2 × M3 nuts (shaft
clamps), 1 × ¼"-20 (camera), 4 × M3×10 (base plate to motor).

## Building it

```
openscad -D 'part="A"' --export-format binstl -o stl/pan_yoke.stl      gimbal_parts.scad
openscad -D 'part="B"' --export-format binstl -o stl/camera_cradle.stl gimbal_parts.scad
openscad -D 'part="C"' --export-format binstl -o stl/base_plate.stl    gimbal_parts.scad
```

Pre-built STLs are in `stl/`. To preview the whole thing:

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
| yoke column | 3.5 mm | −45° |
| tilt motor + driver | 7.5 mm | −45° |
| yoke back rib | 15.6 mm | +90° |
| base plate | 23.8 mm | −39° |

The yoke-column figure is `cradle_gap`, a fixed assembly gap along the shaft
rather than a swept clearance — you set it when you slide the cradle on, and it
does not vary with angle.

## The one number to check against your motors

Everything along the tilt shaft is budgeted against an assumed **20 mm shaft
length** from the front face, which is the conservative end of what 32 mm-body
NEMA17s ship with:

```
plate 4.0  +  gap 3.5  +  cradle hub 12.0  =  19.5 mm of 20.0
```

**Measure your shaft before printing.** If it is longer — 22 or 24 mm is common —
you have spare, and raising `cradle_gap` is the best use of it. If it is shorter,
reduce `cradle_hub_h`, and re-run `check_clearances.py`, which reads all three
numbers out of the `.scad`.

Also confirm the D-flat: the design assumes a 5 mm shaft cut to 4.5 mm across
the flat (`shaft_d` and `shaft_flat`). A different flat depth just needs
`shaft_flat` changed; the bore follows it.

## Viewer

`gimbal_viewer.html` is a self-contained page with pan/tilt sliders, pose
presets and an orbit view. It is generated, not hand-maintained, so its geometry
cannot drift from the printed parts:

```
python make_visualizer.py
```

It reads every dimension out of `gimbal_parts.scad` and runs the real clearance
sweep to fill in the figure it quotes.

Renders of each part, and of the assembly, are in `render/`.

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
