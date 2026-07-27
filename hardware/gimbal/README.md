# Two-axis camera gimbal

A pan/tilt camera gimbal built from two NEMA17 32 mm motors, each carrying an
MKS SERVO42D_CAN driver on its back face. Three printed parts, no supports, and
it stands on a desk.

| Part | Module | Joins | Mass |
| --- | --- | --- | --- |
| **A** | `pan_yoke()` | pan shaft → a fork carrying the tilt axis | 57 g |
| **B** | `camera_cradle()` | tilt axis → camera | 40 g |
| **C** | `pedestal()` | pan motor → desk | 114 g |

![the assembly](render/asm.png)

## How the load gets to the desk

FDM parts are roughly half as strong across layers as along them, so the design's
job is to keep the steady loads out of layer-normal tension:

| Load | Where it goes |
| --- | --- |
| camera weight | down each fork arm as **compression** — the arms print vertically |
| overturning moment | into the yoke's **70 mm pad**, riding flat on the pedestal's top plate |
| pan torque | the only thing the 5 mm shaft joint has to transmit |
| all of it | a **tapered shroud** in shell shear, onto a 96 mm footprint you can screw down |

Three consequences worth stating, because each replaced something that looked
fine and wasn't:

**The tilt axis crosses the pan axis.** The camera turns about a line through its
own body, so it stays inside the fork and balances about the axis instead of
hanging off a bracket. A cantilevered arm puts the payload's *static* weight into
the root as tension across the layers, which is the one direction the material is
weak in.

**The tilt axis is supported at both ends** — the motor's shaft on one side, a
pivot screw in the other arm on the other. A 5 mm shaft in an 11 mm printed bore
is a poor moment bearing and a fine torque coupling, so it is only asked to be
the second.

**The pan bearing is a face, not a shaft.** The yoke's pad slides on the
pedestal's top plate, which is the flattest surface an FDM printer makes: the
first layer. The pad bears on a 13 mm-wide ring at its rim, not across its whole
face, so a couple of tenths of warp cannot decide which three points it rocks on.

## What the driver dictates

The SERVO42D covers the whole back face and puts hardware on all four edges
(manual page 2):

| Edge | What is there |
| --- | --- |
| top | 4-way screw terminal, motor phases |
| left | 5-way terminal — EVCC, EGND, IN_1, CAN_H, CAN_L |
| right | 6-way terminal — V+, GND, COM, EN, STP, DIR |
| **bottom** | **OLED display and the Next / Enter / Menu buttons** |

The bottom edge is the binding constraint. On firmware below V1.0.6 the work
mode, `Ma` (working current) and `HoldMa` cannot be read back over CAN at all —
`Axis.initialize()` reports `work mode unreadable` — so the OLED and its three
buttons are the *only* way to see or change them. A bracket that covers them
makes the motor unconfigurable.

So the pan motor hangs inside a shroud that is open on all four sides, with
14 mm of air below it for fingers and wire bends. This is also why the hold-down
ears are on the corners: an ear on a flat face needs 20 mm of wall above the
opening to blend into, and that 20 mm is exactly where the OLED sits.

![the pedestal, from the desk end](render/C_desk.png)

## Printing

PLA or PETG, 0.2 mm layers, 4 perimeters, ≥40% infill. **No supports**, in the
orientations below — which are also how the parts are modelled, so what
`check_printability.py` reads off the STL is what the slicer sees.

| Part | Orientation | Height | Bed footprint | Widest bridge |
| --- | --- | --- | --- | --- |
| A pan yoke | pad on the bed, arms up | 81 mm | 2190 mm² | 10.0 mm |
| B camera cradle | platform on the bed, cheeks up | 32 mm | 3126 mm² | 4.0 mm |
| C pedestal | top plate on the bed, desk end last | 67 mm | 4772 mm² | 10.0 mm |

The shallowest down-facing wall on any part is 45° from horizontal. Nothing
drafts at exactly 45° on purpose: that is the threshold every slicer compares
against, so surfaces here draft at **40°** and land unambiguously on the right
side of it.

Other things the slicer was allowed to dictate:

- Load-bearing walls are whole multiples of 0.4 mm, so they fill with perimeters
  rather than leaving a sliver of gap-fill down the middle.
- Every bed edge is chamfered 0.6 mm. Elephant's foot squashes the first layer
  outward by a couple of tenths, and on two of these parts the first layer *is*
  the mating face.
- The apertures and the arms taper at both ends instead of stopping square —
  which turns a 42 mm span into a 10 mm one, and hands the wall's shear off into
  the corners diagonally rather than ending in two stress-raising notches.
- The cradle's bore is printed with the **D-flat upward**, so the bore's ceiling
  is a flat 4.5 mm bridge rather than a drooping arc. It is also the face that
  takes the torque.
- Shaft bores have a lead-in chamfer, so a squashed first layer cannot stop the
  part going on.

## Hardware

| Qty | Item | For |
| --- | --- | --- |
| 4 | M3×8 countersunk | pan motor to the pedestal's plate |
| 4 | M3×12 | tilt motor to the fork arm |
| 2 | M4×12 | the shaft joints, bearing on each D-flat |
| 1 | M4×14 | the cradle's pivot, threaded into the far arm |
| 1 | ¼"-20 | the camera |
| 4 | 12 mm self-adhesive rubber feet | the pedestal's ears |
| 4 | M4 wood screws | *optional* — screwing it to the bench |

The two shaft screws are **self-tapping into the plastic** (3.3 mm pilot, ~9 mm
of thread). Drill to 4.0 mm and fit an M3 heat-set insert instead if you would
rather not trust a printed thread.

### Why there is no split clamp

Both hubs are a close D-bore plus one screw bearing on the shaft's flat, and this
is the one place the design argues with its predecessor's *reasoning* rather than
its geometry.

A split clamp has to close, and both of these hubs stand on something stiff — the
yoke's on a 70 mm pad, the cradle's on a 78 mm platform. Slitting a hub that is
bonded along its whole base to a plate like that buys the slit's stress
concentration and none of its compliance. The D-flat is already a *positive*
drive: it carries the torque whether or not anything is clamped. So the screw
only has to take up the bore clearance and stop the part lifting, which a screw
straight onto the flat does without needing anything to flex.

## Assembly order

It matters, because the tilt shaft and the pivot screw come in from opposite
sides:

1. Bolt the pan motor up into the pedestal's plate (countersunk screws, from
   above). Fit the rubber feet.
2. Slide the yoke onto the pan shaft, down onto the plate, and tighten its screw
   onto the flat.
3. Bolt the tilt motor to the outside of the **+X** arm, shaft pointing inward.
4. Slide the cradle onto the tilt shaft, then run the M4×14 pivot through the
   other arm into the cradle's journal. It threads into the *arm*, so its head
   can pull up tight without clamping the axis.
5. Tighten the cradle's screw onto the tilt shaft's flat.
6. Bolt the camera on and slide it along the slot until the tilt axis balances.
7. Zip-tie the tilt motor's harness to the yoke's pad and the pedestal's corner
   slots, leaving a service loop for ±170° of pan.

Which way the D-flat happens to face is set by which of the four bolt
orientations you use and where the rotor is — so **set the tilt zero in firmware**
once it is together, rather than trying to make the flat land somewhere.

## Verifying a change

`gimbal_parts.scad` is parametric, and a plausible-looking edit can quietly move
the camera into the yoke.

```
python check_clearances.py            # exits non-zero on a clash
python check_printability.py          # exits non-zero if anything needs support
python check_clearances.py --fast     # skip the slow exact pass
```

`check_clearances.py` reports three kinds of number, and mixing them up is how
this design shipped a 4.4 mm interpenetration with a check that said +2.5 mm:

**swept** — varies with tilt. Includes the payload at both ends of its balance
slot, because the payload is the largest thing that moves.

**static** — set once when you assemble it. Each carries its own minimum, because
they are not the same requirement: a swept clearance absorbs warp, a slipped
joint and a camera bigger than its screw, while the gap between a rotating cradle
and a fixed arm absorbs warp and nothing else. "Spare shaft" is not air at all —
it is tolerance against a motor whose shaft is shorter than the 20 mm assumed
here, the one dimension on these motors that genuinely varies by supplier.

**exact** — OpenSCAD's own `intersection()` of what moves with what does not,
exported and measured. This is the authoritative one. It does not approximate
anything, and it is the only check that would have caught the fork being narrower
than the cradle:

```
openscad -D 'part="interference"' -D tilt=-45 -D 'against="yoke"' \
         --export-format binstl -o /tmp/x.stl gimbal_parts.scad
```

An empty intersection makes OpenSCAD decline to write a file at all, which is
itself the answer. Where it does write one, the test is on **volume**, not on
whether the mesh is empty: coincident faces are everywhere in an assembly — a
motor's face bolts flat against a plate — and CGAL returns those contacts as a
zero-thickness solid with a few dozen facets in it.

Current state:

| | Clearance | |
| --- | --- | --- |
| pedestal plate | 28.5 mm | swept, worst at tilt +58° |
| yoke pad | 24.1 mm | swept |
| yoke hub | 22.7 mm | swept |
| fork arm | 3.0 mm | swept, worst at tilt −45° |
| camera to cheek | 2.0 mm | static, each side |
| tilt bore vs shaft tip | 1.2 mm | static — bore 11 mm, shaft reaches 9.8 mm in |
| exact overlap | 0 mm³ | at tilt −45, −20, 0, +45, +90 |

### Why the swept margins are so large

They are not chosen. The fork's height is set by its arms' draft angle, not by
the payload: the arms have to splay 19 mm outboard to clear the cradle, and they
cannot do that faster than 40° per millimetre of rise without needing support.
That fixes the tilt axis at 60 mm, which leaves far more room underneath than the
sweep needs — so a payload larger than the 52 × 44 × 34 mm envelope assumed here
will still clear everything below it. What it will *not* clear is the cheeks: the
camera has to fit **between** them, and that gap is 56 mm.

The two couplings worth knowing before you edit anything:

- **The arms must reach full width below the cradle's lowest sweep.** They splay
  to `arm_gap / 2` by `arm_mid_z` and stand vertical above it. Fold that into one
  hull and the arms only reach full width at the motor pad's bottom edge — which
  rises *with* the tilt axis, so raising the axis never opens the fork and the
  cradle interpenetrates it at every angle. That was the bug.
- **The cradle's bore must be deeper than the shaft reaches.** Whatever the bore
  does not contain protrudes past it, into the space the camera occupies.

## Viewer

`gimbal_viewer.html` is a self-contained page with pan/tilt sliders, pose presets
and an orbit view. It is generated, not hand-maintained, so its geometry cannot
drift from the printed parts:

```
python make_visualizer.py
```

It reads every dimension out of `gimbal_parts.scad` — including the assembly
frame the `.scad` derives — and runs the real clearance sweep to fill in the
figures it quotes. Renders of each part are in `render/`.

## Driving it

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
has to come back down past the yoke.

## Balance, and why it matters more here than usual

The tilt axis sits 17 mm above the platform, inside the payload's body rather
than under it, and the ¼"-20 runs in a **20 mm slot** so the fore/aft trim is
continuous rather than three fixed holes.

A stepper holding a static gravity torque burns current continuously — and in the
`OPEN` and `CLOSE` work modes these drivers hold **full `Ma` regardless of
load**, so an unbalanced axis is not merely wasted torque, it is a motor that runs
hot doing nothing. Slide the camera until the tilt axis balances, and re-check
after any lens change. A strip of thin rubber under the camera stops it rotating
about its screw.

## The one number to check against your motors

Everything along both shafts is budgeted against an assumed **20 mm shaft length**
from the front face, which is the conservative end of what 32 mm-body NEMA17s
ship with. **Measure yours before printing.** With a shaft 2 mm shorter than
assumed, the joints still engage 7.8 mm (tilt) and 10.0 mm (pan); the checker
reports both. Also confirm the D-flat: the design assumes a 5 mm shaft cut to
4.5 mm across the flat (`shaft_d`, `shaft_flat`), and a different flat depth just
needs `shaft_flat` changed — every bore follows it.

## Building it

```
openscad -D 'part="A"' --export-format binstl -o stl/pan_yoke.stl      gimbal_parts.scad
openscad -D 'part="B"' --export-format binstl -o stl/camera_cradle.stl gimbal_parts.scad
openscad -D 'part="C"' --export-format binstl -o stl/pedestal.stl      gimbal_parts.scad
```

Pre-built STLs are in `stl/`. To preview the whole thing:

```
openscad -D 'part="assembly"' -D pan=25 -D tilt=-20 gimbal_parts.scad
```
