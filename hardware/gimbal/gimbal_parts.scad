// Two-axis (pan/tilt) camera gimbal built from two NEMA17 32mm motors, each
// carrying an MKS SERVO42D_CAN driver on its back face.
//
//   PART A  pan_yoke()        pan shaft  -> fork carrying the tilt axis
//   PART B  camera_cradle()   tilt axis  -> camera
//   PART C  pedestal()        pan motor  -> desk
//   PART D  pivot_pin()       fork arm   -> the tilt axis' far bearing
//
//
// EVERY PART IS MODELLED IN THE ORIENTATION IT PRINTS IN
// ------------------------------------------------------
// This is the file's one invariant, and it is load-bearing: `check_printability.py`
// measures overhangs straight off the exported STL, so a part modelled in some
// convenient-but-fictional pose would be measured in a pose nobody prints. Each
// module below therefore sits on the bed at z = 0, +Z is up in the printer, and
// `assembly()` does whatever flipping the real machine needs. The pedestal is
// the one that looks odd as a result: it is modelled plate-down, which is
// upside down from how it stands on your desk.
//
//
// WHAT THE DRIVER FORCES ON THE DESIGN
// ------------------------------------
// The SERVO42D occupies the whole back face of the motor (manual page 2) and
// puts hardware on all four edges:
//
//   top edge     4-way screw terminal, motor phases
//   left edge    5-way screw terminal, EVCC/EGND/IN_1/CAN_H/CAN_L
//   right edge   6-way screw terminal, V+/GND/COM/EN/STP/DIR
//   bottom edge  OLED display and the Next / Enter / Menu buttons
//
// The bottom edge is the binding constraint. Work mode, Ma (working current)
// and HoldMa cannot be read back over CAN on firmware older than V1.0.6, so the
// OLED and its three buttons are the *only* way to see or change them. A
// bracket that covers them makes the motor unconfigurable.
//
// So nothing clamps the driver end. Both moving parts locate on a motor's front
// face (the 31 mm bolt square, the 22 mm boss and the D-shaft), and the pedestal
// is a shroud with a full-height opening on all four sides: the pan motor hangs
// inside it with its PCB, all three terminal blocks and the whole user interface
// reachable from outside.
//
//
// HOW THE LOAD GETS TO THE DESK
// -----------------------------
// FDM parts are ~2x weaker across layers than along them, so the design's job is
// to keep the steady loads out of layer-normal tension:
//
//   camera weight -> down each fork arm as *compression* (arms print vertically)
//   overturning   -> the yoke's 70 mm pad, riding on three PTFE glide pads on
//                    the pedestal's top plate, so the moment is carried by a
//                    wide face rather than by a 5 mm shaft and a printed clamp
//   pan torque    -> the only thing the shaft joint has to transmit
//   everything    -> a tapered shroud loaded in shell shear, not four posts in
//                    bending, onto a 96 mm footprint that can be screwed down
//
// The tilt axis is carried at the motor end by the motor's own bearings and at
// the far end by a 6700 ball bearing, so the printed bore never has to be a
// bearing - see PART D.
//
// Render one part at a time:
//   openscad -D 'part="A"' -o stl/pan_yoke.stl      gimbal_parts.scad
//   openscad -D 'part="B"' -o stl/camera_cradle.stl gimbal_parts.scad
//   openscad -D 'part="C"' -o stl/pedestal.stl      gimbal_parts.scad
//   openscad -D 'part="D"' -o stl/pivot_pin.stl     gimbal_parts.scad
//   openscad -D 'part="assembly"' -D pan=30 -D tilt=20 gimbal_parts.scad
//   openscad -D 'part="section"'  -D tilt=-20 gimbal_parts.scad   # cutaway

part = "assembly";  // "A", "B", "C", "D", "assembly", "section"
pan  = 0;           // assembly preview only, degrees
tilt = 0;           // assembly preview only, degrees

$fn = 64;
eps = 0.01;

// ---------------------------------------------------------------------------
// Measured / datasheet dimensions
// ---------------------------------------------------------------------------

// NEMA17, 32 mm body
motor_body      = 42.3;   // square across flats
motor_len       = 32.0;
motor_bolt_span = 31.0;   // square bolt pattern, centre to centre
motor_boss_d    = 22.0;
motor_boss_h    = 2.0;
shaft_d         = 5.0;
shaft_flat      = 4.5;    // across the D: full diameter minus the cut
shaft_len       = 20.0;   // from the front face
// The flat's plane, as a distance from the shaft axis. Derived, not measured -
// getting this wrong is how a bore ends up gripping on one corner.
shaft_flat_off  = shaft_flat - shaft_d / 2;

// MKS SERVO42D_CAN on the back face. Envelope is generous on purpose: the
// screw terminals stand proud of the PCB and the wires need a bend radius.
driver_pcb      = 42.5;
driver_depth    = 16.0;   // PCB plus tallest component, behind the motor

// Fasteners
m3_free        = 3.4;     // clearance, vertical hole
m3_free_h      = 3.6;     // clearance, hole printed on its side (droops closed)
m3_cs_d        = 6.4;     // countersunk head, at the surface
m3_cs_h        = 1.9;     // and its depth
m3_pilot       = 2.5;     // self-tapping into plastic
m4_free        = 4.5;
cam_screw_d    = 6.6;     // 1/4"-20 clearance

// Fit allowances, tuned for a typical 0.4 mm nozzle
fit_shaft      = 0.15;    // added to the bore, taken up by the clamp
fit_boss       = 0.40;

// ---------------------------------------------------------------------------
// How anything gets held to a shaft
// ---------------------------------------------------------------------------
//
// Every joint onto a 5 mm D-shaft in this design is the same three things, and
// they do three different jobs:
//
//   the D-bore      takes the torque. It is a *positive* drive: the flat cannot
//                   slip past the flat, clamped or not.
//   a brass insert  takes the thread. A screw tapped straight into PLA is a
//                   one-shot thread - it strips the second or third time you set
//                   the balance and take the cradle off again, which on this
//                   machine is a routine operation, not an accident.
//   a set screw     takes up the bore clearance and stops the part sliding along
//                   the shaft. It bears on the flat, so it pushes the bore's
//                   round side onto the shaft's round side instead of trying to
//                   grip on one corner.
//
// Both hubs are sized so a *standard* set screw length ends flush: the screw has
// to cross the insert and then a clearance hole to reach the flat, and that total
// is what sets `grub_flat_x` and the hub diameter, not the other way round.
insert_d       = 4.0;     // heat-set insert, hole printed upright (Ruthex M3)
insert_d_h     = 4.2;     // the same hole printed on its side, which droops shut
insert_depth   = 5.7;
grub_clear     = 3.2;     // the screw's own clearance, beyond the insert

// ---------------------------------------------------------------------------
// 6700 ball bearing, 10 x 15 x 4 - the tilt axis' far end
// ---------------------------------------------------------------------------
//
// The far end of the tilt axis used to be a screw shank turning in a printed
// journal. That works for about as long as the plastic lasts: a 4.6 mm plain
// bearing under a 2 N radial load wears into an oval, and every micron of that
// wear is backlash in the axis a tracker is trying to point. The bearing is not
// here for load - a 6700 is good for 200 N static, against the 2 N it sees - it
// is here so the axis has a *defined* radial play of about a hundredth of a
// millimetre, and keeps it.
brg_od         = 15.0;
brg_id         = 10.0;
brg_w          = 4.0;
// Outer race presses into the cradle's cheek; inner race rides on the pin.
brg_seat_d     = brg_od - 0.1;   // light press into a printed pocket
brg_seat_depth = brg_w + 0.2;    // so the race sits fully below the face
// Behind the seat: clears the *inner* race and the shield, which turn relative
// to it, while leaving an annulus for the outer race to bottom against. An
// inner race of a 6700 is about 11.7 across and the outer race starts at 13.2,
// so 12.8 lands between them.
brg_relief_d   = 12.8;
brg_journal_d  = brg_id + 0.05;  // press onto the pin, which prints slightly fat

// Extrusion-friendly wall thicknesses. Anything load bearing is a whole number
// of 0.4 mm extrusions, so the slicer fills it with perimeters instead of
// leaving a sliver of gap-fill down the middle of the part.
w4  = 1.6;
w8  = 3.2;

// Bottom-edge chamfer. Elephant's foot squashes the first layer outward by a
// couple of tenths, which is exactly where two of these parts have to mate flat
// against something. A chamfer means the bulge has nowhere to land.
foot_ch = 0.6;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// Rounded rectangular slab, centred in X/Y, sitting on z = 0. Rounded corners
// are not decoration: a square corner is where a printed part cracks, and the
// slicer's perimeters run round a radius without a stop-start seam.
module rrect(w, d, h, r) {
    hull() for (x = [-1, 1], y = [-1, 1])
        translate([x * (w / 2 - r), y * (d / 2 - r), 0])
            cylinder(r = r, h = h);
}

// A D-shaped shaft bore along +Z from z = 0, flat facing -X.
module d_bore_z(h, extra = fit_shaft) {
    d = shaft_d + extra;
    intersection() {
        cylinder(d = d, h = h);
        translate([-shaft_flat_off - extra / 2, -d, 0]) cube([2 * d, 2 * d, h]);
    }
}

// A D-shaped shaft bore along +X from x = 0, flat facing +Z.
//
// The flat faces *up* on purpose. Printed lying down, the ceiling of a round
// bore droops; the ceiling of this one is a flat 4.5 mm bridge, which is the
// one span FDM does perfectly. It is also the face the D takes its torque on.
module d_bore_x(len, extra = fit_shaft) {
    d = shaft_d + extra;
    intersection() {
        rotate([0, 90, 0]) cylinder(d = d, h = len);
        translate([0, -d, -d]) cube([len, 2 * d, d + shaft_flat_off + extra / 2]);
    }
}

// The four NEMA17 mounting holes, drilled along +Z from z = 0.
module nema_bolt_holes(h, d = m3_free) {
    for (x = [-1, 1], y = [-1, 1])
        translate([x * motor_bolt_span / 2, y * motor_bolt_span / 2, -eps])
            cylinder(d = d, h = h + 2 * eps);
}

// The D-shaft alone. Its own module because it does not belong to the same rigid
// body as the motor it comes out of: it turns with whatever is clamped to it.
module nema17_shaft(shaft_rot = 0) {
    color("#c0c6cc") rotate([0, 0, shaft_rot]) intersection() {
        cylinder(d = shaft_d, h = shaft_len);
        translate([-shaft_flat_off, -shaft_d, 0])
            cube([2 * shaft_d, 2 * shaft_d, shaft_len]);
    }
}

// A stand-in NEMA17 with its driver, for the assembly preview only. Front face
// at z = 0, shaft along +Z, driver on the back.
//
// `shaft_rot` turns the D-flat about the shaft axis. It exists because the flat
// has to end up facing the same way as the bore that grips it, and on the tilt
// motor - which is mounted on its side - it did not: the interference test found
// the shaft's round side pressing 0.5 mm into the bore's flat. Which way the
// flat actually points on your motor is a matter of which of the four bolt
// orientations you use and where the rotor happens to be, so this is a statement
// about the preview, not a constraint on the build. Set the tilt zero in
// firmware once it is together.
module nema17_with_driver(shaft_rot = 0, with_shaft = true) {
    color("#3a3f45")
        translate([-motor_body / 2, -motor_body / 2, -motor_len])
            cube([motor_body, motor_body, motor_len]);
    // Bored for the shaft. The two are one rigid body on a real motor and would
    // never be compared - but the shaft is on the moving side of the
    // interference test now, so a solid boss reads as 208 facets of collision
    // with the shaft coming out of it, at every angle equally.
    color("#9aa3ad") difference() {
        cylinder(d = motor_boss_d, h = motor_boss_h);
        translate([0, 0, -eps]) cylinder(d = shaft_d + 0.4, h = motor_boss_h + 2 * eps);
    }
    if (with_shaft) color("#c0c6cc") rotate([0, 0, shaft_rot]) intersection() {
        cylinder(d = shaft_d, h = shaft_len);
        translate([-shaft_flat_off, -shaft_d, 0])
            cube([2 * shaft_d, 2 * shaft_d, shaft_len]);
    }
    color("#1d5c2f")
        translate([-driver_pcb / 2, -driver_pcb / 2, -motor_len - driver_depth])
            cube([driver_pcb, driver_pcb, driver_depth]);
    // OLED + buttons, on the edge that must stay reachable.
    color("#0b0d12")
        translate([-14, -driver_pcb / 2 - 0.8, -motor_len - driver_depth + 3])
            cube([28, 1.2, 9]);
}

// ---------------------------------------------------------------------------
// PART C - pedestal
// ---------------------------------------------------------------------------
//
// A motor with a driver on its back cannot stand on a bench, so something has to
// hold it up, and that something carries every load in the machine. The previous
// version used four 12 mm posts 60 mm long: the gimbal's overturning moment went
// into four slender columns in bending, which is the weakest structure you can
// build out of that much plastic, and its hold-down holes were in the top plate
// where no screwdriver can reach them.
//
// This is a closed tapered shroud instead. A shell resists an overturning moment
// in shear rather than in bending, it does not care which direction the moment
// comes from (the pan axis turns, so it comes from all of them), and the taper
// puts the width where the leverage is - 84 mm at the desk, 56 mm at the top.
//
// MODELLED UPSIDE DOWN: top plate flat on the bed, shroud rising, desk end last.
// That orientation is what makes it printable without a scrap of support -
// the walls draft *outward* at 13 deg as they rise, and the plate's mating face
// is the one surface an FDM printer makes perfectly flat, the first layer.

// The running gap at the pan axis, and what fills it.
//
// The previous version left 0.4 mm of air here and then claimed the yoke's pad
// carried the overturning moment. It would have - after deflecting 0.4 mm, by
// which point the 5 mm shaft had already taken the load the pad was there to keep
// off it. An interface that only engages once something has bent is not an
// interface, and this is the one place in the machine where two printed faces
// have to slide on each other under load.
//
// So the gap is *filled*, by three Ø8 self-adhesive PTFE discs recessed into the
// plate. Three, not a ring: three points cannot rock, and a printed 70 mm face
// that warps two tenths would otherwise pick its own three. The cost is friction,
// and it is small - about 11 N.mm against the motor's ~400 N.mm of holding torque
// - which is the entire argument for PTFE over plastic on plastic, where it would
// be four times that.
//
// They go on the plate rather than on the yoke because of the four countersunk
// motor screws: a recess in the yoke's ring at this radius would sweep across
// their heads once per revolution, and there is no radius that clears them and
// still lands inside the ring. Fixed to the plate, the pads simply sit between
// them - and `check_clearances.py` checks all three of "inside the yoke's ring",
// "clear of a screw head" and "on the plate", because that placement is now a
// constraint shared between two parts and nothing in either file would notice it
// breaking.
slew_gap     = 0.4;
glide_d      = 8.4;    // recess for a Ø8 pad
glide_t      = 0.8;
glide_recess = glide_t - slew_gap;   // so the pad stands exactly slew_gap proud
glide_r      = 28.5;
glide_a      = 30;     // first pad's angle; the triad is +/-120 from it

ped_plate_t = 5.0;
ped_top     = 74.0;   // across flats at the plate; sized by the yoke's pad
ped_top_r   = 8.0;
ped_foot    = 96.0;   // across flats at the desk
ped_foot_r  = 14.0;
ped_wall    = w8;
// Air below the driver, for finger room on the three buttons and for the wires
// to turn without a sharp bend.
ped_clear   = 14.0;
ped_shroud_h = motor_len + driver_depth + ped_clear;
ped_h       = ped_plate_t + ped_shroud_h;

// Hold-down ears, on the four *corners*. Not on the flat faces, and that is not
// a styling choice: an ear on a flat face needs 20 mm of wall above the aperture
// to blend into, and the top 16 mm of that wall is exactly where the driver's
// OLED and buttons sit. The corners are the only material this part has to
// spare, they are its stiffest feature, and an ear there reaches outboard where
// a screwdriver can get at it with the motor in place.
//
// Each ear carries a hold-down screw *and* a rubber foot, so the shroud's thin
// desk-end rim floats clear of the bench instead of trying to be a bearing
// surface.
ear_t      = 4.5;
ear_blend  = 20.0;  // how far back up the corner the hull reaches
ear_foot_r = 7.5;
ear_bolt_r = 5.5;
ear_seed_w = 12.0;  // spread across the corner, so the blend is a wedge not a cone

// Where the shroud's outer surface actually is, at a given height. The ear has
// to start *on* that surface: the first attempt used a plausible-looking
// constant, which the taper had already moved 4 mm outboard of, so the whole
// blend ended up buried in the wall and the only thing left in the air was a
// flat 13 x 14 mm shelf. check_printability.py found that; a render did not.
function ped_hw(z)     = ped_top / 2 + (ped_foot - ped_top) / 2 * z / ped_h;
function ped_cr(z)     = ped_top_r + (ped_foot_r - ped_top_r) * z / ped_h;
function ped_diag_r(z) = (ped_hw(z) - ped_cr(z)) * sqrt(2) + ped_cr(z);

// Kept as offsets from the shell rather than as absolute radii, so that the one
// number each expresses - "the foot sits inboard of the corner", "the bolt sits
// outboard where a screwdriver reaches it" - survives a change to the taper.
// They are also literals, which is what lets the viewer read them.
ear_foot_inset = 8.0;
ear_bolt_out   = 5.0;
ear_foot_d = ped_diag_r(ped_h) - ear_foot_inset;
ear_bolt_d = ped_diag_r(ped_h) + ear_bolt_out;
ear_seed_z = ped_h - ear_t - ear_blend;
// Buried 3 mm, not grazing the surface: a blend that meets the wall
// tangentially ends in a sliver of near-horizontal underside, which is both a
// scrappy thing to print and impossible to measure cleanly. Starting inside the
// wall means the underside is already at its full angle where it emerges.
ear_seed_d = ped_diag_r(ear_seed_z) - 3.0;

// Self-adhesive rubber feet, 12 mm. Recessed so they locate, and because
// rubber is the honest answer to a four-point stance on a printed part: a large
// flat printed ring will warp a few tenths and rock, compliant feet will not.
foot_d  = 12.4;
foot_z  = 1.2;

// A point at `dia` along the +45 deg diagonal.
function diag(dia) = [dia / sqrt(2), dia / sqrt(2)];

// Aperture in each wall. Hexagonal rather than rectangular so the roof is a
// short bridge with 45 deg flanks either side of it, instead of one long span.
ap_w       = 42.0;  // widest point
ap_roof    = 10.0;  // flat part of the roof: the only real bridge
ap_bottom  = 6.0;   // ring of wall left at the plate, for the load to enter it
ap_top_gap = 7.5;   // ring of wall left at the desk end
ap_flank   = 40.0;  // degrees from vertical

module _ped_shell(inset = 0) {
    // The taper as a single hull between the plate footprint and the desk
    // footprint, less `inset` all round. Two of these differenced gives a shell
    // of constant thickness without minkowski (which is slow and, at this corner
    // radius, unnecessary).
    //
    // The bed chamfer is a third section in the *same* hull rather than a cutter
    // subtracted afterwards. A cutter has to guess where the wall will be, and
    // the wall is already flaring outward by then: the first version guessed
    // 0.13 mm low and left a ledge round the entire 56 mm perimeter. Inside one
    // hull the surface is continuous by construction.
    r0 = max(0.5, ped_top_r - inset);
    hull() {
        rrect(ped_top - 2 * (inset + foot_ch), ped_top - 2 * (inset + foot_ch), eps, r0);
        translate([0, 0, foot_ch])
            rrect(ped_top - 2 * inset, ped_top - 2 * inset, eps, r0);
        translate([0, 0, ped_h - eps])
            rrect(ped_foot - 2 * inset, ped_foot - 2 * inset, eps,
                  max(0.5, ped_foot_r - inset));
    }
}

module _ped_aperture() {
    // Cut through one wall, +Y side, extruded along Y so the shape stays put
    // through a sloping wall.
    //
    // Both ends taper at 45 deg. The top taper is a print constraint - it turns
    // one 38 mm span into a 12 mm one. The bottom taper is a structural one: it
    // hands the wall's shear off into the corners diagonally instead of ending
    // the opening in two square notches right where the load enters the plate.
    z0 = ped_plate_t + ap_bottom;
    z1 = ped_h - ap_top_gap;
    h  = z1 - z0;
    run  = (ap_w - ap_roof) / 2;
    // 40 deg from vertical, not 45. At exactly 45 a facet is neither supported
    // nor not: it sits on the threshold every slicer and every checker compares
    // against, and which side of it you land on comes down to rounding.
    rise = run / tan(ap_flank);
    translate([0, ped_foot, z0]) rotate([90, 0, 0])
        linear_extrude(height = 2 * ped_foot)
            polygon([
                [-ap_roof / 2, 0], [ap_roof / 2, 0],
                [ap_w / 2, rise], [ap_w / 2, h - rise],
                [ap_roof / 2, h], [-ap_roof / 2, h],
                [-ap_w / 2, h - rise], [-ap_w / 2, rise],
            ]);
}

// One hold-down ear: a flat pad at the desk end, wedged back into the corner.
module _ped_ear() {
    z0 = ped_h - ear_t;
    hull() {
        translate(concat(diag(ear_foot_d), z0)) cylinder(r = ear_foot_r, h = ear_t);
        translate(concat(diag(ear_bolt_d), z0)) cylinder(r = ear_bolt_r, h = ear_t);
        // A line *across* the corner rather than a point on it. A point would
        // hull into a cone hanging off the wall; a line hulls into a wedge that
        // hugs it.
        for (s = [-1, 1])
            translate(concat(diag(ear_seed_d), ear_seed_z)
                      + [s * ear_seed_w / 2 / sqrt(2), -s * ear_seed_w / 2 / sqrt(2), 0])
                cylinder(r = 0.5, h = eps);
    }
}

module pedestal() {
    difference() {
        union() {
            difference() {
                _ped_shell();
                // Hollow, from the plate's inner face to the open desk end.
                translate([0, 0, ped_plate_t]) _ped_shell(ped_wall);
                for (a = [0, 90, 180, 270]) rotate([0, 0, a]) _ped_aperture();
            }
            // Hold-down ears, at the corners. `diag()` already places its
            // argument on the +45 deg diagonal, so these rotate by 0/90/180/270
            // and not by 45 - doing both put every ear in the middle of a wall,
            // hanging over the aperture it was designed to keep clear of, which
            // the render looked perfectly happy about.
            //
            // Unioned after the hollowing on purpose: they are
            // meant to thicken the corner inward as well as reach outward, and
            // the only thing inside the shroud at this height is the finger room
            // below the driver - which is entered through the flat faces, not
            // the corners.
            for (a = [0, 90, 180, 270]) rotate([0, 0, a]) _ped_ear();
        }

        // --- the plate's motor interface -------------------------------------
        // Countersunk M3, opening at the bed face. A conical recess that opens
        // downward at the first layer prints perfectly and self-centres the
        // screw; a counterbore for a socket head would eat 3.2 of the 5 mm
        // plate, and the head still has to end up below flush because the
        // yoke's pad slides across this face.
        nema_bolt_holes(ped_plate_t);
        for (x = [-1, 1], y = [-1, 1])
            translate([x * motor_bolt_span / 2, y * motor_bolt_span / 2, -eps])
                cylinder(d1 = m3_cs_d, d2 = m3_free, h = m3_cs_h + eps);
        // Boss clearance and shaft pass-through.
        translate([0, 0, -eps]) cylinder(d = motor_boss_d + fit_boss,
                                        h = ped_plate_t + 2 * eps);

        // Glide pad recesses, in the face the yoke runs on. `glide_a` is 30 and
        // not 0: the pads have to sit between the four countersunk screws at
        // 45/135/225/315, and a 120 deg triad can only be 15 deg off them at
        // best - which it is here, and which leaves 1.9 mm of plate.
        for (a = [glide_a, glide_a + 120, glide_a + 240]) rotate([0, 0, a])
            translate([glide_r, 0, -eps])
                cylinder(d = glide_d, h = glide_recess + eps);

        // --- desk end --------------------------------------------------------
        for (a = [0, 90, 180, 270]) rotate([0, 0, a]) {
            translate(concat(diag(ear_bolt_d), ped_h - ear_t - eps))
                cylinder(d = m4_free, h = ear_t + 2 * eps);
            translate(concat(diag(ear_foot_d), ped_h - foot_z))
                cylinder(d = foot_d, h = foot_z + eps);
        }

        // --- cable strain relief --------------------------------------------
        // Two slots through one corner for a single zip tie: the pan motor's
        // harness gets anchored to the pedestal so nothing can pull on a screw
        // terminal. In the corner, where the wall is solid.
        rotate([0, 0, 45]) for (s = [-1, 1])
            translate([s * 7, 0, ped_plate_t + 9]) rotate([-90, 0, 0])
                linear_extrude(height = ped_foot)
                    square([w4 + 0.4, 9], center = true);

    }
}

// ---------------------------------------------------------------------------
// PART A - pan yoke
// ---------------------------------------------------------------------------
//
// A fork, not an L. The previous version reached up and sideways on a single
// cantilevered plate, so the camera's *static* weight arrived at the column root
// as tension across the layers - the one direction an FDM part is weak in. A
// fork with the tilt axis crossing the pan axis turns the same weight into
// compression straight down two vertical arms, and leaves the payload where it
// belongs: over the axis it is being turned about.
//
// The other half of the job is the pan bearing. A 5 mm shaft in a printed hub is
// a poor thrust bearing and a worse moment bearing, so it is not asked to be
// either: the 70 mm pad underneath carries both, and the shaft is left with
// nothing to transmit but torque.
//
// What the pad runs on is three PTFE glide pads set into the pedestal's plate -
// see `slew_gap` there for why the gap is filled rather than left as air, and why
// the pads live on the other part.
//
// Print as modelled - pad down. Nothing here needs support.

yoke_pad_d  = 70.0;
yoke_pad_t  = 4.0;
// The pad bears on a ring at its rim, not on its whole face, and the glide pads
// run on that ring. Its width is therefore not free: `glide_r` has to land
// inside it with room to spare, and the checker says so out loud.
yoke_ring_w = 13.0;
yoke_relief = 0.6;

// 22, not 20, and it is the set screw that says so: see `grub_flat_x`.
pan_hub_od  = 22.0;
pan_hub_h   = 12.0;

// The shaft joints are a close D-bore plus one screw bearing on the shaft's
// flat. Not a split clamp - and that is the one place this design argues with
// its predecessor's reasoning rather than its geometry.
//
// A split clamp needs to *close*, and both of these hubs stand on something
// stiff: the yoke's on a 64 mm pad, the cradle's on a 76 mm platform. Slitting a
// hub that is bonded along its whole base to a plate like that gives you the
// slit's stress concentration and none of its compliance. The D-flat is already
// a positive drive - it takes the torque whether or not anything is clamped - so
// the screw only has to take up the bore clearance and stop the part lifting,
// which a screw straight onto the flat does without needing anything to flex.
//
// A flat down one side of the hub for the insert to be pressed into square. Its
// distance from the axis is not a styling choice: an M3 set screw has to cross
// the insert and then the clearance hole to touch the shaft's flat, and
//
//   12.0 - (shaft_flat_off + fit_shaft/2) = 9.93 mm
//
// which is what makes a stock M3x10 finish flush instead of standing proud. The
// hub's 22 mm diameter follows from wanting that flat to be a 1 mm boss on it
// rather than a cut into it.
grub_flat_x = 12.0;

// The fork. Wide because the payload sets it: a 52 mm camera has to swing
// *between* the cradle's cheeks for the tilt axis to pass through its body, and
// everything else is stacked outboard of that.
arm_gap     = 84.0;   // clear span between the arms: 3 mm each side of the cradle
arm_t_root  = 10.0;
arm_t_top   = 7.2;
arm_y_root  = 22.0;
arm_x_root  = 23.0;   // inner face where the arm lands on the pad
arm_mid_z   = 27.0;   // splayed out by here, vertical above it
arm_y_mid   = 26.0;
// Above the pad's underside, and set by the cradle's sweep: at tilt -45 the
// platform's rear corner drops to 34 mm below the axis, and it has to still be
// above `arm_mid_z` - where the arms have finished splaying - or it swings into
// the part of the fork that is still leaning inward. 63 leaves 2.8 mm.
tilt_axis_h = 63.0;
tilt_pad    = 42.0;   // motor face pad: 31 mm bolt square + 5.5 mm of material
tilt_pad_r  = 7.0;
pivot_pad_d = 26.0;   // the other arm carries the pivot pin and its two keepers

// The pivot pin's interface with this arm. The pin itself is PART D; what the
// fork owns is a plain through hole for its journal and two keeper screws.
pin_hole_d   = brg_journal_d + 0.35;  // slip fit, and it droops a little shut
pin_flange_d = 22.0;
pin_flange_t = 3.0;
pin_screw_r  = 8.0;   // clear of both the journal and the pad's edge

// A hole through a wall printed on its side, with a peak on top instead of a
// flat roof: turns a 22 mm ceiling into a 12 mm one and puts the sag above the
// circle, where it cannot stop a motor boss seating.
module tear_x(d, len) {
    hull() {
        rotate([0, 90, 0]) cylinder(d = d, h = len);
        translate([0, 0, d * 0.35]) rotate([0, 90, 0]) cylinder(d = d * 0.55, h = len);
    }
}

// Two sections, not one, and this is the design's least obvious coupling.
//
// A single hull from the root straight up to the motor pad makes the arm reach
// its full width only at the *pad's bottom edge* - which sits tilt_pad/2 below
// the tilt axis and therefore rises with it. The cradle swings 31 mm below that
// axis at its full 72 mm width, so the fork was always narrower than the cradle
// exactly where the cradle needed the room, and no amount of extra height ever
// fixed it: raising the axis moved both at once. `intersection()` found it.
//
// Splaying to full width by a fixed height and standing vertical above it
// decouples the two, at the cost of one more hull.
module _yoke_arm(s) {
    mid = [s * (arm_gap / 2 + arm_t_top / 2), 0, arm_mid_z];
    hull() {
        translate([s * (arm_x_root + arm_t_root / 2), 0, yoke_pad_t - eps])
            rrect(arm_t_root, arm_y_root, eps, 3);
        translate(mid) rrect(arm_t_top, arm_y_mid, eps, 3);
    }
    hull() {
        translate(mid) rrect(arm_t_top, arm_y_mid, eps, 3);
        translate([s * arm_gap / 2, 0, tilt_axis_h]) rotate([0, s * 90, 0])
            if (s > 0) rrect(tilt_pad, tilt_pad, arm_t_top, tilt_pad_r);
            else cylinder(d = pivot_pad_d, h = arm_t_top);
    }
}

module pan_yoke() {
    difference() {
        union() {
            // Bearing pad, chamfered where it meets the bed.
            hull() {
                cylinder(d = yoke_pad_d - 2 * foot_ch, h = eps);
                translate([0, 0, foot_ch])
                    cylinder(d = yoke_pad_d, h = yoke_pad_t - foot_ch);
            }
            // Hub, coned into the pad rather than butted onto it, and with a
            // flat down one side for the screw head.
            hull() {
                cylinder(d = pan_hub_od, h = pan_hub_h);
                translate([-grub_flat_x, -6, 0]) cube([1, 12, pan_hub_h]);
            }
            cylinder(d1 = pan_hub_od + 9, d2 = pan_hub_od, h = 4.5);
            for (s = [-1, 1]) _yoke_arm(s);
        }

        // --- pan shaft -------------------------------------------------------
        translate([0, 0, -eps]) d_bore_z(pan_hub_h + 2 * eps);
        // Lead-in, so a squashed first layer cannot stop the yoke going on.
        translate([0, 0, -eps])
            cylinder(d1 = shaft_d + 1.6, d2 = shaft_d + fit_shaft, h = 0.8 + eps);
        // Set screw onto the flat: insert first, then clearance to the shaft.
        // The insert hole is the wider `insert_d_h` because it prints on its
        // side, and a hole printed on its side comes out undersize at the top.
        translate([-grub_flat_x - eps, 0, pan_hub_h / 2]) rotate([0, 90, 0]) {
            cylinder(d = insert_d_h, h = insert_depth + eps);
            cylinder(d = grub_clear, h = grub_flat_x - shaft_flat_off + eps);
        }

        // --- bearing relief --------------------------------------------------
        translate([0, 0, -eps]) difference() {
            cylinder(d = yoke_pad_d - 2 * yoke_ring_w, h = yoke_relief + eps);
            translate([0, 0, -eps]) cylinder(d = pan_hub_od + 5, h = yoke_relief + 3 * eps);
        }

        // --- tilt motor, on the +X arm ---------------------------------------
        translate([arm_gap / 2 - eps, 0, tilt_axis_h]) rotate([0, 90, 0]) {
            nema_bolt_holes(arm_t_top + 2 * eps, m3_free_h);
        }
        translate([arm_gap / 2 - eps, 0, tilt_axis_h])
            tear_x(motor_boss_d + fit_boss, arm_t_top + 2 * eps);

        // --- pivot pin, on the -X arm ----------------------------------------
        // A through hole for the pin's journal and two keeper screws either side
        // of it. The pin goes in from *outside*, which is the whole reason it is
        // a separate part: a journal moulded onto this arm - or onto the cradle -
        // could only be assembled by springing the fork apart by more than the
        // 3 mm of side clearance it has, and the fork cannot be made wider
        // without the tilt shaft running out of engagement.
        translate([-arm_gap / 2 - arm_t_top - eps, 0, tilt_axis_h]) rotate([0, 90, 0])
            cylinder(d = pin_hole_d, h = arm_t_top + 2 * eps);
        for (s = [-1, 1])
            translate([-arm_gap / 2 - arm_t_top - eps, s * pin_screw_r, tilt_axis_h])
                rotate([0, 90, 0]) cylinder(d = m3_pilot, h = arm_t_top + 2 * eps);

        // --- cable tie, at the back of the pad -------------------------------
        for (s = [-1, 1])
            translate([s * 6 - 1, -28.5, -eps]) cube([2, 6, yoke_pad_t + 2 * eps]);
    }
}

// ---------------------------------------------------------------------------
// PART B - camera cradle
// ---------------------------------------------------------------------------
//
// Two cheeks straddling the camera, so the tilt axis passes through the payload
// instead of under it. That is what keeps the axis balanced: these drivers hold
// full Ma continuously in the OPEN and CLOSE work modes, so an unbalanced tilt
// axis is not merely wasted torque, it is a motor running hot doing nothing.
//
// The far cheek carries a 6700 ball bearing, riding on the pin in the other fork
// arm. The tilt axis is therefore supported at both ends, which is the difference
// between a shaft carrying a bending moment and a shaft carrying only torque -
// and the shaft in question is 5 mm of steel in a printed bore.
//
// Print platform-down. Bending from the camera's weight then runs *along* the
// layers, and the bore's ceiling is the D-flat: a 4.5 mm flat bridge, which is
// the one span an FDM printer reproduces exactly.

// Representative payload, and the only place it is written down: the clearance
// check reads these out of this file. A compact mirrorless body - deliberately
// larger than the webcam or Pi camera most builds will carry.
cam_w = 52.0;
cam_h = 44.0;
cam_d = 34.0;

plat_w   = 78.0;   // across the arms; cheeks at +/-39
plat_l   = 44.0;
plat_t   = 5.0;
plat_r   = 6.0;
// Thick enough to swallow the shaft whole, which is a payload constraint before
// it is a strength one: whatever the bore does not contain sticks out past it
// into the space the camera occupies. The tilt motor's shaft ran 3.1 mm into the
// camera envelope before this was 11.
cheek_t  = 11.0;
// The tilt axis, above the platform's underside. This is the *balance* number,
// not a styling one: put it below the payload's centre of mass and the axis is a
// pendulum the motor has to hold up all day. Platform base plus half a 44 mm
// camera is 27; the cradle's own mass sits low and pulls the combined centre
// down to about 25 for anything between 150 g and 600 g of payload, which leaves
// under 7 N.mm of standing torque across that whole range.
axis_z   = 25.0;
hub_od   = 20.0;   // the shaft side
// The bearing side is fatter because it has to be: a 14.9 mm pocket needs wall
// round it, and 24 leaves 4.6 mm.
pivot_hub_od = 24.0;
cam_slot = 20.0;   // fore/aft travel for balancing

module _cradle_cheek(s, hub) {
    x0 = s > 0 ? plat_w / 2 - cheek_t : -plat_w / 2;
    hull() {
        translate([x0, -plat_l / 2, plat_t - eps]) cube([cheek_t, plat_l, eps]);
        translate([x0, -hub / 2 - 2, axis_z - hub / 2])
            cube([cheek_t, hub + 4, eps]);
        // Hulling the cheek into the hub leaves the hub's lower half inside the
        // solid, so the one shape here that would have been an overhang - a
        // cylinder lying on its side - never has an underside at all.
        translate([x0, 0, axis_z]) rotate([0, 90, 0]) cylinder(d = hub, h = cheek_t);
    }
}

module camera_cradle() {
    difference() {
        union() {
            hull() {
                rrect(plat_w - 2 * foot_ch, plat_l - 2 * foot_ch, eps, plat_r);
                translate([0, 0, foot_ch])
                    rrect(plat_w, plat_l, plat_t - foot_ch, plat_r);
            }
            _cradle_cheek(1, hub_od);
            _cradle_cheek(-1, pivot_hub_od);
        }

        // --- tilt shaft, in the +X cheek --------------------------------------
        translate([plat_w / 2 - cheek_t, 0, axis_z]) d_bore_x(cheek_t + eps);
        translate([plat_w / 2 + eps, 0, axis_z]) rotate([0, -90, 0])
            cylinder(d1 = shaft_d + 1.6, d2 = shaft_d + fit_shaft, h = 0.8 + eps);
        // Set screw onto the flat, straight down from the top of the hub, with
        // its insert at the top where a soldering iron can reach it. Printed
        // upright, so this one gets the narrower `insert_d`.
        translate([plat_w / 2 - cheek_t / 2, 0, axis_z + shaft_flat_off]) {
            cylinder(d = grub_clear, h = hub_od / 2 - shaft_flat_off + eps);
            translate([0, 0, hub_od / 2 - shaft_flat_off - insert_depth])
                cylinder(d = insert_d, h = insert_depth + 2 * eps);
        }

        // --- pivot bearing, in the -X cheek -----------------------------------
        // The pocket opens outward and bottoms on a shoulder, so the outer race
        // is located by a printed face rather than by how hard you pressed it.
        // Behind the shoulder the relief runs right through the cheek: it clears
        // the inner race, and it means the bearing can be pushed back out with a
        // rod instead of levered out of a blind hole.
        translate([-plat_w / 2 - eps, 0, axis_z]) rotate([0, 90, 0]) {
            cylinder(d = brg_seat_d, h = brg_seat_depth + eps);
            cylinder(d = brg_relief_d, h = cheek_t + 2 * eps);
        }

        // --- camera screw -----------------------------------------------------
        // A slot, not a row of holes: balance is a continuous adjustment, and it
        // has to be redone after any lens change.
        translate([0, 0, -eps]) hull() for (y = [-1, 1])
            translate([0, y * cam_slot / 2, 0])
                cylinder(d = cam_screw_d, h = plat_t + 2 * eps);
    }
}

// ---------------------------------------------------------------------------
// PART D - pivot pin
// ---------------------------------------------------------------------------
//
// The tilt axis' far journal, and the fourth part exists because of assembly
// order rather than because of load. The bearing has to end up in the cradle's
// cheek and on something attached to the fork, and the cradle only has 3 mm of
// side clearance to slide in - less than the bearing is wide. A journal moulded
// onto either part therefore cannot be got into the other one without springing
// the fork, and the fork cannot be widened to make room because the tilt shaft
// is 20 mm long and already only reaches 9.8 mm into its bore.
//
// A pin fitted from *outside* the arm has none of that problem: the cradle drops
// in with clearance all round, and the pin goes through afterwards. It also comes
// back out, which matters more than it sounds - taking the cradle off is how you
// re-balance after a lens change.
//
// Print flange-down: the journal comes out round because it is a vertical
// cylinder, the countersinks open at the first layer, and the bending stress in
// the journal at 2 N is 0.1 MPa, so which way the layers run is irrelevant here.

tilt_gap  = arm_gap / 2 - plat_w / 2;   // side clearance, per side
// How far the journal stands proud of the arm's inner face: across the gap, into
// the bearing, and 1.5 mm beyond it into the cheek's relief. That overrun is the
// axial tolerance of the whole joint - the cradle can sit 1.5 mm further from
// this arm than nominal and the bearing is still fully on the journal.
pin_overrun = 1.5;
pin_stick   = tilt_gap + brg_seat_depth + pin_overrun;
pin_len     = arm_t_top + pin_stick;

module pivot_pin() {
    difference() {
        union() {
            hull() {
                cylinder(d = pin_flange_d - 2 * foot_ch, h = eps);
                translate([0, 0, foot_ch])
                    cylinder(d = pin_flange_d, h = pin_flange_t - foot_ch);
            }
            translate([0, 0, pin_flange_t - eps]) {
                cylinder(d = brg_journal_d, h = pin_len - 0.6 + eps);
                // Lead-in, so the bearing starts square instead of on one edge.
                translate([0, 0, pin_len - 0.6])
                    cylinder(d1 = brg_journal_d, d2 = brg_journal_d - 1.2, h = 0.6);
            }
        }
        // Two keeper screws. They hold the pin in; they are not in the load path,
        // which is the journal against the walls of its hole.
        for (s = [-1, 1]) translate([0, s * pin_screw_r, -eps]) {
            cylinder(d = m3_free, h = pin_flange_t + 2 * eps);
            cylinder(d1 = m3_cs_d, d2 = m3_free, h = m3_cs_h + eps);
        }
    }
}

// The bearing itself, in two pieces, because its two races belong to different
// rigid bodies: the outer one turns with the cradle and the inner one sits still
// on the pin. Modelling it as one solid ring would make the interference test
// compare the pin with something clamped to the part rotating around it.
//
// Both races are drawn at their *seat* sizes rather than their catalogue sizes,
// so the press fits do not read as collisions. A press fit is an intentional
// overlap of a tenth of a millimetre, and the exact test would report it as
// 14 mm^3 of interpenetration - true, and not what the test is for.
module bearing_outer() {
    color("#8f979f") difference() {
        cylinder(d = brg_seat_d - 0.02, h = brg_w);
        translate([0, 0, -eps]) cylinder(d = brg_od - 1.8, h = brg_w + 2 * eps);
    }
}

module bearing_inner() {
    color("#8f979f") difference() {
        cylinder(d = brg_id + 1.7, h = brg_w);
        translate([0, 0, -eps]) cylinder(d = brg_journal_d + 0.02, h = brg_w + 2 * eps);
    }
}

// ---------------------------------------------------------------------------
// Assembly preview
// ---------------------------------------------------------------------------
//
// Desk at z = 0. This is the only place any part gets reoriented: the pedestal
// is modelled the way it prints, which is upside down from the way it stands.

ped_top_z = ped_h;                       // the plate's mating face, above the desk
yoke_z    = ped_top_z + slew_gap;
tilt_z    = yoke_z + tilt_axis_h;
tilt_face_x = arm_gap / 2 + arm_t_top;   // where the tilt motor's face lands

// Split into what turns with the tilt axis and what does not, so the same
// geometry serves the preview *and* the interference test. A separate copy for
// the test would be a copy that can disagree with the thing it is testing.

// What turns with the tilt axis - and that includes the tilt motor's *shaft*,
// which is clamped to the cradle. Leaving it on the fixed side made the
// interference test hold the shaft still while rotating the bore around it, and
// report a collision at every tilt but zero. A model of what moves is part of
// the test, and it can be wrong in the same way geometry can.
module moving_assembly(tilt_deg = 0, with_camera = true) {
    translate([0, 0, tilt_z]) rotate([tilt_deg, 0, 0]) translate([0, 0, -tilt_z]) {
        color("#2a7fd8") translate([0, 0, tilt_z - axis_z]) camera_cradle();
        translate([tilt_face_x, 0, tilt_z]) rotate([0, -90, 0])
            nema17_shaft(shaft_rot = 180);
        // The bearing's outer race, pressed into the far cheek and turning with
        // it. Its inner race is on the pin, in fixed_assembly().
        translate([-plat_w / 2, 0, tilt_z]) rotate([0, 90, 0])
            translate([0, 0, brg_seat_depth - brg_w]) bearing_outer();
        if (with_camera)
            color("#23272e", 0.55)
                translate([-cam_w / 2, -cam_d / 2, tilt_z - axis_z + plat_t])
                    cube([cam_w, cam_d, cam_h]);
    }
}

// `only` selects one member, so the interference test can name which pair of
// parts overlaps instead of just reporting that something does.
module fixed_assembly(only = "all") {
    if (only == "all" || only == "pedestal")
        color("#6b7480") translate([0, 0, ped_h]) rotate([180, 0, 0]) pedestal();
    // Pan motor, hanging inside the shroud, shaft up through the plate.
    if (only == "all" || only == "pan motor")
        translate([0, 0, ped_top_z - ped_plate_t]) nema17_with_driver();
    if (only == "all" || only == "pedestal")
        // The glide pads, filling the gap the yoke runs on. Drawn with the
        // pedestal because they are stuck to its plate and do not turn.
        color("#e8e8e4") for (a = [glide_a, glide_a + 120, glide_a + 240])
            rotate([0, 0, a]) translate([glide_r, 0, ped_top_z - glide_recess])
                cylinder(d = glide_d - 0.4, h = glide_t);
    if (only == "all" || only == "yoke")
        color("#d8632a") translate([0, 0, yoke_z]) pan_yoke();
    // Tilt motor, bolted to the outside of the +X arm, shaft pointing in.
    if (only == "all" || only == "tilt motor")
        translate([tilt_face_x, 0, tilt_z]) rotate([0, -90, 0])
            nema17_with_driver(with_shaft = false);
    // Pivot pin, through the -X arm from outside, carrying the bearing's inner
    // race. Both are fixed to the fork, so both belong on this side of the test.
    if (only == "all" || only == "pivot pin")
        color("#c0c6cc") translate([-tilt_face_x - pin_flange_t, 0, tilt_z])
            rotate([0, 90, 0]) {
                pivot_pin();
                translate([0, 0, pin_flange_t + arm_t_top + tilt_gap
                                 + brg_seat_depth - brg_w])
                    bearing_inner();
            }
}

module assembly(pan_deg = 0, tilt_deg = 0, show_camera = true) {
    rotate([0, 0, pan_deg]) {
        fixed_assembly();
        moving_assembly(tilt_deg, show_camera);
    }
}

// The assembly cut on the y = 0 plane, which is the plane both axes lie in - so
// one image shows the pan shaft in its hub, the glide pads in their recesses and
// the pivot bearing on its pin. Every one of those is a fit you cannot see from
// outside, and a fit nobody looks at is a fit nobody notices is wrong.
//
// The payload is left out: it is a solid block 52 mm wide and it hides most of
// what the cut is for.
//
//   openscad -D 'part="section"' -D tilt=-20 gimbal_parts.scad
// The cutter is coloured, and that is not decoration: in a preview render the
// faces an intersection exposes take the colour of whatever did the cutting, so
// an uncoloured cube turns the entire cutaway into one flat silhouette.
module section(pan_deg = 0, tilt_deg = 0) {
    intersection() {
        assembly(pan_deg, tilt_deg, false);
        color("#b9bfc7") translate([-300, 0, -50]) cube([600, 400, 400]);
    }
}

// ---------------------------------------------------------------------------
// Interference solid
// ---------------------------------------------------------------------------
//
// The overlap between what moves and what does not. OpenSCAD will not tell you
// whether two solids intersect - but it will happily compute the intersection,
// and an empty one exports as an STL with zero facets. That is an exact test on
// the real geometry, which is worth having: the analytic sweep in
// check_clearances.py samples the cradle as points and describes the fork's arms
// with a plane, and an approximation of a shape is a shape you can get wrong.
//
// It was: the first version of that sweep put the arms' inner faces at a
// constant x. They splay - that is what makes them printable - so lower down
// they stand much closer together than at the top, which is exactly where the
// cradle swings. It reported 2.5 mm of clearance through a 4.4 mm overlap.
//
//   openscad -D 'part="interference"' -D tilt=-45 --export-format binstl \
//            -o /tmp/x.stl gimbal_parts.scad
against = "all";   // "pedestal", "pan motor", "yoke", "tilt motor", "pivot pin"

module interference(pan_deg = 0, tilt_deg = 0, only = "all") {
    intersection() {
        moving_assembly(tilt_deg);
        rotate([0, 0, -pan_deg]) fixed_assembly(only);
    }
}

// The one pair of parts that are *both* fixed and still have to fit each other:
// the pivot pin and the fork arm it passes through. The interference test above
// cannot see it - it compares what moves against what does not, and these are
// both on the same side of that line - so the only thing holding the pin in the
// right place was the same arithmetic written out twice, once to cut the hole and
// once to position the part. Which is exactly the shape of mistake that puts a
// flange on the wrong side of a wall and looks perfectly fine in a render.
//
// The pin's journal is 10.05 in a 10.4 hole and its flange lands on the arm's
// outer face, so the honest answer here is zero: a clearance fit and one
// coincident face.
//
//   openscad -D 'part="fit"' --export-format binstl -o /tmp/x.stl gimbal_parts.scad
module fit_check() {
    intersection() {
        fixed_assembly("pivot pin");
        fixed_assembly("yoke");
    }
}

// ---------------------------------------------------------------------------

if (part == "A") pan_yoke();
else if (part == "B") camera_cradle();
else if (part == "C") pedestal();
else if (part == "D") pivot_pin();
else if (part == "fit") fit_check();
else if (part == "section") section(pan, tilt);
else if (part == "interference") interference(pan, tilt, against);
else assembly(pan, tilt);
