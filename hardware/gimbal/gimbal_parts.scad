// Two-axis (pan/tilt) camera gimbal built from two NEMA17 32mm motors, each
// carrying an MKS SERVO42D_CAN driver on its back face.
//
// Two printed parts are required:
//
//   PART A  pan_yoke()        pan shaft  -> tilt motor
//   PART B  camera_cradle()   tilt shaft -> camera
//
// A third, optional part is included because a motor with a driver on its back
// cannot stand on a flat surface:
//
//   PART C  base_plate()      pan motor  -> tripod / bench
//
//
// WHAT THE DRIVER FORCES ON THE DESIGN
// ------------------------------------
// The SERVO42D occupies the whole back face of the motor (manual page 2) and
// puts hardware on all four edges:
//
//   top edge     4-way screw terminal, motor phases, wires exit upward
//   left edge    5-way screw terminal, EVCC/EGND/IN_1/CAN_H/CAN_L
//   right edge   6-way screw terminal, V+/GND/COM/EN/STP/DIR
//   bottom edge  OLED display and the Next / Enter / Menu buttons
//
// The bottom edge is the binding constraint. Work mode, Ma (working current)
// and HoldMa cannot be read back over CAN on firmware older than V1.0.6, so the
// OLED and its three buttons are the *only* way to see or change them. A
// bracket that covers them makes the motor unconfigurable.
//
// Therefore neither part touches the driver end. Both parts locate on the
// motor's front face (the 31 mm bolt square and the 22 mm boss) and on the
// D-shaft. The entire driver PCB, all three terminal blocks and the whole user
// interface stay in open air.
//
//
// PRINTING
// --------
// PLA or PETG, 0.2 mm layers, 4 perimeters, >=40% infill in the clamp hubs.
// Orientations are given per part below; each is chosen so the clamp screw
// tightens across layer lines rather than trying to peel them apart.
//
// Render one part at a time:
//   openscad -D 'part="A"' -o stl/pan_yoke.stl      gimbal_parts.scad
//   openscad -D 'part="B"' -o stl/camera_cradle.stl gimbal_parts.scad
//   openscad -D 'part="C"' -o stl/base_plate.stl    gimbal_parts.scad
//   openscad -D 'part="assembly"' -D pan=30 -D tilt=20 -o /dev/null gimbal_parts.scad

part = "assembly";  // "A", "B", "C", "assembly"
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
shaft_flat      = 4.5;    // across the D, full diameter minus the cut
shaft_len       = 20.0;   // from the front face

// MKS SERVO42D_CAN on the back face. Envelope is generous on purpose: the
// screw terminals stand proud of the PCB and the wires need a bend radius.
driver_pcb      = 42.5;
driver_depth    = 16.0;   // PCB plus tallest component, behind the motor
driver_wire_gap = 20.0;   // free air needed off the terminal edges

// Fasteners
m3_free        = 3.4;     // clearance
m3_head_d      = 6.2;     // socket cap
m3_nut_af      = 5.6;     // across flats, with a little slop
m3_nut_h       = 2.6;
cam_screw_d    = 6.6;     // 1/4"-20 clearance

// Fit allowances, tuned for a typical 0.4 mm nozzle
fit_shaft      = 0.15;    // added to the bore, taken up by the clamp
fit_boss       = 0.40;

// ---------------------------------------------------------------------------
// Gimbal geometry
// ---------------------------------------------------------------------------

// Height of the tilt axis above the pan motor's front face. Set by
// check_clearances.py, not by eye: the binding case is the *camera* clipping
// the yoke's horizontal beam at about -39 deg of tilt, not the cradle itself.
// 62 mm leaves only 2.8 mm there; 70 mm leaves 10.8 mm. Re-run that script
// after changing this, the beam, or the camera envelope.
tilt_axis_h = 70.0;

// Sideways offset from the pan axis to the tilt motor's mounting face. The
// tilt motor hangs outboard on this face with its shaft pointing back toward
// the pan axis, so the camera ends up sitting over the pan axis rather than
// cantilevered off it.
tilt_face_y = -30.0;

wall        = 3.2;
plate_t     = 4.0;   // every mm here is taken out of the 20 mm shaft
hub_wall    = 5.5;

// Gap between the yoke's plate and the cradle's hub, along the tilt shaft.
// Named because three things depend on it: the assembly preview, the clearance
// checker and the viewer. A magic 4 in each was how they drifted apart.
cradle_gap  = 3.5;

// Base plate (PART C). Declared here because the yoke's hub height depends on
// the plate thickness it has to sit above.
base_w    = motor_body + 28;
base_t    = 5.0;
// The motor hangs underneath: the plate bolts to its front face, the shaft
// passes up through the middle. So the legs must clear the whole motor, its
// driver, and enough air below to reach the buttons and route the wires.
leg_h     = motor_len + driver_depth + 12;
leg_d     = 12.0;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// A D-shaped shaft bore, axis along +Z, starting at z=0.
//   h    bore depth
//   extra  added to the nominal diameter
module d_bore(h, extra = fit_shaft) {
    d = shaft_d + extra;
    intersection() {
        cylinder(d = d, h = h);
        // Trim the flat. The cut plane sits at shaft_flat from the far side,
        // i.e. the D faces -X.
        translate([-d, -d, -eps])
            cube([d + shaft_flat - (shaft_d - shaft_flat) / 2, 2 * d, h + 2 * eps]);
    }
}

// A clamping hub for a D-shaft: bore, relief slit, and a cross screw that
// pulls the slit shut. Axis along +Z.
module shaft_clamp(h, od = shaft_d + 2 * hub_wall) {
    difference() {
        cylinder(d = od, h = h);
        translate([0, 0, -eps]) d_bore(h + 2 * eps);

        // Relief slit, opening toward +Y so the screw crosses it at right
        // angles. 1.6 mm is four extrusions and closes without cracking.
        translate([-0.8, 0, -eps]) cube([1.6, od, h + 2 * eps]);

        // Clamp screw across the slit, plus a captive nut on one side.
        translate([0, od / 2 - 1.0, h / 2]) rotate([0, 90, 0]) {
            cylinder(d = m3_free, h = od, center = true);
            translate([0, 0, od / 2 - 2.2])
                cylinder(d = m3_head_d, h = 4.4);
            translate([0, 0, -od / 2 - 2.2])
                rotate([0, 0, 30])
                    cylinder(d = m3_nut_af / cos(30), h = 4.4, $fn = 6);
        }
    }
}

// The four NEMA17 mounting holes, drilled along +Z from z=0.
module nema_bolt_holes(h, d = m3_free) {
    for (x = [-1, 1], y = [-1, 1])
        translate([x * motor_bolt_span / 2, y * motor_bolt_span / 2, -eps])
            cylinder(d = d, h = h + 2 * eps);
}

// Everything a motor needs cleared on the face it bolts to: the centre boss
// and a pass-through for the shaft.
module nema_face_reliefs(h) {
    translate([0, 0, -eps]) cylinder(d = motor_boss_d + fit_boss, h = motor_boss_h + eps);
    translate([0, 0, -eps]) cylinder(d = motor_boss_d + fit_boss, h = h + 2 * eps);
}

// A stand-in NEMA17 with its driver, for the assembly preview only. Front face
// at z=0, shaft along +Z, driver on the back.
module nema17_with_driver() {
    // body
    color("#404040")
        translate([-motor_body / 2, -motor_body / 2, -motor_len])
            cube([motor_body, motor_body, motor_len]);
    // boss + shaft
    color("#909090") cylinder(d = motor_boss_d, h = motor_boss_h);
    color("#c0c0c0") intersection() {
        cylinder(d = shaft_d, h = shaft_len);
        translate([-shaft_d, -shaft_d, -eps])
            cube([shaft_d + shaft_flat - (shaft_d - shaft_flat) / 2,
                  2 * shaft_d, shaft_len + 2 * eps]);
    }
    // driver PCB on the back
    color("#1d5c2f")
        translate([-driver_pcb / 2, -driver_pcb / 2, -motor_len - driver_depth])
            cube([driver_pcb, driver_pcb, driver_depth]);
    // OLED + buttons, on the edge that must stay reachable
    color("#101018")
        translate([-14, -driver_pcb / 2 - 0.6, -motor_len - driver_depth + 3])
            cube([28, 1.2, 9]);
}

// ---------------------------------------------------------------------------
// PART A - pan yoke
// ---------------------------------------------------------------------------
//
// Clamps the pan motor's D-shaft and presents a vertical face for the tilt
// motor. Print standing on the hub's bottom face (as modelled, +Z up): the
// clamp screw then runs parallel to the bed and squeezes across layer lines,
// and the vertical face needs no support. The arm overhang is bridged by the
// gusset.

// The hub sits above the base plate when one is fitted, so it starts clear of
// both the motor's boss and the 5 mm plate. Shaft is 20 mm, so 6 + 13 = 19 mm
// keeps the clamp fully on metal.
// The hub sits above the base plate when one is fitted, so it starts clear of
// both the motor's boss and the 5 mm plate. Shaft is 20 mm, so 6 + 13 = 19 mm
// keeps the clamp fully on metal.
pan_hub_z   = base_t + 1.0;
pan_hub_h   = 13.0;
pan_hub_od  = shaft_d + 2 * hub_wall + 3.0;

// The arm is an L, not a diagonal, and that is the whole trick. A diagonal
// spine would pass through the volume the camera cradle sweeps when it tilts
// down. Routing the arm as a low horizontal beam and then a vertical column
// hard against the tilt motor's face keeps every part of the yoke either
// below the cradle's swept circle or behind it in Y.
beam_z0     = pan_hub_z;          // top of the boss clearance
beam_h      = 12.0;               // must stay under the cradle's swept circle
col_w       = 26.0;
tilt_plate_w = 42.0;              // corner radius 29.7 from the tilt axis
tilt_plate_h = 42.0;

// Stiffening rib on the far side of the column. The space behind the column is
// free below the tilt motor, and the column is loaded in bending by the
// camera's moment - across its 5 mm thickness, its weakest direction. The rib
// turns that thin wall into a deep T-section for the price of no clearance.
rib_w       = 12.0;
rib_d       = 13.0;

module pan_yoke() {
    difference() {
        union() {
            // Shaft clamp.
            translate([0, 0, pan_hub_z]) cylinder(d = pan_hub_od, h = pan_hub_h);

            // Horizontal beam, blended into the hub with a hull rather than
            // butted against it: a square junction at the most loaded corner
            // of the part is where a printed bracket splits.
            hull() {
                translate([0, 0, beam_z0]) cylinder(d = pan_hub_od, h = beam_h);
                translate([-col_w / 2, tilt_face_y - rib_d, beam_z0])
                    cube([col_w, plate_t + rib_d, beam_h]);
            }

            // Column, tapering from the beam's width up to the motor face.
            // Printed with Z up this is what keeps the plate's wings off
            // support: they arrive gradually at about 10 deg from vertical
            // instead of appearing as an 8 mm ledge in mid-air.
            hull() {
                translate([-col_w / 2, tilt_face_y, beam_z0])
                    cube([col_w, plate_t, eps]);
                translate([-tilt_plate_w / 2, tilt_face_y,
                           tilt_axis_h - tilt_plate_h / 2])
                    cube([tilt_plate_w, plate_t, eps]);
            }

            // The mounting face.
            translate([-tilt_plate_w / 2, tilt_face_y, tilt_axis_h - tilt_plate_h / 2])
                cube([tilt_plate_w, plate_t, tilt_plate_h]);

            // Back rib, tapering out below the tilt motor so it never touches it.
            hull() {
                translate([-rib_w / 2, tilt_face_y - rib_d, beam_z0])
                    cube([rib_w, rib_d, beam_h]);
                translate([-rib_w / 2, tilt_face_y - eps,
                           tilt_axis_h - motor_body / 2 - 5])
                    cube([rib_w, eps, eps]);
            }
        }

        // Shaft bore, slit, clamp screw.
        translate([0, 0, pan_hub_z - eps]) {
            d_bore(pan_hub_h + 2 * eps);
            translate([-0.8, 0, 0]) cube([1.6, pan_hub_od, pan_hub_h + 2 * eps]);
        }
        translate([0, pan_hub_od / 2 - 1.0, pan_hub_z + pan_hub_h / 2])
            rotate([0, 90, 0]) {
                cylinder(d = m3_free, h = pan_hub_od + 4, center = true);
                translate([0, 0, pan_hub_od / 2 - 2.2]) cylinder(d = m3_head_d, h = 5);
                translate([0, 0, -pan_hub_od / 2 - 2.8]) rotate([0, 0, 30])
                    cylinder(d = m3_nut_af / cos(30), h = 5, $fn = 6);
            }

        // Tilt motor interface. The motor bolts to the *back* of this plate,
        // with the shaft passing through toward the cradle.
        translate([0, tilt_face_y - eps, tilt_axis_h]) rotate([-90, 0, 0]) {
            nema_bolt_holes(plate_t + 2 * eps);
            translate([0, 0, -eps])
                cylinder(d = motor_boss_d + fit_boss, h = plate_t + 4 * eps);
        }
    }
}

// ---------------------------------------------------------------------------
// PART B - camera cradle
// ---------------------------------------------------------------------------
//
// Clamps the tilt motor's D-shaft and carries the camera on a 1/4"-20 screw.
// Three screw positions along the platform let the camera be slid until it
// balances about the tilt axis, which is what keeps the motor from holding a
// static gravity torque (and cooking itself, given these boards hold full Ma
// in the fixed-current work modes).
//
// Print flat on the platform's underside, platform down. The clamp screw again
// runs parallel to the bed.

// Exposed shaft is shaft_len minus the plate the motor bolts through, so the
// hub plus the assembly gap has to fit inside that. 5 + 2 + 12 = 19 of 20 mm.
cradle_hub_h  = 12.0;
cradle_hub_od = shaft_d + 2 * hub_wall + 3.0;

// Everything below is measured in the cradle's own frame: +Z runs along the
// tilt shaft, away from the tilt motor. Keeping every feature at z >= 0 is what
// guarantees the cradle can never reach back into the yoke's column, which sits
// 4 mm behind the hub's origin.
plat_l        = 54.0;
plat_w        = 36.0;
plat_t        = 5.0;
// Distance from the tilt axis to the platform's mounting face. Kept small on
// purpose: the cradle sits entirely beyond the motor in Y, so it never has to
// clear the motor radially, and every millimetre here is camera mass held off
// the tilt axis as a standing gravity torque. These drivers hold full Ma
// continuously in the OPEN and CLOSE work modes, so an unbalanced axis is not
// just wasted torque, it is a motor that runs hot doing nothing.
plat_drop     = 12.0;
rib_h         = 4.0;

module camera_cradle() {
    difference() {
        union() {
            cylinder(d = cradle_hub_od, h = cradle_hub_h);

            // Web from hub down to the platform, spanning the hub's length.
            hull() {
                cylinder(d = cradle_hub_od, h = cradle_hub_h);
                translate([-plat_w / 2, -plat_drop - plat_t, 0])
                    cube([plat_w, plat_t, cradle_hub_h]);
            }

            // Platform, cantilevered forward so the camera sits over the pan
            // axis rather than behind it.
            translate([-plat_w / 2, -plat_drop - plat_t, 0])
                cube([plat_w, plat_t, plat_l]);

            // Ribs, on the underside only.
            for (s = [-1, 1])
                translate([s * (plat_w / 2 - wall), -plat_drop - plat_t - rib_h, 0])
                    cube([wall, plat_t + rib_h, plat_l]);
        }

        translate([0, 0, -eps]) {
            d_bore(cradle_hub_h + 2 * eps);
            translate([-0.8, 0, 0]) cube([1.6, cradle_hub_od, cradle_hub_h + 2 * eps]);
        }
        translate([0, cradle_hub_od / 2 - 1.0, cradle_hub_h / 2]) rotate([0, 90, 0]) {
            cylinder(d = m3_free, h = cradle_hub_od + 4, center = true);
            translate([0, 0, cradle_hub_od / 2 - 2.2]) cylinder(d = m3_head_d, h = 5);
            translate([0, 0, -cradle_hub_od / 2 - 2.8]) rotate([0, 0, 30])
                cylinder(d = m3_nut_af / cos(30), h = 5, $fn = 6);
        }

        // Camera screws along the platform, for balancing about the tilt axis.
        for (z = [22, 32, 42])
            translate([0, -plat_drop - plat_t - rib_h - eps, z])
                rotate([-90, 0, 0])
                    cylinder(d = cam_screw_d, h = plat_t + rib_h + 2 * eps);
    }
}

// ---------------------------------------------------------------------------
// PART C - base plate (optional)
// ---------------------------------------------------------------------------
//
// The pan motor cannot sit on a bench: its driver stands 16 mm proud of the
// back face and the terminals stand proud of that. This plate bolts to the
// motor's *front* face and stands it off on four legs, leaving the driver and
// its whole user interface in free air.
//
// Print flat, legs up, no support.


module base_plate() {
    difference() {
        union() {
            translate([-base_w / 2, -base_w / 2, 0])
                cube([base_w, base_w, base_t]);
            for (x = [-1, 1], y = [-1, 1])
                translate([x * (base_w / 2 - leg_d / 2 - 1),
                           y * (base_w / 2 - leg_d / 2 - 1), -leg_h])
                    cylinder(d = leg_d, h = leg_h + eps);
        }
        // Motor interface.
        translate([0, 0, -eps]) {
            nema_bolt_holes(base_t + 2 * eps);
            cylinder(d = motor_boss_d + fit_boss, h = base_t + 2 * eps);
        }
        // Countersinks so the motor sits flat on the plate's top surface.
        for (x = [-1, 1], y = [-1, 1])
            translate([x * motor_bolt_span / 2, y * motor_bolt_span / 2, -eps])
                cylinder(d = m3_head_d, h = 3.2);
        // Tripod screw and a pair of bench slots.
        translate([0, 0, -eps]) cylinder(d = cam_screw_d, h = base_t + 2 * eps);
        for (x = [-1, 1])
            translate([x * (base_w / 2 - 8), 0, -eps])
                cylinder(d = m3_free + 0.6, h = base_t + 2 * eps);
    }
}

// ---------------------------------------------------------------------------
// Assembly preview
// ---------------------------------------------------------------------------

module assembly(pan_deg = 0, tilt_deg = 0) {
    // Pan motor, shaft up, front face at z=0.
    nema17_with_driver();

    rotate([0, 0, pan_deg]) {
        color("#d8632a") pan_yoke();

        // Tilt motor bolted to the yoke face, shaft pointing back at the pan
        // axis (+Y in the yoke frame).
        translate([0, tilt_face_y, tilt_axis_h]) rotate([-90, 0, 0])
            nema17_with_driver();

        // Camera cradle on the tilt shaft.
        translate([0, tilt_face_y + plate_t + cradle_gap, tilt_axis_h])
            rotate([-90, 0, 0])
                rotate([0, 0, tilt_deg])
                    color("#2a7fd8") camera_cradle();
    }

    // Base plate under the pan motor.
    color("#555555") translate([0, 0, 0]) base_plate();
}

// ---------------------------------------------------------------------------

if (part == "A") pan_yoke();
else if (part == "B") camera_cradle();
else if (part == "C") base_plate();
else assembly(pan, tilt);
