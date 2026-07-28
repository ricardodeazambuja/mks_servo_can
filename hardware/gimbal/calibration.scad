// Two parts with known answers, for calibrating check_slicing.py.
//
// A check that only ever returns "fine" is indistinguishable from a check that
// is broken, and a slicer's stability detector is a black box that could change
// between releases. So there is a part it must complain about and a part it must
// not, and `check_slicing.py --calibrate` asks it both questions.
//
// They differ only in how the reach is made: the same 30 mm of overhang, once
// hanging in air and once drafted at 40 degrees from vertical.
which = "bad";   // "bad" needs support, "good" does not

if (which == "bad") {
    cube([10, 20, 20]);
    translate([10, 0, 18]) cube([30, 20, 2]);   // straight out into thin air
} else {
    hull() {
        cube([10, 20, 2]);
        translate([0, 0, 18]) cube([40, 20, 2]);
    }
}
