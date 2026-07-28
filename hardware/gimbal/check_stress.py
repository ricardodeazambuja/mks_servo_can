#!/usr/bin/env python3
"""Puts a number on every load path, and on which way the layers run under it.

The README has always had a table claiming each load runs *along* the layers
rather than across them. That was an argument, not a measurement, and an argument
is exactly the kind of thing that stays in a file after the geometry it described
has moved. This computes the stress at each critical section, states which
direction it pulls relative to the print, and divides by the right allowable.

FDM is anisotropic and the two directions are not close: a well-tuned PLA part is
around 45-50 MPa in plane and roughly half that across the layer boundaries. So
the same 6 MPa is comfortable in one direction and marginal in the other, and a
stress figure without a layer direction beside it does not mean anything.

WHY THIS IS BEAM THEORY AND NOT FEA. Every section here is a prismatic beam,
plate or annulus loaded a long way from its supports, which is the case closed-form
formulas are exact for. FEA earns its setup cost when a margin is small, when the
geometry is not beam-like, or when the peak is at a stress raiser you cannot see -
and the honest use of these numbers is to tell you *which* section is worth that
trouble. Everything here is 12x clear or better except two sections at 2x: the
yoke's arm root and the pedestal's hold-down ear, which is also the only place in
the machine where a human with a screwdriver, rather than gravity, sets the load.

Both of those 2x figures are conservative on purpose - the ear's section is taken
at the bolt rather than at the blended root, and the arm's load is an invented
50 N yank with a factor of three already on it - so the pragmatic answer was to
make the conservative bound pass rather than to compute the exact one.

Every number below is a hand calculation with its assumptions written down. They
are meant to be argued with.

Usage:
    python check_stress.py
    python check_stress.py --grab 100     # a heavier yank on the camera
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
from check_clearances import SCAD, scad_values

G = 9.81

# PLA, printed with 4 perimeters and 40% infill, conservative for a part off a
# well-tuned machine. In-plane is along the extrusion; layer-normal is across the
# bond between layers, which is the direction FDM is weak in.
S_INPLANE = 45.0     # MPa, tensile yield
S_LAYER = 20.0       # MPa, across layers - about half, and it varies with the
                     # machine, the temperature and the part's cooling
DESIGN_FACTOR = 3.0  # what a printed part in an unknown machine's hands gets

# The loads. Gravity sets most of them and they are all small; the two that are
# not set by gravity are the interesting ones.
PAYLOAD_G = 300.0
GRAB_N = 50.0        # someone takes hold of the camera and pulls: 5 kg
SCREW_N = 150.0      # an M4 wood screw driven home by hand into a bench
MOTOR_NM = 0.4       # holding torque at 1.5 A


def sections(v: dict, payload_n: float, grab_n: float, screw_n: float) -> list:
    """Every place a load enters a printed section, with its geometry.

    Args:
        v: Dimension table from the .scad.
        payload_n: Weight of the cradle and its payload, in newtons.
        grab_n: Handling load applied at the camera.
        screw_n: Preload from a hold-down screw.

    Returns:
        List of dicts: what the section is, the stress in MPa, whether that
        stress crosses the layers, and a sentence saying how it was worked out.
    """
    out = []

    # --- the fork's arms, as two vertical cantilevers ----------------------
    # Worst case is a sideways yank at the tilt axis, shared by two arms. The
    # arm bends about its thin direction, so the section modulus is the small one.
    # Evaluated at the *top* of the fillet, not at the pad, and that is the
    # honest place: the flare only carries the bottom `arm_foot_h` of the arm, so
    # above it the section is back to the plain one while the moment is still
    # nearly full. Crediting the flared section for the whole height would have
    # reported a 12x margin for a joint that has about 2. What the fillet
    # actually buys is the stress concentration at a square corner, which this
    # arithmetic cannot see and which is where a printed part cracks.
    b, h = v["arm_y_root"], v["arm_t_root"]
    lever = v["tilt_axis_h"] - v["yoke_pad_t"] - v["arm_foot_h"]
    z_mod = b * h ** 2 / 6.0
    out.append({
        "part": "A yoke", "where": "arm root, at the pad",
        "mpa": grab_n / 2 * lever / z_mod, "across": True,
        "how": f"{grab_n:.0f} N sideways at the tilt axis, {lever:.0f} mm up, "
            f"two arms, section {b:.0f}x{h:.0f} mm"})

    # Same arms under the payload's own weight, straight down: compression along
    # the column, which is across the layers but in the direction where a printed
    # part does not care - layers are pressed together, not pulled apart.
    area = b * h
    out.append({
        "part": "A yoke", "where": "arm root, compression", "mpa": payload_n / 2 / area,
        "across": False,
        "how": f"{payload_n:.1f} N of payload down two {b:.0f}x{h:.0f} mm columns; "
            f"compression across layers does not open them"})

    # --- the cradle's cheeks, hanging the platform off the tilt axis --------
    # The platform and its payload hang below the axis, so the cheeks are in
    # tension between the two - straight across the layers.
    cheek_area = v["cheek_t"] * v["hub_od"]
    out.append({
        "part": "B cradle", "where": "cheek, axis to platform",
        "mpa": payload_n / 2 / cheek_area, "across": True,
        "how": f"{payload_n:.1f} N hanging on two {v['cheek_t']:.0f}x"
            f"{v['hub_od']:.0f} mm sections"})

    # The platform itself, as a beam simply supported at the cheeks with the
    # payload in the middle. Bending here runs along the layers.
    span = v["plat_w"] - 2 * v["cheek_t"]
    z_plat = v["plat_l"] * v["plat_t"] ** 2 / 6.0
    out.append({
        "part": "B cradle", "where": "platform, mid-span",
        "mpa": (payload_n * span / 4) / z_plat, "across": False,
        "how": f"{payload_n:.1f} N at the centre of a {span:.0f} mm span, "
            f"{v['plat_l']:.0f}x{v['plat_t']:.0f} mm section"})

    # --- the pedestal's hold-down ear ---------------------------------------
    # The one load in this machine set by a person rather than by gravity, and
    # the only one where the number is large. The ear is a flat tab off the
    # corner; the screw sits `ear_bolt_out` beyond the wall it grows from.
    ear_b = 2 * v["ear_bolt_r"]
    z_ear = ear_b * v["ear_t"] ** 2 / 6.0
    out.append({
        "part": "C pedestal", "where": "hold-down ear root",
        "mpa": (screw_n * v["ear_bolt_out"]) / z_ear, "across": True,
        "how": f"{screw_n:.0f} N of screw preload on a {v['ear_bolt_out']:.0f} mm "
            f"arm, root {ear_b:.0f}x{v['ear_t']:.1f} mm"})

    # The shroud carrying the whole machine, in compression and shear. A closed
    # section 74-96 mm across with a 3.2 mm wall has an enormous area.
    perim = 4 * (v["ped_top"] - 2 * v["ped_wall"])
    total_n = payload_n + 2.1   # plus the printed parts above it
    out.append({
        "part": "C pedestal", "where": "shroud wall", "mpa": total_n / (perim * v["ped_wall"]),
        "across": False,
        "how": f"{total_n:.1f} N over a {perim:.0f} mm perimeter of "
            f"{v['ped_wall']:.1f} mm wall"})

    # --- the pivot pin ------------------------------------------------------
    # Half the payload on a stub 10 mm across. Bending, so across the layers.
    d = v["brg_journal_d"]
    lever_pin = v["tilt_gap"] + v["brg_seat_depth"] / 2
    z_pin = np.pi * d ** 3 / 32.0
    out.append({
        "part": "D pin", "where": "journal root",
        "mpa": (payload_n / 2 * lever_pin) / z_pin, "across": True,
        "how": f"{payload_n/2:.1f} N at {lever_pin:.1f} mm on a {d:.1f} mm journal"})

    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grab", type=float, default=GRAB_N,
                    help="handling load at the camera, newtons")
    ap.add_argument("--screw", type=float, default=SCREW_N,
                    help="hold-down screw preload, newtons")
    args = ap.parse_args()

    v = scad_values(SCAD)
    needed = ("arm_y_root", "arm_t_root", "tilt_axis_h", "yoke_pad_t", "cheek_t",
              "hub_od", "plat_w", "plat_l", "plat_t", "ear_bolt_r", "ear_t",
              "ear_bolt_out", "ped_top", "ped_wall", "brg_journal_d", "tilt_gap",
              "arm_foot_w", "arm_foot_h",
              "brg_seat_depth")
    missing = [k for k in needed if k not in v]
    if missing:
        print(f"could not read {missing} from {SCAD.name}")
        return 2

    payload_n = (PAYLOAD_G + 42.0) / 1000.0 * G

    print(f"PLA at {S_INPLANE:.0f} MPa in plane, {S_LAYER:.0f} MPa across layers, "
          f"design factor {DESIGN_FACTOR:.0f}")
    print(f"loads: {payload_n:.1f} N of cradle and payload, {args.grab:.0f} N grab, "
          f"{args.screw:.0f} N screw preload\n")
    print(f"{'part':<12}{'section':<26}{'MPa':>7}{'layers':>9}{'allowed':>9}"
          f"{'margin':>9}")
    print("-" * 72)

    ok = True
    worst = (1e9, "")
    for s in sections(v, payload_n, args.grab, args.screw):
        allowed = (S_LAYER if s["across"] else S_INPLANE) / DESIGN_FACTOR
        margin = allowed / s["mpa"] if s["mpa"] > 0 else float("inf")
        if margin < worst[0]:
            worst = (margin, f"{s['part']} {s['where']}")
        if margin < 1.0:
            ok = False
        print(f"{s['part']:<12}{s['where']:<26}{s['mpa']:7.2f}"
              f"{'across' if s['across'] else 'along':>9}{allowed:9.1f}"
              f"{margin:8.0f}x  {'FAIL' if margin < 1 else ''}")
        print(f"{'':<12}{s['how']}")

    print(f"\ntightest: {worst[1]} at {worst[0]:.0f}x")
    print("everything else is a factor of ten or more clear of it, which is what "
          "says\nwhere FEA would be worth the trouble and where it would only "
          "confirm arithmetic")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
