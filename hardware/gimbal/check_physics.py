#!/usr/bin/env python3
"""Measures the things gravity decides, off the exported STLs.

The other two checkers ask whether the parts can be made and whether they can
touch each other. This one asks what happens once they are made and something
heavy is bolted to them, because two of this design's dimensions exist only for
that reason and neither is checked anywhere else:

  * `axis_z` puts the tilt axis at the payload's centre of mass rather than under
    it. Get it wrong and a stepper holds a standing torque all day - which on
    these drivers is not merely wasted torque but a motor running at full `Ma`
    doing nothing, in a printed part whose glass transition is 60 C.
  * the pedestal's footprint has to keep a tall, top-heavy thing upright.

Both are properties of the *whole assembly* including its payload, so neither can
be read off a dimension in the .scad. They are computed here from the real mesh:
volume and centroid by the divergence theorem, which needs no mesh library and no
assumptions about how the part is modelled.

The gates are the design's stated intents, not round numbers:

  * the reference payload has to balance on the tilt axis to within a couple of
    millimetres - that is what `axis_z` is *for*;
  * an *empty* cradle has to come out bottom-heavy, so that a build with no power
    hangs level rather than flopping over;
  * the loaded machine has to stay up when the desk is knocked.

Usage:
    python check_physics.py
    python check_physics.py --payload 450   # grams, if yours is not 300
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
from stl import mesh

from check_clearances import SCAD, scad_values, warn_if_stale

HERE = pathlib.Path(__file__).parent

# PLA at 100% infill. Everything below scales linearly in this, and the numbers
# that matter are ratios of masses rather than masses, so the difference between
# PLA and PETG (1.27) changes nothing that is being tested.
RHO = 1.24e-3   # g/mm^3
G = 9.81

# The payload this design is dimensioned around: a compact mirrorless body, and
# deliberately heavier than the webcam or Pi camera most builds carry.
REF_PAYLOAD_G = 300.0

# How far the reference payload's centre of mass may sit from the tilt axis.
BALANCE_TOL_MM = 2.0

# Desk tilt at which the loaded machine would go over, on its rubber feet.
TIP_MIN_DEG = 25.0


def props(path: pathlib.Path) -> tuple:
    """Volume and centroid of a closed triangle mesh.

    Args:
        path: An STL file.

    Returns:
        `(volume_mm3, centroid_xyz)`, by summing the signed volumes of the
        tetrahedra from the origin to each facet.
    """
    tri = mesh.Mesh.from_file(str(path)).vectors.astype(float)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    centroid = ((a + b + c) / 4.0 * v6[:, None]).sum(axis=0) / v6.sum()
    return abs(v6.sum() / 6.0), centroid


def tilt_balance(v: dict, cradle_g: float, cradle_cg: float,
                 payload_g: float) -> tuple:
    """Where the tilt axis' centre of mass sits, and what that costs.

    The payload is treated as a uniform box of the envelope in the .scad, which is
    the same assumption the clearance sweep makes about it.

    Args:
        v: Dimension table from the .scad.
        cradle_g: The cradle's mass.
        cradle_cg: Its centre of mass, above the platform's underside.
        payload_g: Payload mass in grams.

    Returns:
        `(offset_mm, torque_nmm)` - the centre of mass' height above the tilt
        axis, signed, and the standing torque that offset produces.
    """
    payload_cg = v["plat_t"] + v["cam_h"] / 2
    total = cradle_g + payload_g
    cg = (cradle_g * cradle_cg + payload_g * payload_cg) / total
    offset = cg - v["axis_z"]
    return offset, total / 1000.0 * G * abs(offset)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--payload", type=float, default=REF_PAYLOAD_G,
                    help="payload mass in grams")
    args = ap.parse_args()

    v = scad_values(SCAD)
    needed = ("plat_t", "cam_h", "cam_w", "cam_d", "axis_z", "ped_h", "slew_gap",
              "tilt_axis_h", "ped_foot", "ped_foot_r", "ear_foot_inset",
              "tilt_face_x", "motor_len", "driver_depth")
    missing = [k for k in needed if k not in v]
    if missing:
        print(f"could not read {missing} from {SCAD.name}")
        return 2

    parts = {}
    warn_if_stale((HERE / "stl").glob("*.stl"))
    print(f"{'part':<16}{'volume':>10}{'mass':>9}{'centroid z':>13}")
    print("-" * 48)
    for name in ("pan_yoke", "camera_cradle", "pedestal", "pivot_pin"):
        path = HERE / "stl" / f"{name}.stl"
        if not path.exists():
            print(f"{name:<16}  missing - run the openscad export first")
            return 2
        vol, cen = props(path)
        parts[name] = (vol * RHO, cen)
        print(f"{name:<16}{vol/1000:8.2f} cm^3{vol*RHO:7.1f} g{cen[2]:11.2f} mm")
    printed = sum(m for m, _ in parts.values())
    print(f"{'printed total':<16}{'':>10}{printed:7.1f} g")

    ok = True
    cradle_g, cradle_cen = parts["camera_cradle"]
    cradle_cg = float(cradle_cen[2])

    print(f"\nbalance about the tilt axis (axis_z = {v['axis_z']:.0f} mm, "
          f"cradle's own CG {cradle_cg:.1f} mm)")
    for payload in (0.0, 150.0, args.payload, 600.0):
        offset, torque = tilt_balance(v, cradle_g, cradle_cg, payload)
        where = "below" if offset < 0 else "above"
        note = ""
        if payload == 0.0:
            # An unpowered axis should hang level, not flop: that needs the bare
            # cradle to be bottom-heavy, which is not automatic - it is why the
            # axis sits at the payload's centre of mass and not above it.
            note = "  bare cradle must be bottom-heavy"
            if offset >= 0:
                ok = False
        elif payload == args.payload and abs(offset) > BALANCE_TOL_MM:
            ok = False
            note = f"  over the {BALANCE_TOL_MM:.0f} mm the design claims"
        flag = "FAIL" if note.startswith("  over") or (payload == 0 and offset >= 0) \
            else "OK "
        print(f"  {flag} payload {payload:5.0f} g: CG {abs(offset):5.2f} mm "
              f"{where} the axis, {torque:5.1f} N.mm standing torque{note}")

    print("\nstanding on a desk")
    # The pedestal is modelled upside down from how it stands, so its centroid
    # height has to be read back the other way up - the one place in this file
    # where the print-orientation invariant has to be undone.
    ped_g, ped_cen = parts["pedestal"]
    yoke_g, yoke_cen = parts["pan_yoke"]
    yoke_z = v["ped_h"] + v["slew_gap"]
    tilt_z = yoke_z + v["tilt_axis_h"]
    # Radius of the circle through the four rubber feet.
    foot_r = ((v["ped_foot"] / 2 - v["ped_foot_r"]) * np.sqrt(2)
              + v["ped_foot_r"] - v["ear_foot_inset"])
    for payload in (0.0, args.payload, 600.0):
        offset, _ = tilt_balance(v, cradle_g, cradle_cg, payload)
        items = [(ped_g, v["ped_h"] - float(ped_cen[2])),
                 (yoke_g, yoke_z + float(yoke_cen[2])),
                 (cradle_g + payload, tilt_z + offset)]
        total = sum(m for m, _ in items)
        cg = sum(m * z for m, z in items) / total
        tip = np.degrees(np.arctan(foot_r / cg))
        # Only the reference payload is a requirement. The 600 g row is there to
        # show which way the number moves, not to hold the design to a payload it
        # is not dimensioned for - and it is the row someone fitting a heavier
        # camera should read before deciding whether to use the M4 ears.
        gated = payload == args.payload
        good = tip >= TIP_MIN_DEG or not gated
        ok = ok and good
        flag = ("OK " if good else "FAIL") if gated else "   "
        print(f"  {flag} payload {payload:5.0f} g: "
              f"{total/1000:.2f} kg, CG {cg:5.1f} mm up, feet at r = {foot_r:.0f} mm "
              f"-> tips at {tip:.0f} deg")

    reach = v["tilt_face_x"] + v["motor_len"] + v["driver_depth"]
    print(f"\nswept volume: the tilt motor's driver reaches {reach:.0f} mm from the "
          f"pan axis\n              -> leave a {2*reach:.0f} mm circle clear")
    print(f"\nrequirement: reference payload balances within {BALANCE_TOL_MM:.0f} mm, "
          f"bare cradle bottom-heavy,\n             loaded machine tips no sooner "
          f"than {TIP_MIN_DEG:.0f} deg")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
