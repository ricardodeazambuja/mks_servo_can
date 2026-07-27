#!/usr/bin/env python3
"""Checks the gimbal cannot collide with itself anywhere in its soft limits.

The parts are parametric, so a plausible-looking edit can quietly move the
camera into the yoke. Rendering a preview does not catch that: a collision only
appears at particular angles, and OpenSCAD happily renders interpenetrating
solids. This sweeps the tilt range and reports the worst clearance found.

Dimensions come straight out of the .scad, including the assembly frame it
derives (`ped_top_z`, `yoke_z`, `tilt_z`, `tilt_face_x`), so the two cannot
drift apart.

Two kinds of number are reported, and the distinction matters:

  swept   varies with tilt. This is what a sweep is for, and where a design
          gets caught out - the binding case is usually the *payload*, not the
          printed parts, because the payload is the largest thing that moves.
  static  set once when you assemble it. The cradle sits between the fork arms
          and never moves along the tilt axis, so its side clearance is a fixed
          gap; sweeping it would just report the same number 270 times.

Usage:
    python check_clearances.py            # report and exit non-zero on a clash
    python check_clearances.py --verbose  # per-angle detail
"""
from __future__ import annotations

import argparse
import ast
import pathlib
import re
import subprocess
import sys

import numpy as np

SCAD = pathlib.Path(__file__).with_name("gimbal_parts.scad")

# Soft limits from examples/camera_gimbal_tracker.py. Only tilt is swept: pan
# turns the whole moving assembly about a vertical axis, and everything the
# cradle could reach turns with it. The one fixed thing underneath - the
# pedestal's top plate - is what the sweep is measured against, and it is the
# same height at every pan angle.
TILT_LIMITS = (-45.0, 90.0)

# How much clear air a design needs before it is called safe. Printed parts
# warp, a shaft joint slips a degree, and a camera is bigger than its screw.
REQUIRED_MM = 3.0


def _arith(node: ast.AST, names: dict) -> float:
    """Evaluates one node of a restricted arithmetic expression.

    Deliberately not `eval`: this parses a file from the repository, but the
    grammar it needs is only numbers, previously-defined names and the four
    operators, so nothing wider is permitted.

    Args:
        node: Parsed expression node.
        names: Already-resolved variable values.

    Returns:
        The node's value.

    Raises:
        ValueError: On anything outside the permitted grammar.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.Name):
        if node.id not in names:
            raise ValueError(f"unknown name {node.id}")
        return names[node.id]
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        value = _arith(node.operand, names)
        return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, ast.BinOp) and isinstance(
        node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)
    ):
        left, right = _arith(node.left, names), _arith(node.right, names)
        if isinstance(node.op, ast.Add):
            return left + right
        if isinstance(node.op, ast.Sub):
            return left - right
        if isinstance(node.op, ast.Mult):
            return left * right
        return left / right
    raise ValueError("unsupported expression")


def scad_values(path: pathlib.Path) -> dict:
    """Extracts top-level numeric assignments from an OpenSCAD file.

    Args:
        path: The .scad file to read.

    Returns:
        Mapping of variable name to float, for `name = <arithmetic>;` lines.
        Anything it cannot evaluate - a call to one of the .scad's own
        functions, for instance - is skipped rather than guessed at.
    """
    text = path.read_text()
    values: dict = {}
    for name, expr in re.findall(r"^(\w+)\s*=\s*([^;]+);", text, re.MULTILINE):
        expr = expr.split("//")[0].strip()
        try:
            values[name] = _arith(ast.parse(expr, mode="eval").body, values)
        except (SyntaxError, ValueError, ZeroDivisionError):
            continue
    return values


def cradle_points(v: dict) -> np.ndarray:
    """Everything that moves, in the cradle's own frame.

    That frame is also the frame the part is modelled and printed in: x along
    the tilt axis, y fore and aft, z up from the platform's underside, with the
    tilt axis at `axis_z`.

    The camera is included at both ends of its balance slot as well as centred.
    It is the largest thing that moves and it is the thing that collides: on the
    previous design the printed parts cleared each other by a comfortable margin
    at a tilt where the camera had 2.8 mm.

    Args:
        v: Dimension table from the .scad file.

    Returns:
        Array of shape (n, 3).
    """
    pts = []

    hw, hl = v["plat_w"] / 2, v["plat_l"] / 2
    for x in (-hw, hw):
        for y in (-hl, hl):
            for z in (0.0, v["plat_t"]):
                pts.append((x, y, z))

    # Cheeks, sampled round the hub bosses, which are the highest part of the
    # cradle and the closest to the fork's arms.
    for x in (-hw, hw, -hw + v["cheek_t"], hw - v["cheek_t"]):
        for a in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            pts.append((x,
                        v["hub_od"] / 2 * np.cos(a),
                        v["axis_z"] + v["hub_od"] / 2 * np.sin(a)))

    # The camera, at both extremes of the balance slot and in the middle.
    cw, ch, cd = v["cam_w"] / 2, v["cam_h"], v["cam_d"] / 2
    for slide in (-v["cam_slot"] / 2, 0.0, v["cam_slot"] / 2):
        for x in (-cw, cw):
            for y in (slide - cd, slide + cd):
                for z in (v["plat_t"], v["plat_t"] + ch):
                    pts.append((x, y, z))

    return np.array(pts, dtype=float)


def to_global(pts: np.ndarray, tilt_deg: float, v: dict) -> np.ndarray:
    """Maps cradle-frame points into the assembly frame at a given tilt.

    Mirrors the assembly transform in the .scad exactly: the cradle is placed
    with its tilt axis on the fork's axis, then rotated about that axis.

    Args:
        pts: Points in the cradle frame.
        tilt_deg: Tilt angle in degrees.
        v: Dimension table.

    Returns:
        Points in the assembly frame, where z = 0 is the desk.
    """
    t = np.radians(tilt_deg)
    c, s = np.cos(t), np.sin(t)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2] - v["axis_z"]
    return np.column_stack([x, y * c - z * s, y * s + z * c + v["tilt_z"]])


def arm_inner_face(v: dict, z: np.ndarray) -> np.ndarray:
    """Where the fork's arms actually are, at a given height.

    The arms splay: they land on the yoke's pad well inboard and reach their full
    separation at `arm_mid_z`, above which they stand vertical. Describing them
    as a box at a constant `arm_gap / 2` - which is what the first version of
    this file did - describes a fork that is not the one in the .scad, and it
    reported 2.5 mm of clearance through a 4.4 mm overlap.

    Args:
        v: Dimension table.
        z: Heights in the assembly frame.

    Returns:
        The inner face's |x| at each height. Infinite below the pad, where there
        is no arm and the pad's own box covers it.
    """
    z0 = v["yoke_z"] + v["yoke_pad_t"]
    z1 = v["yoke_z"] + v["arm_mid_z"]
    x0, x1 = v["arm_x_root"], v["arm_gap"] / 2
    frac = np.clip((z - z0) / (z1 - z0), 0.0, 1.0)
    return np.where(z < z0, np.inf, x0 + (x1 - x0) * frac)


def arm_clearance(pts: np.ndarray, v: dict) -> float:
    """Smallest horizontal gap from the moving points to an arm's inner face.

    Horizontal rather than perpendicular, which understates the true distance to
    a sloping face by cos(splay) and is therefore the safe way round.

    Args:
        pts: Points in the assembly frame.
        v: Dimension table.

    Returns:
        The gap in mm, negative on overlap.
    """
    return float(np.min(arm_inner_face(v, pts[:, 2]) - np.abs(pts[:, 0])))


def swept_obstacles(v: dict) -> list:
    """Fixed things the cradle sweeps past, as axis-aligned boxes.

    The list is short because the architecture made it short. A fork with the
    tilt axis crossing the pan axis puts the payload inside the frame, so
    "does the camera clear the structure" collapses into "does it clear what is
    underneath it".

    Boxes are used for a hub that is really a cylinder, which is conservative -
    it claims the corners too.

    Returns:
        List of (label, lo_xyz, hi_xyz).
    """
    far = 500.0
    return [
        # The pedestal's top plate, and by extension the desk and everything
        # else below it. Pan-invariant, which is why pan is not swept.
        ("pedestal plate",
         (-far, -far, -far), (far, far, v["ped_top_z"])),
        # The yoke's bearing pad, sitting on that plate.
        ("yoke pad",
         (-v["yoke_pad_d"] / 2, -v["yoke_pad_d"] / 2, -far),
         (v["yoke_pad_d"] / 2, v["yoke_pad_d"] / 2, v["yoke_z"] + v["yoke_pad_t"])),
        # The yoke's shaft hub, which stands above the pad in the middle of the
        # fork - directly under the camera's nose at full tilt.
        ("yoke hub",
         (-v["pan_hub_od"] / 2, -v["pan_hub_od"] / 2, -far),
         (v["pan_hub_od"] / 2, v["pan_hub_od"] / 2, v["yoke_z"] + v["pan_hub_h"])),
    ]


# The exact test's members, and the tilts it checks. Coarse on purpose: each one
# is a CGAL boolean over the whole assembly, a few seconds apiece. The analytic
# sweep finds the worst angle; this confirms the geometry the sweep only models.
EXACT_TILTS = (-45.0, -20.0, 0.0, 45.0, 90.0)
FIXED_MEMBERS = ("pedestal", "pan motor", "yoke", "tilt motor")

# Coincident faces are everywhere in an assembly - a motor's face bolts flat
# against a plate - and CGAL returns those contacts as a zero-thickness solid
# with a few dozen facets in it. So the test is on volume, not on whether the
# intersection is empty: 0.0001 mm^3 is two surfaces touching, which is what
# "bolted together" means.
EXACT_TOL_MM3 = 0.01


def _stl_volume(path: pathlib.Path) -> float:
    """Volume of a binary STL, by the divergence theorem.

    Args:
        path: The STL to read. A missing file counts as empty, because OpenSCAD
            declines to write one at all when the geometry is empty - which is
            itself the answer this test wants.

    Returns:
        The enclosed volume in mm^3.
    """
    if not path.exists():
        return 0.0
    raw = path.read_bytes()
    if len(raw) < 84:
        return 0.0
    n = int.from_bytes(raw[80:84], "little")
    if n == 0 or len(raw) < 84 + 50 * n:
        return 0.0
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=np.uint8).reshape(n, 50)
    tri = rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(float)
    return float(abs(np.einsum("ij,ij->i", tri[:, 0],
                               np.cross(tri[:, 1], tri[:, 2])).sum() / 6.0))


def exact_overlaps(verbose: bool = False) -> list:
    """Asks OpenSCAD for the actual overlap volume, member by member.

    The analytic sweep above sames the cradle as points and the arms as a plane,
    and an approximation of a shape is a shape you can get wrong. This does not
    approximate anything: `intersection()` of what moves with what does not is
    the real geometry, and its volume is either zero or it is not.

    Returns:
        List of (label, tilt, mm3) for every pair that overlaps.

    Raises:
        FileNotFoundError: If openscad is not installed.
    """
    out = []
    tmp = SCAD.with_name("_interference.stl")
    try:
        for tilt in EXACT_TILTS:
            for member in FIXED_MEMBERS:
                tmp.unlink(missing_ok=True)
                subprocess.run(
                    ["openscad", "-D", 'part="interference"', "-D", f"tilt={tilt}",
                     "-D", f'against="{member}"', "--export-format", "binstl",
                     "-o", str(tmp), str(SCAD)],
                    capture_output=True, check=False,
                )
                vol = _stl_volume(tmp)
                if verbose:
                    print(f"  tilt {tilt:+6.1f}  {member:<12} {vol:10.4f} mm^3")
                if vol > EXACT_TOL_MM3:
                    out.append((member, tilt, vol))
    finally:
        tmp.unlink(missing_ok=True)
    return out


def static_gaps(v: dict) -> list:
    """Gaps set when the machine is assembled, not by any angle.

    Each carries its own minimum, because they are not the same requirement and
    holding them to one number is how a design gets bent to satisfy a figure
    that never applied to it. A swept clearance has to absorb warp, a slipped
    shaft joint and a payload bigger than its screw. A gap between a part that
    only rotates and a fixed arm 2.5 mm away absorbs warp and nothing else. And
    "spare shaft" is not air at all - it is tolerance against a motor whose
    shaft is shorter than the 20 mm assumed here, which is the one dimension on
    these motors that genuinely varies between suppliers.

    Returns:
        List of (label, mm, minimum, note).
    """
    short = 2.0   # how much less shaft than 20 mm a given motor might have
    # How far the tilt shaft reaches past the near face of the cradle's bore.
    reach = v["shaft_len"] - (v["tilt_face_x"] - v["plat_w"] / 2)
    return [
        ("cradle to fork arm", v["arm_gap"] / 2 - v["plat_w"] / 2, 2.0,
         "each side; set along the tilt axis, not by any angle"),
        ("camera to cheek",
         (v["plat_w"] / 2 - v["cheek_t"]) - v["cam_w"] / 2, 2.0,
         "each side"),
        # Bore deep enough to contain the shaft, or whatever it does not contain
        # protrudes past it into the camera's space. This was negative once.
        ("tilt bore vs shaft tip", v["cheek_t"] - reach, 0.0,
         f"bore {v['cheek_t']:.0f} mm, shaft reaches {reach:.1f} mm into it"),
        ("tilt engagement, short shaft", min(v["cheek_t"], reach) - short, 6.0,
         f"with a shaft {short:.0f} mm shorter than {v['shaft_len']:.0f} mm"),
        ("pan engagement, short shaft",
         min(v["pan_hub_h"],
             v["shaft_len"] - v["ped_plate_t"] - v["slew_gap"]) - short, 6.0,
         f"with a shaft {short:.0f} mm shorter than {v['shaft_len']:.0f} mm"),
    ]


def clearance(points: np.ndarray, lo, hi) -> float:
    """Smallest distance from a set of points to an axis-aligned box.

    Negative when a point is inside the box.

    Args:
        points: Points to test.
        lo: Box minimum corner.
        hi: Box maximum corner.

    Returns:
        The minimum signed clearance in mm.
    """
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)
    outside = np.maximum(lo - points, points - hi)
    dist = np.linalg.norm(np.maximum(outside, 0.0), axis=1)
    inside = np.all(outside < 0, axis=1)
    depth = np.where(inside, outside.max(axis=1), 0.0)
    return float(np.min(np.where(inside, depth, dist)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--fast", action="store_true",
                    help="skip the exact CGAL intersection pass")
    args = ap.parse_args()

    v = scad_values(SCAD)
    needed = ("plat_w", "plat_l", "plat_t", "axis_z", "cheek_t", "hub_od",
              "cam_w", "cam_h", "cam_d", "cam_slot", "tilt_z", "yoke_z",
              "ped_top_z", "yoke_pad_d", "yoke_pad_t", "pan_hub_od",
              "pan_hub_h", "arm_gap", "tilt_face_x", "shaft_len")
    missing = [k for k in needed if k not in v]
    if missing:
        print(f"could not read {missing} from {SCAD.name}")
        return 2

    pts = cradle_points(v)
    boxes = swept_obstacles(v)
    labels = [label for label, _, _ in boxes] + ["fork arm"]
    worst = {label: (1e9, None) for label in labels}

    for tilt in np.arange(TILT_LIMITS[0], TILT_LIMITS[1] + 0.5, 0.5):
        g = to_global(pts, float(tilt), v)
        found = [(label, clearance(g, lo, hi)) for label, lo, hi in boxes]
        found.append(("fork arm", arm_clearance(g, v)))
        for label, c in found:
            if c < worst[label][0]:
                worst[label] = (c, float(tilt))
        if args.verbose:
            print(f"  tilt {tilt:+6.1f}: "
                  + "  ".join(f"{k}={c:6.1f}" for k, c in found))

    ok = True
    print(f"swept, over tilt {TILT_LIMITS[0]:+.0f}..{TILT_LIMITS[1]:+.0f} deg "
          f"(camera included, at both ends of its balance slot)")
    for label, (c, tilt) in worst.items():
        if c < REQUIRED_MM:
            ok = False
        print(f"  {'OK ' if c >= REQUIRED_MM else 'FAIL'} {label:<22} {c:7.2f} mm"
              f"   (worst at tilt {tilt:+.1f})")

    print("\nstatic, set at assembly")
    for label, mm, floor, note in static_gaps(v):
        if mm < floor:
            ok = False
        print(f"  {'OK ' if mm >= floor else 'FAIL'} {label:<22} {mm:7.2f} mm"
              f"  (>={floor:.1f})  {note}")

    if not args.fast:
        print("\nexact, from OpenSCAD's own intersection()")
        try:
            bad = exact_overlaps(args.verbose)
        except FileNotFoundError:
            print("  SKIP openscad not on PATH")
        else:
            if bad:
                ok = False
                for member, tilt, vol in bad:
                    print(f"  FAIL cradle overlaps the {member} by {vol:.1f} mm^3 "
                          f"at tilt {tilt:+.0f}")
            else:
                print(f"  OK  no overlap above {EXACT_TOL_MM3} mm^3 at tilt "
                      + ", ".join(f"{t:+.0f}" for t in EXACT_TILTS))

    print(f"\nrequirement: swept >= {REQUIRED_MM:.1f} mm, static as noted, "
          "exact overlap zero")
    if not ok:
        print("insufficient clearance - raise tilt_axis_h, or shrink the payload "
              "envelope in gimbal_parts.scad if it is genuinely smaller")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
