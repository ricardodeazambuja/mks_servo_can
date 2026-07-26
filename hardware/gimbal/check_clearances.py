#!/usr/bin/env python3
"""Checks the gimbal cannot collide with itself anywhere in its soft limits.

The parts are parametric, so a plausible-looking edit can quietly move the
camera cradle into the yoke. Rendering a preview does not catch that: a
collision only appears at particular pan/tilt angles, and OpenSCAD happily
renders interpenetrating solids.

This sweeps the full soft-limit range and reports the worst clearance found
between the moving cradle and every fixed obstacle. It reads the dimensions
straight out of the .scad file so the two cannot drift apart.

Usage:
    python check_clearances.py            # report and exit non-zero on a clash
    python check_clearances.py --verbose  # per-angle detail
"""
from __future__ import annotations

import argparse
import ast
import pathlib
import re
import sys

import numpy as np

SCAD = pathlib.Path(__file__).with_name("gimbal_parts.scad")

# Soft limits from examples/camera_gimbal_tracker.py. Tilt is what matters:
# pan just rotates the whole moving assembly about a vertical axis, so it
# cannot bring the cradle nearer to anything that turns with it.
TILT_LIMITS = (-45.0, 90.0)

# How much clear air a design needs before it is called safe. Printed parts
# warp, clamps slip a degree, and a camera is bigger than its mounting screw.
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
        Anything it cannot evaluate is skipped rather than guessed at.
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
    """Corner points of the camera cradle, in the cradle's own frame.

    +Z runs along the tilt shaft away from the motor; the platform hangs at
    -Y. Only the extreme corners matter for a convex-ish part.

    Args:
        v: Dimension table from the .scad file.

    Returns:
        Array of shape (n, 3).
    """
    hw = v["plat_w"] / 2
    y_top = -v["plat_drop"] - v["plat_t"]
    y_bot = y_top - v["rib_h"]
    z0, z1 = 0.0, v["plat_l"]
    hub_r = v["cradle_hub_od"] / 2
    pts = [
        (x, y, z)
        for x in (-hw, hw)
        for y in (y_top, y_bot)
        for z in (z0, z1)
    ]
    # Hub shell, sampled around its circumference.
    for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        for z in (0.0, v["cradle_hub_h"]):
            pts.append((hub_r * np.cos(a), hub_r * np.sin(a), z))

    # A representative camera. The gimbal is useless if the parts clear each
    # other but the payload does not, and the payload is the largest thing that
    # moves. Sits on the platform's inner face and straddles the tilt axis.
    cam_w, cam_h, cam_d = 52.0, 44.0, 34.0
    y0 = -v["plat_drop"]
    for x in (-cam_w / 2, cam_w / 2):
        for y in (y0, y0 + cam_h):
            for z in (14.0, 14.0 + cam_d):
                pts.append((x, y, z))
    return np.array(pts, dtype=float)


def to_global(pts: np.ndarray, tilt_deg: float, v: dict) -> np.ndarray:
    """Maps cradle-frame points into the gimbal frame at a given tilt.

    Mirrors the assembly transform in the .scad file exactly:
    `translate([0, ty, tz]) rotate([-90,0,0]) rotate([0,0,tilt])`.

    Args:
        pts: Points in the cradle frame.
        tilt_deg: Tilt angle in degrees.
        v: Dimension table.

    Returns:
        Points in the gimbal frame, same shape.
    """
    t = np.radians(tilt_deg)
    c, s = np.cos(t), np.sin(t)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # rotate about local Z
    x1, y1, z1 = x * c - y * s, x * s + y * c, z
    # rotate([-90,0,0]) : (x,y,z) -> (x, z, -y)
    gx, gy, gz = x1, z1, -y1
    ty = v["tilt_face_y"] + v["plate_t"] + 4.0
    return np.column_stack([gx, gy + ty, gz + v["tilt_axis_h"]])


def obstacles(v: dict) -> list:
    """Axis-aligned boxes the cradle must stay out of.

    Returns:
        List of (label, lo_xyz, hi_xyz).
    """
    col_hw = v["col_w"] / 2
    plate_hw = v["tilt_plate_w"] / 2
    base_hw = v["base_w"] / 2
    return [
        # Low horizontal beam of the yoke, from the pan hub out to the column.
        ("yoke beam",
         (-col_hw, v["tilt_face_y"], v["beam_z0"]),
         (col_hw, v["pan_hub_od"] / 2, v["beam_z0"] + v["beam_h"])),
        # Vertical column and the tilt motor's mounting face.
        ("yoke column",
         (-plate_hw, v["tilt_face_y"], v["beam_z0"]),
         (plate_hw, v["tilt_face_y"] + v["plate_t"],
          v["tilt_axis_h"] + v["tilt_plate_h"] / 2)),
        # Tilt motor body and its driver, hanging off the far side of the face.
        ("tilt motor + driver",
         (-v["driver_pcb"] / 2,
          v["tilt_face_y"] - v["motor_len"] - v["driver_depth"],
          v["tilt_axis_h"] - v["driver_pcb"] / 2),
         (v["driver_pcb"] / 2, v["tilt_face_y"],
          v["tilt_axis_h"] + v["driver_pcb"] / 2)),
        # Base plate and everything below it.
        ("base plate",
         (-base_hw, -base_hw, -200.0),
         (base_hw, base_hw, v["base_t"])),
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
    # Per-axis signed distance outside the slab; negative inside.
    outside = np.maximum(lo - points, points - hi)
    dist = np.linalg.norm(np.maximum(outside, 0.0), axis=1)
    inside = np.all(outside < 0, axis=1)
    depth = np.where(inside, outside.max(axis=1), 0.0)
    return float(np.min(np.where(inside, depth, dist)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    v = scad_values(SCAD)
    missing = [k for k in ("plat_w", "plat_drop", "tilt_axis_h", "beam_h", "col_w")
               if k not in v]
    if missing:
        print(f"could not read {missing} from {SCAD.name}")
        return 2

    pts = cradle_points(v)
    boxes = obstacles(v)
    worst = {label: (1e9, None) for label, _, _ in boxes}

    for tilt in np.arange(TILT_LIMITS[0], TILT_LIMITS[1] + 0.5, 0.5):
        g = to_global(pts, float(tilt), v)
        for label, lo, hi in boxes:
            c = clearance(g, lo, hi)
            if c < worst[label][0]:
                worst[label] = (c, float(tilt))
        if args.verbose:
            print(f"  tilt {tilt:+6.1f}: " + "  ".join(
                f"{label}={clearance(g, lo, hi):6.1f}" for label, lo, hi in boxes))

    print(f"cradle clearance over tilt {TILT_LIMITS[0]:+.0f}..{TILT_LIMITS[1]:+.0f} deg")
    ok = True
    for label, (c, tilt) in worst.items():
        flag = "OK " if c >= REQUIRED_MM else "FAIL"
        if c < REQUIRED_MM:
            ok = False
        print(f"  {flag} {label:<22} {c:7.2f} mm   (worst at tilt {tilt:+.1f})")

    print(f"\nrequirement: >= {REQUIRED_MM:.1f} mm everywhere")
    if not ok:
        print("COLLISION or insufficient clearance - adjust plat_drop, beam_h "
              "or tilt_axis_h in gimbal_parts.scad")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
