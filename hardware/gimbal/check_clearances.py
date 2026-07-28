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
import os
import pathlib
import re
import subprocess
import sys
from concurrent import futures

import numpy as np

SCAD = pathlib.Path(__file__).with_name("gimbal_parts.scad")


def _openscad() -> str:
    """Picks an OpenSCAD, preferring one with the Manifold backend.

    2021.01 is the last tagged stable and is CGAL-only; on this design a single
    boolean takes 20-55 s there and 0.2-0.6 s on a nightly, which is the difference
    between a check that gets run and one that does not. Measured on all four
    parts and on four booleans, the two agree on volume to 1e-5 % - Manifold just
    emits ~12 % fewer facets, having produced fewer degenerate slivers.

    One difference that does *not* matter, because of a decision made earlier: for
    a zero-volume result the two disagree about whether an STL gets written at all,
    in both directions. Every test here measures volume rather than asking whether
    the file exists, so both answers read as "no overlap".

    Returns:
        The binary to run. `$OPENSCAD` wins if set; otherwise a nightly AppImage
        in ~/.local/bin if one is there; otherwise whatever is on PATH.
    """
    override = os.environ.get("OPENSCAD")
    if override:
        return override
    nightly = pathlib.Path.home() / ".local/bin/openscad-nightly.AppImage"
    return str(nightly) if nightly.exists() else "openscad"


OPENSCAD = _openscad()

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


def warn_if_stale(paths) -> bool:
    """Warns when an exported STL is older than the model it came from.

    The two checkers that measure meshes rather than dimensions are only as
    truthful as the last export, and the failure is silent in the worst way: the
    numbers all look plausible, they are just describing the previous design. This
    only warns, because a fresh clone hands every file the same timestamp and a
    hard failure there would be noise.

    Args:
        paths: STL paths to check.

    Returns:
        True if anything looked stale.
    """
    stale = [p for p in paths if p.exists()
             and p.stat().st_mtime < SCAD.stat().st_mtime]
    for p in stale:
        print(f"  NOTE {p.name} is older than {SCAD.name} - re-export before "
              f"trusting these numbers")
    return bool(stale)


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
    # cradle and the closest to the fork's arms. The two hubs are different
    # diameters - the pivot side has to hold a 15 mm bearing - so each is sampled
    # at its own, rather than both at the smaller one.
    for x, hub in ((hw, v["hub_od"]), (hw - v["cheek_t"], v["hub_od"]),
                   (-hw, v["pivot_hub_od"]),
                   (-hw + v["cheek_t"], v["pivot_hub_od"])):
        for a in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            pts.append((x,
                        hub / 2 * np.cos(a),
                        v["axis_z"] + hub / 2 * np.sin(a)))

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


# The exact test's members, and the poses it checks. Coarse on purpose: each one
# is a CGAL boolean over the whole assembly, a few seconds apiece. The analytic
# sweep finds the worst angle; this confirms the geometry the sweep only models.
FIXED_MEMBERS = ("pedestal", "pan motor", "yoke", "tilt motor", "pivot pin")

# Pan is swept for the pedestal alone, and that is not laziness. Everything else
# in FIXED_MEMBERS is bolted to the yoke, so it turns *with* the cradle and their
# relative geometry is pan-invariant. The pedestal does not: its top plate is a
# rounded square, so its corners pass under the cradle's nose at 45 deg and not at
# 0. The analytic sweep covers that with a half-space at the plate's height, which
# is conservative but says nothing about the corners themselves.
#
# Every 15 degrees rather than at five hand-picked angles, because that is what
# the Manifold backend bought: 58 booleans in under 4 s where CGAL wanted 20 s
# each. A test's coverage should be set by what the geometry needs, not by what
# the renderer could afford, and picking angles by hand is picking the angles you
# already thought of.
EXACT_CASES = tuple(
    (0.0, float(tilt), FIXED_MEMBERS) for tilt in range(-45, 91, 15)
) + tuple(
    (float(pan), float(tilt), ("pedestal",))
    for pan in (15, 30, 45) for tilt in (-45, 90)
)

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


# One CGAL boolean over this assembly costs about 19 s, and there are 22 of them.
# Run serially that is seven minutes, which is long enough that the check stops
# getting run - and an unrun check is worth nothing at all. They are independent
# processes, so they go in parallel; `subprocess.run` releases the GIL, so threads
# are enough and there is no pickling to arrange.
#
# Capped rather than set to `nproc`: each openscad holds a few hundred MB while it
# works, and swapping 12 of them is slower than running 6.
MAX_WORKERS = 6


def _one_overlap(job: tuple) -> tuple:
    """Runs a single OpenSCAD intersection and measures it.

    Args:
        job: `(index, pan, tilt, member)`. The index only names the scratch file,
            so that parallel workers cannot overwrite each other's answer - which
            they would, silently, and every one of them would read zero.

    Returns:
        `(pan, tilt, member, mm3)`.
    """
    index, pan, tilt, member = job
    tmp = SCAD.with_name(f"_interference_{index}.stl")
    try:
        tmp.unlink(missing_ok=True)
        subprocess.run(
            [OPENSCAD, "-D", 'part="interference"', "-D", f"tilt={tilt}",
             "-D", f"pan={pan}", "-D", f'against="{member}"',
             "--export-format", "binstl", "-o", str(tmp), str(SCAD)],
            capture_output=True, check=False,
        )
        return pan, tilt, member, _stl_volume(tmp)
    finally:
        tmp.unlink(missing_ok=True)


def exact_overlaps(verbose: bool = False) -> list:
    """Asks OpenSCAD for the actual overlap volume, member by member.

    The analytic sweep above samples the cradle as points and the arms as a plane,
    and an approximation of a shape is a shape you can get wrong. This does not
    approximate anything: `intersection()` of what moves with what does not is
    the real geometry, and its volume is either zero or it is not.

    Returns:
        List of (label, pan, tilt, mm3) for every pair that overlaps.

    Raises:
        FileNotFoundError: If openscad is not installed.
    """
    jobs = [(i, pan, tilt, member)
            for i, (pan, tilt, member) in enumerate(
                (pan, tilt, member) for pan, tilt, members in EXACT_CASES
                for member in members)]
    with futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as pool:
        results = list(pool.map(_one_overlap, jobs))
    out = []
    for pan, tilt, member, vol in results:
        if verbose:
            print(f"  pan {pan:+5.1f} tilt {tilt:+6.1f}  {member:<12} "
                  f"{vol:10.4f} mm^3")
        if vol > EXACT_TOL_MM3:
            out.append((member, pan, tilt, vol))
    return out


def _rrect_inset(x: float, y: float, half: float, r: float) -> float:
    """How far a point sits inside a rounded square centred on the origin.

    Args:
        x: Point's x.
        y: Point's y.
        half: Half the square's across-flats size.
        r: Corner radius.

    Returns:
        Distance from the point to the nearest edge, positive inside.
    """
    qx, qy = abs(x) - (half - r), abs(y) - (half - r)
    outside = np.hypot(max(qx, 0.0), max(qy, 0.0))
    return float(r - (outside + min(max(qx, qy), 0.0)))


def glide_gaps(v: dict) -> list:
    """Where the three PTFE glide pads sit, and what they have to miss.

    Their placement is a constraint shared between two parts that never appear in
    the same module: the recesses are in the pedestal's plate, the surface they
    bear on is the ring on the yoke's pad, and the things they must not overlap
    are the pedestal's own countersunk motor screws. Nothing in the .scad relates
    those three, so a change to `glide_r`, `yoke_ring_w` or the plate's size can
    put a pad half off its bearing surface and no render will look wrong.

    Args:
        v: Dimension table.

    Returns:
        List of (label, mm, minimum, note), worst case over the three pads.
    """
    pr = v["glide_d"] / 2
    ring_in = v["yoke_pad_d"] / 2 - v["yoke_ring_w"]
    ring_out = v["yoke_pad_d"] / 2
    bolt = v["motor_bolt_span"] / 2 * np.sqrt(2)          # screws on the diagonals
    inside, on_plate, off_screw = [], [], []
    for k in range(3):
        a = np.radians(v["glide_a"] + 120 * k)
        x, y = v["glide_r"] * np.cos(a), v["glide_r"] * np.sin(a)
        inside.append(min(v["glide_r"] - pr - ring_in, ring_out - v["glide_r"] - pr))
        on_plate.append(_rrect_inset(x, y, v["ped_top"] / 2, v["ped_top_r"]) - pr)
        for b in (45, 135, 225, 315):
            bx, by = bolt * np.cos(np.radians(b)), bolt * np.sin(np.radians(b))
            off_screw.append(np.hypot(x - bx, y - by) - pr - v["m3_cs_d"] / 2)
    return [
        ("glide pad in the ring", min(inside), 1.0,
         f"yoke's bearing ring is {ring_in:.1f}..{ring_out:.1f} mm radius"),
        ("glide pad on the plate", min(on_plate), 1.0,
         f"plate is {v['ped_top']:.0f} mm across flats"),
        ("glide pad to a motor screw", min(off_screw), 1.0,
         "countersinks at 45/135/225/315"),
    ]


# Every fastener, and the pose it is done up in. A screw you cannot get a driver
# onto is not a fastener, and no other check in this file looks at the air *in
# front* of a screw - only at parts avoiding parts.
#
# The camera screw is checked at tilt +90 and not at 0 because at 0 it genuinely
# does not work: the pan hub is 26 mm below the slot and a driver needs 45. That is
# why the assembly order says to tilt up first, and checking it at +90 is what
# holds that instruction to being true.
ACCESS_CASES = (
    ("pan motor", 0.0),
    ("tilt motor", 0.0),
    ("pan set screw", 0.0),
    ("pan set screw, assembled", 0.0),
    ("tilt set screw", 0.0),
    ("pivot keepers", 0.0),
    ("camera screw", 90.0),
)


def _one_access(job: tuple) -> tuple:
    """Measures how much of one driver's path is inside something solid.

    Args:
        job: `(index, screw, tilt)`; the index names the scratch file.

    Returns:
        `(screw, tilt, mm3)` - zero when the tool has a clear run at it.
    """
    index, name, tilt = job
    tmp = SCAD.with_name(f"_access_{index}.stl")
    try:
        tmp.unlink(missing_ok=True)
        subprocess.run(
            [OPENSCAD, "-D", 'part="access"', "-D", f'screw="{name}"',
             "-D", f"tilt={tilt}", "--export-format", "binstl",
             "-o", str(tmp), str(SCAD)],
            capture_output=True, check=False,
        )
        return name, tilt, _stl_volume(tmp)
    finally:
        tmp.unlink(missing_ok=True)


def exact_access(verbose: bool = False) -> list:
    """Checks a driver can reach every screw at its step in the assembly order.

    The obstacles are the parts present at that step rather than the finished
    machine, which is what turns the assembly order in the README from an assertion
    into something this file checks: sequence the steps wrongly and one of these
    starts failing.

    It has already earned it twice. The pan set screw's flat faced -X for two
    revisions and aimed the hex key straight into the fork's own arm - 122 mm^3 of
    key inside solid plastic - and the camera screw cannot be reached at all with
    the cradle level.

    Returns:
        List of (screw, tilt, mm3) for every fastener a tool cannot reach.

    Raises:
        FileNotFoundError: If openscad is not installed.
    """
    jobs = [(i, name, tilt) for i, (name, tilt) in enumerate(ACCESS_CASES)]
    with futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as pool:
        results = list(pool.map(_one_access, jobs))
    out = []
    for name, tilt, vol in results:
        if verbose:
            print(f"  {name:<26} tilt {tilt:+5.1f} {vol:10.4f} mm^3")
        if vol > EXACT_TOL_MM3:
            out.append((name, tilt, vol))
    return out


def exact_fit() -> float:
    """Overlap between the pivot pin and the fork arm it passes through, in mm^3.

    Both are fixed, so `exact_overlaps` - which compares what moves against what
    does not - cannot see this pair at all. Until this existed, the only thing
    holding the pin in the right place was the same arithmetic written twice: once
    to cut the hole in the arm, once to position the part in the assembly. A
    clearance fit and one coincident face should come to zero.

    Returns:
        The overlap volume in mm^3.

    Raises:
        FileNotFoundError: If openscad is not installed.
    """
    tmp = SCAD.with_name("_fit.stl")
    try:
        tmp.unlink(missing_ok=True)
        subprocess.run(
            [OPENSCAD, "-D", 'part="fit"', "--export-format", "binstl",
             "-o", str(tmp), str(SCAD)],
            capture_output=True, check=False,
        )
        return _stl_volume(tmp)
    finally:
        tmp.unlink(missing_ok=True)


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
    # What is left of the cheek behind the bearing's pocket, for the pin's tip to
    # run into.
    relief = v["cheek_t"] - v["brg_seat_depth"]
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
        # The pivot bearing. Three separate things can be got wrong here and only
        # the first of them is about strength.
        ("bearing seat wall",
         (v["pivot_hub_od"] - v["brg_seat_d"]) / 2, 1.6,
         f"round a {v['brg_seat_d']:.1f} mm pocket in the cheek"),
        # The pin's journal runs past the bearing into the cheek's relief, and
        # that overrun *is* the axial tolerance of the joint: the cradle can sit
        # this much further from the pivot arm than nominal with the bearing still
        # fully supported.
        ("pin overrun past the race", v["pin_overrun"], 0.5,
         "axial slack the tilt joint can absorb"),
        ("relief left past the pin tip", relief - v["pin_overrun"], 1.0,
         f"cheek is {v['cheek_t']:.0f} mm, pocket takes {v['brg_seat_depth']:.1f}"),
        # A screw at this radius has to clear the journal's hole on one side and
        # the pad's edge on the other, in an arm only as thick as its own length.
        ("keeper screw to journal",
         v["pin_screw_r"] - (v["m3_pilot"] + v["pin_hole_d"]) / 2, 1.0,
         f"{v['pin_screw_r']:.0f} mm out from the axis"),
        ("keeper screw to pad edge",
         v["pivot_pad_d"] / 2 - v["pin_screw_r"] - v["m3_pilot"] / 2, 1.6,
         "in the -X arm's pivot pad"),
        # The tilt motor's screw heads land on the *inner* face of the arm, with
        # the cheek 3 mm away and the lower two directly opposite it. Countersunk
        # they are flush; a socket cap head is 3.0 mm tall and would touch.
        ("tilt screw shank in the arm", v["arm_t_top"] - v["m3_cs_h"], 3.2,
         f"{v['arm_t_top']:.1f} mm arm, less a {v['m3_cs_h']:.1f} mm countersink"),
    ] + glide_gaps(v)


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
              "pan_hub_h", "arm_gap", "tilt_face_x", "shaft_len",
              "pivot_hub_od", "pivot_pad_d", "brg_seat_d", "brg_seat_depth",
              "pin_overrun", "pin_screw_r", "pin_hole_d", "m3_pilot",
              "glide_d", "glide_r", "glide_a", "yoke_ring_w", "ped_top",
              "ped_top_r", "motor_bolt_span", "m3_cs_d", "slew_gap",
              "ped_plate_t", "arm_t_top", "m3_cs_h")
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
                for member, pan, tilt, vol in bad:
                    print(f"  FAIL cradle overlaps the {member} by {vol:.1f} mm^3 "
                          f"at pan {pan:+.0f}, tilt {tilt:+.0f}")
            else:
                runs = sum(len(m) for _, _, m in EXACT_CASES)
                print(f"  OK  no overlap above {EXACT_TOL_MM3} mm^3 in "
                      f"{runs} booleans over {len(EXACT_CASES)} poses: tilt "
                      f"{TILT_LIMITS[0]:+.0f}..{TILT_LIMITS[1]:+.0f} every 15 deg, "
                      "and pan 15/30/45 against the pedestal")
            fit = exact_fit()
            if fit > EXACT_TOL_MM3:
                ok = False
                print(f"  FAIL the pivot pin overlaps the fork arm by "
                      f"{fit:.1f} mm^3 - it is not in its hole")
            else:
                print("  OK  the pivot pin fits its hole in the fork arm")

        print("\nreach, a driver's 45 mm at each screw's step in the order")
        try:
            blocked = exact_access(args.verbose)
        except FileNotFoundError:
            print("  SKIP openscad not on PATH")
        else:
            for name, tilt, vol in blocked:
                ok = False
                print(f"  FAIL no room for a driver on the {name} "
                      f"(tilt {tilt:+.0f}): {vol:.1f} mm^3 of it is inside a part")
            if not blocked:
                print(f"  OK  all {len(ACCESS_CASES)} fasteners: "
                      + ", ".join(n for n, _ in ACCESS_CASES))

    print(f"\nrequirement: swept >= {REQUIRED_MM:.1f} mm, static as noted, "
          "exact overlap zero")
    if not ok:
        print("look at which line failed before changing anything: a swept "
              "failure usually wants tilt_axis_h raised, a static one wants the "
              "feature that owns the number - they are not the same repair")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
