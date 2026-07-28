#!/usr/bin/env python3
"""Measures whether each part actually prints without support.

Renders and intuition are not evidence. This reads the exported STLs and
measures, in the orientation each part is documented to print in:

  * down-facing patches steep enough to need support, and how far each one
    has to span unsupported
  * the bed footprint, because a tall part on a small footprint tips
  * the shallowest wall angle, which is what actually decides support

**Area alone is the wrong test, and saying so is the point of this file.**
Every bolt hole through a vertical plate has a down-facing ceiling, and a
part with a dozen of them accumulates a large overhang area while printing
perfectly - because each ceiling only has to bridge the width of the hole.
What decides whether support is needed is the *span*, so patches are found
by connectivity and reported by span, and anything narrower than a routine
bridge passes.

Which leaves the question of what a span is, and that took three attempts on
this design alone. `_span` measures the one thing that decides it: how far
the overhang reaches from material in the layer below. Both cheaper proxies
were wrong in opposite directions on the same feature - see the note there,
because the wrong ones were the plausible-looking ones.

The measurements have earned their keep. On this design they found a hold-down
ear whose 45 deg blend was buried in a wall that had already tapered out from
under it, leaving a flat 13 x 14 mm shelf printing into thin air; four ears
rotated onto the wall faces they were specified to avoid; and a 0.13 mm ledge
running round a 56 mm perimeter. All three looked correct in a render.

Usage:
    python check_printability.py
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
from check_clearances import warn_if_stale
from stl import mesh

HERE = pathlib.Path(__file__).parent

# Each part and the orientation it is documented to print in.
#
# There is no rotation column any more, and that is the point: `gimbal_parts.scad`
# models every part in its print orientation, so what this reads off the STL is
# what the slicer will see. The previous version had to flip the base plate here,
# which meant this file and the .scad could disagree about which way up a part
# printed and nothing would notice.
PARTS = [
    ("pan_yoke.stl", "pad on the bed, fork arms up"),
    ("camera_cradle.stl", "platform on the bed, cheeks up"),
    ("pedestal.stl", "top plate on the bed, desk end last"),
    ("pivot_pin.stl", "flange on the bed, journal up"),
]

# A facet normal within this angle of straight down is a near-horizontal
# down-face: unsupported unless it bridges. 44 rather than 45 deliberately - a
# surface drafted at exactly the 45 deg rule sits on the threshold, and whether
# it lands inside or outside comes down to rounding. Parts here draft at 40.
OVERHANG_COS = np.cos(np.radians(44.0))

# The bed. This is a modelling tolerance, not a distance - just above the `eps`
# the .scad uses for its zero-height hull sections.
#
# There used to be one "near the bed" threshold of 0.5 mm serving both of the
# measurements below, and it could not: a 0.6 mm bed chamfer has to count as bed
# and the ceiling of a 0.4 mm-deep glide-pad recess has to not, and no single
# height separates 0.4 from 0.6. Three 10 mm bridges dropped out of the
# measurement the moment those pockets were added, and the part still passed. So
# the two questions get the two thresholds they actually need.
BED_TOL = 0.02

# For the wall-angle measurement only: a down-facing surface that begins within a
# chamfer's height of the bed is a chamfer, not a wall. Every bed edge here is
# chamfered at 45 deg, which is shallower than any wall in the design, so without
# this the figure is always "45 deg" and always means the chamfer.
#
# Asking instead whether a facet *touches* the bed nearly works and is not
# robust: CGAL re-triangulates during the booleans, and 5 mm^2 of the pivot pin's
# chamfer came out as triangles with no vertex on the bed at all.
CHAMFER_MAX = 1.0

# The shallowest-wall figure walks up from the shallowest facet until it has this
# much area, and reports where it got to. Without it the number is set by whatever
# single sliver the tessellator left where a lead-in cone meets a bore: one
# 0.14 mm^2 triangle at 45.4 deg was being reported as this design's shallowest
# wall while the shallowest actual wall was 48 deg. A wall you can measure with a
# protractor is not 0.14 mm^2 in area.
WALL_MIN_MM2 = 1.0

# A patch narrower than this bridges without support. Conservative: FDM
# routinely spans 20 mm or more, and every patch in this design is a hole
# ceiling crossing a 4-5 mm plate.
BRIDGE_MM = 12.0


def analyse(path: pathlib.Path) -> dict:
    """Measures the printability of one STL as modelled.

    Args:
        path: The STL to read.

    Returns:
        Dict of measurements: bed footprint area, unsupported down-face area
        and its height above the bed, and the part's height.
    """
    m = mesh.Mesh.from_file(str(path))
    tri = m.vectors.copy()

    # Per-facet normal and area from the triangle itself, rather than trusting
    # the normals stored in the file.
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    cross = np.cross(v1 - v0, v2 - v0)
    area = np.linalg.norm(cross, axis=1) / 2.0
    with np.errstate(invalid="ignore", divide="ignore"):
        nz = np.divide(cross[:, 2], np.linalg.norm(cross, axis=1))
    nz = np.nan_to_num(nz)

    z_min = tri[:, :, 2].min()
    z_max = tri[:, :, 2].max()

    # A down-face lying *on* the bed is the first layer: the extruder is laying it
    # onto glass. A down-face that starts anywhere above it is an overhang however
    # low it is - which is what the glide-pad recesses are, 0.4 mm up.
    low = tri[:, :, 2].min(axis=1)
    on_bed = (nz < -OVERHANG_COS) & (tri[:, :, 2].max(axis=1) <= z_min + BED_TOL)
    overhang = (nz < -OVERHANG_COS) & (low > z_min + BED_TOL)

    patches = _patches(tri[overhang], area[overhang], tri)

    # Shallowest wall: the facet closest to horizontal that still faces
    # downward at all, ignoring the bed and true horizontals.
    sloping = (nz < 0) & (nz >= -OVERHANG_COS) & (low > z_min + CHAMFER_MAX)
    worst_wall, sliver = _shallowest(nz, area, sloping)

    return {
        "height": z_max - z_min,
        "bed_area": float(area[on_bed].sum()),
        "overhang_area": float(area[overhang].sum()),
        "patches": patches,
        "worst_span": max((p["span"] for p in patches), default=0.0),
        "worst_wall_deg": worst_wall,
        "sliver_mm2": sliver,
    }


def _shallowest(nz: np.ndarray, area: np.ndarray, mask: np.ndarray) -> tuple:
    """The shallowest down-facing wall, discounting tessellation slivers.

    Args:
        nz: Per-facet normal z component.
        area: Per-facet area.
        mask: Which facets count as sloping walls.

    Returns:
        `(degrees_from_horizontal, sliver_mm2)` - the angle reached once
        `WALL_MIN_MM2` of facet area has been accumulated from the shallowest
        facet upward, and how much area was shallower than that. Reporting the
        second number is the point: it is what tells you whether the first one
        skipped a real surface or a triangle you could not see.
    """
    if not mask.any():
        return None, 0.0
    deg = np.degrees(np.arccos(np.clip(-nz[mask], -1, 1)))
    order = np.argsort(deg)
    cum = np.cumsum(area[mask][order])
    i = min(int(np.searchsorted(cum, WALL_MIN_MM2)), len(order) - 1)
    return float(deg[order][i]), float(cum[i - 1]) if i else 0.0


def _bary(tri: np.ndarray, pts: np.ndarray) -> tuple:
    """Projects points onto facets in plan and returns barycentric coordinates.

    Args:
        tri: Facets, shape (n, 3, 3). Only x and y are used.
        pts: Query points, shape (m, 2).

    Returns:
        `(hit, zf)`, both shape (m, n): whether each point falls inside each
        facet's plan projection, and the facet's z there.
    """
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v0 = (b - a)[:, :2]
    v1 = (c - a)[:, :2]
    v2 = pts[:, None, :] - a[None, :, :2]
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    ok = np.abs(den) > 1e-12
    safe = np.where(ok, den, 1.0)
    u = (v2[:, :, 0] * v1[None, :, 1] - v1[None, :, 0] * v2[:, :, 1]) / safe
    v = (v0[None, :, 0] * v2[:, :, 1] - v2[:, :, 0] * v0[None, :, 1]) / safe
    hit = ok[None, :] & (u >= 0) & (v >= 0) & (u + v <= 1)
    zf = (a[None, :, 2] + u * (b - a)[None, :, 2] + v * (c - a)[None, :, 2])
    return hit, zf


def _inside(tri: np.ndarray, pts: np.ndarray, z: float) -> np.ndarray:
    """Which of the points are inside the solid, by ray parity straight upward.

    Args:
        tri: Facets to test against, shape (n, 3, 3).
        pts: Query points, shape (m, 2).
        z: The height to test at.

    Returns:
        Boolean array of shape (m). True where an upward ray crosses an odd
        number of facets, which for a closed mesh means it started in material.
    """
    hit, zf = _bary(tri, pts)
    return (np.count_nonzero(hit & (zf > z), axis=1) % 2).astype(bool)


def _span(patch: np.ndarray, tri: np.ndarray, layer: float = 0.25,
          pitch: float = 0.5, margin: float = 4.0) -> float:
    """How far the overhang reaches from material in the layer below it.

    This is the question a slicer answers, so it is the question asked here:
    sample the patch's footprint on a grid, ask of each sample whether there is
    solid material one layer lower, and measure how far the samples that have
    none are from the ones that do. Twice that distance is the width of air the
    extruder has to cross.

    Two earlier attempts got this wrong in opposite directions, and both looked
    reasonable:

      * the shorter of the patch's two bbox dimensions - which called a 12 mm
        aperture roof in a 4 mm wall "4 mm", the wall's thickness being the one
        number in it that is not a span;
      * the distance to the nearest boundary edge with material beside and below
        it - which called the same roof "0.8 mm", because the 40 deg flank
        beside it is material, just not material the next layer can stand on.

    Args:
        patch: The patch's facets, shape (m, 3, 3).
        tri: Every facet in the mesh.
        layer: How far below the patch to look for support.
        pitch: Grid spacing for the sampling.
        margin: How far outside the patch to look for supported ground.

    Returns:
        The span in mm: 0 when every sample is supported, and infinity when none
        of them is and there is no supported ground nearby either.
    """
    z = float(patch[:, :, 2].min()) - layer
    lo = patch[:, :, :2].reshape(-1, 2).min(axis=0) - margin
    hi = patch[:, :, :2].reshape(-1, 2).max(axis=0) + margin

    # Only facets overlapping the window can matter, and cutting the mesh down
    # to those is what keeps this affordable: it turns 30k triangles into a few
    # hundred.
    keep = ((tri[:, :, 0].max(1) >= lo[0]) & (tri[:, :, 0].min(1) <= hi[0])
            & (tri[:, :, 1].max(1) >= lo[1]) & (tri[:, :, 1].min(1) <= hi[1]))
    sub = tri[keep]

    gx = np.arange(lo[0], hi[0] + pitch, pitch)
    gy = np.arange(lo[1], hi[1] + pitch, pitch)
    pts = np.stack(np.meshgrid(gx, gy, indexing="ij"), axis=-1).reshape(-1, 2)

    in_patch = _bary(patch, pts)[0].any(axis=1)
    supported = _inside(sub, pts, z)

    bare = in_patch & ~supported
    if not bare.any():
        return 0.0
    if not supported.any():
        return float("inf")
    d = np.linalg.norm(pts[bare][:, None, :] - pts[supported][None, :, :], axis=2)
    return float(2.0 * d.min(axis=1).max())


def _patches(tri: np.ndarray, area: np.ndarray, mesh_tri: np.ndarray) -> list:
    """Groups overhang facets into connected patches and measures each span.

    Facets are joined when they share a vertex, so a hole's ceiling comes out
    as one patch rather than as the dozens of triangles it is tessellated into.

    Args:
        tri: Overhang facets, shape (n, 3, 3).
        area: Their areas.
        mesh_tri: Every facet in the mesh, for the span measurement.

    Returns:
        One dict per patch with its area and its span.
    """
    if len(tri) == 0:
        return []
    # Union-find over vertices quantised to 1 micron, so shared corners match.
    parent: dict = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    keys = np.round(tri * 1000).astype(np.int64)
    for f in keys:
        a, b, c = map(tuple, f)
        union(a, b)
        union(a, c)

    groups: dict = {}
    for f, k, ar in zip(tri, keys, area):
        root = find(tuple(k[0]))
        g = groups.setdefault(root, {"pts": [], "area": 0.0})
        g["pts"].append(f)
        g["area"] += float(ar)

    out = []
    for g in groups.values():
        f = np.array(g["pts"])
        out.append({"area": g["area"], "span": _span(f, mesh_tri),
                    "z": float(f[:, :, 2].min())})
    return sorted(out, key=lambda p: -p["span"])


def main() -> int:
    warn_if_stale(HERE / "stl" / name for name, _ in PARTS)
    print(f"{'part':<20}{'height':>8}{'bed':>10}{'overhang':>10}{'widest span':>14}")
    print(f"{'':<20}{'mm':>8}{'mm^2':>10}{'mm^2':>10}{'mm':>14}")
    print("-" * 62)

    problems = []
    for name, orientation in PARTS:
        path = HERE / "stl" / name
        if not path.exists():
            print(f"{name:<20}  missing - run the openscad export first")
            problems.append(name)
            continue
        r = analyse(path)
        print(f"{name:<20}{r['height']:>8.1f}{r['bed_area']:>10.0f}"
              f"{r['overhang_area']:>10.1f}{r['worst_span']:>14.1f}")
        print(f"{'  ' + orientation:<20}")
        if r["worst_wall_deg"] is not None:
            sliver = (f", past {r['sliver_mm2']:.2f} mm^2 of slivers"
                      if r["sliver_mm2"] > 0.005 else "")
            print(f"    shallowest down-facing wall: {r['worst_wall_deg']:.0f} deg "
                  f"from horizontal (45+ is self-supporting){sliver}")
        if r["patches"]:
            verdict = ("" if r["worst_span"] > BRIDGE_MM
                       else f" - all bridge below {BRIDGE_MM:.0f} mm")
            print(f"    {len(r['patches'])} down-facing patches, widest span "
                  f"{r['worst_span']:.1f} mm{verdict}")
        # A tall part on a small footprint will tip regardless of overhangs.
        if r["bed_area"] > 0 and r["height"] / np.sqrt(r["bed_area"]) > 3.0:
            print("    NOTE tall relative to its footprint - use a brim")
        if r["worst_span"] > BRIDGE_MM:
            wide = [p for p in r["patches"] if p["span"] > BRIDGE_MM]
            print(f"    SUPPORT NEEDED: {len(wide)} patch(es) span up to "
                  f"{r['worst_span']:.1f} mm")
            problems.append(name)

    print()
    if problems:
        print("parts needing attention: " + ", ".join(problems))
        return 1
    print("all parts print in the stated orientation with no support")
    return 0


if __name__ == "__main__":
    sys.exit(main())
