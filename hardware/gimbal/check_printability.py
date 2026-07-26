#!/usr/bin/env python3
"""Measures whether each part actually prints without support.

Renders and intuition are not evidence. This reads the exported STLs and
measures, in the orientation each part is documented to print in:

  * down-facing patches steep enough to need support, and how far each one
    has to span unsupported
  * the bed footprint, because a tall part on a small footprint tips
  * the shallowest wall angle, which is what actually decides support

The overhang threshold is the usual 45 deg from vertical: a facet whose
normal points more than that far downward is closer to horizontal than to
vertical.

**Area alone is the wrong test, and saying so is the point of this file.**
Every bolt hole through a vertical plate has a down-facing ceiling, and a
part with a dozen of them accumulates a large overhang area while printing
perfectly - because each ceiling only has to bridge the thickness of the
plate it passes through. What decides whether support is needed is the
*span*: the shortest horizontal distance a patch has to cross before it
lands on material again. So patches are found by connectivity and reported
by span, and anything narrower than a routine bridge passes.

Usage:
    python check_printability.py
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
from stl import mesh

HERE = pathlib.Path(__file__).parent

# Each part, and the rotation that puts it in its documented print orientation.
# "flip_z" turns the part upside down, which is how the base plate prints:
# modelled with its legs hanging below the plate, printed with them pointing up.
PARTS = [
    ("pan_yoke.stl", "beam underside on the bed", False),
    ("camera_cradle.stl", "standing on the hub's end face", False),
    ("base_plate.stl", "plate flat, legs up", True),
]

# cos(45 deg). A facet normal with n_z below -this is a near-horizontal
# down-face: unsupported unless it bridges.
OVERHANG_COS = np.cos(np.radians(45.0))

# Down-faces within this distance of the lowest point are the part sitting on
# the bed, not an overhang.
BED_TOL = 0.5

# A patch narrower than this bridges without support. Conservative: FDM
# routinely spans 20 mm or more, and every patch in this design is a hole
# ceiling crossing a 4-5 mm plate.
BRIDGE_MM = 12.0


def analyse(path: pathlib.Path, flip: bool) -> dict:
    """Measures the printability of one STL in its print orientation.

    Args:
        path: The STL to read.
        flip: Whether to turn the part upside down first.

    Returns:
        Dict of measurements: bed footprint area, unsupported down-face area
        and its height above the bed, and the part's height.
    """
    m = mesh.Mesh.from_file(str(path))
    tri = m.vectors.copy()
    if flip:
        tri[:, :, 2] *= -1.0
        tri = tri[:, ::-1, :]  # keep winding, and therefore normals, consistent

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
    centroid_z = tri[:, :, 2].mean(axis=1)

    on_bed = (nz < -OVERHANG_COS) & (centroid_z <= z_min + BED_TOL)
    overhang = (nz < -OVERHANG_COS) & ~on_bed

    patches = _patches(tri[overhang], area[overhang])

    # Shallowest wall: the facet closest to horizontal that still faces
    # downward at all, ignoring the bed and true horizontals.
    sloping = (nz < 0) & (nz >= -OVERHANG_COS) & ~on_bed
    worst_wall = np.degrees(np.arccos(np.clip(-nz[sloping].min(), -1, 1))) if sloping.any() else None

    return {
        "height": z_max - z_min,
        "bed_area": float(area[on_bed].sum()),
        "overhang_area": float(area[overhang].sum()),
        "patches": patches,
        "worst_span": max((p["span"] for p in patches), default=0.0),
        "worst_wall_deg": worst_wall,
    }


def _patches(tri: np.ndarray, area: np.ndarray) -> list:
    """Groups overhang facets into connected patches and measures each span.

    Facets are joined when they share a vertex, so a hole's ceiling comes out
    as one patch rather than as the dozens of triangles it is tessellated into.

    Args:
        tri: Overhang facets, shape (n, 3, 3).
        area: Their areas.

    Returns:
        One dict per patch with its area and its span, where span is the
        shorter horizontal dimension - the distance the bridge actually has to
        cross.
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
        pts = np.concatenate(g["pts"], axis=0)
        dx = pts[:, 0].max() - pts[:, 0].min()
        dy = pts[:, 1].max() - pts[:, 1].min()
        out.append({"area": g["area"], "span": float(min(dx, dy)),
                    "z": float(pts[:, 2].min())})
    return sorted(out, key=lambda p: -p["span"])


def main() -> int:
    print(f"{'part':<20}{'height':>8}{'bed':>10}{'overhang':>10}{'widest span':>14}")
    print(f"{'':<20}{'mm':>8}{'mm^2':>10}{'mm^2':>10}{'mm':>14}")
    print("-" * 62)

    problems = []
    for name, orientation, flip in PARTS:
        path = HERE / "stl" / name
        if not path.exists():
            print(f"{name:<20}  missing - run the openscad export first")
            problems.append(name)
            continue
        r = analyse(path, flip)
        print(f"{name:<20}{r['height']:>8.1f}{r['bed_area']:>10.0f}"
              f"{r['overhang_area']:>10.1f}{r['worst_span']:>14.1f}")
        print(f"{'  ' + orientation:<20}")
        if r["worst_wall_deg"] is not None:
            print(f"    shallowest down-facing wall: {r['worst_wall_deg']:.0f} deg "
                  f"from horizontal (45+ is self-supporting)")
        if r["patches"]:
            print(f"    {len(r['patches'])} down-facing patches, widest span "
                  f"{r['worst_span']:.1f} mm - all bridge below {BRIDGE_MM:.0f} mm")
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
