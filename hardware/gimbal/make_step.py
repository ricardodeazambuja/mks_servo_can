#!/usr/bin/env python3
"""Exports each part as a STEP solid, for opening in CAD.

OpenSCAD cannot write STEP and never will: it is a mesh kernel, and STEP is a
boundary representation of analytic surfaces. So the parts go out through
FreeCAD's OpenCASCADE kernel - the STL is sewn into a shell, the shell is made a
solid, and `removeSplitter()` merges coplanar facets back into single faces.

BE CLEAR ABOUT WHAT YOU GET. Flat faces come back as *real* planar faces, so the
plate, the walls and the pad are one face each and you can measure and mate to
them. Curved surfaces do not: a cylinder arrives as the 64 facets OpenSCAD drew
it with, because the information that it was ever a cylinder was thrown away at
export. A hole will not snap to a centre and a fillet is not a fillet. It is a
solid you can boolean against and build a mount around, not an editable model.

If you need true analytic surfaces, that is not a converter problem - it means
authoring in a B-rep kernel (build123d, CadQuery, FreeCAD itself) rather than in
OpenSCAD.

The refine step is worth its runtime: on these parts it takes the face count down
by 53-63 % - every flat region collapses to one face while the tessellated
cylinders stay as they are, which is exactly the split you would expect and a
reasonable check that it did what it claims.

Volumes are compared against the source mesh afterwards and agree to 2e-4 %. That
matters more than it sounds: sewing a mesh into a solid is where a conversion goes
quietly wrong, and a STEP that opens without complaint can still be a different
shape from the one you printed.

Usage:
    python make_step.py
    python make_step.py --raw     # skip the refine, if it ever chokes on a part
"""
from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import textwrap

from check_clearances import _stl_volume

HERE = pathlib.Path(__file__).parent
OUT = HERE / "step"

# The flatpak cannot write outside its sandbox and says so in a warning while
# exiting 0, so the conversion happens under $HOME and the results are moved.
SCRATCH = pathlib.Path.home()

FREECAD = ["flatpak", "run", "--command=FreeCADCmd", "org.freecad.FreeCAD"]

# How far the sewn solid's volume may differ from the mesh it came from. This
# is a sanity bound on the conversion, not a tolerance anyone chose: it lands
# at 2e-4 %% in practice.
VOLUME_TOL_PCT = 0.01

PARTS = ("pan_yoke", "camera_cradle", "pedestal", "pivot_pin")

# The paths are baked into the script rather than passed after it: FreeCADCmd
# treats every argument after the script as another *file to open*, so passing
# them made it try to read the not-yet-written STEP and fail with "Cannot read
# STEP file" - a message about the wrong file entirely.
SCRIPT = textwrap.dedent("""
    import Mesh, Part
    stl, step, refine = {stl!r}, {step!r}, {refine}
    m = Mesh.Mesh(stl)
    shape = Part.Shape()
    # 0.05 mm sewing tolerance: well under any feature here, well over the
    # float32 rounding an STL's vertices carry.
    shape.makeShapeFromMesh(m.Topology, 0.05)
    solid = Part.makeSolid(shape)
    before = len(solid.Faces)
    if refine:
        solid = solid.removeSplitter()
    # No f-string here on purpose: this whole block goes through .format(), and
    # braces in it would be read as placeholders.
    print("RESULT %d %d %.3f %s" % (before, len(solid.Faces), solid.Volume,
                                    "valid" if solid.isValid() else "INVALID"))
    solid.exportStep(step)
""").strip()


def convert(name: str, refine: bool) -> tuple:
    """Runs one STL through FreeCAD and lands a STEP file.

    Args:
        name: Part name, without extension.
        refine: Whether to merge coplanar facets.

    Returns:
        `(faces_before, faces_after, volume_mm3, validity)`, or None on failure.
    """
    script = SCRATCH / "_stl2step.py"
    tmp_stl = SCRATCH / f"_conv_{name}.stl"
    tmp_step = SCRATCH / f"_conv_{name}.step"
    script.write_text(SCRIPT.format(stl=str(tmp_stl), step=str(tmp_step),
                                    refine=bool(refine)))
    try:
        tmp_stl.write_bytes((HERE / "stl" / f"{name}.stl").read_bytes())
        r = subprocess.run(FREECAD + [str(script)], capture_output=True, text=True)
        line = next((out for out in (r.stdout + r.stderr).splitlines()
                     if out.startswith("RESULT")), None)
        if not line or not tmp_step.exists():
            print(f"  {name}: FAILED\n{(r.stdout + r.stderr)[-400:]}")
            return None
        OUT.mkdir(exist_ok=True)
        (OUT / f"{name}.step").write_bytes(tmp_step.read_bytes())
        before, after, vol, valid = line.split()[1:]
        return int(before), int(after), float(vol), valid
    finally:
        for p in (script, tmp_stl, tmp_step):
            p.unlink(missing_ok=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", action="store_true", help="skip the refine pass")
    args = ap.parse_args()

    print(f"{'part':<16}{'faces':>18}{'volume mm3':>13}{'vs mesh':>10}"
          f"{'size':>8}  solid")
    print("-" * 72)
    ok = True
    for name in PARTS:
        stl = HERE / "stl" / f"{name}.stl"
        if not stl.exists():
            print(f"{name:<16}  no STL - run the openscad export first")
            return 2
        res = convert(name, not args.raw)
        if res is None:
            ok = False
            continue
        before, after, vol, valid = res
        kb = (OUT / f"{name}.step").stat().st_size / 1024
        # Sewing a mesh into a solid is where this goes quietly wrong, and a STEP
        # that opens cleanly can still be a different shape from the one you
        # printed. Compare against the mesh it came from.
        mesh_vol = _stl_volume(stl)
        drift = 100 * abs(vol - mesh_vol) / mesh_vol if mesh_vol else 0.0
        if valid != "valid" or drift > VOLUME_TOL_PCT:
            ok = False
        print(f"{name:<16}{before:8d} -> {after:<6d}{vol:13.1f}{drift:9.4f}%"
              f"{kb:7.0f}k  {valid}"
              + ("  VOLUME DRIFT" if drift > VOLUME_TOL_PCT else ""))
    print(f"\nwrote {OUT.relative_to(HERE.parent)}/*.step")
    print("flat faces are real planar faces; curved ones are still the facets "
          "OpenSCAD drew")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
