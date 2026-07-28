#!/usr/bin/env python3
"""Asks a real slicer whether these parts print, instead of inferring it.

`check_printability.py` measures the geometry - overhang angles, bridge spans -
which is a model of what a slicer will do. This runs the slicer itself, with
supports off, the way the parts are meant to be printed, and reads back its own
verdict. PrusaSlicer 2.9 has a print-stability detector that names what it finds:
"Floating bridge anchors", "Loose extrusions", "Long bridging extrusions".

Two different tools agreeing is worth more than either alone, and they check each
other: the geometric one says *where* and *how wide*, this one says whether the
software that generates the actual toolpaths is happy.

CALIBRATE BEFORE BELIEVING. `_calibration.scad` holds a part that obviously needs
support and one that obviously does not, and `--calibrate` slices both. If the bad
one does not raise the alarm, this check is not working and its silence on the
real parts means nothing. That is not paranoia - the first run of this against the
yoke reported "Long bridging extrusions", which took a while to pin down as the
pad's top solid shell bridging over *sparse infill*: it disappears at 100% infill
and has nothing to do with the shape. A slicer warning is evidence, not a verdict.

Usage:
    python check_slicing.py
    python check_slicing.py --calibrate    # prove the detector still detects
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
from concurrent import futures

HERE = pathlib.Path(__file__).parent

# The flatpak cannot write outside its sandbox, and fails by *warning* while
# exiting 0 - the same trap the OpenSCAD flatpak sets. Everything goes to $HOME.
SCRATCH = pathlib.Path.home()

SLICER = ["flatpak", "run", "com.prusa3d.PrusaSlicer"]

# Print settings the README documents. Infill is 40% there and the pad's top
# shell bridges over it, which the stability detector calls out; the parts are
# sliced here at the same 40% and that one message is expected and excused by
# name below rather than by turning the check off.
SETTINGS = ["--layer-height", "0.2", "--perimeters", "4", "--fill-density", "40%",
            "--nozzle-diameter", "0.4", "--filament-diameter", "1.75",
            "--temperature", "215", "--bed-temperature", "60"]

PARTS = ("pan_yoke", "camera_cradle", "pedestal", "pivot_pin")

# Issues that mean the geometry is wrong, and the one that does not.
#
# "Long bridging extrusions" on a part whose widest real span is 9 mm is the top
# solid layer crossing sparse infill - it is a function of `--fill-density`, not
# of the shape, and it goes away at 100%. Excusing it by name keeps the other two
# live; suppressing the whole warning would not.
FATAL = ("Floating bridge anchors", "Loose extrusions")
EXCUSED = ("Long bridging extrusions",)


def slice_part(stl: pathlib.Path, tag: str) -> tuple:
    """Slices one STL with supports off and reads back what the slicer thought.

    Args:
        stl: The part to slice.
        tag: Short name, used for the scratch G-code file.

    Returns:
        `(issues, cm3, minutes)` - the stability issues it named, the filament it
        wants, and its own time estimate. Empty issues means it was happy.
    """
    out = SCRATCH / f"_slice_{tag}.gcode"
    try:
        out.unlink(missing_ok=True)
        r = subprocess.run(SLICER + ["--export-gcode"] + SETTINGS
                           + ["--output", str(out), str(stl)],
                           capture_output=True, text=True)
        text = r.stdout + r.stderr
        # The issues come back comma-separated on one line when there are several
        # and on their own line when there is one, so match the names in the text
        # rather than trying to match the layout. Parsing this by line silently
        # found nothing on a part that had three problems, and the calibration
        # run is what caught it - which is the entire reason it exists.
        issues = [name for name in FATAL + EXCUSED if name in text]
        if not out.exists():
            return ["slicer wrote no G-code"], 0.0, 0.0
        gcode = out.read_text()
        cm3 = re.search(r"; filament used \[cm3\] = ([\d.]+)", gcode)
        tm = re.search(r"; estimated printing time \(normal mode\) = (.+)", gcode)
        return issues, float(cm3.group(1)) if cm3 else 0.0, tm.group(1) if tm else "?"
    finally:
        out.unlink(missing_ok=True)


def calibrate() -> int:
    """Proves the detector still detects, on two parts with known answers.

    Returns:
        0 if the bad part raises the alarm and the good one does not.
    """
    cal = HERE / "_calibration.scad"
    ok = True
    for which, expect in (("bad", True), ("good", False)):
        stl = SCRATCH / f"_cal_{which}.stl"
        subprocess.run([_openscad(), "-D", f'which="{which}"', "--export-format",
                        "binstl", "-o", str(stl), str(cal)], capture_output=True)
        issues, _, _ = slice_part(stl, f"cal_{which}")
        stl.unlink(missing_ok=True)
        hit = bool(issues)
        good = hit == expect
        ok = ok and good
        print(f"  {'OK ' if good else 'FAIL'} {which:<5} expected "
              f"{'issues' if expect else 'silence'}, got "
              f"{', '.join(issues) if issues else 'silence'}")
    print("\n" + ("calibrated: the detector reacts to a part that needs support"
                  if ok else
                  "NOT calibrated - a clean result on the real parts means nothing"))
    return 0 if ok else 1


def _openscad() -> str:
    """The same OpenSCAD the other checkers use."""
    from check_clearances import OPENSCAD
    return OPENSCAD


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true",
                    help="slice the known-good and known-bad parts instead")
    args = ap.parse_args()

    if args.calibrate:
        return calibrate()

    jobs = [(HERE / "stl" / f"{n}.stl", n) for n in PARTS]
    missing = [n for p, n in jobs if not p.exists()]
    if missing:
        print(f"missing STLs: {missing} - run the openscad export first")
        return 2

    with futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda j: slice_part(*j), jobs))

    print(f"{'part':<16}{'filament':>10}{'print time':>14}   slicer's verdict")
    print("-" * 68)
    ok = True
    total_cm3 = total_min = 0.0
    for (stl, name), (issues, cm3, tm) in zip(jobs, results):
        fatal = [i for i in issues if i in FATAL]
        note = ", ".join(issues) if issues else "no stability issues"
        if fatal:
            ok = False
        total_cm3 += cm3
        m = re.findall(r"(\d+)([hms])", tm)
        total_min += sum(int(n) * {"h": 60, "m": 1, "s": 1 / 60}[u] for n, u in m)
        print(f"{name:<16}{cm3:8.1f} cm3{tm:>14}   "
              f"{'FAIL ' if fatal else 'OK   '}{note}")
    print(f"{'whole machine':<16}{total_cm3:8.1f} cm3"
          f"{f'{total_min/60:.1f} h':>14}")
    if EXCUSED:
        print(f"\nexcused by name: {', '.join(EXCUSED)} - the pad's top shell over "
              f"40% infill,\nwhich disappears at 100% and is a setting, not a shape")
    print("\nrun --calibrate if you want to know the detector is still awake")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
