"""
The printed gimbal's soft limits must be one number, wherever they are written.

`examples/camera_gimbal_tracker.py` owns them: `gimbal_cli.py` imports them from
there rather than restating them, and `hardware/gimbal/README.md` says so. The
files under `hardware/gimbal/` cannot import that module - it pulls in the whole
library, and the design checks are meant to run in a checkout with nothing but
numpy installed - so they keep a copy, and a copy is a thing that drifts.

It drifted. Pan was cut from +/-170 to +/-90 when the machine was built and the
loom turned out to run out of slack there. `check_clearances.py` never saw the
change because it does not sweep pan; `make_visualizer.py` held its own second
copy, so `gimbal_viewer.html` went on offering a +/-170 slider and a "pan max"
preset 80 degrees past anything the cables allow - on a page whose README says
it is generated precisely so it cannot drift from the machine.

The copy is fine. The copy being *unchecked* was the defect, so this reads both
files and fails if they disagree. Parsed rather than imported, so it costs
nothing and runs on an installed wheel where `hardware/` is not packaged.
"""
import ast
import pathlib

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SOURCE_OF_TRUTH = REPO_ROOT / "examples" / "camera_gimbal_tracker.py"
COPIES = [REPO_ROOT / "hardware" / "gimbal" / "check_clearances.py"]

LIMIT_NAMES = ["PAN_LIMITS", "TILT_LIMITS"]


def _limits(path):
    """
    Reads the module-level limit tuples out of a source file without importing it.

    Args:
        path: The file to parse.

    Returns:
        A mapping of limit name to the tuple it is assigned, for the names in
        `LIMIT_NAMES` that the file assigns at module level.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in LIMIT_NAMES:
                found[target.id] = ast.literal_eval(node.value)
    return found


@pytest.fixture(scope="module")
def reference():
    """The limits every other file must agree with."""
    limits = _limits(SOURCE_OF_TRUTH)
    missing = [name for name in LIMIT_NAMES if name not in limits]
    assert not missing, (
        f"{SOURCE_OF_TRUTH.name} no longer defines {missing} at module level. "
        "It is where these live; move them back or repoint this test."
    )
    return limits


@pytest.mark.parametrize("copy_path", COPIES, ids=lambda p: p.name)
class TestGimbalLimitsAgree:
    """The hardware checks' copies must match the tracker's originals."""

    def test_every_limit_it_copies_matches(self, copy_path, reference):
        """A file that restates a limit must restate it correctly."""
        copied = _limits(copy_path)
        assert copied, (
            f"{copy_path.name} defines none of {LIMIT_NAMES}. If the copy moved, "
            "point this test at wherever it went - deleting it leaves the viewer "
            "free to disagree with the machine again."
        )
        for name, value in copied.items():
            assert value == reference[name], (
                f"{copy_path.name} says {name} = {value}, "
                f"{SOURCE_OF_TRUTH.name} says {reference[name]}. "
                "The tracker is the source of truth; update the copy."
            )

    def test_it_copies_both_limits(self, copy_path, reference):
        """
        Both limits, not just the one a given script happens to sweep.

        `check_clearances.py` uses only the tilt limit. It carries the pan one
        for `make_visualizer.py`, which is exactly the value that went stale, so
        an unused copy is the one most worth checking.
        """
        copied = _limits(copy_path)
        assert set(copied) == set(LIMIT_NAMES), (
            f"{copy_path.name} defines {sorted(copied)}, expected "
            f"{sorted(LIMIT_NAMES)}."
        )
