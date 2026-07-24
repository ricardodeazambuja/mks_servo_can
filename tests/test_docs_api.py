"""
The documentation must describe the API that exists.

Every code block in `docs/` and `README.md` is parsed and every name it uses is
checked against the real package. Internal links in the documentation index are
checked to resolve. Neither had ever been verified, and both had drifted badly:
26 references to methods and parameters that do not exist, 7 blocks that are not
valid Python, and 27 of 52 index links pointing at documents that were never
written.

This is a **ratchet**, not a clean gate. Fixing all of that at once is a separate
job from stopping it getting worse, so the known problems are listed in
`tests/fixtures/docs_known_issues.json` and the test enforces two things:

- a finding that is *not* in the baseline fails the build - documentation cannot
  regress further;
- a baseline entry that no longer occurs also fails the build, with an
  instruction to delete it - so the baseline can only shrink, and cannot quietly
  grant permission for a problem that has already been fixed.

To fix documentation: correct the file, run this test, and delete the entries it
tells you are stale.

Findings are recorded as `kind|path|detail` so a moved or reworded problem shows
up as one removal and one addition rather than silently matching.
"""
import ast
import inspect
import json
import pathlib
import re

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DOCS_DIR = REPO_ROOT / "docs"
BASELINE_PATH = REPO_ROOT / "tests" / "fixtures" / "docs_known_issues.json"

PYTHON_BLOCK = re.compile(r"```python\n(.*?)```", re.S)
MARKDOWN_LINK = re.compile(r"\]\(([^)]+\.md)\)")


def _documentation_files():
    """
    Returns every documentation file that is checked.

    Returns:
        A sorted list of paths, relative paths being resolved from the repo root.
    """
    return sorted(DOCS_DIR.rglob("*.md")) + [REPO_ROOT / "README.md"]


def _relative(path):
    """Returns `path` relative to the repository root, for stable finding keys."""
    return str(path.relative_to(REPO_ROOT))


def _public_api():
    """
    Returns the classes whose surface the documentation is checked against.

    Returns:
        A mapping of class name to class object.

    Raises:
        pytest.skip.Exception: If the library is not importable.
    """
    try:
        from mks_servo_can import Axis, CANInterface, MultiAxisController
        from mks_servo_can.kinematics import (
            EccentricKinematics,
            LinearKinematics,
            RotaryKinematics,
        )
    except ImportError as exc:  # pragma: no cover - install problem, not a doc problem
        pytest.skip(f"mks_servo_can is not importable: {exc}")

    return {
        "Axis": Axis,
        "CANInterface": CANInterface,
        "MultiAxisController": MultiAxisController,
        "RotaryKinematics": RotaryKinematics,
        "LinearKinematics": LinearKinematics,
        "EccentricKinematics": EccentricKinematics,
    }


# Variables the documentation uses for objects it never constructs in the same
# block. Blocks that do construct their variable are resolved automatically by
# _receiver_types(), so this only covers the ones written as bare examples.
ASSUMED_RECEIVERS = {
    "axis": "Axis",
    "motor1": "Axis",
    "ax": "Axis",
    "axis_x": "Axis",
    "axis_y": "Axis",
    "axis_z": "Axis",
    "my_axis": "Axis",
    "rotary_axis": "Axis",
    "linear_axis": "Axis",
    "can_if": "CANInterface",
    "can_interface": "CANInterface",
    "controller": "MultiAxisController",
    "multi_controller": "MultiAxisController",
}


def _receiver_types(tree, api):
    """
    Works out which variables in a block hold which library objects.

    Prefers evidence over assumption: a block containing `axis = Axis(...)`
    establishes the type directly, and only names with no such assignment fall
    back to `ASSUMED_RECEIVERS`.

    Args:
        tree: The parsed block.
        api: Mapping of class name to class.

    Returns:
        A mapping of variable name to class.
    """
    types = {
        name: api[cls] for name, cls in ASSUMED_RECEIVERS.items() if cls in api
    }
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            func = node.value.func
            cls_name = func.id if isinstance(func, ast.Name) else None
            if cls_name in api:
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        types[target.id] = api[cls_name]
    return types


def _block_fingerprint(block):
    """
    Returns a short, position-independent identifier for a code block.

    Args:
        block: The block's source text.

    Returns:
        The block's first non-empty line, truncated.
    """
    for line in block.splitlines():
        if line.strip():
            return line.strip()[:60]
    return "<empty block>"


def _check_block(block, path, api, findings):
    """
    Checks one code block, appending any findings.

    Args:
        block: The block's source text.
        path: Documentation file the block came from.
        api: Mapping of class name to class.
        findings: List to append `kind|path|detail` strings to.
    """
    rel = _relative(path)
    try:
        tree = ast.parse(block)
    except SyntaxError as exc:
        # Recorded rather than skipped: an unparseable block hides every other
        # problem inside it, so it must be visible in the baseline.
        #
        # Identified by its opening line rather than by its position in the
        # file, so that inserting a block above does not renumber every entry
        # into a wave of false regressions, and so that two broken blocks in one
        # file with the same error message stay distinguishable.
        findings.append(f"unparseable|{rel}|{_block_fingerprint(block)}: {exc.msg}")
        return

    receivers = _receiver_types(tree, api)

    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            cls = api.get(node.func.id)
            if cls is not None:
                valid = set(inspect.signature(cls.__init__).parameters)
                for keyword in node.keywords:
                    if keyword.arg and keyword.arg not in valid:
                        findings.append(
                            f"bad-argument|{rel}|"
                            f"{node.func.id}() has no parameter '{keyword.arg}'"
                        )

        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            receiver = node.func.value
            if isinstance(receiver, ast.Name):
                cls = receivers.get(receiver.id)
                if cls is not None and not hasattr(cls, node.func.attr):
                    findings.append(
                        f"bad-method|{rel}|"
                        f"{cls.__name__} has no method '{node.func.attr}()'"
                    )

        if isinstance(node, ast.ImportFrom) and node.module:
            if not node.module.startswith("mks_servo_can"):
                continue
            for alias in node.names:
                try:
                    module = __import__(node.module, fromlist=[alias.name])
                except ImportError as exc:
                    findings.append(f"bad-import|{rel}|cannot import {node.module}: {exc}")
                    break
                if not hasattr(module, alias.name):
                    findings.append(
                        f"bad-import|{rel}|'{alias.name}' is not in {node.module}"
                    )


def collect_findings():
    """
    Runs every documentation check and returns the findings.

    Returns:
        A sorted list of unique `kind|path|detail` strings.
    """
    api = _public_api()
    findings = []

    for path in _documentation_files():
        text = path.read_text()
        for block in PYTHON_BLOCK.findall(text):
            _check_block(block, path, api, findings)

        # Internal links must resolve. External URLs are not checked - this test
        # must not need a network.
        for link in MARKDOWN_LINK.findall(text):
            if link.startswith(("http://", "https://", "#")):
                continue
            target = (path.parent / link.split("#")[0]).resolve()
            if not target.exists():
                findings.append(f"dead-link|{_relative(path)}|{link}")

    return sorted(set(findings))


def _load_baseline():
    """
    Returns the recorded known problems.

    Returns:
        A sorted list of finding strings, empty if the baseline is absent.
    """
    if not BASELINE_PATH.exists():
        return []
    return sorted(set(json.loads(BASELINE_PATH.read_text())["known_issues"]))


class TestDocumentationMatchesTheAPI:
    """The docs describe a real API, and get no worse than they already are."""

    def test_no_new_problems(self):
        """
        Nothing may be added to the documentation that does not check out.

        A failure here means the block or link you just wrote references
        something that does not exist. Fix it rather than adding it to the
        baseline - the baseline exists for pre-existing debt only.
        """
        new = [f for f in collect_findings() if f not in _load_baseline()]
        assert not new, (
            f"{len(new)} new documentation problem(s):\n  "
            + "\n  ".join(new)
            + "\n\nThese reference things that do not exist. Fix the "
            "documentation; do not add them to "
            f"{_relative(BASELINE_PATH)}."
        )

    def test_baseline_has_no_stale_entries(self):
        """
        The baseline may only shrink.

        An entry that no longer occurs has been fixed, and leaving it recorded
        would silently re-grant permission if the problem came back.
        """
        findings = collect_findings()
        stale = [f for f in _load_baseline() if f not in findings]
        assert not stale, (
            f"{len(stale)} baseline entr(y/ies) no longer occur - well done. "
            f"Delete them from {_relative(BASELINE_PATH)}:\n  "
            + "\n  ".join(stale)
        )

    def test_the_check_actually_detects_something(self):
        """
        Guard against the checker silently doing nothing.

        If a refactor broke block extraction or the API import, every other
        assertion here would pass vacuously. This asserts the machinery still
        finds the code blocks it is supposed to be checking.
        """
        blocks = sum(
            len(PYTHON_BLOCK.findall(p.read_text())) for p in _documentation_files()
        )
        assert blocks > 50, f"only {blocks} python blocks found; extraction is broken"
