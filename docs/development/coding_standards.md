# Coding Standards

## The tool

`ruff` is the only linter, configured once in `pyproject.toml` under
`[tool.ruff]`. It replaces the black + isort + flake8 + pylint combination that
this project used to carry configuration for — none of which was ever actually
run anywhere.

```bash
ruff check .
```

CI runs exactly that and it must be clean. There is no per-package config file;
if your editor disagrees with CI, point it at `ruff` and the repository root.

## What is enforced

The selected rule sets are `E`/`W` (pycodestyle), `F` (pyflakes), `I` (import
sorting), `B` (bugbear), `UP` (pyupgrade), `C4` (comprehensions) and `RUF`.
Line length is 88.

Imports are sorted by `ruff`, with `mks_servo_can` and `mks_simulator` treated
as first-party.

## What is deliberately not enforced

Two categories, both with reasons recorded in `pyproject.toml`:

**Python 3.9 compatibility.** `UP006`, `UP007` and `UP035` are off. 3.9 is the
support floor, so the codebase uses `typing.List` and `typing.Optional` rather
than `list[...]` and `X | Y`. `tests/unit/test_regressions.py` enforces that the
package still imports on 3.9, and the CI matrix runs it. Do not "modernise"
these annotations.

**Cosmetics, for now.** `W291`, `W293`, `E701`, `E702`, `RUF005`, `RUF046`,
`RUF012` and `RUF043` are switched off, and `ruff format` is not run in CI. The
codebase predates any linter being run; turning them on means a whole-repository
reformat that would bury every real change under whitespace noise. The intent is
to enable them together with a single formatting commit at a moment when nothing
is in flight — not to leave them off forever.

`E501` is off because line length belongs to the formatter.

Per-file exemptions are listed in `[tool.ruff.lint.per-file-ignores]`: the star
import of `constants` is the package's documented public surface, tests may use
bare asserts, and `examples/` may carry illustrative unused imports and
`sys.path` manipulation before imports.

## Docstrings

Every public module, class and function gets a docstring, in Google style —
`Args:`, `Returns:`, `Raises:`. This is not decoration. The API reference for
this project is the docstrings; prose pages that restate a signature drift away
from it within a release, and several of the ones in `docs/` did exactly that.

Two things worth putting in a docstring that are easy to leave out:

* **`Raises:`** — this is an asyncio library talking to hardware over a bus.
  What a call does when the motor does not answer is part of its contract, not
  an implementation detail.
* **Why, where the *what* is not enough.** The manual this protocol comes from
  is ambiguous in places and self-contradictory in a few; where the code has
  made a decision about a reading, the docstring is where that decision is
  recorded. `mks_servo_can/data/manual_commands_v106.json` carries an `errata`
  block for the same purpose.

## Type hints

Public functions are annotated. `mypy` is available in the `[dev]` extra but is
not run in CI and the codebase does not currently pass it clean — treat hints as
documentation that the reader and the editor can both use, and do not remove
them.

## Naming and structure

* Modules, functions and variables `snake_case`; classes `CamelCase`; constants
  `UPPER_SNAKE_CASE`.
* Protocol constants belong in `constants.py`, not inline in the call site. A
  bare `0xF5` in the middle of a method is how a misreading of the manual gets
  duplicated into three places.
* Anything user-facing goes through the library's own exceptions in
  `exceptions.py` rather than bare `ValueError`/`RuntimeError`.

## The one that matters most here

**Do not report success you have not achieved.** The recurring defect in this
codebase is not the crash — it is the silent no-op: a status endpoint returning
zeros for a moving motor, an absolute move skipped against a stale cache that
returned success without sending a frame, a playback that printed
`PLAYBACK COMPLETE` after failing to command anything.

They share a shape. Somewhere a failure is caught, or a shortcut is taken, and
the caller is told everything is fine. When you write a `try`/`except` here, or
an early return that skips work, ask what the caller now believes — and if the
answer is "something that might not be true", make the failure visible.

`REVIEW_NOTES.md` records each of these with its original reproduction.
