# How to Contribute

Contributions are welcome. This page says what a change needs to carry before it
can be merged, and where to look for something to work on.

## Where to start

[`roadmap.md`](roadmap.md) is the current work list in priority order, with what
"done" means for each item and how to verify it. It is kept honest — items are
removed when they are finished, not when they are started.

`REVIEW_NOTES.md` at the repository root records the defects found in past
reviews, each with the reproduction that proved it. If you are changing code in
an area, it is worth reading what has already gone wrong there.

## Getting set up

See [Setting up a Development Environment](setup.md) and
[Running Tests](running_tests.md).

## The workflow

1. **Fork and branch.** Name it for what it does — `fix/absolute-move-cache`,
   `feature/stepped-time`.
2. **Write the test first, or at least before you claim you are done.** See the
   standard below.
3. **Run the suite and the linter.**
   ```bash
   pytest -q
   ruff check .
   ```
   The suite should be green in a random order too — a test that only passes in
   file order is depending on state a previous test left behind.
4. **Update `CHANGELOG.md`** under `[Unreleased]`, saying what changed and why.
5. **Open a pull request against `main`** describing the change and what
   evidence you have that it works.

## The standard a change is held to

This is stricter than "tests pass", and deliberately so. Every defect fixed in
this repository's recent history was one that the existing suite reported as
working.

* **A fix has a test that fails against the code as it was.** Not a test that
  covers the area — a test that goes red when the fix is removed.

* **The test is verified by mutation.** Undo the fix, run the whole test file,
  confirm the test fails, and confirm the mutation actually landed on disk
  before you conclude anything. Then restore. Restore from a copy you saved
  yourself; `git checkout --` will take your uncommitted work with it.

  If a test survives the mutation, it is not testing what you think. A common
  cause is that some earlier test in the file left behind the state that makes
  it pass.

* **The test asserts an effect, not a call.** Where the motor ended up, what
  frame went out, what the next command received. `assert mock.called` tells you
  nothing about whether the thing worked.

* **The test drives real objects where it can.** The simulator starts in
  milliseconds and the fixtures in `tests/conftest.py` do it for you.
  `MagicMock(spec=SomeClass)` protects you only until a test assigns an
  attribute to it — at which point the mock has learned an API the real class
  does not have, and the suite will pass while the product is broken.

* **A defect found on the way is recorded**, in `REVIEW_NOTES.md` with its
  reproduction and in `CHANGELOG.md` under `[Unreleased]`. The reproduction is
  what makes the fix checkable a year later.

## Documentation changes

`tests/test_docs_api.py` checks that every Python block in `docs/` and
`README.md` refers to things that exist, and that internal links resolve. It is
a ratchet: a new problem fails the build, and so does a baseline entry in
`tests/fixtures/docs_known_issues.json` that no longer occurs.

So: fix the document, run the test, and delete the entries it reports as stale.
Never add an entry — a new finding means the text you just wrote describes an
API that is not there.

Prefer pointing at something executable over restating it in prose. The scripts
in `examples/` are exercised; a paragraph describing what they do is not, and
will drift.

## Protocol changes

The MKS manual is transcribed into
`mks_servo_can_library/mks_servo_can/data/manual_commands_v106.json`, which
ships inside the wheel because both the simulator and the conformance tests read
it at runtime. If you change how a command is encoded, change it there too — and
if the manual is ambiguous about the point you are changing, add to the `errata`
block rather than picking a reading silently.

Be aware of what the simulator can and cannot prove. It was written from the
same reading of the manual as the library, so it will agree with a mistake as
readily as with a correct implementation. A change that turns on a genuinely
new behaviour wants a hardware trace behind it; see `tests/hil/`.

## Reporting a bug

Include what you ran, what you expected, and what happened — and if you can, the
smallest script against the simulator that shows it. A reproduction against
`mks-servo-simulator` can be run by anyone; one that needs your bench cannot.
