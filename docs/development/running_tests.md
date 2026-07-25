# Running Tests

`pytest` is the runner and `pytest.ini` at the repository root already selects
`tests/` and registers the markers, so from the project root:

```bash
pytest -q
```

is the whole story for the common case. Nothing needs to be plugged in: the
suite starts its own simulator subprocesses and the hardware tests skip
themselves.

## The suites

| directory | what it covers | needs |
|---|---|---|
| `tests/unit/` | library logic in isolation — kinematics, CRC, protocol encoding, the digitizer's data structures | nothing |
| `tests/integration/` | the library driving a real simulator over a socket: moves, supersession, streaming, digitizer playback | a simulator, started for you |
| `tests/simulator_compliance/` | that the simulator's wire format matches the packaged transcription of the MKS manual | a simulator, started for you |
| `tests/hil/` | the same protocol against a physical motor | CAN hardware; **skips** without it |
| `tests/determinism/` | reserved for tests under simulated time; currently a placeholder | — |
| `tests/test_docs_api.py` | that this documentation describes an API that exists | nothing |

Run one of them by path:

```bash
pytest tests/unit
pytest tests/integration/test_move_supersede.py -v
```

The markers declared in `pytest.ini` (`integration`, `compliance`,
`performance`, `hil`) can also be used to select or exclude:

```bash
pytest -m "not integration"
```

## Simulator fixtures

Tests that need a motor do not stub one — they drive the real simulator. The
fixtures are in `tests/conftest.py`:

| fixture | motors | port | latency |
|---|---|---|---|
| `basic_simulator` | 2, CAN IDs 1–2 | 6789 | 1 ms |
| `compliance_simulator` | 3, CAN IDs 1–3 | 6790 | 1 ms |
| `performance_simulator` | 4, CAN IDs 1–4 | 6791 | 5 ms |

Each has a matching `*_can_interface` fixture yielding a connected
`CANInterface`, and a `*_api` fixture yielding a `LowLevelAPI` on top of it —
so `basic_api`, `compliance_api` and `performance_api` are usually what a test
actually asks for. The simulator fixtures are module-scoped; the interfaces and
APIs are function-scoped.

Two things follow from this that are worth knowing before you debug a strange
failure:

* **A stray simulator on one of those ports will silently corrupt the run.** The
  fixture starts its own process, but a leftover one from an interrupted session
  answers on the same port and the tests will happily talk to it — with whatever
  state the previous run left in it. If results stop making sense, look for
  orphaned `mks-servo-simulator` processes first.
* **CAN ID 99 answers nothing.** That is how timeout and error paths are
  provoked in the compliance suite without stubbing anything out.

## Coverage

Coverage is configured in `pyproject.toml` and scoped to the library:

```bash
pytest --cov=mks_servo_can_library/mks_servo_can --cov-report=term-missing
```

CI runs this on every matrix entry and uploads `coverage.xml` from the 3.11 job.
Coverage is a symptom, not a target: a module at 90% whose tests only assert that
calls were made is worse than one at 40% with three tests that assert effects.

## Hardware-in-the-loop

`tests/hil/test_hardware_conformance.py` talks to a physical SERVO42D/57D. It
skips unless `MKS_HIL_CHANNEL` is set:

```bash
export MKS_HIL_CHANNEL=can0          # or /dev/ttyACM0 for an slcan adapter
export MKS_HIL_INTERFACE=socketcan   # default: socketcan
export MKS_HIL_BITRATE=500000        # default: 500000
export MKS_HIL_CAN_ID=1              # default: 1
pytest tests/hil
```

The tests that rotate the shaft are separately gated — they need
`MKS_HIL_ALLOW_MOTION=1` as well, so that pointing the suite at a motor bolted
into a machine cannot move it by accident.

The suite can also record what the hardware actually did:

```bash
pytest tests/hil --hil-record=tests/fixtures/hardware_trace.json
```

One capture then replays in CI forever with no hardware attached. This matters
more than it looks: the simulator was written from the manual and so was the
library, so a shared misreading of the manual is invisible to every other test
here. See [the roadmap](roadmap.md) for the specific questions a trace settles.

## Writing tests for this codebase

Three habits, each of which exists because its absence let broken code ship
green:

1. **Assert the effect, not the call.** That a method was invoked says nothing
   about whether it worked. Assert where the motor ended up, what frame went
   out, what the next command saw.
2. **Prefer real objects to mocks.** `MagicMock(spec=X)` constrains a mock only
   until something writes to it; a test that assigns the attribute it wants has
   taught the mock an API the real class never had, and will pass while
   production returns HTTP 500. The simulator is cheap to start — use it.
3. **Verify a new guard test by mutation.** Undo the fix it is meant to guard,
   confirm the test fails, confirm the mutation actually landed on disk, then
   restore. A test that still passes under mutation is not a test. Restore from
   a copy you saved yourself — never `git checkout --`, which will take
   uncommitted work with it.

## Linting

```bash
ruff check .
```

`ruff` replaces the black/isort/flake8/pylint combination that was configured
here previously and never run. See [Coding Standards](coding_standards.md) for
what is enforced and what is deliberately not.

## Documentation checks

`tests/test_docs_api.py` parses every Python block in `docs/` and `README.md`,
checks that the classes, methods, constructor arguments and imports they
mention actually exist, and checks that internal links resolve.

It is a **ratchet**. Pre-existing problems live in
`tests/fixtures/docs_known_issues.json`; the test fails if a new one appears,
*and* fails if a baseline entry no longer occurs, telling you to delete it. The
list can only shrink. If you hit a failure for something you just wrote, fix the
documentation — do not add it to the baseline.
