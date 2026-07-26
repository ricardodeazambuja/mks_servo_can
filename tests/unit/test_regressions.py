"""Regression tests for defects found in the 2026-07 library review.

Each test here pins a specific bug that the pre-existing suite did not catch.
The identifiers (C1..C5, H3) match REVIEW_NOTES.md.

These are deliberately behavioural rather than implementation-shaped: they
assert what a caller observes, so a future refactor of the internals cannot
quietly reintroduce the defect.
"""
import ast
import asyncio
import pathlib
import time
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from mks_servo_can import constants as const
from mks_servo_can.axis import Axis
from mks_servo_can.can_interface import CANInterface
from mks_servo_can.kinematics import RotaryKinematics
from mks_servo_can.low_level_api import LowLevelAPI

LIB_ROOT = pathlib.Path(__file__).resolve().parents[2] / "mks_servo_can_library" / "mks_servo_can"


@pytest.fixture
def mock_can_if():
    """A CANInterface mock whose response futures never resolve on their own."""
    mock = AsyncMock(spec=CANInterface)
    mock.send_message = AsyncMock()
    mock.send_and_wait_for_response = AsyncMock()
    # Synchronous method -> MagicMock, not AsyncMock.
    mock.create_response_future = MagicMock(
        side_effect=lambda *a, **k: asyncio.get_running_loop().create_future()
    )
    mock.expect_stale_notification = MagicMock()
    return mock


@pytest.fixture
def mock_api():
    """A LowLevelAPI mock that reports every move as successfully started."""
    mock = AsyncMock(spec=LowLevelAPI)
    mock.read_encoder_value_addition = AsyncMock(return_value=0)
    mock.read_en_pin_status = AsyncMock(return_value=True)
    mock.enable_motor = AsyncMock()
    for name in (
        "run_position_mode_relative_pulses",
        "run_position_mode_absolute_pulses",
        "run_position_mode_relative_axis",
        "run_position_mode_absolute_axis",
    ):
        setattr(mock, name, AsyncMock(return_value=const.POS_RUN_STARTING))
    return mock


@pytest.fixture
def axis(mock_can_if, mock_api):
    """An Axis wired to mocks, already enabled so moves dispatch immediately."""
    with patch("mks_servo_can.axis.LowLevelAPI", return_value=mock_api):
        ax = Axis(
            can_interface_manager=mock_can_if,
            motor_can_id=1,
            name="RegressionAxis",
            kinematics=RotaryKinematics(
                steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION
            ),
        )
    ax._is_enabled = True
    return ax


# --------------------------------------------------------------------------
# C1: wait=False must return before the move completes.
# --------------------------------------------------------------------------

# The motor never sends a completion frame in these tests, so any code path
# that waits for one will hang until the timeout below fires.
NONBLOCKING_BUDGET_S = 0.5


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "method,args",
    [
        ("move_to_position_abs_user", (90.0,)),
        ("move_relative_user", (45.0,)),
        ("move_to_position_abs_axis", (4096,)),
        ("move_relative_axis", (4096,)),
        ("move_to_position_abs_pulses", (3200,)),
        ("move_relative_pulses", (3200,)),
    ],
)
async def test_c1_wait_false_returns_before_move_completes(axis, method, args):
    """wait=False must dispatch and return, not block for the whole move.

    Regression: Axis._execute_move awaited the completion future
    unconditionally, so wait=False blocked for the full move duration
    (measured 1535 ms for a 90 deg move at 60 deg/s).
    """
    call = getattr(axis, method)
    started = time.perf_counter()
    await asyncio.wait_for(call(*args, wait=False), timeout=NONBLOCKING_BUDGET_S)
    elapsed = time.perf_counter() - started

    assert elapsed < NONBLOCKING_BUDGET_S, (
        f"{method}(wait=False) took {elapsed*1000:.1f} ms; it must return "
        "immediately after dispatching the command"
    )
    # And the move must still be tracked as in flight.
    assert not axis.is_move_complete(), (
        f"{method}(wait=False) returned but left no active move to await"
    )


@pytest.mark.asyncio
async def test_c1_wait_true_still_blocks_until_completion(axis, mock_can_if):
    """The wait=True path must genuinely wait for the completion frame."""
    completion = asyncio.get_running_loop().create_future()
    mock_can_if.create_response_future = MagicMock(return_value=completion)

    task = asyncio.create_task(axis.move_to_position_abs_user(90.0, wait=True))
    await asyncio.sleep(0.05)
    assert not task.done(), "wait=True returned before the motor signalled completion"

    msg = MagicMock()
    msg.data = bytes(
        [const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, const.POS_RUN_COMPLETE]
    )
    completion.set_result(msg)
    await asyncio.wait_for(task, timeout=1.0)
    assert axis.is_move_complete()


@pytest.mark.asyncio
async def test_c1_wait_true_propagates_move_failure(axis, mock_can_if):
    """A failed move must raise out of the wait=True call."""
    from mks_servo_can.exceptions import MotorError

    completion = asyncio.get_running_loop().create_future()
    mock_can_if.create_response_future = MagicMock(return_value=completion)

    task = asyncio.create_task(axis.move_to_position_abs_user(90.0, wait=True))
    await asyncio.sleep(0.05)
    msg = MagicMock()
    msg.data = bytes(
        [const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, const.POS_RUN_FAIL]
    )
    completion.set_result(msg)
    with pytest.raises(MotorError):
        await asyncio.wait_for(task, timeout=1.0)


@pytest.mark.asyncio
async def test_c1_dispatched_move_is_awaitable_later(axis, mock_can_if):
    """A wait=False move must be completable via wait_for_move_completion()."""
    completion = asyncio.get_running_loop().create_future()
    mock_can_if.create_response_future = MagicMock(return_value=completion)

    await asyncio.wait_for(
        axis.move_to_position_abs_user(90.0, wait=False), timeout=NONBLOCKING_BUDGET_S
    )
    waiter = asyncio.create_task(axis.wait_for_move_completion())
    await asyncio.sleep(0.05)
    assert not waiter.done()

    msg = MagicMock()
    msg.data = bytes(
        [const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, const.POS_RUN_COMPLETE]
    )
    completion.set_result(msg)
    await asyncio.wait_for(waiter, timeout=1.0)


# --------------------------------------------------------------------------
# C3: absolute moves must not issue a hidden extra read round-trip.
# --------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_c3_absolute_move_does_not_read_position_first(axis, mock_api):
    """Dispatching an absolute move must not poll the encoder first.

    Regression: _move_absolute_handler called get_current_position_steps()
    before every absolute move purely to decide 'am I already there', adding
    a full CAN round trip to the latency of every commanded move.
    """
    axis._current_position_steps = 0
    mock_api.read_encoder_value_addition.reset_mock()

    await asyncio.wait_for(
        axis.move_to_position_abs_user(90.0, wait=False), timeout=NONBLOCKING_BUDGET_S
    )

    assert mock_api.read_encoder_value_addition.call_count == 0, (
        "absolute move issued "
        f"{mock_api.read_encoder_value_addition.call_count} encoder read(s) "
        "before dispatching; it must use cached position for the no-op check"
    )
    mock_api.run_position_mode_absolute_axis.assert_awaited_once()


@pytest.mark.asyncio
async def test_c3_absolute_move_to_current_position_is_a_noop(axis, mock_api):
    """The 'already there' short-circuit must still work, from a fresh cache."""
    steps = axis.kinematics.user_to_steps(90.0)
    mock_api.read_encoder_value_addition.return_value = steps
    await axis.get_current_position_steps()  # the read is what makes it fresh
    mock_api.read_encoder_value_addition.reset_mock()

    await asyncio.wait_for(
        axis.move_to_position_abs_user(90.0, wait=False), timeout=NONBLOCKING_BUDGET_S
    )
    mock_api.run_position_mode_absolute_axis.assert_not_awaited()
    assert mock_api.read_encoder_value_addition.call_count == 0


@pytest.mark.asyncio
async def test_absolute_move_is_dispatched_when_the_cache_is_stale(axis, mock_api):
    """A stale cache must not be allowed to cancel a move.

    Dispatching a move invalidates the cached position, and it stays invalid
    until the completion watcher refreshes it. A caller who commands the
    position the axis started from before that happens would otherwise hit the
    short-circuit and get success without motion.
    """
    steps = axis.kinematics.user_to_steps(90.0)
    mock_api.read_encoder_value_addition.return_value = steps
    await axis.get_current_position_steps()

    # Dispatch anything: the axis is now somewhere unknown.
    await asyncio.wait_for(
        axis.move_to_position_abs_user(180.0, wait=False), timeout=NONBLOCKING_BUDGET_S
    )
    mock_api.run_position_mode_absolute_axis.reset_mock()
    mock_api.read_encoder_value_addition.reset_mock()

    await asyncio.wait_for(
        axis.move_to_position_abs_user(90.0, wait=False), timeout=NONBLOCKING_BUDGET_S
    )
    mock_api.run_position_mode_absolute_axis.assert_awaited_once()
    assert mock_api.read_encoder_value_addition.call_count == 0, (
        "the stale cache was refreshed with an encoder read instead of simply "
        "dispatching the move, which is the round trip C3 removed"
    )


# --------------------------------------------------------------------------
# C4: no executable statements stranded at class-body scope.
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "module_path", sorted(p for p in LIB_ROOT.rglob("*.py")), ids=lambda p: p.name
)
def test_c4_no_stray_statements_in_class_bodies(module_path):
    """Class bodies must not contain stray calls that run at import time.

    Regression: multi_axis_controller.py had a logger.info() indented at
    class-body scope. It executed once on import and never at the end of the
    method it was meant to belong to.
    """
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for stmt in node.body:
            # Docstrings and other bare literals are fine; bare calls are not.
            if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
                offenders.append(f"{node.name} line {stmt.lineno}")
    assert not offenders, (
        f"{module_path.name}: statements execute at class-definition time: "
        + ", ".join(offenders)
    )


# --------------------------------------------------------------------------
# C5: annotations must not break the minimum supported Python.
# --------------------------------------------------------------------------


def test_c5_no_pep585_annotations_without_future_import():
    """Builtin generics (list[int]) break Python 3.8 unless deferred.

    Regression: crc.py annotated `data_bytes: list[int]`, which is evaluated
    at def time and raises TypeError on 3.8 - a version setup.py claims to
    support.
    """
    offenders = []
    for path in LIB_ROOT.rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        has_future = any(
            isinstance(n, ast.ImportFrom)
            and n.module == "__future__"
            and any(a.name == "annotations" for a in n.names)
            for n in tree.body
        )
        if has_future:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            annotations = [a.annotation for a in node.args.args if a.annotation]
            if node.returns:
                annotations.append(node.returns)
            for ann in annotations:
                for sub in ast.walk(ann):
                    if (
                        isinstance(sub, ast.Subscript)
                        and isinstance(sub.value, ast.Name)
                        and sub.value.id in {"list", "dict", "set", "tuple", "frozenset"}
                    ):
                        offenders.append(
                            f"{path.name}:{node.lineno} {node.name}() uses "
                            f"{sub.value.id}[...]"
                        )
    assert not offenders, (
        "PEP 585 builtin generics in evaluated annotations break Python 3.8; "
        "use typing.List/Dict or `from __future__ import annotations`:\n  "
        + "\n  ".join(offenders)
    )


# --------------------------------------------------------------------------
# H3: the per-frame hot path must not log at INFO.
# --------------------------------------------------------------------------

HOT_PATH_FUNCTIONS = [
    ("low_level_api.py", "_send_command_and_get_response"),
    ("low_level_api.py", "_send_command_no_response"),
    ("low_level_api.py", "_run_motor_command"),
    ("can_interface.py", "send_message"),
    ("can_interface.py", "_process_received_message"),
    ("axis.py", "_execute_move"),
    # Every motion command is per-frame on a streaming path, not just the 0xF5
    # one that was converted first (L5).
    ("low_level_api.py", "run_position_mode_relative_pulses"),
    ("low_level_api.py", "run_position_mode_absolute_pulses"),
    ("low_level_api.py", "run_position_mode_relative_axis"),
    ("low_level_api.py", "run_position_mode_absolute_axis"),
    ("low_level_api.py", "run_speed_mode"),
    ("low_level_api.py", "stop_speed_mode"),
    ("low_level_api.py", "stop_position_mode_relative_axis"),
    ("low_level_api.py", "stop_position_mode_absolute_axis"),
    ("low_level_api.py", "emergency_stop"),
]


@pytest.mark.parametrize("filename,funcname", HOT_PATH_FUNCTIONS)
def test_h3_hot_path_does_not_log_at_info(filename, funcname):
    """Per-frame code must log at DEBUG, not INFO.

    Regression: every CAN frame emitted several logger.info() records with
    eagerly-formatted f-strings (including .hex() calls), so an application
    that enabled INFO logging drowned and paid the formatting cost even when
    the record was discarded.
    """
    tree = ast.parse((LIB_ROOT / filename).read_text(encoding="utf-8"))
    target = next(
        (
            n
            for n in ast.walk(tree)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
            and n.name == funcname
        ),
        None,
    )
    assert target is not None, f"{funcname} not found in {filename}"

    offenders = [
        node.lineno
        for node in ast.walk(target)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"info", "warning"}
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "logger"
        # warning is acceptable for genuine anomalies; only flag info here
        and node.func.attr == "info"
    ]
    assert not offenders, (
        f"{filename}:{funcname} logs at INFO on the per-frame hot path "
        f"(lines {offenders}); use logger.debug()"
    )


def test_h3_hot_path_uses_lazy_log_formatting():
    """Hot-path log calls must not pre-format f-strings.

    logger.debug(f"...") builds the string even when DEBUG is disabled.
    logger.debug("...%s", x) defers it to the handler.
    """
    offenders = []
    for filename, funcname in HOT_PATH_FUNCTIONS:
        tree = ast.parse((LIB_ROOT / filename).read_text(encoding="utf-8"))
        target = next(
            n
            for n in ast.walk(tree)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
            and n.name == funcname
        )
        for node in ast.walk(target):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr in {"debug", "info"}
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "logger"
                and node.args
                and isinstance(node.args[0], ast.JoinedStr)
            ):
                offenders.append(f"{filename}:{node.lineno} in {funcname}()")
    assert not offenders, (
        "eagerly-formatted f-strings in hot-path log calls:\n  "
        + "\n  ".join(offenders)
    )


# --- Move-completion timeout estimation (found on real hardware, 2026-07) ---
#
# Commanding an MKS axis faster than it can actually turn raised a spurious
# CommunicationError: the motor finished the move, but the library had already
# given up waiting. Two causes, both pinned below.


@pytest.mark.parametrize(
    "work_mode",
    [const.MODE_SR_VFOC, const.MODE_SR_CLOSE, const.MODE_CR_CLOSE],
)
def test_speed_param_is_clamped_to_work_mode_not_scaled_by_it(axis, work_mode):
    """The speed parameter is RPM, so a mode with headroom must not change it.

    Manual section 6.1: the parameter *is* RPM (at 16/32/64 subdivisions) and a
    value above the work mode's ceiling makes the motor run at that ceiling.
    The old estimator computed `param / 3000 * ceiling`, which silently halved
    the assumed speed in CLOSE mode and so distorted every timeout.
    """
    speed_param = 1000  # below every ceiling under test
    # Far enough to clear the five-CAN-timeout floor, which would otherwise
    # flatten both branches to the same number and hide a wrong estimate.
    distance = const.ENCODER_PULSES_PER_REVOLUTION * 200

    axis._work_mode = const.MODE_SR_VFOC
    reference = axis._calculate_move_timeout(
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, distance, speed_param, 0
    )
    axis._work_mode = work_mode
    actual = axis._calculate_move_timeout(
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, distance, speed_param, 0
    )

    assert actual == pytest.approx(reference), (
        f"work mode {work_mode} changed the timeout for speed param "
        f"{speed_param} even though the mode's ceiling is above it"
    )


def test_speed_above_work_mode_ceiling_estimates_at_the_ceiling(axis):
    """Over-commanding speed must not shrink the timeout.

    In an OPEN mode the motor tops out at 400 RPM, so asking for 3000 takes
    exactly as long as asking for 400. The old estimator scaled instead of
    clamping and produced a far shorter timeout for the larger number.
    """
    distance = const.ENCODER_PULSES_PER_REVOLUTION * 10
    axis._work_mode = const.MODE_SR_OPEN

    at_ceiling = axis._calculate_move_timeout(
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
        distance, const.MAX_RPM_OPEN_MODE, 0,
    )
    way_over = axis._calculate_move_timeout(
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
        distance, const.MAX_SPEED_PARAM, 0,
    )

    assert way_over == pytest.approx(at_ceiling)


def test_timeout_allows_for_the_acceleration_ramp(axis):
    """A slower ramp must buy the move more time.

    Manual section 6.1: with acceleration `acc` the speed changes by 1 RPM
    every (256 - acc) * ACCEL_TICK_SECONDS seconds, so a small `acc` can
    dominate the move. The old estimator ignored acceleration entirely.
    """
    distance = const.ENCODER_PULSES_PER_REVOLUTION * 10
    speed_param = 600

    def timeout_for(accel):
        return axis._calculate_move_timeout(
            const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, distance, speed_param, accel
        )

    no_ramp = timeout_for(0)          # acc == 0 means "run straight at speed"
    brisk = timeout_for(200)
    sluggish = timeout_for(20)

    assert sluggish > brisk > no_ramp, (
        f"acceleration ignored: acc=20 -> {sluggish:.2f}s, "
        f"acc=200 -> {brisk:.2f}s, acc=0 -> {no_ramp:.2f}s"
    )


def test_long_fast_move_gets_a_timeout_it_can_actually_meet(axis):
    """The field case: 40 revolutions at speed param 600, default acceleration.

    On hardware this move ran for roughly 7.7 s (the motor plateaus below the
    commanded speed and spends real time ramping) while the old estimator
    allowed only ~5.6 s, so a completed move was reported as a timeout.
    """
    axis._work_mode = const.MODE_SR_VFOC
    distance = const.ENCODER_PULSES_PER_REVOLUTION * 40

    timeout = axis._calculate_move_timeout(
        const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS,
        distance,
        600,
        axis.default_accel_param,
    )

    assert timeout > 8.0, (
        f"timeout {timeout:.2f}s is below the ~7.7s this move really takes"
    )


@pytest.mark.asyncio
async def test_return_move_is_not_sized_from_a_stale_cache(axis, mock_api):
    """A move back to a previously occupied position must get a real budget.

    Nothing refreshes the position cache when a move completes, so after
    travelling away from zero the cache still says "at zero". Sizing the next
    move's timeout from that stale value gave a 40-revolution return trip the
    same budget as a standing-still move, and the motor was still going when
    the library gave up.
    """
    far_away = const.ENCODER_PULSES_PER_REVOLUTION * 40

    axis._current_position_steps = 0
    axis._position_cache_is_fresh = True

    captured = []
    original = axis._calculate_move_timeout

    def capture(cmd, pulses, speed, accel=None):
        captured.append(pulses)
        return original(cmd, pulses, speed, accel)

    axis._calculate_move_timeout = capture

    # Travel out. Dispatching leaves the cache stale, and nothing refreshes it.
    await axis.move_to_position_abs_axis(far_away, speed_param=150, wait=False)
    assert not axis._position_cache_is_fresh

    # ...then head back. This is the move that used to get almost no budget.
    await axis.move_to_position_abs_axis(0, speed_param=150, wait=False)

    assert captured[-1] == pytest.approx(far_away, rel=0.01), (
        f"return move sized from {captured[-1]} steps, but the axis is "
        f"{far_away} steps from the target"
    )


@pytest.mark.asyncio
async def test_fresh_cache_still_avoids_the_extra_encoder_read(axis, mock_api):
    """The stale-cache fix must not add a round trip to every absolute move."""
    axis._current_position_steps = 1000
    axis._position_cache_is_fresh = True
    mock_api.read_encoder_value_addition = AsyncMock(return_value=1000)

    await axis.move_to_position_abs_axis(5000, speed_param=150, wait=False)

    mock_api.read_encoder_value_addition.assert_not_called()
