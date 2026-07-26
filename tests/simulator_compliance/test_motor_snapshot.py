"""
The simulator's reporting surfaces must describe the motor that actually exists.

Every one of these tests drives a real `SimulatedMotor` and checks that
`status_snapshot()` follows it. None of them use a mock, and that is the entire
point.

The interface layer previously read sixteen attributes from the motor that the
motor had never had - `enabled`, `encoder_position`, `current_speed`,
`is_moving` and so on - each wrapped in `getattr(motor, name, default)`. The
mismatch therefore did not raise; it reported the defaults. Every field an agent
read back was a constant zero, and the one attribute without a default (`name`)
returned HTTP 500 from `/status` and crashed `--json-output` on startup.

The suite stayed green throughout, because it built `MagicMock(spec=SimulatedMotor)`
and then assigned the fictitious attributes to it. `spec` restricts what you may
*read* from a mock only until you write to it, so the mock grew exactly the API
the code wished for, and the real class was never consulted.

So: no mock motors here. `test_snapshot_fields_are_backed_by_real_state` is the
specific guard against that failure returning - it asserts the snapshot changes
when the motor changes, which a fabricated default cannot do.
"""
import asyncio

import pytest

from mks_servo_can import constants as const
from mks_simulator.motor_model import MotorSnapshot, SimulatedMotor


@pytest.fixture
def motor(event_loop_for_motor):
    """A real simulated motor, with no simulation task running."""
    return SimulatedMotor(can_id=1, loop=event_loop_for_motor)


@pytest.fixture
def event_loop_for_motor():
    """
    An event loop for motors that are inspected but never stepped.

    `SimulatedMotor` needs a loop reference at construction time to create
    futures; these tests drive its state directly rather than running it.
    """
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


class TestSnapshotTruthfulness:
    """The snapshot must reflect the motor, not a plausible-looking default."""

    def test_snapshot_fields_are_backed_by_real_state(self, motor):
        """
        Mutating the motor must change the snapshot.

        This is the direct guard against the defaulted-getattr bug: a field
        wired to a constant passes a "has the right keys" test but fails this
        one, because a constant cannot follow the motor.
        """
        before = motor.status_snapshot()

        motor.is_enabled = True
        motor.position_steps = 4096.0
        motor.target_position_steps = 8192.0
        motor.current_rpm = 120.0
        motor.target_rpm = 240.0
        motor.motor_status_code = const.MOTOR_STATUS_FULL_SPEED
        motor.is_calibrated = False
        motor.is_homed = True
        motor.is_stalled = True
        motor.microsteps = 32
        motor.working_current_ma = 2400
        motor.work_mode = const.MODE_SR_CLOSE
        motor.slave_respond_enabled = False
        motor.slave_active_initiation_enabled = False
        motor.target_accel_mks = 250

        after = motor.status_snapshot()

        changed = {
            field
            for field in MotorSnapshot.__dataclass_fields__
            if getattr(before, field) != getattr(after, field)
        }
        # Everything touched above, plus the values derived from them.
        expected_changed = {
            "enabled", "position_steps", "position_degrees",
            "target_position_steps", "target_position_degrees",
            "position_error_steps", "current_rpm", "target_rpm",
            "speed_deg_per_s", "moving", "status_code", "status_text",
            "calibrated", "homed", "stalled", "microsteps",
            "working_current_ma", "work_mode", "work_mode_name",
            "responses_enabled", "active_notifications_enabled",
            "accel_param", "accel_deg_per_s2",
        }
        assert changed == expected_changed, (
            "snapshot fields that did not track the motor: "
            f"{sorted(expected_changed - changed)}; unexpected changes: "
            f"{sorted(changed - expected_changed)}"
        )

    def test_no_field_is_silently_defaulted(self, motor):
        """
        Every snapshot field must come from an attribute the motor really has.

        Reading the snapshot with `getattr` disabled on the motor would be the
        purest form of this check; short of that, asserting the motor carries
        every source attribute catches a renamed field at the point of the
        rename rather than three surfaces downstream.
        """
        required_attributes = [
            "original_can_id", "can_id", "motor_type", "is_enabled",
            "is_calibrated", "is_homed", "motor_status_code", "position_steps",
            "target_position_steps", "current_rpm", "target_rpm", "work_mode",
            "microsteps", "steps_per_rev_encoder", "working_current_ma",
            "holding_current_percentage_code", "is_stalled",
            "is_protected_by_stall", "is_protected_by_pos_error",
            "slave_respond_enabled", "slave_active_initiation_enabled",
            "target_accel_mks",
        ]
        missing = [a for a in required_attributes if not hasattr(motor, a)]
        assert not missing, f"SimulatedMotor is missing {missing}"

    def test_angles_derive_from_the_motors_own_resolution(self, event_loop_for_motor):
        """
        A non-standard encoder must not be reported through a hardcoded 16384.
        """
        motor = SimulatedMotor(
            can_id=1, loop=event_loop_for_motor, steps_per_rev_encoder=4000
        )
        motor.position_steps = 1000.0
        assert motor.status_snapshot().position_degrees == pytest.approx(90.0)

        standard = SimulatedMotor(can_id=2, loop=event_loop_for_motor)
        standard.position_steps = const.ENCODER_PULSES_PER_REVOLUTION / 4
        assert standard.status_snapshot().position_degrees == pytest.approx(90.0)

    def test_idle_motor_reports_no_target(self, motor):
        """A motor with no move in flight reports None, not a misleading zero."""
        snapshot = motor.status_snapshot()
        assert snapshot.target_position_steps is None
        assert snapshot.target_position_degrees is None
        assert snapshot.position_error_steps is None
        assert snapshot.moving is False

    def test_position_error_is_signed_distance_remaining(self, motor):
        """Error must be target minus current, so its sign gives direction."""
        motor.position_steps = 1000.0
        motor.target_position_steps = 1500.0
        assert motor.status_snapshot().position_error_steps == pytest.approx(500.0)

        motor.target_position_steps = 500.0
        assert motor.status_snapshot().position_error_steps == pytest.approx(-500.0)

    def test_snapshot_is_json_serialisable(self, motor):
        """The agent surfaces serialise this directly; it must survive that."""
        import json

        payload = json.dumps(motor.status_snapshot().as_dict())
        assert json.loads(payload)["can_id"] == 1

    def test_snapshot_is_immutable(self, motor):
        """
        A snapshot is a reading, not a handle.

        Freezing it stops a renderer from "fixing up" a value in place and
        making two surfaces disagree about the same instant.
        """
        snapshot = motor.status_snapshot()
        with pytest.raises(AttributeError):
            snapshot.position_steps = 42.0  # type: ignore[misc]

    def test_listening_id_is_distinct_from_reporting_id(self, motor):
        """
        After a 0x8B the motor answers on a new ID but still reports its own.

        Collapsing these two into one field is how the CAN-ID change command
        becomes invisible to an operator watching the dashboard.
        """
        motor.can_id = 0x123
        snapshot = motor.status_snapshot()
        assert snapshot.can_id == 1
        assert snapshot.listening_can_id == 0x123


class TestAnomalyReporting:
    """
    The simulator must surface what a client cannot deduce from the wire.

    A superseded move is the case that matters. The abort frame for the old
    move and the acknowledgement of the new one carry the same command byte, so
    from the client's side a retarget looks like a move that failed for no
    reason. The simulator knows which frame is which and must say so.
    """

    @pytest.mark.asyncio
    async def test_superseding_a_move_is_reported(self):
        """Retargeting mid-move must produce a labelled anomaly, not silence."""
        loop = asyncio.get_running_loop()
        motor = SimulatedMotor(can_id=1, loop=loop)
        motor.is_enabled = True
        recorded = []
        motor.set_anomaly_sink(
            lambda mid, kind, desc, ctx: recorded.append((mid, kind, desc, ctx))
        )
        await motor.start()
        try:
            async def _noop(_id, _payload):
                return None

            def _move_payload(counts):
                return bytes(
                    [0x00, 0x64, 250,
                     (counts >> 16) & 0xFF, (counts >> 8) & 0xFF, counts & 0xFF]
                )

            motor.process_command(
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, _move_payload(16384), _noop
            )
            await asyncio.sleep(0.1)
            assert recorded == [], "no anomaly should be reported for the first move"

            # Retarget while the first move is still running.
            motor.process_command(
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, _move_payload(8192), _noop
            )
            await asyncio.sleep(0.1)

            assert len(recorded) == 1, f"expected one anomaly, got {recorded}"
            motor_id, kind, description, context = recorded[0]
            assert motor_id == 1
            assert kind == "move_superseded"
            assert context["new_target_steps"] == pytest.approx(8192)
            assert context["superseded_command"] == (
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS
            )
            # The description must explain the ambiguity, since that is the
            # whole reason the event is worth reporting.
            assert "acknowledgement" in description
        finally:
            await motor.stop_simulation()

    def test_anomalies_are_silent_without_a_sink(self):
        """A motor with no sink attached must not raise when reporting."""
        loop = asyncio.new_event_loop()
        try:
            motor = SimulatedMotor(can_id=1, loop=loop)
            motor.report_anomaly("test", "no sink attached", detail=1)
        finally:
            loop.close()


class TestSnapshotTracksSimulation:
    """The snapshot must follow the motor while it is genuinely running."""

    @pytest.mark.asyncio
    async def test_snapshot_follows_a_real_move(self):
        """
        Drive an actual positional move and watch the snapshot track it.

        This exercises the whole chain - command parsing, the integration loop,
        and the snapshot - rather than asserting on hand-set attributes.
        """
        loop = asyncio.get_running_loop()
        motor = SimulatedMotor(can_id=1, loop=loop)
        motor.is_enabled = True
        await motor.start()
        try:
            async def _noop(_id, _payload):
                return None

            target_counts = 8192
            payload = bytes(
                [
                    0x00, 0xC8,  # speed 200
                    250,         # acceleration
                    (target_counts >> 16) & 0xFF,
                    (target_counts >> 8) & 0xFF,
                    target_counts & 0xFF,
                ]
            )
            motor.process_command(
                const.CMD_RUN_POSITION_MODE_ABSOLUTE_AXIS, payload, _noop
            )

            # Let the move get under way, then check the snapshot describes it.
            await asyncio.sleep(0.15)
            moving = motor.status_snapshot()
            assert moving.moving is True
            assert moving.target_position_steps == pytest.approx(target_counts)
            assert moving.current_rpm > 0
            assert moving.position_error_steps > 0
            assert moving.status_text != ""

            for _ in range(200):
                await asyncio.sleep(0.05)
                if motor.status_snapshot().target_position_steps is None:
                    break

            done = motor.status_snapshot()
            assert done.position_steps == pytest.approx(target_counts, abs=1.0)
            assert done.moving is False
        finally:
            await motor.stop_simulation()
