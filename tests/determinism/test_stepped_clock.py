"""
The clock that makes simulated time deterministic.

`tests/determinism/` had been an empty placeholder since May 2025. This is the
first thing in it, and the foundation for the rest: every simulator test that
currently paces itself with `asyncio.sleep` against a wall clock is measuring
the machine as much as the motion model.

These tests take no measurable wall-clock time on purpose. If one of them ever
starts taking a second, the clock has fallen back to sleeping.
"""
import asyncio

import pytest

from mks_simulator.clock import (
    DEFAULT_STEP_SECONDS,
    RealTimeClock,
    SteppedClock,
    make_clock,
)


class TestTheClockDoesNotMoveOnItsOwn:
    """The whole point: nothing happens until it is told to."""

    @pytest.mark.asyncio
    async def test_time_starts_where_it_was_told_to(self):
        assert SteppedClock().now() == 0.0
        assert SteppedClock(start=12.5).now() == 12.5

    @pytest.mark.asyncio
    async def test_time_does_not_pass_by_itself(self):
        clock = SteppedClock()
        for _ in range(100):
            await asyncio.sleep(0)
        assert clock.now() == 0.0, "simulated time moved without being advanced"

    @pytest.mark.asyncio
    async def test_advancing_moves_time_by_exactly_that_much(self):
        clock = SteppedClock()
        advanced = await clock.advance(0.25)
        assert advanced == pytest.approx(0.25)
        assert clock.now() == pytest.approx(0.25)

    @pytest.mark.asyncio
    async def test_advancing_is_cumulative(self):
        clock = SteppedClock()
        await clock.advance(0.1)
        await clock.advance(0.2)
        assert clock.now() == pytest.approx(0.3)

    @pytest.mark.asyncio
    @pytest.mark.parametrize("amount", [0.0, -1.0])
    async def test_a_non_positive_advance_does_nothing(self, amount):
        clock = SteppedClock()
        assert await clock.advance(amount) == 0.0
        assert clock.now() == 0.0


class TestParticipantsMoveInLockstep:
    """Every motor must see the same dt, and none may run ahead."""

    @staticmethod
    async def _ticker(clock, samples, stop):
        """
        Stands in for a motor's integration loop.

        Args:
            clock: The clock to tick on.
            samples: List to append the observed time to on each tick.
            stop: Event that ends the loop.
        """
        clock.register()
        try:
            while not stop.is_set():
                await clock.tick()
                samples.append(clock.now())
        finally:
            clock.unregister()

    @pytest.mark.asyncio
    async def test_one_participant_sees_every_substep(self):
        clock = SteppedClock(step_seconds=0.01)
        samples, stop = [], asyncio.Event()
        task = asyncio.create_task(self._ticker(clock, samples, stop))
        await asyncio.sleep(0)

        await clock.advance(0.05)

        assert samples == [
            pytest.approx(t) for t in (0.01, 0.02, 0.03, 0.04, 0.05)
        ], f"expected five 10 ms sub-steps, saw {samples}"

        stop.set()
        await clock.advance(0.01)
        await task

    @pytest.mark.asyncio
    async def test_participants_see_identical_time(self):
        """
        Three tickers, one advance: all three must observe the same sequence.

        If any of them could run ahead, the motors would integrate different
        intervals and a multi-axis result would depend on task scheduling - the
        exact non-determinism this replaces.
        """
        clock = SteppedClock(step_seconds=0.01)
        stop = asyncio.Event()
        samples = [[], [], []]
        tasks = [
            asyncio.create_task(self._ticker(clock, s, stop)) for s in samples
        ]
        await asyncio.sleep(0)

        await clock.advance(0.03)

        assert samples[0] == samples[1] == samples[2], (
            f"participants disagreed about the time: {samples}"
        )
        assert len(samples[0]) == 3

        stop.set()
        await clock.advance(0.01)
        await asyncio.gather(*tasks)

    @pytest.mark.asyncio
    async def test_a_large_advance_is_split_into_substeps(self):
        """
        Ten seconds must not arrive as one dt.

        The motion model integrates acceleration over dt; handed the whole
        interval at once it would overshoot wildly. The sub-step is what keeps a
        big jump faithful to a small one.
        """
        clock = SteppedClock(step_seconds=0.01)
        samples, stop = [], asyncio.Event()
        task = asyncio.create_task(self._ticker(clock, samples, stop))
        await asyncio.sleep(0)

        await clock.advance(1.0)

        assert len(samples) == 100, f"expected 100 sub-steps, got {len(samples)}"
        assert clock.now() == pytest.approx(1.0)

        stop.set()
        await clock.advance(0.01)
        await task

    @pytest.mark.asyncio
    async def test_advance_returns_with_every_participant_parked(self):
        """
        A caller that reads state straight after stepping must see the result.

        If `advance()` returned while a motor was still midway through its
        step, a `/status` read immediately afterwards would catch it half
        updated - which is precisely the race that reading after a sleep has.
        """
        clock = SteppedClock(step_seconds=0.01)
        samples, stop = [], asyncio.Event()
        task = asyncio.create_task(self._ticker(clock, samples, stop))
        await asyncio.sleep(0)

        await clock.advance(0.02)
        observed = list(samples)
        # No further scheduling happens, so nothing may be added behind us.
        for _ in range(50):
            await asyncio.sleep(0)
        assert list(samples) == observed, (
            "a participant was still running after advance() returned"
        )

        stop.set()
        await clock.advance(0.01)
        await task

    @pytest.mark.asyncio
    async def test_a_participant_that_yields_mid_step_is_still_waited_for(self):
        """
        The barrier has to hold for a motor that does not return promptly.

        A real motor's step is not a straight line: it may await a completion
        callback, a queued frame, an anomaly report. Each of those hands the
        event loop back, and without `_settle` waiting for the motor to come
        *round again*, `advance()` would keep moving time while the motor was
        still partway through the previous step - and the motor would miss
        sub-steps entirely.

        A ticker that simply appends and loops does not catch this: it happens
        to be rescheduled in time. This one yields repeatedly, the way a motor
        with work to do does.
        """
        clock = SteppedClock(step_seconds=0.01)
        samples, stop = [], asyncio.Event()

        async def slow_ticker():
            clock.register()
            try:
                while not stop.is_set():
                    await clock.tick()
                    # Work that gives up the loop several times per step.
                    for _ in range(5):
                        await asyncio.sleep(0)
                    samples.append(clock.now())
            finally:
                clock.unregister()

        task = asyncio.create_task(slow_ticker())
        await asyncio.sleep(0)

        await clock.advance(0.05)

        assert len(samples) == 5, (
            f"the slow participant saw {len(samples)} of 5 sub-steps; time ran "
            "ahead of it"
        )
        assert samples == [pytest.approx(t) for t in (0.01, 0.02, 0.03, 0.04, 0.05)]

        stop.set()
        await clock.advance(0.01)
        await task

    @pytest.mark.asyncio
    async def test_a_departed_participant_does_not_deadlock_the_clock(self):
        """
        A motor being torn down must not hang the next advance forever.

        This is why `stop_simulation` unregisters before awaiting the
        cancellation.
        """
        clock = SteppedClock(step_seconds=0.01)
        samples, stop = [], asyncio.Event()
        task = asyncio.create_task(self._ticker(clock, samples, stop))
        await asyncio.sleep(0)
        await clock.advance(0.01)

        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

        # Would hang if the cancelled ticker were still counted.
        advanced = await asyncio.wait_for(clock.advance(0.05), timeout=5.0)
        assert advanced == pytest.approx(0.05)

    @pytest.mark.asyncio
    async def test_advancing_with_no_participants_still_moves_time(self):
        clock = SteppedClock(step_seconds=0.01)
        await asyncio.wait_for(clock.advance(0.1), timeout=5.0)
        assert clock.now() == pytest.approx(0.1)


class TestSimulatedSleep:
    """Delays modelled outside the integration loop use simulated time too."""

    @pytest.mark.asyncio
    async def test_a_sleeper_wakes_when_time_reaches_its_deadline(self):
        clock = SteppedClock(step_seconds=0.01)
        woke = []

        async def sleeper():
            await clock.sleep(0.05)
            woke.append(clock.now())

        task = asyncio.create_task(sleeper())
        await asyncio.sleep(0)

        await clock.advance(0.04)
        assert not woke, f"woke early, at {clock.now()}"

        await clock.advance(0.01)
        await asyncio.wait_for(task, timeout=5.0)
        assert woke and woke[0] == pytest.approx(0.05)

    @pytest.mark.asyncio
    async def test_a_zero_sleep_returns_at_once(self):
        clock = SteppedClock()
        await asyncio.wait_for(clock.sleep(0), timeout=5.0)


class TestDeterminism:
    """The property the whole exercise exists for."""

    @pytest.mark.asyncio
    async def test_identical_runs_produce_identical_timelines(self):
        async def run():
            clock = SteppedClock(step_seconds=0.01)
            samples, stop = [], asyncio.Event()
            task = asyncio.create_task(
                TestParticipantsMoveInLockstep._ticker(clock, samples, stop)
            )
            await asyncio.sleep(0)
            await clock.advance(0.1)
            stop.set()
            await clock.advance(0.01)
            await task
            return samples

        first = await run()
        second = await run()
        assert first == second, "two identical runs produced different timelines"

    @pytest.mark.asyncio
    async def test_time_is_exact_not_merely_close(self):
        """
        Accumulated sub-steps must land on the value asked for, to the bit.

        Time was accumulated as a float, so ten 10 ms steps came to
        0.09999999999999999 and a sleeper due at 0.1 was never woken at all.
        `pytest.approx` would have hidden it; this is why the assertion is `==`.
        """
        clock = SteppedClock(step_seconds=0.01)
        for _ in range(10):
            await clock.advance(0.01)
        assert clock.now() == 0.1

        clock = SteppedClock(step_seconds=0.01)
        await clock.advance(1.0)
        assert clock.now() == 1.0

    @pytest.mark.asyncio
    async def test_an_advance_smaller_than_a_substep_is_still_exact(self):
        """A partial sub-step must move time by exactly that much."""
        clock = SteppedClock(step_seconds=0.01)
        assert await clock.advance(0.004) == 0.004
        assert clock.now() == 0.004


class TestTheDefaultIsUnchanged:
    """Nothing about the wall-clock path may have moved."""

    def test_make_clock_picks_the_right_one(self):
        assert isinstance(make_clock(stepped=False), RealTimeClock)
        assert isinstance(make_clock(stepped=True), SteppedClock)
        assert make_clock(stepped=False).is_stepped is False
        assert make_clock(stepped=True).is_stepped is True

    @pytest.mark.asyncio
    async def test_real_time_advances_by_itself(self):
        clock = RealTimeClock(step_seconds=0.001)
        before = clock.now()
        await clock.tick()
        assert clock.now() > before

    @pytest.mark.asyncio
    async def test_real_time_register_is_harmless(self):
        """The motor calls these unconditionally; on real time they do nothing."""
        clock = RealTimeClock(step_seconds=0.001)
        clock.register()
        clock.unregister()
        await asyncio.wait_for(clock.advance(0.001), timeout=5.0)

    def test_the_default_substep_matches_the_simulators(self):
        """
        The clock's sub-step and the motor model's tick must agree.

        They are declared in different modules; if they drift apart, stepped
        runs integrate at a different resolution from real-time ones and the two
        stop being comparable.
        """
        from mks_simulator.motor_model import SIM_TIME_STEP_MS

        assert DEFAULT_STEP_SECONDS == pytest.approx(SIM_TIME_STEP_MS / 1000.0)
