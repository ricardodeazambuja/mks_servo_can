"""
Where simulated time comes from.

Every simulated motor integrates its position over a `dt` taken from a clock.
By default that clock is the wall clock and the integration loop paces itself
with `asyncio.sleep`, which is what a simulator talking to a real client over a
socket has to do.

Under `--step` the clock is replaced by one that does not move on its own. Time
advances only when `advance()` is called - from `POST /step` on the debug API,
or directly in a test. That buys two things:

- **Determinism.** A move takes exactly the simulated time the motion model says
  it takes, every run, on any machine, under any load. Tests that currently
  measure a move with a stopwatch and compare against a tolerance become exact.
- **No waiting.** Ten seconds of simulated motion costs whatever the arithmetic
  costs, not ten seconds. It also removes the guesswork for anything driving the
  simulator programmatically: command, step, read - with no sleep anywhere, and
  no question of whether you waited long enough.

The stepped clock advances in fixed sub-steps and holds every motor at a barrier
between them, so all motors see the same `dt` and observe it in lockstep. Time
never moves while a motor is midway through a step, which is what makes a run
reproducible rather than merely fast.
"""
import asyncio
import time

# The integration sub-step. Simulated time is advanced in increments of at most
# this, so a single large `advance()` still integrates the motion model at the
# resolution it was written for rather than in one jump.
DEFAULT_STEP_SECONDS = 0.010


class RealTimeClock:
    """
    Simulated time that follows the wall clock.

    The default, and what the simulator has always done: the integration loop
    sleeps for one step and reads `time.monotonic()`.
    """

    #: Whether `advance()` means anything on this clock.
    is_stepped = False

    def __init__(self, step_seconds: float = DEFAULT_STEP_SECONDS):
        """
        Args:
            step_seconds: Nominal interval between integration ticks.
        """
        self._step_seconds = step_seconds

    def now(self) -> float:
        """Returns the current time on a monotonic scale, in seconds."""
        return time.monotonic()

    async def tick(self) -> None:
        """Waits for one integration step to elapse."""
        await asyncio.sleep(self._step_seconds)

    def register(self) -> None:
        """No-op; real time does not wait for anyone."""

    def unregister(self) -> None:
        """No-op; real time does not wait for anyone."""

    async def sleep(self, seconds: float) -> None:
        """
        Waits for `seconds` of simulated time.

        For anything modelling a delay that is not the integration loop - the
        time a homing sequence takes, the time a stop takes to decelerate.

        Args:
            seconds: How long to wait.
        """
        await asyncio.sleep(seconds)

    async def advance(self, seconds: float) -> float:
        """
        Waits out `seconds` of real time.

        Stepping a real-time clock is not an error - it is just a sleep - so
        that a caller need not care which clock is installed.

        Args:
            seconds: How long to advance.

        Returns:
            The time actually advanced.
        """
        await asyncio.sleep(seconds)
        return seconds


class SteppedClock:
    """
    Simulated time that moves only when it is told to.

    Motors call `tick()` and park there. `advance()` releases them one sub-step
    at a time and waits for every one of them to come back before moving time
    again, so no motor can run ahead of another and `dt` is identical for all of
    them.
    """

    is_stepped = True

    def __init__(self, step_seconds: float = DEFAULT_STEP_SECONDS, start: float = 0.0):
        """
        Args:
            step_seconds: Largest increment time may move in one go.
            start: Initial value of the simulated clock.
        """
        # Time is counted in whole nanoseconds, not accumulated as a float.
        # Adding 0.01 to itself five times gives 0.049999999999999996, so a
        # sleeper due at 0.05 was never woken and `now()` differed in its last
        # digits between one run and the next - in a class whose entire purpose
        # is reproducibility. Integers make both exact.
        self._step_ns = max(1, round(step_seconds * 1e9))
        self._step_seconds = self._step_ns / 1e9
        self._now_ns = round(start * 1e9)
        self._participants = 0
        self._waiting = 0
        # Replaced on every release, so a motor released for step N cannot fall
        # through the gate for step N+1 without having gone round the loop.
        self._gate = asyncio.Event()
        # Set while every registered participant is parked at the gate.
        self._quorum = asyncio.Event()
        # Tasks waiting for simulated time to reach a deadline: the homing and
        # stop delays, which are modelled outside the integration loop. They are
        # not barrier participants - `advance()` does not wait for them - but
        # their deadlines are measured on simulated time like everything else,
        # so `--step` does not leave them running on the wall clock.
        self._sleepers: list = []
        # With no participants there is nobody to wait for, so the barrier
        # starts open.
        self._update_quorum()

    def now(self) -> float:
        """Returns the current simulated time in seconds."""
        return self._now_ns / 1e9

    def register(self) -> None:
        """Declares a participant that `advance()` must wait for."""
        self._participants += 1
        self._update_quorum()

    def unregister(self) -> None:
        """
        Withdraws a participant.

        A motor that stops must not leave `advance()` waiting for a tick that
        will never come.
        """
        self._participants = max(0, self._participants - 1)
        self._update_quorum()

    def _update_quorum(self) -> None:
        """
        Sets or clears the "there is nobody left to wait for" flag.

        Note the condition holds when there are *no* participants at all, not
        merely when every one of them is parked. Written the other way round -
        requiring at least one participant - the last motor to unregister while
        `advance()` was inside `_settle()` cleared the flag with nothing left
        that could ever set it again, and the clock hung for good. Tearing a
        motor down is exactly when that happens.
        """
        if self._waiting >= self._participants:
            self._quorum.set()
        else:
            self._quorum.clear()

    async def tick(self) -> None:
        """
        Parks until simulated time is advanced by one sub-step.

        Raises:
            asyncio.CancelledError: If the waiting task is cancelled. The
                participant count is corrected on the way out, so a cancelled
                motor cannot deadlock a later `advance()`.
        """
        gate = self._gate
        self._waiting += 1
        self._update_quorum()
        try:
            await gate.wait()
        finally:
            # `_waiting` counts arrivals at the *current* gate. If the gate we
            # parked on is still current we are leaving without having been
            # released - a cancellation - so our arrival has to be taken back.
            # If it has been swapped, `advance()` has already reset the count
            # for the new gate and decrementing here would push it negative,
            # leaving the barrier permanently satisfied.
            if gate is self._gate:
                self._waiting -= 1
                self._update_quorum()

    async def sleep(self, seconds: float) -> None:
        """
        Waits until simulated time has moved on by `seconds`.

        Args:
            seconds: How much simulated time to wait for.
        """
        if seconds <= 0:
            return
        deadline_ns = self._now_ns + round(seconds * 1e9)
        waiter = asyncio.get_event_loop().create_future()
        self._sleepers.append((deadline_ns, waiter))
        try:
            await waiter
        finally:
            self._sleepers = [
                entry for entry in self._sleepers if entry[1] is not waiter
            ]

    def _wake_sleepers(self) -> None:
        """Resolves every sleeper whose deadline simulated time has passed."""
        still_waiting = []
        for deadline_ns, waiter in self._sleepers:
            if deadline_ns <= self._now_ns and not waiter.done():
                waiter.set_result(None)
            elif not waiter.done():
                still_waiting.append((deadline_ns, waiter))
        self._sleepers = still_waiting

    async def _settle(self) -> None:
        """
        Waits until every participant is parked at the gate.

        Unconditional: the flag already accounts for there being no
        participants, and re-checking `self._participants` here would reintroduce
        the deadlock described on `_update_quorum` - the count can change while
        this is waiting.
        """
        await self._quorum.wait()

    async def advance(self, seconds: float) -> float:
        """
        Moves simulated time forward, running the motors as it goes.

        Args:
            seconds: How far to advance. Non-positive values do nothing.

        Returns:
            The simulated time actually advanced.
        """
        if seconds <= 0:
            return 0.0

        # Whole nanoseconds throughout, so the loop terminates exactly and an
        # advance of 1.0 s is precisely 100 sub-steps of 10 ms rather than 100
        # plus a floating-point crumb.
        remaining_ns = round(seconds * 1e9)
        advanced_ns = 0

        while remaining_ns > 0:
            # Nothing moves until every motor is back at the barrier, so a
            # motor is never observed midway through a step.
            await self._settle()

            step_ns = min(self._step_ns, remaining_ns)
            self._now_ns += step_ns
            advanced_ns += step_ns
            remaining_ns -= step_ns
            self._wake_sleepers()

            # Swap in a fresh gate and reset the arrival count for it. Everyone
            # parked on the old gate is now released, so none of them counts
            # towards the next barrier until it comes round and ticks again.
            released, self._gate = self._gate, asyncio.Event()
            self._waiting = 0
            self._update_quorum()
            released.set()

            # Give the released motors the event loop. `_settle` above is what
            # actually guarantees they finished; this just gets them going.
            await asyncio.sleep(0)

        # Return with every motor parked, so a caller that reads state straight
        # after stepping sees the result of the whole advance.
        await self._settle()
        return advanced_ns / 1e9


def make_clock(stepped: bool, step_seconds: float = DEFAULT_STEP_SECONDS):
    """
    Builds the clock the simulator should run on.

    Args:
        stepped: True for explicit stepping, False for wall-clock time.
        step_seconds: Integration sub-step.

    Returns:
        A `SteppedClock` or a `RealTimeClock`.
    """
    if stepped:
        return SteppedClock(step_seconds=step_seconds)
    return RealTimeClock(step_seconds=step_seconds)
