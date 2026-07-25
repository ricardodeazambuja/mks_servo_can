"""
An in-process simulator on a stepped clock, for tests that measure time.

The wall-clock integration tests each drive a real `Axis` over a socket to a
simulator *subprocess*, so they measure the machine they run on as much as the
motion model: their assertions carry tolerances of 25%, 50 ms and a ratio, and
they cost seconds of real time apiece. Swapping the clock is not enough on its
own - a subprocess has its own event loop, and `POST /step` blocks on a barrier
the client's motor can never reach.

What this module provides instead is the simulator running **inside the test's
own event loop**, on a `SteppedClock`, with the library connecting to it over a
loopback socket exactly as it would to the real thing. Nothing about the client
changes; only where time comes from.

Time then moves only when `run_under_simulated_time` advances it, so a move
takes the simulated duration the motion model says it takes, on any machine,
under any load. Two caveats worth stating plainly:

- **Bus latency is set to zero.** `--step` governs the motors; the socket and
  the client's own code still run in real time, so a fully deterministic
  measurement needs nothing else in the path taking time.
- **The measurement is not exact to the nanosecond.** Time is advanced in
  fixed steps while the awaited operation runs, so a result is observed at most
  one step after it actually happened. With `STEP_SECONDS` at 1 ms, a move of a
  few hundred milliseconds is measured to within a couple of milliseconds -
  three orders of magnitude tighter than a stopwatch and a 25% tolerance, but
  not the bit-for-bit determinism that `tests/determinism/` gets by driving the
  motor model directly with no socket in the way.
"""
import asyncio
import contextlib
from typing import Any, Awaitable, List, Tuple

from mks_servo_can import CANInterface
from mks_servo_can import constants as const
from mks_simulator.clock import SteppedClock
from mks_simulator.motor_model import SimulatedMotor
from mks_simulator.virtual_can_bus import VirtualCANBus

#: How far simulated time moves per pump iteration. Small enough that a
#: measurement is tight, large enough that a second of simulated motion does not
#: cost a thousand round trips through the event loop.
STEP_SECONDS = 0.001

#: Simulated seconds after which a pumped operation is considered hung. Real
#: time never enters into it, so this cannot fire because the machine is busy.
DEFAULT_LIMIT_SECONDS = 120.0


class SteppedSimulator:
    """
    A simulator, its clock and a connected `CANInterface`, all in one loop.

    Attributes:
        clock: The `SteppedClock` every motor integrates against.
        bus: The `VirtualCANBus` serving the loopback socket.
        motors: The simulated motors, in CAN ID order.
        can_interface: A connected `CANInterface` for the library to use.
    """

    def __init__(self, clock, bus, motors, can_interface, server, motor_tasks):
        self.clock: SteppedClock = clock
        self.bus: VirtualCANBus = bus
        self.motors: List[SimulatedMotor] = motors
        self.can_interface: CANInterface = can_interface
        self._server = server
        self._motor_tasks = motor_tasks

    async def close(self) -> None:
        """Disconnects the client and shuts the simulator down."""
        with contextlib.suppress(Exception):
            await self.can_interface.disconnect()

        self._server.close()
        with contextlib.suppress(Exception):
            await self._server.wait_closed()

        for motor in self.motors:
            with contextlib.suppress(Exception):
                await motor.stop_simulation()

        for task in self._motor_tasks:
            task.cancel()
        await asyncio.gather(*self._motor_tasks, return_exceptions=True)


async def start_stepped_simulator(
    num_motors: int = 1,
    start_can_id: int = 1,
    steps_per_rev: int = const.ENCODER_PULSES_PER_REVOLUTION,
) -> SteppedSimulator:
    """
    Starts a simulator in this event loop with time under the test's control.

    Args:
        num_motors: How many motors to serve.
        start_can_id: CAN ID of the first motor; the rest follow it.
        steps_per_rev: Encoder counts per revolution for every motor.

    Returns:
        A `SteppedSimulator`, whose `can_interface` is already connected.
    """
    loop = asyncio.get_running_loop()
    clock = SteppedClock()

    bus = VirtualCANBus(loop)
    bus.simulation_clock = clock
    # `--step` governs the motors only. Anything else in the path that takes
    # real time reintroduces exactly the jitter this exists to remove.
    bus.set_latency(0)

    motors = []
    for offset in range(num_motors):
        motor = SimulatedMotor(
            can_id=start_can_id + offset,
            loop=loop,
            clock=clock,
            steps_per_rev_encoder=steps_per_rev,
        )
        bus.add_motor(motor)
        motors.append(motor)

    # Port 0 asks the OS for a free one, so a stray simulator on 6789/6790/6791
    # cannot quietly answer in place of this one.
    server = await asyncio.start_server(bus.client_handler_loop, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]

    motor_tasks = []
    for motor in motors:
        await motor.start()
        if motor.is_running_task is not None:
            motor_tasks.append(motor.is_running_task)
    # Let every motor reach its first barrier before time is allowed to move.
    await asyncio.sleep(0)

    can_interface = CANInterface(
        use_simulator=True, simulator_host="127.0.0.1", simulator_port=port
    )
    await can_interface.connect()

    return SteppedSimulator(clock, bus, motors, can_interface, server, motor_tasks)


async def run_under_simulated_time(
    clock: SteppedClock,
    awaitable: Awaitable,
    step: float = STEP_SECONDS,
    limit: float = DEFAULT_LIMIT_SECONDS,
) -> Tuple[Any, float]:
    """
    Runs an operation to completion while advancing simulated time.

    The operation is a normal client call - a move, a read, a playback. It
    blocks on frames that will only arrive once the motors have moved, and the
    motors only move when this advances the clock, so the two have to be driven
    together.

    Args:
        clock: The stepped clock the motors are registered against.
        awaitable: The client operation to drive.
        step: How far to advance per iteration.
        limit: Simulated seconds after which the operation is declared hung.

    Returns:
        A `(result, simulated_elapsed_seconds)` pair.

    Raises:
        asyncio.TimeoutError: If `limit` simulated seconds pass first.
    """
    task = asyncio.ensure_future(awaitable)
    started = clock.now()
    try:
        while not task.done():
            await clock.advance(step)
            # Give the loop a turn so frames produced by that step reach the
            # client before more time passes.
            await asyncio.sleep(0)
            await asyncio.sleep(0)
            if clock.now() - started > limit:
                task.cancel()
                with contextlib.suppress(Exception):
                    await task
                raise asyncio.TimeoutError(
                    f"operation did not finish within {limit}s of simulated time"
                )
    except BaseException:
        if not task.done():
            task.cancel()
            with contextlib.suppress(Exception):
                await task
        raise

    return await task, clock.now() - started


async def settle(clock: SteppedClock, seconds: float, step: float = STEP_SECONDS) -> None:
    """
    Advances simulated time with nothing awaited, letting motion finish.

    Args:
        clock: The stepped clock.
        seconds: How much simulated time to pass.
        step: How far to advance per iteration.
    """
    target = clock.now() + seconds
    while clock.now() < target:
        await clock.advance(step)
        await asyncio.sleep(0)
