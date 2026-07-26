"""
`POST /step` - the way a client outside the process drives simulated time.

The endpoint is what makes stepped time usable by anything that is not a Python
test: an agent, a script, a debugger. Its contract is that after the response
comes back, the state in that response is settled - so a caller can command,
step and read with no sleeping and no guessing.

These tests serve the API from a real debug interface over real motors, with
nothing stubbed in between. The alternative - stubbing `get_system_status` - is
how `/status` and `/health` both once shipped returning HTTP 500 with the suite
green.
"""
import asyncio

import pytest

fastapi = pytest.importorskip("fastapi", reason="the debug API needs fastapi")
httpx = pytest.importorskip("httpx", reason="the debug API tests need httpx")

import pytest_asyncio  # noqa: E402

from mks_servo_can import constants as const  # noqa: E402
from mks_simulator.clock import RealTimeClock, SteppedClock  # noqa: E402
from mks_simulator.interface.http_debug_server import DebugHTTPServer  # noqa: E402
from mks_simulator.interface.llm_debug_interface import (  # noqa: E402
    LLMDebugInterface,
)
from mks_simulator.motor_model import SimulatedMotor  # noqa: E402
from mks_simulator.virtual_can_bus import VirtualCANBus  # noqa: E402

STEPS_PER_REV = const.ENCODER_PULSES_PER_REVOLUTION


def _asgi_client(app):
    """
    Builds an HTTP client that drives the app in the *caller's* event loop.

    Deliberately not `fastapi.testclient.TestClient`: that runs the application
    on a loop of its own, on another thread. The motor's integration task lives
    on this loop, so `POST /step` would block on the stepped clock's barrier
    waiting for a motor that could never reach it - a deadlock, not a failure,
    which is a much worse way to find out.

    Args:
        app: The ASGI application.

    Returns:
        An `httpx.AsyncClient` bound to it.
    """
    return httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app), base_url="http://sim"
    )


@pytest_asyncio.fixture
async def stepped_api():
    """
    A debug API over one real motor on a stepped clock.

    Yields:
        A tuple of (AsyncClient, SimulatedMotor, SteppedClock).
    """
    loop = asyncio.get_event_loop()
    clock = SteppedClock()

    bus = VirtualCANBus(loop)
    bus.simulation_clock = clock
    motor = SimulatedMotor(can_id=1, loop=loop, clock=clock)
    motor.is_enabled = True
    bus.add_motor(motor)
    await motor.start()
    await asyncio.sleep(0)

    interface = LLMDebugInterface(motors=bus.simulated_motors, can_bus=bus)
    bus.debug_interface = interface

    async with _asgi_client(DebugHTTPServer(debug_interface=interface).app) as client:
        yield client, motor, clock

    await motor.stop_simulation()


@pytest.mark.asyncio
async def test_stepping_reports_what_it_did(stepped_api):
    client, _motor, clock = stepped_api

    response = await client.post("/step", json={"seconds": 0.25})

    assert response.status_code == 200, response.text
    body = response.json()
    assert body["stepped"] is True
    assert body["advanced_seconds"] == pytest.approx(0.25)
    assert body["simulated_time"] == pytest.approx(0.25)
    assert clock.now() == pytest.approx(0.25)


@pytest.mark.asyncio
async def test_the_default_step_is_a_tenth_of_a_second(stepped_api):
    client, _motor, _clock = stepped_api
    body = (await client.post("/step", json={})).json()
    assert body["advanced_seconds"] == pytest.approx(0.1)


@pytest.mark.asyncio
async def test_time_does_not_move_without_a_step(stepped_api):
    """The whole point, asserted through the HTTP layer."""
    client, motor, _clock = stepped_api
    motor.target_accel_mks = 0
    motor.current_rpm = motor.target_rpm = 60.0

    before = (await client.get("/status")).json()["motors"]["1"]["position_steps"]
    after = (await client.get("/status")).json()["motors"]["1"]["position_steps"]

    assert before == after, "the motor moved between two reads with no step"


@pytest.mark.asyncio
async def test_the_returned_state_reflects_the_step(stepped_api):
    """
    The response carries settled post-step state.

    If `/step` returned before the motors had finished the last sub-step, this
    would be the read that caught one half-updated - which is exactly the race a
    caller has today when it sleeps and then polls.
    """
    client, motor, _clock = stepped_api
    motor.target_accel_mks = 0
    motor.current_rpm = motor.target_rpm = 60.0
    start = motor.position_steps

    body = (await client.post("/step", json={"seconds": 1.0})).json()

    travelled = body["status"]["motors"]["1"]["position_steps"] - start
    assert travelled == pytest.approx(STEPS_PER_REV, rel=1e-9), (
        f"one simulated second at 60 RPM moved {travelled} steps, expected "
        f"one revolution ({STEPS_PER_REV})"
    )
    # And the same figure is what a follow-up read sees.
    assert body["status"]["motors"]["1"]["position_steps"] == pytest.approx(
        (await client.get("/status")).json()["motors"]["1"]["position_steps"]
    )


@pytest.mark.asyncio
async def test_repeating_the_same_sequence_gives_the_same_answer(stepped_api):
    """Determinism, over the wire."""
    client, motor, _clock = stepped_api
    motor.target_accel_mks = 100
    motor.target_rpm = 120.0

    trace = []
    for _ in range(10):
        body = (await client.post("/step", json={"seconds": 0.05})).json()
        trace.append(body["status"]["motors"]["1"]["position_steps"])
    assert trace == sorted(trace), "position went backwards under a forward move"
    assert len(set(trace)) > 1, "ten steps produced no movement at all"


@pytest.mark.asyncio
async def test_a_negative_step_is_rejected(stepped_api):
    """
    Time does not go backwards, and saying so beats quietly doing nothing.

    Returning 200 with `advanced_seconds: 0` would leave a caller believing it
    had stepped.
    """
    client, _motor, clock = stepped_api
    response = await client.post("/step", json={"seconds": -1.0})
    assert response.status_code == 422, response.text
    assert clock.now() == 0.0


@pytest.mark.asyncio
async def test_a_zero_step_is_allowed_and_moves_nothing(stepped_api):
    client, _motor, clock = stepped_api
    body = (await client.post("/step", json={"seconds": 0})).json()
    assert body["advanced_seconds"] == 0.0
    assert clock.now() == 0.0


@pytest.mark.asyncio
async def test_stepping_a_real_time_simulator_says_it_was_not_stepped():
    """
    On the default clock the endpoint must not claim to have stepped.

    It waits out the interval - which is the honest thing a caller can act on -
    and reports `stepped: false` so a client can tell the difference between
    "time was advanced" and "we slept". Reporting success for something that did
    not happen is this codebase's recurring defect; the endpoint does not get to
    join in.
    """
    loop = asyncio.get_event_loop()
    bus = VirtualCANBus(loop)
    assert isinstance(bus.simulation_clock, RealTimeClock)
    motor = SimulatedMotor(can_id=1, loop=loop)
    bus.add_motor(motor)
    interface = LLMDebugInterface(motors=bus.simulated_motors, can_bus=bus)
    bus.debug_interface = interface

    async with _asgi_client(DebugHTTPServer(debug_interface=interface).app) as client:
        body = (await client.post("/step", json={"seconds": 0.01})).json()

    assert body["stepped"] is False
    assert body["advanced_seconds"] == pytest.approx(0.01)


@pytest.mark.asyncio
async def test_the_endpoint_is_advertised(stepped_api):
    """A capability nobody can discover may as well not exist."""
    client, _motor, _clock = stepped_api
    assert "/step" in (await client.get("/")).json()["endpoints"]
