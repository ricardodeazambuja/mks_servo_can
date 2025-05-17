# mks_servo_can_project/tests/integration/test_with_simulator.py
import pytest
import pytest_asyncio # For async fixtures
import asyncio
import subprocess
import time
import os # For checking if simulator executable exists

from mks_servo_can_library.mks_servo_can import (
    CANInterface, Axis, MultiAxisController, Kinematics, LinearKinematics, RotaryKinematics,
    const,
    exceptions
)

SIMULATOR_HOST = 'localhost'
SIMULATOR_PORT = 6789
SIMULATOR_STARTUP_TIMEOUT = 5 # seconds to wait for simulator to start

# Attempt to find the simulator executable more robustly
# This assumes the tests are run from the project root, and the simulator
# is installed in a typical virtual environment bin path, or system path.
# Or, if the 'mks_servo_simulator' package is installed editable,
# its scripts should be on the PATH of the venv.
SIMULATOR_CMD = "mks-servo-simulator" # Assume it's in PATH

@pytest.fixture(scope="module") # Regular synchronous fixture, module-scoped
def running_simulator():
    """
    Fixture to start the MKS Servo Simulator as a subprocess for the test module
    and ensure it's terminated afterwards. This is a synchronous fixture.
    """
    # Check if simulator command exists (optional, but good for diagnostics)
    if not subprocess.check_output(f"command -v {SIMULATOR_CMD}", shell=True, text=True).strip():
        pytest.skip(f"'{SIMULATOR_CMD}' not found in PATH. Skipping integration tests.")
        return # Must return for pytest.skip to work correctly in a fixture

    simulator_process = None
    cmd = [
        SIMULATOR_CMD,
        "--port", str(SIMULATOR_PORT),
        "--num-motors", "2",
        "--start-can-id", "1",
        "--log-level", "INFO"
    ]
    print(f"\nAttempting to start simulator for module: {' '.join(cmd)}")
    try:
        # Start the simulator process
        simulator_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Simulator process starting with PID: {simulator_process.pid}. Waiting for it to initialize...")
        
        # Synchronous sleep to give the simulator time to start up
        time.sleep(SIMULATOR_STARTUP_TIMEOUT)

        if simulator_process.poll() is not None:
            stdout, stderr = simulator_process.communicate()
            print("Simulator STDOUT on premature exit:\n", stdout.decode(errors='replace'))
            print("Simulator STDERR on premature exit:\n", stderr.decode(errors='replace'))
            pytest.fail(f"Simulator failed to start or stay running. Exit code: {simulator_process.returncode}")

        print("Simulator assumed to be running for module.")
        yield # Test module runs here
    
    finally:
        if simulator_process and simulator_process.poll() is None:
            print("\nTerminating simulator process after module tests...")
            simulator_process.terminate()
            try:
                stdout, stderr = simulator_process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                print("Simulator did not terminate gracefully after module, killing.")
                simulator_process.kill()
                simulator_process.communicate() # To clear pipes
            print("Simulator process stopped after module.")
        elif simulator_process and simulator_process.poll() is not None:
            print("Simulator process already terminated before explicit stop.")


@pytest_asyncio.fixture(scope="function")
async def can_interface_sim(running_simulator): # Depends on module-scoped running_simulator
    """Provides a CANInterface connected to the simulator for each test function."""
    iface = CANInterface(use_simulator=True, simulator_host=SIMULATOR_HOST, simulator_port=SIMULATOR_PORT)
    try:
        # The test name part was for debugging, can be removed or kept
        # current_test_name = request.node.name if hasattr(request, 'node') else 'unknown_test'
        # print(f"can_interface_sim: Attempting to connect for test: {current_test_name}")
        await iface.connect()
        print("can_interface_sim: Connected to simulator.")
        yield iface
    except exceptions.SimulatorError as e:
        pytest.fail(f"Failed to connect to simulator in can_interface_sim: {e}. Ensure simulator started by 'running_simulator' fixture is accessible.")
    finally:
        print("can_interface_sim: Disconnecting from simulator.")
        await iface.disconnect()

@pytest_asyncio.fixture(scope="function")
async def sim_axis1(can_interface_sim: CANInterface):
    """Provides an Axis instance for CAN ID 1 for each test function."""
    axis = Axis(can_interface_manager=can_interface_sim, motor_can_id=1, name="SimAxis1")
    # await axis.initialize() # Initialize in the test if needed, or if all tests need it.
    print("sim_axis1: Fixture created.")
    return axis

@pytest_asyncio.fixture(scope="function")
async def sim_axis1_linear(can_interface_sim: CANInterface):
    """Provides an Axis instance with LinearKinematics for CAN ID 1 for each test function."""
    kin = LinearKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION,
        pitch=10.0, units="mm"
    )
    axis = Axis(
        can_interface_manager=can_interface_sim, motor_can_id=1, name="SimAxis1Linear", kinematics=kin
    )
    print("sim_axis1_linear: Fixture created.")
    # await axis.initialize()
    return axis

@pytest.mark.asyncio
async def test_sim_axis_ping(sim_axis1: Axis):
    """Test basic communication with a simulated axis."""
    try:
        await sim_axis1.initialize()
        initial_pos = await sim_axis1.get_current_position_steps()
        assert isinstance(initial_pos, int)
        
        await sim_axis1.enable_motor()
        assert await sim_axis1.read_en_status() is True, "Motor should be enabled"
        
        await sim_axis1.disable_motor()
        assert await sim_axis1.read_en_status() is False, "Motor should be disabled"
        print("test_sim_axis_ping completed successfully.")

    except exceptions.CommunicationError as e:
        pytest.fail(f"CommunicationError during test_sim_axis_ping: {e}. Is the simulator running and connectable?")
    except Exception as e:
        pytest.fail(f"Unexpected exception in test_sim_axis_ping: {e}")


@pytest.mark.asyncio
async def test_sim_axis_relative_move(sim_axis1: Axis):
    """Test a relative move on a simulated axis."""
    await sim_axis1.initialize() 
    await sim_axis1.enable_motor()
    await sim_axis1.set_work_mode(const.MODE_SR_VFOC) 
    initial_pos_steps = await sim_axis1.get_current_position_steps()
    
    relative_move_pulses = const.ENCODER_PULSES_PER_REVOLUTION // 4 # Move 1/4 revolution
    sim_axis1.default_speed_param = 1000 
    sim_axis1.default_accel_param = 150

    await sim_axis1.move_relative_pulses(relative_move_pulses, wait=True)
    
    final_pos_steps = await sim_axis1.get_current_position_steps()
    print(f"test_sim_axis_relative_move: Initial={initial_pos_steps}, MovedBy={relative_move_pulses}, Final={final_pos_steps}")
    
    # Check if the final position reflects the relative move from the initial position
    # The simulator's internal position is float, so allow for small rounding differences.
    expected_final_pos = initial_pos_steps + relative_move_pulses
    assert abs(final_pos_steps - expected_final_pos) < 2.0 # Allow tolerance of 1-2 steps due to simulation step rounding
    print("test_sim_axis_relative_move completed successfully.")


@pytest.mark.asyncio
async def test_sim_axis_absolute_move_user(sim_axis1_linear: Axis):
    """ Test absolute move with user units on a simulated axis with linear kinematics. """
    await sim_axis1_linear.initialize()
    await sim_axis1_linear.enable_motor()
    await sim_axis1_linear.set_work_mode(const.MODE_SR_VFOC)
    
    await sim_axis1_linear.set_current_position_as_zero() 
    
    initial_pos_user = await sim_axis1_linear.get_current_position_user()
    assert abs(initial_pos_user - 0.0) < 0.001, "Position should be zero after setting zero"
    
    target_pos_mm = 5.0 # Target 5mm
    # Kinematics: pitch=10mm/rev, steps_per_rev=16384. So 5mm = 0.5 rev = 8192 steps.
    await sim_axis1_linear.move_to_position_abs_user(target_pos_mm, speed_user=20.0, wait=True) # 20mm/s
   
    final_pos_user = await sim_axis1_linear.get_current_position_user()
    print(f"test_sim_axis_absolute_move_user: TargetUser={target_pos_mm}, FinalUser={final_pos_user}")
    assert abs(final_pos_user - target_pos_mm) < 0.1 # User unit tolerance
    print("test_sim_axis_absolute_move_user completed successfully.")