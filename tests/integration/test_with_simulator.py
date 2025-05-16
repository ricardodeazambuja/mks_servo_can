# mks_servo_can_project/tests/integration/test_with_simulator.py
"""
Integration tests for the mks-servo-can library using the MKS Servo Simulator.
These tests will require the simulator to be running.
"""
import pytest
import asyncio
import subprocess # To start and stop simulator if needed, or assume it's running
import time

from mks_servo_can_library.mks_servo_can import (
    CANInterface, Axis, MultiAxisController, Kinematics, LinearKinematics,
    const, exceptions
)

SIMULATOR_HOST = 'localhost'
SIMULATOR_PORT = 6789 # Should match simulator default or be configurable

@pytest.fixture(scope="module")
async def running_simulator():
    """Fixture to ensure the simulator is running. Can start/stop it."""
    # For simplicity, this example assumes the simulator is started manually.
    # A more robust fixture would manage the simulator process.
    # Example:
    # proc = subprocess.Popen(["mks-servo-simulator", "--port", str(SIMULATOR_PORT), "--num-motors", "2"])
    # await asyncio.sleep(2) # Give simulator time to start
    # yield
    # proc.terminate()
    # proc.wait()
    print("Assuming MKS Servo Simulator is running externally.")
    await asyncio.sleep(0.1) # Minimal yield for event loop
    yield
    print("Finished simulator-based tests.")


@pytest.fixture(scope="module")
async def can_interface_sim():
    """Provides a CANInterface connected to the simulator."""
    iface = CANInterface(use_simulator=True, simulator_host=SIMULATOR_HOST, simulator_port=SIMULATOR_PORT)
    await iface.connect()
    yield iface
    await iface.disconnect()

@pytest.fixture
async def sim_axis1(can_interface_sim):
    """Provides an Axis instance connected via the simulator interface."""
    # Assuming simulator has a motor with CAN ID 1
    axis = Axis(can_interface_manager=can_interface_sim, motor_can_id=1, name="SimAxis1")
    await axis.initialize() # Basic initialization
    return axis

@pytest.mark.asyncio
async def test_sim_axis_ping(running_simulator, sim_axis1):
    """Test basic communication with a simulated axis."""
    try:
        initial_pos = await sim_axis1.get_current_position_steps()
        assert isinstance(initial_pos, int)
        print(f"SimAxis1 initial position (steps): {initial_pos}")

        # Test enabling/disabling
        await sim_axis1.enable_motor()
        assert await sim_axis1.read_en_status() is True
        await sim_axis1.disable_motor()
        assert await sim_axis1.read_en_status() is False

    except exceptions.CommunicationError as e:
        pytest.fail(f"CommunicationError during basic test: {e}. Is the simulator running at {SIMULATOR_HOST}:{SIMULATOR_PORT}?")


@pytest.mark.asyncio
async def test_sim_axis_relative_move(running_simulator, sim_axis1):
    """Test a relative move on a simulated axis."""
    await sim_axis1.enable_motor()
    initial_pos_steps = await sim_axis1.get_current_position_steps()
    print(f"SimAxis1 before move: {initial_pos_steps} steps")

    relative_move_pulses = 1000
    # Using default speed/accel params of the Axis class, or set them
    sim_axis1.default_speed_param = 500
    sim_axis1.default_accel_param = 100

    await sim_axis1.move_relative_pulses(relative_move_pulses, wait=True)
    
    final_pos_steps = await sim_axis1.get_current_position_steps()
    print(f"SimAxis1 after move: {final_pos_steps} steps")

    # Simulator might have slight float inaccuracies, allow small tolerance
    assert abs((final_pos_steps - initial_pos_steps) - relative_move_pulses) < 2

@pytest.mark.asyncio
async def test_sim_axis_absolute_move_user(running_simulator, sim_axis1_linear):
   """ Test absolute move with user units on a simulated axis with linear kinematics. """
   # Setup axis with linear kinematics
   sim_axis1_linear.kinematics = LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0) # 10mm per rev
   await sim_axis1_linear.enable_motor()
   await sim_axis1_linear.set_current_position_as_zero() # Start from known 0

   target_pos_mm = 5.0
   await sim_axis1_linear.move_to_position_abs_user(target_pos_mm, speed_user=2.0, wait=True)
   
   final_pos_user = await sim_axis1_linear.get_current_position_user()
   assert abs(final_pos_user - target_pos_mm) < 0.01 # Tolerance for user units


# Add more tests for:
# - Different commands (homing, calibration - if sim supports)
# - Error conditions (e.g., try to move beyond simulated limits)
# - MultiAxisController operations with multiple simulated axes
# - Reading various status parameters
# - Different kinematic setups

# Note: The actual test implementations will depend heavily on the final features
# and responses implemented in the SimulatedMotor and VirtualCANBus.
# These are conceptual examples.