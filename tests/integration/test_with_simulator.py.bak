# mks_servo_can_project/tests/integration/test_with_simulator.py
# This line specifies the path to the test file within the project structure.
# It's an integration test file, meaning it tests the interaction between different parts of the system,
# specifically focusing on tests that utilize the MKS Servo Simulator.

import math

import os  # For checking if simulator executable exists
# Imports the 'os' module, which provides a way of using operating system dependent functionality.
# In this script, it's mentioned as potentially being used for checking if the simulator executable exists,
# though the actual check uses 'subprocess.check_output("command -v ...")' which is more common on Unix-like systems.

import pytest
# Imports the 'pytest' framework, which is used for writing and running tests.
# Pytest offers a powerful and flexible way to define test functions, fixtures, and run test suites.

import pytest_asyncio  # For async fixtures
# Imports 'pytest_asyncio', an extension for pytest that allows testing asynchronous code
# written with 'asyncio'. This is crucial for this project as the 'mks_servo_can' library is heavily async.

import subprocess
# Imports the 'subprocess' module, used to run and manage child processes.
# Here, it's used to start and stop the MKS Servo Simulator executable as a separate process for integration testing.

import time
# Imports the 'time' module, providing various time-related functions.
# In this script, it's used for 'time.sleep()' to pause execution, allowing the simulator process time to start up.

from mks_servo_can import Axis
# Imports the 'Axis' class from the 'mks_servo_can' library.
# The 'Axis' class is a high-level interface for controlling a single MKS servo motor.

from mks_servo_can import CANInterface
# Imports the 'CANInterface' class, responsible for managing the communication link
# (either to real hardware or the simulator).

from mks_servo_can import const
# Imports the 'const' module (likely 'constants.py') from the 'mks_servo_can' library.
# This module is expected to contain various constants used throughout the library, such as default values, command codes, etc.

from mks_servo_can import exceptions
# Imports the 'exceptions' module, which defines custom exception classes for the library.
# This allows for more specific error handling.

from mks_servo_can import LinearKinematics
# Imports the 'LinearKinematics' class, used for converting between linear physical units (e.g., mm)
# and motor steps, for axes that perform linear motion.

from mks_servo_can import MultiAxisController

SIMULATOR_HOST = "localhost"
# Defines a constant for the hostname where the simulator is expected to be running.
# "localhost" means the simulator is running on the same machine as the tests.

SIMULATOR_PORT = 6789
# Defines a constant for the TCP port number the simulator will listen on.
# The test environment will try to connect to the simulator on this port.

SIMULATOR_STARTUP_TIMEOUT = 5  # seconds to wait for simulator to start
# Defines a timeout duration in seconds. The test setup will wait for this amount of time
# for the simulator process to initialize before proceeding with tests. This prevents tests
# from failing due to the simulator not being ready.

# Attempt to find the simulator executable more robustly
# This assumes the tests are run from the project root, and the simulator
# is installed in a typical virtual environment bin path, or system path.
# Or, if the 'mks_servo_simulator' package is installed editable,
# its scripts should be on the PATH of the venv.
# This comment explains the strategy for locating the simulator executable.
# It expects 'mks-servo-simulator' to be available in the system's PATH,
# common if it's installed as a package or an editable install in a virtual environment.

SIMULATOR_CMD = "mks-servo-simulator"  # Assume it's in PATH
# Defines the command string used to launch the MKS Servo Simulator.
# The tests will execute this command to start the simulator.

@pytest.fixture(scope="module")  # Regular synchronous fixture, module-scoped
# Defines a pytest fixture named 'running_simulator'.
# 'scope="module"' means this fixture will be set up once per test module and torn down after all tests in the module have run.
# It's a synchronous fixture because it manages an external process.

def running_simulator():
    # This docstring explains the purpose of the 'running_simulator' fixture.
    # It starts the MKS Servo Simulator as a subprocess before tests in a module run
    # and ensures it's terminated afterwards.
    """
    Fixture to start the MKS Servo Simulator as a subprocess for the test module
    and ensure it's terminated afterwards. This is a synchronous fixture.
    """

    # Check if simulator command exists (optional, but good for diagnostics)
    # This section attempts to verify if the simulator command is available in the system's PATH.
    # 'command -v' is a Unix-like command to find the path of an executable.
    # If not found, it skips the integration tests in this module.
    if not subprocess.check_output(
        f"command -v {SIMULATOR_CMD}", shell=True, text=True
    ).strip():
        # If 'command -v' fails or returns an empty string, the simulator is not found.
        pytest.skip(
            f"'{SIMULATOR_CMD}' not found in PATH. Skipping integration tests."
        )
        # 'pytest.skip' causes pytest to skip all tests that depend on this fixture within the current module.
        return  # Must return for pytest.skip to work correctly in a fixture
        # Exiting the fixture function is important after calling pytest.skip.

    simulator_process = None
    # Initializes 'simulator_process' to None. This variable will hold the subprocess object for the simulator.

    cmd = [
        SIMULATOR_CMD,
        "--port",
        str(SIMULATOR_PORT),
        "--num-motors",
        "2", # Specifies that the simulator should emulate 2 motors.
        "--start-can-id",
        "1", # The CAN ID for the first motor will be 1, the second will be 2.
        "--log-level",
        "INFO", # Sets the logging level for the simulator.
    ]
    # Defines the command and its arguments to start the simulator.
    # It configures the port, number of motors, starting CAN ID, and log level for the simulator instance.

    print(f"\nAttempting to start simulator for module: {' '.join(cmd)}")
    # Prints a message indicating the command being used to start the simulator. Useful for debugging.

    try:
        # This 'try' block handles the lifecycle of the simulator process.
        # Start the simulator process
        simulator_process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        # Starts the simulator using 'subprocess.Popen'.
        # 'stdout=subprocess.PIPE' and 'stderr=subprocess.PIPE' capture the simulator's output and error streams.

        print(
            f"Simulator process starting with PID: {simulator_process.pid}. Waiting for it to initialize..."
        )
        # Prints the Process ID (PID) of the started simulator and a message about waiting for initialization.

        # Synchronous sleep to give the simulator time to start up
        time.sleep(SIMULATOR_STARTUP_TIMEOUT)
        # Pauses execution for 'SIMULATOR_STARTUP_TIMEOUT' seconds to allow the simulator to initialize fully.
        # This is a simple way to wait; more robust methods might involve checking for a specific output from the simulator.

        if simulator_process.poll() is not None:
            # Checks if the simulator process has already terminated ('poll()' returns the exit code if terminated, None otherwise).
            # This indicates a failure during startup.
            stdout, stderr = simulator_process.communicate()
            # Retrieves the standard output and standard error from the terminated process.
            print(
                "Simulator STDOUT on premature exit:\n",
                stdout.decode(errors="replace"),
            )
            # Prints the simulator's standard output.
            print(
                "Simulator STDERR on premature exit:\n",
                stderr.decode(errors="replace"),
            )
            # Prints the simulator's standard error.
            pytest.fail(
                f"Simulator failed to start or stay running. Exit code: {simulator_process.returncode}"
            )
            # Fails the test setup if the simulator didn't start correctly.

        print("Simulator assumed to be running for module.")
        # If the simulator hasn't terminated, it's assumed to be running.

        yield  # Test module runs here
        # 'yield' is the point where the fixture provides control to the tests.
        # Execution of tests that depend on this fixture happens here.
        # After all tests in the module are done, the code in the 'finally' block will execute.

    finally:
        # The 'finally' block ensures that cleanup (terminating the simulator) happens
        # regardless of whether the tests passed or failed.
        if simulator_process and simulator_process.poll() is None:
            # Checks if the simulator process exists and is still running.
            print("\nTerminating simulator process after module tests...")
            # Prints a message indicating simulator termination.
            simulator_process.terminate()
            # Sends a SIGTERM signal to the simulator process, requesting it to terminate gracefully.
            try:
                stdout, stderr = simulator_process.communicate(timeout=5)
                # Waits for the process to terminate and reads any remaining output, with a 5-second timeout.
            except subprocess.TimeoutExpired:
                # If the simulator doesn't terminate gracefully within the timeout.
                print(
                    "Simulator did not terminate gracefully after module, killing."
                )
                # Prints a message that a forceful kill is needed.
                simulator_process.kill()
                # Sends a SIGKILL signal to forcefully terminate the simulator process.
                simulator_process.communicate()  # To clear pipes
                # Clears any remaining output from the pipes after killing.
            print("Simulator process stopped after module.")
            # Confirms that the simulator process has been stopped.
        elif simulator_process and simulator_process.poll() is not None:
            # This case handles if the simulator terminated unexpectedly before the 'finally' block was reached.
            print("Simulator process already terminated before explicit stop.")
            # Logs that the simulator was already stopped.


@pytest_asyncio.fixture(scope="function")
# Defines an asynchronous pytest fixture named 'can_interface_sim'.
# 'scope="function"' means this fixture will be set up once per test function that uses it.
# It depends on 'running_simulator' to ensure the simulator is active.

async def can_interface_sim(
    running_simulator,
):  # Depends on module-scoped running_simulator
    # This docstring explains the fixture's purpose: providing a CANInterface connected to the simulator.
    """Provides a CANInterface connected to the simulator for each test function."""
    iface = CANInterface(
        use_simulator=True, # Configures the CANInterface to use the simulator.
        simulator_host=SIMULATOR_HOST, # Specifies the host of the simulator.
        simulator_port=SIMULATOR_PORT, # Specifies the port of the simulator.
    )
    # Creates an instance of 'CANInterface' configured for the simulator.

    try:
        # This 'try' block handles the connection and a_yield of the interface.
        # The test name part was for debugging, can be removed or kept
        # current_test_name = request.node.name if hasattr(request, 'node') else 'unknown_test'
        # print(f"can_interface_sim: Attempting to connect for test: {current_test_name}")
        # This commented-out section was likely for debugging, showing which test is trying to connect.

        await iface.connect()
        # Asynchronously connects the 'CANInterface' instance to the simulator.
        print("can_interface_sim: Connected to simulator.")
        # Logs successful connection.
        yield iface
        # 'yield iface' provides the connected 'CANInterface' instance to the test function.
        # After the test function completes, the code in the 'finally' block is executed.
    except exceptions.SimulatorError as e:
        # Catches 'SimulatorError' if the connection fails.
        pytest.fail(
            f"Failed to connect to simulator in can_interface_sim: {e}. Ensure simulator started by 'running_simulator' fixture is accessible."
        )
        # Fails the test if connection to the simulator cannot be established, providing an informative message.
    finally:
        # The 'finally' block ensures disconnection from the simulator after each test function.
        print("can_interface_sim: Disconnecting from simulator.")
        # Logs the disconnection attempt.
        await iface.disconnect()
        # Asynchronously disconnects the 'CANInterface'.


@pytest_asyncio.fixture(scope="function")
# Defines an asynchronous fixture 'sim_axis1' with function scope.
# It depends on 'can_interface_sim' to get a connected CAN interface.

async def sim_axis1(can_interface_sim: CANInterface):
    # This docstring explains that the fixture provides an 'Axis' instance for CAN ID 1.
    """Provides an Axis instance for CAN ID 1 for each test function."""
    axis = Axis(
        can_interface_manager=can_interface_sim, # Uses the connected CAN interface.
        motor_can_id=1, # Configures the axis for motor with CAN ID 1.
        name="SimAxis1" # Assigns a name to the axis instance.
    )
    # Creates an 'Axis' instance.
    # await axis.initialize() # Initialize in the test if needed, or if all tests need it.
    # Initialization of the axis (which involves communication) is commented out here,
    # suggesting it should be done explicitly within tests if required for a specific test case,
    # or if all tests require an initialized axis, it could be uncommented.
    print("sim_axis1: Fixture created.")
    # Logs that the fixture has been created.
    return axis
    # Returns the created 'Axis' instance.


@pytest_asyncio.fixture(scope="function")
# Defines an asynchronous fixture 'sim_axis1_linear' with function scope.
# It also depends on 'can_interface_sim'.

async def sim_axis1_linear(can_interface_sim: CANInterface):
    # This docstring indicates that the fixture provides an 'Axis' instance with 'LinearKinematics'.
    """Provides an Axis instance with LinearKinematics for CAN ID 1 for each test function."""
    kin = LinearKinematics(
        steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, # Standard encoder resolution.
        pitch=10.0, # Defines that 1 revolution of the output shaft results in 10.0 mm of linear motion.
        units="mm" # Specifies that the user units for this axis are millimeters.
    )
    # Creates a 'LinearKinematics' object.
    axis = Axis(
        can_interface_manager=can_interface_sim, # Uses the connected CAN interface.
        motor_can_id=1, # Configures for motor with CAN ID 1.
        name="SimAxis1Linear", # Assigns a name.
        kinematics=kin # Assigns the created 'LinearKinematics' object.
    )
    # Creates an 'Axis' instance with the specified linear kinematics.
    print("sim_axis1_linear: Fixture created.")
    # Logs fixture creation.
    # await axis.initialize()
    # Similar to 'sim_axis1', axis initialization is commented out.
    return axis
    # Returns the 'Axis' instance.


@pytest.mark.asyncio
# Marks the test function as an asynchronous test to be run with 'pytest-asyncio'.

async def test_sim_axis_ping(sim_axis1: Axis):
    # This test aims to verify basic communication and state changes (enable/disable)
    # with a simulated motor axis.
    # It uses the 'sim_axis1' fixture, which provides an 'Axis' object connected to the simulator.
    """Test basic communication with a simulated axis."""
    try:
        # This 'try' block encapsulates the test logic and catches potential exceptions.
        await sim_axis1.initialize()
        # Initializes the axis. This step usually involves some basic communication
        # to confirm the motor is reachable and to fetch initial status.
        initial_pos = await sim_axis1.get_current_position_steps()
        # Reads the initial position of the motor in raw encoder steps.
        assert isinstance(initial_pos, int)
        # Asserts that the returned position is an integer.

        await sim_axis1.enable_motor()
        # Sends a command to enable the motor.
        assert (
            await sim_axis1.read_en_status() is True
        ), "Motor should be enabled"
        # Reads the enable status from the motor and asserts that it is True.
        # This verifies that the enable command was processed correctly by the simulator.

        await sim_axis1.disable_motor()
        # Sends a command to disable the motor.
        assert (
            await sim_axis1.read_en_status() is False
        ), "Motor should be disabled"
        # Reads the enable status again and asserts that it is False, verifying the disable command.
        print("test_sim_axis_ping completed successfully.")
        # Logs successful completion of the test.

    except exceptions.CommunicationError as e:
        # Catches 'CommunicationError' which might occur if the simulator is not responding.
        pytest.fail(
            f"CommunicationError during test_sim_axis_ping: {e}. Is the simulator running and connectable?"
        )
        # Fails the test with a message indicating a communication problem.
    except Exception as e:
        # Catches any other unexpected exceptions.
        pytest.fail(f"Unexpected exception in test_sim_axis_ping: {e}")
        # Fails the test with the details of the unexpected exception.


@pytest.mark.asyncio
# Marks this as an asynchronous test function.

async def test_sim_axis_relative_move(sim_axis1: Axis):
    # This test verifies the functionality of relative movement commands on a simulated axis.
    # It checks if the motor moves by the specified amount from its initial position.
    """Test a relative move on a simulated axis."""
    await sim_axis1.initialize()
    # Initializes the axis.
    await sim_axis1.enable_motor()
    # Enables the motor for movement.
    await sim_axis1.set_work_mode(const.MODE_SR_VFOC)
    # Sets the work mode, often a prerequisite for certain types of control or performance characteristics.
    # SR_VFOC (Serial, Vector Field Oriented Control) is a common high-performance mode.
    initial_pos_steps = await sim_axis1.get_current_position_steps()
    # Gets the starting position in steps.

    delta_encoder_steps = const.ENCODER_PULSES_PER_REVOLUTION // 4 

    relative_move_microsteps = sim_axis1._raw_encoder_steps_to_command_microsteps(
        delta_encoder_steps
    )
    # Calculates the number of pulses for a 1/4 revolution, using a constant for encoder pulses per revolution.

    sim_axis1.default_speed_param = 1000
    # Sets the default speed parameter for the axis instance for this test. This parameter (0-3000) influences motor RPM.
    
    sim_axis1.default_accel_param = 150
    # Sets the default acceleration parameter (0-255).

    await sim_axis1.move_relative_pulses(relative_move_microsteps, wait=True)
    # Commands the motor to move by 'relative_move_pulses'.
    # 'wait=True' means the function will not return until the simulated move is complete.

    final_pos_steps = await sim_axis1.get_current_position_steps()
    # Gets the position after the move.
    print(
        f"test_sim_axis_relative_move: Initial={initial_pos_steps}, MovedBy={delta_encoder_steps}, Final={final_pos_steps}"
    )
    # Logs the initial, relative, and final positions for debugging.

    # Check if the final position reflects the relative move from the initial position
    # The simulator's internal position is float, so allow for small rounding differences.
    # This comment explains the need for a tolerance in the assertion due to potential floating-point inaccuracies in the simulation.
    expected_final_pos = initial_pos_steps + delta_encoder_steps
    # Calculates the expected final position.
    assert (
        abs(final_pos_steps - expected_final_pos) < 2.0
    )  # Allow tolerance of 1-2 steps due to simulation step rounding
    # Asserts that the actual final position is close to the expected final position, within a tolerance of 2 steps.
    # This accounts for potential rounding in the simulator's internal calculations.
    print("test_sim_axis_relative_move completed successfully.")
    # Logs successful completion.


@pytest.mark.asyncio
# Marks this as an asynchronous test function.

async def test_sim_axis_absolute_move_user(sim_axis1_linear: Axis):
    # This test verifies absolute movement commands using user-defined units (e.g., mm)
    # by utilizing an axis configured with 'LinearKinematics'.
    # It checks if the motor moves to the specified absolute target position.
    """Test absolute move with user units on a simulated axis with linear kinematics."""
    await sim_axis1_linear.initialize()
    # Initializes the axis (which has LinearKinematics).
    await sim_axis1_linear.enable_motor()
    # Enables the motor.
    await sim_axis1_linear.set_work_mode(const.MODE_SR_VFOC)
    # Sets the work mode.

    await sim_axis1_linear.set_current_position_as_zero()
    # Sets the current motor position to be the zero point in user units (e.g., 0 mm).

    initial_pos_user = await sim_axis1_linear.get_current_position_user()
    # Gets the current position in user units (should be 0.0 after setting zero).
    assert (
        abs(initial_pos_user - 0.0) < 0.001
    ), "Position should be zero after setting zero"
    # Asserts that the initial position is indeed very close to zero.

    target_pos_mm = 5.0  # Target 5mm
    # Defines the target absolute position in millimeters.
    # Kinematics: pitch=10mm/rev, steps_per_rev=16384. So 5mm = 0.5 rev = 8192 steps.
    # This comment provides a manual calculation for context, relating the user unit target to motor steps
    # based on the kinematics defined in the 'sim_axis1_linear' fixture.
    await sim_axis1_linear.move_to_position_abs_user(
        target_pos_mm, speed_user=20.0, wait=True
    )  # 20mm/s
    # Commands the motor to move to the absolute position 'target_pos_mm' (5.0 mm)
    # at a speed of 20.0 mm/s. 'wait=True' ensures the function awaits completion.

    final_pos_user = await sim_axis1_linear.get_current_position_user()
    # Gets the final position in user units after the move.
    print(
        f"test_sim_axis_absolute_move_user: TargetUser={target_pos_mm}, FinalUser={final_pos_user}"
    )
    # Logs the target and final user positions.
    assert abs(final_pos_user - target_pos_mm) < 0.1  # User unit tolerance
    # Asserts that the final position is close to the target position, within a tolerance of 0.1 user units (mm).
    # This tolerance accounts for simulation precision and kinematic conversions.
    print("test_sim_axis_absolute_move_user completed successfully.")
    # Logs successful completion.

@pytest_asyncio.fixture(scope="function")
async def sim_plotter_controller(can_interface_sim: CANInterface) -> MultiAxisController:
    """
    Provides a MultiAxisController with two linear axes (X and Y) connected
    to the simulator, mimicking a simple plotter.
    """
    # Create the controller with the shared CAN interface from the simulator
    controller = MultiAxisController(can_interface_manager=can_interface_sim)

    # Define linear kinematics for the X and Y axes (e.g., 10mm of travel per revolution)
    kin_x = LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0, units="mm")
    kin_y = LinearKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION, pitch=10.0, units="mm")

    # Create and add the Axis objects to the controller
    # Assumes the simulator is running with motors at CAN IDs 1 and 2
    axis_x = Axis(can_interface_sim, 1, "AxisX", kin_x)
    axis_y = Axis(can_interface_sim, 2, "AxisY", kin_y)

    controller.add_axis(axis_x)
    controller.add_axis(axis_y)

    print("Fixture 'sim_plotter_controller' created with X and Y linear axes.")
    return controller

@pytest.mark.asyncio
async def test_sim_linear_move_with_interpolation(
    # You would need a fixture that provides a connected MultiAxisController
    # with at least two linear axes, similar to the setup in the examples.
    sim_plotter_controller: MultiAxisController 
):
    """
    Tests the move_linearly_to method for correct end-to-end behavior
    with the simulator.
    """
    # 1. Setup: Enable axes and set to a known starting state (e.g., zero)
    await sim_plotter_controller.enable_all_axes()
    for axis in sim_plotter_controller.axes.values():
        await axis.set_current_position_as_zero()

    # 2. Define and execute the linear move
    target_pos = {"AxisX": 30.0, "AxisY": 40.0}
    tool_speed = 25.0 # mm/s
    
    await sim_plotter_controller.move_linearly_to(
        target_positions=target_pos,
        tool_speed_user=tool_speed,
        wait_for_all=True
    )

    # 3. Verify the result
    final_positions = await sim_plotter_controller.get_all_positions_user()
    
    # Check that both axes reached their target positions accurately
    assert math.isclose(final_positions.get("AxisX"), 30.0, abs_tol=0.1)
    assert math.isclose(final_positions.get("AxisY"), 40.0, abs_tol=0.1)
    