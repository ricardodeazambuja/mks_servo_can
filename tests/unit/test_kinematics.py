"""Unit tests for the kinematics classes.

These tests verify the conversion logic and initialization of various
kinematics models like LinearKinematics, RotaryKinematics, and
EccentricKinematics, ensuring they correctly translate between user units
and motor steps/pulses.
"""
# Kinematics classes are responsible for converting between user-defined physical units
# (like mm or degrees) and the motor's native units (encoder steps or pulses).
# These unit tests verify the correctness of these conversions and the initialization logic
# of the various kinematics classes.

import math
# Imports the 'math' module, which provides access to mathematical functions like 'sin', 'cos', 'radians', etc.
# This is used in kinematics calculations, especially for non-linear relationships like in EccentricKinematics.

import pytest
# Imports the 'pytest' framework, used for writing and running these tests.
# Pytest allows for structured tests, fixtures, and assertions.

from mks_servo_can.constants import \
    ENCODER_PULSES_PER_REVOLUTION
# Imports 'ENCODER_PULSES_PER_REVOLUTION' from the library's constants module.
# This constant represents the typical number of encoder pulses a MKS servo motor has per revolution (e.g., 16384).
# It's used as a default or reference value in kinematics tests.

from mks_servo_can.exceptions import KinematicsError
# Imports the 'KinematicsError' custom exception from the library.
# This exception is expected to be raised by kinematics classes when invalid parameters
# are provided during initialization or when a conversion is impossible.

from mks_servo_can.kinematics import EccentricKinematics
# Imports the 'EccentricKinematics' class. This is an example of a non-linear kinematics model,
# and these tests will verify its behavior.

from mks_servo_can.kinematics import Kinematics
# Imports the base 'Kinematics' abstract class.
# While not directly instantiated in tests for functionality (it's abstract),
# it's relevant for understanding the inheritance structure and for testing
# that concrete classes correctly implement its abstract methods.

from mks_servo_can.kinematics import LinearKinematics
# Imports the 'LinearKinematics' class, used for systems where motor rotation
# results in linear motion (e.g., a lead screw).

from mks_servo_can.kinematics import RotaryKinematics
# Imports the 'RotaryKinematics' class, used for systems where motor rotation
# results in rotary motion of an output shaft.

class TestBaseKinematics:
    # Defines a test class 'TestBaseKinematics' to group tests related to the
    # common initialization logic and abstract nature of the base 'Kinematics' class.
    # Although 'Kinematics' is abstract, its constructor logic (parameter validation)
    # is called by subclasses, so that can be indirectly tested via a concrete subclass.

    def test_base_kinematics_init_valid(self):
        # Tests the valid initialization of a concrete kinematics class (RotaryKinematics is used as an example)
        # to ensure that the base class constructor logic correctly processes valid inputs.
        kin = RotaryKinematics(
            steps_per_revolution=200, gear_ratio=2.0
        )  # Example concrete class
        # Instantiates RotaryKinematics with valid steps and gear ratio.
        assert kin.steps_per_revolution == 200
        # Verifies that 'steps_per_revolution' is stored correctly.
        assert kin.gear_ratio == 2.0
        # Verifies that 'gear_ratio' is stored correctly.
        assert kin.effective_steps_per_output_revolution == 400
        # Verifies that 'effective_steps_per_output_revolution' (which is steps_per_revolution * gear_ratio)
        # is calculated correctly by the base class logic.

    def test_base_kinematics_init_invalid_steps(self):
        # Tests that initializing a kinematics class with invalid 'steps_per_revolution'
        # (zero or negative) raises a 'KinematicsError'.
        with pytest.raises(
            KinematicsError, match="steps_per_revolution must be positive."
        ):
            # Attempts to initialize with 0 steps, expecting an error.
            RotaryKinematics(steps_per_revolution=0)
        with pytest.raises(
            KinematicsError, match="steps_per_revolution must be positive."
        ):
            # Attempts to initialize with -100 steps, expecting an error.
            RotaryKinematics(steps_per_revolution=-100)

    def test_base_kinematics_init_invalid_gear_ratio(self):
        # Tests that initializing a kinematics class with an invalid 'gear_ratio'
        # (zero or negative) raises a 'KinematicsError'.
        with pytest.raises(
            KinematicsError, match="gear_ratio must be positive."
        ):
            # Attempts to initialize with gear_ratio = 0, expecting an error.
            RotaryKinematics(steps_per_revolution=200, gear_ratio=0)
        with pytest.raises(
            KinematicsError, match="gear_ratio must be positive."
        ):
            # Attempts to initialize with gear_ratio = -1.0, expecting an error.
            RotaryKinematics(steps_per_revolution=200, gear_ratio=-1.0)

    def test_base_kinematics_abstract_methods(self):
        # Tests that the base 'Kinematics' class itself cannot be instantiated directly
        # because it's an abstract base class (ABC) with abstract methods.
        # Ensure abstract methods are defined (cannot be instantiated directly)
        with pytest.raises(TypeError):  # Cannot instantiate abstract class
            # Attempting to instantiate 'Kinematics' directly should raise a TypeError.
            Kinematics(steps_per_revolution=100)

class TestLinearKinematics:
    # Defines a test class 'TestLinearKinematics' to group tests specific to the 'LinearKinematics' class.

    def test_linear_init_valid(self):
        # Tests the valid initialization of 'LinearKinematics'.
        # It verifies that all provided parameters are stored correctly and derived values are calculated as expected.
        kin = LinearKinematics(
            steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0, units="mm"
        )
        # Instantiates 'LinearKinematics' with typical valid parameters.
        assert kin.steps_per_revolution == 1600
        # Verifies 'steps_per_revolution'.
        assert kin.pitch == 5.0
        # Verifies 'pitch' (linear distance per output revolution).
        assert kin.gear_ratio == 1.0
        # Verifies 'gear_ratio'.
        assert kin.units == "mm"
        # Verifies 'units'.
        assert kin.effective_steps_per_output_revolution == 1600
        # Verifies effective steps calculation.
        assert kin.steps_per_user_unit == 1600 / 5.0  # 320 steps/mm
        # Verifies 'steps_per_user_unit' (e.g., steps per mm) calculation.

    def test_linear_init_invalid_pitch(self):
        # Tests that 'LinearKinematics' raises a 'KinematicsError' if an invalid 'pitch'
        # (zero or negative) is provided.
        with pytest.raises(KinematicsError, match="Pitch must be positive."):
            # Attempts to initialize with pitch = 0.
            LinearKinematics(steps_per_revolution=1600, pitch=0)
        with pytest.raises(KinematicsError, match="Pitch must be positive."):
            # Attempts to initialize with pitch = -5.0.
            LinearKinematics(steps_per_revolution=1600, pitch=-5.0)

    def test_linear_user_to_steps(self):
        # Tests the 'user_to_steps' conversion for 'LinearKinematics'.
        # Given a linear distance in user units (e.g., mm), it should return the equivalent motor steps.
        # Setup: 1600 steps/revolution, 5.0 mm/revolution pitch, 1.0 gear_ratio.
        # This results in (1600 steps / 5 mm) = 320 steps/mm.
        kin = LinearKinematics(
            steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0
        )
        assert kin.user_to_steps(0.0) == 0
        # 0 mm should be 0 steps.
        assert kin.user_to_steps(1.0) == 320  # 1mm = 320 steps
        # 1 mm should be 320 steps.
        assert kin.user_to_steps(5.0) == 1600  # 5mm = 1 revolution = 1600 steps
        # 5 mm (one pitch) should be one full revolution (1600 steps).
        assert kin.user_to_steps(-1.0) == -320
        # Negative distance should result in negative steps.
        assert kin.user_to_steps(0.5) == 160
        # 0.5 mm should be 160 steps.
        assert kin.user_to_steps(0.003125) == 1  # 1/320 mm = 1 step
        # Smallest resolvable distance (1 step) should convert correctly. (0.003125 = 1/320).

    def test_linear_steps_to_user(self):
        # Tests the 'steps_to_user' conversion for 'LinearKinematics'.
        # Given a number of motor steps, it should return the equivalent linear distance in user units.
        # Same setup as above: 320 steps/mm.
        kin = LinearKinematics(
            steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0
        )
        assert kin.steps_to_user(0) == 0.0
        # 0 steps should be 0.0 mm.
        assert kin.steps_to_user(320) == 1.0
        # 320 steps should be 1.0 mm.
        assert kin.steps_to_user(1600) == 5.0
        # 1600 steps (one revolution) should be 5.0 mm (one pitch).
        assert kin.steps_to_user(-320) == -1.0
        # Negative steps should result in negative distance.
        assert kin.steps_to_user(1) == 1.0 / 320.0
        # 1 step should be 1/320th of a mm.

    def test_linear_with_gearing(self):
        # Tests 'LinearKinematics' with a non-unity gear ratio.
        # Setup: Motor 200 steps/rev. Gearbox 10:1 (motor turns 10 times for 1 output shaft rev). Pitch 5mm/output_rev.
        kin = LinearKinematics(
            steps_per_revolution=200, pitch=5.0, gear_ratio=10.0
        )
        # Effective steps for 1 output rev = motor_steps_per_rev * gear_ratio = 200 * 10 = 2000 steps.
        # Steps per mm = effective_steps_per_output_rev / pitch = 2000 steps / 5 mm = 400 steps/mm.
        assert kin.effective_steps_per_output_revolution == 2000
        # Verifies effective steps calculation.
        assert kin.steps_per_user_unit == 400
        # Verifies steps per mm.
        assert kin.user_to_steps(1.0) == 400  # 1mm
        # 1 mm should be 400 steps.
        assert kin.user_to_steps(5.0) == 2000  # 5mm (1 output rev)
        # 5 mm (one output pitch) should be 2000 effective steps.
        assert kin.steps_to_user(400) == 1.0
        # 400 steps should be 1.0 mm.
        assert kin.steps_to_user(2000) == 5.0
        # 2000 steps should be 5.0 mm.

    def test_linear_speed_conversion_simplified(self):
        # Tests the speed conversion methods ('user_speed_to_motor_speed' and 'motor_speed_to_user_speed')
        # for 'LinearKinematics'.
        # The MKS speed parameter (0-3000) in commands is assumed to map to motor RPM,
        # especially in VFOC (Vector Field Oriented Control) mode. This test uses this simplification.
        # Setup: Motor 1600 steps/rev. Pitch 5mm/rev. No gearing (gear_ratio=1.0).
        kin = LinearKinematics(
            steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0
        )
        # Conversion from user speed (mm/s) to MKS motor speed parameter:
        # Target: 1 mm/s linear speed.
        # Output revolutions/sec = (1 mm/s) / (5 mm/rev_output) = 0.2 rev_output/s.
        # Motor revolutions/sec = 0.2 rev_output/s * (1 motor_rev / 1 rev_output) (gear_ratio=1.0) = 0.2 motor_rev/s.
        # Motor RPM = 0.2 motor_rev/s * 60 s/min = 12 RPM.
        # Simplified MKS speed parameter is assumed to be equal to motor RPM, so 12.
        assert (
            kin.user_speed_to_motor_speed(1.0) == 12
        )  # 1 mm/s -> 12 RPM -> param 12
        # Target: 5 mm/s linear speed (which is 1 output rev/s).
        # Motor RPM = (5 mm/s / 5 mm/rev) * 60 = 60 RPM.
        assert (
            kin.user_speed_to_motor_speed(5.0) == 60
        )  # 5 mm/s -> 60 RPM -> param 60 (1 rev/sec)

        # Conversion from MKS motor speed parameter to user speed (mm/s):
        # Input: MKS parameter 60 (assumed 60 motor RPM).
        # Motor revs/sec = 60 RPM / 60 = 1 motor_rev/s.
        # Output revs/sec = 1 motor_rev/s / 1.0 (gear_ratio) = 1 rev_output/s.
        # User speed (mm/s) = 1 rev_output/s * 5 mm/rev_output = 5 mm/s.
        assert kin.motor_speed_to_user_speed(60) == pytest.approx(5.0)
        # Input: MKS parameter 3000 (assumed 3000 motor RPM).
        # Motor revs/sec = 3000 / 60 = 50 motor_rev/s.
        # User speed = (50 / 1.0) * 5 = 250 mm/s.
        assert kin.motor_speed_to_user_speed(3000) == pytest.approx(
            250.0
        )  # 3000 RPM -> 50 rps -> 250 mm/s

class TestRotaryKinematics:
    # Defines a test class 'TestRotaryKinematics' for the 'RotaryKinematics' class.

    def test_rotary_init_valid(self):
        # Tests valid initialization of 'RotaryKinematics'.
        # Uses 'ENCODER_PULSES_PER_REVOLUTION' as a typical value for motor steps.
        kin = RotaryKinematics(
            steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0
        )
        assert kin.steps_per_revolution == ENCODER_PULSES_PER_REVOLUTION
        # Verifies 'steps_per_revolution'.
        assert kin.degrees_per_output_revolution == 360.0
        # Verifies that 'degrees_per_output_revolution' defaults to 360.0.
        assert kin.steps_per_degree == ENCODER_PULSES_PER_REVOLUTION / 360.0
        # Verifies 'steps_per_degree' calculation.

    def test_rotary_user_to_steps(self):
        # Tests 'user_to_steps' for 'RotaryKinematics'.
        # User units are degrees by default.
        # Setup: ENCODER_PULSES_PER_REVOLUTION (16384) steps / 360 degrees.
        # So, steps_per_degree = 16384 / 360 = 45.5111...
        kin = RotaryKinematics(
            steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0
        )
        assert kin.user_to_steps(0.0) == 0
        # 0 degrees should be 0 steps.
        assert kin.user_to_steps(360.0) == ENCODER_PULSES_PER_REVOLUTION
        # 360 degrees (one full revolution) should be ENCODER_PULSES_PER_REVOLUTION steps.
        assert kin.user_to_steps(180.0) == round(
            ENCODER_PULSES_PER_REVOLUTION / 2.0
        )
        # 180 degrees should be half the steps, rounded.
        assert kin.user_to_steps(1.0) == round(
            ENCODER_PULSES_PER_REVOLUTION / 360.0
        )
        # 1 degree should be 'steps_per_degree', rounded.
        assert kin.user_to_steps(-90.0) == round(
            -ENCODER_PULSES_PER_REVOLUTION / 4.0
        )
        # -90 degrees should be -1/4 of total steps, rounded.

    def test_rotary_steps_to_user(self):
        # Tests 'steps_to_user' for 'RotaryKinematics'.
        # Converts motor steps to degrees.
        kin = RotaryKinematics(
            steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0
        )
        assert kin.steps_to_user(0) == 0.0
        # 0 steps is 0.0 degrees.
        assert kin.steps_to_user(ENCODER_PULSES_PER_REVOLUTION) == 360.0
        # Full revolution in steps is 360.0 degrees.
        assert kin.steps_to_user(
            ENCODER_PULSES_PER_REVOLUTION // 2 # Integer division for steps.
        ) == pytest.approx(180.0)
        # Half steps is approx 180.0 degrees (pytest.approx for float comparison).
        assert kin.steps_to_user(1) == pytest.approx(
            360.0 / ENCODER_PULSES_PER_REVOLUTION
        )
        # 1 step is approx (360.0 / total_steps) degrees.

    def test_rotary_with_gearing(self):
        # Tests 'RotaryKinematics' with a non-unity gear ratio.
        # Setup: Motor 200 steps/rev. Gearbox 2:1 (motor turns 2x for 1 output rev).
        kin = RotaryKinematics(steps_per_revolution=200, gear_ratio=2.0)
        # Effective steps for 1 output rev = 200 steps/motor_rev * 2 motor_rev/output_rev = 400 steps/output_rev.
        # Steps per degree of output = 400 steps / 360 degrees.
        assert kin.effective_steps_per_output_revolution == 400
        # Verifies effective steps.
        assert kin.steps_per_degree == 400.0 / 360.0
        # Verifies steps_per_degree calculation.

        assert kin.user_to_steps(360.0) == 400
        # 360 degrees output should be 400 effective steps.
        assert kin.user_to_steps(1.0) == round(400.0 / 360.0)
        # 1 degree output conversion.
        assert kin.steps_to_user(400) == 360.0
        # 400 effective steps should be 360 degrees output.

    def test_rotary_speed_conversion_simplified(self):
        # Tests speed conversions for 'RotaryKinematics', assuming MKS speed parameter (0-3000)
        # maps to motor RPM (VFOC mode).
        # Setup: Motor 16384 steps/rev (encoder pulses). No gearing.
        kin = RotaryKinematics(
            steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0
        )
        # User speed to MKS parameter:
        # Target: 360 deg/s angular speed (which is 1 output rev/s).
        # Output revs/sec = (360 deg/s) / (360 deg/rev_output) = 1 rev_output/s.
        # Motor revs/sec = 1 rev_output/s * 1.0 (gear_ratio) = 1 motor_rev/s.
        # Motor RPM = 1 motor_rev/s * 60 s/min = 60 RPM.
        # Simplified MKS speed parameter assumed equal to motor RPM, so 60.
        assert (
            kin.user_speed_to_motor_speed(360.0) == 60
        )  # 360 deg/s -> 60 RPM -> param 60

        # MKS parameter to user speed:
        # Input: MKS parameter 60 (assumed 60 motor RPM).
        # Motor RPM = 60.
        # Motor revs/sec = 1 motor_rev/s.
        # Output revs/sec = 1 motor_rev/s / 1.0 (gear_ratio) = 1 rev_output/s.
        # User speed (deg/s) = 1 rev_output/s * 360 deg/rev_output = 360 deg/s.
        assert kin.motor_speed_to_user_speed(60) == pytest.approx(360.0)
        # Input: Max MKS parameter 3000 (assumed 3000 motor RPM = 50 RPS).
        # User speed = (50 RPS_motor / 1.0 gear_ratio) * 360 deg/rev_output = 18000 deg/s.
        assert kin.motor_speed_to_user_speed(3000) == pytest.approx(18000.0)

class TestEccentricKinematics:  # Basic tests for the example implementation
    # Defines a test class 'TestEccentricKinematics' for the 'EccentricKinematics' class.
    # This class models a non-linear relationship, so tests will check specific points in its behavior.

    def test_eccentric_init(self):
        # Tests the initialization of 'EccentricKinematics'.
        kin = EccentricKinematics(
            steps_per_revolution=1600, arm_length=10.0, gear_ratio=1.0
        )
        # Instantiates with valid parameters.
        assert kin.arm_length == 10.0
        # Verifies 'arm_length' is stored.
        # Other base parameters (steps_per_revolution, gear_ratio) are tested in TestBaseKinematics.

    def test_eccentric_user_to_steps_center(self):
        # Tests 'user_to_steps' at the center position (zero displacement).
        # For the model x = L * sin(motor_output_angle), x=0 implies motor_output_angle=0.
        kin = EccentricKinematics(
            steps_per_revolution=1600, arm_length=10.0
        )  # 1600 steps / 360 deg output shaft rotation.
        # x = 0 (center) -> motor_output_angle = 0 deg -> 0 steps.
        assert kin.user_to_steps(0.0) == 0
        # 0.0 user displacement should correspond to 0 steps.

    def test_eccentric_user_to_steps_quarter_turn(self):
        # Tests 'user_to_steps' at maximum displacement, which corresponds to a 90-degree
        # rotation of the output shaft in the simplified model x = L * sin(angle).
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        # x = 10 (max displacement, equal to arm_length) -> motor_output_angle = 90 deg.
        # Steps = 90 deg_output * (1600 steps / 360 deg_output) = 90 * (40/9) = 400 steps.
        assert kin.user_to_steps(10.0) == 400
        # x = -10 (max displacement in other direction) -> motor_output_angle = -90 deg.
        # Steps = -90 deg_output * (1600 / 360) = -400 steps.
        assert kin.user_to_steps(-10.0) == -400

    def test_eccentric_steps_to_user(self):
        # Tests 'steps_to_user' for 'EccentricKinematics'.
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        assert kin.steps_to_user(0) == 0.0
        # 0 steps should be 0.0 user displacement.
        # 400 steps corresponds to 90 degrees output shaft rotation.
        # x = 10.0 * sin(90 deg) = 10.0.
        assert kin.steps_to_user(400) == pytest.approx(10.0)  # 90 deg output
        # -400 steps corresponds to -90 degrees output.
        # x = 10.0 * sin(-90 deg) = -10.0.
        assert kin.steps_to_user(-400) == pytest.approx(-10.0)  # -90 deg output
        # 200 steps corresponds to 45 degrees output shaft rotation (200 / (1600/360) = 45).
        # x = 10.0 * sin(45 deg) = 10.0 * (sqrt(2)/2) approx 7.071.
        assert kin.steps_to_user(200) == pytest.approx(
            10.0 * math.sin(math.radians(45)) # Compares with direct calculation.
        )

    def test_eccentric_clamping(self):
        # Tests that user input values exceeding the theoretical maximum displacement (arm_length)
        # are clamped during the conversion to motor angle.
        # The EccentricKinematics model's _user_to_motor_angle_deg method implements this clamping.
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        # User value > arm_length should be clamped for angle calculation.
        # motor_output_angle for 12.0 should be the same as for 10.0 (which is 90 deg, resulting in 400 steps).
        assert kin.user_to_steps(12.0) == kin.user_to_steps(10.0)  # Should both result in 400 steps.
        assert kin.user_to_steps(-12.0) == kin.user_to_steps(
            -10.0
        )  # Should both result in -400 steps.

    def test_eccentric_speed_conversion_simplified_at_center(self):
        # Tests speed conversion for 'EccentricKinematics'.
        # The conversion is simplified in the implementation, assuming motion near the center (theta=0),
        # where the relationship dx/dt approx L * d_theta/dt holds.
        kin = EccentricKinematics(
            steps_per_revolution=1600, arm_length=10.0, gear_ratio=1.0
        )
        # User speed (dx/dt) = 10 mm/s. At center (theta approx 0), arm_length L = 10 mm.
        # Angular speed of output shaft (d_theta/dt) = (dx/dt) / L = 10 mm/s / 10 mm = 1 rad/s.
        # 1 rad/s is approx 57.3 deg/s.
        # Output revs/sec = (57.3 deg/s) / (360 deg/rev) approx 0.159 rev_output/s.
        # Motor revs/sec = 0.159 rev_output/s * 1.0 (gear_ratio) = 0.159 motor_rev/s.
        # Motor RPM = 0.159 * 60 approx 9.55 RPM.
        # Simplified MKS speed parameter assumed to be motor RPM, rounded = 10.
        assert kin.user_speed_to_motor_speed(10.0) == 10  # 10 mm/s at center -> param 10

        # Reverse conversion: MKS parameter 10 (assumed 10 motor RPM).
        # Motor RPM = 10.
        # Motor revs/sec = 10/60 = 1/6 motor_rev/s.
        # Output revs/sec = (1/6) / 1.0 = 1/6 rev_output/s.
        # Output angular speed (rad/s) = (1/6 rev/s) * 2*pi rad/rev = pi/3 rad/s.
        # User speed (dx/dt at center) = L * (d_theta/dt) = 10 mm * (pi/3 rad/s) approx 10.47 mm/s.
        assert kin.motor_speed_to_user_speed(10) == pytest.approx(
            10.0 * (math.pi / 3.0)
        )
        