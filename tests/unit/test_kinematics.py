# mks_servo_can/tests/unit/test_kinematics.py
import pytest
import math
from mks_servo_can_library.mks_servo_can.kinematics import (
    Kinematics, LinearKinematics, RotaryKinematics, EccentricKinematics
)
from mks_servo_can_library.mks_servo_can.exceptions import KinematicsError
from mks_servo_can_library.mks_servo_can.constants import ENCODER_PULSES_PER_REVOLUTION

class TestBaseKinematics:
    def test_base_kinematics_init_valid(self):
        kin = RotaryKinematics(steps_per_revolution=200, gear_ratio=2.0) # Example concrete class
        assert kin.steps_per_revolution == 200
        assert kin.gear_ratio == 2.0
        assert kin.effective_steps_per_output_revolution == 400

    def test_base_kinematics_init_invalid_steps(self):
        with pytest.raises(KinematicsError, match="steps_per_revolution must be positive."):
            RotaryKinematics(steps_per_revolution=0)
        with pytest.raises(KinematicsError, match="steps_per_revolution must be positive."):
            RotaryKinematics(steps_per_revolution=-100)

    def test_base_kinematics_init_invalid_gear_ratio(self):
        with pytest.raises(KinematicsError, match="gear_ratio must be positive."):
            RotaryKinematics(steps_per_revolution=200, gear_ratio=0)
        with pytest.raises(KinematicsError, match="gear_ratio must be positive."):
            RotaryKinematics(steps_per_revolution=200, gear_ratio=-1.0)

    def test_base_kinematics_abstract_methods(self):
        # Ensure abstract methods are defined (cannot be instantiated directly)
        with pytest.raises(TypeError): # Cannot instantiate abstract class
            Kinematics(steps_per_revolution=100)


class TestLinearKinematics:
    def test_linear_init_valid(self):
        kin = LinearKinematics(steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0, units="mm")
        assert kin.steps_per_revolution == 1600
        assert kin.pitch == 5.0
        assert kin.gear_ratio == 1.0
        assert kin.units == "mm"
        assert kin.effective_steps_per_output_revolution == 1600
        assert kin.steps_per_user_unit == 1600 / 5.0  # 320 steps/mm

    def test_linear_init_invalid_pitch(self):
        with pytest.raises(KinematicsError, match="Pitch must be positive."):
            LinearKinematics(steps_per_revolution=1600, pitch=0)
        with pytest.raises(KinematicsError, match="Pitch must be positive."):
            LinearKinematics(steps_per_revolution=1600, pitch=-5.0)

    def test_linear_user_to_steps(self):
        # 320 steps/mm
        kin = LinearKinematics(steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0)
        assert kin.user_to_steps(0.0) == 0
        assert kin.user_to_steps(1.0) == 320   # 1mm = 320 steps
        assert kin.user_to_steps(5.0) == 1600  # 5mm = 1 revolution = 1600 steps
        assert kin.user_to_steps(-1.0) == -320
        assert kin.user_to_steps(0.5) == 160
        assert kin.user_to_steps(0.003125) == 1 # 1/320 mm = 1 step

    def test_linear_steps_to_user(self):
        # 320 steps/mm
        kin = LinearKinematics(steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0)
        assert kin.steps_to_user(0) == 0.0
        assert kin.steps_to_user(320) == 1.0
        assert kin.steps_to_user(1600) == 5.0
        assert kin.steps_to_user(-320) == -1.0
        assert kin.steps_to_user(1) == 1.0 / 320.0

    def test_linear_with_gearing(self):
        # Motor: 200 steps/rev. Gearbox: 10:1 (motor turns 10 times for 1 output rev). Pitch: 5mm/output_rev
        kin = LinearKinematics(steps_per_revolution=200, pitch=5.0, gear_ratio=10.0)
        # Effective steps for 1 output rev = 200 * 10 = 2000 steps
        # Steps per mm = 2000 steps / 5 mm = 400 steps/mm
        assert kin.effective_steps_per_output_revolution == 2000
        assert kin.steps_per_user_unit == 400
        assert kin.user_to_steps(1.0) == 400   # 1mm
        assert kin.user_to_steps(5.0) == 2000  # 5mm (1 output rev)
        assert kin.steps_to_user(400) == 1.0
        assert kin.steps_to_user(2000) == 5.0

    def test_linear_speed_conversion_simplified(self):
        # Assuming MKS speed param 0-3000 maps to 0-3000 RPM (VFOC mode)
        # Motor: 1600 steps/rev. Pitch: 5mm/rev. No gearing.
        kin = LinearKinematics(steps_per_revolution=1600, pitch=5.0, gear_ratio=1.0)
        # 1 mm/s linear speed:
        # Output revs/sec = (1 mm/s) / (5 mm/rev) = 0.2 revs/sec
        # Motor revs/sec = 0.2 revs/sec * 1.0 (gear_ratio) = 0.2 revs/sec
        # Motor RPM = 0.2 revs/sec * 60 = 12 RPM
        # MKS speed param (simplified) = 12
        assert kin.user_speed_to_motor_speed(1.0) == 12 # 1 mm/s -> 12 RPM -> param 12
        assert kin.user_speed_to_motor_speed(5.0) == 60 # 5 mm/s -> 60 RPM -> param 60 (1 rev/sec)

        # Reverse: MKS param 60 (assumed 60 RPM)
        # Motor RPM = 60
        # Motor revs/sec = 1
        # Output revs/sec = 1 / 1.0 = 1
        # User speed = 1 revs/sec * 5 mm/rev = 5 mm/s
        assert kin.motor_speed_to_user_speed(60) == pytest.approx(5.0)
        assert kin.motor_speed_to_user_speed(3000) == pytest.approx(250.0) # 3000 RPM -> 50 rps -> 250 mm/s


class TestRotaryKinematics:
    def test_rotary_init_valid(self):
        kin = RotaryKinematics(steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0)
        assert kin.steps_per_revolution == ENCODER_PULSES_PER_REVOLUTION
        assert kin.degrees_per_output_revolution == 360.0
        assert kin.steps_per_degree == ENCODER_PULSES_PER_REVOLUTION / 360.0

    def test_rotary_user_to_steps(self):
        # steps_per_degree = 16384 / 360 = 45.5111...
        kin = RotaryKinematics(steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0)
        assert kin.user_to_steps(0.0) == 0
        assert kin.user_to_steps(360.0) == ENCODER_PULSES_PER_REVOLUTION
        assert kin.user_to_steps(180.0) == round(ENCODER_PULSES_PER_REVOLUTION / 2.0)
        assert kin.user_to_steps(1.0) == round(ENCODER_PULSES_PER_REVOLUTION / 360.0)
        assert kin.user_to_steps(-90.0) == round(-ENCODER_PULSES_PER_REVOLUTION / 4.0)

    def test_rotary_steps_to_user(self):
        kin = RotaryKinematics(steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0)
        assert kin.steps_to_user(0) == 0.0
        assert kin.steps_to_user(ENCODER_PULSES_PER_REVOLUTION) == 360.0
        assert kin.steps_to_user(ENCODER_PULSES_PER_REVOLUTION // 2) == pytest.approx(180.0)
        assert kin.steps_to_user(1) == pytest.approx(360.0 / ENCODER_PULSES_PER_REVOLUTION)

    def test_rotary_with_gearing(self):
        # Motor: 200 steps/rev. Gearbox 2:1 (motor turns 2x for 1 output rev)
        kin = RotaryKinematics(steps_per_revolution=200, gear_ratio=2.0)
        # Effective steps for 1 output rev = 200 * 2 = 400 steps
        # Steps per degree = 400 / 360
        assert kin.effective_steps_per_output_revolution == 400
        assert kin.steps_per_degree == 400.0 / 360.0

        assert kin.user_to_steps(360.0) == 400
        assert kin.user_to_steps(1.0) == round(400.0 / 360.0)
        assert kin.steps_to_user(400) == 360.0

    def test_rotary_speed_conversion_simplified(self):
        # Assuming MKS speed param 0-3000 maps to 0-3000 RPM (VFOC mode)
        # Motor: 16384 steps/rev (encoder pulses). No gearing.
        kin = RotaryKinematics(steps_per_revolution=ENCODER_PULSES_PER_REVOLUTION, gear_ratio=1.0)
        # 360 deg/s angular speed (1 rev/s):
        # Output revs/sec = (360 deg/s) / (360 deg/rev) = 1 rev/s
        # Motor revs/sec = 1 revs/sec * 1.0 (gear_ratio) = 1 revs/sec
        # Motor RPM = 1 revs/sec * 60 = 60 RPM
        # MKS speed param (simplified) = 60
        assert kin.user_speed_to_motor_speed(360.0) == 60 # 360 deg/s -> 60 RPM -> param 60

        # Reverse: MKS param 60 (assumed 60 RPM)
        # Motor RPM = 60
        # Motor revs/sec = 1
        # Output revs/sec = 1 / 1.0 = 1
        # User speed = 1 revs/sec * 360 deg/rev = 360 deg/s
        assert kin.motor_speed_to_user_speed(60) == pytest.approx(360.0)
        # Max MKS speed param 3000 (assumed 3000 RPM = 50 RPS)
        # User speed = 50 RPS * 360 deg/rev = 18000 deg/s
        assert kin.motor_speed_to_user_speed(3000) == pytest.approx(18000.0)


class TestEccentricKinematics: # Basic tests for the example implementation
    def test_eccentric_init(self):
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0, gear_ratio=1.0)
        assert kin.arm_length == 10.0

    def test_eccentric_user_to_steps_center(self):
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0) # 1600 steps / 360 deg output
        # x = 0 (center) -> motor_output_angle = 0 deg -> 0 steps
        assert kin.user_to_steps(0.0) == 0

    def test_eccentric_user_to_steps_quarter_turn(self):
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        # x = 10 (max displacement) -> motor_output_angle = 90 deg
        # steps = 90 deg * (1600 steps / 360 deg) = 90 * (40/9) = 400 steps
        assert kin.user_to_steps(10.0) == 400
        # x = -10 -> motor_output_angle = -90 deg -> -400 steps
        assert kin.user_to_steps(-10.0) == -400

    def test_eccentric_steps_to_user(self):
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        assert kin.steps_to_user(0) == 0.0
        assert kin.steps_to_user(400) == pytest.approx(10.0) # 90 deg output
        assert kin.steps_to_user(-400) == pytest.approx(-10.0) # -90 deg output
        # 200 steps = 45 deg output
        # x = 10 * sin(45 deg) = 10 * (sqrt(2)/2) = 7.071
        assert kin.steps_to_user(200) == pytest.approx(10.0 * math.sin(math.radians(45)))

    def test_eccentric_clamping(self):
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0)
        # User value > arm_length should be clamped for angle calculation
        # motor_output_angle for 12.0 should be same as for 10.0 (90 deg)
        assert kin.user_to_steps(12.0) == kin.user_to_steps(10.0) # 400 steps
        assert kin.user_to_steps(-12.0) == kin.user_to_steps(-10.0) # -400 steps

    def test_eccentric_speed_conversion_simplified_at_center(self):
        # Speed conversion is simplified assuming motion near theta=0 (center)
        kin = EccentricKinematics(steps_per_revolution=1600, arm_length=10.0, gear_ratio=1.0)
        # User speed = 10 mm/s. At center, d_theta/dt = (dx/dt) / L = 10/10 = 1 rad/s
        # 1 rad/s = approx 57.3 deg/s
        # Output revs/sec = (57.3 deg/s) / (360 deg/rev) = ~0.159 revs/sec
        # Motor revs/sec = 0.159 revs/sec
        # Motor RPM = 0.159 * 60 = ~9.55 RPM
        # MKS speed param (simplified) = round(9.55) = 10
        assert kin.user_speed_to_motor_speed(10.0) == 10 # 10 mm/s at center

        # Reverse: MKS param 10 (assumed 10 RPM)
        # Motor RPM = 10
        # Motor revs/sec = 10/60 = 1/6 rps
        # Output revs/sec = 1/6 rps
        # Output angular speed rad/sec = (1/6) * 2*pi = pi/3 rad/s
        # User speed (at center) = L * (d_theta/dt) = 10 * (pi/3) = ~10.47 mm/s
        assert kin.motor_speed_to_user_speed(10) == pytest.approx(10.0 * (math.pi / 3.0))

