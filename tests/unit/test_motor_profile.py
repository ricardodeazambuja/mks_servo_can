"""Tests for the MKS motion parameter model.

The reference values here come from section 6.1 of the MKS SERVO42D/57D_CAN
user manual V1.0.6, which is the only authority on what the opaque speed and
acceleration parameters actually mean.
"""
import math

import pytest

from mks_servo_can import constants as const
from mks_servo_can import motor_profile as mp
from mks_servo_can.exceptions import ParameterError


class TestSpeedParameter:
    """Speed parameter <-> RPM, including the microstep calibration."""

    @pytest.mark.parametrize("microsteps", [16, 32, 64])
    def test_parameter_equals_rpm_at_calibrated_microsteps(self, microsteps):
        """Manual 6.1: speed=1200 gives 1200 RPM at 16/32/64 subdivisions."""
        assert mp.speed_param_to_rpm(1200, microsteps=microsteps) == 1200.0

    def test_eight_microsteps_doubles_the_speed(self):
        """Manual 6.1 worked example: speed=1200 at 8 subdivisions is 2400 RPM."""
        assert mp.speed_param_to_rpm(1200, microsteps=8) == 2400.0

    def test_one_twenty_eight_microsteps_is_one_eighth(self):
        """Manual 6.1 worked example: speed=1200 at 128 subdivisions is 150 RPM."""
        assert mp.speed_param_to_rpm(1200, microsteps=128) == 150.0

    @pytest.mark.parametrize(
        "work_mode,ceiling",
        [
            (const.MODE_SR_OPEN, 400),
            (const.MODE_CR_OPEN, 400),
            (const.MODE_SR_CLOSE, 1500),
            (const.MODE_CR_CLOSE, 1500),
            (const.MODE_SR_VFOC, 3000),
            (const.MODE_CR_VFOC, 3000),
        ],
    )
    def test_speed_is_clamped_to_the_work_mode_ceiling(self, work_mode, ceiling):
        """Manual 6.1: the motor runs at the mode maximum if asked for more."""
        assert mp.speed_param_to_rpm(3000, 16, work_mode) == float(ceiling)

    @pytest.mark.parametrize("microsteps", [8, 16, 32, 64, 128])
    @pytest.mark.parametrize("rpm", [0, 1, 100, 400, 1500, 2999])
    def test_rpm_round_trips_through_the_parameter(self, microsteps, rpm):
        """Converting RPM -> parameter -> RPM must be stable, up to saturation."""
        reachable = min(rpm, mp.max_rpm_at_microsteps(microsteps))
        param = mp.rpm_to_speed_param(rpm, microsteps=microsteps)
        back = mp.speed_param_to_rpm(param, microsteps=microsteps)
        assert back == pytest.approx(
            reachable, abs=mp.microstep_speed_factor(microsteps)
        )

    @pytest.mark.parametrize(
        "microsteps,expected",
        [(8, 3000.0), (16, 3000.0), (32, 3000.0), (64, 3000.0), (128, 375.0)],
    )
    def test_parameter_range_caps_speed_at_high_microstepping(
        self, microsteps, expected
    ):
        """
        At 128 microsteps each parameter unit is only 0.125 RPM, so the 0-3000
        range tops out at 375 RPM - an eighth of the vFOC ceiling. Easy way to
        lose most of the motor's speed range without noticing.
        """
        assert mp.max_rpm_at_microsteps(microsteps) == pytest.approx(expected)

    def test_max_rpm_also_respects_the_work_mode(self):
        assert mp.max_rpm_at_microsteps(16, const.MODE_SR_OPEN) == 400.0

    def test_parameter_is_clamped_to_protocol_range(self):
        assert mp.rpm_to_speed_param(1e9) == const.MAX_SPEED_PARAM
        assert mp.rpm_to_speed_param(-500) >= 0

    def test_direction_is_not_encoded_in_the_parameter(self):
        """Sign is carried elsewhere in the frame, so magnitude is used."""
        assert mp.rpm_to_speed_param(-600) == mp.rpm_to_speed_param(600)

    def test_calibration_predicate(self):
        assert mp.is_speed_calibrated(32)
        assert not mp.is_speed_calibrated(128)

    @pytest.mark.parametrize("bad", [0, -1, -16])
    def test_non_positive_microsteps_rejected(self, bad):
        with pytest.raises(ParameterError):
            mp.speed_param_to_rpm(100, microsteps=bad)

    def test_negative_speed_parameter_rejected(self):
        with pytest.raises(ParameterError):
            mp.speed_param_to_rpm(-1)


class TestAccelerationParameter:
    """Acceleration parameter semantics: 1 RPM per (256-acc)*50us."""

    def test_manual_worked_example_acc_236(self):
        """Manual 6.1: acc=236 advances 1 RPM per ms, so 0->3000 takes 3 s."""
        assert mp.accel_param_to_rpm_per_second(236) == pytest.approx(1000.0)
        assert mp.ramp_time_seconds(236, 0, 3000) == pytest.approx(3.0)

    def test_maximum_acceleration(self):
        """acc=255 is one 50us tick per RPM."""
        assert mp.accel_param_to_rpm_per_second(255) == pytest.approx(20000.0)
        assert mp.ramp_time_seconds(255, 0, 3000) == pytest.approx(0.15)

    def test_zero_means_no_ramp(self):
        assert math.isinf(mp.accel_param_to_rpm_per_second(0))
        assert math.isinf(mp.accel_param_to_deg_per_s2(0))
        assert mp.ramp_time_seconds(0, 0, 3000) == 0.0

    def test_degrees_conversion(self):
        """1 RPM/s is 6 deg/s^2 at the motor shaft."""
        assert mp.accel_param_to_deg_per_s2(250) == pytest.approx(20000.0)
        assert mp.accel_param_to_deg_per_s2(236) == pytest.approx(6000.0)

    def test_gearing_divides_output_acceleration(self):
        direct = mp.accel_param_to_deg_per_s2(250, gear_ratio=1.0)
        geared = mp.accel_param_to_deg_per_s2(250, gear_ratio=5.0)
        assert geared == pytest.approx(direct / 5.0)

    @pytest.mark.parametrize("param", range(1, 256, 17))
    def test_acceleration_round_trips(self, param):
        target = mp.accel_param_to_deg_per_s2(param)
        assert mp.deg_per_s2_to_accel_param(target) == param

    def test_excessive_request_saturates_rather_than_wrapping_to_zero(self):
        """acc=0 means 'no ramp'; extrapolating past 255 must not land there."""
        assert mp.deg_per_s2_to_accel_param(1e9) == const.MAX_ACCEL_PARAM

    def test_tiny_request_clamps_to_one(self):
        assert mp.deg_per_s2_to_accel_param(1e-6) == 1

    @pytest.mark.parametrize("bad", [-1, 256, 1000])
    def test_out_of_range_rejected(self, bad):
        with pytest.raises(ParameterError):
            mp.accel_param_to_rpm_per_second(bad)

    @pytest.mark.parametrize("bad", [0, -1])
    def test_non_positive_target_rejected(self, bad):
        with pytest.raises(ParameterError):
            mp.deg_per_s2_to_accel_param(bad)


class TestEncoderGeometry:
    """Encoder counts, resolution and speed ceilings."""

    def test_full_revolution(self):
        assert mp.encoder_steps_to_degrees(
            const.ENCODER_PULSES_PER_REVOLUTION
        ) == pytest.approx(360.0)

    def test_resolution_at_direct_drive(self):
        """16384 counts per revolution is ~79 arcsec."""
        assert mp.encoder_resolution_degrees() == pytest.approx(360.0 / 16384)
        assert mp.encoder_resolution_degrees() * 3600 == pytest.approx(79.1, abs=0.1)

    def test_gearing_improves_resolution_proportionally(self):
        assert mp.encoder_resolution_degrees(5.0) == pytest.approx(
            mp.encoder_resolution_degrees(1.0) / 5.0
        )

    @pytest.mark.parametrize("degrees", [0.0, 1.0, -1.0, 90.0, 360.0, -720.5])
    @pytest.mark.parametrize("gear_ratio", [1.0, 3.0, 27.0])
    def test_degrees_round_trip(self, degrees, gear_ratio):
        steps = mp.degrees_to_encoder_steps(degrees, gear_ratio)
        back = mp.encoder_steps_to_degrees(steps, gear_ratio)
        assert back == pytest.approx(
            degrees, abs=mp.encoder_resolution_degrees(gear_ratio)
        )

    def test_max_output_speed_direct_drive(self):
        """3000 RPM is 18000 deg/s."""
        assert mp.max_output_speed_deg_per_s() == pytest.approx(18000.0)

    def test_max_output_speed_respects_gearing_and_mode(self):
        assert mp.max_output_speed_deg_per_s(
            gear_ratio=5.0, work_mode=const.MODE_SR_CLOSE
        ) == pytest.approx(1500.0 / 5.0 * 6.0)

    def test_usable_rpm_override(self):
        """Torque falls off with speed, so callers can substitute a real figure."""
        assert mp.max_output_speed_deg_per_s(
            max_usable_rpm=1000
        ) == pytest.approx(6000.0)

    @pytest.mark.parametrize("bad", [0, -1.0])
    def test_non_positive_gear_ratio_rejected(self, bad):
        for fn in (
            mp.encoder_resolution_degrees,
            mp.max_output_speed_deg_per_s,
        ):
            with pytest.raises(ParameterError):
                fn(bad)


class TestSimulatorSharesTheModel:
    """The simulator must not carry its own copy of these conversions."""

    def test_simulator_imports_library_conversions(self):
        """
        A divergent model in the test double would let the library and the
        thing that validates it drift apart silently.
        """
        simulator_model = pytest.importorskip("mks_simulator.motor_model")
        assert simulator_model.mks_speed_param_to_rpm is not None
        # Same input, same answer, for every mode.
        for work_mode in const.MAX_RPM_BY_WORK_MODE:
            for param in (0, 300, 1200, 3000):
                assert simulator_model.mks_speed_param_to_rpm(
                    param, work_mode
                ) == pytest.approx(
                    mp.speed_param_to_rpm(param, 16, work_mode)
                )
