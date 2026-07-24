"""
Physical model of MKS SERVO42D/57D motion parameters.

The MKS CAN protocol expresses motion in two opaque integers: a *speed
parameter* (0-3000) and an *acceleration parameter* (0-255). Neither is in
engineering units, and the mapping from each to real motor behaviour is
described only in prose in section 6.1 of the user manual. This module makes
that mapping explicit and testable.

It is deliberately the single source of truth for these conversions: the
simulator imports the same functions, so the library's idea of what a parameter
means cannot silently drift from the behaviour that tests are validated against.

Two facts from the manual drive everything here:

**Speed.** The parameter maps onto RPM, but is calibrated only for 16, 32 and
64 microsteps. Outside that band the motor scales it relative to 16: the manual
gives ``speed=1200`` producing 2400 RPM at 8 microsteps and 150 RPM at 128. The
result is also clamped to the work mode's ceiling (400 RPM open loop, 1500 RPM
closed loop, 3000 RPM vFOC).

**Acceleration.** The parameter is not an acceleration; it is an inverse rate.
The motor changes speed by exactly 1 RPM every ``(256 - acc) * 50 us``. So
``acc=255`` ramps at 1 RPM per 50 us (120000 deg/s^2 at the shaft) and
``acc=236`` at 1 RPM per ms (6000 deg/s^2). ``acc=0`` means no ramp at all.
"""
import math
from typing import Optional

from . import constants as const
from .exceptions import ParameterError

# Degrees of shaft rotation per revolution. Named for readability below.
_DEGREES_PER_REV = 360.0
_SECONDS_PER_MINUTE = 60.0


def speed_param_to_rpm(
    speed_param: int,
    microsteps: int = 16,
    work_mode: int = const.MODE_SR_VFOC,
) -> float:
    """
    Converts an MKS speed parameter into motor shaft RPM.

    Applies both corrections described in manual section 6.1: the microstep
    calibration factor, and the work mode's speed ceiling.

    Args:
        speed_param: The 0-3000 value sent in a run command.
        microsteps: The motor's subdivision setting (the MSTEP value, e.g. 16).
            The parameter equals RPM only at 16, 32 or 64; other settings are
            scaled relative to 16.
        work_mode: One of the `const.MODE_*` values. Determines the RPM ceiling.

    Returns:
        The resulting steady-state shaft speed in RPM.

    Raises:
        ParameterError: If `speed_param` is negative or `microsteps` is not
            positive.

    Example:
        >>> speed_param_to_rpm(1200, microsteps=16)
        1200.0
        >>> speed_param_to_rpm(1200, microsteps=8)
        2400.0
        >>> speed_param_to_rpm(1200, microsteps=128)
        150.0
    """
    if speed_param < 0:
        raise ParameterError(f"speed_param must be non-negative, got {speed_param}.")
    if microsteps <= 0:
        raise ParameterError(f"microsteps must be positive, got {microsteps}.")

    rpm = float(speed_param) * microstep_speed_factor(microsteps)
    ceiling = const.MAX_RPM_BY_WORK_MODE.get(work_mode, const.MAX_RPM_VFOC_MODE)
    return min(rpm, float(ceiling))


def rpm_to_speed_param(
    rpm: float,
    microsteps: int = 16,
    work_mode: int = const.MODE_SR_VFOC,
) -> int:
    """
    Converts a desired motor shaft RPM into an MKS speed parameter.

    The inverse of `speed_param_to_rpm`. The result is rounded to the nearest
    integer and clamped to the protocol's 0-3000 range; a request faster than
    the work mode allows will therefore silently saturate, which mirrors what
    the motor itself does.

    Args:
        rpm: Desired shaft speed in RPM. The sign is ignored - direction is
            expressed separately in the CAN frame.
        microsteps: The motor's subdivision setting.
        work_mode: One of the `const.MODE_*` values.

    Returns:
        A speed parameter in the range 0 to `const.MAX_SPEED_PARAM`.

    Raises:
        ParameterError: If `microsteps` is not positive.
    """
    if microsteps <= 0:
        raise ParameterError(f"microsteps must be positive, got {microsteps}.")

    ceiling = const.MAX_RPM_BY_WORK_MODE.get(work_mode, const.MAX_RPM_VFOC_MODE)
    effective_rpm = min(abs(float(rpm)), float(ceiling))
    param = int(round(effective_rpm / microstep_speed_factor(microsteps)))
    return max(0, min(param, const.MAX_SPEED_PARAM))


def microstep_speed_factor(microsteps: int) -> float:
    """
    Returns the RPM-per-speed-parameter factor for a microstep setting.

    Manual section 6.1: the speed parameter is calibrated at 16, 32 and 64
    microsteps, where one unit of parameter is one RPM. Other settings scale
    relative to 16.

    Args:
        microsteps: The motor's subdivision setting.

    Returns:
        RPM produced per unit of speed parameter.

    Raises:
        ParameterError: If `microsteps` is not positive.
    """
    if microsteps <= 0:
        raise ParameterError(f"microsteps must be positive, got {microsteps}.")
    if microsteps in const.SPEED_CALIBRATED_MICROSTEPS:
        return 1.0
    return 16.0 / float(microsteps)


def max_rpm_at_microsteps(
    microsteps: int = 16, work_mode: int = const.MODE_SR_VFOC
) -> float:
    """
    Returns the fastest RPM actually reachable at a microstep setting.

    Two independent ceilings apply, and the lower one wins. The work mode caps
    speed at 400/1500/3000 RPM, but at high subdivisions the 0-3000 parameter
    range runs out first: at 128 microsteps each unit is only 0.125 RPM, so even
    `speed_param=3000` reaches just 375 RPM - below the open-loop ceiling, let
    alone vFOC.

    This is a common way to lose most of a motor's speed range without noticing.

    Args:
        microsteps: The motor's subdivision setting.
        work_mode: One of the `const.MODE_*` values.

    Returns:
        The reachable maximum in RPM.

    Raises:
        ParameterError: If `microsteps` is not positive.

    Example:
        >>> max_rpm_at_microsteps(16)
        3000.0
        >>> max_rpm_at_microsteps(128)
        375.0
    """
    return speed_param_to_rpm(const.MAX_SPEED_PARAM, microsteps, work_mode)


def is_speed_calibrated(microsteps: int) -> bool:
    """
    Reports whether a microstep setting is one where speed parameter equals RPM.

    Useful for warning users away from settings that make speed commands
    surprising.

    Args:
        microsteps: The motor's subdivision setting.

    Returns:
        True if the setting is 16, 32 or 64.
    """
    return microsteps in const.SPEED_CALIBRATED_MICROSTEPS


def accel_param_to_rpm_per_second(accel_param: int) -> float:
    """
    Converts an MKS acceleration parameter into RPM per second.

    Manual section 6.1: the motor changes speed by 1 RPM every
    ``(256 - acc) * 50 us``, so the rate is the reciprocal of that interval.
    A parameter of 0 means the motor jumps straight to the commanded speed.

    Args:
        accel_param: The 0-255 value sent in a run command.

    Returns:
        Acceleration in RPM/s, or `math.inf` when `accel_param` is 0.

    Raises:
        ParameterError: If `accel_param` is outside 0-255.

    Example:
        >>> accel_param_to_rpm_per_second(236)
        1000.0
        >>> accel_param_to_rpm_per_second(255)
        20000.0
    """
    _validate_accel_param(accel_param)
    if accel_param == 0:
        return math.inf
    interval = (256 - accel_param) * const.ACCEL_TICK_SECONDS
    return 1.0 / interval


def accel_param_to_deg_per_s2(accel_param: int, gear_ratio: float = 1.0) -> float:
    """
    Converts an MKS acceleration parameter into output-shaft deg/s^2.

    This is the form needed to reason about a real mechanism: compare it against
    the angular acceleration your application demands, or against what your
    payload's inertia and the motor's torque actually permit.

    Args:
        accel_param: The 0-255 value sent in a run command.
        gear_ratio: Reduction between motor and output. A ratio above 1 means
            the output turns slower than the motor, so its acceleration is
            divided by the ratio.

    Returns:
        Acceleration of the output shaft in deg/s^2, or `math.inf` when
        `accel_param` is 0.

    Raises:
        ParameterError: If `accel_param` is out of range or `gear_ratio` is not
            positive.

    Example:
        >>> accel_param_to_deg_per_s2(250)
        20000.0
    """
    if gear_ratio <= 0:
        raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")
    rpm_per_s = accel_param_to_rpm_per_second(accel_param)
    if math.isinf(rpm_per_s):
        return math.inf
    # 1 RPM = 360 deg / 60 s = 6 deg/s at the motor shaft.
    motor_deg_per_s2 = rpm_per_s * (_DEGREES_PER_REV / _SECONDS_PER_MINUTE)
    return motor_deg_per_s2 / gear_ratio


def deg_per_s2_to_accel_param(
    deg_per_s2: float, gear_ratio: float = 1.0
) -> int:
    """
    Finds the acceleration parameter that gets closest to a target deg/s^2.

    The parameter space is coarse and strongly non-linear at the top end (254
    and 255 differ by a factor of two), so this returns the nearest achievable
    setting rather than an exact inverse. A request faster than `acc=255` can
    deliver saturates at 255 rather than returning 0, because 0 means "no ramp
    at all" and is rarely what a caller extrapolating a curve wants.

    Args:
        deg_per_s2: Desired output-shaft angular acceleration.
        gear_ratio: Reduction between motor and output.

    Returns:
        An acceleration parameter in the range 1 to `const.MAX_ACCEL_PARAM`.

    Raises:
        ParameterError: If `deg_per_s2` is not positive or `gear_ratio` is not
            positive.
    """
    if deg_per_s2 <= 0:
        raise ParameterError(f"deg_per_s2 must be positive, got {deg_per_s2}.")
    if gear_ratio <= 0:
        raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")

    motor_deg_per_s2 = deg_per_s2 * gear_ratio
    rpm_per_s = motor_deg_per_s2 / (_DEGREES_PER_REV / _SECONDS_PER_MINUTE)
    if rpm_per_s <= 0:
        return 1

    # Invert t = (256 - acc) * tick and rate = 1 / t.
    ideal = 256.0 - 1.0 / (rpm_per_s * const.ACCEL_TICK_SECONDS)
    param = int(round(ideal))
    return max(1, min(param, const.MAX_ACCEL_PARAM))


def ramp_time_seconds(
    accel_param: int, from_rpm: float = 0.0, to_rpm: float = 3000.0
) -> float:
    """
    Computes how long the motor takes to ramp between two speeds.

    Because the ramp advances in fixed 1 RPM steps, the time is simply the
    number of steps times the per-step interval.

    Args:
        accel_param: The 0-255 value sent in a run command.
        from_rpm: Starting speed in RPM.
        to_rpm: Target speed in RPM.

    Returns:
        Ramp duration in seconds. Zero when `accel_param` is 0 (no ramp) or the
        two speeds are equal.

    Raises:
        ParameterError: If `accel_param` is outside 0-255.

    Example:
        >>> ramp_time_seconds(236, 0, 3000)
        3.0
    """
    _validate_accel_param(accel_param)
    if accel_param == 0:
        return 0.0
    steps = abs(float(to_rpm) - float(from_rpm))
    return steps * (256 - accel_param) * const.ACCEL_TICK_SECONDS


def encoder_steps_to_degrees(steps: float, gear_ratio: float = 1.0) -> float:
    """
    Converts raw encoder counts into degrees of output-shaft rotation.

    Args:
        steps: Raw encoder counts (16384 per motor revolution).
        gear_ratio: Reduction between motor and output.

    Returns:
        Output rotation in degrees.

    Raises:
        ParameterError: If `gear_ratio` is not positive.
    """
    if gear_ratio <= 0:
        raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")
    motor_revs = float(steps) / const.ENCODER_PULSES_PER_REVOLUTION
    return (motor_revs / gear_ratio) * _DEGREES_PER_REV


def degrees_to_encoder_steps(degrees: float, gear_ratio: float = 1.0) -> int:
    """
    Converts degrees of output-shaft rotation into raw encoder counts.

    Args:
        degrees: Output rotation in degrees.
        gear_ratio: Reduction between motor and output.

    Returns:
        Raw encoder counts, rounded to the nearest integer.

    Raises:
        ParameterError: If `gear_ratio` is not positive.
    """
    if gear_ratio <= 0:
        raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")
    motor_revs = (float(degrees) / _DEGREES_PER_REV) * gear_ratio
    return int(round(motor_revs * const.ENCODER_PULSES_PER_REVOLUTION))


def encoder_resolution_degrees(gear_ratio: float = 1.0) -> float:
    """
    Returns the smallest resolvable output-shaft angle, in degrees.

    This is the quantisation floor on any position measurement, and therefore
    on the pointing accuracy of any mechanism built on these motors.

    Args:
        gear_ratio: Reduction between motor and output.

    Returns:
        Degrees per encoder count at the output.

    Raises:
        ParameterError: If `gear_ratio` is not positive.

    Example:
        >>> round(encoder_resolution_degrees(), 6)
        0.021973
    """
    return encoder_steps_to_degrees(1, gear_ratio)


def max_output_speed_deg_per_s(
    gear_ratio: float = 1.0,
    work_mode: int = const.MODE_SR_VFOC,
    max_usable_rpm: Optional[float] = None,
) -> float:
    """
    Returns the fastest output-shaft rotation the drive can sustain.

    Note that a stepper's torque falls off sharply with speed, so the work
    mode's nominal ceiling is usually optimistic for a loaded axis. Pass
    `max_usable_rpm` to substitute a figure you trust for your payload.

    Args:
        gear_ratio: Reduction between motor and output.
        work_mode: One of the `const.MODE_*` values.
        max_usable_rpm: Override for the motor's usable RPM. Defaults to the
            work mode's nominal ceiling.

    Returns:
        Maximum output speed in deg/s.

    Raises:
        ParameterError: If `gear_ratio` is not positive.
    """
    if gear_ratio <= 0:
        raise ParameterError(f"gear_ratio must be positive, got {gear_ratio}.")
    ceiling = (
        max_usable_rpm
        if max_usable_rpm is not None
        else const.MAX_RPM_BY_WORK_MODE.get(work_mode, const.MAX_RPM_VFOC_MODE)
    )
    return (float(ceiling) / gear_ratio) * (_DEGREES_PER_REV / _SECONDS_PER_MINUTE)


def _validate_accel_param(accel_param: int) -> None:
    """
    Raises if an acceleration parameter is outside the protocol's range.

    Args:
        accel_param: The value to check.

    Raises:
        ParameterError: If not within 0 to `const.MAX_ACCEL_PARAM`.
    """
    if not 0 <= accel_param <= const.MAX_ACCEL_PARAM:
        raise ParameterError(
            f"accel_param must be 0-{const.MAX_ACCEL_PARAM}, got {accel_param}."
        )
