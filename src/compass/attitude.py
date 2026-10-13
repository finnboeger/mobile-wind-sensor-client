"""
Roll and pitch from the gyro and accelerometer, and tilt compensation.

All vectors are in the boat's body frame: x points forward, y to starboard and z down.
Rates are positive clockwise around each axis when looking along it, so a positive
z rate turns the bow to starboard.

The magnetic field dips steeply in northern Europe (about 69° near Hamburg), so every
degree of tilt error turns into up to 2.6° of heading error. The accelerometer alone
is no good as a tilt reference on a moving boat, because waves and turns accelerate
the sensor. The filter below therefore propagates the direction of gravity with the
gyro and only slowly pulls it towards the accelerometer (a Mahony complementary
filter), after subtracting the boat's own acceleration where it is known.
"""

import math
from dataclasses import dataclass

from utils.linalg import Vector3, cross, norm

#: standard gravity in m/s²
GRAVITY = 9.80665

#: proportional gain in 1/s that pulls the gravity estimate towards the accelerometer.
#: Its inverse is the time constant over which wave accelerations are averaged.
DEFAULT_PROPORTIONAL_GAIN = 0.1
#: integral gain in 1/s² for the roll and pitch gyro bias
DEFAULT_INTEGRAL_GAIN = 0.005
#: accelerometer samples whose magnitude differs from gravity by more than this
#: fraction are not used to correct the tilt
DEFAULT_ACCELERATION_TOLERANCE = 0.1
#: limit for the estimated gyro bias in rad/s
_MAX_GYRO_BIAS = math.radians(5)


@dataclass(frozen=True, kw_only=True)
class Attitude:
    #: roll in radians, positive with the starboard side down
    roll: float
    #: pitch in radians, positive with the bow up
    pitch: float


def attitude_from_down(down: Vector3) -> Attitude:
    """Return roll and pitch for the direction of gravity in the body frame."""
    return Attitude(
        roll=math.atan2(down[1], down[2]),
        pitch=math.atan2(-down[0], math.hypot(down[1], down[2])),
    )


def level(vector: Vector3, attitude: Attitude) -> Vector3:
    """
    Rotate a body frame vector into the level frame.

    The level frame has the same heading as the boat, but no roll or pitch: x is the
    horizontal direction the bow points to, y is horizontal to starboard, z is down.
    """
    sin_roll, cos_roll = math.sin(attitude.roll), math.cos(attitude.roll)
    sin_pitch, cos_pitch = math.sin(attitude.pitch), math.cos(attitude.pitch)
    x, y, z = vector
    return (
        x * cos_pitch + y * sin_roll * sin_pitch + z * cos_roll * sin_pitch,
        y * cos_roll - z * sin_roll,
        -x * sin_pitch + y * sin_roll * cos_pitch + z * cos_roll * cos_pitch,
    )


def heading_from_level_field(x: float, y: float) -> float:
    """
    Return the magnetic heading for the horizontal field in the level frame.

    :return: heading in radians in ``[0, 2π)``, clockwise from magnetic north
    """
    return math.atan2(-y, x) % math.tau


def vertical_rate(gyro: Vector3, attitude: Attitude) -> float:
    """Return the rate of turn around the vertical axis (heading rate) in rad/s."""
    cos_pitch = max(math.cos(attitude.pitch), 0.1)
    return (
        gyro[1] * math.sin(attitude.roll) + gyro[2] * math.cos(attitude.roll)
    ) / cos_pitch


class AttitudeFilter:
    """Complementary filter for the direction of gravity in the body frame."""

    def __init__(
        self,
        *,
        proportional_gain: float = DEFAULT_PROPORTIONAL_GAIN,
        integral_gain: float = DEFAULT_INTEGRAL_GAIN,
        acceleration_tolerance: float = DEFAULT_ACCELERATION_TOLERANCE,
    ) -> None:
        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain
        self.acceleration_tolerance = acceleration_tolerance
        #: unit vector pointing down, in the body frame
        self._down: Vector3 | None = None
        self.gyro_bias: Vector3 = (0.0, 0.0, 0.0)
        #: whether the last accelerometer sample was used for a correction
        self.accelerometer_used = False

    @property
    def initialized(self) -> bool:
        return self._down is not None

    @property
    def attitude(self) -> Attitude | None:
        if self._down is None:
            return None
        return attitude_from_down(self._down)

    def update(
        self,
        dt: float,
        gyro: Vector3,
        specific_force: Vector3,
        boat_acceleration: Vector3 = (0.0, 0.0, 0.0),
    ) -> Attitude | None:
        """
        Propagate the attitude by one sample.

        :param dt: time since the previous sample in seconds
        :param gyro: angular rate in rad/s
        :param specific_force: accelerometer reading in m/s². A level sensor at rest
            reads ``(0, 0, -g)``.
        :param boat_acceleration: known acceleration of the boat in the body frame in
            m/s², e.g. the centripetal acceleration in a turn
        :return: the new attitude, or None until the first usable accelerometer sample
        """
        measured_down = tuple(
            a - f for a, f in zip(boat_acceleration, specific_force, strict=True)
        )
        magnitude = norm(measured_down)
        usable = abs(magnitude - GRAVITY) < self.acceleration_tolerance * GRAVITY
        self.accelerometer_used = usable

        if self._down is None:
            if not usable:
                return None
            self._down = (
                measured_down[0] / magnitude,
                measured_down[1] / magnitude,
                measured_down[2] / magnitude,
            )
            return self.attitude

        rate = (
            gyro[0] - self.gyro_bias[0],
            gyro[1] - self.gyro_bias[1],
            gyro[2] - self.gyro_bias[2],
        )
        if usable:
            unit = (
                measured_down[0] / magnitude,
                measured_down[1] / magnitude,
                measured_down[2] / magnitude,
            )
            error = cross(unit, self._down)
            rate = (
                rate[0] + self.proportional_gain * error[0],
                rate[1] + self.proportional_gain * error[1],
                rate[2] + self.proportional_gain * error[2],
            )
            self.gyro_bias = (
                _clamp(self.gyro_bias[0] - self.integral_gain * error[0] * dt),
                _clamp(self.gyro_bias[1] - self.integral_gain * error[1] * dt),
                _clamp(self.gyro_bias[2] - self.integral_gain * error[2] * dt),
            )

        # a vector fixed in the world turns the opposite way in the body frame
        change = cross(rate, self._down)
        down = (
            self._down[0] - change[0] * dt,
            self._down[1] - change[1] * dt,
            self._down[2] - change[2] * dt,
        )
        length = norm(down)
        self._down = (down[0] / length, down[1] / length, down[2] / length)
        return self.attitude

    def state_dict(self) -> dict[str, list[float]]:
        return {"gyro_bias": list(self.gyro_bias)}

    def load_state_dict(self, state: dict[str, list[float]]) -> None:
        bias = state.get("gyro_bias")
        if bias is not None and len(bias) == 3:  # noqa: PLR2004
            self.gyro_bias = (
                _clamp(bias[0]),
                _clamp(bias[1]),
                _clamp(bias[2]),
            )


def _clamp(value: float) -> float:
    return max(-_MAX_GYRO_BIAS, min(_MAX_GYRO_BIAS, value))
