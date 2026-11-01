"""
Roll and pitch of the measurement unit from its gyro and accelerometer.

The estimate is the direction of "down" in the unit's axes. The gyro rotates it
between samples, and the accelerometer slowly pulls it towards the measured
gravity, with an integral term that learns the gyro bias (a Mahony complementary
filter). Waves accelerate the unit for a few seconds at a time, so the
accelerometer is trusted only over tens of seconds. Because the estimate is
learned from gravity, a mast that leans or is set up differently needs no
calibration.
"""

import math

from structs import ImuSample, Vector3

#: standard gravity in m/s²
GRAVITY = 9.80665
#: proportional gain in 1/s, the time constant of the accelerometer correction is
#: its inverse
DEFAULT_PROPORTIONAL_GAIN = 0.05
#: integral gain in 1/s² for the gyro bias
DEFAULT_INTEGRAL_GAIN = 0.0005
#: limit for the estimated gyro bias in rad/s
MAX_GYRO_BIAS = math.radians(5)
#: accelerometer readings further than this from 1 g (relative) are not used
ACCELERATION_TOLERANCE = 0.2
#: gaps between samples longer than this in seconds are not integrated
MAX_TIME_STEP = 0.2
#: an angle within this of ±90° pitch is treated as vertical, in rad
_SINGULARITY = 1e-6


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vector3) -> float:
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def _normalized(a: Vector3) -> Vector3:
    length = _norm(a)
    return (a[0] / length, a[1] / length, a[2] / length)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class AttitudeEstimator:
    """Estimates roll, pitch and the gyro bias of the measurement unit."""

    def __init__(
        self,
        *,
        proportional_gain: float = DEFAULT_PROPORTIONAL_GAIN,
        integral_gain: float = DEFAULT_INTEGRAL_GAIN,
    ) -> None:
        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain
        #: direction of down in the unit's axes, None until the first sample
        self.down: Vector3 | None = None
        #: estimated gyro bias in rad/s
        self.gyro_bias: Vector3 = (0.0, 0.0, 0.0)
        self._time: int | None = None

    @property
    def initialized(self) -> bool:
        return self.down is not None

    @property
    def roll(self) -> float:
        """Roll in rad, positive when the right side is down."""
        if self.down is None:
            return 0.0
        return math.atan2(self.down[1], self.down[2])

    @property
    def pitch(self) -> float:
        """Pitch in rad, positive when the front is up."""
        if self.down is None:
            return 0.0
        return math.asin(max(-1.0, min(1.0, -self.down[0])))

    def angular_rate(self, sample: ImuSample) -> Vector3:
        """Return the sample's angular rate with the estimated gyro bias removed."""
        rate, bias = sample.angular_rate, self.gyro_bias
        return (rate[0] - bias[0], rate[1] - bias[1], rate[2] - bias[2])

    def heading_rate(self, sample: ImuSample) -> float:
        """
        Return how fast the unit's heading changes, in rad/s.

        This is the yaw rate about the vertical, positive turning clockwise seen
        from above, i.e. the rate of change of the compass heading.
        """
        _, q, r = self.angular_rate(sample)
        roll, pitch = self.roll, self.pitch
        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) < _SINGULARITY:
            return 0.0
        return (q * math.sin(roll) + r * math.cos(roll)) / cos_pitch

    def update(self, sample: ImuSample) -> None:
        """Add an IMU sample."""
        acceleration = sample.acceleration
        acceleration_norm = _norm(acceleration)

        if self.down is None or self._time is None:
            if acceleration_norm == 0:
                return
            # the accelerometer measures the reaction to gravity, which points up
            self.down = _normalized(
                (-acceleration[0], -acceleration[1], -acceleration[2]),
            )
            self._time = sample.timestamp
            return

        dt = (sample.timestamp - self._time) / 1000
        self._time = sample.timestamp
        if dt <= 0 or dt > MAX_TIME_STEP:
            return

        rate = self.angular_rate(sample)

        # correction towards the measured gravity, only when the unit isn't
        # accelerating much
        correction: Vector3 = (0.0, 0.0, 0.0)
        if abs(acceleration_norm / GRAVITY - 1) < ACCELERATION_TOLERANCE:
            measured_down = _normalized(
                (-acceleration[0], -acceleration[1], -acceleration[2]),
            )
            # rotation that turns the estimate towards the measurement
            error = _cross(measured_down, self.down)
            correction = (
                self.proportional_gain * error[0],
                self.proportional_gain * error[1],
                self.proportional_gain * error[2],
            )
            self.gyro_bias = (
                _clamp(
                    self.gyro_bias[0] - self.integral_gain * error[0] * dt,
                    MAX_GYRO_BIAS,
                ),
                _clamp(
                    self.gyro_bias[1] - self.integral_gain * error[1] * dt,
                    MAX_GYRO_BIAS,
                ),
                _clamp(
                    self.gyro_bias[2] - self.integral_gain * error[2] * dt,
                    MAX_GYRO_BIAS,
                ),
            )

        omega = (
            rate[0] + correction[0],
            rate[1] + correction[1],
            rate[2] + correction[2],
        )
        # a vector fixed in the world turns the other way in the unit's axes:
        # d(down)/dt = down x omega
        change = _cross(self.down, omega)
        self.down = _normalized(
            (
                self.down[0] + change[0] * dt,
                self.down[1] + change[1] * dt,
                self.down[2] + change[2] * dt,
            ),
        )
