"""
Simulated IMU and magnetometer samples of a measurement unit on a RIB.

The world frame is north, east, down. The unit's attitude is given as heading,
pitch and roll (applied in that order), the usual aircraft convention that the
attitude estimator also uses.
"""

import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field

from structs import ImuSample, MagnetometerSample, Vector3

GRAVITY = 9.80665
#: earth's field near Hamburg in µT, north, east, down
EARTH_FIELD: Vector3 = (18.0, 0.0, 47.0)

Matrix = list[list[float]]


def world_to_unit(heading: float, pitch: float, roll: float) -> Matrix:
    """Return the rotation from north, east, down to the unit's axes."""
    ch, sh = math.cos(heading), math.sin(heading)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return [
        [cp * ch, cp * sh, -sp],
        [sr * sp * ch - cr * sh, sr * sp * sh + cr * ch, sr * cp],
        [cr * sp * ch + sr * sh, cr * sp * sh - sr * ch, cr * cp],
    ]


def rotate(matrix: Matrix, vector: Vector3) -> Vector3:
    x, y, z = (sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))
    return (x, y, z)


def body_rates(
    pitch: float,
    roll: float,
    rates: Vector3,
) -> Vector3:
    """Angular rate in the unit's axes from the heading, pitch and roll rates."""
    heading_rate, pitch_rate, roll_rate = rates
    return (
        roll_rate - heading_rate * math.sin(pitch),
        pitch_rate * math.cos(roll) + heading_rate * math.cos(pitch) * math.sin(roll),
        -pitch_rate * math.sin(roll) + heading_rate * math.cos(pitch) * math.cos(roll),
    )


#: heading, pitch and roll in rad at a time in seconds
Attitude = Callable[[float], tuple[float, float, float]]


def _derivative(attitude: Attitude, time: float) -> Vector3:
    step = 1e-4
    before, after = attitude(time - step), attitude(time + step)
    heading_rate = ((after[0] - before[0] + math.pi) % math.tau - math.pi) / (2 * step)
    return (
        heading_rate,
        (after[1] - before[1]) / (2 * step),
        (after[2] - before[2]) / (2 * step),
    )


@dataclass(kw_only=True)
class Magnetometer:
    #: constant field of the iron around the sensor in its axes, in µT
    hard_iron: Vector3 = (0.0, 0.0, 0.0)
    #: soft iron distortion, applied to the field in the sensor's axes
    soft_iron: Matrix = field(
        default_factory=lambda: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    )
    #: rotation from the unit's axes to the sensor's axes
    rotation: Matrix = field(
        default_factory=lambda: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    )
    #: standard deviation of the noise in µT
    noise: float = 0.0
    #: extra field in the sensor's axes at a time in seconds, e.g. a drifting offset
    disturbance: Callable[[float], Vector3] = lambda _time: (0.0, 0.0, 0.0)

    def measure(self, unit_field: Vector3, time: float, rng: random.Random) -> Vector3:
        sensor_field = rotate(self.soft_iron, rotate(self.rotation, unit_field))
        extra = self.disturbance(time)
        x, y, z = (
            sensor_field[i] + self.hard_iron[i] + extra[i] + rng.gauss(0, self.noise)
            for i in range(3)
        )
        return (x, y, z)


def simulate(  # noqa: PLR0913
    attitude: Attitude,
    duration: float,
    *,
    magnetometers: list[Magnetometer] | None = None,
    boat_acceleration: Callable[[float], Vector3] = lambda _time: (0.0, 0.0, 0.0),
    gyro_bias: Vector3 = (0.0, 0.0, 0.0),
    imu_rate: float = 104.0,
    magnetometer_rate: float = 20.0,
    earth_field: Vector3 = EARTH_FIELD,
    seed: int = 1,
) -> tuple[list[ImuSample], list[MagnetometerSample]]:
    """
    Simulate the samples of a moving unit.

    :param attitude: heading, pitch and roll in rad over the time in seconds
    :param boat_acceleration: acceleration of the unit in north, east, down, in m/s²
    :return: the IMU and the magnetometer samples
    """
    rng = random.Random(seed)
    magnetometers = magnetometers or [Magnetometer()]
    imu_samples: list[ImuSample] = []
    for index in range(int(duration * imu_rate)):
        time = index / imu_rate
        heading, pitch, roll = attitude(time)
        to_unit = world_to_unit(heading, pitch, roll)
        acceleration = boat_acceleration(time)
        specific_force = rotate(
            to_unit,
            (acceleration[0], acceleration[1], acceleration[2] - GRAVITY),
        )
        rate = body_rates(pitch, roll, _derivative(attitude, time))
        imu_samples.append(
            ImuSample(
                timestamp=round(time * 1000),
                acceleration=specific_force,
                angular_rate=(
                    rate[0] + gyro_bias[0],
                    rate[1] + gyro_bias[1],
                    rate[2] + gyro_bias[2],
                ),
            ),
        )

    magnetometer_samples: list[MagnetometerSample] = []
    for index in range(int(duration * magnetometer_rate)):
        time = (index + 0.5) / magnetometer_rate
        heading, pitch, roll = attitude(time)
        unit_field = rotate(world_to_unit(heading, pitch, roll), earth_field)
        magnetometer_samples.append(
            MagnetometerSample(
                timestamp=round(time * 1000),
                magnetic_fields=tuple(
                    magnetometer.measure(unit_field, time, rng)
                    for magnetometer in magnetometers
                ),
            ),
        )
    return imu_samples, magnetometer_samples


def wrap(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi
