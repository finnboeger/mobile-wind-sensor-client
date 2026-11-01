import math

from attitude import GRAVITY, AttitudeEstimator
from structs import ImuSample, Vector3


def _gravity_reading(roll: float, pitch: float) -> Vector3:
    """Specific force of a unit at rest, in its axes (x forward, y right, z down)."""
    return (
        GRAVITY * math.sin(pitch),
        -GRAVITY * math.sin(roll) * math.cos(pitch),
        -GRAVITY * math.cos(roll) * math.cos(pitch),
    )


def _sample(
    timestamp: int,
    acceleration: Vector3,
    angular_rate: Vector3 = (0.0, 0.0, 0.0),
) -> ImuSample:
    return ImuSample(
        timestamp=timestamp,
        acceleration=acceleration,
        angular_rate=angular_rate,
    )


def test_first_sample_gives_roll_and_pitch() -> None:
    estimator = AttitudeEstimator()
    estimator.update(_sample(0, _gravity_reading(math.radians(10), math.radians(-5))))
    assert abs(math.degrees(estimator.roll) - 10) < 1e-6
    assert abs(math.degrees(estimator.pitch) + 5) < 1e-6


def test_gyro_carries_the_attitude_through_a_roll() -> None:
    estimator = AttitudeEstimator(proportional_gain=0, integral_gain=0)
    estimator.update(_sample(0, _gravity_reading(0, 0)))
    # roll right at 10 °/s for 2 s at 100 Hz
    rate = math.radians(10)
    for i in range(1, 201):
        roll = rate * i / 100
        estimator.update(_sample(i * 10, _gravity_reading(roll, 0), (rate, 0.0, 0.0)))
    assert abs(math.degrees(estimator.roll) - 20) < 0.1


def test_learns_a_mounting_tilt_and_gyro_bias() -> None:
    estimator = AttitudeEstimator()
    estimator.update(_sample(0, _gravity_reading(0, 0)))
    bias = math.radians(0.5)
    tilt = math.radians(8)
    # the mast leans 8°, the gyro has a bias about x; 10 minutes at 50 Hz
    for i in range(1, 30_000):
        estimator.update(_sample(i * 20, _gravity_reading(tilt, 0), (bias, 0.0, 0.0)))
    assert abs(math.degrees(estimator.roll) - 8) < 0.2
    assert abs(estimator.gyro_bias[0] - bias) < math.radians(0.05)


def test_short_accelerations_barely_tilt_the_estimate() -> None:
    estimator = AttitudeEstimator()
    estimator.update(_sample(0, _gravity_reading(0, 0)))
    # a wave pushes the unit forward with 1.5 m/s² for 2 s
    for i in range(1, 101):
        down = _gravity_reading(0, 0)
        estimator.update(_sample(i * 20, (down[0] - 1.5, down[1], down[2])))
    # the accelerometer alone would read 8.7° of pitch
    assert abs(math.degrees(estimator.pitch)) < 1.0


def test_heading_rate_is_the_yaw_rate_about_the_vertical() -> None:
    estimator = AttitudeEstimator()
    estimator.update(_sample(0, _gravity_reading(0, 0)))
    turning = _sample(10, _gravity_reading(0, 0), (0.0, 0.0, math.radians(5)))
    assert abs(math.degrees(estimator.heading_rate(turning)) - 5) < 1e-6

    # heeled 20°, the same turn shows up partly on the y axis
    estimator = AttitudeEstimator()
    roll = math.radians(20)
    estimator.update(_sample(0, _gravity_reading(roll, 0)))
    rate = math.radians(5)
    turning = _sample(
        10,
        _gravity_reading(roll, 0),
        (0.0, rate * math.sin(roll), rate * math.cos(roll)),
    )
    assert abs(math.degrees(estimator.heading_rate(turning)) - 5) < 1e-6
