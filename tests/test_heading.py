import math
import random

from heading import HeadingFilter, wrap_angle
from motion import UnitMotion
from structs import HeadingData, ImuSample

GRAVITY_READING = (0.0, 0.0, -9.80665)


def test_wrap_angle() -> None:
    assert abs(wrap_angle(math.radians(370)) - math.radians(10)) < 1e-9
    assert abs(wrap_angle(math.radians(-190)) - math.radians(170)) < 1e-9


def test_first_compass_reading_initializes_the_heading() -> None:
    heading_filter = HeadingFilter()
    assert heading_filter.heading_at(0) is None
    heading_filter.predict(0, 0.1)
    assert not heading_filter.initialized
    heading_filter.update(0, math.radians(90))
    assert heading_filter.heading_at(0) == math.radians(90)


def test_gyro_follows_a_turn_the_compass_lags_behind() -> None:
    heading_filter = HeadingFilter(compass_lag=1.0)
    heading_filter.update(0, 0.0)
    rate = math.radians(30)
    compass = 0.0
    worst_error = 0.0
    # turn at 30 °/s for 3 s, then hold the heading, with a compass damped by 1 s
    for i in range(1, 1001):
        timestamp = i * 10
        true_heading = rate * min(timestamp, 3000) / 1000
        heading_filter.predict(timestamp, rate if timestamp <= 3000 else 0.0)
        compass += (true_heading - compass) * (1 - math.exp(-0.01))
        if i % 10 == 0:
            heading_filter.update(timestamp, compass)
        heading = heading_filter.heading_at(timestamp)
        assert heading is not None
        worst_error = max(worst_error, abs(wrap_angle(heading - true_heading)))
    # the compass alone is up to 30° behind
    assert math.degrees(worst_error) < 3


def test_compass_corrects_the_gyro_bias() -> None:
    rng = random.Random(1)
    heading_filter = HeadingFilter()
    heading_filter.update(0, math.radians(45))
    bias = math.radians(0.5)
    # holding a steady heading for 10 minutes with a biased gyro
    for i in range(1, 60_001):
        timestamp = i * 10
        heading_filter.predict(timestamp, bias)
        if i % 10 == 0:
            heading_filter.update(
                timestamp,
                math.radians(45) + rng.gauss(0, math.radians(2)),
            )
    assert abs(math.degrees(heading_filter.bias) - 0.5) < 0.1
    heading = heading_filter.heading_at(600_000)
    assert heading is not None
    assert abs(math.degrees(heading) - 45) < 1


def test_heading_crosses_north() -> None:
    heading_filter = HeadingFilter(compass_lag=0)
    heading_filter.update(0, math.radians(355))
    for i in range(1, 101):
        heading_filter.predict(i * 10, math.radians(10))
    heading = heading_filter.heading_at(1000)
    assert heading is not None
    assert abs(math.degrees(heading) - 5) < 0.5
    heading_filter.update(1000, math.radians(5))
    heading = heading_filter.heading_at(1000)
    assert heading is not None
    assert abs(math.degrees(heading) - 5) < 0.5


def test_heading_follows_the_compass_without_gyro() -> None:
    heading_filter = HeadingFilter()
    heading_filter.update(0, 0.0)
    heading_filter.predict(10, 0.0)
    heading_filter.update(5000, math.radians(120))
    heading = heading_filter.heading_at(5000)
    assert heading is not None
    assert abs(math.degrees(heading) - 120) < 1e-6


def test_unit_motion_applies_events_in_time_order() -> None:
    motion = UnitMotion()
    rate = math.radians(20)
    samples = [
        ImuSample(
            timestamp=i * 10,
            acceleration=GRAVITY_READING,
            angular_rate=(0.0, 0.0, rate),
        )
        for i in range(101)
    ]
    # the compass reading comes first in time but last in the list
    motion.add(samples, [HeadingData(heading=0.0, timestamp=0)])
    heading = motion.heading_at(1000)
    assert heading is not None
    assert abs(math.degrees(heading) - 20) < 1
