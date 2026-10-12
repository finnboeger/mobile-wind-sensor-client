import math
import random

from kalman import VelocityKalmanFilter
from utils.vector import Vector2D


def _rms(errors: list[float]) -> float:
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def test_first_measurement_initializes_filter() -> None:
    velocity_filter = VelocityKalmanFilter()
    assert velocity_filter.velocity_at(0) is None

    velocity_filter.update(1000, Vector2D(x=2.0, y=-1.0), 0.1)

    velocity = velocity_filter.velocity_at(1000)
    assert velocity is not None
    assert velocity.x == 2.0
    assert velocity.y == -1.0


def test_constant_velocity_reduces_noise() -> None:
    rng = random.Random(1)
    velocity_filter = VelocityKalmanFilter()
    sigma = 0.3
    raw_errors = []
    filtered_errors = []
    for i in range(400):
        timestamp = i * 500
        measurement = Vector2D(x=3.0 + rng.gauss(0, sigma), y=1.0 + rng.gauss(0, sigma))
        velocity_filter.update(timestamp, measurement, sigma)
        estimate = velocity_filter.velocity_at(timestamp)
        assert estimate is not None
        if i > 20:
            raw_errors.append(measurement.x - 3.0)
            filtered_errors.append(estimate.x - 3.0)

    assert _rms(filtered_errors) < 0.7 * _rms(raw_errors)


def test_follows_acceleration_and_extrapolates() -> None:
    velocity_filter = VelocityKalmanFilter()
    # accelerate north with 0.5 m/s², measured at 2 Hz without noise
    for i in range(40):
        timestamp = i * 500
        velocity_filter.update(timestamp, Vector2D(x=0.5 * i * 0.5, y=0.0), 0.05)

    # between two fixes the estimate is closer to the truth than the last fix
    last_time = 39 * 500
    last_fix = 0.5 * last_time / 1000
    expected = 0.5 * (last_time + 400) / 1000
    estimate = velocity_filter.velocity_at(last_time + 400)
    assert estimate is not None
    assert abs(estimate.x - expected) < 0.5 * abs(last_fix - expected)


def test_extrapolation_is_limited() -> None:
    velocity_filter = VelocityKalmanFilter(max_extrapolation_ms=1000)
    for i in range(40):
        velocity_filter.update(i * 500, Vector2D(x=0.5 * i * 0.5, y=0.0), 0.05)

    one_second = velocity_filter.velocity_at(39 * 500 + 1000)
    one_minute = velocity_filter.velocity_at(39 * 500 + 60_000)
    assert one_second is not None
    assert one_minute is not None
    assert one_minute.x == one_second.x


def test_outlier_is_rejected_and_persistent_change_resets() -> None:
    velocity_filter = VelocityKalmanFilter()
    for i in range(20):
        velocity_filter.update(i * 500, Vector2D(x=3.0, y=0.0), 0.1)

    assert not velocity_filter.update(10_000, Vector2D(x=30.0, y=0.0), 0.1)
    estimate = velocity_filter.velocity_at(10_000)
    assert estimate is not None
    assert abs(estimate.x - 3.0) < 0.1

    # the same jump three times in a row is treated as real
    assert not velocity_filter.update(10_500, Vector2D(x=30.0, y=0.0), 0.1)
    assert velocity_filter.update(11_000, Vector2D(x=30.0, y=0.0), 0.1)
    estimate = velocity_filter.velocity_at(11_000)
    assert estimate is not None
    assert estimate.x == 30.0


def test_tack_has_less_error_than_holding_the_last_fix() -> None:
    """
    Simulate a tack and compare against the previous behaviour.

    The boat turns through 90° in 6 s at 3 m/s and sails on for 14 s. GNSS velocity
    arrives at 2 Hz with 0.1 m/s noise, wind at 10 Hz. Previously the wind
    calculation used the average of the fixes received since the last wind message,
    which at 10 Hz wind is the last fix, held until the next one arrives.
    """
    rng = random.Random(2)
    speed = 3.0

    def true_velocity(t: float) -> Vector2D:
        heading = math.radians(45) + math.radians(-90) * min(max(t - 10, 0) / 6, 1)
        return Vector2D(x=speed * math.cos(heading), y=speed * math.sin(heading))

    velocity_filter = VelocityKalmanFilter()
    held: Vector2D | None = None
    filter_errors = []
    held_errors = []
    for step in range(300):  # 30 s at 10 Hz
        timestamp = step * 100
        if step % 5 == 0:
            truth = true_velocity(timestamp / 1000)
            measurement = Vector2D(
                x=truth.x + rng.gauss(0, 0.1),
                y=truth.y + rng.gauss(0, 0.1),
            )
            velocity_filter.update(timestamp, measurement, 0.1)
            held = measurement
        # wind arrives 50 ms after the 10 Hz tick
        wind_time = timestamp + 50
        truth = true_velocity(wind_time / 1000)
        estimate = velocity_filter.velocity_at(wind_time)
        assert estimate is not None
        assert held is not None
        filter_errors.append(math.hypot(estimate.x - truth.x, estimate.y - truth.y))
        held_errors.append(math.hypot(held.x - truth.x, held.y - truth.y))

    assert _rms(filter_errors) < _rms(held_errors)
