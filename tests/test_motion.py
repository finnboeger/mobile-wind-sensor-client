import math

from motion import UnitMotion, to_north_east
from structs import HeadingData, ImuSample, Vector3

GRAVITY_READING: Vector3 = (0.0, 0.0, -9.80665)


def _rolling_motion(heading: float, roll_rate: float) -> UnitMotion:
    """A level unit rolling at the given rate, after one second of samples."""
    motion = UnitMotion()
    samples = [
        ImuSample(
            timestamp=i * 10,
            acceleration=GRAVITY_READING,
            angular_rate=(roll_rate, 0.0, 0.0),
        )
        for i in range(101)
    ]
    motion.add(samples, [HeadingData(heading=heading, timestamp=0)])
    return motion


def test_to_north_east() -> None:
    forward = to_north_east((1.0, 0.0, 0.0), 0.0, 0.0, math.radians(90))
    assert abs(forward.x) < 1e-9
    assert abs(forward.y - 1) < 1e-9
    # the right axis points up when rolled 90° to the left
    right = to_north_east((0.0, 1.0, 0.0), math.radians(-90), 0.0, 0.0)
    assert abs(right.x) < 1e-9
    assert abs(right.y) < 1e-9


def test_masthead_moves_with_the_roll() -> None:
    motion = _rolling_motion(heading=0.0, roll_rate=0.5)
    # wind sensor 2 m above the IMU, rolling to the right moves it to the east.
    # The attitude has rolled 0.5 rad by now, which reduces the horizontal part.
    velocity = motion.lever_arm_velocity(1000, (0.0, 0.0, -2.0))
    expected = 2 * 0.5 * math.cos(motion.attitude.roll)
    assert abs(velocity.x) < 1e-6
    assert abs(velocity.y - expected) < 0.05


def test_lever_arm_follows_the_heading() -> None:
    motion = _rolling_motion(heading=math.radians(90), roll_rate=0.1)
    # facing east, rolling to the right moves the masthead to the south
    velocity = motion.lever_arm_velocity(1000, (0.0, 0.0, -1.0))
    assert velocity.x < -0.09
    assert abs(velocity.y) < 0.01


def test_lever_arm_is_zero_without_recent_samples() -> None:
    motion = _rolling_motion(heading=0.0, roll_rate=0.5)
    velocity = motion.lever_arm_velocity(5000, (0.0, 0.0, -2.0))
    assert (velocity.x, velocity.y) == (0, 0)
    assert motion.angular_rate_at(5000) is None
    assert motion.angular_rate_at(1050) is not None
