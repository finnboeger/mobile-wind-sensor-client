"""
Motion of the measurement unit from the IMU and the compass.

On a mast the measurement unit sways with the boat's roll and pitch. The wind sensor
and the GPS antenna sit at some distance from the IMU, so relative to it they move
with the cross product of the angular rate and their position relative to the IMU.
Positions are given in the unit's axes: x forward, y right and z down, in metres.
"""

import bisect
import math
from collections import deque

from attitude import AttitudeEstimator
from heading import HeadingFilter
from structs import HeadingData, ImuSample, Vector3
from utils.vector import Vector2D

#: how long angular rates are kept for lookups, in milliseconds
RATE_HISTORY_MS = 2000
#: maximum distance in milliseconds to the nearest angular rate for a lookup
MAX_RATE_OFFSET_MS = 100


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def to_north_east(
    vector: Vector3,
    roll: float,
    pitch: float,
    heading: float,
) -> Vector2D:
    """
    Rotate a vector from the unit's axes to the world.

    :return: the horizontal part, x is north and y is east
    """
    x, y, z = vector
    # roll about x
    y, z = (
        y * math.cos(roll) - z * math.sin(roll),
        y * math.sin(roll) + z * math.cos(roll),
    )
    # pitch about y
    x = x * math.cos(pitch) + z * math.sin(pitch)
    # heading about the vertical
    return Vector2D(
        x=x * math.cos(heading) - y * math.sin(heading),
        y=x * math.sin(heading) + y * math.cos(heading),
    )


class UnitMotion:
    """Tracks the unit's attitude, heading and angular rate."""

    def __init__(
        self,
        heading_filter: HeadingFilter | None = None,
        *,
        wind_sensor_position: Vector3 = (0.0, 0.0, 0.0),
        gps_antenna_position: Vector3 = (0.0, 0.0, 0.0),
    ) -> None:
        self.attitude = AttitudeEstimator()
        self.heading = heading_filter or HeadingFilter()
        self.wind_sensor_position = wind_sensor_position
        self.gps_antenna_position = gps_antenna_position
        #: timestamps and bias corrected angular rates of the recent samples
        self._rate_times: deque[int] = deque()
        self._rates: deque[Vector3] = deque()

    def add_imu(self, samples: list[ImuSample]) -> None:
        for sample in samples:
            self.attitude.update(sample)
            if not self.attitude.initialized:
                continue
            self.heading.predict(sample.timestamp, self.attitude.heading_rate(sample))
            self._rate_times.append(sample.timestamp)
            self._rates.append(self.attitude.angular_rate(sample))
            while self._rate_times[0] < sample.timestamp - RATE_HISTORY_MS:
                self._rate_times.popleft()
                self._rates.popleft()

    def add_compass(self, heading: HeadingData) -> None:
        self.heading.update(heading.timestamp, heading.heading)

    def add(self, samples: list[ImuSample], headings: list[HeadingData]) -> None:
        """Add IMU samples and compass readings in the order they were measured."""
        events: list[ImuSample | HeadingData] = [*samples, *headings]
        # sorted() is stable, so samples with the same timestamp keep their order
        for event in sorted(events, key=lambda event: event.timestamp):
            if isinstance(event, ImuSample):
                self.add_imu([event])
            else:
                self.add_compass(event)

    def heading_at(self, timestamp: int) -> float | None:
        """Return the heading in rad at the given time, or None if not known yet."""
        return self.heading.heading_at(timestamp)

    def angular_rate_at(self, timestamp: int) -> Vector3 | None:
        """Return the angular rate in rad/s of the sample nearest to the given time."""
        if not self._rate_times:
            return None
        index = bisect.bisect_left(self._rate_times, timestamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(self._rate_times)]
        nearest = min(candidates, key=lambda i: abs(self._rate_times[i] - timestamp))
        if abs(self._rate_times[nearest] - timestamp) > MAX_RATE_OFFSET_MS:
            return None
        return self._rates[nearest]

    def lever_arm_velocity(self, timestamp: int, position: Vector3) -> Vector2D:
        """
        Return the velocity of a point on the unit relative to the IMU.

        :param timestamp: time in milliseconds
        :param position: the point in the unit's axes relative to the IMU, in m
        :return: horizontal velocity, x is north and y is east, in m/s, zero if the
            motion isn't known at that time
        """
        rate = self.angular_rate_at(timestamp)
        heading = self.heading_at(timestamp)
        if rate is None or heading is None or position == (0.0, 0.0, 0.0):
            return Vector2D(x=0, y=0)
        return to_north_east(
            _cross(rate, position),
            self.attitude.roll,
            self.attitude.pitch,
            heading,
        )
