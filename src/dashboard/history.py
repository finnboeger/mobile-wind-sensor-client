"""
One second measurements for the local dashboard.

The records use the same fields as the ``measurements`` collection of the online
dashboard (mobile-wind-sensor-server), so both can be drawn the same way.
"""

import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from structs import (
    CorrectedApparentWindData,
    PositionData,
    PositionQueue,
    TrueWindData,
    WindOutputQueue,
)
from utils.vector import PolarCoordinates, Vector2D

#: number of one second records kept, enough for the longest time frame (90 min)
DEFAULT_MAX_RECORDS = 2 * 60 * 60
#: positions older than this many milliseconds are dropped if no wind arrives
MAX_POSITION_AGE_MS = 10_000


def _average_direction(vectors: list[Vector2D]) -> PolarCoordinates:
    total = sum(vectors, start=Vector2D(x=0, y=0))
    return (total / len(vectors)).to_polar()


@dataclass
class _Second:
    """Data collected during one second."""

    second: int
    apparent: list[Vector2D] = field(default_factory=list)
    true: list[Vector2D] = field(default_factory=list)


def _to_vector(angle: float, magnitude: float) -> Vector2D:
    return Vector2D.from_polar(PolarCoordinates(angle=angle, magnitude=magnitude))


class MeasurementHistory:
    """Thread safe ring buffer of one second averages."""

    def __init__(self, max_records: int = DEFAULT_MAX_RECORDS) -> None:
        self._lock = threading.Lock()
        self._records: deque[dict[str, Any]] = deque(maxlen=max_records)
        self._current: _Second | None = None
        self._positions: list[PositionData] = []
        self._latest_position: PositionData | None = None

    def add_position(self, position: PositionData) -> None:
        with self._lock:
            # only keep the positions that may still be matched with wind data
            self._positions = [
                p
                for p in self._positions
                if p.timestamp > position.timestamp - MAX_POSITION_AGE_MS
            ]
            self._positions.append(position)
            self._latest_position = position

    def add_wind(
        self,
        apparent: CorrectedApparentWindData,
        true: TrueWindData,
    ) -> None:
        second = apparent.timestamp // 1000
        with self._lock:
            if self._current is not None and second > self._current.second:
                self._finish_second()
            if self._current is None or second > self._current.second:
                self._current = _Second(second=second)
            self._current.apparent.append(
                _to_vector(apparent.wind_angle, apparent.wind_speed),
            )
            self._current.true.append(_to_vector(true.wind_angle, true.wind_speed))

    def _finish_second(self) -> None:
        current = self._current
        if current is None or not current.true:
            return
        positions = [
            p for p in self._positions if p.timestamp // 1000 <= current.second
        ]
        self._positions = [
            p for p in self._positions if p.timestamp // 1000 > current.second
        ]
        if not positions and self._latest_position is not None:
            positions = [self._latest_position]

        record: dict[str, Any] = {"ts": current.second * 1000}
        true = _average_direction(current.true)
        apparent = _average_direction(current.apparent)
        record["true_wind_speed_mps"] = round(true.magnitude, 2)
        record["true_wind_dir_deg"] = round(math.degrees(true.angle) % 360, 1)
        record["apparent_wind_speed_mps"] = round(apparent.magnitude, 2)
        record["apparent_wind_dir_deg"] = round(math.degrees(apparent.angle) % 360, 1)
        if positions:
            movement = _average_direction(
                [_to_vector(math.radians(p.true_course), p.speed) for p in positions],
            )
            record["gps_lat"] = round(
                sum(p.latitude for p in positions) / len(positions),
                6,
            )
            record["gps_lng"] = round(
                sum(p.longitude for p in positions) / len(positions),
                6,
            )
            record["sensor_speed_mps"] = round(movement.magnitude, 2)
            record["sensor_heading_deg"] = round(math.degrees(movement.angle) % 360, 1)
        self._records.append(record)

    def since(self, timestamp: int | None = None) -> list[dict[str, Any]]:
        """Return the records newer than ``timestamp`` (ms), oldest first."""
        with self._lock:
            if timestamp is None:
                return list(self._records)
            return [r for r in self._records if r["ts"] > timestamp]


def position_worker(history: MeasurementHistory, queue: PositionQueue) -> None:
    while True:
        history.add_position(queue.get())


def wind_worker(history: MeasurementHistory, queue: WindOutputQueue) -> None:
    while True:
        apparent, true = queue.get()
        history.add_wind(apparent, true)
