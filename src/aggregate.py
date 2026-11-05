"""
One second averages of the wind and the position.

The MQTT upload and the local dashboard both show one record per second, built here
so the two always agree.
"""

import logging
from collections import defaultdict
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import NoReturn

import n2k

from structs import (
    CorrectedApparentWindData,
    PositionData,
    PositionQueue,
    TrueWindData,
    WindData,
    WindOutputQueue,
)
from utils.vector import PolarCoordinates, Vector2D

logger = logging.getLogger(__name__)

#: A second without a position is finished once the wind is this many seconds ahead
#: of it. wind.py stops sending wind once the position is more than 1 s old, so this
#: only happens around a short gap in the positions.
POSITION_WAIT_S = 5
#: Positions this many seconds older than the newest one are dropped if no wind
#: arrives to use them. Positions from seconds without wind otherwise stay until
#: the next second is finished.
MAX_POSITION_BACKLOG_S = 60


def _avg(values: Iterable[float]) -> float:
    values_list = list(values)
    if len(values_list) == 0:
        return 0.0
    return sum(values_list) / len(values_list)


def avg_polar_coordinates(coords: Iterable[PolarCoordinates]) -> PolarCoordinates:
    vectors = [Vector2D.from_polar(c) for c in coords]
    if len(vectors) == 0:
        return PolarCoordinates(angle=0.0, magnitude=0.0)
    vector_sum = sum(vectors, start=Vector2D(x=0, y=0))
    avg_vector = vector_sum / len(vectors)
    return avg_vector.to_polar()


@dataclass(frozen=True)
class ShortPositionData:
    latitude: float
    longitude: float
    speed: float
    true_course: float


def aggregate_position(samples: list[PositionData]) -> ShortPositionData:
    lat = _avg(p.latitude for p in samples)
    lon = _avg(p.longitude for p in samples)

    avg_coordinate = avg_polar_coordinates(
        PolarCoordinates(
            angle=n2k.utils.deg_to_rad(p.true_course),
            magnitude=p.speed,
        )
        for p in samples
    )
    heading = n2k.utils.rad_to_deg(avg_coordinate.angle)
    speed = avg_coordinate.magnitude

    return ShortPositionData(
        latitude=lat,
        longitude=lon,
        speed=speed,
        true_course=heading,
    )


def avg_wind(samples: Iterable[WindData]) -> PolarCoordinates:
    return avg_polar_coordinates(
        PolarCoordinates(
            angle=p.wind_angle,
            magnitude=p.wind_speed,
        )
        for p in samples
    )


@dataclass(frozen=True)
class Second:
    """Averages of one second that had wind data."""

    #: seconds since epoch
    timestamp_s: int
    true_wind: PolarCoordinates
    apparent_wind: PolarCoordinates
    #: None if the GPS sent no position in this second
    position: ShortPositionData | None


class SecondAggregator:
    """
    Collects wind and positions and finishes each second once both have moved past it.

    A second is finished when wind from a later second has arrived, and either a
    position from a later second has arrived or the wind is :data:`POSITION_WAIT_S`
    ahead (then the second has no position). Seconds without wind are skipped.
    """

    def __init__(self) -> None:
        self._wind: dict[int, list[tuple[CorrectedApparentWindData, TrueWindData]]] = (
            defaultdict(list)
        )
        self._positions: dict[int, list[PositionData]] = defaultdict(list)
        self._latest_wind_s: int | None = None
        self._latest_position_s: int | None = None
        self._finished_s: int | None = None

    def add_position(self, position: PositionData) -> list[Second]:
        second = position.timestamp // 1000
        if self._finished_s is not None and second <= self._finished_s:
            return []
        self._positions[second].append(position)
        self._latest_position_s = max(second, self._latest_position_s or second)
        oldest_kept_s = self._latest_position_s - MAX_POSITION_BACKLOG_S
        for old in [s for s in self._positions if s < oldest_kept_s]:
            del self._positions[old]
        return self._finish()

    def add_wind(
        self,
        wind: tuple[CorrectedApparentWindData, TrueWindData],
    ) -> list[Second]:
        # the timestamp of the apparent and true wind is always the same
        second = wind[0].timestamp // 1000
        if self._finished_s is not None and second <= self._finished_s:
            return []
        self._wind[second].append(wind)
        self._latest_wind_s = max(second, self._latest_wind_s or second)
        return self._finish()

    def _finish(self) -> list[Second]:
        finished = []
        for second in sorted(self._wind):
            if self._latest_wind_s is None or second >= self._latest_wind_s:
                break
            position_passed = (
                self._latest_position_s is not None and self._latest_position_s > second
            )
            if not position_passed and self._latest_wind_s - second < POSITION_WAIT_S:
                break
            wind = self._wind.pop(second)
            positions = self._positions.pop(second, [])
            finished.append(
                Second(
                    timestamp_s=second,
                    true_wind=avg_wind(w[1] for w in wind),
                    apparent_wind=avg_wind(w[0] for w in wind),
                    position=aggregate_position(positions) if positions else None,
                ),
            )
            self._finished_s = second
        if self._finished_s is not None:
            for old in [s for s in self._positions if s <= self._finished_s]:
                logger.warning(
                    "No data for timestamp %d. 0 Wind messages, %d Position messages",
                    old,
                    len(self._positions[old]),
                )
                del self._positions[old]
        return finished


def run(
    position_queue: PositionQueue,
    wind_queue: WindOutputQueue,
    on_second: Callable[[Second], None],
) -> NoReturn:
    """Read both queues forever and call ``on_second`` for every finished second."""
    aggregator = SecondAggregator()
    while True:
        # Always block for wind to avoid a busy-loop, seconds only finish with wind
        winds = [wind_queue.get()]
        while not wind_queue.empty():
            winds.append(wind_queue.get())
        seconds: list[Second] = []
        while not position_queue.empty():
            seconds += aggregator.add_position(position_queue.get())
        for wind in winds:
            seconds += aggregator.add_wind(wind)
        for second in seconds:
            on_second(second)
