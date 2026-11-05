import math

import pytest

from aggregate import POSITION_WAIT_S, Second, SecondAggregator
from structs import CorrectedApparentWindData, PositionData, TrueWindData


def wind(timestamp: int, speed: float = 5.0, angle_deg: float = 45.0) -> tuple:
    angle = math.radians(angle_deg)
    return (
        CorrectedApparentWindData(
            wind_speed=speed + 1,
            wind_angle=angle,
            timestamp=timestamp,
        ),
        TrueWindData(wind_speed=speed, wind_angle=angle, timestamp=timestamp),
    )


def position(timestamp: int, latitude: float = 54.0) -> PositionData:
    return PositionData(
        timestamp=timestamp,
        valid=True,
        latitude=latitude,
        longitude=10.0,
        altitude=0,
        geoidal_separation=0,
        speed=2.0,
        true_course=90.0,
    )


def test_second_finishes_once_both_streams_moved_past_it() -> None:
    aggregator = SecondAggregator()
    assert aggregator.add_wind(wind(1_000, speed=4.0)) == []
    assert aggregator.add_wind(wind(1_500, speed=6.0)) == []
    assert aggregator.add_position(position(1_200, latitude=54.0)) == []
    assert aggregator.add_position(position(1_800, latitude=54.2)) == []
    # wind has moved on, the position not yet
    assert aggregator.add_wind(wind(2_100)) == []
    [second] = aggregator.add_position(position(2_000))
    assert second.timestamp_s == 1
    assert second.true_wind.magnitude == pytest.approx(5.0)
    assert second.apparent_wind.magnitude == pytest.approx(6.0)
    assert second.position is not None
    assert second.position.latitude == pytest.approx(54.1)
    assert second.position.speed == pytest.approx(2.0)


def test_second_without_position_finishes_after_the_wait() -> None:
    aggregator = SecondAggregator()
    aggregator.add_position(position(500))
    aggregator.add_wind(wind(1_000))
    finished: list[Second] = []
    for s in range(2, 2 + POSITION_WAIT_S):
        finished += aggregator.add_wind(wind(s * 1_000))
    assert [s.timestamp_s for s in finished] == [1]
    assert finished[0].position is None


def test_seconds_without_wind_are_skipped() -> None:
    aggregator = SecondAggregator()
    aggregator.add_wind(wind(1_000))
    aggregator.add_position(position(1_000))
    aggregator.add_position(position(2_000))
    aggregator.add_position(position(3_000))
    finished = aggregator.add_wind(wind(4_000))
    assert [s.timestamp_s for s in finished] == [1]
    # the positions of the seconds 2 and 3 aren't used for second 4
    finished = aggregator.add_position(position(5_000))
    finished += aggregator.add_wind(wind(5_000))
    assert [(s.timestamp_s, s.position) for s in finished] == [(4, None)]
