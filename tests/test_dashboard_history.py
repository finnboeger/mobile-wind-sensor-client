import json
import math

import pytest

from aggregate import Second, ShortPositionData
from dashboard.history import MeasurementHistory
from utils.vector import PolarCoordinates


def second(
    timestamp_s: int,
    speed: float = 5.0,
    angle_deg: float = 0.0,
    *,
    with_position: bool = True,
) -> Second:
    wind = PolarCoordinates(angle=math.radians(angle_deg), magnitude=speed)
    return Second(
        timestamp_s=timestamp_s,
        true_wind=wind,
        apparent_wind=PolarCoordinates(angle=wind.angle, magnitude=speed + 1),
        position=ShortPositionData(
            latitude=54.0,
            longitude=10.0,
            speed=2.0,
            true_course=450.0,
        )
        if with_position
        else None,
    )


def test_history_turns_seconds_into_records() -> None:
    history = MeasurementHistory()
    history.add_second(second(1, 4.921, -10))
    (record,) = history.since()
    assert record["ts"] == 1_000
    assert record["true_wind_speed_mps"] == pytest.approx(4.92)
    assert record["true_wind_dir_deg"] == pytest.approx(350.0)
    assert record["apparent_wind_speed_mps"] == pytest.approx(5.92)
    assert record["sensor_speed_mps"] == 2.0
    assert record["sensor_heading_deg"] == 90.0


def test_history_leaves_out_missing_positions() -> None:
    history = MeasurementHistory()
    history.add_second(second(1, with_position=False))
    (record,) = history.since()
    assert "gps_lat" not in record


def test_history_since_returns_newer_records() -> None:
    history = MeasurementHistory(max_records=3)
    for timestamp_s in range(1, 6):
        history.add_second(second(timestamp_s))
    assert [r["ts"] for r in history.since()] == [3_000, 4_000, 5_000]
    assert [r["ts"] for r in history.since(3_500)] == [4_000, 5_000]
    assert [r["ts"] for r in history.since(4_000)] == [5_000]
    assert history.since(5_000) == []


def test_history_directions_stay_below_360() -> None:
    history = MeasurementHistory()
    history.add_second(second(1, angle_deg=359.97))
    (record,) = history.since()
    assert record["true_wind_dir_deg"] == 0.0
    assert record["apparent_wind_dir_deg"] == 0.0


def test_history_since_json_matches_since() -> None:
    history = MeasurementHistory()
    for t in range(1, 6):
        history.add_second(second(t))
    assert json.loads(history.since_json()) == history.since()
    assert json.loads(history.since_json(3_000)) == history.since(3_000)
    assert [r["ts"] for r in json.loads(history.since_json(limit=2))] == [4_000, 5_000]
    assert [r["ts"] for r in json.loads(history.since_json(4_500, 2))] == [5_000]
    assert history.since_json(5_000) == "[]"


def test_history_replaces_nan_with_none() -> None:
    history = MeasurementHistory()
    history.add_second(second(1, float("nan")))
    (record,) = json.loads(history.since_json())
    assert record["true_wind_speed_mps"] is None
