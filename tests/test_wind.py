import json
import math
import threading
from queue import Empty, Queue

import jsonpickle

import wind
from structs import (
    ApparentWindData,
    HeadingData,
    PositionData,
    WindOutputQueue,
)


def _position(timestamp: int, **kwargs: object) -> PositionData:
    return PositionData(
        timestamp=timestamp,
        valid=True,
        latitude=54.0,
        longitude=10.0,
        altitude=0.0,
        geoidal_separation=0.0,
        speed=3.0,
        true_course=90.0,
        **kwargs,  # pyright: ignore[reportArgumentType]
    )


def test_position_logged_before_velocity_fields_uses_course_and_speed() -> None:
    encoded = jsonpickle.encode(_position(1000))
    assert isinstance(encoded, str)
    # drop the fields added for the filter to mimic an old data log
    state = json.loads(encoded)
    for field in (
        "velocity_north",
        "velocity_east",
        "speed_accuracy",
        "horizontal_accuracy",
        "source",
    ):
        del state[field]
    encoded = json.dumps(state)

    position = jsonpickle.decode(encoded)  # noqa: S301 # nosec
    assert isinstance(position, PositionData)
    assert position.source is None

    velocity = wind.position_velocity(position)
    assert abs(velocity.x) < 1e-9  # north
    assert abs(velocity.y - 3.0) < 1e-9  # east


def test_worker_subtracts_filtered_velocity() -> None:
    position_queue: Queue[PositionData] = Queue()
    heading_queue: Queue[HeadingData] = Queue()
    wind_queue: Queue[ApparentWindData] = Queue()
    output_queue: WindOutputQueue = Queue()

    threading.Thread(
        target=wind.worker,
        args=(position_queue, heading_queue, wind_queue, [output_queue]),
        daemon=True,
    ).start()

    # boat sails east at 3 m/s, heading east, apparent wind from straight ahead
    # at 3 m/s, i.e. there is no true wind
    for i in range(10):
        position_queue.put(
            _position(
                i * 500,
                velocity_north=0.0,
                velocity_east=3.0,
                speed_accuracy=0.1,
                source="NAV-PVT",
            ),
        )
    heading_queue.put(HeadingData(heading=math.pi / 2, timestamp=4500))
    wind_queue.put(ApparentWindData(wind_speed=3.0, wind_angle=0.0, timestamp=4600))

    try:
        _apparent, true_wind = output_queue.get(timeout=2)
    except Empty as e:
        msg = "wind worker produced no output"
        raise AssertionError(msg) from e

    assert true_wind.wind_speed < 1e-6
