import json
import logging
import threading
from collections.abc import Iterable
from ssl import PROTOCOL_TLS
from typing import Any

import n2k
import paho.mqtt.client as mqtt

from config import Config
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


def _avg(values: Iterable[float]) -> float:
    values_list = list(values)
    if len(values_list) == 0:
        return 0.0
    return sum(values_list) / len(values_list)


def _avg_polar_coordinates(coords: Iterable[PolarCoordinates]) -> PolarCoordinates:
    vectors = [Vector2D.from_polar(c) for c in coords]
    if len(vectors) == 0:
        return PolarCoordinates(angle=0.0, magnitude=0.0)
    vector_sum = sum(vectors, start=Vector2D(x=0, y=0))
    avg_vector = vector_sum / len(vectors)
    return avg_vector.to_polar()


def _aggregate_position(samples: list[PositionData]) -> dict[str, Any]:
    lat = _avg(p.latitude for p in samples)
    lon = _avg(p.longitude for p in samples)

    avg_coordinate = _avg_polar_coordinates(
        PolarCoordinates(
            angle=n2k.utils.deg_to_rad(p.true_course),
            magnitude=p.speed,
        )
        for p in samples
    )
    heading = n2k.utils.rad_to_deg(avg_coordinate.angle)
    speed = avg_coordinate.magnitude

    return {
        "lat": round(lat, 6),
        "lon": round(lon, 6),
        "heading": round(heading),
        "speed": round(speed, 1),
    }


def _avg_wind(
    samples: list[WindData],
) -> PolarCoordinates:
    return _avg_polar_coordinates(
        PolarCoordinates(
            angle=p.wind_angle,
            magnitude=p.wind_speed,
        )
        for p in samples
    )


def worker(position_queue: PositionQueue, wind_queue: WindOutputQueue) -> None:
    config = Config()

    if config.MQTT is None:
        logger.warning("Attempted to start MQTT worker without a configured broker")
        return
    client = mqtt.Client(
        client_id=config.MQTT.CLIENT_ID,
        userdata=None,
        protocol=mqtt.MQTTv5,
    )
    client.tls_set(tls_version=PROTOCOL_TLS)
    client.username_pw_set(username=config.MQTT.USERNAME, password=config.MQTT.PASSWORD)
    client.connect(config.MQTT.BROKER, config.MQTT.PORT)
    client.loop_start()

    # Wait for both queues to produce at least one message.
    wind_buffer: list[tuple[CorrectedApparentWindData, TrueWindData]] = [
        wind_queue.get(),
    ]
    position_buffer: list[PositionData] = [
        position_queue.get(),
    ]

    def sec_position(p: PositionData) -> int:
        return p.timestamp // 1000

    def sec_wind(w: tuple[CorrectedApparentWindData, TrueWindData]) -> int:
        # the timestamp of the apparent and true wind is always the same
        return w[0].timestamp // 1000

    # Start at the first second where both streams have data.
    timestamp_s = max(sec_wind(wind_buffer[0]), sec_position(position_buffer[0]))

    # Drop any buffered data older than the starting second.
    wind_buffer = [w for w in wind_buffer if sec_wind(w) >= timestamp_s]
    position_buffer = [p for p in position_buffer if sec_position(p) >= timestamp_s]

    while True:
        # Always block for at least one wind message to avoid a busy-loop.
        wind_buffer.append(wind_queue.get())
        while not wind_queue.empty():
            wind_buffer.append(wind_queue.get())
        while not position_queue.empty():
            position_buffer.append(position_queue.get())

        # Keep aggregating until we have at least one message beyond timestamp_s
        # in both streams.
        if (
            len(wind_buffer) == 0
            or sec_wind(wind_buffer[-1]) == timestamp_s
            or len(position_buffer) == 0
            or sec_position(position_buffer[-1]) == timestamp_s
        ):
            continue

        current_values = {
            "apparent": [w[0] for w in wind_buffer if sec_wind(w) == timestamp_s],
            "true": [w[1] for w in wind_buffer if sec_wind(w) == timestamp_s],
            "position": [p for p in position_buffer if sec_position(p) == timestamp_s],
        }

        # remove used messages from the buffer
        wind_buffer = [w for w in wind_buffer if sec_wind(w) > timestamp_s]
        position_buffer = [p for p in position_buffer if sec_position(p) > timestamp_s]

        # If one of the buckets is empty,
        # advance to the first second that exists in both.
        if len(current_values["apparent"]) == 0 or len(current_values["position"]) == 0:
            logger.warning(
                "No data for timestamp %d. %d Wind messages, %d Position messages",
                timestamp_s,
                len(current_values["apparent"]),
                len(current_values["position"]),
            )
            oldest_wind_sec = sec_wind(wind_buffer[0])
            oldest_pos_sec = sec_position(position_buffer[0])
            timestamp_s = max(oldest_wind_sec, oldest_pos_sec)
            continue

        avg_true_wind = _avg_wind(current_values["true"])
        avg_apparent_wind = _avg_wind(current_values["apparent"])
        avg_position = _aggregate_position(current_values["position"])

        payload = {
            "timestamp": timestamp_s,
            "gps": avg_position,
            "true": {
                "direction": round(n2k.utils.rad_to_deg(avg_true_wind.angle)),
                "speed": round(avg_true_wind.magnitude, 1),
            },
            "apparent": {
                "direction": round(n2k.utils.rad_to_deg(avg_apparent_wind.angle)),
                "speed": round(avg_apparent_wind.magnitude, 1),
            },
        }

        # we know that both buffers contain at least one message,
        # as we kept waiting for new messages until then
        timestamp_s = max(sec_wind(wind_buffer[0]), sec_position(position_buffer[0]))

        client.publish(config.MQTT.TOPIC, json.dumps(payload), qos=0)


def init(position_queue: PositionQueue, wind_queue: WindOutputQueue) -> None:
    worker_thread = threading.Thread(
        target=worker,
        args=(
            position_queue,
            wind_queue,
        ),
        daemon=True,
    )
    worker_thread.start()
