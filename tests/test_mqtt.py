import json
import math
import socket
import time
from queue import Queue
from types import SimpleNamespace

import pytest

import aggregate
import mqtt
from structs import CorrectedApparentWindData, PositionData, TrueWindData
from utils.vector import PolarCoordinates

TIMEOUT_S = 10
# MQTT v5 CONNACK: no session present, reason "Success", no properties
CONNACK = bytes([0x20, 0x03, 0x00, 0x00, 0x00])
PUBLISH = 3


def _unused_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _read_exactly(conn: socket.socket, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError
        data += chunk
    return data


def _read_packet(conn: socket.socket) -> tuple[int, bytes]:
    """Read one MQTT packet; returns (packet type, variable header + payload)."""
    packet_type = _read_exactly(conn, 1)[0] >> 4
    length, shift = 0, 0
    while True:
        byte = _read_exactly(conn, 1)[0]
        length |= (byte & 0x7F) << shift
        shift += 7
        if not byte & 0x80:
            break
    return packet_type, _read_exactly(conn, length)


def _parse_publish(body: bytes) -> tuple[str, dict]:
    """Topic and JSON payload of a QoS 0 MQTT v5 PUBLISH."""
    topic_length = int.from_bytes(body[:2])
    topic = body[2 : 2 + topic_length].decode()
    rest = body[2 + topic_length :]
    properties_length = rest[0]  # a single byte for the empty property list paho sends
    return topic, json.loads(rest[1 + properties_length :])


def _wind(timestamp: int) -> tuple[CorrectedApparentWindData, TrueWindData]:
    return (
        CorrectedApparentWindData(wind_speed=5.0, wind_angle=0.5, timestamp=timestamp),
        TrueWindData(wind_speed=4.0, wind_angle=0.7, timestamp=timestamp),
    )


def _position(timestamp: int) -> PositionData:
    return PositionData(
        timestamp=timestamp,
        valid=True,
        latitude=54.3,
        longitude=10.1,
        altitude=0.0,
        geoidal_separation=0.0,
        speed=3.0,
        true_course=90.0,
    )


def test_worker_connects_once_broker_is_reachable_and_publishes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Simulates booting before the 4G modem is online: nothing accepts the
    # connection at first. The worker must keep retrying, connect once the
    # broker is reachable and then publish.
    port = _unused_port()
    config = SimpleNamespace(
        MQTT=SimpleNamespace(
            BROKER="127.0.0.1",
            PORT=port,
            USE_TLS=False,
            USERNAME=None,
            PASSWORD=None,
            CLIENT_ID="test",
            TOPIC="test/topic",
        ),
    )
    monkeypatch.setattr(mqtt, "connection_status", {"state": None, "error_count": 0})

    position_queue: Queue[PositionData] = Queue()
    wind_queue: Queue[tuple[CorrectedApparentWindData, TrueWindData]] = Queue()
    aggregate.init(position_queue, wind_queue, [mqtt.init(config.MQTT)])

    # Let the first connection attempts fail before the broker appears
    time.sleep(1.5)
    with socket.create_server(("127.0.0.1", port)) as server:
        server.settimeout(TIMEOUT_S)
        conn, _ = server.accept()
        with conn:
            conn.settimeout(TIMEOUT_S)
            packet_type, _ = _read_packet(conn)
            assert packet_type == 1  # CONNECT
            conn.sendall(CONNACK)

            # Second 1 is published once both streams have moved past it
            wind_queue.put(_wind(1_000))
            position_queue.put(_position(1_000))
            position_queue.put(_position(2_500))
            wind_queue.put(_wind(2_500))

            packet_type, body = _read_packet(conn)
            while packet_type != PUBLISH:
                packet_type, body = _read_packet(conn)

            # Shown on the local dashboard. Checked while the connection is
            # still open: closing it makes the client report "fail". paho can
            # send the PUBLISH before its network thread has handled the
            # CONNACK and run the connect callback, so give it a moment.
            deadline = time.monotonic() + TIMEOUT_S
            while (
                mqtt.connection_status["state"] != "success"
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            assert mqtt.connection_status["state"] == "success"
            assert mqtt.connection_status["error_count"] >= 1

    topic, payload = _parse_publish(body)
    assert topic == "test/topic"
    assert payload["timestamp"] == 1
    assert payload["true"]["speed"] == 4.0


def test_publish_message_keeps_directions_below_360() -> None:
    published: list[str] = []

    class Client:
        def publish(self, _topic: str, payload: str, qos: int) -> SimpleNamespace:  # noqa: ARG002
            published.append(payload)
            return SimpleNamespace(rc=mqtt.mqtt.MQTT_ERR_SUCCESS)

    wind = PolarCoordinates(angle=math.radians(359.6), magnitude=4.0)
    second = aggregate.Second(
        timestamp_s=1,
        true_wind=wind,
        apparent_wind=wind,
        position=aggregate.ShortPositionData(
            latitude=54.0,
            longitude=10.0,
            speed=3.0,
            true_course=359.7,
        ),
    )
    mqtt.publish_message(Client(), "test/topic", second)  # type: ignore[arg-type]
    payload = json.loads(published[0])
    assert payload["true"]["direction"] == 0
    assert payload["apparent"]["direction"] == 0
    assert payload["gps"]["heading"] == 0
