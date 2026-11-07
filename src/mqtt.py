import json
import logging
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from ssl import PROTOCOL_TLS
from typing import Any, Final

import n2k
import paho.mqtt.client as mqtt
import paho.mqtt.properties as mqtt_properties
import paho.mqtt.reasoncodes as mqtt_reasoncodes

import aggregate
from config import MQTTConfig

logger = logging.getLogger(__name__)


# Keepalive in seconds. A dead connection (e.g. the modem re-attached with a new IP)
# is only noticed after up to two keepalive intervals, so keep this short.
KEEPALIVE_S: Final = 30
RECONNECT_MIN_DELAY_S: Final = 1
RECONNECT_MAX_DELAY_S: Final = 30
# Messages wait in memory until the broker confirms them, including while offline.
# The queue holds 6 hours of data; beyond that the oldest seconds are dropped.
QOS: Final = 1
MAX_QUEUED_MESSAGES: Final = 6 * 60 * 60
# Messages handed to paho at once. paho itself can only refuse new messages when
# full, so the backlog stays in our own queue, which drops the oldest instead.
# Matches paho's default limit of unacknowledged messages in flight.
WINDOW: Final = 20

# Read by the local dashboard. "state" is None until the first attempt finishes,
# then "success" while connected and "fail" while not. The counters cover the
# time since start: "queued" messages were produced, "confirmed" ones acknowledged
# by the broker and "dropped" ones discarded (e.g. the oldest when the queue is full).
# "last_confirmed" is the Unix time of the latest acknowledgement.
connection_status: dict[str, Any] = {
    "state": None,
    "error_count": 0,
    "queued": 0,
    "confirmed": 0,
    "dropped": 0,
    "last_confirmed": None,
}


def status() -> dict[str, Any]:
    """Return the connection status plus the number of unconfirmed messages."""
    current = dict(connection_status)
    current["pending"] = max(
        0,
        current["queued"] - current["confirmed"] - current["dropped"],
    )
    return current


def connect_callback(
    _client: mqtt.Client | None,
    _userdata: Any,  # noqa: ANN401
    _connect_flags: dict,
    reason_code: mqtt_reasoncodes.ReasonCode,
    _properties: mqtt_properties.Properties,
) -> None:
    if not reason_code.is_failure:
        connection_status["state"] = "success"
        logger.info(
            "Connected to MQTT broker with response: %s",
            reason_code.getName(),
        )
        return

    connection_status["state"] = "fail"
    connection_status["error_count"] += 1
    logger.error(
        "Failed to connect to MQTT broker with error: %s",
        reason_code.getName(),
    )


def connect_fail_callback(_client: mqtt.Client, _userdata: Any) -> None:  # noqa: ANN401
    connection_status["state"] = "fail"
    connection_status["error_count"] += 1
    logger.warning("Failed to connect to MQTT broker due to network error, retrying")


def disconnect_callback(
    _client: mqtt.Client,
    _userdata: Any,  # noqa: ANN401
    reason: mqtt_reasoncodes.ReasonCode | int,
    _properties: mqtt_properties.Properties | None,
) -> None:
    # Called with the paho v1 callback signature for MQTTv5: the reason is a
    # ReasonCode if the broker sent a DISCONNECT, otherwise a local error code.
    connection_status["state"] = "fail"
    logger.warning("Disconnected from MQTT broker (%s), reconnecting", reason)


def _degrees(value: float) -> int:
    """Round a direction to whole degrees from 0 to 359."""
    # rounding after the modulo would turn e.g. 359.6 into 360
    return round(value) % 360


def message_payload(second: aggregate.Second) -> str | None:
    """Return the JSON message for one second, or None without a position."""
    position = second.position
    if position is None:
        # The online dashboard needs a position for every record
        logger.warning(
            "No data for timestamp %d. Wind messages but 0 Position messages",
            second.timestamp_s,
        )
        return None
    return json.dumps(
        {
            "timestamp": second.timestamp_s,
            "gps": {
                "lat": round(position.latitude, 6),
                "lon": round(position.longitude, 6),
                "heading": _degrees(position.true_course),
                "speed": round(position.speed, 1),
            },
            "true": {
                "direction": _degrees(n2k.utils.rad_to_deg(second.true_wind.angle)),
                "speed": round(second.true_wind.magnitude, 1),
            },
            "apparent": {
                "direction": _degrees(
                    n2k.utils.rad_to_deg(second.apparent_wind.angle),
                ),
                "speed": round(second.apparent_wind.magnitude, 1),
            },
        },
    )


class Publisher:
    """
    Queue messages and hand them to paho a few at a time while connected.

    Messages paho already holds when the connection drops are resent by paho
    after reconnecting. Everything newer waits here, and once the queue is full
    the oldest message is dropped to make room.
    """

    def __init__(
        self,
        client: mqtt.Client,
        topic: str,
        max_queued: int = MAX_QUEUED_MESSAGES,
    ) -> None:
        self._client = client
        self._topic = topic
        self._backlog: deque[tuple[int, str]] = deque(maxlen=max_queued)
        self._in_paho = 0
        # Never call into paho while holding this lock: paho calls on_publish
        # with its own lock held, so the reverse order could deadlock.
        self._lock = threading.Lock()

    def __call__(self, second: aggregate.Second) -> None:
        payload = message_payload(second)
        if payload is None:
            return
        with self._lock:
            connection_status["queued"] += 1
            if len(self._backlog) == self._backlog.maxlen:
                connection_status["dropped"] += 1
                # Log once per queue length while offline, not every second
                if connection_status["dropped"] % self._backlog.maxlen == 1:
                    logger.warning(
                        "MQTT queue is full with %d messages, dropping the oldest",
                        self._backlog.maxlen,
                    )
            self._backlog.append((second.timestamp_s, payload))
        self.flush()

    def flush(self) -> None:
        """Hand queued messages to paho while connected and the window has room."""
        if connection_status["state"] != "success":
            return
        with self._lock:
            batch = []
            while self._backlog and self._in_paho < WINDOW:
                batch.append(self._backlog.popleft())
                self._in_paho += 1
        for timestamp_s, payload in batch:
            self._send(timestamp_s, payload)

    def _send(self, timestamp_s: int, payload: str) -> None:
        message_info = self._client.publish(self._topic, payload, qos=QOS)
        if message_info.rc == mqtt.MQTT_ERR_NO_CONN:
            # Just disconnected. paho keeps QoS 1 messages and resends them.
            logger.debug(
                "MQTT message for timestamp %d waits for the reconnect",
                timestamp_s,
            )
        elif message_info.rc != mqtt.MQTT_ERR_SUCCESS:
            with self._lock:
                self._in_paho -= 1
                connection_status["dropped"] += 1
            logger.error(
                "Failed to publish MQTT message for timestamp %d "
                "to topic %s with error code %d",
                timestamp_s,
                self._topic,
                message_info.rc,
            )
        else:
            logger.debug(
                "Published MQTT message for timestamp %d to topic %s",
                timestamp_s,
                self._topic,
            )

    def on_publish(self, _client: mqtt.Client, _userdata: Any, _mid: int) -> None:  # noqa: ANN401
        # With QoS 1 this runs when the broker's PUBACK arrives.
        with self._lock:
            self._in_paho = max(0, self._in_paho - 1)
            connection_status["confirmed"] += 1
            connection_status["last_confirmed"] = time.time()
        self.flush()

    def on_connect(
        self,
        client: mqtt.Client | None,
        userdata: Any,  # noqa: ANN401
        connect_flags: dict,
        reason_code: mqtt_reasoncodes.ReasonCode,
        properties: mqtt_properties.Properties,
    ) -> None:
        connect_callback(client, userdata, connect_flags, reason_code, properties)
        self.flush()


def init(config: MQTTConfig) -> Callable[[aggregate.Second], None]:
    """Connect to the broker in the background and return the publisher."""
    logger.debug(
        "Attempting to connect to MQTT broker at %s:%d",
        config.BROKER,
        config.PORT,
    )
    client = mqtt.Client(
        client_id=config.CLIENT_ID,
        userdata=None,
        protocol=mqtt.MQTTv5,
    )
    if config.USE_TLS:
        client.tls_set(tls_version=PROTOCOL_TLS)
    client.username_pw_set(username=config.USERNAME, password=config.PASSWORD)
    publisher = Publisher(client, config.TOPIC)
    client.on_connect = publisher.on_connect  # pyright: ignore[reportAttributeAccessIssue]
    client.on_connect_fail = connect_fail_callback
    client.on_disconnect = disconnect_callback  # pyright: ignore[reportAttributeAccessIssue]
    client.on_publish = publisher.on_publish  # pyright: ignore[reportAttributeAccessIssue]
    client.reconnect_delay_set(
        min_delay=RECONNECT_MIN_DELAY_S,
        max_delay=RECONNECT_MAX_DELAY_S,
    )
    # connect_async + loop_start retries forever, including the first connection.
    # A blocking connect() raises when the modem is not online yet (e.g. right
    # after boot), which would stop all MQTT uploads.
    client.connect_async(config.BROKER, config.PORT, keepalive=KEEPALIVE_S)
    client.loop_start()

    return publisher


class LiveTimestamps:
    """
    Wrap a publisher so a replay goes out as if it were live.

    Every second is shifted by one offset from recorded time to now, so seconds keep
    their recorded spacing and never share a timestamp. Call :meth:`sync` with the
    current replay time regularly. The offset only grows, so pausing leaves a gap
    instead of sending timestamps from the past. Replaying faster than real time
    would publish timestamps in the future, so the replay only allows 1x.

    Seconds recorded before the first :meth:`sync` are dropped, e.g. the ones a
    ``--jump`` fast-forwards through.
    """

    def __init__(self, publish: Callable[[aggregate.Second], None]) -> None:
        self._publish = publish
        self._start_s: int | None = None
        self._offset_s: float | None = None

    def sync(self, replay_time_s: float) -> None:
        """Align the offset so ``replay_time_s`` maps to the current time."""
        offset_s = time.time() - replay_time_s
        if self._offset_s is None:
            self._start_s = int(replay_time_s)
            self._offset_s = offset_s
        else:
            self._offset_s = max(self._offset_s, offset_s)

    def __call__(self, second: aggregate.Second) -> None:
        # read once, the replay thread updates it concurrently
        offset_s = self._offset_s
        if offset_s is None or self._start_s is None:
            return
        if second.timestamp_s < self._start_s:
            return
        shifted_s = int(second.timestamp_s + offset_s)
        self._publish(replace(second, timestamp_s=shifted_s))
