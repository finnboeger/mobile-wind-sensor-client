import json
import logging
from collections.abc import Callable
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

# Read by the local dashboard. "state" is None until the first attempt finishes,
# then "success" while connected and "fail" while not.
connection_status = {
    "state": None,
    "error_count": 0,
}


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


def publish_message(
    client: mqtt.Client,
    topic: str,
    second: aggregate.Second,
) -> None:
    position = second.position
    if position is None:
        # The online dashboard needs a position for every record
        logger.warning(
            "No data for timestamp %d. Wind messages but 0 Position messages",
            second.timestamp_s,
        )
        return
    payload = {
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
            "direction": _degrees(n2k.utils.rad_to_deg(second.apparent_wind.angle)),
            "speed": round(second.apparent_wind.magnitude, 1),
        },
    }

    message_info = client.publish(topic, json.dumps(payload), qos=0)
    if message_info.rc == mqtt.MQTT_ERR_NO_CONN:
        # Expected while offline, the disconnect itself is logged once.
        logger.debug(
            "Dropped MQTT message for timestamp %d, not connected",
            second.timestamp_s,
        )
    elif message_info.rc != mqtt.MQTT_ERR_SUCCESS:
        logger.error(
            "Failed to publish MQTT message for timestamp %d "
            "to topic %s with error code %d",
            second.timestamp_s,
            topic,
            message_info.rc,
        )
    else:
        logger.debug(
            "Published MQTT message for timestamp %d to topic %s",
            second.timestamp_s,
            topic,
        )


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
    client.on_connect = connect_callback  # pyright: ignore[reportAttributeAccessIssue]
    client.on_connect_fail = connect_fail_callback
    client.on_disconnect = disconnect_callback  # pyright: ignore[reportAttributeAccessIssue]
    client.reconnect_delay_set(
        min_delay=RECONNECT_MIN_DELAY_S,
        max_delay=RECONNECT_MAX_DELAY_S,
    )
    # connect_async + loop_start retries forever, including the first connection.
    # A blocking connect() raises when the modem is not online yet (e.g. right
    # after boot), which would stop all MQTT uploads.
    client.connect_async(config.BROKER, config.PORT, keepalive=KEEPALIVE_S)
    client.loop_start()

    topic = config.TOPIC
    return lambda second: publish_message(client, topic, second)
