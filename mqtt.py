import configparser
from enum import Enum
from multiprocessing import Queue
from typing import List, Iterable, Union

import paho.mqtt.client as mqtt
import struct
from time import time

from typings import MQTTMessage


class GroupMethod(Enum):
    avg = "avg"
    combine = "combine"


config = configparser.ConfigParser()
config.read("config.ini")

BROKER = "broker.hivemq.com"
PORT = 1883
CLIENT_ID = "wind pi"
TOPIC = "luv"
USER = None
PASS = None
QOS = 0  # 0: fire and forget, 1: assert it has been received at least once, 2: assert it has been received exactly once
FREQUENCY = 1  # send messages every n seconds
GROUP_METHOD: GroupMethod = GroupMethod("avg")  # how to combine the individual measurements for sending

if "MQTT" in config:
    mqtt_config = config["MQTT"]
    BROKER = mqtt_config.get("BROKER", BROKER)
    PORT = int(mqtt_config.get("PORT", str(PORT)))
    CLIENT_ID = mqtt_config.get("CLIENT_ID", CLIENT_ID)
    TOPIC = mqtt_config.get("TOPIC", TOPIC)
    USER = mqtt_config.get("USER", USER)
    PASS = mqtt_config.get("PASS", PASS)
    QOS = int(mqtt_config.get("QOS", str(QOS)))
    FREQUENCY = int(mqtt_config.get("FREQUENCY", str(FREQUENCY)))
    gm = mqtt_config.get("GROUP_METHOD", None)
    try:
        GROUP_METHOD = GroupMethod(gm)
    except ValueError:
        if gm is not None:
            # TODO: log warning of incorrect value being set and defaulting to avg
            pass


def number_to_base(number, base):
    if number == 0:
        return [0]
    digits = []
    while number > 0:
        digits.append(int(number % base))
        number //= base
    return digits[::-1]


def base_to_number(digits, base):
    number = 0
    for i, d in enumerate(digits[::-1]):
        number += d * base ** i
    return number


def strip_leading_zeros(bytes):
    # assuming 8 byte for timestamp + 4 byte for data, remove leading zeros from timestamp
    i = 0
    for i in range(len(bytes)):
        if bytes[i] != 0:
            break
    return bytes[i:]


def add_leading_zeros(bytes):
    zeros = 12 - len(bytes)
    return zeros * b"\x00" + bytes


def encode(bytes):
    unpacked = strip_leading_zeros(struct.unpack("!" + "B" * len(bytes), bytes))
    digits = [d + 1 for d in number_to_base(base_to_number(unpacked, 256), 255)]
    return struct.pack("!" + "B" * len(digits), *digits)


def decode(bytes):
    unpacked = struct.unpack("!" + "B" * len(bytes), bytes)
    digits = [d - 1 for d in number_to_base(base_to_number(unpacked, 255), 256)]
    return add_leading_zeros(struct.pack("!" + "B" * len(digits), *digits))


def avg(iterable: Iterable[Union[float, int]]) -> float:
    length = 0
    counter = 0

    for x in iterable:
        counter += x
        length += 1

    if length == 0:
        return 0
    return counter / length


def combine_msgs(msgs: List[MQTTMessage], method: GroupMethod) -> MQTTMessage:
    assert len(msgs) > 0

    if len(msgs) == 0:
        return msgs[0]

    if method == GroupMethod.combine:
        raise NotImplementedError
    elif method == GroupMethod.avg:
        return MQTTMessage(
            time_ms=int(avg(map(lambda x: x.time_ms, msgs))),
            apparent_wind_speed=avg(map(lambda x: x.apparent_wind_speed, msgs)),
            apparent_wind_direction=avg(map(lambda x: x.apparent_wind_direction, msgs)),
            true_wind_speed=avg(map(lambda x: x.true_wind_speed, msgs)),
            true_wind_direction=avg(map(lambda x: x.true_wind_direction, msgs)),
        )
    else:
        raise ValueError("invalid method")


def worker(recv: Queue) -> None:
    # TODO: retry logic for connection? might be mqtt does it already
    client = mqtt.Client(client_id=CLIENT_ID, userdata=None, protocol=mqtt.MQTTv5)
    client.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
    client.username_pw_set(username=USER, password=PASS)
    client.connect(BROKER, PORT)
    client.loop_start()

    last = 0
    buf = []
    while True:
        msg: MQTTMessage = recv.get()
        if last > msg.time_ms - FREQUENCY:
            buf.append(msg)
            continue

        buf.append(msg)
        msg = combine_msgs(buf, GROUP_METHOD)
        buf.clear()

        # True Wind
        client.publish(TOPIC + "/t",
                       encode(struct.pack(
                           "!QHH",
                           msg.time_ms // 1000,
                           int(msg.true_wind_speed / 0.01),
                           int(msg.true_wind_direction / 0.0001))),
                       qos=QOS)
        # Apparent Wind
        client.publish(TOPIC + "/a",
                       encode(struct.pack(
                           "!QHH",
                           msg.time_ms // 1000,
                           int(msg.apparent_wind_speed / 0.01),
                           int(msg.apparent_wind_direction / 0.0001))),
                       qos=QOS)

        last = msg.time_ms
