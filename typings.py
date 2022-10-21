from typing import NamedTuple


class MQTTMessage(NamedTuple):
    time_ms: int
    apparent_wind_speed: float  # in meters per second
    apparent_wind_direction: float  # in radians
    true_wind_speed: float  # in meters per second
    true_wind_direction: float  # in radians

