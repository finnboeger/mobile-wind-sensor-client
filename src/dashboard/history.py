"""
One second measurements for the local dashboard.

The records are the one second averages of :mod:`aggregate`, which the MQTT upload
sends too, with the same fields as the ``measurements`` collection of the online
dashboard (mobile-wind-sensor-server), so both can be drawn the same way.
"""

import bisect
import json
import math
import threading
from collections import deque
from typing import Any

from aggregate import Second

#: number of one second records kept. The longest time frame on the page is 90 min,
#: the extra 30 min are just headroom (a record is a few hundred bytes)
DEFAULT_MAX_RECORDS = 2 * 60 * 60


def _direction(angle: float) -> float:
    """Return ``angle`` (radians) in degrees from 0 up to, but not including, 360."""
    # rounding after the modulo would turn e.g. 359.97 into 360.0
    return round(math.degrees(angle), 1) % 360


def _finite(value: object) -> object:
    """Return None for NaN and infinity, which JSON can't represent."""
    return None if isinstance(value, float) and not math.isfinite(value) else value


class MeasurementHistory:
    """
    Thread safe ring buffer of one second averages.

    Each record is also kept encoded as JSON. Encoding the whole history takes a few
    hundred milliseconds on a Pi Zero 2 W, during which the GIL stops the wind
    processing; joining the encoded records takes a few milliseconds.
    """

    def __init__(self, max_records: int = DEFAULT_MAX_RECORDS) -> None:
        self._lock = threading.Lock()
        self._records: deque[dict[str, Any]] = deque(maxlen=max_records)
        #: ``_records`` encoded as JSON, in the same order
        self._encoded: deque[str] = deque(maxlen=max_records)

    def add_second(self, second: Second) -> None:
        """Store a finished second, called by the aggregate thread."""
        record: dict[str, Any] = {
            "ts": second.timestamp_s * 1000,
            "true_wind_speed_mps": round(second.true_wind.magnitude, 2),
            "true_wind_dir_deg": _direction(second.true_wind.angle),
            "apparent_wind_speed_mps": round(second.apparent_wind.magnitude, 2),
            "apparent_wind_dir_deg": _direction(second.apparent_wind.angle),
        }
        position = second.position
        if position is not None:
            record["gps_lat"] = round(position.latitude, 6)
            record["gps_lng"] = round(position.longitude, 6)
            record["sensor_speed_mps"] = round(position.speed, 2)
            record["sensor_heading_deg"] = round(position.true_course, 1) % 360
        record = {key: _finite(value) for key, value in record.items()}
        encoded = json.dumps(record, separators=(",", ":"), allow_nan=False)
        with self._lock:
            self._records.append(record)
            self._encoded.append(encoded)

    def _start(self, timestamp: int | None) -> int:
        """Return the index of the first record newer than ``timestamp``."""
        if timestamp is None:
            return 0
        # the records are sorted by time
        return bisect.bisect_right(self._records, timestamp, key=lambda r: r["ts"])

    def since(self, timestamp: int | None = None) -> list[dict[str, Any]]:
        """Return the records newer than ``timestamp`` (ms), oldest first."""
        with self._lock:
            return list(self._records)[self._start(timestamp) :]

    def since_json(self, timestamp: int | None = None, limit: int | None = None) -> str:
        """
        Return the records newer than ``timestamp`` (ms) as a JSON array, oldest first.

        With ``limit`` only the newest ``limit`` of them.
        """
        with self._lock:
            encoded = list(self._encoded)[self._start(timestamp) :]
        if limit is not None:
            encoded = encoded[-limit:] if limit > 0 else []
        return "[" + ",".join(encoded) + "]"
