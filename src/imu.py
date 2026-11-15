"""
Read the accelerometer and gyro of the IMU.

The samples are converted from the sensor's own axes into the axes of the
measurement unit: x points to the unit's forward direction (the direction the
compass heading and the wind angle refer to), y to its right and z down. The unit
can stand on a mast in any orientation relative to the boat, so nothing here
assumes where the bow is.
"""

import logging
import threading
import time
from collections.abc import Callable
from queue import Queue
from typing import Protocol

import config
import log
from structs import ImuSample, Vector3

logger = logging.getLogger(__name__)

#: standard gravity in m/s²
GRAVITY = 9.80665
#: number of batches per second passed on to the consumers
BATCHES_PER_SECOND = 10
#: number of axes of a vector
_AXES = 3


class ImuDevice(Protocol):
    def read(self) -> tuple[Vector3, Vector3]:
        """
        Read one sample.

        :return: specific force in m/s² and angular rate in rad/s, in the sensor's
            own axes. A sensor lying still reads +1 g on its upward axis.
        """
        ...


class MockImu:
    """Stand-in for a real sensor: lies still and level with its z axis up."""

    def read(self) -> tuple[Vector3, Vector3]:
        return (0.0, 0.0, GRAVITY), (0.0, 0.0, 0.0)


#: available devices by their name in the config
DEVICES: dict[str, Callable[[config.ImuConfig], ImuDevice]] = {
    "mock": lambda _: MockImu(),
}


def parse_axes(specification: str) -> Callable[[Vector3], Vector3]:
    """
    Parse an axis mapping like ``"x,-y,-z"``.

    The three entries name the sensor axes that point forward, right and down in
    the unit, in that order. ``"x,-y,-z"`` is right for a breakout lying flat,
    components up, with its x arrow pointing forward.

    :param specification: comma separated sensor axes, each optionally negated
    :return: function converting a vector from sensor axes to unit axes
    """
    entries = [entry.strip().lower() for entry in specification.split(",")]
    if len(entries) != _AXES:
        msg = f"Axis mapping {specification!r} needs three entries"
        raise ValueError(msg)

    mapping: list[tuple[int, float]] = []
    for entry in entries:
        sign = -1.0 if entry.startswith("-") else 1.0
        name = entry.lstrip("+-")
        if name not in ("x", "y", "z"):
            msg = f"Unknown axis {entry!r} in {specification!r}"
            raise ValueError(msg)
        mapping.append(("xyz".index(name), sign))

    if len({index for index, _ in mapping}) != _AXES:
        msg = f"Axis mapping {specification!r} must use each axis once"
        raise ValueError(msg)

    def apply(vector: Vector3) -> Vector3:
        (i, si), (j, sj), (k, sk) = mapping
        return (si * vector[i], sj * vector[j], sk * vector[k])

    return apply


class ImuReader:
    """Reads samples from a device and converts them into the unit's axes."""

    def __init__(self, device: ImuDevice, axes: str) -> None:
        self.device = device
        self.to_unit_axes = parse_axes(axes)

    def read_sample(self) -> ImuSample:
        acceleration, angular_rate = self.device.read()
        return ImuSample(
            timestamp=int(time.time() * 1000),
            acceleration=self.to_unit_axes(acceleration),
            angular_rate=self.to_unit_axes(angular_rate),
        )


def worker(
    reader: ImuReader,
    sample_rate_hz: float,
    output_queues: list[Queue[list[ImuSample]]],
    stop: threading.Event | None = None,
) -> None:
    """
    Read samples at a fixed rate and pass them on in batches.

    :param reader: reader for the IMU
    :param sample_rate_hz: sample rate in Hz
    :param output_queues: queues receiving lists of samples
    :param stop: ends the loop when set, used by the tests
    """
    period = 1 / sample_rate_hz
    batch_size = max(1, round(sample_rate_hz / BATCHES_PER_SECOND))
    batch: list[ImuSample] = []
    next_time = time.monotonic()
    failed_reads = 0

    while stop is None or not stop.is_set():
        next_time += period
        delay = next_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        elif delay < -1:
            # more than a second behind, e.g. after a stall: don't try to catch up
            next_time = time.monotonic()

        try:
            batch.append(reader.read_sample())
        except OSError:
            failed_reads += 1
            if failed_reads == 1 or failed_reads % 1000 == 0:
                logger.warning("Reading the IMU failed %d times", failed_reads)
            continue

        if len(batch) >= batch_size:
            for queue in output_queues:
                queue.put(batch)
            log.data("imu", batch)
            batch = []


def init(output_queues: list[Queue[list[ImuSample]]]) -> None:
    """
    Start reading the IMU if it is enabled in the config.

    :param output_queues: queues receiving lists of samples
    """
    imu_config = config.Config().IMU
    if not imu_config.ENABLED:
        return

    if imu_config.DEVICE not in DEVICES:
        logger.error(
            "Unknown IMU device %s, known devices: %s",
            imu_config.DEVICE,
            ", ".join(DEVICES),
        )
        return

    device = DEVICES[imu_config.DEVICE](imu_config)
    reader = ImuReader(device, imu_config.AXES)
    logger.info(
        "Reading the %s IMU at %.0f Hz",
        imu_config.DEVICE,
        imu_config.SAMPLE_RATE_HZ,
    )
    threading.Thread(
        target=worker,
        args=(reader, imu_config.SAMPLE_RATE_HZ, output_queues),
        daemon=True,
    ).start()
