"""
Read the magnetometers of the measurement unit.

Like the IMU samples, the readings are converted into the unit's axes: x points to
the unit's forward direction, y to its right and z down. Several magnetometers can
be read at once, each with its own axis mapping. They are read one after another,
never overlapping, because a magnetometer's own pulses (e.g. the SET/RESET pulse of
an AMR sensor) would disturb a neighbour that is measuring at the same time.
"""

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from queue import Queue
from typing import Protocol

import config
import log
from imu import parse_axes
from structs import MagnetometerSample, Vector3

logger = logging.getLogger(__name__)

#: number of batches per second passed on to the consumers
BATCHES_PER_SECOND = 10

#: earth's field near Hamburg in µT: 18 µT towards north, 47 µT down
MOCK_FIELD: Vector3 = (18.0, 0.0, 47.0)


class MagnetometerDevice(Protocol):
    def read(self) -> Vector3:
        """Read the magnetic field in µT in the sensor's own axes."""
        ...


class MockMagnetometer:
    """Stand-in for a real sensor: lies level with its z axis up, facing north."""

    def read(self) -> Vector3:
        # the field points down, which is the sensor's -z
        return (MOCK_FIELD[0], -MOCK_FIELD[1], -MOCK_FIELD[2])


#: available devices by their name in the config, created from the I2C bus number
#: and the address (None for the chip's default)
DEVICES: dict[str, Callable[[int, int | None], MagnetometerDevice]] = {
    "mock": lambda _bus, _address: MockMagnetometer(),
}


@dataclass(frozen=True, kw_only=True)
class ConfiguredMagnetometer:
    name: str
    device: MagnetometerDevice
    #: sensor axes pointing forward, right and down in the unit, e.g. "x,-y,-z"
    axes: str


class MagnetometerReader:
    """Reads all magnetometers and converts the fields into the unit's axes."""

    def __init__(self, magnetometers: list[ConfiguredMagnetometer]) -> None:
        self.magnetometers = magnetometers
        self._to_unit_axes = [parse_axes(m.axes) for m in magnetometers]

    def read_sample(self) -> MagnetometerSample:
        timestamp = int(time.time() * 1000)
        fields = tuple(
            to_unit_axes(magnetometer.device.read())
            for magnetometer, to_unit_axes in zip(
                self.magnetometers,
                self._to_unit_axes,
                strict=True,
            )
        )
        return MagnetometerSample(timestamp=timestamp, magnetic_fields=fields)


def worker(
    reader: MagnetometerReader,
    sample_rate_hz: float,
    output_queues: list[Queue[list[MagnetometerSample]]],
    stop: threading.Event | None = None,
) -> None:
    """
    Read samples at a fixed rate and pass them on in batches.

    :param reader: reader for the magnetometers
    :param sample_rate_hz: sample rate in Hz
    :param output_queues: queues receiving lists of samples
    :param stop: ends the loop when set, used by the tests
    """
    period = 1 / sample_rate_hz
    batch_size = max(1, round(sample_rate_hz / BATCHES_PER_SECOND))
    batch: list[MagnetometerSample] = []
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
                logger.warning(
                    "Reading the magnetometers failed %d times",
                    failed_reads,
                )
            continue

        if len(batch) >= batch_size:
            for queue in output_queues:
                queue.put(batch)
            log.data("magnetometer", batch)
            batch = []


def create_reader(
    magnetometer_config: config.MagnetometerConfig,
) -> MagnetometerReader | None:
    """Create the reader for the configured magnetometers, None if one is unknown."""
    unknown = [name for name in magnetometer_config.DEVICES if name not in DEVICES]
    if unknown or not magnetometer_config.DEVICES:
        logger.error(
            "Unknown magnetometer %s, known devices: %s",
            ", ".join(unknown) or "(none configured)",
            ", ".join(DEVICES),
        )
        return None
    return MagnetometerReader(
        [
            ConfiguredMagnetometer(
                name=name,
                device=DEVICES[name](magnetometer_config.I2C_BUS, address),
                axes=axes,
            )
            for name, address, axes in zip(
                magnetometer_config.DEVICES,
                magnetometer_config.ADDRESSES,
                magnetometer_config.AXES,
                strict=True,
            )
        ],
    )


def init(output_queues: list[Queue[list[MagnetometerSample]]]) -> bool:
    """
    Start reading the magnetometers if they are enabled in the config.

    :param output_queues: queues receiving lists of samples
    :return: whether the magnetometers are read
    """
    magnetometer_config = config.Config().MAGNETOMETER
    if not magnetometer_config.ENABLED:
        return False

    reader = create_reader(magnetometer_config)
    if reader is None:
        return False
    logger.info(
        "Reading the %s magnetometer at %.0f Hz",
        ", ".join(magnetometer_config.DEVICES),
        magnetometer_config.SAMPLE_RATE_HZ,
    )
    threading.Thread(
        target=worker,
        args=(reader, magnetometer_config.SAMPLE_RATE_HZ, output_queues),
        daemon=True,
    ).start()
    return True
