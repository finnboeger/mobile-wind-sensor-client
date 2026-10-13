"""
Heading from a local magnetometer and IMU.

Reads the sensors, runs :py:class:`compass.processor.CompassProcessor` and passes the
heading on as :py:class:`structs.HeadingData`, the same type the NMEA2000 compass
produced, so the wind calculation doesn't need to know where the heading comes from.
"""

import json
import logging
import math
import threading
import time
from pathlib import Path
from queue import Queue

from smbus2 import SMBus

import log
from compass.calibration import (
    BenchCalibration,
    bench_calibration_file,
    write_json_atomically,
)
from compass.drivers import IMUS, MAGNETOMETERS, parse_axes
from compass.processor import CompassOutput, CompassProcessor, CompassSettings
from config import CompassConfig, Config
from structs import CompassSample, HeadingData, PositionData

logger = logging.getLogger(__name__)

#: how often the continuous calibration is written to disk, in seconds
SAVE_INTERVAL = 60.0
#: wait this long before trying again after the sensors failed, in seconds
RETRY_DELAY = 5.0


def create_processor(compass_config: CompassConfig) -> CompassProcessor:
    """Create a processor and restore the stored calibration."""
    processor = CompassProcessor(
        settings=CompassSettings(
            declination=math.radians(compass_config.DECLINATION_DEGREES),
            continuous_calibration=compass_config.CONTINUOUS_CALIBRATION,
            gnss_alignment=compass_config.GNSS_ALIGNMENT,
        ),
        magnetometers={
            name: BenchCalibration.load(
                bench_calibration_file(compass_config.BENCH_CALIBRATION_FILE, name),
            )
            for name in compass_config.MAGNETOMETERS
        },
    )
    path = compass_config.CALIBRATION_FILE
    if Path(path).exists():
        try:
            with Path(path).open(encoding="utf-8") as file:
                processor.load_state_dict(json.load(file))
            logger.info("Loaded compass calibration from %s", path)
        except (OSError, ValueError, KeyError, TypeError):
            logger.exception("Could not load compass calibration from %s", path)
    return processor


def save_processor(processor: CompassProcessor, path: str) -> None:
    try:
        write_json_atomically(path, processor.state_dict())
    except OSError:
        logger.exception("Could not save compass calibration to %s", path)


class SensorReader:
    """Reads the magnetometer and IMU and maps them to the body frame."""

    def __init__(self, compass_config: CompassConfig) -> None:
        for name in compass_config.MAGNETOMETERS:
            if name not in MAGNETOMETERS:
                msg = f"Unknown magnetometer {name}"
                raise ValueError(msg)
        if compass_config.IMU not in IMUS:
            msg = f"Unknown IMU {compass_config.IMU}"
            raise ValueError(msg)
        self.bus = SMBus(compass_config.I2C_BUS)
        self.magnetometers = []
        for name, address in zip(
            compass_config.MAGNETOMETERS,
            compass_config.MAGNETOMETER_ADDRESSES,
            strict=True,
        ):
            magnetometer_type = MAGNETOMETERS[name]
            self.magnetometers.append(
                magnetometer_type(
                    self.bus,
                    address or magnetometer_type.DEFAULT_ADDRESS,
                ),
            )
        imu_type = IMUS[compass_config.IMU]
        self.imu = imu_type(
            self.bus,
            compass_config.IMU_ADDRESS or imu_type.DEFAULT_ADDRESS,
        )
        self.magnetometer_axes = [
            parse_axes(axes) for axes in compass_config.MAGNETOMETER_AXES
        ]
        self.imu_axes = parse_axes(compass_config.IMU_AXES)

    def read(self) -> CompassSample:
        acceleration, angular_rate = self.imu.read()
        return CompassSample(
            timestamp=int(time.time() * 1000),
            acceleration=self.imu_axes(acceleration),
            angular_rate=self.imu_axes(angular_rate),
            # read one after another: the MMC5983MA's SET/RESET pulse draws a large
            # current, so no other magnetometer may be measuring at the same time
            magnetic_fields=tuple(
                axes(magnetometer.read())
                for magnetometer, axes in zip(
                    self.magnetometers,
                    self.magnetometer_axes,
                    strict=True,
                )
            ),
        )

    def close(self) -> None:
        self.bus.close()


class _Output:
    """Passes the heading on at the output rate."""

    def __init__(self, queues: list[Queue[HeadingData]], rate_hz: float) -> None:
        self.queues = queues
        self.interval_ms = 1000 / rate_hz
        self.last_time = 0

    def put(self, output: CompassOutput | None) -> None:
        if output is None or output.timestamp - self.last_time < self.interval_ms:
            return
        self.last_time = output.timestamp
        heading = HeadingData(heading=output.heading, timestamp=output.timestamp)
        log.data("heading", heading)
        for queue in self.queues:
            queue.put(heading)


def worker(
    position_queue: Queue[PositionData],
    output_queues: list[Queue[HeadingData]],
) -> None:
    compass_config = Config().COMPASS
    processor = create_processor(compass_config)
    output = _Output(output_queues, compass_config.OUTPUT_RATE_HZ)
    sample_interval = 1 / compass_config.SAMPLE_RATE_HZ
    last_save = time.monotonic()

    while True:
        try:
            reader = SensorReader(compass_config)
        except (OSError, ValueError):
            logger.exception("Could not open the compass sensors")
            time.sleep(RETRY_DELAY)
            continue

        next_sample = time.monotonic()
        try:
            while True:
                while not position_queue.empty():
                    processor.add_position(position_queue.get())

                sample = reader.read()
                log.data("compass_raw", sample)
                output.put(processor.process(sample))

                if time.monotonic() - last_save > SAVE_INTERVAL:
                    last_save = time.monotonic()
                    save_processor(processor, compass_config.CALIBRATION_FILE)

                next_sample += sample_interval
                delay = next_sample - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    # running behind, don't try to catch up
                    next_sample = time.monotonic()
        except OSError:
            # includes TimeoutError
            logger.exception("Reading the compass sensors failed")
            reader.close()
            time.sleep(RETRY_DELAY)


def init(
    position_queue: Queue[PositionData],
    output_queues: list[Queue[HeadingData]],
) -> None:
    """Start reading the compass sensors in a background thread."""
    threading.Thread(
        target=worker,
        args=(position_queue, output_queues),
        daemon=True,
    ).start()
