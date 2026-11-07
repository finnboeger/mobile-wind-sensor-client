"""
Compass heading from the magnetometers on the measurement unit.

This replaces the heading of an external NMEA 2000 compass. Roll and pitch come from
the IMU (:py:class:`attitude.AttitudeEstimator`), the magnetic field is levelled with
them and its horizontal direction is the magnetic heading of the unit's forward
direction. The gyro carries the heading between magnetometer readings
(:py:class:`heading.HeadingFilter`), so a wave that tilts the unit for a moment
doesn't swing the heading.

Near Hamburg the field points about 69° down, so the horizontal part is only 0.36 of
the total field. A roll or pitch error therefore shows up about 2.6 times larger in
the heading (tan 69°), which is why the tilt comes from the gyro supported attitude
and not from the accelerometer alone.

:py:class:`CompassProcessor` doesn't access any hardware, so the same code runs live
and in the tests.
"""

import logging
import math
import threading
from dataclasses import dataclass
from queue import Empty, Queue
from typing import TypeVar

import config
import log
from attitude import AttitudeEstimator
from heading import HeadingFilter
from motion import to_north_east
from structs import HeadingData, ImuSample, MagnetometerSample, Vector3

logger = logging.getLogger(__name__)

_T = TypeVar("_T")

#: standard deviation of a single magnetometer heading in rad
DEFAULT_MAGNETIC_SIGMA = math.radians(3.0)
#: how long the worker waits for magnetometer samples before warning, in seconds
_SAMPLE_TIMEOUT = 5.0


def heading_from_field(field: Vector3, roll: float, pitch: float) -> float:
    """
    Return the magnetic heading of the unit's forward direction.

    :param field: magnetic field in the unit's axes (forward, right, down)
    :param roll: roll in rad, positive with the right side down
    :param pitch: pitch in rad, positive with the front up
    :return: heading in rad in [0, 2π), clockwise from magnetic north
    """
    level = to_north_east(field, roll, pitch, 0.0)
    # level.x is the field along the levelled forward direction, level.y to the
    # right: facing east, north is to the left and level.y negative
    return math.atan2(-level.y, level.x) % math.tau


@dataclass(frozen=True, kw_only=True)
class CompassSettings:
    #: magnetic declination in rad, positive when magnetic north is east of true north
    declination: float = 0.0
    #: standard deviation of a single magnetometer heading in rad
    magnetic_sigma: float = DEFAULT_MAGNETIC_SIGMA


class MagnetometerChannel:
    """Correction of one magnetometer's readings."""

    def __init__(self, name: str) -> None:
        self.name = name

    def correct(self, field: Vector3) -> Vector3:
        """
        Return the field with the iron around the sensor removed.

        Not calibrated yet, the raw field is used as it is.
        """
        return field


class CompassProcessor:
    """Turns IMU and magnetometer samples into a true heading."""

    def __init__(
        self,
        names: list[str],
        settings: CompassSettings | None = None,
    ) -> None:
        """
        Create a processor.

        :param names: names of the magnetometers, in the order of
            :py:attr:`structs.MagnetometerSample.magnetic_fields`
        :param settings: settings, or None for the defaults
        """
        self.settings = settings or CompassSettings()
        self.channels = [MagnetometerChannel(name) for name in names]
        self.attitude = AttitudeEstimator()
        self.heading = HeadingFilter(
            compass_sigma=self.settings.magnetic_sigma,
            compass_lag=0.0,
        )

    def add_imu(self, sample: ImuSample) -> None:
        self.attitude.update(sample)
        if self.attitude.initialized:
            self.heading.predict(sample.timestamp, self.attitude.heading_rate(sample))

    def magnetic_heading(self, sample: MagnetometerSample) -> float | None:
        """Return the magnetic heading measured by the sample, without the gyro."""
        if not sample.magnetic_fields:
            return None
        # only the first magnetometer for now
        field = self.channels[0].correct(sample.magnetic_fields[0])
        return heading_from_field(field, self.attitude.roll, self.attitude.pitch)

    def add_magnetometer(self, sample: MagnetometerSample) -> HeadingData | None:
        """
        Add a magnetometer sample.

        :return: the true heading at the time of the sample, None while the IMU
            hasn't delivered a sample yet
        """
        if not self.attitude.initialized:
            return None
        magnetic_heading = self.magnetic_heading(sample)
        if magnetic_heading is not None:
            self.heading.update(sample.timestamp, magnetic_heading)
        heading = self.heading.heading_at(sample.timestamp)
        if heading is None:
            return None
        return HeadingData(
            heading=(heading + self.settings.declination) % math.tau,
            timestamp=sample.timestamp,
        )

    def add(
        self,
        imu_samples: list[ImuSample],
        magnetometer_samples: list[MagnetometerSample],
    ) -> list[HeadingData]:
        """
        Add IMU and magnetometer samples in the order they were measured.

        :return: the heading at the time of each magnetometer sample
        """
        events: list[ImuSample | MagnetometerSample] = [
            *imu_samples,
            *magnetometer_samples,
        ]
        headings: list[HeadingData] = []
        for event in sorted(events, key=lambda event: event.timestamp):
            if isinstance(event, ImuSample):
                self.add_imu(event)
                continue
            heading = self.add_magnetometer(event)
            if heading is not None:
                headings.append(heading)
        return headings


def _drain(queue: Queue[list[_T]]) -> list[_T]:
    items: list[_T] = []
    while True:
        try:
            items.extend(queue.get_nowait())
        except Empty:
            return items


def worker(
    processor: CompassProcessor,
    imu_queue: Queue[list[ImuSample]],
    magnetometer_queue: Queue[list[MagnetometerSample]],
    output_queues: list[Queue[HeadingData]],
    stop: threading.Event | None = None,
) -> None:
    """
    Turn the IMU and magnetometer batches into headings.

    :param output_queues: queues receiving the headings, like the compass headings
        received over NMEA 2000
    :param stop: ends the loop when set, used by the tests
    """
    while stop is None or not stop.is_set():
        try:
            magnetometer_samples = magnetometer_queue.get(timeout=_SAMPLE_TIMEOUT)
        except Empty:
            logger.warning("No magnetometer samples for %.0f s", _SAMPLE_TIMEOUT)
            continue
        magnetometer_samples += _drain(magnetometer_queue)
        # the IMU batches arrive independently, so the latest IMU samples may lag the
        # magnetometer by up to a batch; the heading filter extrapolates over that
        for heading in processor.add(_drain(imu_queue), magnetometer_samples):
            log.data("heading", heading)
            for queue in output_queues:
                queue.put(heading)


def create_processor() -> CompassProcessor:
    magnetometer_config = config.Config().MAGNETOMETER
    return CompassProcessor(
        magnetometer_config.DEVICES,
        CompassSettings(
            declination=math.radians(magnetometer_config.DECLINATION_DEGREES),
        ),
    )


def init(
    imu_queue: Queue[list[ImuSample]],
    magnetometer_queue: Queue[list[MagnetometerSample]],
    output_queues: list[Queue[HeadingData]],
) -> None:
    """Start turning the IMU and magnetometer samples into headings."""
    threading.Thread(
        target=worker,
        args=(create_processor(), imu_queue, magnetometer_queue, output_queues),
        daemon=True,
    ).start()
