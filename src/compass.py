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

import json
import logging
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
from typing import Any, TypeVar

import config
import log
import magnetometer
from attitude import AttitudeEstimator
from compass_calibration import (
    BenchCalibration,
    ContinuousCalibration,
    HorizontalCalibration,
    bench_calibration_file,
    write_json_atomically,
)
from heading import HeadingFilter
from motion import to_north_east
from structs import HeadingData, ImuSample, MagnetometerSample, PositionData, Vector3

logger = logging.getLogger(__name__)

_T = TypeVar("_T")

#: standard deviation of a single magnetometer heading in rad
DEFAULT_MAGNETIC_SIGMA = math.radians(3.0)
#: how long the worker waits for magnetometer samples before warning, in seconds
_SAMPLE_TIMEOUT = 5.0
#: GNSS speed and course are used for this long after a fix, in milliseconds
_MAX_POSITION_AGE_MS = 3000


def heading_from_field(
    field: Vector3,
    roll: float,
    pitch: float,
    horizontal: HorizontalCalibration | None = None,
) -> float:
    """
    Return the magnetic heading of the unit's forward direction.

    :param field: magnetic field in the unit's axes (forward, right, down)
    :param roll: roll in rad, positive with the right side down
    :param pitch: pitch in rad, positive with the front up
    :param horizontal: correction of the boat's iron, if known
    :return: heading in rad in [0, 2π), clockwise from magnetic north
    """
    if horizontal is not None:
        field = horizontal.remove_offset(field)
    level = to_north_east(field, roll, pitch, 0.0)
    x, y = level.x, level.y
    if horizontal is not None:
        x, y = horizontal.apply_soft_iron(x, y)
    # x is the field along the levelled forward direction, y to the right: facing
    # east, north is to the left and y negative
    return math.atan2(-y, x) % math.tau


@dataclass(frozen=True, kw_only=True)
class CompassSettings:
    #: magnetic declination in rad, positive when magnetic north is east of true north
    declination: float = 0.0
    #: standard deviation of a single magnetometer heading in rad
    magnetic_sigma: float = DEFAULT_MAGNETIC_SIGMA
    #: learn the boat's iron while underway
    continuous_calibration: bool = True
    #: calibration samples are only taken while tilted less than this, in rad
    max_calibration_tilt: float = math.radians(20)
    #: and while turning slower than this, in rad/s
    max_calibration_rate: float = math.radians(45)
    #: time between calibration fits in milliseconds
    refit_interval_ms: int = 30_000


class MagnetometerChannel:
    """Calibration of one magnetometer."""

    def __init__(self, name: str, bench: BenchCalibration | None = None) -> None:
        self.name = name
        self.bench = bench or BenchCalibration()
        self.calibration = ContinuousCalibration()

    def correct(self, field: Vector3) -> Vector3:
        """Return the field with the iron in the unit removed."""
        return self.bench.apply(field)

    def heading(self, field: Vector3, roll: float, pitch: float) -> float:
        """
        Return the magnetic heading with the boat's iron removed.

        :param field: field after :py:meth:`correct`
        """
        return heading_from_field(field, roll, pitch, self.calibration.calibration)

    def add_calibration_sample(self, field: Vector3, roll: float, pitch: float) -> None:
        """Add a field after :py:meth:`correct` to the continuous calibration."""
        level = to_north_east(field, roll, pitch, 0.0)
        self.calibration.add_sample(level.x, level.y)

    def state_dict(self) -> dict[str, Any]:
        return {"name": self.name, "calibration": self.calibration.state_dict()}

    def load_state_dict(self, state: dict[str, Any]) -> None:
        if "calibration" in state:
            self.calibration.load_state_dict(state["calibration"])


def world_to_unit(
    vector: Vector3,
    heading: float,
    roll: float,
    pitch: float,
) -> Vector3:
    """Rotate a vector from north, east, down into the unit's axes."""
    north, east, down = vector
    # heading about the vertical
    x = north * math.cos(heading) + east * math.sin(heading)
    y = -north * math.sin(heading) + east * math.cos(heading)
    # pitch about y, then roll about x
    x, z = (
        x * math.cos(pitch) - down * math.sin(pitch),
        x * math.sin(pitch) + down * math.cos(pitch),
    )
    y, z = (
        y * math.cos(roll) + z * math.sin(roll),
        -y * math.sin(roll) + z * math.cos(roll),
    )
    return (x, y, z)


class TurnAcceleration:
    """
    Acceleration of the boat from its GNSS speed and the gyro.

    In a turn the boat accelerates towards the inside of the turn by its speed times
    the rate of turn: 0.2 g at 8 m/s and 15°/s. The accelerometer can't tell that
    apart from a tilt, and its magnitude stays within a few percent of 1 g, so the
    attitude estimator would take it for a roll of 12° and the heading would be off
    by 2.6 times that. Removing it keeps the attitude right in turns.
    """

    def __init__(self) -> None:
        self._time: int | None = None
        self._speed = 0.0
        self._course = 0.0
        #: smoothed acceleration along the course in m/s²
        self._along = 0.0

    def add_position(self, position: PositionData) -> None:
        # HNR-PVT repeats or interpolates the GNSS solution, see wind.py
        if not position.valid or position.source not in (None, "NAV-PVT"):
            return
        if position.velocity_north is not None and position.velocity_east is not None:
            course = math.atan2(position.velocity_east, position.velocity_north)
        else:
            course = math.radians(position.true_course)
        if (
            self._time is not None
            and 0 < position.timestamp - self._time < _MAX_POSITION_AGE_MS
        ):
            dt = (position.timestamp - self._time) / 1000
            # smooth over about a second, GNSS speed differences are noisy
            self._along += min(1.0, dt) * (
                (position.speed - self._speed) / dt - self._along
            )
        else:
            self._along = 0.0
        self._time = position.timestamp
        self._speed = position.speed
        self._course = course

    def acceleration(self, timestamp: int, heading_rate: float) -> Vector3:
        """
        Return the boat's acceleration in north, east, down in m/s².

        :param heading_rate: rate of turn in rad/s from the gyro
        """
        if self._time is None or abs(timestamp - self._time) > _MAX_POSITION_AGE_MS:
            return (0.0, 0.0, 0.0)
        across = self._speed * heading_rate
        sin_course, cos_course = math.sin(self._course), math.cos(self._course)
        return (
            self._along * cos_course - across * sin_course,
            self._along * sin_course + across * cos_course,
            0.0,
        )


class CompassProcessor:
    """Turns IMU and magnetometer samples into a true heading."""

    def __init__(
        self,
        magnetometers: dict[str, BenchCalibration | None],
        settings: CompassSettings | None = None,
    ) -> None:
        """
        Create a processor.

        :param magnetometers: name and bench calibration of each magnetometer, in
            the order of :py:attr:`structs.MagnetometerSample.magnetic_fields`
        :param settings: settings, or None for the defaults
        """
        self.settings = settings or CompassSettings()
        self.channels = [
            MagnetometerChannel(name, bench) for name, bench in magnetometers.items()
        ]
        self.attitude = AttitudeEstimator()
        self.heading = HeadingFilter(
            compass_sigma=self.settings.magnetic_sigma,
            compass_lag=0.0,
        )
        self.turn = TurnAcceleration()
        #: set when a calibration changed, cleared once it has been saved
        self.calibration_changed = False
        #: rate of turn in rad/s of the latest IMU sample
        self._rate = 0.0
        self._last_fit_time: int | None = None

    def add_position(self, position: PositionData) -> None:
        """Add a GNSS fix."""
        self.turn.add_position(position)

    def _boat_acceleration(self, sample: ImuSample) -> Vector3:
        heading = self.heading.heading_at(sample.timestamp)
        if heading is None or not self.attitude.initialized:
            return (0.0, 0.0, 0.0)
        return world_to_unit(
            self.turn.acceleration(
                sample.timestamp,
                self.attitude.heading_rate(sample),
            ),
            heading,
            self.attitude.roll,
            self.attitude.pitch,
        )

    def add_imu(self, sample: ImuSample) -> None:
        self.attitude.update(sample, self._boat_acceleration(sample))
        if self.attitude.initialized:
            self._rate = self.attitude.heading_rate(sample)
            self.heading.predict(sample.timestamp, self._rate)

    @property
    def calibrated(self) -> bool:
        """Whether the continuous calibration has made a fit."""
        return any(channel.calibration.converged for channel in self.channels)

    def _calibration_sample_ok(self) -> bool:
        settings = self.settings
        return (
            settings.continuous_calibration
            and abs(self.attitude.roll) < settings.max_calibration_tilt
            and abs(self.attitude.pitch) < settings.max_calibration_tilt
            and abs(self._rate) < settings.max_calibration_rate
        )

    def _refit_if_due(self, timestamp: int) -> bool:
        """Fit the continuous calibrations every refit interval."""
        if self._last_fit_time is None:
            self._last_fit_time = timestamp
        if timestamp - self._last_fit_time < self.settings.refit_interval_ms:
            return False
        self._last_fit_time = timestamp
        changed = False
        if self.settings.continuous_calibration:
            for channel in self.channels:
                changed = channel.calibration.refit() or changed
        return changed

    def magnetic_heading(self, sample: MagnetometerSample) -> float | None:
        """Return the magnetic heading measured by the sample, without the gyro."""
        if not sample.magnetic_fields:
            return None
        roll, pitch = self.attitude.roll, self.attitude.pitch
        # only the first magnetometer for now
        channel = self.channels[0]
        field = channel.correct(sample.magnetic_fields[0])
        if self._calibration_sample_ok():
            channel.add_calibration_sample(field, roll, pitch)
        return channel.heading(field, roll, pitch)

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
        self.calibration_changed |= self._refit_if_due(sample.timestamp)
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
        positions: list[PositionData] | None = None,
    ) -> list[HeadingData]:
        """
        Add IMU and magnetometer samples and GNSS fixes in the order they were made.

        :return: the heading at the time of each magnetometer sample
        """
        events: list[ImuSample | MagnetometerSample | PositionData] = [
            *imu_samples,
            *magnetometer_samples,
            *(positions or []),
        ]
        headings: list[HeadingData] = []
        for event in sorted(events, key=lambda event: event.timestamp):
            if isinstance(event, ImuSample):
                self.add_imu(event)
            elif isinstance(event, PositionData):
                self.add_position(event)
            else:
                heading = self.add_magnetometer(event)
                if heading is not None:
                    headings.append(heading)
        return headings

    def state_dict(self) -> dict[str, Any]:
        """Return what has been learned, to be restored after a restart."""
        return {
            "version": 1,
            "magnetometers": [channel.state_dict() for channel in self.channels],
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        if state.get("version") != 1:
            logger.warning("Unknown compass calibration version, ignoring it")
            return
        stored = {entry.get("name"): entry for entry in state.get("magnetometers", [])}
        for channel in self.channels:
            if channel.name in stored:
                channel.load_state_dict(stored[channel.name])

    def save(self, path: str | Path) -> None:
        write_json_atomically(path, self.state_dict())
        self.calibration_changed = False

    def load(self, path: str | Path) -> None:
        """Restore a saved state, if there is one."""
        try:
            with Path(path).open(encoding="utf-8") as file:
                self.load_state_dict(json.load(file))
        except FileNotFoundError:
            logger.info("No compass calibration at %s, starting a new one", path)
        except (OSError, ValueError, KeyError, TypeError):
            logger.exception("Couldn't read the compass calibration at %s", path)


def _drain(queue: Queue[list[_T]] | Queue[_T]) -> list[_T]:
    """Return everything waiting in a queue of items or of lists of items."""
    items: list[_T] = []
    while True:
        try:
            item = queue.get_nowait()
        except Empty:
            return items
        if isinstance(item, list):
            items.extend(item)
        else:
            items.append(item)


def worker(  # noqa: PLR0913
    processor: CompassProcessor,
    imu_queue: Queue[list[ImuSample]],
    magnetometer_queue: Queue[list[MagnetometerSample]],
    output_queues: list[Queue[HeadingData]],
    position_queue: Queue[PositionData] | None = None,
    calibration_file: str | None = None,
    stop: threading.Event | None = None,
) -> None:
    """
    Turn the IMU and magnetometer batches and the GNSS fixes into headings.

    :param output_queues: queues receiving the headings, like the compass headings
        received over NMEA 2000
    :param calibration_file: where the continuous calibration is saved when it
        changes
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
        positions: list[PositionData] = (
            [] if position_queue is None else _drain(position_queue)
        )
        for heading in processor.add(
            _drain(imu_queue),
            magnetometer_samples,
            positions,
        ):
            log.data("heading", heading)
            for queue in output_queues:
                queue.put(heading)
        if processor.calibration_changed and calibration_file is not None:
            try:
                processor.save(calibration_file)
            except OSError:
                logger.exception("Couldn't save the compass calibration")


def create_processor() -> CompassProcessor:
    magnetometer_config = config.Config().MAGNETOMETER
    pattern = magnetometer_config.BENCH_CALIBRATION_FILE
    processor = CompassProcessor(
        {
            name: BenchCalibration.load(bench_calibration_file(pattern, name))
            for name in magnetometer.channel_names(magnetometer_config.DEVICES)
        },
        CompassSettings(
            declination=math.radians(magnetometer_config.DECLINATION_DEGREES),
            continuous_calibration=magnetometer_config.CONTINUOUS_CALIBRATION,
        ),
    )
    processor.load(magnetometer_config.CALIBRATION_FILE)
    return processor


def init(
    imu_queue: Queue[list[ImuSample]],
    magnetometer_queue: Queue[list[MagnetometerSample]],
    output_queues: list[Queue[HeadingData]],
    position_queue: Queue[PositionData],
) -> None:
    """Start turning the IMU and magnetometer samples into headings."""
    threading.Thread(
        target=worker,
        args=(
            create_processor(),
            imu_queue,
            magnetometer_queue,
            output_queues,
            position_queue,
            config.Config().MAGNETOMETER.CALIBRATION_FILE,
        ),
        daemon=True,
    ).start()
