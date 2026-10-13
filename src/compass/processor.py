"""
Heading from the magnetometer, gyro and accelerometer, with continuous calibration.

:py:class:`CompassProcessor` contains no hardware access, so the same code runs live,
in tests and on recorded ``compass_raw`` data.
"""

import logging
import math
from collections import deque
from dataclasses import dataclass
from typing import Any

from compass.attitude import (
    Attitude,
    AttitudeFilter,
    heading_from_level_field,
    level,
    vertical_rate,
)
from compass.calibration import BenchCalibration, ContinuousCalibration, GnssAlignment
from compass.heading import HeadingFilter, wrap_angle
from structs import CompassSample, PositionData
from utils.linalg import Vector3, norm

logger = logging.getLogger(__name__)

#: largest time step in seconds, longer gaps are treated as this long
_MAX_DT = 0.5
#: magnetometer samples whose magnitude differs from the expected field strength by
#: more than this fraction are ignored, e.g. while a current flows nearby
_FIELD_TOLERANCE = 0.15
#: time constant in seconds of the expected field strength
_FIELD_TIME_CONSTANT = 600.0
#: if the field strength stays off for this long, it is accepted as the new normal,
#: in milliseconds
_FIELD_RESET_AFTER_MS = 60_000
#: speed and acceleration from GNSS are used for this long after a fix, in ms
_MAX_POSITION_AGE_MS = 3000
#: how long the heading history for the GNSS alignment reaches back, in ms
_HEADING_HISTORY_MS = 5000
#: the heading for a GNSS fix must have been recorded this close to it, in ms
_MAX_HEADING_TIME_OFFSET_MS = 250
#: the boat must have gone straight for this long before a fix is used, in ms
_STRAIGHT_FOR_MS = 3000
#: samples taken while turning faster than this are not used for calibration
_MAX_CALIBRATION_RATE = math.radians(45)
#: two magnetometers further apart than this disagree, only the one closer to the
#: gyro's prediction is used
_MAX_SENSOR_DISAGREEMENT = math.radians(8)
#: warn when the magnetometers have disagreed for this long, in milliseconds
_DISAGREEMENT_WARNING_MS = 60_000


class MagnetometerChannel:
    """Calibration and plausibility check of one magnetometer."""

    def __init__(self, name: str, bench_calibration: BenchCalibration) -> None:
        self.name = name
        self.bench_calibration = bench_calibration
        self.calibration = ContinuousCalibration()
        #: expected strength of the earth's field after calibration in µT
        self.field_strength: float | None = bench_calibration.field_strength
        self._field_rejected_since: int | None = None

    def field_ok(self, timestamp: int, magnitude: float, dt: float) -> bool:
        """Check the field strength, e.g. to ignore samples while a current flows."""
        if self.field_strength is None:
            self.field_strength = magnitude
            return True
        if (
            abs(magnitude - self.field_strength)
            > _FIELD_TOLERANCE * self.field_strength
        ):
            if self._field_rejected_since is None:
                self._field_rejected_since = timestamp
            elif timestamp - self._field_rejected_since > _FIELD_RESET_AFTER_MS:
                logger.warning(
                    "Magnetic field strength at %s changed from %.1f µT to %.1f µT",
                    self.name,
                    self.field_strength,
                    magnitude,
                )
                self.field_strength = magnitude
                self._field_rejected_since = None
            return False
        self._field_rejected_since = None
        self.field_strength += min(1.0, dt / _FIELD_TIME_CONSTANT) * (
            magnitude - self.field_strength
        )
        return True

    def heading(self, field_body: Vector3, attitude: Attitude) -> float:
        horizontal = self.calibration.calibration
        level_x, level_y, _ = level(horizontal.remove_offset(field_body), attitude)
        corrected_x, corrected_y = horizontal.apply_soft_iron(level_x, level_y)
        return heading_from_level_field(corrected_x, corrected_y)

    def state_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "field_strength": self.field_strength,
            "calibration": self.calibration.state_dict(),
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        field_strength = state.get("field_strength")
        if field_strength is not None:
            self.field_strength = float(field_strength)
        if "calibration" in state:
            self.calibration.load_state_dict(state["calibration"])


@dataclass(frozen=True, kw_only=True)
class CompassSettings:
    #: magnetic declination in radians, positive east
    declination: float = 0.0
    #: learn the boat's iron while underway
    continuous_calibration: bool = True
    #: learn the heading offset from the course over ground
    gnss_alignment: bool = True
    #: calibration samples are only taken while tilted less than this, in radians
    max_calibration_tilt: float = math.radians(20)
    #: time between calibration fits in milliseconds
    refit_interval_ms: int = 30_000
    #: minimum speed over ground in m/s for the GNSS alignment
    min_alignment_speed: float = 3.0
    #: maximum rate of turn in rad/s for the GNSS alignment
    max_alignment_rate: float = math.radians(3)


@dataclass(frozen=True, kw_only=True)
class CompassOutput:
    #: timestamp in milliseconds
    timestamp: int
    #: heading in radians, magnetic heading plus declination and GNSS alignment
    heading: float
    #: heading in radians relative to magnetic north
    magnetic_heading: float
    #: roll in radians, positive with the starboard side down
    roll: float
    #: pitch in radians, positive with the bow up
    pitch: float
    #: whether the continuous calibration has made at least one fit
    calibrated: bool
    #: whether the GNSS alignment offset is known
    aligned: bool


class CompassProcessor:
    def __init__(
        self,
        *,
        settings: CompassSettings | None = None,
        magnetometers: dict[str, BenchCalibration] | None = None,
    ) -> None:
        """
        Create a processor.

        :param settings: settings, or None for the defaults
        :param magnetometers: name and bench calibration of each magnetometer, in the
            order of :py:attr:`structs.CompassSample.magnetic_fields`
        """
        self.settings = settings or CompassSettings()
        if not magnetometers:
            magnetometers = {"magnetometer": BenchCalibration()}
        self.channels = [
            MagnetometerChannel(name, bench) for name, bench in magnetometers.items()
        ]
        self.attitude_filter = AttitudeFilter()
        self.heading_filter = HeadingFilter()
        self.alignment = GnssAlignment()
        self._disagreeing_since: int | None = None

        self._last_time: int | None = None
        self._last_fit_time: int | None = None
        self._last_turn_time: int | None = None
        #: (timestamp, heading before the GNSS alignment)
        self._headings: deque[tuple[int, float]] = deque()

        self._speed = 0.0
        self._along_track_acceleration = 0.0
        self._position_time: int | None = None

    # ----------------------------------------------------------------------------------
    # GNSS input

    def add_position(self, position: PositionData) -> None:
        """Use a GNSS fix for the motion compensation and the alignment."""
        # HNR-PVT repeats or interpolates the GNSS solution, see wind.py
        if not position.valid or position.source not in (None, "NAV-PVT"):
            return

        if (
            self._position_time is not None
            and 0 < position.timestamp - self._position_time < _MAX_POSITION_AGE_MS
        ):
            dt = (position.timestamp - self._position_time) / 1000
            acceleration = (position.speed - self._speed) / dt
            # smooth over about a second, GNSS speed differences are noisy
            weight = min(1.0, dt)
            self._along_track_acceleration += weight * (
                acceleration - self._along_track_acceleration
            )
        else:
            self._along_track_acceleration = 0.0
        self._speed = position.speed
        self._position_time = position.timestamp

        if self.settings.gnss_alignment:
            self._align(position)

    def _align(self, position: PositionData) -> None:
        # differences collected before the calibration has converged would stay in
        # the alignment for a long time
        if self.settings.continuous_calibration and not self.calibrated:
            return
        if position.speed < self.settings.min_alignment_speed:
            return
        if (
            position.speed_accuracy is not None
            and position.speed_accuracy > 0.2 * position.speed
        ):
            return
        if (
            self._last_turn_time is None
            or position.timestamp - self._last_turn_time < _STRAIGHT_FOR_MS
        ):
            return
        heading = self._heading_at(position.timestamp)
        if heading is None:
            return
        if position.velocity_north is not None and position.velocity_east is not None:
            course = math.atan2(position.velocity_east, position.velocity_north)
        else:
            course = math.radians(position.true_course)
        self.alignment.add_sample(heading, course)

    def _heading_at(self, timestamp: int) -> float | None:
        best = min(
            self._headings,
            key=lambda entry: abs(entry[0] - timestamp),
            default=None,
        )
        if best is None or abs(best[0] - timestamp) > _MAX_HEADING_TIME_OFFSET_MS:
            return None
        return best[1]

    # ----------------------------------------------------------------------------------
    # IMU input

    def _boat_acceleration(self, timestamp: int, yaw_rate: float) -> Vector3:
        if (
            self._position_time is None
            or abs(timestamp - self._position_time) > _MAX_POSITION_AGE_MS
        ):
            return (0.0, 0.0, 0.0)
        # centripetal acceleration in a turn points to the inside of the turn
        return (self._along_track_acceleration, self._speed * yaw_rate, 0.0)

    @property
    def calibrated(self) -> bool:
        """Whether the continuous calibration of any magnetometer has converged."""
        return any(channel.calibration.converged for channel in self.channels)

    def _magnetic_heading(
        self,
        timestamp: int,
        headings: list[tuple[MagnetometerChannel, float]],
    ) -> float | None:
        """
        Combine the headings of the magnetometers that passed the field check.

        Uncalibrated magnetometers are ignored once another one is calibrated. If two
        disagree, the one closer to the gyro's prediction is used, which catches a
        disturbance close to one of them, e.g. a current on the board.
        """
        if any(channel.calibration.converged for channel, _ in headings):
            headings = [h for h in headings if h[0].calibration.converged]
        if len(headings) == 0:
            return None
        values = [heading for _, heading in headings]
        spread = max(abs(wrap_angle(value - values[0])) for value in values)
        predicted = self.heading_filter.heading
        if spread <= _MAX_SENSOR_DISAGREEMENT or predicted is None:
            self._disagreeing_since = None
            return (
                math.atan2(
                    sum(math.sin(v) for v in values),
                    sum(math.cos(v) for v in values),
                )
                % math.tau
            )

        if self._disagreeing_since is None:
            self._disagreeing_since = timestamp
        elif timestamp - self._disagreeing_since > _DISAGREEMENT_WARNING_MS:
            logger.warning(
                "Magnetometers have disagreed by up to %.1f° for a minute: %s",
                math.degrees(spread),
                ", ".join(
                    f"{channel.name} {math.degrees(heading):.1f}°"
                    for channel, heading in headings
                ),
            )
            self._disagreeing_since = timestamp
        return min(values, key=lambda value: abs(wrap_angle(value - predicted)))

    def _collect_calibration_sample(
        self,
        channel: MagnetometerChannel,
        field_body: Vector3,
        attitude: Attitude,
    ) -> None:
        if (
            self.settings.continuous_calibration
            and self.attitude_filter.accelerometer_used
            and abs(attitude.roll) < self.settings.max_calibration_tilt
            and abs(attitude.pitch) < self.settings.max_calibration_tilt
            and abs(self.heading_filter.rate) < _MAX_CALIBRATION_RATE
        ):
            uncorrected_x, uncorrected_y, _ = level(field_body, attitude)
            channel.calibration.add_sample(uncorrected_x, uncorrected_y)

    def _refit_if_due(self, timestamp: int) -> None:
        if self._last_fit_time is None:
            self._last_fit_time = timestamp
        elif timestamp - self._last_fit_time >= self.settings.refit_interval_ms:
            self._last_fit_time = timestamp
            if self.settings.continuous_calibration:
                for channel in self.channels:
                    channel.calibration.refit()

    def process(self, sample: CompassSample) -> CompassOutput | None:
        """
        Process one sample of all three sensors.

        :return: the heading, or None while the filters are starting up
        """
        timestamp = sample.timestamp
        dt = 0.0
        if self._last_time is not None:
            dt = min(max(0, timestamp - self._last_time) / 1000, _MAX_DT)
        self._last_time = timestamp

        gyro = sample.angular_rate
        attitude = self.attitude_filter.update(
            dt,
            gyro,
            sample.acceleration,
            self._boat_acceleration(timestamp, gyro[2]),
        )
        if attitude is None:
            return None

        bias = self.attitude_filter.gyro_bias
        rate = vertical_rate((gyro[0] - bias[0], gyro[1] - bias[1], gyro[2]), attitude)
        self.heading_filter.predict(dt, rate)
        if (
            self._last_turn_time is None
            or abs(self.heading_filter.rate) > self.settings.max_alignment_rate
        ):
            self._last_turn_time = timestamp

        headings: list[tuple[MagnetometerChannel, float]] = []
        for channel, raw in zip(self.channels, sample.magnetic_fields, strict=False):
            field_body = channel.bench_calibration.apply(raw)
            if channel.field_ok(timestamp, norm(field_body), dt):
                headings.append((channel, channel.heading(field_body, attitude)))
                self._collect_calibration_sample(channel, field_body, attitude)
        magnetic_heading = self._magnetic_heading(timestamp, headings)
        if magnetic_heading is not None:
            self.heading_filter.update(timestamp, magnetic_heading)
        self._refit_if_due(timestamp)

        magnetic_heading = self.heading_filter.heading
        if magnetic_heading is None:
            return None
        unaligned = (magnetic_heading + self.settings.declination) % math.tau
        self._headings.append((timestamp, unaligned))
        while self._headings and self._headings[0][0] < timestamp - _HEADING_HISTORY_MS:
            self._headings.popleft()

        alignment = self.alignment.offset if self.settings.gnss_alignment else None
        heading = unaligned if alignment is None else (unaligned + alignment) % math.tau
        return CompassOutput(
            timestamp=timestamp,
            heading=heading,
            magnetic_heading=magnetic_heading,
            roll=attitude.roll,
            pitch=attitude.pitch,
            calibrated=self.calibrated,
            aligned=alignment is not None,
        )

    # ----------------------------------------------------------------------------------
    # Persistence

    def state_dict(self) -> dict[str, Any]:
        return {
            "version": 2,
            "attitude": self.attitude_filter.state_dict(),
            "heading_gyro_bias": self.heading_filter.bias,
            "magnetometers": [channel.state_dict() for channel in self.channels],
            "alignment": self.alignment.state_dict(),
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        version = state.get("version")
        if version == 1:
            # a single magnetometer, stored before more than one was supported
            magnetometers = [
                {
                    "name": self.channels[0].name,
                    "field_strength": state.get("field_strength"),
                    "calibration": state.get("calibration", {}),
                },
            ]
        elif version == 2:  # noqa: PLR2004
            magnetometers = state.get("magnetometers", [])
        else:
            logger.warning("Unknown compass calibration version, ignoring it")
            return
        self.attitude_filter.load_state_dict(state.get("attitude", {}))
        self.heading_filter.bias = float(state.get("heading_gyro_bias", 0.0))
        stored = {entry.get("name"): entry for entry in magnetometers}
        for channel in self.channels:
            if channel.name in stored:
                channel.load_state_dict(stored[channel.name])
        if "alignment" in state:
            self.alignment.load_state_dict(state["alignment"])


def heading_difference(a: float, b: float) -> float:
    """Return the signed difference ``a - b`` of two headings in radians."""
    return wrap_angle(a - b)
