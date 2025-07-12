import enum
from dataclasses import dataclass
from typing import Literal


class FAAMode(enum.Enum):
    """
    FAA mode of GPS fixes.

    Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_sentence_mixes_and_nmea_variations
    """

    AUTONOMOUS = "A"
    DIFFERENTIAL = "D"
    #: Estimated position (e.g. using dead reckoning)
    ESTIMATED = "E"
    RTK_FLOAT = "F"
    NOT_VALID = "N"
    PRECISE = "P"
    RTK_INTEGER = "R"
    SIMULATED = "S"
    QUECTEL_QUERK_UNSAFE = "U"
    QUECTEL_QUERK_CAUTION = "C"


class Quality(enum.IntEnum):
    """
    Quality of GPS fix.

    Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_gga_global_positioning_system_fix_data
    """

    FIX_NOT_AVAILABLE = 0
    GPS_FIX = 1
    DIFFERENTIAL_GPS_FIX = 2
    PPS_FIX = 3
    RTK = 5
    RTK_FLOAT = 5
    ESTIMATED = 6
    MANUAL_INPUT = 7
    SIMULATED = 8


class FixType(enum.IntEnum):
    """
    Type of GPS fix.

    Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_gsa_gps_dop_and_active_satellites
    """

    NO_FIX = 1
    TWO_DIMENSIONAL = 2
    THREE_DIMENSIONAL = 3


@dataclass(frozen=True, kw_only=True)
class Satellite:
    #: Satellite ID or PRN number
    id: int
    #: Elevation in degrees, -90 - 90
    elevation: int
    #: Azimuth in degrees to true north, 0 - 359
    azimuth: int
    #: Signal-to-noise ratio in dB, 0 - 99.
    #: Can be None if satellite that is not tracked is reported.
    signal_to_noise_ratio: int | None


@dataclass(frozen=False, kw_only=True)
class PositionData:
    """Positioning data from one GPS fix"""

    #: UTC time of the fix in seconds since epoch
    timestamp: int
    #: Fix status, A = valid, V = invalid
    status: Literal["A", "V"]
    #: Latitude in degrees, positive for North, negative for South
    latitude: float | None = None
    #: Longitude in degrees, positive for East, negative for West
    longitude: float | None = None
    #: Altitude in meters above mean sea level (geoid)
    altitude: float | None = None
    #: Difference between WGS-84 ellipsoid and mean sea level (geoid) in meters.
    #: Negative values indicate that the mean-sea-level is below the ellipsoid.
    geoidal_separation: float | None = None
    #: Speed over ground in knots
    speed: float | None = None
    #: True course over ground in degrees. None if the receiver is not moving
    true_course: float | None = None
    #: Magnetic variation in degrees, positive for East, negative for West
    magnetic_variation: float | None = None
    #: Magnetic course over ground in degrees.
    #: This is what a compass would show if it was pointing in the direction of travel.
    magnetic_course: float | None = None
    #: FAA mode indicator
    faa_mode: FAAMode
    #: Quality of the GPS fix
    quality: Quality | None = None
    #: Age of differential GPS data in seconds since last SC104 type 1 or 9 update.
    #: None if Differential GPS is not used.
    differential_gps_data_age: int | None = None
    #: ID of the differential reference station used. 0000 - 1023.
    #: Seems to default to 0000 if Differential GPS is not used.
    differential_reference_station_id: int | None = None
    #: Fix type
    fix_type: FixType | None = None
    #: IDs of the satellites used in the fix.
    used_satellites: list[int] | None = None
    #: Number of satellites used in the fix. Can differ from `len(used_satellites)`
    #: if more than 12 satellites are used, as that is the maximum that can be reported
    #: by the GSA sentence.
    number_satellites_used: int | None = None
    #: Positional dilution of precision in meters.
    positional_dilution_of_precision: float | None = None
    #: Horizontal dilution of precision in meters.
    horizontal_dilution_of_precision: float | None = None
    #: Vertical dilution of precision in meters.
    vertical_dilution_of_precision: float | None = None
    #: Satellites visible to the receiver, including those not used in the fix.
    satellites_in_view: set[Satellite]


@dataclass(frozen=True, kw_only=True)
class HeadingData:
    """Data class for heading data."""

    #: heading in radians, 0 is north, positive is clockwise
    heading: float
    #: timestamp of the heading data in milliseconds
    timestamp: int


@dataclass(frozen=True, kw_only=True)
class WindData:
    """Data class for wind data."""

    #: Wind speed in meters per second
    wind_speed: float
    #: Wind angle in radians
    wind_angle: float
    #: timestamp of the wind data in milliseconds
    timestamp: int


class ApparentWindData(WindData):
    pass


class TrueWindData(WindData):
    pass
