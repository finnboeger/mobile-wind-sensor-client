from dataclasses import dataclass
from queue import Queue

type PositionQueue = Queue[PositionData]
type WindOutputQueue = Queue[tuple[CorrectedApparentWindData, TrueWindData]]


@dataclass(frozen=False, kw_only=True)
class PositionData:
    """Positioning data from one GPS fix"""

    #: UTC time of the fix in milliseconds since epoch
    timestamp: int  # Used when forwarding
    #: Fix status
    valid: bool
    #: Latitude in degrees, positive for North, negative for South
    latitude: float  # Used when forwarding
    #: Longitude in degrees, positive for East, negative for West
    longitude: float  # Used when forwarding
    #: Altitude in meters above mean sea level (geoid)
    altitude: float  # Used when forwarding
    #: Difference between WGS-84 ellipsoid and mean sea level (geoid) in meters.
    #: Negative values indicate that the mean-sea-level is below the ellipsoid.
    geoidal_separation: float  # Used when forwarding
    #: Speed over ground in knots
    speed: float  # Used when forwarding and for calc
    #: True course over ground in degrees.
    true_course: float  # Used when forwarding and for calc
    #: Age of differential GPS data in seconds since last SC104 type 1 or 9 update.
    #: None if Differential GPS is not used.
    differential_gps_data_age: int | None = None  # Used when forwarding
    #: ID of the differential reference station used. 0000 - 1023.
    #: Seems to default to 0000 if Differential GPS is not used.
    differential_reference_station_id: int | None = None  # Used when forwarding
    #: Number of satellites used in the fix. Can differ from `len(used_satellites)`
    #: if more than 12 satellites are used, as that is the maximum that can be reported
    #: by the GSA sentence.
    number_satellites_used: int | None = None  # Used when forwarding
    #: Positional dilution of precision in meters.
    positional_dilution_of_precision: float | None = None  # Used when forwarding
    #: Horizontal dilution of precision in meters.
    horizontal_dilution_of_precision: float | None = None  # Used when forwarding


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
    """Apparent wind with wind speed and angle."""


class CorrectedApparentWindData(WindData):
    """Apparent wind with wind speed and direction."""


class TrueWindData(WindData):
    """True wind with wind speed and direction."""
