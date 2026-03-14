from dataclasses import dataclass

from pyubx2 import UBXMessage

from ubx.shared_types import GnssFixType


@dataclass
class UbxHnrPvt:
    """Typed wrapper for UBX-HNR-PVT (High Rate Output of PVT)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Year (UTC)
    year: int
    #: Month, range 1..12 (UTC)
    month: int
    #: Day of month, range 1..31 (UTC)
    day: int
    #: Hour of day, range 0..23 (UTC)
    hour: int
    #: Minute of hour, range 0..59 (UTC)
    minute: int
    #: Seconds of minute, range 0..60 (UTC)
    second: int
    #: Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
    nano: int
    #: Validity flags
    valid_date: int
    valid_time: int
    fully_resolved: int
    #: GNSSfix Type
    fix_type: GnssFixType
    #: Fix status flags
    gps_fix_ok: bool
    dgps_used: bool
    week_number_set: bool
    time_of_week_set: bool
    heading_of_vehicle_valid: bool
    #: Longitude [deg]
    lon: float
    #: Latitude [deg]
    lat: float
    #: Height above ellipsoid [mm]
    height: int
    #: Height above mean sea level [mm]
    height_above_mean_sea_level: int
    #: Ground Speed (2-D) [mm/s]
    ground_speed: int
    #: Heading of motion (2-D) [deg]
    heading_of_motion: float
    #: Heading of vehicle (2-D) [deg]
    heading_of_vehicle: float
    #: Horizontal accuracy estimate [mm]
    horizontal_accuracy: int
    #: Vertical accuracy estimate [mm]
    vertical_accuracy: int
    #: Speed accuracy estimate [mm/s]
    speed_accuracy: int
    #: Heading accuracy estimate [deg]
    heading_accuracy: float


def parse_ubx_hnr_pvt_message(msg: UBXMessage) -> UbxHnrPvt:
    """Parse an HNR-PVT UBXMessage into a UbxHnrPvt object."""
    if msg.identity != "HNR-PVT":
        error = f"Expected HNR-PVT message, got {msg.identity}."
        raise ValueError(error)

    return UbxHnrPvt(
        i_tow=getattr(msg, "iTOW"),
        year=getattr(msg, "year"),
        month=getattr(msg, "month"),
        day=getattr(msg, "day"),
        hour=getattr(msg, "hour"),
        minute=getattr(msg, "min"),
        second=getattr(msg, "second"),
        nano=getattr(msg, "nano"),
        valid_date=getattr(msg, "validDate"),
        valid_time=getattr(msg, "validTime"),
        fully_resolved=getattr(msg, "fullyResolved"),
        fix_type=GnssFixType(
            getattr(msg, "gpsFix"),
        ),
        gps_fix_ok=bool(getattr(msg, "GPSfixOK")),
        dgps_used=bool(getattr(msg, "DiffSoln")),
        week_number_set=bool(getattr(msg, "WKNSET")),
        time_of_week_set=bool(getattr(msg, "TOWSET")),
        heading_of_vehicle_valid=bool(getattr(msg, "headVehValid")),
        lon=getattr(msg, "lon"),
        lat=getattr(msg, "lat"),
        height=getattr(msg, "height"),
        height_above_mean_sea_level=getattr(msg, "hMSL"),
        ground_speed=getattr(msg, "gSpeed"),
        heading_of_motion=getattr(msg, "headMot"),
        heading_of_vehicle=getattr(msg, "headVeh"),
        horizontal_accuracy=getattr(msg, "hAcc"),
        vertical_accuracy=getattr(msg, "vAcc"),
        speed_accuracy=getattr(msg, "sAcc"),
        heading_accuracy=getattr(msg, "headAcc"),
    )
