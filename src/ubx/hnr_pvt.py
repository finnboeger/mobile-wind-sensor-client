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
    #: Validity flags
    valid: int
    #: Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
    nano: int
    #: GNSSfix Type
    fix_type: GnssFixType
    #: Fix status flags
    flags: int
    #: Longitude [deg]
    lon: float
    #: Latitude [deg]
    lat: float
    #: Height above ellipsoid [mm]
    height: int
    #: Height above mean sea level [mm]
    h_msl: int
    #: Ground Speed (2-D) [mm/s]
    g_speed: int
    #: Heading of motion (2-D) [deg]
    head_mot: float
    #: Heading of vehicle (2-D) [deg]
    head_veh: float
    #: Horizontal accuracy estimate [mm]
    h_acc: int
    #: Vertical accuracy estimate [mm]
    v_acc: int
    #: Speed accuracy estimate [mm/s]
    s_acc: int
    #: Heading accuracy estimate [deg]
    head_acc: float


def parse_ubx_hnr_pvt_message(msg: UBXMessage) -> UbxHnrPvt:
    """Parse an HNR-PVT UBXMessage into a UbxHnrPvt object."""
    if msg.identity != "HNR-PVT":
        error = f"Expected HNR-PVT message, got {msg.identity}."
        raise ValueError(error)

    return UbxHnrPvt(
        i_tow=getattr(msg, "iTOW", 0),
        year=getattr(msg, "year", 0),
        month=getattr(msg, "month", 0),
        day=getattr(msg, "day", 0),
        hour=getattr(msg, "hour", 0),
        minute=getattr(msg, "min", 0),
        second=getattr(msg, "sec", 0),
        valid=getattr(msg, "valid", 0),
        nano=getattr(msg, "nano", 0),
        fix_type=GnssFixType(
            getattr(msg, "gpsFix", 0),
        ),  # HNR uses 'gpsFix' field name usually
        flags=getattr(msg, "flags", 0),
        lon=getattr(msg, "lon", 0.0),
        lat=getattr(msg, "lat", 0.0),
        height=getattr(msg, "height", 0),
        h_msl=getattr(msg, "hMSL", 0),
        g_speed=getattr(msg, "gSpeed", 0),
        head_mot=getattr(msg, "headMot", 0.0),
        head_veh=getattr(msg, "headVeh", 0.0),
        h_acc=getattr(msg, "hAcc", 0),
        v_acc=getattr(msg, "vAcc", 0),
        s_acc=getattr(msg, "sAcc", 0),
        head_acc=getattr(msg, "headAcc", 0.0),
    )
