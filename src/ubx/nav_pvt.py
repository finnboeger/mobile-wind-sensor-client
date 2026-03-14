from dataclasses import dataclass

from pyubx2 import UBXMessage

from ubx.shared_types import GnssFixType


@dataclass
class UbxNavPvt:
    """Typed wrapper for UBX-NAV-PVT (Position Velocity Time)."""

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
    #: Time accuracy estimate (UTC) [ns]
    t_acc: int
    #: Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
    nano: int
    #: GNSSfix Type
    fix_type: GnssFixType
    #: Fix status flags
    flags: int
    #: Additional flags
    flags2: int
    #: Number of satellites used in Nav Solution
    num_sv: int
    #: Longitude [deg]
    lon: float
    #: Latitude [deg]
    lat: float
    #: Height above ellipsoid [mm]
    height: int
    #: Height above mean sea level [mm]
    h_msl: int
    #: Horizontal accuracy estimate [mm]
    h_acc: int
    #: Vertical accuracy estimate [mm]
    v_acc: int
    #: NED north velocity [mm/s]
    vel_n: int
    #: NED east velocity [mm/s]
    vel_e: int
    #: NED down velocity [mm/s]
    vel_d: int
    #: Ground Speed (2-D) [mm/s]
    g_speed: int
    #: Heading of motion (2-D) [deg]
    head_mot: float
    #: Speed accuracy estimate [mm/s]
    s_acc: int
    #: Heading accuracy estimate [deg]
    head_acc: float
    #: Position DOP
    p_dop: float
    #: Heading of vehicle (2-D) [deg]
    head_veh: float
    #: Magnetic declination [deg]
    mag_dec: float
    #: Magnetic declination accuracy [deg]
    mag_acc: float


def parse_ubx_nav_pvt_message(msg: UBXMessage) -> UbxNavPvt:
    """Parse a NAV-PVT UBXMessage into a UbxNavPvt object."""
    if msg.identity != "NAV-PVT":
        error = f"Expected NAV-PVT message, got {msg.identity}."
        raise ValueError(error)

    return UbxNavPvt(
        i_tow=getattr(msg, "iTOW"),
        year=getattr(msg, "year"),
        month=getattr(msg, "month"),
        day=getattr(msg, "day"),
        hour=getattr(msg, "hour"),
        minute=getattr(msg, "min"),
        second=getattr(msg, "sec"),
        valid=getattr(msg, "valid"),
        t_acc=getattr(msg, "tAcc"),
        nano=getattr(msg, "nano"),
        fix_type=GnssFixType(getattr(msg, "fixType")),
        flags=getattr(msg, "flags"),
        flags2=getattr(msg, "flags2"),
        num_sv=getattr(msg, "numSV"),
        lon=getattr(msg, "lon"),
        lat=getattr(msg, "lat"),
        height=getattr(msg, "height"),
        h_msl=getattr(msg, "hMSL"),
        h_acc=getattr(msg, "hAcc"),
        v_acc=getattr(msg, "vAcc"),
        vel_n=getattr(msg, "velN"),
        vel_e=getattr(msg, "velE"),
        vel_d=getattr(msg, "velD"),
        g_speed=getattr(msg, "gSpeed"),
        head_mot=getattr(msg, "headMot"),
        s_acc=getattr(msg, "sAcc"),
        head_acc=getattr(msg, "headAcc"),
        p_dop=getattr(msg, "pDOP"),
        head_veh=getattr(msg, "headVeh"),
        mag_dec=getattr(msg, "magDec"),
        mag_acc=getattr(msg, "magAcc"),
    )
