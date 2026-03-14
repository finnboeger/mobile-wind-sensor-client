from dataclasses import dataclass

from pyubx2 import UBXMessage

from ubx.shared_types import (
    CarrierPhaseSolution,
    GnssFixType,
    LastCorrectionAge,
    PowerSaveModeState,
)


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
    #: Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
    nano: int
    #: Validity flags
    valid_date: bool
    valid_time: bool
    fully_resolved: bool
    valid_magnetic_declination: bool
    #: Time accuracy estimate (UTC) [ns]
    time_accuracy: int
    #: GNSSfix Type
    fix_type: GnssFixType
    #: Fix status flags
    gnss_fix_ok: bool
    dgps_used: bool
    power_save_mode: PowerSaveModeState
    carrier_phase_solution: CarrierPhaseSolution
    heading_of_vehicle_valid: bool
    #: Additional flags
    date_time_confirmation_available: bool
    confirmed_date: bool
    confirmed_time: bool
    #: Number of satellites used in Nav Solution
    num_sv: int
    #: Longitude [deg]
    lon: float
    #: Latitude [deg]
    lat: float
    #: Height above ellipsoid [mm]
    height: int
    #: Height above mean sea level [mm]
    height_above_mean_sea_level: int
    #: Horizontal accuracy estimate [mm]
    horizontal_accuracy: int
    #: Vertical accuracy estimate [mm]
    vertical_accuracy: int
    #: NED north velocity [mm/s]
    vel_n: int
    #: NED east velocity [mm/s]
    vel_e: int
    #: NED down velocity [mm/s]
    vel_d: int
    #: Ground Speed (2-D) [mm/s]
    ground_speed: int
    #: Heading of motion (2-D) [deg]
    heading_of_motion: float
    #: Speed accuracy estimate [mm/s]
    speed_accuracy: int
    #: Heading accuracy estimate [deg]
    heading_accuracy: float
    #: Position DOP
    p_dop: float
    # Flags3
    #: longitude, latitude and height are invalid
    invalid_lon_lat_height: bool
    last_correction_age: LastCorrectionAge
    #: output time has been validated against external trusted time source
    authenticated_time: bool
    #: solution has been verified using data authenticated through
    #:  Navigation Message Authentication (NMA) methods
    nma_fix_status: bool
    #: Heading of vehicle (2-D) [deg]
    heading_of_vehicle: float
    #: Magnetic declination [deg]
    magnetic_declination: float
    #: Magnetic declination accuracy [deg]
    magnetic_declination_accuracy: float


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
        second=getattr(msg, "second"),
        valid_date=bool(getattr(msg, "validDate")),
        valid_time=bool(getattr(msg, "validTime")),
        fully_resolved=bool(getattr(msg, "fullyResolved")),
        valid_magnetic_declination=bool(getattr(msg, "validMag")),
        time_accuracy=getattr(msg, "tAcc"),
        nano=getattr(msg, "nano"),
        fix_type=GnssFixType(getattr(msg, "fixType")),
        gnss_fix_ok=bool(getattr(msg, "gnssFixOk")),
        dgps_used=bool(getattr(msg, "diffSoln")),
        power_save_mode=PowerSaveModeState(getattr(msg, "psmState")),
        carrier_phase_solution=CarrierPhaseSolution(getattr(msg, "carrSoln")),
        heading_of_vehicle_valid=bool(getattr(msg, "headVehValid")),
        date_time_confirmation_available=bool(getattr(msg, "confirmedAvai")),
        confirmed_date=bool(getattr(msg, "confirmedDate")),
        confirmed_time=bool(getattr(msg, "confirmedTime")),
        num_sv=getattr(msg, "numSV"),
        lon=getattr(msg, "lon"),
        lat=getattr(msg, "lat"),
        height=getattr(msg, "height"),
        height_above_mean_sea_level=getattr(msg, "hMSL"),
        horizontal_accuracy=getattr(msg, "hAcc"),
        vertical_accuracy=getattr(msg, "vAcc"),
        vel_n=getattr(msg, "velN"),
        vel_e=getattr(msg, "velE"),
        vel_d=getattr(msg, "velD"),
        ground_speed=getattr(msg, "gSpeed"),
        heading_of_motion=getattr(msg, "headMot"),
        speed_accuracy=getattr(msg, "sAcc"),
        heading_accuracy=getattr(msg, "headAcc"),
        p_dop=getattr(msg, "pDOP"),
        invalid_lon_lat_height=bool(getattr(msg, "invalidLlh")),
        last_correction_age=LastCorrectionAge(getattr(msg, "lastCorrectionAge")),
        authenticated_time=bool(getattr(msg, "authTime")),
        nma_fix_status=bool(getattr(msg, "nmaFixStatus")),
        heading_of_vehicle=getattr(msg, "headVeh"),
        magnetic_declination=getattr(msg, "magDec"),
        magnetic_declination_accuracy=getattr(msg, "magAcc"),
    )
