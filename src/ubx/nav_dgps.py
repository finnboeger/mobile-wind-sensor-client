from dataclasses import dataclass, field

from pyubx2 import UBXMessage


@dataclass
class NavDgpsSv:
    """Single Satellite DGPS Information (Part of NAV-DGPS)."""

    #: Satellite identifier
    sv_id: int
    #: Bitmask / Channel number
    flags: int
    #: Age of the latest correction data [ms]
    age_c: int
    #: Pseudo Range Correction [m]
    prc: float
    #: Pseudo Range Rate Correction [m/s]
    prrc: float


@dataclass
class UbxNavDgps:
    """Typed wrapper for UBX-NAV-DGPS (DGPS Data Used)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Age of the newest correction data used [ms]
    age: int
    #: DGPS Base Station ID
    base_id: int
    #: DGPS Base Station Health Status
    base_health: int
    #: Number of channels for which correction data is available
    num_ch: int
    #: DGPS Status flags
    status: int
    #: Reserved
    reserved1: int
    #: List of DGPS Satellite Objects
    svs: list[NavDgpsSv] = field(default_factory=list)


def parse_ubx_nav_dgps_message(msg: UBXMessage) -> UbxNavDgps:
    """Parse a NAV-DGPS UBXMessage into a UbxNavDgps object."""
    if msg.identity != "NAV-DGPS":
        error = f"Expected NAV-DGPS message, got {msg.identity}."
        raise ValueError(error)

    sv_list = []
    num_ch = getattr(msg, "numCh")

    for i in range(1, num_ch + 1):
        idx = f"{i:02d}"
        sv_list.append(
            NavDgpsSv(
                sv_id=getattr(msg, f"svid_{idx}"),
                flags=getattr(msg, f"flags_{idx}"),
                age_c=getattr(msg, f"ageC_{idx}"),
                prc=getattr(msg, f"prc_{idx}"),
                prrc=getattr(msg, f"prrc_{idx}"),
            ),
        )

    return UbxNavDgps(
        i_tow=getattr(msg, "iTOW"),
        age=getattr(msg, "age"),
        base_id=getattr(msg, "baseId"),
        base_health=getattr(msg, "baseHealth"),
        num_ch=num_ch,
        status=getattr(msg, "status"),
        reserved1=getattr(msg, "reserved1"),
        svs=sv_list,
    )
