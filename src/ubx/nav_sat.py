from dataclasses import dataclass, field
from typing import Any

from pyubx2 import UBXMessage

from ubx.shared_types import NavSatSv


@dataclass
class UbxNavSat:
    """Typed wrapper for UBX-NAV-SAT (Satellite Information)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Number of satellites
    num_svs: int
    #: Reserved
    reserved0: Any
    #: List of Satellite Objects
    svs: list[NavSatSv] = field(default_factory=list)


def parse_ubx_nav_sat_message(msg: UBXMessage) -> UbxNavSat:
    """Parse a NAV-SAT UBXMessage into a UbxNavSat object."""
    if msg.identity != "NAV-SAT":
        error = f"Expected NAV-SAT message, got {msg.identity}."
        raise ValueError(error)

    sv_list = []
    num_svs = getattr(msg, "numSvs", 0)

    for i in range(1, num_svs + 1):
        idx = f"{i:02d}"  # format "01", "02", etc.
        sv_list.append(
            NavSatSv(
                gnss_id=getattr(msg, f"gnssId_{idx}", 0),
                sv_id=getattr(msg, f"svId_{idx}", 0),
                cno=getattr(msg, f"cno_{idx}", 0),
                elev=getattr(msg, f"elev_{idx}", 0),
                azim=getattr(msg, f"azim_{idx}", 0),
                pr_res=getattr(msg, f"prRes_{idx}", 0.0),
                flags=getattr(msg, f"flags_{idx}", 0),
            ),
        )

    return UbxNavSat(
        i_tow=getattr(msg, "iTOW", 0),
        version=getattr(msg, "version", 0),
        num_svs=num_svs,
        reserved0=getattr(msg, "reserved0", None),
        svs=sv_list,
    )
