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
    num_svs = getattr(msg, "numSvs")

    for i in range(1, num_svs + 1):
        idx = f"{i:02d}"  # format "01", "02", etc.
        sv_list.append(
            NavSatSv(
                gnss_id=getattr(msg, f"gnssId_{idx}"),
                sv_id=getattr(msg, f"svId_{idx}"),
                cno=getattr(msg, f"cno_{idx}"),
                elev=getattr(msg, f"elev_{idx}"),
                azim=getattr(msg, f"azim_{idx}"),
                pr_res=getattr(msg, f"prRes_{idx}"),
                flags=getattr(msg, f"flags_{idx}"),
            ),
        )

    return UbxNavSat(
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        num_svs=num_svs,
        reserved0=getattr(msg, "reserved0"),
        svs=sv_list,
    )
