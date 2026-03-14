from dataclasses import dataclass, field
from typing import Any

from pyubx2 import UBXMessage

from ubx.shared_types import NavSatSv, OrbitSource, SignalHealth, SignalQualityIndicator


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
                quality_ind=SignalQualityIndicator(
                    getattr(msg, f"qualityInd_{idx}"),
                ),
                sv_used=bool(getattr(msg, f"svUsed_{idx}")),
                health=SignalHealth(getattr(msg, f"health_{idx}")),
                differential_correction_data_available=bool(
                    getattr(msg, f"diffCorr_{idx}"),
                ),
                smoothed=bool(getattr(msg, f"smoothed_{idx}")),
                orbit_source=OrbitSource(getattr(msg, f"orbitSource_{idx}")),
                ephemeris_available=bool(getattr(msg, f"ephAvail_{idx}")),
                almanac_available=bool(getattr(msg, f"almAvail_{idx}")),
                assist_now_offline_available=bool(getattr(msg, f"anoAvail_{idx}")),
                assist_now_autonomous_available=bool(
                    getattr(msg, f"aopAvail_{idx}"),
                ),
                sbas_correction_used=bool(getattr(msg, f"sbasCorrUsed_{idx}")),
                rtcm_correction_used=bool(getattr(msg, f"rtcmCorrUsed_{idx}")),
                slas_correction_used=bool(getattr(msg, f"slasCorrUsed_{idx}")),
                spartn_correction_used=bool(getattr(msg, f"spartnCorrUsed_{idx}")),
                pseudorange_correction_used=bool(getattr(msg, f"prCorrUsed_{idx}")),
                carrier_range_correction_used=bool(getattr(msg, f"crCorrUsed_{idx}")),
                doppler_correction_used=bool(getattr(msg, f"doCorrUsed_{idx}")),
                clas_correction_used=bool(getattr(msg, f"clasCorrUsed_{idx}")),
            ),
        )

    return UbxNavSat(
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        num_svs=num_svs,
        reserved0=getattr(msg, "reserved0"),
        svs=sv_list,
    )
