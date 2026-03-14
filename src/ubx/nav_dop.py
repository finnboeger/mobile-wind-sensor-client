from dataclasses import dataclass

from pyubx2 import UBXMessage


@dataclass
class UbxNavDop:
    """Typed wrapper for UBX-NAV-DOP (Dilution of Precision)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Geometric DOP
    g_dop: float
    #: Position DOP
    p_dop: float
    #: Time DOP
    t_dop: float
    #: Vertical DOP
    v_dop: float
    #: Horizontal DOP
    h_dop: float
    #: Northing DOP
    n_dop: float
    #: Easting DOP
    e_dop: float


def parse_ubx_nav_dop_message(msg: UBXMessage) -> UbxNavDop:
    """Parse a NAV-DOP UBXMessage into a UbxNavDop object."""
    if msg.identity != "NAV-DOP":
        error = f"Expected NAV-DOP message, got {msg.identity}."
        raise ValueError(error)

    return UbxNavDop(
        i_tow=getattr(msg, "iTOW"),
        g_dop=getattr(msg, "gDOP"),
        p_dop=getattr(msg, "pDOP"),
        t_dop=getattr(msg, "tDOP"),
        v_dop=getattr(msg, "vDOP"),
        h_dop=getattr(msg, "hDOP"),
        n_dop=getattr(msg, "nDOP"),
        e_dop=getattr(msg, "eDOP"),
    )
