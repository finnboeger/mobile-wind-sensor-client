from dataclasses import dataclass

from pyubx2 import UBXMessage


@dataclass
class UbxNavAtt:
    """Typed wrapper for UBX-NAV-ATT (Attitude Solution)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Vehicle Roll [deg] (Scaled 1e-5)
    roll: float
    #: Vehicle Pitch [deg] (Scaled 1e-5)
    pitch: float
    #: Vehicle Heading [deg] (Scaled 1e-5)
    heading: float
    #: Vehicle Roll Accuracy [deg] (Scaled 1e-5)
    acc_roll: float
    #: Vehicle Pitch Accuracy [deg] (Scaled 1e-5)
    acc_pitch: float
    #: Vehicle Heading Accuracy [deg] (Scaled 1e-5)
    acc_heading: float


def parse_ubx_nav_att_message(msg: UBXMessage) -> UbxNavAtt:
    """Parse a NAV-ATT UBXMessage into a UbxNavAtt object."""
    if msg.identity != "NAV-ATT":
        error = f"Expected NAV-ATT message, got {msg.identity}."
        raise ValueError(error)

    return UbxNavAtt(
        i_tow=getattr(msg, "iTOW", 0),
        version=getattr(msg, "version", 0),
        roll=getattr(msg, "roll", 0.0),
        pitch=getattr(msg, "pitch", 0.0),
        heading=getattr(msg, "heading", 0.0),
        acc_roll=getattr(msg, "accRoll", 0.0),
        acc_pitch=getattr(msg, "accPitch", 0.0),
        acc_heading=getattr(msg, "accHeading", 0.0),
    )
