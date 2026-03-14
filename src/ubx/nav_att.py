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
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        roll=getattr(msg, "roll"),
        pitch=getattr(msg, "pitch"),
        heading=getattr(msg, "heading"),
        acc_roll=getattr(msg, "accRoll"),
        acc_pitch=getattr(msg, "accPitch"),
        acc_heading=getattr(msg, "accHeading"),
    )
