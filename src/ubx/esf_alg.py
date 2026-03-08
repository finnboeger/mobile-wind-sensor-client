from dataclasses import dataclass

from pyubx2 import UBXMessage


@dataclass
class UbxEsfAlg:
    """Typed wrapper for UBX-ESF-ALG (IMU Alignment Information)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Flags
    flags: int
    #: Error flags
    errors: int
    #: IMU-mount Yaw angle [deg] (Scaled 1e-2)
    yaw: float
    #: IMU-mount Pitch angle [deg] (Scaled 1e-2)
    pitch: float
    #: IMU-mount Roll angle [deg] (Scaled 1e-2)
    roll: float


def parse_ubx_esf_alg_message(msg: UBXMessage) -> UbxEsfAlg:
    """Parse an ESF-ALG UBXMessage into a UbxEsfAlg object."""
    if msg.identity != "ESF-ALG":
        error = f"Expected ESF-ALG message, got {msg.identity}."
        raise ValueError(error)

    return UbxEsfAlg(
        i_tow=getattr(msg, "iTOW", 0),
        version=getattr(msg, "version", 0),
        flags=getattr(msg, "flags", 0),
        errors=getattr(msg, "errors", 0),
        yaw=getattr(msg, "yaw", 0.0),
        pitch=getattr(msg, "pitch", 0.0),
        roll=getattr(msg, "roll", 0.0),
    )
