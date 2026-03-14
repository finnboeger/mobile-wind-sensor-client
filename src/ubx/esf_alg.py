from dataclasses import dataclass

from pyubx2 import UBXMessage

from ubx.shared_types import IMUMountAlignmentStatus


@dataclass
class UbxEsfAlg:
    """Typed wrapper for UBX-ESF-ALG (IMU Alignment Information)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Automatic IMU-mount alignment is running
    automatic_mount_alignment_running: bool
    #: Status of the IMU-mount alignment
    status: IMUMountAlignmentStatus
    #: IMU-mount tilt (roll and/or pitch) alignment error
    tilt_alg_error: bool
    #: IMU-mount yaw alignment error
    yaw_alg_error: bool
    #: IMU-mount misalignment Euler angle singularity error
    angle_error: bool
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
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        automatic_mount_alignment_running=getattr(
            msg,
            "autoMntAlgOn",
        ),
        status=IMUMountAlignmentStatus(getattr(msg, "status")),
        tilt_alg_error=getattr(msg, "tiltAlgError"),
        yaw_alg_error=getattr(msg, "yawAlgError"),
        angle_error=getattr(msg, "angleError"),
        yaw=getattr(msg, "yaw"),
        pitch=getattr(msg, "pitch"),
        roll=getattr(msg, "roll"),
    )
