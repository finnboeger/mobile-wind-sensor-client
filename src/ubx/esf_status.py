from dataclasses import dataclass, field

from pyubx2 import UBXMessage

from ubx.shared_types import EsfSensorInfo, EsfSensorStatus, EsfSensorType


@dataclass
class UbxEsfStatus:
    """Typed wrapper for UBX-ESF-STATUS (External Sensor Fusion Status)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Initialization status
    init_status: int  # tuple (wtInit, mntAlgStatus, insInitStatus)
    #: Fusion mode
    fusion_mode: int
    #: Number of sensors attached
    num_sens: int
    #: List of Sensor Status Objects
    sensors: list[EsfSensorInfo] = field(default_factory=list)


def parse_ubx_esf_status_message(msg: UBXMessage) -> UbxEsfStatus:
    """Parse an ESF-STATUS UBXMessage into a UbxEsfStatus object."""
    if msg.identity != "ESF-STATUS":
        error = f"Expected ESF-STATUS message, got {msg.identity}."
        raise ValueError(error)

    sensor_list = []
    num_sens = getattr(msg, "numSens")

    for i in range(1, num_sens + 1):
        idx = f"{i:02d}"

        s1 = getattr(msg, f"sensStatus1_{idx}")
        s2 = getattr(msg, f"sensStatus2_{idx}")
        freq = getattr(msg, f"freq_{idx}")

        # Parsing sensStatus1
        s_type = s1 & 0x3F
        s_used = bool((s1 >> 6) & 1)
        s_ready = bool((s1 >> 7) & 1)

        # Parsing sensStatus2
        s_calib = s2 & 0x03
        s_time_status = (s2 >> 2) & 0x03
        # bits 4-15 are faults, we simplify here
        faults = s2 >> 4

        sensor_list.append(
            EsfSensorInfo(
                type=EsfSensorType(s_type)
                if s_type in list(map(int, EsfSensorType))
                else EsfSensorType.NONE,
                used=s_used,
                ready=s_ready,
                calib_status=EsfSensorStatus(s_calib),
                time_status=s_time_status,
                freq=freq,
                faults_bad_meas=bool(faults & 1),
                faults_bad_tag=bool((faults >> 1) & 1),
                faults_missing=bool((faults >> 2) & 1),
                faults_noise=bool((faults >> 3) & 1),
            ),
        )

    # initStatus is often a tuple or list in pyubx2
    init_st = getattr(msg, "initStatus")

    return UbxEsfStatus(
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        init_status=init_st,
        fusion_mode=getattr(msg, "fusionMode"),
        num_sens=num_sens,
        sensors=sensor_list,
    )
