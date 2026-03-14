from dataclasses import dataclass, field

from pyubx2 import UBXMessage

from ubx.shared_types import (
    EsfFusionMode,
    EsfSensorCalibrationStatus,
    EsfSensorInfo,
    EsfSensorInitializationStatus,
    EsfSensorTimeStatus,
    EsfSensorType,
)


@dataclass
class UbxEsfStatus:
    """Typed wrapper for UBX-ESF-STATUS (External Sensor Fusion Status)."""

    #: GPS time of week of the navigation epoch [ms]
    i_tow: int
    #: Message version
    version: int
    #: Wheel tick factor initialization status
    wheel_tick_factor_init_status: EsfSensorInitializationStatus
    #: Automatic IMU-mount alignment status
    imu_mount_alignment_status: EsfSensorInitializationStatus
    #: INS initialization status
    ins_init_status: EsfSensorInitializationStatus
    #: IMU initialization status
    imu_init_status: EsfSensorInitializationStatus
    #: Fusion mode
    fusion_mode: EsfFusionMode
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

        sensor_list.append(
            EsfSensorInfo(
                type=EsfSensorType(getattr(msg, f"type_{idx}")),
                used=bool(getattr(msg, f"used_{idx}")),
                ready=bool(getattr(msg, f"ready_{idx}")),
                calib_status=EsfSensorCalibrationStatus(
                    getattr(msg, f"calibStatus_{idx}"),
                ),
                time_status=EsfSensorTimeStatus(getattr(msg, f"timeStatus_{idx}")),
                freq=getattr(msg, f"freq_{idx}"),
                bad_measurements=bool(getattr(msg, f"badMeas_{idx}")),
                bad_time_tag=bool(getattr(msg, f"badTTag_{idx}")),
                missing_measurements=bool(getattr(msg, f"missingMeas_{idx}")),
                noisy_measurements=bool(getattr(msg, f"noisyMeas_{idx}")),
            ),
        )

    return UbxEsfStatus(
        i_tow=getattr(msg, "iTOW"),
        version=getattr(msg, "version"),
        wheel_tick_factor_init_status=EsfSensorInitializationStatus(
            getattr(msg, "wtInitStatus"),
        ),
        imu_mount_alignment_status=EsfSensorInitializationStatus(
            getattr(msg, "mntAlgStatus"),
        ),
        ins_init_status=EsfSensorInitializationStatus(
            getattr(msg, "insInitStatus"),
        ),
        imu_init_status=EsfSensorInitializationStatus(
            getattr(msg, "imuInitStatus"),
        ),
        fusion_mode=EsfFusionMode(getattr(msg, "fusionMode")),
        num_sens=num_sens,
        sensors=sensor_list,
    )
