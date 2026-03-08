from dataclasses import dataclass
from enum import IntEnum


class GnssFixType(IntEnum):
    """GNSS fix Type (NAV-PVT)."""

    NO_FIX = 0
    DEAD_RECKONING_ONLY = 1
    FIX_2D = 2
    FIX_3D = 3
    GNSS_AND_DEAD_RECKONING = 4
    TIME_ONLY = 5


class EsfSensorType(IntEnum):
    """Sensor Types for ESF-STATUS."""

    NONE = 0
    ACCEL_Z_AXIS = 5
    ACCEL_Y_AXIS = 13
    ACCEL_X_AXIS = 14
    GYRO_Z_AXIS = 10
    GYRO_Y_AXIS = 11
    GYRO_X_AXIS = 12
    REAR_LEFT_WHEEL_TICKS = 8
    REAR_RIGHT_WHEEL_TICKS = 9
    TICK_TICK_TEMP = 15  # Temperature of tick sensor


class EsfSensorStatus(IntEnum):
    """Sensor Status for ESF-STATUS."""

    DATA_MISSING = 0
    INITIALIZING = 1
    CALIBRATING = 2
    DATA_OK = 3


@dataclass
class NavSatSv:
    """Single Satellite Information (Part of NAV-SAT)."""

    #: GNSS identifier
    gnss_id: int
    #: Satellite identifier
    sv_id: int
    #: Carrier to Noise Ratio (Signal Strength) [dBHz]
    cno: int
    #: Elevation (range +/- 90) [deg]
    elev: int
    #: Azimuth (range 0-360) [deg]
    azim: int
    #: Pseudo range residual [m] (scaled by 0.1)
    pr_res: float
    #: Flags (Bitmask)
    flags: int


@dataclass
class EsfSensorInfo:
    """Single Sensor Information (Part of ESF-STATUS)."""

    #: Sensor Type (Accelerometer, Gyro, etc.)
    type: EsfSensorType
    #: True if the sensor is used
    used: bool
    #: True if the sensor is ready
    ready: bool
    #: Calibration status
    calib_status: EsfSensorStatus
    #: Time tag status
    time_status: int
    #: Frequency of data [Hz]
    freq: int
    #: Faults bad measurements
    faults_bad_meas: bool
    #: Faults bad time tag
    faults_bad_tag: bool
    #: Faults missing measurements
    faults_missing: bool
    #: Faults noise
    faults_noise: bool
