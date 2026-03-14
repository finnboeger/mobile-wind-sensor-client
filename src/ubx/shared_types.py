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
    """
    Sensor Types for ESF-STATUS / ESF-MEAS.

    IDs and units based on the ZED-F9R ESF sensor data type table.
    (https://content.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf)
    """

    NONE = 0
    # Gyroscope angular rate [deg/s * 2^-12] (signed, 24-bit payload)
    GYRO_Z_AXIS_ANGULAR_RATE = 5
    GYRO_Y_AXIS_ANGULAR_RATE = 13
    GYRO_X_AXIS_ANGULAR_RATE = 14

    # Gyroscope temperature [°C * 1e-2] (signed, 24-bit payload)
    GYRO_TEMPERATURE = 12

    REAR_LEFT_WHEEL_TICKS = 8
    REAR_RIGHT_WHEEL_TICKS = 9

    # Single tick (speed tick): bits 0-22 unsigned tick value, bit 23 direction
    SPEED_TICK = 10
    # Speed [m/s * 1e-3] (signed, 24-bit payload)
    SPEED = 11

    # Accelerometer specific force [m/s^2 * 2^-10] (signed, 24-bit payload)
    ACCEL_X_AXIS = 16
    ACCEL_Y_AXIS = 17
    ACCEL_Z_AXIS = 18


class EsfSensorInitializationStatus(IntEnum):
    """Sensor Initialization Status for ESF-STATUS."""

    OFF = 0
    INITIALIZING = 1
    INITIALIZED = 2
    INITIALIZED_3 = 3


class EsfSensorCalibrationStatus(IntEnum):
    """Sensor Calibration Status for ESF-STATUS."""

    NOT_CALIBRATED = 0
    CALIBRATING = 1
    CALIBRATED = 2
    CALIBRATED_3 = 3


class EsfSensorTimeStatus(IntEnum):
    """Sensor Time Tag Status for ESF-STATUS."""

    NO_DATA = 0
    FIRST_BYTE_RECEPTION_USED = 1
    EVENT_INPUT_USED = 2
    TIME_TAG_PROVIDED = 3


class EsfFusionMode(IntEnum):
    """Fusion Mode for ESF-STATUS."""

    #: receiver is initializing some unknown values required for doing sensor fusion
    INITIALIZATION = 0
    #: GNSS and sensor data are used for navigation solution computation
    FUSION = 1
    #: sensor fusion is temporarily disabled due to
    #: e.g. invalid sensor data or detected ferry
    SUSPENDED_FUSION = 2
    #: sensor fusion is permanently disabled until
    #: receiver reset due e.g. to sensor error
    DISABLED = 3


class IMUMountAlignmentStatus(IntEnum):
    """IMU Mount Alignment Status for ESF-ALG."""

    #: user-defined/fixed angles are used
    FIXED = 0
    #: IMU-mount roll/pitch angles alignment is ongoing
    ROLL_PITCH_ALIGNMENT_ONGOING = 1
    #: IMU-mount roll/pitch/yaw angles alignment is ongoing
    ROLL_PITCH_YAW_ALIGNMENT_ONGOING = 2
    #: coarse IMU-mount alignment are used
    COARSE = 3
    #: fine IMU-mount alignment are used
    FINE = 4


class PowerSaveModeState(IntEnum):
    """Power Save Mode State."""

    PSM_NOT_ACTIVE = 0
    ENABLED = 1
    ACQUISITION = 2
    TRACKING = 3
    POWER_OPTIMIZED_TRACKING = 4
    INACTIVE = 5


class CarrierPhaseSolution(IntEnum):
    """Carrier Phase Solution Status."""

    NO_SOLUTION = 0
    FLOATING_AMBIGUITIES = 1
    FIXED_AMBIGUITIES = 2


class LastCorrectionAge(IntEnum):
    """Age of the most recently received differential correction."""

    NOT_AVAILABLE = 0
    AGE_0_TO_1_SECONDS = 1
    AGE_1_TO_2_SECONDS = 2
    AGE_2_TO_5_SECONDS = 3
    AGE_5_TO_10_SECONDS = 4
    AGE_10_TO_15_SECONDS = 5
    AGE_15_TO_20_SECONDS = 6
    AGE_20_TO_30_SECONDS = 7
    AGE_30_TO_45_SECONDS = 8
    AGE_45_TO_60_SECONDS = 9
    AGE_60_TO_90_SECONDS = 10
    AGE_90_TO_120_SECONDS = 11
    AGE_120_SECONDS_OR_MORE = 12


class SignalQualityIndicator(IntEnum):
    """Signal Quality Indicator for NAV-SAT."""

    NO_SIGNAL = 0
    SEARCHING_SIGNAL = 1
    SIGNAL_ACQUIRED = 2
    SIGNAL_DETECTED_BUT_UNUSABLE = 3
    CODE_LOCKED_AND_TIME_SYNCHRONIZED = 4
    # values 5-7 indicate the same state
    CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED = 5
    CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_6 = 6
    CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_7 = 7


class SignalHealth(IntEnum):
    """Signal Health for NAV-SAT."""

    UNKNOWN = 0
    HEALTHY = 1
    UNHEALTHY = 2


class OrbitSource(IntEnum):
    """Orbit Source for NAV-SAT."""

    UNKNOWN = 0
    EPHEMERIS = 1
    ALMANAC = 2
    ASSIST_NOW_OFFLINE = 3
    ASSIST_NOW_AUTONOMOUS = 4
    # values 5-7 indicate the same state
    OTHER = 5
    OTHER_6 = 6
    OTHER_7 = 7


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
    quality_ind: SignalQualityIndicator
    sv_used: bool
    health: SignalHealth
    differential_correction_data_available: bool
    smoothed: bool
    orbit_source: OrbitSource
    ephemeris_available: bool
    almanac_available: bool
    assist_now_offline_available: bool
    assist_now_autonomous_available: bool
    sbas_correction_used: bool
    rtcm_correction_used: bool
    slas_correction_used: bool
    spartn_correction_used: bool
    pseudorange_correction_used: bool
    carrier_range_correction_used: bool
    doppler_correction_used: bool
    clas_correction_used: bool


@dataclass
class EsfSensorInfo:
    """Single Sensor Information (Part of ESF-STATUS)."""

    #: Sensor Type (Accelerometer, Gyro, etc.)
    type: EsfSensorType
    #: Sensor is used for the current sensor fusion solution
    used: bool
    #: Sensor is ready but not used for the current sensor fusion solution
    ready: bool
    #: Calibration status
    calib_status: EsfSensorCalibrationStatus
    #: Time tag status
    time_status: EsfSensorTimeStatus
    #: Frequency of data [Hz]
    freq: int
    #: Bad measurements detected
    bad_measurements: bool
    #: Bad measurement time-tags detected
    bad_time_tag: bool
    #: Missing or time-misaligned measurements detected
    missing_measurements: bool
    #: High measurement noise-level detected
    noisy_measurements: bool
