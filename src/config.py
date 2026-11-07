import configparser
import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal, TypeVar

logger = logging.getLogger(__name__)


@dataclass(frozen=True, kw_only=True)
class SensorConfig:
    COMPASS_OFFSET_DEGREES: float


@dataclass(frozen=True, kw_only=True)
class LoggingConfig:
    LOG_FILE: str | None
    LOG_LEVEL: int
    MAX_LOG_SIZE: int
    MAX_LOG_FILES: int
    DATA_LOG_DIR: str | None
    MAX_DATA_LOG_SIZE: int


@dataclass(frozen=True, kw_only=True)
class NetworkConfig:
    INTERFACE_PRIORITY: list[str]
    ENABLE_LOCAL_WEBSERVER: bool
    AP_SSID: str | None
    AP_PASSWORD: str | None


@dataclass(frozen=True, kw_only=True)
class MQTTConfig:
    BROKER: str
    PORT: int
    USERNAME: str | None
    PASSWORD: str | None
    USE_TLS: bool
    CLIENT_ID: str
    TOPIC: str
    QUEUE_SIZE: int


GPSPort = Literal["I2C", "UART1", "UART2", "USB", "SPI"]


#: u-blox dynamic platform models (CFG-NAV5 dynModel)
GPS_DYNAMIC_MODELS: dict[str, int] = {
    "portable": 0,
    "stationary": 2,
    "pedestrian": 3,
    "automotive": 4,
    "sea": 5,
    "airborne1g": 6,
    "airborne2g": 7,
    "airborne4g": 8,
    "wrist": 9,
}


@dataclass(frozen=True, kw_only=True)
class GPSConfig:
    SERIAL_PORT: str
    PORTS: list[GPSPort]
    #: name of the dynamic platform model, a key of GPS_DYNAMIC_MODELS
    DYNAMIC_MODEL: str
    #: GNSS navigation rate in Hz
    NAVIGATION_RATE_HZ: float


@dataclass(frozen=True, kw_only=True)
class ImuConfig:
    #: read the IMU
    ENABLED: bool
    #: name of the IMU device, a key of imu.DEVICES
    DEVICE: str
    #: number of the I2C bus, e.g. 1 for /dev/i2c-1
    I2C_BUS: int
    #: I2C address of the IMU
    ADDRESS: int
    #: sensor axes pointing forward, right and down in the unit, e.g. "x,-y,-z"
    AXES: str
    #: sample rate in Hz
    SAMPLE_RATE_HZ: float
    #: standard deviation of a compass reading in degrees, for the gyro aided heading
    COMPASS_SIGMA_DEGREES: float
    #: time constant of the compass damping in seconds, 0 if it doesn't lag
    COMPASS_LAG_SECONDS: float
    #: position of the wind sensor relative to the IMU, forward, right and down in m
    WIND_SENSOR_POSITION: tuple[float, float, float]
    #: position of the GPS antenna relative to the IMU, forward, right and down in m
    GPS_ANTENNA_POSITION: tuple[float, float, float]


@dataclass(frozen=True, kw_only=True)
class MagnetometerConfig:
    #: read the magnetometers
    ENABLED: bool
    #: names of the magnetometers, keys of magnetometer.DEVICES
    DEVICES: list[str]
    #: number of the I2C bus, e.g. 1 for /dev/i2c-1
    I2C_BUS: int
    #: I2C address of each magnetometer, None for the chip's default
    ADDRESSES: list[int | None]
    #: sensor axes pointing forward, right and down in the unit, for each magnetometer
    AXES: list[str]
    #: sample rate in Hz
    SAMPLE_RATE_HZ: float
    #: magnetic declination in degrees, positive when magnetic north is east of true
    DECLINATION_DEGREES: float
    #: bench calibration file of each magnetometer, "{sensor}" is replaced by its name
    BENCH_CALIBRATION_FILE: str
    #: learn the boat's iron while underway
    CONTINUOUS_CALIBRATION: bool
    #: where the continuously learned calibration is kept
    CALIBRATION_FILE: str


@dataclass(frozen=True, kw_only=True)
class FilterConfig:
    #: use the Kalman filter for the boat velocity instead of averaging GPS samples
    ENABLED: bool
    #: typical acceleration of the boat in m/s²
    ACCELERATION_SIGMA: float
    #: time constant in seconds after which an acceleration has decayed to 1/e
    ACCELERATION_TIME_CONSTANT: float


def parse_position(value: str) -> tuple[float, float, float]:
    """Parse a position like ``"0, 0, -0.3"``."""
    entries = [float(entry) for entry in value.split(",")]
    if len(entries) != 3:  # noqa: PLR2004
        msg = f"Position {value!r} needs three comma separated entries"
        raise ValueError(msg)
    return (entries[0], entries[1], entries[2])


def parse_list(value: str, separator: str = ",") -> list[str]:
    """Parse a list like ``"rm3100, mmc5983ma"``, dropping empty entries."""
    return [entry.strip() for entry in value.split(separator) if entry.strip()]


def per_device(values: list[str], count: int, name: str) -> list[str]:
    """
    Return one setting for each of ``count`` devices.

    :param values: either a single value used for all devices, or one per device
    :param name: name of the setting, for the error message
    """
    if len(values) == 1:
        return values * count
    if len(values) != count:
        msg = f"{name} needs one entry, or one for each of the {count} devices"
        raise ValueError(msg)
    return values


def parse_bool(value: str) -> bool:
    return configparser.ConfigParser.BOOLEAN_STATES.get(value.lower(), False)


class Config:
    """Singleton class to manage configuration settings."""

    _instance: "Config | None" = None

    SENSOR: SensorConfig
    LOGGING: LoggingConfig
    NETWORK: NetworkConfig
    MQTT: MQTTConfig | None
    GPS: GPSConfig
    FILTER: FilterConfig
    IMU: ImuConfig
    MAGNETOMETER: MagnetometerConfig

    def __new__(cls) -> "Config":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.update()
        return cls._instance

    def update(self) -> None:
        """Update the configuration from the config file."""
        config = configparser.ConfigParser()
        config.read("config.ini")

        T = TypeVar("T")

        def get(
            config: configparser.ConfigParser,
            section: str,
            option: str,
            parser: Callable[[str], T],
            fallback: T | None = None,
        ) -> T:
            """Get a configuration value with a fallback."""
            try:
                return parser(config.get(section, option))
            except (configparser.NoSectionError, configparser.NoOptionError) as e:
                if fallback is None:
                    error_msg = f"Configuration option {section}.{option} is missing."
                    raise ValueError(
                        error_msg,
                    ) from e
                logger.info(
                    "%s.%s not found in config file. Defaulting to %s.",
                    section,
                    option,
                    str(fallback),
                )
                return fallback

        def get_optional(
            config: configparser.ConfigParser,
            section: str,
            option: str,
            parser: Callable[[str], T],
        ) -> T | None:
            """Get an optional configuration value, returning None if it's not found."""
            try:
                return parser(config.get(section, option))
            except (configparser.NoSectionError, configparser.NoOptionError):
                logger.info(
                    "%s.%s not found in config file. Defaulting to None.",
                    section,
                    option,
                )
                return None

        def parse_log_level(level: str) -> int:
            mapping = logging._nameToLevel  # noqa: SLF001
            if level in mapping:
                return mapping[level]
            logger.warning("%s is not a valid log level. Defaulting to WARNING.", level)
            return logging.WARNING

        def parse_dynamic_model(model: str) -> str:
            normalized = model.strip().lower()
            if normalized in GPS_DYNAMIC_MODELS:
                return normalized
            logger.warning("%s is not a valid dynamic model. Defaulting to sea.", model)
            return "sea"

        self.SENSOR = SensorConfig(
            COMPASS_OFFSET_DEGREES=get(
                config,
                "SENSOR",
                "COMPASS_OFFSET_DEGREES",
                float,
            ),
        )
        self.LOGGING = LoggingConfig(
            LOG_FILE=get_optional(config, "LOGGING", "LOG_FILE", str),
            LOG_LEVEL=get(
                config,
                "LOGGING",
                "LOG_LEVEL",
                parse_log_level,
                logging.WARNING,
            ),
            MAX_LOG_SIZE=get(config, "LOGGING", "MAX_LOG_SIZE", int, 5000),
            MAX_LOG_FILES=get(config, "LOGGING", "MAX_LOG_FILES", int, 10),
            DATA_LOG_DIR=get_optional(config, "LOGGING", "DATA_LOG_DIR", str),
            MAX_DATA_LOG_SIZE=get(config, "LOGGING", "MAX_DATA_LOG_SIZE", int, 100_000),
        )
        self.NETWORK = NetworkConfig(
            INTERFACE_PRIORITY=[
                interface.strip()
                for interface in get(
                    config,
                    "NETWORK",
                    "INTERFACE_PRIORITY",
                    str,
                    fallback="",
                ).split(",")
                if interface.strip() != ""
            ],
            ENABLE_LOCAL_WEBSERVER=get(
                config,
                "NETWORK",
                "ENABLE_LOCAL_WEBSERVER",
                parse_bool,
                fallback=False,
            ),
            AP_SSID=get_optional(config, "NETWORK", "AP_SSID", str),
            AP_PASSWORD=get_optional(config, "NETWORK", "AP_PASSWORD", str),
        )
        broker = get_optional(config, "MQTT", "BROKER", str)
        self.MQTT = (
            MQTTConfig(
                BROKER=broker,
                PORT=get(config, "MQTT", "PORT", int, 8883),
                USE_TLS=get(config, "MQTT", "USE_TLS", parse_bool, True),
                USERNAME=get_optional(config, "MQTT", "USERNAME", str),
                PASSWORD=get_optional(config, "MQTT", "PASSWORD", str),
                CLIENT_ID=get(config, "MQTT", "CLIENT_ID", str),
                TOPIC=get(config, "MQTT", "TOPIC", str),
                QUEUE_SIZE=get(config, "MQTT", "QUEUE_SIZE", int, 5),
            )
            if broker is not None
            else None
        )
        ports: list[GPSPort] = ["I2C", "UART1", "UART2", "USB", "SPI"]
        self.GPS = GPSConfig(
            SERIAL_PORT=get(config, "GPS", "SERIAL_PORT", str, "/dev/ttyACM0"),
            PORTS=[
                port
                for port in (
                    x.strip()
                    for x in get(config, "GPS", "PORTS", str, "USB").split(",")
                )
                if port in ports
            ],
            DYNAMIC_MODEL=get(
                config,
                "GPS",
                "DYNAMIC_MODEL",
                parse_dynamic_model,
                "sea",
            ),
            NAVIGATION_RATE_HZ=get(config, "GPS", "NAVIGATION_RATE_HZ", float, 2.0),
        )
        self.FILTER = FilterConfig(
            ENABLED=get(config, "FILTER", "ENABLED", parse_bool, True),
            ACCELERATION_SIGMA=get(config, "FILTER", "ACCELERATION_SIGMA", float, 0.3),
            ACCELERATION_TIME_CONSTANT=get(
                config,
                "FILTER",
                "ACCELERATION_TIME_CONSTANT",
                float,
                2.0,
            ),
        )

        self.IMU = ImuConfig(
            ENABLED=get(config, "IMU", "ENABLED", parse_bool, False),
            DEVICE=get(config, "IMU", "DEVICE", str, "ism330dhcx").strip().lower(),
            I2C_BUS=get(config, "IMU", "I2C_BUS", int, 1),
            ADDRESS=get(config, "IMU", "ADDRESS", lambda value: int(value, 0), 0x6A),
            AXES=get(config, "IMU", "AXES", str, "x,-y,-z"),
            SAMPLE_RATE_HZ=get(config, "IMU", "SAMPLE_RATE_HZ", float, 104.0),
            COMPASS_SIGMA_DEGREES=get(
                config,
                "IMU",
                "COMPASS_SIGMA_DEGREES",
                float,
                2.0,
            ),
            COMPASS_LAG_SECONDS=get(config, "IMU", "COMPASS_LAG_SECONDS", float, 1.0),
            WIND_SENSOR_POSITION=get(
                config,
                "IMU",
                "WIND_SENSOR_POSITION",
                parse_position,
                (0.0, 0.0, 0.0),
            ),
            GPS_ANTENNA_POSITION=get(
                config,
                "IMU",
                "GPS_ANTENNA_POSITION",
                parse_position,
                (0.0, 0.0, 0.0),
            ),
        )

        devices = [
            device.lower()
            for device in parse_list(
                get(config, "MAGNETOMETER", "DEVICES", str, "rm3100"),
            )
        ]
        addresses = parse_list(get(config, "MAGNETOMETER", "ADDRESSES", str, "auto"))
        self.MAGNETOMETER = MagnetometerConfig(
            ENABLED=get(config, "MAGNETOMETER", "ENABLED", parse_bool, False),
            DEVICES=devices,
            I2C_BUS=get(config, "MAGNETOMETER", "I2C_BUS", int, 1),
            ADDRESSES=[
                None if address.lower() == "auto" else int(address, 0)
                for address in per_device(addresses, len(devices), "ADDRESSES")
            ],
            AXES=per_device(
                parse_list(get(config, "MAGNETOMETER", "AXES", str, "x,-y,-z"), ";"),
                len(devices),
                "AXES",
            ),
            SAMPLE_RATE_HZ=get(config, "MAGNETOMETER", "SAMPLE_RATE_HZ", float, 20.0),
            DECLINATION_DEGREES=get(
                config,
                "MAGNETOMETER",
                "DECLINATION_DEGREES",
                float,
                0.0,
            ),
            BENCH_CALIBRATION_FILE=get(
                config,
                "MAGNETOMETER",
                "BENCH_CALIBRATION_FILE",
                str,
                "compass_bench_{sensor}.json",
            ),
            CONTINUOUS_CALIBRATION=get(
                config,
                "MAGNETOMETER",
                "CONTINUOUS_CALIBRATION",
                parse_bool,
                True,
            ),
            CALIBRATION_FILE=get(
                config,
                "MAGNETOMETER",
                "CALIBRATION_FILE",
                str,
                "compass_calibration.json",
            ),
        )

    def __repr__(self) -> str:
        return f"Config(SENSOR={self.SENSOR})"
