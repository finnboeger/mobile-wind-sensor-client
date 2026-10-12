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
class FilterConfig:
    #: use the Kalman filter for the boat velocity instead of averaging GPS samples
    ENABLED: bool
    #: typical acceleration of the boat in m/s²
    ACCELERATION_SIGMA: float
    #: time constant in seconds after which an acceleration has decayed to 1/e
    ACCELERATION_TIME_CONSTANT: float


CompassSource = Literal["nmea2000", "magnetometer"]


@dataclass(frozen=True, kw_only=True)
class CompassConfig:
    #: where the heading comes from: an NMEA2000 compass or the local magnetometer
    SOURCE: CompassSource
    #: magnetometer chip, a key of compass.drivers.MAGNETOMETERS
    MAGNETOMETER: str
    #: I²C address of the magnetometer, None for the chip's default
    MAGNETOMETER_ADDRESS: int | None
    #: IMU chip, a key of compass.drivers.IMUS
    IMU: str
    #: I²C address of the IMU, None for the chip's default
    IMU_ADDRESS: int | None
    #: number of the I²C bus, /dev/i2c-<n>
    I2C_BUS: int
    #: sensor axes pointing forward, starboard and down, e.g. "x,-y,-z"
    MAGNETOMETER_AXES: str
    IMU_AXES: str
    #: rate in Hz the sensors are read at
    SAMPLE_RATE_HZ: float
    #: rate in Hz the heading is passed on at
    OUTPUT_RATE_HZ: float
    #: file with the bench calibration written by calibrate_compass.py
    BENCH_CALIBRATION_FILE: str
    #: file the continuous calibration is stored in
    CALIBRATION_FILE: str
    #: learn the boat's iron while underway
    CONTINUOUS_CALIBRATION: bool
    #: learn the heading offset from the course over ground
    GNSS_ALIGNMENT: bool
    #: magnetic declination in degrees, positive east
    DECLINATION_DEGREES: float
    #: send the heading to the NMEA2000 network (PGN 127250)
    SEND_HEADING: bool


def parse_bool(value: str) -> bool:
    return configparser.ConfigParser.BOOLEAN_STATES.get(value.lower(), False)


def parse_compass_source(source: str) -> CompassSource:
    normalized = source.strip().lower()
    if normalized == "magnetometer":
        return "magnetometer"
    if normalized != "nmea2000":
        logger.warning(
            "%s is not a valid compass source. Defaulting to nmea2000.",
            source,
        )
    return "nmea2000"


def parse_address(address: str) -> int:
    return int(address, 0)


class Config:
    """Singleton class to manage configuration settings."""

    _instance: "Config | None" = None

    SENSOR: SensorConfig
    LOGGING: LoggingConfig
    NETWORK: NetworkConfig
    MQTT: MQTTConfig | None
    GPS: GPSConfig
    FILTER: FilterConfig
    COMPASS: CompassConfig

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

        self.COMPASS = CompassConfig(
            SOURCE=get(
                config,
                "COMPASS",
                "SOURCE",
                parse_compass_source,
                "nmea2000",
            ),
            MAGNETOMETER=get(config, "COMPASS", "MAGNETOMETER", str, "mmc5983ma")
            .strip()
            .lower(),
            MAGNETOMETER_ADDRESS=get_optional(
                config,
                "COMPASS",
                "MAGNETOMETER_ADDRESS",
                parse_address,
            ),
            IMU=get(config, "COMPASS", "IMU", str, "ism330dhcx").strip().lower(),
            IMU_ADDRESS=get_optional(config, "COMPASS", "IMU_ADDRESS", parse_address),
            I2C_BUS=get(config, "COMPASS", "I2C_BUS", int, 1),
            MAGNETOMETER_AXES=get(
                config,
                "COMPASS",
                "MAGNETOMETER_AXES",
                str,
                "x,-y,-z",
            ),
            IMU_AXES=get(config, "COMPASS", "IMU_AXES", str, "x,-y,-z"),
            SAMPLE_RATE_HZ=get(config, "COMPASS", "SAMPLE_RATE_HZ", float, 50.0),
            OUTPUT_RATE_HZ=get(config, "COMPASS", "OUTPUT_RATE_HZ", float, 10.0),
            BENCH_CALIBRATION_FILE=get(
                config,
                "COMPASS",
                "BENCH_CALIBRATION_FILE",
                str,
                "compass_bench.json",
            ),
            CALIBRATION_FILE=get(
                config,
                "COMPASS",
                "CALIBRATION_FILE",
                str,
                "compass_calibration.json",
            ),
            CONTINUOUS_CALIBRATION=get(
                config,
                "COMPASS",
                "CONTINUOUS_CALIBRATION",
                parse_bool,
                True,
            ),
            GNSS_ALIGNMENT=get(config, "COMPASS", "GNSS_ALIGNMENT", parse_bool, True),
            DECLINATION_DEGREES=get(
                config,
                "COMPASS",
                "DECLINATION_DEGREES",
                float,
                0.0,
            ),
            SEND_HEADING=get(config, "COMPASS", "SEND_HEADING", parse_bool, True),
        )

    def __repr__(self) -> str:
        return f"Config(SENSOR={self.SENSOR})"
