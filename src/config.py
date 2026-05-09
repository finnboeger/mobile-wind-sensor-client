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
    CLIENT_ID: str
    TOPIC: str
    QUEUE_SIZE: int


GPSPort = Literal["I2C", "UART1", "UART2", "USB", "SPI"]


@dataclass(frozen=True, kw_only=True)
class GPSConfig:
    SERIAL_PORT: str
    PORTS: list[GPSPort]


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
            SERIAL_PORT=get(config, "GPS", "SERIAL_PORT", str, "/dev/ttyUSB0"),
            PORTS=[
                port
                for port in (
                    x.strip()
                    for x in get(config, "GPS", "PORTS", str, "USB").split(",")
                )
                if port in ports
            ],
        )

    def __repr__(self) -> str:
        return f"Config(SENSOR={self.SENSOR})"
