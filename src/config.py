import configparser
import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal, TypeVar

logger = logging.getLogger(__name__)

MAX_PORT = 65535


def parse_port(value: str, option: str) -> int | None:
    """Return the TCP port in ``value``, or log an error and return None."""
    value = value.strip()
    if not value.isdigit() or not 1 <= int(value) <= MAX_PORT:
        logger.error("%s %r is not a port between 1 and %d", option, value, MAX_PORT)
        return None
    return int(value)


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
    #: TCP port of the local dashboard
    WEBSERVER_PORT: int
    #: password required for changing the Wi-Fi settings on the dashboard
    ADMIN_PASSWORD: str | None
    #: open an access point when the Wi-Fi isn't connected to a network
    ENABLE_HOTSPOT: bool
    #: Wi-Fi interface used for both networks and the access point, and the only
    #: one besides localhost that the dashboard listens on
    WIFI_INTERFACE: str
    AP_SSID: str
    AP_PASSWORD: str | None
    #: seconds after startup to wait for a known network before opening the AP
    AP_STARTUP_DELAY_SECONDS: int
    #: seconds without a network connection before the AP is opened again
    AP_FALLBACK_DELAY_SECONDS: int


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
        webserver_port = parse_port(
            get(config, "NETWORK", "WEBSERVER_PORT", str, "80"),
            "NETWORK.WEBSERVER_PORT",
        )
        # without a valid port the dashboard is disabled instead of crashing on start
        enable_webserver = webserver_port is not None and get(
            config,
            "NETWORK",
            "ENABLE_LOCAL_WEBSERVER",
            parse_bool,
            fallback=True,
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
            ENABLE_LOCAL_WEBSERVER=enable_webserver,
            WEBSERVER_PORT=webserver_port or 80,
            ADMIN_PASSWORD=get_optional(config, "NETWORK", "ADMIN_PASSWORD", str)
            or None,
            ENABLE_HOTSPOT=get(
                config,
                "NETWORK",
                "ENABLE_HOTSPOT",
                parse_bool,
                fallback=True,
            ),
            WIFI_INTERFACE=get(config, "NETWORK", "WIFI_INTERFACE", str, "wlan0"),
            AP_SSID=get(config, "NETWORK", "AP_SSID", str, "WindSensor"),
            AP_PASSWORD=get_optional(config, "NETWORK", "AP_PASSWORD", str),
            AP_STARTUP_DELAY_SECONDS=get(
                config,
                "NETWORK",
                "AP_STARTUP_DELAY_SECONDS",
                int,
                60,
            ),
            AP_FALLBACK_DELAY_SECONDS=get(
                config,
                "NETWORK",
                "AP_FALLBACK_DELAY_SECONDS",
                int,
                180,
            ),
        )
        broker = (get_optional(config, "MQTT", "BROKER", str) or "").strip()
        mqtt_port = get(config, "MQTT", "PORT", str, "8883").strip()
        if not broker:
            logger.info("No MQTT broker configured, MQTT is disabled")
            self.MQTT = None
        elif not mqtt_port.isdigit() or not 1 <= int(mqtt_port) <= MAX_PORT:
            logger.error(
                "MQTT.PORT %r is not a port between 1 and %d, MQTT is disabled",
                mqtt_port,
                MAX_PORT,
            )
            self.MQTT = None
        else:
            self.MQTT = MQTTConfig(
                BROKER=broker,
                PORT=int(mqtt_port),
                USE_TLS=get(config, "MQTT", "USE_TLS", parse_bool, True),
                USERNAME=get_optional(config, "MQTT", "USERNAME", str),
                PASSWORD=get_optional(config, "MQTT", "PASSWORD", str),
                CLIENT_ID=get(config, "MQTT", "CLIENT_ID", str),
                TOPIC=get(config, "MQTT", "TOPIC", str),
                QUEUE_SIZE=get(config, "MQTT", "QUEUE_SIZE", int, 5),
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
        )

    def __repr__(self) -> str:
        return f"Config(SENSOR={self.SENSOR})"
