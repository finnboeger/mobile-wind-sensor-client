import logging
import logging.handlers
import threading
from queue import Queue

import gps
import mqtt
import nmea
import wind
from config import Config
from log import DATA_SEPARATOR
from structs import PositionQueue, WindOutputQueue


def init_logging() -> None:
    # Begin initializing logging
    root_logger = logging.getLogger()
    logger = logging.getLogger(__name__)

    formatter = logging.Formatter(
        "%(asctime)s,%(msecs).03d"
        + DATA_SEPARATOR
        + "%(levelname)s"
        + DATA_SEPARATOR
        + "%(name)s"
        + DATA_SEPARATOR
        + "%(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    startup_handler = logging.StreamHandler()
    startup_handler.setFormatter(formatter)
    root_logger.addHandler(startup_handler)
    root_logger.setLevel(logging.INFO)

    # Load config
    config = Config()

    # Finish initializing logging
    logger.info("Setting log level to %d", config.LOGGING.LOG_LEVEL)
    root_logger.setLevel(config.LOGGING.LOG_LEVEL)

    if config.LOGGING.LOG_FILE is not None:
        logger.debug("Replacing stderr handler with file handler")
        handler = logging.handlers.RotatingFileHandler(
            config.LOGGING.LOG_FILE,
            encoding="utf-8",
            maxBytes=config.LOGGING.MAX_LOG_SIZE * 1000,
            backupCount=config.LOGGING.MAX_LOG_FILES,
        )
        handler.setFormatter(formatter)
        root_logger.removeHandler(startup_handler)
        root_logger.addHandler(handler)


if __name__ == "__main__":
    init_logging()

    nmea_position_queue: PositionQueue = Queue()
    wind_position_queue: PositionQueue = Queue()
    position_output_queues: list[PositionQueue] = [
        nmea_position_queue,
        wind_position_queue,
    ]

    nmea_wind_queue: WindOutputQueue = Queue()
    wind_output_queues: list[WindOutputQueue] = [nmea_wind_queue]

    if Config().MQTT is not None:
        mqtt_position_queue: PositionQueue = Queue()
        mqtt_wind_queue: WindOutputQueue = Queue()
        position_output_queues.append(mqtt_position_queue)
        wind_output_queues.append(mqtt_wind_queue)

        mqtt.init(mqtt_position_queue, mqtt_wind_queue)

    gps.init(position_output_queues)
    n2k_node, heading_queue, wind_queue = nmea.init()

    # Forward the GPS data to the NMEA2000 network
    threading.Thread(
        target=nmea.forward_position,
        args=(n2k_node, nmea_position_queue),
        daemon=True,
    ).start()

    # TODO: potentially init local server
    # Start the worker thread to compute the true wind and send it the the consumers
    worker_thread = threading.Thread(
        target=wind.worker,
        args=(
            wind_position_queue,
            heading_queue,
            wind_queue,
            wind_output_queues,
        ),
    )
    worker_thread.start()

    # Forward the true and corrected apparent wind data to the NMEA2000 network
    forward_wind_to_nmea_thread = threading.Thread(
        target=nmea.forward_wind,
        args=(n2k_node, nmea_wind_queue),
        daemon=True,
    )
    forward_wind_to_nmea_thread.start()

    worker_thread.join()
