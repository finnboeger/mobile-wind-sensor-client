import logging
import logging.handlers
import threading
from queue import Queue

import gps
import nmea
import wind
from config import Config
from structs import CorrectedApparentWindData, PositionData, TrueWindData


def init_logging() -> None:
    # Begin initializing logging
    root_logger = logging.getLogger()
    logger = logging.getLogger(__name__)

    formatter = logging.Formatter(
        "%(asctime)s,%(msecs).03d :: %(levelname)s :: %(name)s ::  %(message)s",
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

    position_queue = gps.init()
    n2k_node, heading_queue, wind_queue = nmea.init()

    position_queue2: Queue[PositionData] = Queue()

    # Forward the GPS data to the NMEA2000 network
    threading.Thread(
        target=nmea.forward_position,
        args=(n2k_node, position_queue, position_queue2),
        daemon=True,
    ).start()

    # TODO: init mqtt
    # TODO: potentially init local server

    forward_wind_to_nmea_queue: Queue[
        tuple[CorrectedApparentWindData, TrueWindData]
    ] = Queue()

    # Start the worker thread to compute the true wind and send it the the consumers
    worker_thread = threading.Thread(
        target=wind.worker,
        args=(position_queue2, heading_queue, wind_queue, [forward_wind_to_nmea_queue]),
    )
    worker_thread.start()

    # Forward the true and corrected apparent wind data to the NMEA2000 network
    forward_wind_to_nmea_thread = threading.Thread(
        target=nmea.forward_wind,
        args=(n2k_node, forward_wind_to_nmea_queue),
        daemon=True,
    )
    forward_wind_to_nmea_thread.start()

    worker_thread.join()
