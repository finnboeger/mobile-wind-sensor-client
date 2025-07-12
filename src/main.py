import logging
import logging.handlers
import threading

import gps
import nmea
import wind
from config import Config


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

    # TODO: forward position to n2k network
    # TODO: init mqtt
    # TODO: potentially init local server

    # Start the worker thread to compute the true wind and send it the the consumers
    worker_thread = threading.Thread(
        target=wind.worker,
        args=(position_queue, heading_queue, wind_queue, []),
    )
    worker_thread.start()
    worker_thread.join()
