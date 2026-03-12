import logging
import logging.handlers
from pathlib import Path
from typing import Any, Literal

import jsonpickle

from config import Config

DATA_SEPARATOR = " :: "

config = Config()

DataConsumer = Literal[
    "gps",
    "position",
    "heading",
    "wind",
    "true_wind",
]

_DATA_FORMATTER = logging.Formatter("%(created).3f :: %(message)s")
_data_loggers: dict[str, logging.Logger] = {}


def _build_rotating_handler(file_path: str) -> logging.Handler:
    handler = logging.handlers.RotatingFileHandler(
        file_path,
        encoding="utf-8",
        maxBytes=config.LOGGING.MAX_DATA_LOG_SIZE * 1000,
        backupCount=config.LOGGING.MAX_LOG_FILES,
    )
    handler.setFormatter(_DATA_FORMATTER)
    return handler


def _get_data_logger(consumer: str) -> logging.Logger | None:
    """Return a logger for this consumer, or None if data logging is disabled."""
    if config.LOGGING.DATA_LOG_DIR is None:
        return None

    log_dir = Path(config.LOGGING.DATA_LOG_DIR)
    log_dir.mkdir(parents=True, exist_ok=True)
    file_path = str(log_dir / f"{consumer}.log")
    logger_name = f"data.{consumer}"

    existing = _data_loggers.get(logger_name)
    if existing is not None:
        return existing

    data_logger = logging.getLogger(logger_name)
    data_logger.setLevel(logging.DEBUG)

    data_logger.addHandler(_build_rotating_handler(file_path))

    data_logger.propagate = False
    _data_loggers[logger_name] = data_logger
    return data_logger


def data(consumer: DataConsumer, data: Any) -> None:  # noqa: ANN401
    data_logger = _get_data_logger(consumer)
    if data_logger is None:
        return
    data_logger.debug(jsonpickle.encode(data))
