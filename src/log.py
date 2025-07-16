import logging
import logging.handlers
from typing import Any, Literal

import jsonpickle

from config import Config

DATA_SEPARATOR = " :: "

config = Config()
data_logger: logging.Logger | None = None

if config.LOGGING.DATA_LOG_FILE is not None:
    data_logger = logging.getLogger("data")
    data_logger.setLevel(logging.DEBUG)
    handler = logging.handlers.RotatingFileHandler(
        config.LOGGING.DATA_LOG_FILE,
        encoding="utf-8",
        maxBytes=200 * 1000 * 1000,  # 200 MB
        backupCount=10,
    )
    formatter = logging.Formatter(
        "%(created).3f :: %(message)s",
    )
    handler.setFormatter(formatter)
    data_logger.addHandler(handler)
    data_logger.propagate = False


def data(consumer: Literal["gps", "heading", "wind"], data: Any) -> None:  # noqa: ANN401
    if data_logger is None:
        return
    data_logger.debug("%s%s%s", consumer, DATA_SEPARATOR, jsonpickle.encode(data))
