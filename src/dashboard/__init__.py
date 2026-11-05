"""
Local dashboard served by the windbot itself.

It shows the same wind data as the online dashboard without a trip through the
internet.
"""

import logging
import threading
from collections.abc import Callable

from aggregate import Second
from config import NetworkConfig

from . import history, server

logger = logging.getLogger(__name__)


def init(network_config: NetworkConfig) -> Callable[[Second], None]:
    """Start the dashboard webserver and return where the seconds go."""
    measurements = history.MeasurementHistory()

    try:
        dashboard_server = server.DashboardServer(
            ("", network_config.WEBSERVER_PORT),
            measurements,
        )
    except OSError:
        logger.exception(
            "Could not start the dashboard on port %d",
            network_config.WEBSERVER_PORT,
        )
        return measurements.add_second
    threading.Thread(target=server.serve, args=(dashboard_server,), daemon=True).start()
    return measurements.add_second
