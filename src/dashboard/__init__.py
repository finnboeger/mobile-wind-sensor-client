"""
Local dashboard served by the windbot itself.

It shows the same wind data as the online dashboard without a trip through the
internet, live values for debugging, and lets the Wi-Fi be managed. When the Wi-Fi
isn't connected to a known network, an access point is opened to reach it.
"""

import logging
import threading

from config import Config
from structs import PositionQueue, WindOutputQueue

from . import history, server, wifi

logger = logging.getLogger(__name__)


def init(position_queue: PositionQueue, wind_queue: WindOutputQueue) -> None:
    """Start the Wi-Fi supervision and the dashboard webserver."""
    network_config = Config().NETWORK

    wifi_manager = wifi.init(network_config)

    measurements = history.MeasurementHistory()
    threading.Thread(
        target=history.position_worker,
        args=(measurements, position_queue),
        daemon=True,
    ).start()
    threading.Thread(
        target=history.wind_worker,
        args=(measurements, wind_queue),
        daemon=True,
    ).start()

    try:
        dashboard_server = server.DashboardServer(
            ("", network_config.WEBSERVER_PORT),
            measurements,
            wifi_manager,
            network_config.ADMIN_PASSWORD,
        )
    except OSError:
        logger.exception(
            "Could not start the dashboard on port %d",
            network_config.WEBSERVER_PORT,
        )
        return
    threading.Thread(target=server.serve, args=(dashboard_server,), daemon=True).start()
