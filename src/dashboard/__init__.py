"""
Local dashboard served by the windbot itself.

It shows the same wind data as the online dashboard without a trip through the
internet.
"""

import threading
from collections.abc import Callable

from aggregate import Second
from config import NetworkConfig

from . import history, server


def init(network_config: NetworkConfig) -> Callable[[Second], None]:
    """Start the dashboard webserver and return where the seconds go."""
    measurements = history.MeasurementHistory()

    def make_server(address: tuple[str, int], interface: str) -> server.DashboardServer:
        return server.DashboardServer(address, measurements, interface)

    # lo for e.g. curl on the Pi itself, whose own connections arrive on it
    for interface in ("lo", network_config.WIFI_INTERFACE):
        watcher = server.InterfaceServer(
            interface,
            network_config.WEBSERVER_PORT,
            make_server,
        )
        threading.Thread(target=watcher.run, daemon=True).start()
    return measurements.add_second
