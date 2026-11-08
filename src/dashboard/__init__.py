"""
Local dashboard served by the windbot itself.

It shows the same wind data as the online dashboard without a trip through the
internet, live values for debugging, and lets the Wi-Fi be managed. When the Wi-Fi
isn't connected to a known network, an access point is opened to reach it.
"""

import threading
from collections.abc import Callable

from aggregate import Second
from config import Config, NetworkConfig

from . import history, internet, server, wifi


def init(network_config: NetworkConfig) -> Callable[[Second], None]:
    """Start the Wi-Fi supervision and the webserver, return where the seconds go."""
    wifi_manager = wifi.init(network_config)

    measurements = history.MeasurementHistory()
    mqtt_config = Config().MQTT
    # shared by the servers on every interface, so they reuse one check's result
    internet_check = internet.InternetCheck(
        (mqtt_config.BROKER, mqtt_config.PORT) if mqtt_config else None,
    )

    def make_server(address: tuple[str, int], interface: str) -> server.DashboardServer:
        return server.DashboardServer(
            address,
            measurements,
            wifi_manager,
            network_config.ADMIN_PASSWORD,
            interface,
            internet=internet_check,
        )

    # lo for e.g. curl on the Pi itself, whose own connections arrive on it
    for interface in ("lo", network_config.WIFI_INTERFACE):
        watcher = server.InterfaceServer(
            interface,
            network_config.WEBSERVER_PORT,
            make_server,
        )
        threading.Thread(target=watcher.run, daemon=True).start()
    return measurements.add_second
