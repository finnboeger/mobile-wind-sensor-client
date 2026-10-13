import time
from dataclasses import replace

import pytest

from config import NetworkConfig
from dashboard.wifi import (
    HOTSPOT_CONNECTION,
    WifiError,
    WifiManager,
    parse_properties,
    split_terse,
)

CONFIG = NetworkConfig(
    INTERFACE_PRIORITY=[],
    ENABLE_LOCAL_WEBSERVER=True,
    WEBSERVER_PORT=0,
    ADMIN_PASSWORD=None,
    ENABLE_HOTSPOT=True,
    WIFI_INTERFACE="wlan0",
    AP_SSID="WindSensor",
    AP_PASSWORD="WindSensor",  # noqa: S106
    AP_STARTUP_DELAY_SECONDS=60,
    AP_FALLBACK_DELAY_SECONDS=180,
)


class FakeNetworkManager:
    """Answers nmcli commands like NetworkManager with one Wi-Fi interface."""

    def __init__(self) -> None:
        self.state = 30
        self.connection = ""
        self.connections = {
            "u-home": {"name": "Home", "ssid": "Home", "priority": 5},
            "u-marina": {"name": "Marina", "ssid": "Marina", "priority": 0},
        }
        self.in_reach = ["Marina", "Elsewhere"]
        self.fail_connect: set[str] = set()
        self.commands: list[list[str]] = []

    def __call__(self, args: list[str], _timeout: float) -> str:  # noqa: C901, PLR0911
        self.commands.append(args)
        command = " ".join(args)
        if "device show" in command:
            return (
                f"GENERAL.STATE:{self.state} (state)\n"
                f"GENERAL.CONNECTION:{self.connection}\n"
                "IP4.ADDRESS[1]:192.168.1.20/24\n"
            )
        if "device wifi list" in command:
            return "".join(
                f"{'*' if ssid == self.connection else ' '}:{ssid}:50:WPA2\n"
                for ssid in self.in_reach
            )
        if args[:3] == [
            "-t",
            "-f",
            "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY,ACTIVE",
        ]:
            rows = [
                f"{c['name']}:{uuid}:802-11-wireless:yes:{c['priority']}:no\n"
                for uuid, c in self.connections.items()
            ]
            return "".join(rows) + "Wired:u-eth:802-3-ethernet:yes:0:no\n"
        if "802-11-wireless.ssid" in command:
            connection = self.connections[args[-1]]
            return (
                f"802-11-wireless.ssid:{connection['ssid']}\n"
                f"802-11-wireless.mode:{connection.get('mode', 'infrastructure')}\n"
            )
        if args[:3] == ["connection", "up", HOTSPOT_CONNECTION]:
            self.state, self.connection = 100, HOTSPOT_CONNECTION
            return ""
        if args[:2] == ["connection", "down"]:
            self.state, self.connection = 30, ""
            return ""
        if args[3:5] == ["up", "uuid"]:
            uuid = args[5]
            if uuid in self.fail_connect:
                msg = "Connection activation failed"
                raise WifiError(msg)
            self.state, self.connection = 100, self.connections[uuid]["name"]
            return ""
        if args[:2] == ["connection", "add"] and "con-name" in args:
            name = args[args.index("con-name") + 1]
            if name != HOTSPOT_CONNECTION:
                self.connections[f"u-{name}"] = {
                    "name": name,
                    "ssid": args[args.index("ssid") + 1],
                    "priority": int(
                        args[args.index("connection.autoconnect-priority") + 1],
                    ),
                }
            return ""
        if args[:3] == ["connection", "delete", "uuid"]:
            del self.connections[args[3]]
        return ""


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


def wait_for_action(manager: WifiManager) -> None:
    for _ in range(200):
        if manager.status()["action"] is None:
            return
        time.sleep(0.01)
    pytest.fail("Wi-Fi action did not finish")


def test_split_terse_unescapes_colons() -> None:
    assert split_terse(r"*:My\:Net:70:WPA2") == ["*", "My:Net", "70", "WPA2"]
    assert split_terse(r"a\\b:c") == ["a\\b", "c"]


def test_parse_properties() -> None:
    output = "GENERAL.STATE:100 (connected)\nGENERAL.CONNECTION:My\\:Net\n"
    assert parse_properties(output) == {
        "GENERAL.STATE": "100 (connected)",
        "GENERAL.CONNECTION": "My:Net",
    }


def test_hotspot_opens_after_startup_delay() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)

    manager.tick()
    assert manager.status()["mode"] == "searching"
    assert manager.status()["hotspot_in"] == 60

    clock.now += 59
    manager.tick()
    assert nm.connection == ""

    clock.now += 2
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION
    manager.tick()
    status = manager.status()
    assert status["mode"] == "hotspot"
    assert status["hotspot_in"] is None
    hotspot = next(c for c in nm.commands if c[:2] == ["connection", "add"])
    assert "ap" in hotspot
    assert "shared" in hotspot
    assert hotspot[hotspot.index("wifi-sec.psk") + 1] == "WindSensor"


def test_activating_connection_delays_the_hotspot() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    nm.state = 50  # connecting
    clock.now += 70
    manager.tick()
    assert nm.connection == ""
    clock.now += 60
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION


def test_dropped_network_waits_for_fallback_delay() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, "Home"
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    assert manager.status()["mode"] == "client"
    assert manager.status()["ssid"] == "Home"

    nm.state, nm.connection = 30, ""
    manager.tick()
    assert manager.status()["hotspot_in"] == 180
    clock.now += 181
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION


def test_hotspot_disabled() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(replace(CONFIG, ENABLE_HOTSPOT=False), run=nm, clock=clock)
    manager.tick()
    clock.now += 1000
    manager.tick()
    assert nm.connection == ""
    assert manager.status()["hotspot_in"] is None


def test_stop_hotspot_joins_known_network_in_reach() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()

    manager.stop_hotspot()
    wait_for_action(manager)
    assert nm.connection == "Marina"
    assert manager.status()["message"] == "Connected to Marina."


def test_stop_hotspot_without_network_reopens_after_fallback() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    nm.in_reach = ["Elsewhere"]
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()

    manager.stop_hotspot()
    wait_for_action(manager)
    assert nm.connection == ""
    manager.tick()
    assert manager.status()["hotspot_in"] == 180
    clock.now += 181
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION


def test_failed_connect_from_hotspot_reopens_it() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    nm.fail_connect = {"u-home"}
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()

    manager.connect("u-home")
    wait_for_action(manager)
    assert nm.connection == HOTSPOT_CONNECTION
    assert "Could not connect to Home" in manager.status()["error"]


def test_connect_unknown_network() -> None:
    manager = WifiManager(CONFIG, run=FakeNetworkManager(), clock=Clock())
    with pytest.raises(WifiError):
        manager.connect("nope")


def test_known_networks_skip_hotspot_and_other_types() -> None:
    nm = FakeNetworkManager()
    nm.connections["u-ap"] = {
        "name": "other-ap",
        "ssid": "x",
        "priority": 0,
        "mode": "ap",
    }
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    assert [n["ssid"] for n in manager.known_networks()] == ["Home", "Marina"]


def test_save_network_adds_and_validates() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.save_network("Club", "secret123", 3)
    assert {"name": "Club", "ssid": "Club", "priority": 3} in nm.connections.values()

    with pytest.raises(WifiError):
        manager.save_network("Club", "short")
    with pytest.raises(WifiError):
        manager.save_network("", "secret123")


def test_save_network_updates_existing_ssid() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.save_network("Home", None, 9)
    modify = nm.commands[-1]
    assert modify[:4] == ["connection", "modify", "uuid", "u-home"]
    assert "wifi-sec.psk" not in modify
    assert modify[modify.index("connection.autoconnect-priority") + 1] == "9"


def test_delete_network() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.delete_network("u-marina")
    assert "u-marina" not in nm.connections
    with pytest.raises(WifiError):
        manager.delete_network("u-marina")
