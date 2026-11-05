import time

import pytest

from config import NetworkConfig
from dashboard.wifi import (
    WifiBusyError,
    WifiError,
    WifiManager,
    WifiRequestError,
    parse_properties,
    split_terse,
)

CONFIG = NetworkConfig(
    INTERFACE_PRIORITY=[],
    ENABLE_LOCAL_WEBSERVER=True,
    WEBSERVER_PORT=0,
    ADMIN_PASSWORD=None,
    WIFI_INTERFACE="wlan0",
    AP_SSID=None,
    AP_PASSWORD=None,
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

    def __call__(self, args: list[str], _timeout: float) -> str:
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
            uuids = args[args.index("show") + 2 :: 2]
            connections = [(uuid, self.connections[uuid]) for uuid in uuids]
            return "".join(
                f"connection.uuid:{uuid}\n"
                f"802-11-wireless.ssid:{c['ssid']}\n"
                f"802-11-wireless.mode:{c.get('mode', 'infrastructure')}\n"
                for uuid, c in connections
            )
        if args[3:5] == ["up", "uuid"]:
            uuid = args[5]
            if uuid in self.fail_connect:
                msg = "Connection activation failed"
                raise WifiError(msg)
            self.state, self.connection = 100, self.connections[uuid]["name"]
            return ""
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


def test_tick_reports_the_mode() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.tick()
    assert manager.status()["mode"] == "searching"

    nm.state, nm.connection = 100, "Marina"
    manager.tick()
    status = manager.status()
    assert status["mode"] == "client"
    assert status["ssid"] == "Marina"
    assert status["address"] == "192.168.1.20"
    assert status["signal"] == 50

    for state in (10, 20):  # 20: e.g. the Wi-Fi switched off
        nm.state = state
        manager.tick()
        assert manager.status()["mode"] == "unavailable"


def test_status_reads_the_state_only_when_asked() -> None:
    nm = FakeNetworkManager()
    clock = Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    assert nm.commands == []
    assert manager.status()["mode"] == "searching"
    reads = sum("device show" in " ".join(c) for c in nm.commands)
    assert reads == 1
    manager.status()  # cached
    assert sum("device show" in " ".join(c) for c in nm.commands) == reads
    clock.now += 5
    manager.status()
    assert sum("device show" in " ".join(c) for c in nm.commands) == reads + 1


def test_known_networks_skip_access_points_and_other_types() -> None:
    nm = FakeNetworkManager()
    nm.connections["u-ap"] = {
        "name": "other-ap",
        "ssid": "x",
        "priority": 0,
        "mode": "ap",
    }
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    assert [n["ssid"] for n in manager.known_networks()] == ["Home", "Marina"]
    # one call for the list, one for the details of all of them
    assert len(nm.commands) == 2


def test_connect_to_known_network() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.connect("u-marina")
    wait_for_action(manager)
    status = manager.status()
    assert status["message"] == "Connected to Marina."
    assert status["mode"] == "client"
    assert status["ssid"] == "Marina"


def test_failed_connect_is_reported() -> None:
    nm = FakeNetworkManager()
    nm.fail_connect = {"u-home"}
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.connect("u-home")
    wait_for_action(manager)
    assert "Could not connect to Home" in manager.status()["error"]


def test_connect_unknown_network() -> None:
    manager = WifiManager(CONFIG, run=FakeNetworkManager(), clock=Clock())
    with pytest.raises(WifiRequestError):
        manager.connect("nope")


def test_second_action_is_refused_while_one_runs() -> None:
    manager = WifiManager(CONFIG, run=FakeNetworkManager(), clock=Clock())
    manager._action_lock.acquire()  # noqa: SLF001 - an action in progress
    try:
        with pytest.raises(WifiBusyError):
            manager.connect("u-home")
    finally:
        manager._action_lock.release()  # noqa: SLF001
