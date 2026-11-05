import time
from dataclasses import replace

import pytest

from config import NetworkConfig
from dashboard.wifi import (
    HOTSPOT_CONNECTION,
    WifiBusyError,
    WifiError,
    WifiManager,
    WifiRequestError,
    parse_properties,
    split_terse,
)

CONFIG = NetworkConfig(  # nosec B106
    INTERFACE_PRIORITY=[],
    ENABLE_LOCAL_WEBSERVER=True,
    WEBSERVER_PORT=0,
    ADMIN_PASSWORD=None,
    WIFI_INTERFACE="wlan0",
    AP_SSID="WindSensor",
    AP_PASSWORD="WindSensor",  # noqa: S106
)


@pytest.fixture(autouse=True)
def forwarding_blocked(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(WifiManager, "forwarding_blocked", lambda _self: True)


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
        self.hotspot: list[str] | None = None

    def __call__(self, args: list[str], _timeout: float) -> str:  # noqa: C901, PLR0911
        self.commands.append(args)
        command = " ".join(args)
        if args[0] == "--show-secrets":
            if self.hotspot is None:
                msg = "no such connection profile"
                raise WifiError(msg)
            return "".join(f"{value}\n" for value in self.hotspot)
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
            self._add(args)
            return ""
        if args[:3] == ["connection", "delete", "uuid"]:
            del self.connections[args[3]]
        return ""

    def _add(self, args: list[str]) -> None:
        name = args[args.index("con-name") + 1]
        if name == HOTSPOT_CONNECTION:
            psk = args[args.index("wifi-sec.psk") + 1] if "wifi-sec.psk" in args else ""
            key_mgmt = "wpa-psk" if psk else ""
            self.hotspot = [args[5], args[args.index("ssid") + 1], key_mgmt, psk]
        else:
            self.connections[f"u-{name}"] = {
                "name": name,
                "ssid": args[args.index("ssid") + 1],
                "priority": int(
                    args[args.index("connection.autoconnect-priority") + 1],
                ),
            }


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


def hotspot_adds(nm: FakeNetworkManager) -> int:
    return sum(1 for c in nm.commands if c[:2] == ["connection", "add"])


def test_start_hotspot() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.start_hotspot()
    wait_for_action(manager)
    assert nm.connection == HOTSPOT_CONNECTION
    hotspot = next(c for c in nm.commands if c[:2] == ["connection", "add"])
    assert "ap" in hotspot
    assert "shared" in hotspot
    assert hotspot[hotspot.index("wifi-sec.psk") + 1] == "WindSensor"
    manager.tick()
    status = manager.status()
    assert status["mode"] == "hotspot"
    assert status["ssid"] == "WindSensor"
    assert [n["ssid"] for n in status["networks"]] == ["Home", "Marina"]


def test_hotspot_needs_the_forwarding_block(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(WifiManager, "forwarding_blocked", lambda _self: False)
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.start_hotspot()
    wait_for_action(manager)
    assert nm.connection == ""
    assert "setup-hotspot.sh" in manager.status()["error"]


def test_invalid_hotspot_password_keeps_it_closed() -> None:
    nm = FakeNetworkManager()
    config = replace(CONFIG, AP_PASSWORD="short")  # noqa: S106  # nosec B106
    manager = WifiManager(config, run=nm, clock=Clock())
    manager.start_hotspot()
    wait_for_action(manager)
    assert nm.connection == ""
    assert nm.hotspot is None
    assert "AP_PASSWORD" in manager.status()["error"]


def test_hotspot_profile_is_only_recreated_when_changed() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.start_hotspot()
    wait_for_action(manager)
    manager.start_hotspot()
    wait_for_action(manager)
    assert hotspot_adds(nm) == 1

    changed = WifiManager(
        replace(CONFIG, AP_PASSWORD="new-password"),  # noqa: S106  # nosec B106
        run=nm,
        clock=Clock(),
    )
    changed.start_hotspot()
    wait_for_action(changed)
    assert hotspot_adds(nm) == 2
    assert nm.hotspot is not None
    assert nm.hotspot[3] == "new-password"


def test_stop_hotspot_joins_known_network_in_reach() -> None:
    nm = FakeNetworkManager()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.tick()

    manager.stop_hotspot()
    wait_for_action(manager)
    assert nm.connection == "Marina"
    assert manager.status()["message"] == "Connected to Marina."


def test_stop_hotspot_without_network_reopens_it() -> None:
    nm = FakeNetworkManager()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    nm.in_reach = ["Elsewhere"]
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.tick()

    manager.stop_hotspot()
    wait_for_action(manager)
    assert ["connection", "down", HOTSPOT_CONNECTION] in nm.commands
    assert nm.connection == HOTSPOT_CONNECTION
