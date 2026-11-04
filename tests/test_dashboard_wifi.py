import logging
import subprocess  # nosec B404
import time
from dataclasses import replace

import pytest

from config import NetworkConfig
from dashboard import commands
from dashboard.wifi import (
    ACTIVE_CLIENT_SECONDS,
    HOTSPOT_CONNECTION,
    KNOWN_NETWORK_RETRY_SECONDS,
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
    ENABLE_HOTSPOT=True,
    WIFI_INTERFACE="wlan0",
    AP_SSID="WindSensor",
    AP_PASSWORD="WindSensor",  # noqa: S106
    AP_STARTUP_DELAY_SECONDS=60,
    AP_FALLBACK_DELAY_SECONDS=180,
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
        self.fail_device_show = False
        self.fail_rescan = False

    def __call__(self, args: list[str], _timeout: float) -> str:  # noqa: C901, PLR0911, PLR0912
        self.commands.append(args)
        command = " ".join(args)
        if args[0] == "--show-secrets":
            if self.hotspot is None:
                msg = "no such connection profile"
                raise WifiError(msg)
            return "".join(f"{value}\n" for value in self.hotspot)
        if "device show" in command:
            if self.fail_device_show:
                msg = "timed out"
                raise WifiError(msg)
            return (
                f"GENERAL.STATE:{self.state} (state)\n"
                f"GENERAL.CONNECTION:{self.connection}\n"
                "IP4.ADDRESS[1]:192.168.1.20/24\n"
            )
        if "device wifi list" in command:
            if self.fail_rescan and args[-1] == "yes":
                msg = "Scanning not allowed while the interface is busy"
                raise WifiError(msg)
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


def test_tick_reports_client_and_unavailable() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    nm.state, nm.connection = 100, "Marina"
    manager.tick()
    status = manager.status()
    assert status["mode"] == "client"
    assert status["ssid"] == "Marina"
    assert status["signal"] == 50

    for state in (10, 20):  # 20: e.g. the Wi-Fi switched off
        nm.state = state
        manager.tick()
        assert manager.status()["mode"] == "unavailable"


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


def test_wifi_switched_off_restarts_the_delay() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, "Home"
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    nm.state, nm.connection = 30, ""
    manager.tick()

    nm.state = 20  # the Wi-Fi switched off for an hour
    manager.tick()
    clock.now += 3600
    manager.tick()
    nm.state = 30
    manager.tick()
    assert nm.connection == ""
    assert manager.status()["hotspot_in"] == 180


def test_failed_state_read_keeps_the_deadline() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    nm.fail_device_show = True
    manager.tick()
    nm.fail_device_show = False
    clock.now += 61
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION


def test_change_started_during_a_check_skips_it(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    clock.now += 61
    read_state = manager.device_state

    def request_starts_meanwhile() -> object:
        manager._action_lock.acquire()  # noqa: SLF001
        return read_state()

    monkeypatch.setattr(manager, "device_state", request_starts_meanwhile)
    manager.tick()  # doesn't raise
    assert nm.connection == ""


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
    with pytest.raises(WifiRequestError):
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
    # one call for the list, one for the details of all of them
    assert len(nm.commands) == 2


def hotspot_adds(nm: FakeNetworkManager) -> int:
    return sum(1 for c in nm.commands if c[:2] == ["connection", "add"])


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


def test_second_action_is_refused_while_one_runs() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager._action_lock.acquire()  # noqa: SLF001 - an action in progress
    try:
        with pytest.raises(WifiBusyError):
            manager.start_hotspot()
    finally:
        manager._action_lock.release()  # noqa: SLF001


def test_connect_to_known_network() -> None:
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.connect("u-marina")
    wait_for_action(manager)
    status = manager.status()  # without waiting for the next check
    assert status["message"] == "Connected to Marina."
    assert status["mode"] == "client"
    assert status["ssid"] == "Marina"


def test_hotspot_needs_the_forwarding_block(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(WifiManager, "forwarding_blocked", lambda _self: False)
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager.start_hotspot()
    wait_for_action(manager)
    assert nm.connection == ""
    assert "setup-hotspot.sh" in manager.status()["error"]


def test_hotspot_is_closed_when_the_forwarding_block_disappears(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    manager.tick()
    assert nm.connection == HOTSPOT_CONNECTION

    monkeypatch.setattr(WifiManager, "forwarding_blocked", lambda _self: False)
    manager.tick()
    assert (
        nm.connection == HOTSPOT_CONNECTION
    )  # a single failed check could be a glitch
    manager.tick()
    assert nm.connection == ""
    assert "setup-hotspot.sh" in manager.status()["message"]
    # and not opened again without the block
    nm.in_reach = []
    clock.now += 181
    manager.tick()
    manager.tick()
    assert nm.connection == ""
    assert "setup-hotspot.sh" in manager.status()["error"]


def test_missing_forwarding_block_is_logged_once(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    monkeypatch.setattr(WifiManager, "forwarding_blocked", lambda _self: False)
    nm, clock = FakeNetworkManager(), Clock()
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    with caplog.at_level(logging.WARNING):
        manager.tick()
        for _ in range(3):
            clock.now += 181
            manager.tick()
    assert nm.connection == ""
    assert "setup-hotspot.sh" in manager.status()["error"]
    assert len([r for r in caplog.records if "setup-hotspot.sh" in r.message]) == 1


def retry_after(clock: Clock) -> None:
    clock.now += KNOWN_NETWORK_RETRY_SECONDS + 1


def downs(nm: FakeNetworkManager) -> int:
    return sum(c == ["connection", "down", HOTSPOT_CONNECTION] for c in nm.commands)


def test_unused_hotspot_retries_known_networks(monkeypatch: pytest.MonkeyPatch) -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    nm.in_reach = ["Elsewhere"]
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    stations = [[500]]
    monkeypatch.setattr(manager, "hotspot_stations", lambda: stations[0])

    manager.tick()
    retry_after(clock)
    manager.tick()
    # a device uses the hotspot, so it stays and the wait starts again
    assert downs(nm) == 0

    stations[0] = [ACTIVE_CLIENT_SECONDS * 1000 + 1]  # connected, but idle
    manager.tick()
    retry_after(clock)
    manager.tick()
    # nothing in reach: closed, searched and opened again
    assert downs(nm) == 1
    assert nm.connection == HOTSPOT_CONNECTION
    assert manager.status()["error"] is None

    nm.in_reach = ["Marina"]
    manager.tick()
    retry_after(clock)
    manager.tick()
    assert nm.connection == "Marina"
    assert manager.status()["message"] == "Connected to Marina."


def test_retry_does_not_use_old_scan_results(monkeypatch: pytest.MonkeyPatch) -> None:
    nm, clock = FakeNetworkManager(), Clock()
    nm.state, nm.connection = 100, HOTSPOT_CONNECTION
    nm.in_reach = ["Marina"]  # what NetworkManager saw hours ago
    nm.fail_rescan = True
    manager = WifiManager(CONFIG, run=nm, clock=clock)
    monkeypatch.setattr(manager, "hotspot_stations", list)

    manager.tick()
    retry_after(clock)
    manager.tick()
    assert downs(nm) == 1
    assert nm.connection == HOTSPOT_CONNECTION
    assert not any(c[3:5] == ["up", "uuid"] for c in nm.commands)
    # the usual outcome of a retry, so not shown as an error
    assert manager.status()["error"] is None


def test_hotspot_stations_parses_inactive_times(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = (
        "Station 12:34:56:78:9a:bc (on wlan0)\n"
        "\tinactive time:\t304 ms\n"
        "\trx bytes:\t1234\n"
        "Station 12:34:56:78:9a:bd (on wlan0)\n"
        "\tinactive time:\t120000 ms\n"
        "Station 12:34:56:78:9a:be (on wlan0)\n"
    )
    monkeypatch.setattr(
        commands,
        "run",
        lambda *_args: subprocess.CompletedProcess([], 0, output, ""),
    )
    manager = WifiManager(CONFIG, run=FakeNetworkManager(), clock=Clock())
    assert manager.hotspot_stations() == [304, 120000, 0]


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


def test_new_network_needs_a_password_unless_open() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    for password in (None, ""):
        with pytest.raises(WifiRequestError):
            manager.save_network("Phone", password)
    assert "u-Phone" not in nm.connections

    manager.save_network("Cafe", None, open_network=True)
    add = nm.commands[-1]
    assert add[:2] == ["connection", "add"]
    assert "wifi-sec.psk" not in add


def test_network_priority_range() -> None:
    manager = WifiManager(CONFIG, run=FakeNetworkManager(), clock=Clock())
    with pytest.raises(WifiRequestError):
        manager.save_network("Phone", "secret123", 1000)


def test_network_changes_wait_for_other_changes() -> None:
    nm = FakeNetworkManager()
    manager = WifiManager(CONFIG, run=nm, clock=Clock())
    manager._action_lock.acquire()  # noqa: SLF001 - e.g. a connect in progress
    try:
        with pytest.raises(WifiBusyError):
            manager.save_network("Phone", "secret123")
        with pytest.raises(WifiBusyError):
            manager.delete_network("u-marina")
    finally:
        manager._action_lock.release()  # noqa: SLF001
    assert "u-marina" in nm.connections
    manager.delete_network("u-marina")
    assert "u-marina" not in nm.connections
