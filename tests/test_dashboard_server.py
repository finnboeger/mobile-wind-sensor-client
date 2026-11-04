import gzip
import json
import socket
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator
from pathlib import Path

import pytest

import dashboard.server
import telemetry
from dashboard.history import MeasurementHistory
from dashboard.server import (
    DashboardHandler,
    DashboardServer,
    InterfaceServer,
)
from dashboard.wifi import WifiManager
from tests.test_dashboard_history import second
from tests.test_dashboard_wifi import CONFIG, Clock, FakeNetworkManager


@pytest.fixture
def server() -> Iterator[DashboardServer]:
    history = MeasurementHistory()
    history.add_second(second(1, 5.0, 45))
    dashboard = DashboardServer(("127.0.0.1", 0), history, None, "secret")
    thread = threading.Thread(target=dashboard.serve_forever, daemon=True)
    thread.start()
    yield dashboard
    dashboard.shutdown()
    dashboard.server_close()


def request(
    server: DashboardServer,
    path: str,
    body: dict | None = None,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict]:
    url = f"http://127.0.0.1:{server.server_address[1]}{path}"
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers=headers or {})  # noqa: S310 # nosec B310
    try:
        with urllib.request.urlopen(req, timeout=5) as response:  # noqa: S310 # nosec B310
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def get(server: DashboardServer, path: str) -> tuple[int, dict[str, str], bytes]:
    url = f"http://127.0.0.1:{server.server_address[1]}{path}"
    try:
        with urllib.request.urlopen(url, timeout=5) as response:  # noqa: S310 # nosec B310
            return response.status, dict(response.headers), response.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


def test_index_not_built(server: DashboardServer, tmp_path: Path) -> None:
    server.static_dir = tmp_path
    status, _, body = get(server, "/")
    assert status == 200
    assert b"hasn't been built" in body


def test_static_files(server: DashboardServer, tmp_path: Path) -> None:
    (tmp_path / "assets").mkdir()
    (tmp_path / "index.html").write_text("<title>Windbot</title>")
    (tmp_path / "assets" / "index-abc.js").write_text("console.log(1)")
    server.static_dir = tmp_path
    status, headers, body = get(server, "/")
    assert body == b"<title>Windbot</title>"
    assert headers["Cache-Control"] == "no-store"
    status, headers, body = get(server, "/assets/index-abc.js")
    assert status == 200
    assert body == b"console.log(1)"
    assert "javascript" in headers["Content-Type"]
    assert "immutable" in headers["Cache-Control"]
    request = urllib.request.Request(
        f"http://127.0.0.1:{server.server_address[1]}/assets/index-abc.js",
        headers={"Accept-Encoding": "gzip"},
    )
    with urllib.request.urlopen(request, timeout=5) as response:  # noqa: S310 # nosec B310
        assert response.headers["Content-Encoding"] == "gzip"
        assert gzip.decompress(response.read()) == b"console.log(1)"
    assert get(server, "/assets/missing.js")[0] == 404
    assert get(server, "/assets/..%2Findex.html")[0] == 404


def test_measurements(server: DashboardServer) -> None:
    status, data = request(server, "/api/measurements")
    assert status == 200
    assert [r["ts"] for r in data["items"]] == [1_000]
    status, data = request(server, "/api/measurements?since=1000")
    assert data["items"] == []
    status, _ = request(server, "/api/measurements?since=abc")
    assert status == 400


def test_measurements_limit(server: DashboardServer) -> None:
    server.history.add_second(second(2))
    status, data = request(server, "/api/measurements?limit=1")
    assert status == 200
    assert [r["ts"] for r in data["items"]] == [2_000]
    for limit in ("0", "-1", "abc"):
        status, _ = request(server, f"/api/measurements?limit={limit}")
        assert status == 400


def test_status_replaces_nan(
    server: DashboardServer,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # without a config.ini there is no MQTT configuration to read
    monkeypatch.setattr(dashboard.server, "mqtt_status", lambda: {"enabled": False})
    telemetry.update("gps", speed=float("nan"), satellites=7)
    status, data = request(server, "/api/status")
    assert status == 200
    assert data["values"]["gps"]["speed"] is None
    assert data["values"]["gps"]["satellites"] == 7
    assert data["admin_password_required"] is True


def test_wifi_unavailable(server: DashboardServer) -> None:
    status, data = request(server, "/api/wifi")
    assert status == 503
    assert "isn't available" in data["error"]


def test_post_requires_json_and_password(server: DashboardServer) -> None:
    body = {"action": "start"}
    status, _ = request(
        server,
        "/api/wifi/hotspot",
        body,
        {"Content-Type": "text/plain"},
    )
    assert status == 415
    json_header = {"Content-Type": "application/json"}
    status, _ = request(server, "/api/wifi/hotspot", body, json_header)
    assert status == 401
    status, _ = request(
        server,
        "/api/wifi/hotspot",
        body,
        {**json_header, "X-Admin-Password": "secret"},
    )
    assert status == 503  # authorized, but no Wi-Fi in the tests


def test_passwords_during_a_wrong_one_get_429(server: DashboardServer) -> None:
    json_header = {"Content-Type": "application/json"}
    # as if a wrong password were being answered right now
    assert dashboard.server._password_check_lock.acquire(blocking=False)  # noqa: SLF001
    try:
        # the right one too, so the answer doesn't tell them apart
        for password in ("wrong", "secret"):
            status, data = request(
                server,
                "/api/wifi/connect",
                {"uuid": "x"},
                {**json_header, "X-Admin-Password": password},
            )
            assert status == 429
            assert "try again" in data["error"]
    finally:
        dashboard.server._password_check_lock.release()  # noqa: SLF001


def test_not_found(server: DashboardServer) -> None:
    status, _ = request(server, "/api/nope")
    assert status == 404


def test_idle_connections_are_closed(
    server: DashboardServer,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(DashboardHandler, "timeout", 0.2)
    with socket.create_connection(("127.0.0.1", server.server_address[1])) as conn:
        conn.settimeout(5)
        # an idle client that never sends a request is disconnected by the server
        assert conn.recv(1) == b""


def test_interface_server_follows_the_interface() -> None:
    indexes: list[int | None] = [None]

    def if_index(name: str) -> int:
        assert name == "wlan0"
        if indexes[-1] is None:
            raise OSError(name)
        return indexes[-1]

    history = MeasurementHistory()
    history.add_second(second(1))
    interfaces: list[str] = []

    def make_server(_address: tuple[str, int], interface: str) -> DashboardServer:
        # Without SO_BINDTODEVICE, which only Linux has, so the test runs anywhere
        interfaces.append(interface)
        return DashboardServer(("127.0.0.1", 0), history, None, None)

    watcher = InterfaceServer("wlan0", 0, make_server, if_index)
    try:
        watcher.check()
        assert watcher.server is None  # stays closed without the interface

        indexes.append(1)
        watcher.check()
        first = watcher.server
        assert first is not None
        assert request(first, "/api/measurements")[0] == 200
        watcher.check()
        assert watcher.server is first

        indexes.append(7)  # the interface came back with a new index
        watcher.check()
        assert watcher.server is not None
        assert watcher.server is not first
        assert request(watcher.server, "/api/measurements")[0] == 200

        indexes.append(None)
        watcher.check()
        assert watcher.server is None
        assert interfaces == ["wlan0", "wlan0"]
    finally:
        indexes.append(None)
        watcher.check()


@pytest.fixture
def wifi_server() -> Iterator[tuple[DashboardServer, FakeNetworkManager]]:
    nm = FakeNetworkManager()
    wifi = WifiManager(CONFIG, run=nm, clock=Clock())
    dashboard = DashboardServer(("127.0.0.1", 0), MeasurementHistory(), wifi, "secret")
    thread = threading.Thread(target=dashboard.serve_forever, daemon=True)
    thread.start()
    yield dashboard, nm
    dashboard.shutdown()
    dashboard.server_close()


def test_save_network_checks_the_request(
    wifi_server: tuple[DashboardServer, FakeNetworkManager],
) -> None:
    server, nm = wifi_server
    headers = {"Content-Type": "application/json", "X-Admin-Password": "secret"}

    def save(**fields: object) -> tuple[int, dict]:
        body = {"ssid": "Phone", "password": "secret123", **fields}
        return request(server, "/api/wifi/networks", body, headers)

    assert save(priority=5000)[0] == 400
    assert save(priority=True)[0] == 400
    assert save(hidden="false")[0] == 400
    status, data = save(password="")  # nosec B106
    assert status == 400
    assert "password" in data["error"]
    assert not any(c[:2] == ["connection", "add"] for c in nm.commands)

    assert save(priority=3, hidden=False)[0] == 200
    assert save(ssid="Cafe", password=None, open=True)[0] == 200
