import json
import socket
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

from dashboard.history import MeasurementHistory
from dashboard.server import (
    DashboardHandler,
    DashboardServer,
    InterfaceServer,
    _finite,
)
from tests.test_dashboard_history import second


@pytest.fixture
def server() -> Iterator[DashboardServer]:
    history = MeasurementHistory()
    history.add_second(second(1, 5.0, 45))
    dashboard = DashboardServer(("127.0.0.1", 0), history)
    thread = threading.Thread(target=dashboard.serve_forever, daemon=True)
    thread.start()
    yield dashboard
    dashboard.shutdown()
    dashboard.server_close()


def request(server: DashboardServer, path: str) -> tuple[int, dict]:
    url = f"http://127.0.0.1:{server.server_address[1]}{path}"
    try:
        with urllib.request.urlopen(url, timeout=5) as response:  # noqa: S310 # nosec B310
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


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


def test_finite_replaces_nan() -> None:
    data = {"a": [1.0, float("nan")], "b": float("inf"), "c": "x"}
    assert _finite(data) == {"a": [1.0, None], "b": None, "c": "x"}


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
        return DashboardServer(("127.0.0.1", 0), history)

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
