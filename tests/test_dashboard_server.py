import json
import math
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

import telemetry
from dashboard.history import MeasurementHistory
from dashboard.server import DashboardServer
from structs import CorrectedApparentWindData, PositionData, TrueWindData


def position(timestamp: int, speed: float = 2.0, course: float = 90.0) -> PositionData:
    return PositionData(
        timestamp=timestamp,
        valid=True,
        latitude=54.0,
        longitude=10.0,
        altitude=0,
        geoidal_separation=0,
        speed=speed,
        true_course=course,
    )


def wind(timestamp: int, speed: float, angle_deg: float) -> tuple:
    angle = math.radians(angle_deg)
    return (
        CorrectedApparentWindData(
            wind_speed=speed + 1,
            wind_angle=angle,
            timestamp=timestamp,
        ),
        TrueWindData(wind_speed=speed, wind_angle=angle, timestamp=timestamp),
    )


def test_history_averages_each_second() -> None:
    history = MeasurementHistory()
    history.add_position(position(1_000))
    history.add_wind(*wind(1_100, 4.0, 350))
    history.add_wind(*wind(1_600, 6.0, 10))
    assert history.since() == []  # the second isn't finished yet

    history.add_wind(*wind(2_050, 5.0, 0))
    (record,) = history.since()
    assert record["ts"] == 1_000
    assert record["true_wind_speed_mps"] == pytest.approx(4.92, abs=0.01)
    # vector average, the stronger wind from 10° pulls it past north
    assert record["true_wind_dir_deg"] == pytest.approx(2.0, abs=0.1)
    assert record["sensor_speed_mps"] == 2.0
    assert record["sensor_heading_deg"] == 90.0
    assert history.since(1_000) == []


def test_history_drops_old_positions() -> None:
    history = MeasurementHistory()
    for second in range(100):
        history.add_position(position(second * 1000))
    assert len(history._positions) <= 11  # noqa: SLF001


@pytest.fixture
def server() -> Iterator[DashboardServer]:
    history = MeasurementHistory()
    history.add_position(position(1_000))
    history.add_wind(*wind(1_000, 5.0, 45))
    history.add_wind(*wind(2_000, 5.0, 45))
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
    req = urllib.request.Request(url, data=data, headers=headers or {})  # noqa: S310
    try:
        with urllib.request.urlopen(req, timeout=5) as response:  # noqa: S310
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_index(server: DashboardServer) -> None:
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    with urllib.request.urlopen(url, timeout=5) as response:  # noqa: S310
        assert b"<title>Windbot</title>" in response.read()


def test_measurements(server: DashboardServer) -> None:
    status, data = request(server, "/api/measurements")
    assert status == 200
    assert [r["ts"] for r in data["items"]] == [1_000]
    status, data = request(server, "/api/measurements?since=1000")
    assert data["items"] == []
    status, _ = request(server, "/api/measurements?since=abc")
    assert status == 400


def test_status_replaces_nan(server: DashboardServer) -> None:
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


def test_not_found(server: DashboardServer) -> None:
    status, _ = request(server, "/api/nope")
    assert status == 404
