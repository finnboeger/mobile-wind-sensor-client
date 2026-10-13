"""
HTTP server of the local dashboard.

It only uses the standard library to keep the load on the Pi Zero low. The page in
``static/index.html`` polls the JSON endpoints below.

- ``GET /api/measurements?since=<ms>``: one second records, see ``history.py``
- ``GET /api/status``: live values of the pipeline and the system
- ``GET /api/wifi``, ``GET /api/wifi/scan``: Wi-Fi status and networks in reach
- ``POST /api/wifi/...``: Wi-Fi changes, see ``_post_routes``
"""

import hmac
import json
import logging
import math
import os
import shutil
import subprocess  # nosec B404
import sys
import threading
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, cast
from urllib.parse import parse_qs, urlparse

import telemetry
from config import Config

from . import commands
from .history import MeasurementHistory
from .wifi import WifiError, WifiManager

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"
#: largest accepted request body in bytes
MAX_BODY_SIZE = 4096
#: seconds the slower system values are cached
SYSTEM_CACHE_SECONDS = 5


def _read_first_line(path: str) -> str | None:
    try:
        with Path(path).open(encoding="utf-8") as file:
            return file.readline().strip()
    except OSError:
        return None


def _command(*args: str) -> str | None:
    try:
        result = commands.run(list(args), timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout if result.returncode == 0 else None


class SystemInfo:
    """Cached state of the Pi: load, temperature, memory and network interfaces."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._time = 0.0
        self._slow: dict[str, Any] = {}

    def _slow_values(self) -> dict[str, Any]:
        with self._lock:
            if time.monotonic() - self._time < SYSTEM_CACHE_SECONDS:
                return self._slow
            interfaces = []
            output = _command("ip", "-j", "-4", "address", "show")
            if output:
                try:
                    for interface in json.loads(output):
                        if interface.get("ifname") == "lo":
                            continue
                        interfaces.append(
                            {
                                "name": interface.get("ifname"),
                                "up": "UP" in interface.get("flags", []),
                                "addresses": [
                                    a.get("local")
                                    for a in interface.get("addr_info", [])
                                ],
                            },
                        )
                except ValueError:
                    pass
            throttled = _command("vcgencmd", "get_throttled")
            disk = shutil.disk_usage(Path.cwd())
            self._slow = {
                "interfaces": interfaces,
                "throttled": throttled.strip().partition("=")[2] if throttled else None,
                "disk_free_mb": disk.free // 1_000_000,
                "disk_total_mb": disk.total // 1_000_000,
            }
            self._time = time.monotonic()
            return self._slow

    def values(self) -> dict[str, Any]:
        uptime = _read_first_line("/proc/uptime")
        temperature = _read_first_line("/sys/class/thermal/thermal_zone0/temp")
        memory: dict[str, int] = {}
        try:
            with Path("/proc/meminfo").open(encoding="utf-8") as file:
                for line in file:
                    key, _, value = line.partition(":")
                    if key in ("MemTotal", "MemAvailable"):
                        memory[key] = int(value.split()[0]) // 1000
        except (OSError, ValueError):
            pass
        try:
            load = os.getloadavg()[0]
        except OSError:
            load = None
        return {
            "time": time.time(),
            "uptime": float(uptime.split()[0]) if uptime else None,
            "load": load,
            "cpu_temperature": int(temperature) / 1000 if temperature else None,
            "memory_total_mb": memory.get("MemTotal"),
            "memory_available_mb": memory.get("MemAvailable"),
            **self._slow_values(),
        }


def mqtt_status() -> dict[str, Any]:
    """Return the MQTT connection state, if the MQTT worker is running."""
    config = Config()
    if config.MQTT is None:
        return {"enabled": False}
    module = sys.modules.get("mqtt")
    status = getattr(module, "connection_status", None)
    return {
        "enabled": True,
        "broker": config.MQTT.BROKER,
        "topic": config.MQTT.TOPIC,
        **(dict(status) if isinstance(status, dict) else {}),
    }


def _finite(data: object) -> object:
    """Replace NaN and infinity, which JSON can't represent, with None."""
    if isinstance(data, float):
        return data if math.isfinite(data) else None
    if isinstance(data, dict):
        return {key: _finite(value) for key, value in data.items()}
    if isinstance(data, list | tuple):
        return [_finite(value) for value in data]
    return data


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        history: MeasurementHistory,
        wifi: WifiManager | None,
        admin_password: str | None,
    ) -> None:
        super().__init__(address, DashboardHandler)
        self.history = history
        self.wifi = wifi
        self.admin_password = admin_password
        self.system = SystemInfo()


class RequestError(Exception):
    def __init__(self, status: HTTPStatus, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


class DashboardHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    @property
    def dashboard(self) -> DashboardServer:
        return cast("DashboardServer", self.server)

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002, ANN401
        logger.debug("%s - %s", self.address_string(), format % args)

    # ---------------------------------------------------------------- helpers

    def _send(
        self,
        status: HTTPStatus,
        body: bytes,
        content_type: str,
        headers: dict[str, str] | None = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(_finite(data), separators=(",", ":")).encode()
        self._send(status, body, "application/json")

    def _wifi(self) -> WifiManager:
        if self.dashboard.wifi is None:
            raise RequestError(
                HTTPStatus.SERVICE_UNAVAILABLE,
                "Wi-Fi management isn't available on this device.",
            )
        return self.dashboard.wifi

    def _read_json(self) -> dict[str, Any]:
        content_type = self.headers.get("Content-Type", "")
        # Requiring JSON stops other websites from posting plain forms to this API
        if not content_type.startswith("application/json"):
            raise RequestError(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "Expected JSON.")
        length = int(self.headers.get("Content-Length") or 0)
        if length > MAX_BODY_SIZE:
            raise RequestError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Too large.")
        try:
            data = json.loads(self.rfile.read(length) or b"{}")
        except ValueError as e:
            raise RequestError(HTTPStatus.BAD_REQUEST, "Invalid JSON.") from e
        if not isinstance(data, dict):
            raise RequestError(HTTPStatus.BAD_REQUEST, "Expected a JSON object.")
        return data

    def _check_admin(self) -> None:
        password = self.dashboard.admin_password
        if password is None:
            return
        given = self.headers.get("X-Admin-Password", "")
        if not hmac.compare_digest(given.encode(), password.encode()):
            raise RequestError(HTTPStatus.UNAUTHORIZED, "Wrong admin password.")

    # ----------------------------------------------------------------- routes

    def do_HEAD(self) -> None:  # noqa: N802
        self.do_GET()

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        routes: dict[str, Callable[[dict[str, list[str]]], None]] = {
            "/": self._get_index,
            "/index.html": self._get_index,
            "/api/measurements": self._get_measurements,
            "/api/status": self._get_status,
            "/api/wifi": self._get_wifi,
            "/api/wifi/scan": self._get_scan,
        }
        self._dispatch(routes.get(url.path), parse_qs(url.query))

    def do_POST(self) -> None:  # noqa: N802
        routes: dict[str, Callable[[dict[str, Any]], None]] = {
            "/api/wifi/hotspot": self._post_hotspot,
            "/api/wifi/connect": self._post_connect,
            "/api/wifi/networks": self._post_network,
            "/api/wifi/networks/delete": self._post_delete_network,
        }
        route = routes.get(urlparse(self.path).path)

        def handle(_: object) -> None:
            if route is None:
                raise RequestError(HTTPStatus.NOT_FOUND, "Not found.")
            data = self._read_json()
            self._check_admin()
            route(data)

        self._dispatch(handle, None)

    def _dispatch(self, route: Callable[[Any], None] | None, argument: object) -> None:
        if route is None:
            self._json({"error": "Not found."}, HTTPStatus.NOT_FOUND)
            return
        try:
            route(argument)
        except RequestError as e:
            self._json({"error": e.message}, e.status)
        except WifiError as e:
            self._json({"error": str(e)}, HTTPStatus.BAD_REQUEST)
        except Exception:
            logger.exception("Dashboard request %s failed", self.path)
            self._json({"error": "Internal error."}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _get_index(self, _: object) -> None:
        body = (STATIC_DIR / "index.html").read_bytes()
        self._send(HTTPStatus.OK, body, "text/html; charset=utf-8")

    def _get_measurements(self, query: dict[str, list[str]]) -> None:
        since = query.get("since", [None])[0]
        try:
            timestamp = int(since) if since else None
        except ValueError as e:
            raise RequestError(HTTPStatus.BAD_REQUEST, "Invalid since.") from e
        self._json(
            {
                "server_time": int(time.time() * 1000),
                "items": self.dashboard.history.since(timestamp),
            },
        )

    def _get_status(self, _: object) -> None:
        self._json(
            {
                "server_time": time.time(),
                "values": telemetry.snapshot(),
                "mqtt": mqtt_status(),
                "system": self.dashboard.system.values(),
                "admin_password_required": self.dashboard.admin_password is not None,
            },
        )

    def _get_wifi(self, _: object) -> None:
        self._json(
            {
                **self._wifi().status(),
                "admin_password_required": self.dashboard.admin_password is not None,
            },
        )

    def _get_scan(self, _: object) -> None:
        self._json({"networks": self._wifi().scan()})

    def _post_hotspot(self, data: dict[str, Any]) -> None:
        action = data.get("action")
        if action == "start":
            self._wifi().start_hotspot()
        elif action == "stop":
            self._wifi().stop_hotspot()
        else:
            raise RequestError(HTTPStatus.BAD_REQUEST, "Unknown action.")
        self._json({"ok": True}, HTTPStatus.ACCEPTED)

    def _post_connect(self, data: dict[str, Any]) -> None:
        self._wifi().connect(str(data.get("uuid", "")))
        self._json({"ok": True}, HTTPStatus.ACCEPTED)

    def _post_network(self, data: dict[str, Any]) -> None:
        password = data.get("password")
        try:
            priority = int(data.get("priority") or 0)
        except (TypeError, ValueError) as e:
            raise RequestError(HTTPStatus.BAD_REQUEST, "Invalid priority.") from e
        self._wifi().save_network(
            str(data.get("ssid", "")),
            None if password is None else str(password),
            priority,
            autoconnect=bool(data.get("autoconnect", True)),
            hidden=bool(data.get("hidden", False)),
            uuid=str(data["uuid"]) if data.get("uuid") else None,
        )
        self._json({"ok": True})

    def _post_delete_network(self, data: dict[str, Any]) -> None:
        self._wifi().delete_network(str(data.get("uuid", "")))
        self._json({"ok": True})


def serve(server: DashboardServer) -> None:
    logger.info("Dashboard listening on port %d", server.server_address[1])
    server.serve_forever()
