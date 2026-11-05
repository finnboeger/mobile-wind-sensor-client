"""
HTTP server of the local dashboard.

It only uses the standard library to keep the load on the Pi Zero low. The page is a
Vue app in ``web/``, built into ``static/`` (``npm run build``, shipped in the
releases). It polls the JSON endpoints below.

- ``GET /api/measurements?since=<ms>&limit=<n>``: one second records, see
  ``history.py``. ``limit`` keeps only the newest ``n`` of them.
- ``GET /api/status``: live values of the pipeline and the system
- ``GET /api/wifi``, ``GET /api/wifi/scan``: Wi-Fi status and networks in reach
- ``POST /api/wifi/...``: Wi-Fi changes, see ``do_POST``
"""

import gzip
import hmac
import json
import logging
import math
import mimetypes
import os
import re
import shutil
import socket
import subprocess  # nosec B404
import threading
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Final, cast
from urllib.parse import parse_qs, urlparse

import mqtt
import telemetry
from config import Config

from . import commands
from .history import MeasurementHistory
from .wifi import WifiBusyError, WifiError, WifiManager, WifiRequestError

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"
#: largest accepted request body in bytes
MAX_BODY_SIZE = 4096
# Delay before a wrong admin password is rejected. Passwords are checked one at a
# time, and requests meanwhile get 429 right away, so guessing is limited to about one
# try per second without parallel attempts piling up waiting threads.
WRONG_PASSWORD_DELAY_SECONDS = 1.0
_password_check_lock = threading.Lock()
#: names of the built files in ``static/assets``, which contain a content hash
ASSET_NAME = re.compile(r"[A-Za-z0-9_.-]+")
NOT_BUILT_PAGE = b"""<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8"><title>Windbot</title></head>
<body><h1>Windbot</h1><p>The dashboard hasn't been built. Run <code>npm ci &amp;&amp;
npm run build</code> in <code>src/dashboard/web</code>, or install a release.</p>
</body></html>
"""
#: seconds between checks whether the Wi-Fi interface appeared or changed
INTERFACE_POLL_S: Final = 5


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
        self._slow: dict[str, Any] = {
            "interfaces": [],
            "throttled": None,
            "disk_free_mb": None,
            "disk_total_mb": None,
        }

    def _slow_values(self) -> dict[str, Any]:
        if time.monotonic() - self._time < SYSTEM_CACHE_SECONDS:
            return self._slow
        # The commands can take seconds on a busy Pi. While one request refreshes
        # the values, the others answer with the previous ones instead of waiting.
        if not self._lock.acquire(blocking=False):
            return self._slow
        try:
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
        finally:
            self._lock.release()

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
    return {
        "enabled": True,
        "broker": config.MQTT.BROKER,
        "topic": config.MQTT.TOPIC,
        **mqtt.connection_status,
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

    def __init__(  # noqa: PLR0913
        self,
        address: tuple[str, int],
        history: MeasurementHistory,
        wifi: WifiManager | None,
        admin_password: str | None,
        interface: str | None = None,
        static_dir: Path = STATIC_DIR,
    ) -> None:
        #: only accept connections arriving on this network interface, e.g. so the
        #: dashboard isn't reachable over 4G. Unlike binding to the interface's
        #: address, this keeps working when the address changes (hotspot or not).
        self.interface = interface
        self.static_dir = static_dir
        #: gzipped assets by name, compressed once on the first request
        self.gzipped: dict[str, bytes] = {}
        super().__init__(address, DashboardHandler)
        self.history = history
        self.wifi = wifi
        self.admin_password = admin_password
        self.system = SystemInfo()

    def server_bind(self) -> None:
        if self.interface:
            self.socket.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_BINDTODEVICE,
                self.interface.encode(),
            )
        super().server_bind()


class RequestError(Exception):
    def __init__(self, status: HTTPStatus, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


class DashboardHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    #: seconds until an idle keep-alive connection is closed, so the threads of
    #: clients that left the Wi-Fi or went to sleep don't pile up
    timeout = 30

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
        cache_control: str = "no-store",
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache_control)
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
        # Taken before comparing, so while a wrong password is answered every other
        # attempt gets 429, right or wrong, and the answer reveals nothing
        if not _password_check_lock.acquire(blocking=False):
            raise RequestError(
                HTTPStatus.TOO_MANY_REQUESTS,
                "Too many wrong passwords, try again in a second.",
            )
        try:
            if not hmac.compare_digest(given.encode(), password.encode()):
                time.sleep(WRONG_PASSWORD_DELAY_SECONDS)
                raise RequestError(HTTPStatus.UNAUTHORIZED, "Wrong admin password.")
        finally:
            _password_check_lock.release()

    # ----------------------------------------------------------------- routes

    def do_HEAD(self) -> None:  # noqa: N802
        self.do_GET()

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        if url.path.startswith("/assets/"):
            self._dispatch(self._get_asset, url.path.removeprefix("/assets/"))
            return
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
        except WifiRequestError as e:
            self._json({"error": str(e)}, HTTPStatus.BAD_REQUEST)
        except WifiBusyError as e:
            self._json({"error": str(e)}, HTTPStatus.CONFLICT)
        except WifiError as e:
            # NetworkManager failed, not the request
            self._json({"error": str(e)}, HTTPStatus.BAD_GATEWAY)
        except Exception:
            logger.exception("Dashboard request %s failed", self.path)
            self._json({"error": "Internal error."}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _get_index(self, _: object) -> None:
        try:
            body = (self.dashboard.static_dir / "index.html").read_bytes()
        except FileNotFoundError:
            body = NOT_BUILT_PAGE
        self._send(HTTPStatus.OK, body, "text/html; charset=utf-8")

    def _get_asset(self, name: str) -> None:
        path = self.dashboard.static_dir / "assets" / name
        if not ASSET_NAME.fullmatch(name) or not path.is_file():
            raise RequestError(HTTPStatus.NOT_FOUND, "Not found.")
        content_type = mimetypes.guess_type(name)[0] or "application/octet-stream"
        body = path.read_bytes()
        headers = {"Vary": "Accept-Encoding"}
        # The bundle is several hundred kB, about a third of that gzipped
        if "gzip" in self.headers.get("Accept-Encoding", ""):
            if name not in self.dashboard.gzipped:
                self.dashboard.gzipped[name] = gzip.compress(body)
            body = self.dashboard.gzipped[name]
            headers["Content-Encoding"] = "gzip"
        # the names change with the content, so browsers may keep them forever
        self._send(
            HTTPStatus.OK,
            body,
            content_type,
            headers,
            cache_control="public, max-age=31536000, immutable",
        )

    def _get_measurements(self, query: dict[str, list[str]]) -> None:
        since = query.get("since", [None])[0]
        try:
            timestamp = int(since) if since else None
        except ValueError as e:
            raise RequestError(HTTPStatus.BAD_REQUEST, "Invalid since.") from e
        limit = query.get("limit", [""])[0]
        if limit and not (limit.isdigit() and int(limit) > 0):
            raise RequestError(HTTPStatus.BAD_REQUEST, "Invalid limit.")
        # from the records encoded when they were added, see MeasurementHistory
        items = self.dashboard.history.since_json(
            timestamp,
            int(limit) if limit else None,
        )
        server_time = int(time.time() * 1000)
        body = f'{{"server_time":{server_time},"items":{items}}}'.encode()
        self._send(HTTPStatus.OK, body, "application/json")

    def _get_status(self, _: object) -> None:
        self._json(
            {
                "server_time": int(time.time() * 1000),
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


def serve(server: DashboardServer) -> None:
    logger.info(
        "Dashboard listening on port %d of %s",
        server.server_address[1],
        server.interface or "every interface",
    )
    server.serve_forever()


class InterfaceServer:
    """
    Serve the dashboard on one network interface for as long as it exists.

    ``SO_BINDTODEVICE`` remembers the interface's index, not its name. When the
    interface disappears and comes back (e.g. after a Wi-Fi firmware crash or a
    driver reload) it gets a new index, and the old socket accepts nothing anymore.
    So the server is only started once the interface exists, and restarted whenever
    its index changes. Without the interface nothing is served, instead of falling
    back to every interface (4G included).
    """

    def __init__(
        self,
        interface: str,
        port: int,
        make_server: Callable[[tuple[str, int], str], DashboardServer],
        if_index: Callable[[str], int] = socket.if_nametoindex,
    ) -> None:
        self.interface = interface
        self.port = port
        self.server: DashboardServer | None = None
        self._make_server = make_server
        self._if_index = if_index
        #: index of the interface at the last check, -1 before the first one
        self._index: int | None = -1
        self._start_failed = False

    def check(self) -> None:
        """Start, restart or stop the server to match the interface."""
        try:
            index: int | None = self._if_index(self.interface)
        except OSError:
            index = None
        if index != self._index:
            if self._index not in (-1, None):
                logger.warning(
                    "Network interface %s changed, restarting the dashboard on it",
                    self.interface,
                )
            self._stop()
            self._index = index
            self._start_failed = False
            if index is None:
                logger.error(
                    "Network interface %s not found, the dashboard is only reachable "
                    "on localhost until it appears",
                    self.interface,
                )
        if index is not None and self.server is None:
            self._start()

    def run(self, stop: threading.Event | None = None) -> None:
        """Check the interface every few seconds until ``stop`` is set."""
        stop = stop or threading.Event()
        while True:
            self.check()
            if stop.wait(INTERFACE_POLL_S):
                self._stop()
                return

    def _start(self) -> None:
        try:
            self.server = self._make_server(("", self.port), self.interface)
        except OSError:
            # retried on the next check, but only logged once
            if not self._start_failed:
                logger.exception(
                    "Could not start the dashboard on port %d of %s",
                    self.port,
                    self.interface,
                )
                self._start_failed = True
            return
        threading.Thread(target=serve, args=(self.server,), daemon=True).start()

    def _stop(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
