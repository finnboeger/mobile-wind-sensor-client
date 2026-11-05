"""
HTTP server of the local dashboard.

It only uses the standard library to keep the load on the Pi Zero low. The page is a
Vue app in ``web/``, built into ``static/`` (``npm run build``, shipped in the
releases). It polls the JSON endpoints below.

- ``GET /api/measurements?since=<ms>&limit=<n>``: one second records, see
  ``history.py``. ``limit`` keeps only the newest ``n`` of them.
"""

import gzip
import json
import logging
import math
import mimetypes
import re
import socket
import threading
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Final, cast
from urllib.parse import parse_qs, urlparse

from .history import MeasurementHistory

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"
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
        }
        self._dispatch(routes.get(url.path), parse_qs(url.query))

    def _dispatch(self, route: Callable[[Any], None] | None, argument: object) -> None:
        if route is None:
            self._json({"error": "Not found."}, HTTPStatus.NOT_FOUND)
            return
        try:
            route(argument)
        except RequestError as e:
            self._json({"error": e.message}, e.status)
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
