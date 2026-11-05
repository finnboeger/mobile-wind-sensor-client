"""
HTTP server of the local dashboard.

It only uses the standard library to keep the load on the Pi Zero low. It serves the
JSON endpoints below.

- ``GET /api/measurements?since=<ms>&limit=<n>``: one second records, see
  ``history.py``. ``limit`` keeps only the newest ``n`` of them.
"""

import json
import logging
import math
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, cast
from urllib.parse import parse_qs, urlparse

from .history import MeasurementHistory

logger = logging.getLogger(__name__)


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
    ) -> None:
        super().__init__(address, DashboardHandler)
        self.history = history


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

    # ----------------------------------------------------------------- routes

    def do_HEAD(self) -> None:  # noqa: N802
        self.do_GET()

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        routes: dict[str, Callable[[dict[str, list[str]]], None]] = {
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
    logger.info("Dashboard listening on port %d", server.server_address[1])
    server.serve_forever()
