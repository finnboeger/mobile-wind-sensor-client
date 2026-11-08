"""
Check whether the windbot can reach the internet and the MQTT broker.

Both checks use the normal routing of the Pi, like the MQTT client does, so they show
whether ordinary traffic gets out, not only traffic bound to one interface. DNS and
the connection are checked separately to tell the two failures apart.

- internet: an HTTP request to Google's connectivity check, which answers 204 and is
  meant for exactly this. Anything else, e.g. a captive portal's login page, fails.
- broker: a TCP connection to the configured MQTT broker and port.
"""

import http.client
import socket
import threading
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from typing import Any, Final

#: host and path of the connectivity check, answered with 204 and an empty body
CHECK_HOST: Final = "connectivitycheck.gstatic.com"
CHECK_PATH: Final = "/generate_204"
HTTP_PORT: Final = 80
#: seconds each step (DNS, connection, answer) may take
TIMEOUT_S: Final = 5.0
#: seconds a result is reused, so several open dashboards don't each start a check
MIN_INTERVAL_S: Final = 5.0


class CheckError(Exception):
    pass


def _connect(host: str, port: int, timeout: float) -> socket.socket:
    """Resolve ``host`` and connect to it, with a message saying which step failed."""
    try:
        addresses = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
    except (socket.gaierror, UnicodeError) as e:
        message = e.strerror if isinstance(e, socket.gaierror) else str(e)
        msg = f"DNS lookup of {host} failed: {message}"
        raise CheckError(msg) from e
    error: OSError | None = None
    for family, kind, proto, _, address in addresses:
        sock = socket.socket(family, kind, proto)
        sock.settimeout(timeout)
        try:
            sock.connect(address)
        except OSError as e:
            sock.close()
            error = e
            continue
        return sock
    reason = "timed out" if isinstance(error, TimeoutError) else str(error)
    msg = f"Connection to {host} port {port} failed: {reason}"
    raise CheckError(msg)


def check_tcp(host: str, port: int, timeout: float = TIMEOUT_S) -> dict[str, Any]:
    """Open and close a TCP connection to ``host``."""
    start = time.monotonic()
    try:
        sock = _connect(host, port, timeout)
    except CheckError as e:
        return {"ok": False, "error": str(e), "target": f"{host}:{port}"}
    with sock:
        local = sock.getsockname()[0]
    return {
        "ok": True,
        "target": f"{host}:{port}",
        "latency_ms": round((time.monotonic() - start) * 1000),
        "local_address": local,
    }


def check_http(
    host: str = CHECK_HOST,
    path: str = CHECK_PATH,
    port: int = HTTP_PORT,
    timeout: float = TIMEOUT_S,
) -> dict[str, Any]:
    """Request ``path`` over HTTP and expect 204 No Content."""
    target = (
        f"http://{host}{path}" if port == HTTP_PORT else f"http://{host}:{port}{path}"
    )
    start = time.monotonic()
    try:
        sock = _connect(host, port, timeout)
    except CheckError as e:
        return {"ok": False, "error": str(e), "target": target}
    local = sock.getsockname()[0]
    connection = http.client.HTTPConnection(host, port, timeout=timeout)
    # already connected above, so the DNS and connection errors are told apart
    connection.sock = sock
    try:
        connection.request("GET", path, headers={"User-Agent": "windbot"})
        status = connection.getresponse().status
    except (OSError, http.client.HTTPException) as e:
        reason = (
            "timed out" if isinstance(e, TimeoutError) else str(e) or type(e).__name__
        )
        return {
            "ok": False,
            "error": f"No answer from {host}: {reason}",
            "target": target,
        }
    finally:
        connection.close()
    result: dict[str, Any] = {
        "ok": status == HTTPStatus.NO_CONTENT,
        "target": target,
        "latency_ms": round((time.monotonic() - start) * 1000),
        "local_address": local,
    }
    if status != HTTPStatus.NO_CONTENT:
        # e.g. a captive portal of a Wi-Fi that wants a login
        result["error"] = f"Unexpected answer {status} from {host}"
    return result


class InternetCheck:
    """Runs the checks on request and keeps the last result."""

    def __init__(
        self,
        broker: tuple[str, int] | None,
        check_internet: Callable[[], dict[str, Any]] = check_http,
        check_broker: Callable[[str, int], dict[str, Any]] = check_tcp,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.broker = broker
        self._check_internet = check_internet
        self._check_broker = check_broker
        self._clock = clock
        self._lock = threading.Lock()
        self._result: dict[str, Any] | None = None
        self._checked = -MIN_INTERVAL_S

    def result(self) -> dict[str, Any]:
        """
        Check now and return the result, or the last one if it is only seconds old.

        One check runs at a time. Requests that arrive meanwhile wait for it and get
        its result, instead of starting another one.
        """
        with self._lock:
            if self._result is None or self._clock() - self._checked >= MIN_INTERVAL_S:
                self._result = self._run()
                self._checked = self._clock()
            return self._result

    def _run(self) -> dict[str, Any]:
        with ThreadPoolExecutor(max_workers=2) as pool:
            internet = pool.submit(self._check_internet)
            broker = (
                pool.submit(self._check_broker, *self.broker) if self.broker else None
            )
            return {
                "checked": int(time.time() * 1000),
                "internet": internet.result(),
                "broker": broker.result() if broker else None,
            }
