import socket
import threading
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

import pytest

from dashboard import internet
from dashboard.internet import InternetCheck, check_http, check_tcp


def http_server(status: int) -> Iterator[int]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            self.send_response(status)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002, ANN401
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server.server_address[1]
    server.shutdown()
    server.server_close()


@pytest.fixture
def no_content() -> Iterator[int]:
    yield from http_server(204)


@pytest.fixture
def portal() -> Iterator[int]:
    yield from http_server(200)


def closed_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_http_204_is_reachable(no_content: int) -> None:
    result = check_http("127.0.0.1", "/generate_204", no_content)
    assert result["ok"] is True
    assert result["local_address"] == "127.0.0.1"
    assert result["latency_ms"] >= 0


def test_http_other_answer_fails(portal: int) -> None:
    result = check_http("127.0.0.1", "/generate_204", portal)
    assert result["ok"] is False
    assert "Unexpected answer 200" in result["error"]


def test_http_connection_refused() -> None:
    result = check_http("127.0.0.1", "/", closed_port())
    assert result["ok"] is False
    assert result["error"].startswith("Connection to 127.0.0.1")


def test_dns_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail(*_: object, **__: object) -> None:
        raise socket.gaierror(socket.EAI_NONAME, "Name or service not known")

    monkeypatch.setattr(internet.socket, "getaddrinfo", fail)
    result = check_tcp("broker.example", 8883)
    assert result == {
        "ok": False,
        "target": "broker.example:8883",
        "error": "DNS lookup of broker.example failed: Name or service not known",
    }


def test_tcp(no_content: int) -> None:
    assert check_tcp("127.0.0.1", no_content)["ok"] is True
    assert check_tcp("127.0.0.1", closed_port())["ok"] is False


def test_result_is_reused_for_a_few_seconds() -> None:
    now = [100.0]
    calls: list[str] = []

    def check_internet() -> dict[str, Any]:
        calls.append("internet")
        return {"ok": True}

    def check_broker(host: str, port: int) -> dict[str, Any]:
        calls.append(f"{host}:{port}")
        return {"ok": False}

    check = InternetCheck(
        ("broker", 8883),
        check_internet,
        check_broker,
        lambda: now[0],
    )
    first = check.result()
    assert first["internet"] == {"ok": True}
    assert first["broker"] == {"ok": False}
    assert sorted(calls) == ["broker:8883", "internet"]

    now[0] += internet.MIN_INTERVAL_S - 1
    assert check.result() is first
    assert len(calls) == 2

    now[0] += 1
    assert check.result() is not first
    assert len(calls) == 4


def test_without_broker() -> None:
    check = InternetCheck(None, lambda: {"ok": True})
    assert check.result()["broker"] is None
