import socket
import threading
import time
from queue import Queue
from types import SimpleNamespace

import pytest

import mqtt


def _unused_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_worker_keeps_running_when_broker_is_unreachable_at_start(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Simulates booting before the 4G modem is online: nothing accepts the
    # connection. The worker must keep retrying instead of exiting.
    config = SimpleNamespace(
        MQTT=SimpleNamespace(
            BROKER="127.0.0.1",
            PORT=_unused_port(),
            USE_TLS=False,
            USERNAME=None,
            PASSWORD=None,
            CLIENT_ID="test",
            TOPIC="test",
        ),
    )
    monkeypatch.setattr(mqtt, "Config", lambda: config)

    thread = threading.Thread(
        target=mqtt.worker,
        args=(Queue(), Queue()),
        daemon=True,
    )
    thread.start()
    time.sleep(2)

    assert thread.is_alive()
