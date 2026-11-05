from pathlib import Path

import pytest

from config import Config

DEFAULT_CONFIG = Path(__file__).parent.parent / "config.default.ini"


def load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    broker: str,
    port: str,
) -> Config:
    text = DEFAULT_CONFIG.read_text()
    text = text.replace("BROKER = broker.hivemq.com", f"BROKER = {broker}")
    text = text.replace("PORT = 8883", f"PORT = {port}")
    (tmp_path / "config.ini").write_text(text)
    monkeypatch.chdir(tmp_path)
    # a fresh instance, the old one is restored after the test
    monkeypatch.setattr(Config, "_instance", None)
    return Config()


def test_mqtt_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config = load(tmp_path, monkeypatch, "broker.example", "1883")
    assert config.MQTT is not None
    assert config.MQTT.BROKER == "broker.example"
    assert config.MQTT.PORT == 1883


def test_empty_broker_disables_mqtt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert load(tmp_path, monkeypatch, "", "8883").MQTT is None


@pytest.mark.parametrize("port", ["0", "65536", "-1", "mqtt", ""])
def test_invalid_port_disables_mqtt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    port: str,
) -> None:
    assert load(tmp_path, monkeypatch, "broker.example", port).MQTT is None
