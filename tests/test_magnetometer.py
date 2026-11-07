import threading
from queue import Queue

import pytest

import config
import magnetometer
from structs import MagnetometerSample, Vector3


def test_mock_reads_the_field_forward_and_down_in_unit_axes() -> None:
    reader = magnetometer.MagnetometerReader(
        [
            magnetometer.ConfiguredMagnetometer(
                name="mock",
                device=magnetometer.MockMagnetometer(),
                axes="x,-y,-z",
            ),
        ],
    )
    sample = reader.read_sample()
    assert sample.magnetic_fields == (magnetometer.MOCK_FIELD,)


class _Fixed:
    def __init__(self, field: Vector3) -> None:
        self.field = field

    def read(self) -> Vector3:
        return self.field


def test_each_magnetometer_has_its_own_axes() -> None:
    reader = magnetometer.MagnetometerReader(
        [
            magnetometer.ConfiguredMagnetometer(
                name="a",
                device=_Fixed((1.0, 2.0, 3.0)),
                axes="x,y,z",
            ),
            magnetometer.ConfiguredMagnetometer(
                name="b",
                device=_Fixed((1.0, 2.0, 3.0)),
                axes="y,x,-z",
            ),
        ],
    )
    assert reader.read_sample().magnetic_fields == ((1.0, 2.0, 3.0), (2.0, 1.0, -3.0))


class _FailingOnce:
    def __init__(self) -> None:
        self.calls = 0

    def read(self) -> Vector3:
        self.calls += 1
        if self.calls == 1:
            raise OSError
        return (1.0, 0.0, 0.0)


def test_worker_passes_on_batches_and_survives_read_errors() -> None:
    queue: Queue[list[MagnetometerSample]] = Queue()
    stop = threading.Event()
    reader = magnetometer.MagnetometerReader(
        [
            magnetometer.ConfiguredMagnetometer(
                name="a",
                device=_FailingOnce(),
                axes="x,y,z",
            ),
        ],
    )
    thread = threading.Thread(
        target=magnetometer.worker,
        args=(reader, 100.0, [queue]),
        kwargs={"stop": stop},
        daemon=True,
    )
    thread.start()
    batch = queue.get(timeout=2)
    stop.set()
    thread.join(timeout=2)

    assert len(batch) == 10
    assert batch[0].magnetic_fields == ((1.0, 0.0, 0.0),)


def _config(devices: list[str]) -> config.MagnetometerConfig:
    return config.MagnetometerConfig(
        ENABLED=True,
        DEVICES=devices,
        I2C_BUS=1,
        ADDRESSES=[None] * len(devices),
        AXES=["x,-y,-z"] * len(devices),
        SAMPLE_RATE_HZ=20,
    )


def test_unknown_devices_are_reported() -> None:
    assert magnetometer.create_reader(_config(["hmc5883"])) is None
    reader = magnetometer.create_reader(_config(["mock", "mock"]))
    assert reader is not None
    assert len(reader.read_sample().magnetic_fields) == 2


def test_per_device_settings() -> None:
    assert config.per_device(["x,y,z"], 2, "AXES") == ["x,y,z", "x,y,z"]
    assert config.per_device(["a", "b"], 2, "AXES") == ["a", "b"]
    with pytest.raises(ValueError, match="AXES"):
        config.per_device(["a", "b"], 3, "AXES")
