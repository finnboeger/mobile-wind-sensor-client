import itertools
import threading
from queue import Queue

import pytest

import imu
from structs import ImuSample, Vector3


def test_axes_for_a_flat_breakout() -> None:
    to_unit = imu.parse_axes("x,-y,-z")
    assert to_unit((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


def test_axes_for_a_breakout_standing_on_its_edge() -> None:
    # sensor z points forward, sensor x down, sensor y to the left
    to_unit = imu.parse_axes("z, -y, x")
    assert to_unit((1.0, 2.0, 3.0)) == (3.0, -2.0, 1.0)


@pytest.mark.parametrize("specification", ["x,y", "x,y,w", "x,x,z", "x,-x,z"])
def test_invalid_axes_are_rejected(specification: str) -> None:
    with pytest.raises(ValueError, match="Axis mapping|Unknown axis"):
        imu.parse_axes(specification)


def test_mock_reads_gravity_downwards_in_unit_axes() -> None:
    reader = imu.ImuReader(imu.MockImu(), "x,-y,-z")
    sample = reader.read_sample()
    assert sample.acceleration == (0.0, 0.0, -imu.GRAVITY)
    assert sample.angular_rate == (0.0, 0.0, 0.0)


class _FailingOnce:
    def __init__(self) -> None:
        self.calls = 0

    def read(self) -> tuple[Vector3, Vector3]:
        self.calls += 1
        if self.calls == 1:
            raise OSError
        return (0.0, 0.0, imu.GRAVITY), (0.0, 0.0, 0.1)


def test_worker_passes_on_batches_and_survives_read_errors() -> None:
    queue: Queue[list[ImuSample]] = Queue()
    stop = threading.Event()
    thread = threading.Thread(
        target=imu.worker,
        args=(imu.ImuReader(_FailingOnce(), "x,y,z"), 200.0, [queue]),
        kwargs={"stop": stop},
        daemon=True,
    )
    thread.start()
    batch = queue.get(timeout=2)
    stop.set()
    thread.join(timeout=2)

    # 200 Hz in batches of a tenth of a second
    assert len(batch) == 20
    assert batch[0].angular_rate == (0.0, 0.0, 0.1)
    assert all(a.timestamp <= b.timestamp for a, b in itertools.pairwise(batch))
