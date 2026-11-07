import math
import threading
from collections.abc import Callable
from queue import Queue

import pytest

import compass
from structs import HeadingData, ImuSample, MagnetometerSample
from tests.simulation import EARTH_FIELD, rotate, simulate, world_to_unit, wrap


@pytest.mark.parametrize("heading", [0, 45, 90, 180, 270, 300])
@pytest.mark.parametrize(("pitch", "roll"), [(0, 0), (10, 0), (0, -20), (15, 25)])
def test_heading_from_a_tilted_field(heading: float, pitch: float, roll: float) -> None:
    rotation = world_to_unit(
        math.radians(heading),
        math.radians(pitch),
        math.radians(roll),
    )
    field = rotate(rotation, EARTH_FIELD)
    measured = compass.heading_from_field(
        field,
        math.radians(roll),
        math.radians(pitch),
    )
    assert abs(math.degrees(wrap(measured - math.radians(heading)))) < 1e-6


def _errors(
    processor: compass.CompassProcessor,
    imu_samples: list[ImuSample],
    magnetometer_samples: list[MagnetometerSample],
    truth: Callable[[float], float],
    after: float,
) -> list[float]:
    headings = processor.add(imu_samples, magnetometer_samples)
    return [
        math.degrees(wrap(h.heading - truth(h.timestamp / 1000)))
        for h in headings
        if h.timestamp / 1000 > after
    ]


def test_heading_follows_a_turn_in_waves() -> None:
    def attitude(time: float) -> tuple[float, float, float]:
        # 10 s straight, then a turn at 10°/s, rolling ±8° and pitching ±4° in waves
        heading = math.radians(30 + 10 * max(0.0, time - 10))
        pitch = math.radians(4) * math.sin(math.tau * time / 3.5)
        roll = math.radians(8) * math.sin(math.tau * time / 5)
        return heading, pitch, roll

    imu_samples, magnetometer_samples = simulate(attitude, 60)
    processor = compass.CompassProcessor(["mock"])
    errors = _errors(
        processor,
        imu_samples,
        magnetometer_samples,
        lambda time: attitude(time)[0],
        after=5,
    )
    assert max(abs(e) for e in errors) < 2


def test_declination_is_added() -> None:
    imu_samples, magnetometer_samples = simulate(lambda _: (0.0, 0.0, 0.0), 5)
    processor = compass.CompassProcessor(
        ["mock"],
        compass.CompassSettings(declination=math.radians(3)),
    )
    headings = processor.add(imu_samples, magnetometer_samples)
    assert abs(math.degrees(wrap(headings[-1].heading)) - 3) < 0.1


def test_no_heading_before_the_imu_has_started() -> None:
    _, magnetometer_samples = simulate(lambda _: (0.0, 0.0, 0.0), 1)
    processor = compass.CompassProcessor(["mock"])
    assert processor.add([], magnetometer_samples) == []


def test_worker_passes_on_headings() -> None:
    imu_samples, magnetometer_samples = simulate(lambda _: (1.0, 0.0, 0.0), 1)
    imu_queue: Queue[list[ImuSample]] = Queue()
    magnetometer_queue: Queue[list[MagnetometerSample]] = Queue()
    output: Queue[HeadingData] = Queue()
    imu_queue.put(imu_samples)
    magnetometer_queue.put(magnetometer_samples)
    stop = threading.Event()
    thread = threading.Thread(
        target=compass.worker,
        args=(compass.CompassProcessor(["mock"]), imu_queue, magnetometer_queue),
        kwargs={"output_queues": [output], "stop": stop},
        daemon=True,
    )
    thread.start()
    heading = output.get(timeout=2)
    stop.set()
    magnetometer_queue.put([])
    thread.join(timeout=2)
    assert abs(heading.heading - 1.0) < 0.01
