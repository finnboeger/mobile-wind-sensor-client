import math
import random
import threading
from collections.abc import Callable
from pathlib import Path
from queue import Queue

import jsonpickle
import pytest

import calibrate_compass
import compass
import compass_calibration
from log import DATA_SEPARATOR
from structs import HeadingData, ImuSample, MagnetometerSample, Vector3
from tests.simulation import (
    EARTH_FIELD,
    Magnetometer,
    rotate,
    simulate,
    world_to_unit,
    wrap,
)


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
    processor = compass.CompassProcessor({"mock": None})
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
        {"mock": None},
        compass.CompassSettings(declination=math.radians(3)),
    )
    headings = processor.add(imu_samples, magnetometer_samples)
    assert abs(math.degrees(wrap(headings[-1].heading)) - 3) < 0.1


def test_no_heading_before_the_imu_has_started() -> None:
    _, magnetometer_samples = simulate(lambda _: (0.0, 0.0, 0.0), 1)
    processor = compass.CompassProcessor({"mock": None})
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
        args=(compass.CompassProcessor({"mock": None}), imu_queue, magnetometer_queue),
        kwargs={"output_queues": [output], "stop": stop},
        daemon=True,
    )
    thread.start()
    heading = output.get(timeout=2)
    stop.set()
    magnetometer_queue.put([])
    thread.join(timeout=2)
    assert abs(heading.heading - 1.0) < 0.01


def _random_orientations(count: int, seed: int) -> list[tuple[float, float, float]]:
    rng = random.Random(seed)
    return [
        (
            rng.uniform(0, math.tau),
            math.asin(rng.uniform(-1, 1)),
            rng.uniform(-math.pi, math.pi),
        )
        for _ in range(count)
    ]


HARD_IRON = (12.0, -7.0, 20.0)
SOFT_IRON = [[1.1, 0.05, 0.0], [0.05, 0.92, 0.03], [0.0, 0.03, 1.02]]


def _bench_samples(sensor: Magnetometer, seed: int = 5) -> list[Vector3]:
    rng = random.Random(seed)
    return [
        sensor.measure(rotate(world_to_unit(*orientation), EARTH_FIELD), 0, rng)
        for orientation in _random_orientations(400, seed)
    ]


def test_bench_fit_recovers_hard_and_soft_iron() -> None:
    sensor = Magnetometer(hard_iron=HARD_IRON, soft_iron=SOFT_IRON, noise=0.05)
    result = compass_calibration.fit_ellipsoid(_bench_samples(sensor))
    assert result is not None
    calibration = result.calibration
    for found, expected in zip(calibration.offset, HARD_IRON, strict=True):
        assert abs(found - expected) < 0.2
    assert calibration.field_strength is not None
    assert abs(calibration.field_strength - math.hypot(*EARTH_FIELD)) < 2.5

    # headings from the corrected field are right to well under a degree
    for heading, steep_pitch, any_roll in _random_orientations(50, 7):
        pitch, roll = steep_pitch / 4, any_roll / 4
        raw = sensor.measure(
            rotate(world_to_unit(heading, pitch, roll), EARTH_FIELD),
            0,
            random.Random(0),
        )
        measured = compass.heading_from_field(calibration.apply(raw), roll, pitch)
        assert abs(math.degrees(wrap(measured - heading))) < 0.5


def test_bench_fit_needs_samples() -> None:
    assert compass_calibration.fit_ellipsoid([(1.0, 2.0, 3.0)] * 10) is None


def test_bench_calibration_round_trip(tmp_path: Path) -> None:
    sensor = Magnetometer(hard_iron=HARD_IRON, soft_iron=SOFT_IRON)
    result = compass_calibration.fit_ellipsoid(_bench_samples(sensor))
    assert result is not None
    path = tmp_path / "bench.json"
    result.calibration.save(path)
    loaded = compass_calibration.BenchCalibration.load(path)
    assert loaded == result.calibration
    assert compass_calibration.BenchCalibration.load(tmp_path / "missing.json") == (
        compass_calibration.BenchCalibration()
    )


def _turning(time: float) -> tuple[float, float, float]:
    return math.radians(10 * time), 0.0, math.radians(5) * math.sin(time)


def test_bench_calibration_removes_the_iron_in_the_unit() -> None:
    sensor = Magnetometer(hard_iron=HARD_IRON, soft_iron=SOFT_IRON, noise=0.05)
    imu_samples, magnetometer_samples = simulate(
        _turning,
        72,
        magnetometers=[sensor],
    )
    uncalibrated = _errors(
        compass.CompassProcessor({"sensor": None}),
        imu_samples,
        magnetometer_samples,
        lambda time: _turning(time)[0],
        after=5,
    )
    result = compass_calibration.fit_ellipsoid(_bench_samples(sensor))
    assert result is not None
    calibrated = _errors(
        compass.CompassProcessor({"sensor": result.calibration}),
        imu_samples,
        magnetometer_samples,
        lambda time: _turning(time)[0],
        after=5,
    )
    assert max(abs(e) for e in uncalibrated) > 30
    assert max(abs(e) for e in calibrated) < 1.5


def test_calibration_tool_reads_the_data_log(tmp_path: Path) -> None:
    _, samples = simulate(_turning, 1, magnetometers=[Magnetometer(), Magnetometer()])
    path = tmp_path / "magnetometer.log"
    path.write_text(
        f"1.0{DATA_SEPARATOR}{jsonpickle.encode(samples[:10])}\n"
        f"2.0{DATA_SEPARATOR}{jsonpickle.encode(samples[10:])}\n",
        encoding="utf-8",
    )
    fields = calibrate_compass.read_log([path], 2)
    assert len(fields) == 2
    assert fields[1] == [sample.magnetic_fields[1] for sample in samples]
