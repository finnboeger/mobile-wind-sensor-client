import math
import random
import threading
from collections.abc import Callable
from pathlib import Path
from queue import Queue
from typing import Any

import jsonpickle
import pytest

import calibrate_compass
import compass
import compass_calibration
from log import DATA_SEPARATOR
from structs import HeadingData, ImuSample, MagnetometerSample, PositionData, Vector3
from tests.simulation import (
    EARTH_FIELD,
    Magnetometer,
    RibCourse,
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


def _errors(  # noqa: PLR0913
    processor: compass.CompassProcessor,
    imu_samples: list[ImuSample],
    magnetometer_samples: list[MagnetometerSample],
    truth: Callable[[float], float],
    after: float,
    positions: list[PositionData] | None = None,
) -> list[float]:
    headings = processor.add(imu_samples, magnetometer_samples, positions)
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


def test_world_to_unit_rotation() -> None:
    for heading, pitch, roll in _random_orientations(20, 3):
        vector = (1.0, -2.0, 3.0)
        expected = rotate(world_to_unit(heading, pitch, roll), vector)
        found = compass.world_to_unit(vector, heading, roll, pitch)
        assert all(abs(a - b) < 1e-9 for a, b in zip(found, expected, strict=True))


def _rib_errors(course: RibCourse, *, use_gnss: bool) -> list[float]:
    imu_samples, magnetometer_samples = simulate(
        course.attitude,
        course.duration,
        boat_acceleration=course.acceleration,
    )
    return _errors(
        compass.CompassProcessor({"mock": None}),
        imu_samples,
        magnetometer_samples,
        course.heading,
        after=5,
        positions=course.positions() if use_gnss else None,
    )


def test_gnss_speed_removes_the_turn_from_the_attitude() -> None:
    course = RibCourse(targets=[0, 180, 0, 120], straight_time=30)
    with_gnss = _rib_errors(course, use_gnss=True)
    without_gnss = _rib_errors(course, use_gnss=False)
    # each turn tilts the attitude towards the outside, 2.6 times that in the heading
    assert max(abs(e) for e in without_gnss) > 5
    assert max(abs(e) for e in with_gnss) < 1.5


def test_continuous_calibration_needs_coverage() -> None:
    calibration = compass_calibration.ContinuousCalibration()
    radius = EARTH_FIELD[0]
    # a quarter circle is not enough
    for i in range(500):
        angle = math.radians(i * 90 / 500)
        calibration.add_sample(
            3 + radius * math.cos(angle),
            -2 + radius * math.sin(angle),
        )
    assert not calibration.refit()

    for i in range(2000):
        angle = math.radians(i * 360 / 2000)
        calibration.add_sample(
            3 + radius * math.cos(angle),
            -2 + radius * math.sin(angle),
        )
    assert calibration.refit()
    offset = calibration.calibration.offset
    assert abs(offset[0] - 3) < 0.1
    assert abs(offset[1] + 2) < 0.1


#: a RIB going around for about ten minutes
RIB = RibCourse(
    targets=[0, 90, 200, 300, 45, 160, 250, 330, 120, 20, 270, 180, 0],
    current=(0.3, -0.4),
)
#: iron of the boat near the unit
BOAT_IRON = Magnetometer(
    hard_iron=(6.0, -4.0, 2.0),
    soft_iron=[[1.04, 0.02, 0.0], [0.02, 0.97, 0.0], [0.0, 0.0, 1.0]],
    noise=0.05,
)


def _run_rib(
    processor: compass.CompassProcessor,
    magnetometers: list[Magnetometer],
    course: RibCourse = RIB,
    earth_field: Vector3 = EARTH_FIELD,
    after: float | None = None,
) -> list[float]:
    """Return the heading errors in degrees, by default in the last two minutes."""
    imu_samples, magnetometer_samples = simulate(
        course.attitude,
        course.duration,
        magnetometers=magnetometers,
        earth_field=earth_field,
        boat_acceleration=course.acceleration,
        gyro_bias=(math.radians(0.2), math.radians(-0.1), math.radians(0.3)),
    )
    return _errors(
        processor,
        imu_samples,
        magnetometer_samples,
        course.heading,
        after=course.duration - 120 if after is None else after,
        positions=course.positions(),
    )


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values))


def test_continuous_calibration_learns_the_boat_iron() -> None:
    processor = compass.CompassProcessor({"sensor": None})
    errors = _run_rib(processor, [BOAT_IRON])
    assert processor.calibrated
    assert _rms(errors) < 2
    assert max(abs(e) for e in errors) < 4

    uncalibrated = _run_rib(
        compass.CompassProcessor(
            {"sensor": None},
            compass.CompassSettings(continuous_calibration=False),
        ),
        [BOAT_IRON],
    )
    assert max(abs(e) for e in uncalibrated) > 15


def test_calibration_state_round_trip(tmp_path: Path) -> None:
    processor = compass.CompassProcessor({"sensor": None})
    _run_rib(processor, [BOAT_IRON])
    path = tmp_path / "compass.json"
    processor.save(path)
    assert not processor.calibration_changed

    restored = compass.CompassProcessor({"sensor": None})
    restored.load(path)
    assert (
        restored.channels[0].calibration.calibration
        == processor.channels[0].calibration.calibration
    )
    # a missing file starts a new calibration
    compass.CompassProcessor({"sensor": None}).load(tmp_path / "missing.json")


def test_gnss_alignment_cancels_current() -> None:
    alignment = compass_calibration.GnssAlignment()
    true_offset = math.radians(4)
    speed = 4.0
    current = (0.0, 0.8)  # 0.8 m/s towards east
    for heading_deg in range(0, 360, 2):
        heading = math.radians(heading_deg)
        for _ in range(20):
            course = heading + true_offset
            velocity_north = speed * math.cos(course) + current[0]
            velocity_east = speed * math.sin(course) + current[1]
            alignment.add_sample(heading, math.atan2(velocity_east, velocity_north))
    assert alignment.offset is not None
    # the current alone shifts single headings by up to 11°
    assert abs(math.degrees(alignment.offset - true_offset)) < 0.5


def _declined_field(declination: float) -> Vector3:
    horizontal, _, down = EARTH_FIELD
    return (
        horizontal * math.cos(math.radians(declination)),
        horizontal * math.sin(math.radians(declination)),
        down,
    )


def test_gnss_alignment_corrects_a_wrong_declination() -> None:
    # magnetic north is 4° east, the setting says 1°
    processor = compass.CompassProcessor(
        {"sensor": None},
        compass.CompassSettings(declination=math.radians(1)),
    )
    errors = _run_rib(processor, [BOAT_IRON], earth_field=_declined_field(4))
    assert processor.alignment.offset is not None
    assert abs(math.degrees(processor.alignment.offset) - 3) < 1
    assert _rms(errors) < 2

    unaligned = compass.CompassProcessor(
        {"sensor": None},
        compass.CompassSettings(declination=math.radians(1), gnss_alignment=False),
    )
    errors = _run_rib(unaligned, [BOAT_IRON], earth_field=_declined_field(4))
    assert abs(sum(errors) / len(errors) + 3) < 1


def test_bow_angle_is_taken_into_account() -> None:
    # the unit's forward points 30° to the right of the bow, so the boat moves 30°
    # to the left of where the unit points
    course = RibCourse(targets=RIB.targets, current=(0.3, -0.4))
    processor = compass.CompassProcessor(
        {"sensor": None},
        compass.CompassSettings(bow_angle=math.radians(-30)),
    )
    imu_samples, magnetometer_samples = simulate(
        lambda time: (
            (course.heading(time) + math.radians(30)) % math.tau,
            *course.attitude(time)[1:],
        ),
        course.duration,
        magnetometers=[BOAT_IRON],
        boat_acceleration=course.acceleration,
    )
    processor.add(imu_samples, magnetometer_samples, course.positions())
    assert processor.alignment.offset is not None
    assert abs(math.degrees(processor.alignment.offset)) < 1


def _current_near_second_sensor(time: float) -> Vector3:
    """A current near the second sensor adds 6 µT sideways for 30 s, near the end."""
    if RIB.duration - 100 <= time < RIB.duration - 70:
        return (0.0, 6.0, 0.0)
    return (0.0, 0.0, 0.0)


def _boat_iron(hard_iron: Vector3, **kwargs: Any) -> Magnetometer:  # noqa: ANN401
    return Magnetometer(
        hard_iron=hard_iron,
        soft_iron=BOAT_IRON.soft_iron,
        noise=BOAT_IRON.noise,
        **kwargs,
    )


def test_second_magnetometer_catches_a_local_disturbance() -> None:
    first = _boat_iron((6.0, -4.0, 2.0))
    second = _boat_iron((-3.0, 5.0, -1.0), disturbance=_current_near_second_sensor)
    processor = compass.CompassProcessor({"first": None, "second": None})
    errors = _run_rib(processor, [first, second])
    assert all(channel.calibration.converged for channel in processor.channels)
    assert _rms(errors) < 2
    assert max(abs(e) for e in errors) < 4

    # with only the disturbed sensor the heading is pulled off by about 20°
    single = _run_rib(compass.CompassProcessor({"second": None}), [second])
    assert max(abs(e) for e in single) > 10


def test_field_strength_check() -> None:
    channel = compass.MagnetometerChannel("sensor")
    assert channel.field_ok(0, (50.0, 0.0, 0.0))
    assert channel.field_ok(100, (52.0, 0.0, 0.0))
    # a magnet nearby
    assert not channel.field_ok(200, (70.0, 0.0, 0.0))
    # still there after a minute: the new normal
    assert not channel.field_ok(30_000, (70.0, 0.0, 0.0))
    assert not channel.field_ok(61_000, (70.0, 0.0, 0.0))
    assert channel.field_ok(61_100, (70.0, 0.0, 0.0))


def test_fit_rotation_recovers_a_rotation() -> None:
    rotation = world_to_unit(math.radians(3), math.radians(-2), math.radians(1.5))
    rng = random.Random(2)
    source = [
        rotate(world_to_unit(*orientation), EARTH_FIELD)
        for orientation in _random_orientations(200, 4)
    ]
    target = [
        tuple(x + rng.gauss(0, 0.05) for x in rotate(rotation, vector))
        for vector in source
    ]
    found = compass_calibration.fit_rotation(source, target)  # type: ignore[arg-type]
    assert found is not None
    for i in range(3):
        for j in range(3):
            assert abs(found[i][j] - rotation[i][j]) < 1e-3
    angle = compass_calibration.rotation_angle(found)
    assert 3 < math.degrees(angle) < 4.5


def _rotated_chip(degrees: tuple[float, float, float], **kwargs: Any) -> Magnetometer:  # noqa: ANN401
    return Magnetometer(
        rotation=world_to_unit(*(math.radians(d) for d in degrees)),
        noise=0.05,
        **kwargs,
    )


def _angle_between(a: Vector3, b: Vector3) -> float:
    """Return the angle between two vectors in degrees."""
    cosine = sum(x * y for x, y in zip(a, b, strict=True)) / (
        math.dist(a, (0, 0, 0)) * math.dist(b, (0, 0, 0))
    )
    return math.degrees(math.acos(min(1.0, cosine)))


def test_bench_calibration_aligns_the_second_magnetometer() -> None:
    first = _rotated_chip((1, 0.5, -1), hard_iron=(5.0, 3.0, -8.0))
    second = _rotated_chip((-3, 2, 1), hard_iron=HARD_IRON, soft_iron=SOFT_IRON)
    rng = random.Random(3)
    samples: list[list[Vector3]] = [[], []]
    for orientation in _random_orientations(400, 8):
        field = rotate(world_to_unit(*orientation), EARTH_FIELD)
        samples[0].append(first.measure(field, 0, rng))
        samples[1].append(second.measure(field, 0, rng))

    aligned = calibrate_compass.calibrate_all(["first", "second"], samples)
    unaligned = [compass_calibration.fit_ellipsoid(s) for s in samples]
    assert aligned[0] is not None
    assert aligned[1] is not None
    assert unaligned[1] is not None

    reference = aligned[0]

    def largest_angle(calibration: compass_calibration.BenchCalibration) -> float:
        return max(
            _angle_between(calibration.apply(b), reference.apply(a))
            for a, b in zip(samples[0], samples[1], strict=True)
        )

    # the chips sit about 4° apart, after the alignment their fields agree
    assert largest_angle(unaligned[1].calibration) > 3
    assert largest_angle(aligned[1]) < 0.5


#: the RIB's turns, then a long straight run on which the MEMS sensor drifts
LONG_RUN = RibCourse(
    targets=RIB.targets,
    current=RIB.current,
    final_straight_time=900,
)
DRIFT_START = LONG_RUN.duration - 900


def _drifting_offset(time: float) -> Vector3:
    """The MEMS sensor's offset drifts by 4 µT sideways and 3 µT down."""
    progress = min(1.0, max(0.0, (time - DRIFT_START) / 600))
    return (0.0, 4.0 * progress, 3.0 * progress)


def _drift_errors(settings: compass.CompassSettings) -> list[float]:
    rm3100 = _boat_iron((0.0, 0.0, 0.0))
    mems = _boat_iron((0.0, 0.0, 0.0), disturbance=_drifting_offset)
    processor = compass.CompassProcessor({"rm3100": None, "mems": None}, settings)
    return _run_rib(processor, [rm3100, mems], course=LONG_RUN, after=DRIFT_START)


def test_drift_of_the_mems_sensor_is_followed() -> None:
    tracked = _drift_errors(compass.CompassSettings())
    untracked = _drift_errors(compass.CompassSettings(drift_tracking=False))
    # without tracking the average of both drifts away by half the MEMS sensor's
    # error, until they disagree by more than 8° and only the RM3100 is used
    assert max(abs(e) for e in untracked) > 2.5
    assert max(abs(e) for e in tracked) < 1


def test_drift_tracker_ignores_a_short_disturbance() -> None:
    tracker = compass_calibration.DriftTracker()
    for second in range(600):
        tracker.add(second * 1000, (1.0, 0.0, 0.0))
    for second in range(600, 630):
        tracker.add(second * 1000, (1.0, 8.0, 0.0))
    assert math.dist(tracker.offset, (1.0, 0.0, 0.0)) < 0.01
    # a lasting change is accepted after ten minutes
    for second in range(630, 1300):
        tracker.add(second * 1000, (1.0, 8.0, 0.0))
    assert math.dist(tracker.offset, (1.0, 8.0, 0.0)) < 0.5
