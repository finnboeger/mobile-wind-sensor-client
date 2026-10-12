import json
import math
import random
from pathlib import Path

import pytest

from compass.attitude import (
    GRAVITY,
    Attitude,
    AttitudeFilter,
    heading_from_level_field,
    level,
)
from compass.calibration import (
    BenchCalibration,
    ContinuousCalibration,
    GnssAlignment,
    fit_ellipsoid,
)
from compass.drivers import parse_axes
from compass.heading import HeadingFilter, wrap_angle
from compass.processor import CompassProcessor, CompassSettings
from structs import CompassSample, PositionData
from utils.linalg import mat_vec, symmetric_eigen

#: horizontal and vertical earth field in µT, roughly northern Germany
HORIZONTAL_FIELD = 18.0
VERTICAL_FIELD = 46.0


def _ned_to_body(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cp * cy, cp * sy, -sp],
        [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
        [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
    ]


def _body_field(
    roll: float,
    pitch: float,
    heading: float,
    declination: float = 0.0,
) -> tuple[float, float, float]:
    field = [
        HORIZONTAL_FIELD * math.cos(declination),
        HORIZONTAL_FIELD * math.sin(declination),
        VERTICAL_FIELD,
    ]
    x, y, z = mat_vec(_ned_to_body(roll, pitch, heading), field)
    return (x, y, z)


def _degrees(angle: float) -> float:
    return math.degrees(wrap_angle(angle))


# --------------------------------------------------------------------------------------
# Building blocks


def test_tilt_compensated_heading_is_exact() -> None:
    for heading_deg in range(0, 360, 30):
        for roll_deg, pitch_deg in ((0, 0), (25, -10), (-30, 15)):
            heading = math.radians(heading_deg)
            attitude = Attitude(
                roll=math.radians(roll_deg),
                pitch=math.radians(pitch_deg),
            )
            field = _body_field(attitude.roll, attitude.pitch, heading)
            x, y, z = level(field, attitude)
            assert abs(z - VERTICAL_FIELD) < 1e-9
            assert abs(_degrees(heading_from_level_field(x, y) - heading)) < 1e-9


def test_attitude_filter_ignores_accelerations() -> None:
    attitude_filter = AttitudeFilter()
    roll = math.radians(10)
    rng = random.Random(3)
    gravity = mat_vec(_ned_to_body(roll, 0, 0), [0, 0, -GRAVITY])
    errors = []
    for i in range(6000):
        t = i * 0.02
        # waves accelerate the sensor by up to 3 m/s² sideways
        disturbance = 3 * math.sin(2 * math.pi * t / 3) + rng.gauss(0, 0.3)
        force = (gravity[0], gravity[1] + disturbance, gravity[2])
        attitude = attitude_filter.update(0.02, (0.0, 0.0, 0.0), force)
        assert attitude is not None
        if i > 3000:
            errors.append(math.degrees(attitude.roll - roll))
    # an accelerometer alone would be off by up to 17°
    assert max(abs(e) for e in errors) < 3


def test_parse_axes() -> None:
    assert parse_axes("x,-y,-z")((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)
    assert parse_axes("y, x, -z")((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
    with pytest.raises(ValueError, match="each axis once"):
        parse_axes("x,x,z")


def test_symmetric_eigen_reconstructs_matrix() -> None:
    matrix = [[4.0, 1.0, 0.5], [1.0, 3.0, -0.2], [0.5, -0.2, 2.0]]
    values, vectors = symmetric_eigen(matrix)
    for i in range(3):
        for j in range(3):
            rebuilt = sum(vectors[i][k] * values[k] * vectors[j][k] for k in range(3))
            assert abs(rebuilt - matrix[i][j]) < 1e-9


def test_bench_fit_recovers_hard_and_soft_iron() -> None:
    rng = random.Random(5)
    offset = (12.0, -7.0, 20.0)
    distortion = [[1.1, 0.05, 0.0], [0.05, 0.92, 0.03], [0.0, 0.03, 1.02]]
    samples = []
    for _ in range(400):
        roll = rng.uniform(-math.pi, math.pi)
        pitch = rng.uniform(-math.pi / 2, math.pi / 2)
        heading = rng.uniform(0, math.tau)
        true_field = _body_field(roll, pitch, heading)
        distorted = mat_vec(distortion, list(true_field))
        samples.append(
            (
                distorted[0] + offset[0] + rng.gauss(0, 0.05),
                distorted[1] + offset[1] + rng.gauss(0, 0.05),
                distorted[2] + offset[2] + rng.gauss(0, 0.05),
            ),
        )

    result = fit_ellipsoid(samples)
    assert result is not None
    calibration = result.calibration
    for found, expected in zip(calibration.offset, offset, strict=True):
        assert abs(found - expected) < 0.2
    strength = math.hypot(HORIZONTAL_FIELD, VERTICAL_FIELD)
    assert calibration.field_strength is not None
    assert abs(calibration.field_strength - strength) < 0.05 * strength

    # headings from the corrected field are right to well under a degree
    for heading_deg in range(0, 360, 45):
        heading = math.radians(heading_deg)
        attitude = Attitude(roll=0.2, pitch=-0.1)
        true_field = _body_field(attitude.roll, attitude.pitch, heading)
        distorted = mat_vec(distortion, list(true_field))
        raw = (
            distorted[0] + offset[0],
            distorted[1] + offset[1],
            distorted[2] + offset[2],
        )
        x, y, _ = level(calibration.apply(raw), attitude)
        assert abs(_degrees(heading_from_level_field(x, y) - heading)) < 0.5


def test_continuous_calibration_needs_coverage() -> None:
    calibration = ContinuousCalibration()
    # a quarter circle is not enough
    for i in range(500):
        angle = math.radians(i * 90 / 500)
        calibration.add_sample(
            3 + HORIZONTAL_FIELD * math.cos(angle),
            -2 + HORIZONTAL_FIELD * math.sin(angle),
        )
    assert not calibration.refit()

    for i in range(2000):
        angle = math.radians(i * 360 / 2000)
        calibration.add_sample(
            3 + HORIZONTAL_FIELD * math.cos(angle),
            -2 + HORIZONTAL_FIELD * math.sin(angle),
        )
    assert calibration.refit()
    offset = calibration.calibration.offset
    assert abs(offset[0] - 3) < 0.1
    assert abs(offset[1] + 2) < 0.1


def test_gnss_alignment_cancels_current() -> None:
    alignment = GnssAlignment()
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


def test_heading_filter_rejects_short_disturbance() -> None:
    heading_filter = HeadingFilter()
    timestamp = 0
    for _ in range(500):
        timestamp += 20
        heading_filter.predict(0.02, 0.0)
        heading_filter.update(timestamp, math.radians(100))
    # a passing steel boat pulls the magnetometer 40° off for a second
    for _ in range(50):
        timestamp += 20
        heading_filter.predict(0.02, 0.0)
        heading_filter.update(timestamp, math.radians(140))
    assert heading_filter.heading is not None
    assert abs(math.degrees(heading_filter.heading) - 100) < 1


# --------------------------------------------------------------------------------------
# Simulated RIB


def _simulate(  # noqa: PLR0913
    *,
    continuous_calibration: bool = True,
    gnss_alignment: bool = True,
    hard_iron: tuple[float, float, float] = (6.0, -4.0, 2.0),
    soft_iron: tuple[tuple[float, float, float], ...] = (
        (1.04, 0.02, 0.0),
        (0.02, 0.97, 0.0),
        (0.0, 0.0, 1.0),
    ),
    declination: float = math.radians(3),
    seed: int = 1,
) -> tuple[CompassProcessor, list[float]]:
    """
    Drive a RIB around for about ten minutes and return the heading errors.

    The boat goes straight on a number of headings at 8 m/s with turns at 15°/s in
    between, rolls and pitches in a short sea, and drifts in a current. The
    magnetometer sees hard and soft iron the processor doesn't know about, and the
    gyro has a bias.
    """
    rng = random.Random(seed)
    processor = CompassProcessor(
        settings=CompassSettings(
            continuous_calibration=continuous_calibration,
            gnss_alignment=gnss_alignment,
        ),
    )
    rate_hz = 50
    dt = 1 / rate_hz
    speed = 8.0
    current = (0.3, -0.4)
    gyro_bias = (math.radians(0.2), math.radians(-0.1), math.radians(0.3))
    turn_rate = math.radians(15)

    targets = [90, 200, 300, 45, 160, 250, 330, 120, 20, 270, 180, 0]
    heading = 0.0
    timestamp = 1_700_000_000_000
    errors: list[float] = []
    step = 0
    for target_deg in targets:
        target = math.radians(target_deg)
        straight_steps = 40 * rate_hz
        while True:
            t = step * dt
            step += 1
            timestamp += 1000 // rate_hz

            remaining = wrap_angle(target - heading)
            turning = abs(remaining) > turn_rate * dt
            if turning:
                yaw_rate = math.copysign(turn_rate, remaining)
            else:
                yaw_rate = 0.0
                heading = target
                straight_steps -= 1
                if straight_steps <= 0:
                    break
            heading = (heading + yaw_rate * dt) % math.tau

            # short sea: roll ±6°, pitch ±4°
            roll = math.radians(6) * math.sin(2 * math.pi * t / 3.1)
            pitch = math.radians(4) * math.sin(2 * math.pi * t / 2.3 + 1)
            roll_rate = (
                math.radians(6)
                * 2
                * math.pi
                / 3.1
                * math.cos(
                    2 * math.pi * t / 3.1,
                )
            )
            pitch_rate = (
                math.radians(4)
                * 2
                * math.pi
                / 2.3
                * math.cos(
                    2 * math.pi * t / 2.3 + 1,
                )
            )
            body_rates = (
                roll_rate - yaw_rate * math.sin(pitch),
                pitch_rate * math.cos(roll)
                + yaw_rate * math.sin(roll) * math.cos(pitch),
                -pitch_rate * math.sin(roll)
                + yaw_rate * math.cos(roll) * math.cos(pitch),
            )
            gyro = (
                body_rates[0] + gyro_bias[0] + rng.gauss(0, 0.003),
                body_rates[1] + gyro_bias[1] + rng.gauss(0, 0.003),
                body_rates[2] + gyro_bias[2] + rng.gauss(0, 0.003),
            )

            gravity = mat_vec(_ned_to_body(roll, pitch, heading), [0, 0, -GRAVITY])
            waves = (
                rng.gauss(0, 0.5),
                rng.gauss(0, 0.5),
                1.5 * math.sin(2 * math.pi * t / 2.3) + rng.gauss(0, 0.5),
            )
            acceleration = (
                gravity[0] + waves[0],
                gravity[1] + speed * yaw_rate + waves[1],
                gravity[2] + waves[2],
            )

            true_field = _body_field(roll, pitch, heading, declination)
            distorted = mat_vec([list(row) for row in soft_iron], list(true_field))
            magnetic_field = (
                distorted[0] + hard_iron[0] + rng.gauss(0, 0.05),
                distorted[1] + hard_iron[1] + rng.gauss(0, 0.05),
                distorted[2] + hard_iron[2] + rng.gauss(0, 0.05),
            )

            output = processor.process(
                CompassSample(
                    timestamp=timestamp,
                    acceleration=acceleration,
                    angular_rate=gyro,
                    magnetic_field=magnetic_field,
                ),
            )

            if step % (rate_hz // 2) == 0:
                velocity_north = speed * math.cos(heading) + current[0]
                velocity_east = speed * math.sin(heading) + current[1]
                processor.add_position(
                    PositionData(
                        timestamp=timestamp,
                        valid=True,
                        latitude=54.0,
                        longitude=10.0,
                        altitude=0.0,
                        geoidal_separation=0.0,
                        speed=math.hypot(velocity_north, velocity_east),
                        true_course=math.degrees(
                            math.atan2(velocity_east, velocity_north),
                        ),
                        velocity_north=velocity_north,
                        velocity_east=velocity_east,
                        speed_accuracy=0.1,
                        source="NAV-PVT",
                    ),
                )

            if output is not None:
                errors.append(_degrees(output.heading - heading))
    return processor, errors


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values))


def test_rib_converges_without_manual_calibration() -> None:
    processor, errors = _simulate()
    assert processor.calibration.converged
    assert processor.alignment.offset is not None
    last_two_minutes = errors[-6000:]
    assert _rms(last_two_minutes) < 2
    assert max(abs(e) for e in last_two_minutes) < 4


def test_uncalibrated_heading_is_far_off() -> None:
    _, errors = _simulate(continuous_calibration=False, gnss_alignment=False)
    assert max(abs(e) for e in errors[-6000:]) > 15


def test_calibration_state_round_trip(tmp_path: Path) -> None:
    processor, _ = _simulate()
    path = tmp_path / "compass.json"
    path.write_text(json.dumps(processor.state_dict()), encoding="utf-8")

    restored = CompassProcessor(bench_calibration=BenchCalibration())
    restored.load_state_dict(json.loads(path.read_text(encoding="utf-8")))
    assert restored.calibration.calibration == processor.calibration.calibration
    assert restored.alignment.offset == processor.alignment.offset
