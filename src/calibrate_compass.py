"""
Bench calibration of the magnetometers, done once before the unit is mounted.

It removes the iron that travels with the sensors (board, enclosure, connectors,
screws). Turn the finished, closed unit slowly in every direction while samples are
collected, so that it points every way at least once: roll it over, stand it on each
side, and turn it around in each of those positions. Keep it away from steel tables,
laptops and power cables while doing so.

Usage, from the ``src`` directory with the config.ini of the unit:

- ``python calibrate_compass.py --live 90`` reads the magnetometers for 90 seconds
- ``python calibrate_compass.py <data log dir>/magnetometer.log`` uses a recording

All configured magnetometers are calibrated from the same rotation, which also
gives the angles between them: every further magnetometer is turned into the axes of
the first one, so that the compass can compare their fields. Each result is written
to ``MAGNETOMETER.BENCH_CALIBRATION_FILE`` (or ``--output``) with ``{sensor}``
replaced by the magnetometer's name. Changing ``MAGNETOMETER.AXES`` afterwards
invalidates it.

If the IMU is read during the same rotation (live, or ``--imu <data log dir>/imu.log``
with recordings), the angle between the magnetometers and the IMU is fitted as well
and included in the calibration. This removes the tilt error from a magnetometer that
sits slightly crooked, or whose sensing axes aren't exactly square, e.g. a leaning Z
coil on an RM3100. Pause briefly in each position, the accelerometer is only used
while the unit is held still.
"""

import argparse
import bisect
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import imu
import magnetometer
from compass_calibration import (
    BenchCalibration,
    align_bench_calibration,
    bench_calibration_file,
    fit_ellipsoid,
    fit_imu_alignment,
    fit_rotation,
    rotation_angle,
)
from config import Config
from log import DATA_SEPARATOR
from structs import Vector3

#: samples per second in live mode
LIVE_RATE = 20
#: warn if less than this fraction of the directions has been seen
MIN_COVERAGE = 0.75
#: standard gravity in m/s²
GRAVITY = 9.80665
#: the accelerometer counts as showing gravity alone while the specific force is
#: within this fraction of gravity and the unit turns slower than this rate in rad/s
STILL_TOLERANCE = 0.02
STILL_RATE = 0.3
#: an IMU sample belongs to a magnetometer reading if they are this close in ms
MATCH_MILLISECONDS = 30
#: a larger angle between magnetometer and IMU points to a wrong axis mapping
MAX_ALIGNMENT_DEGREES = 15.0


def _unwrap(value: Any) -> Any:  # noqa: ANN401
    """Undo jsonpickle's tuple encoding."""
    if isinstance(value, dict) and "py/tuple" in value:
        return value["py/tuple"]
    return value


def _vector(value: Any) -> Vector3 | None:  # noqa: ANN401
    value = _unwrap(value)
    if isinstance(value, list) and len(value) == 3:  # noqa: PLR2004
        return (float(value[0]), float(value[1]), float(value[2]))
    return None


def _log_entries(paths: list[Path]) -> list[dict[str, Any]]:
    """Return the records of data logs, in the order they were written."""
    entries: list[dict[str, Any]] = []
    for path in paths:
        with path.open(encoding="utf-8") as file:
            for line in file:
                _, _, payload = line.partition(DATA_SEPARATOR)
                try:
                    batch = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                entries += (
                    [e for e in batch if isinstance(e, dict)]
                    if isinstance(
                        batch,
                        list,
                    )
                    else []
                )
    return entries


def read_log(paths: list[Path], count: int) -> list[list[Vector3]]:
    """Return the samples of each of ``count`` magnetometers from recordings."""
    return read_log_with_times(paths, count)[0]


def read_log_with_times(
    paths: list[Path],
    count: int,
) -> tuple[list[list[Vector3]], list[int]]:
    """
    Return the samples of each magnetometer and the times of the first one's.

    :return: samples per magnetometer, and the timestamp in ms of each sample of the
        first magnetometer
    """
    samples: list[list[Vector3]] = [[] for _ in range(count)]
    times: list[int] = []
    for entry in _log_entries(paths):
        fields = _unwrap(entry.get("magnetic_fields")) or []
        for index, field in enumerate(fields[:count]):
            vector = _vector(field)
            if vector is not None:
                samples[index].append(vector)
                if index == 0:
                    times.append(int(entry.get("timestamp", 0)))
    return samples, times


def read_imu_log(paths: list[Path]) -> list[tuple[int, Vector3 | None]]:
    """Return the time in ms and the direction down (if still) of each IMU sample."""
    samples: list[tuple[int, Vector3 | None]] = []
    for entry in _log_entries(paths):
        acceleration = _vector(entry.get("acceleration"))
        angular_rate = _vector(entry.get("angular_rate"))
        if acceleration is not None and angular_rate is not None:
            samples.append(
                (
                    int(entry.get("timestamp", 0)),
                    still_down(acceleration, angular_rate),
                ),
            )
    return samples


def still_down(acceleration: Vector3, angular_rate: Vector3) -> Vector3 | None:
    """
    Return the direction down from an IMU sample, or None if the unit moved.

    While still, the accelerometer reads the specific force, which points up.
    """
    strength = math.hypot(*acceleration)
    if abs(strength - GRAVITY) > STILL_TOLERANCE * GRAVITY:
        return None
    if math.hypot(*angular_rate) > STILL_RATE:
        return None
    x, y, z = (-a / strength for a in acceleration)
    return (x, y, z)


def match_downs(
    times: list[int],
    imu_samples: list[tuple[int, Vector3 | None]],
) -> list[Vector3 | None]:
    """Return the direction down at each magnetometer time, None if not known."""
    imu_samples = sorted(imu_samples, key=lambda sample: sample[0])
    imu_times = [timestamp for timestamp, _ in imu_samples]
    downs: list[Vector3 | None] = []
    for timestamp in times:
        index = bisect.bisect_left(imu_times, timestamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(imu_times)]
        best = min(
            candidates,
            key=lambda i: abs(imu_times[i] - timestamp),
            default=None,
        )
        if best is None or abs(imu_times[best] - timestamp) > MATCH_MILLISECONDS:
            downs.append(None)
        else:
            downs.append(imu_samples[best][1])
    return downs


def _create_imu_reader() -> imu.ImuReader | None:
    imu_config = Config().IMU
    if imu_config.DEVICE not in imu.DEVICES:
        sys.stdout.write(f"Unknown IMU {imu_config.DEVICE}, not aligning with it\n")
        return None
    try:
        return imu.ImuReader(
            imu.DEVICES[imu_config.DEVICE](imu_config),
            imu_config.AXES,
        )
    except OSError as error:
        sys.stdout.write(f"Can't read the IMU ({error}), not aligning with it\n")
        return None


def read_live(seconds: float) -> tuple[list[list[Vector3]], list[Vector3 | None]]:
    """
    Read all magnetometers, and the IMU if possible, for the given time.

    :return: samples per magnetometer, and the direction down at each sample (None
        while moving or without IMU)
    """
    magnetometer_config = Config().MAGNETOMETER
    reader = magnetometer.create_reader(magnetometer_config)
    if reader is None:
        sys.exit(1)
    imu_reader = _create_imu_reader()
    samples: list[list[Vector3]] = [[] for _ in magnetometer_config.DEVICES]
    downs: list[Vector3 | None] = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for index, field in enumerate(reader.read_sample().magnetic_fields):
            samples[index].append(field)
        down = None
        if imu_reader is not None:
            try:
                sample = imu_reader.read_sample()
                down = still_down(sample.acceleration, sample.angular_rate)
            except OSError:
                pass
        downs.append(down)
        still = sum(d is not None for d in downs)
        remaining = end - time.monotonic()
        sys.stdout.write(
            f"\r{len(samples[0])} samples ({still} still), {remaining:4.0f} s left ",
        )
        sys.stdout.flush()
        time.sleep(1 / LIVE_RATE)
    sys.stdout.write("\n")
    return samples, downs


def coverage(samples: list[Vector3], center: Vector3) -> float:
    """Return the fraction of 20 directions around the centre that have samples."""
    # vertices of a dodecahedron, roughly evenly spread directions
    phi = (1 + math.sqrt(5)) / 2
    directions: list[Vector3] = [
        (x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)
    ]
    for a in (-1, 1):
        for b in (-1, 1):
            directions += [
                (0, a / phi, b * phi),
                (a / phi, b * phi, 0),
                (a * phi, 0, b / phi),
            ]
    seen = set()
    for sample in samples:
        offset = (sample[0] - center[0], sample[1] - center[1], sample[2] - center[2])
        scores = [
            sum(o * d for o, d in zip(offset, direction, strict=True))
            for direction in directions
        ]
        seen.add(scores.index(max(scores)))
    return len(seen) / len(directions)


def calibrate(name: str, samples: list[Vector3]) -> BenchCalibration | None:
    """Fit the bench calibration of one magnetometer."""
    result = fit_ellipsoid(samples)
    if result is None:
        sys.stdout.write(
            f"{name}: calibration failed with {len(samples)} samples. Turn it more.\n",
        )
        return None

    calibration = result.calibration
    covered = coverage(samples, calibration.offset)
    sys.stdout.write(
        f"{name}: {len(samples)} samples, {covered:.0%} of directions covered\n"
        f"  offset {', '.join(f'{x:.2f}' for x in calibration.offset)} µT\n"
        f"  field strength {calibration.field_strength:.2f} µT, "
        f"residual {result.residual:.2f} µT\n",
    )
    if covered < MIN_COVERAGE:
        sys.stdout.write("  Warning: turn the unit in more directions and repeat.\n")
    return calibration


def align(
    name: str,
    calibration: BenchCalibration,
    samples: list[Vector3],
    reference: BenchCalibration,
    reference_samples: list[Vector3],
) -> BenchCalibration | None:
    """
    Turn a magnetometer's calibrated field into the axes of the reference.

    Both were turned together, so after their calibrations they measured the same
    field at the same time, only in axes that differ by how the chips sit on the
    board. Once aligned, the two can be compared vector by vector.
    """
    if len(samples) != len(reference_samples):
        sys.stdout.write(f"  {name}: samples don't match the reference, not aligned\n")
        return None
    rotation = fit_rotation(
        [calibration.apply(sample) for sample in samples],
        [reference.apply(sample) for sample in reference_samples],
    )
    if rotation is None:
        return None
    sys.stdout.write(
        f"  {name}: turned by {math.degrees(rotation_angle(rotation)):.1f}° "
        "into the axes of the first magnetometer\n",
    )
    return align_bench_calibration(calibration, rotation)


def align_with_imu(
    calibrations: list[BenchCalibration | None],
    samples: list[Vector3],
    downs: list[Vector3 | None],
) -> list[BenchCalibration | None]:
    """
    Turn all calibrations into the IMU's axes, fitted on the first magnetometer.

    The others are already turned into the first one's axes, so the same rotation
    applies to all of them.
    """
    reference = calibrations[0]
    if reference is None:
        return calibrations
    pairs = [
        (reference.apply(sample), down)
        for sample, down in zip(samples, downs, strict=True)
        if down is not None
    ]
    result = fit_imu_alignment([f for f, _ in pairs], [d for _, d in pairs])
    if result is None:
        sys.stdout.write(
            f"IMU alignment: not enough still positions ({len(pairs)} samples). "
            "Pause briefly in more orientations and repeat, the calibration is "
            "written without it.\n",
        )
        return calibrations
    angle = math.degrees(rotation_angle(result.rotation))
    sys.stdout.write(
        f"IMU alignment: turned by {angle:.1f}° into the axes of the IMU, "
        f"inclination {math.degrees(result.inclination):.1f}° "
        f"(± {math.degrees(result.residual):.1f}°)\n",
    )
    if angle > MAX_ALIGNMENT_DEGREES:
        sys.stdout.write(
            "  That's too far for a crooked chip. Check IMU.AXES and "
            "MAGNETOMETER.AXES, the calibration is written without it.\n",
        )
        return calibrations
    return [
        None if c is None else align_bench_calibration(c, result.rotation)
        for c in calibrations
    ]


def calibrate_all(
    names: list[str],
    samples: list[list[Vector3]],
    downs: list[Vector3 | None] | None = None,
) -> list[BenchCalibration | None]:
    """
    Calibrate every magnetometer and align them with the first one.

    :param downs: direction down from the IMU at each sample of the first
        magnetometer (None while moving), to align the magnetometers with the IMU
    """
    calibrations = [
        calibrate(name, sensor_samples)
        for name, sensor_samples in zip(names, samples, strict=True)
    ]
    reference = calibrations[0]
    if reference is None:
        return calibrations
    for index in range(1, len(names)):
        calibration = calibrations[index]
        if calibration is not None:
            calibrations[index] = align(
                names[index],
                calibration,
                samples[index],
                reference,
                samples[0],
            )
    if downs is not None and any(d is not None for d in downs):
        calibrations = align_with_imu(calibrations, samples[0], downs)
    return calibrations


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bench calibration of the magnetometers.",
    )
    parser.add_argument("logs", nargs="*", type=Path, help="magnetometer data logs")
    parser.add_argument("--live", type=float, help="read the sensors for N seconds")
    parser.add_argument(
        "--imu",
        nargs="+",
        type=Path,
        default=[],
        help="IMU data logs recorded together with the magnetometer logs",
    )
    parser.add_argument(
        "--output",
        help='where to write the calibration, "{sensor}" is replaced by the name',
    )
    args = parser.parse_args()

    magnetometer_config = Config().MAGNETOMETER
    names = magnetometer.channel_names(magnetometer_config.DEVICES)
    downs: list[Vector3 | None] | None = None
    if args.live:
        samples, downs = read_live(args.live)
    elif args.logs:
        samples, times = read_log_with_times(args.logs, len(names))
        if args.imu:
            downs = match_downs(times, read_imu_log(args.imu))
    else:
        parser.error("give either log files or --live SECONDS")

    pattern = args.output or magnetometer_config.BENCH_CALIBRATION_FILE
    calibrations = calibrate_all(names, samples, downs)
    for name, calibration in zip(names, calibrations, strict=True):
        if calibration is not None:
            output = bench_calibration_file(pattern, name)
            calibration.save(output)
            sys.stdout.write(f"{name}: written to {output}\n")
    if not all(calibrations):
        sys.exit(1)


if __name__ == "__main__":
    main()
