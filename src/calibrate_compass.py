"""
Bench calibration of the magnetometer, done once before the unit is mounted.

It removes the iron that travels with the sensor (board, enclosure, connectors,
screws). Turn the finished, closed unit slowly in every direction while samples are
collected, so that the sensor points every way at least once: roll it over, stand it
on each side, and turn it around in each of those positions. Keep it away from
steel tables, laptops and power cables while doing so.

Usage:

- ``python src/calibrate_compass.py --live 90`` reads the sensors for 90 seconds
- ``python src/calibrate_compass.py data_logs/<run>/compass_raw.log`` uses a recording

All configured magnetometers are calibrated from the same rotation. Each result is
written to ``COMPASS.BENCH_CALIBRATION_FILE`` (or ``--output``) with ``{sensor}``
replaced by the magnetometer's name.
Changing ``COMPASS.MAGNETOMETER_AXES`` afterwards invalidates it.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from compass.calibration import bench_calibration_file, fit_ellipsoid
from config import Config
from log import DATA_SEPARATOR
from utils.linalg import Vector3

#: samples per second in live mode
LIVE_RATE = 20


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


def read_log(paths: list[Path], count: int) -> list[list[Vector3]]:
    """Return the samples of each of ``count`` magnetometers from recordings."""
    samples: list[list[Vector3]] = [[] for _ in range(count)]
    for path in paths:
        with path.open(encoding="utf-8") as file:
            for line in file:
                _, _, payload = line.partition(DATA_SEPARATOR)
                if not payload:
                    continue
                try:
                    data = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                fields = _unwrap(data.get("magnetic_fields"))
                if not isinstance(fields, list):
                    # recorded before more than one magnetometer was supported
                    fields = [data.get("magnetic_field")]
                for index, field in enumerate(fields[:count]):
                    vector = _vector(field)
                    if vector is not None:
                        samples[index].append(vector)
    return samples


def read_live(seconds: float, count: int) -> list[list[Vector3]]:
    """Read all magnetometers for the given time."""
    from compass.worker import SensorReader

    reader = SensorReader(Config().COMPASS)
    samples: list[list[Vector3]] = [[] for _ in range(count)]
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            for index, field in enumerate(reader.read().magnetic_fields):
                samples[index].append(field)
            remaining = end - time.monotonic()
            sys.stdout.write(f"\r{len(samples[0])} samples, {remaining:4.0f} s left ")
            sys.stdout.flush()
            time.sleep(1 / LIVE_RATE)
    finally:
        reader.close()
    sys.stdout.write("\n")
    return samples


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


def calibrate(name: str, samples: list[Vector3], output: str) -> bool:
    """Fit and save the bench calibration of one magnetometer."""
    result = fit_ellipsoid(samples)
    if result is None:
        sys.stdout.write(
            f"{name}: calibration failed with {len(samples)} samples. Turn it more.\n",
        )
        return False

    calibration = result.calibration
    covered = coverage(samples, calibration.offset)
    sys.stdout.write(
        f"{name}: {len(samples)} samples, {covered:.0%} of directions covered\n"
        f"  offset {', '.join(f'{x:.2f}' for x in calibration.offset)} µT\n"
        f"  field strength {calibration.field_strength:.2f} µT, "
        f"residual {result.residual:.2f} µT\n",
    )
    if covered < 0.75:  # noqa: PLR2004
        sys.stdout.write("  Warning: turn the unit in more directions and repeat.\n")
    calibration.save(output)
    sys.stdout.write(f"  Written to {output}\n")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bench calibration of the magnetometers.",
    )
    parser.add_argument("logs", nargs="*", type=Path, help="compass_raw log files")
    parser.add_argument("--live", type=float, help="read the sensors for N seconds")
    parser.add_argument(
        "--output",
        help='where to write the calibration, "{sensor}" is replaced by the name',
    )
    args = parser.parse_args()

    compass_config = Config().COMPASS
    names = compass_config.MAGNETOMETERS
    if args.live:
        samples = read_live(args.live, len(names))
    elif args.logs:
        samples = read_log(args.logs, len(names))
    else:
        parser.error("give either log files or --live SECONDS")

    pattern = args.output or compass_config.BENCH_CALIBRATION_FILE
    results = [
        calibrate(name, sensor_samples, bench_calibration_file(pattern, name))
        for name, sensor_samples in zip(names, samples, strict=True)
    ]
    if not all(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
