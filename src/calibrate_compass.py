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

All configured magnetometers are calibrated from the same rotation. Each result is
written to ``MAGNETOMETER.BENCH_CALIBRATION_FILE`` (or ``--output``) with
``{sensor}`` replaced by the magnetometer's name. Changing ``MAGNETOMETER.AXES``
afterwards invalidates it.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import magnetometer
from compass_calibration import bench_calibration_file, fit_ellipsoid
from config import Config
from log import DATA_SEPARATOR
from structs import Vector3

#: samples per second in live mode
LIVE_RATE = 20
#: warn if less than this fraction of the directions has been seen
MIN_COVERAGE = 0.75


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
                try:
                    batch = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                for entry in batch if isinstance(batch, list) else []:
                    fields = _unwrap(entry.get("magnetic_fields"))
                    for index, field in enumerate(fields[:count]):
                        vector = _vector(field)
                        if vector is not None:
                            samples[index].append(vector)
    return samples


def read_live(seconds: float) -> list[list[Vector3]]:
    """Read all magnetometers for the given time."""
    magnetometer_config = Config().MAGNETOMETER
    reader = magnetometer.create_reader(magnetometer_config)
    if reader is None:
        sys.exit(1)
    samples: list[list[Vector3]] = [[] for _ in magnetometer_config.DEVICES]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for index, field in enumerate(reader.read_sample().magnetic_fields):
            samples[index].append(field)
        remaining = end - time.monotonic()
        sys.stdout.write(f"\r{len(samples[0])} samples, {remaining:4.0f} s left ")
        sys.stdout.flush()
        time.sleep(1 / LIVE_RATE)
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
    if covered < MIN_COVERAGE:
        sys.stdout.write("  Warning: turn the unit in more directions and repeat.\n")
    calibration.save(output)
    sys.stdout.write(f"  Written to {output}\n")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bench calibration of the magnetometers.",
    )
    parser.add_argument("logs", nargs="*", type=Path, help="magnetometer data logs")
    parser.add_argument("--live", type=float, help="read the sensors for N seconds")
    parser.add_argument(
        "--output",
        help='where to write the calibration, "{sensor}" is replaced by the name',
    )
    args = parser.parse_args()

    magnetometer_config = Config().MAGNETOMETER
    names = magnetometer.channel_names(magnetometer_config.DEVICES)
    if args.live:
        samples = read_live(args.live)
    elif args.logs:
        samples = read_log(args.logs, len(names))
    else:
        parser.error("give either log files or --live SECONDS")

    pattern = args.output or magnetometer_config.BENCH_CALIBRATION_FILE
    results = [
        calibrate(name, sensor_samples, bench_calibration_file(pattern, name))
        for name, sensor_samples in zip(names, samples, strict=True)
    ]
    if not all(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
