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

The result is written to ``COMPASS.BENCH_CALIBRATION_FILE`` (or ``--output``).
Changing ``COMPASS.MAGNETOMETER_AXES`` afterwards invalidates it.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from compass.calibration import fit_ellipsoid
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


def read_log(paths: list[Path]) -> list[Vector3]:
    samples: list[Vector3] = []
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
                field = _unwrap(data.get("magnetic_field"))
                if isinstance(field, list) and len(field) == 3:  # noqa: PLR2004
                    samples.append((float(field[0]), float(field[1]), float(field[2])))
    return samples


def read_live(seconds: float) -> list[Vector3]:
    from compass.worker import SensorReader

    reader = SensorReader(Config().COMPASS)
    samples: list[Vector3] = []
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            samples.append(reader.read().magnetic_field)
            remaining = end - time.monotonic()
            sys.stdout.write(f"\r{len(samples)} samples, {remaining:4.0f} s left ")
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bench calibration of the magnetometer.",
    )
    parser.add_argument("logs", nargs="*", type=Path, help="compass_raw log files")
    parser.add_argument("--live", type=float, help="read the sensors for N seconds")
    parser.add_argument("--output", type=Path, help="where to write the calibration")
    args = parser.parse_args()

    if args.live:
        samples = read_live(args.live)
    elif args.logs:
        samples = read_log(args.logs)
    else:
        parser.error("give either log files or --live SECONDS")

    result = fit_ellipsoid(samples)
    if result is None:
        sys.exit(f"Calibration failed with {len(samples)} samples. Turn it more.")

    calibration = result.calibration
    covered = coverage(samples, calibration.offset)
    sys.stdout.write(
        f"{len(samples)} samples, {covered:.0%} of directions covered\n"
        f"offset {', '.join(f'{x:.2f}' for x in calibration.offset)} µT\n"
        f"field strength {calibration.field_strength:.2f} µT, "
        f"residual {result.residual:.2f} µT\n",
    )
    if covered < 0.75:  # noqa: PLR2004
        sys.stdout.write("Warning: turn the unit in more directions and repeat.\n")

    output = args.output or Config().COMPASS.BENCH_CALIBRATION_FILE
    calibration.save(output)
    sys.stdout.write(f"Written to {output}\n")


if __name__ == "__main__":
    main()
