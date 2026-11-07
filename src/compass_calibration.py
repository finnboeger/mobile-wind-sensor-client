"""
Bench calibration of the magnetometers.

The iron that travels with the sensor (the board, the enclosure, connectors and
screws) adds a constant field (hard iron) and distorts the earth's field depending on
its direction (soft iron). Seen from the sensor both turn with it, so they can be
measured once on the bench: turn the finished unit in every direction and fit an
ellipsoid to the readings. The correction maps that ellipsoid back to a sphere. See
``calibrate_compass.py``.
"""

import json
import logging
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from structs import Vector3
from utils.linalg import (
    Matrix,
    identity,
    least_squares,
    mat_vec,
    solve,
    symmetric_power,
)

logger = logging.getLogger(__name__)

#: minimum number of samples for a bench calibration
MIN_BENCH_SAMPLES = 50


def write_json_atomically(path: str | Path, data: dict[str, Any]) -> None:
    """Write JSON to a temporary file first, so a power cut can't corrupt it."""
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.flush()
        os.fsync(file.fileno())
    temporary.replace(path)


@dataclass(kw_only=True)
class BenchCalibration:
    """Hard and soft iron correction ``corrected = matrix @ (raw - offset)``."""

    #: hard iron offset in µT
    offset: Vector3 = (0.0, 0.0, 0.0)
    #: soft iron correction matrix
    matrix: Matrix = field(default_factory=lambda: identity(3))
    #: strength of the earth's field at the place of calibration in µT, if known
    field_strength: float | None = None

    def apply(self, raw: Vector3) -> Vector3:
        x, y, z = mat_vec(
            self.matrix,
            [raw[0] - self.offset[0], raw[1] - self.offset[1], raw[2] - self.offset[2]],
        )
        return (x, y, z)

    def to_dict(self) -> dict[str, Any]:
        return {
            "offset": list(self.offset),
            "matrix": self.matrix,
            "field_strength": self.field_strength,
        }

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "BenchCalibration":
        offset = data["offset"]
        return BenchCalibration(
            offset=(float(offset[0]), float(offset[1]), float(offset[2])),
            matrix=[[float(x) for x in row] for row in data["matrix"]],
            field_strength=data.get("field_strength"),
        )

    @staticmethod
    def load(path: str | Path | None) -> "BenchCalibration":
        """Load a bench calibration, or return the identity if there is none."""
        if path is None or not Path(path).exists():
            logger.warning(
                "No bench calibration found at %s. The heading will be off by the "
                "on-board iron until the continuous calibration has seen enough "
                "turns. Run calibrate_compass.py before mounting the unit.",
                path,
            )
            return BenchCalibration()
        with Path(path).open(encoding="utf-8") as file:
            return BenchCalibration.from_dict(json.load(file))

    def save(self, path: str | Path) -> None:
        write_json_atomically(path, self.to_dict())


def bench_calibration_file(pattern: str, name: str) -> str:
    """Return the bench calibration file of one magnetometer."""
    return pattern.replace("{sensor}", name)


@dataclass(frozen=True, kw_only=True)
class BenchFitResult:
    calibration: BenchCalibration
    #: root mean square deviation of the corrected magnitudes from the field strength
    residual: float


def fit_ellipsoid(samples: list[Vector3]) -> BenchFitResult | None:
    """
    Fit a general ellipsoid to raw magnetometer samples.

    The samples must cover as many orientations as possible, ideally the whole
    sphere. The returned correction maps the ellipsoid to a sphere with the same
    volume, so the corrected values stay in µT.

    :return: the calibration and its residual, or None if the fit failed
    """
    if len(samples) < MIN_BENCH_SAMPLES:
        return None

    # a x² + b y² + c z² + 2f yz + 2g xz + 2h xy + 2p x + 2q y + 2r z = 1
    rows = [
        [x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z]
        for x, y, z in samples
    ]
    solution = least_squares(rows, [1.0] * len(samples))
    if solution is None:
        return None
    a, b, c, f, g, h, p, q, r = solution
    quadric = [[a, h, g], [h, b, f], [g, f, c]]
    center_solution = solve(quadric, [-p, -q, -r])
    if center_solution is None:
        return None
    center = (center_solution[0], center_solution[1], center_solution[2])

    # (m - center)^T quadric (m - center) = scale
    scale = 1 + sum(
        center[i] * quadric[i][j] * center[j] for i in range(3) for j in range(3)
    )
    if scale <= 0:
        return None
    shape = [[x / scale for x in row] for row in quadric]
    root = symmetric_power(shape, 0.5)
    if root is None:
        return None

    # radius of the sphere with the same volume as the ellipsoid
    determinant = (
        shape[0][0] * (shape[1][1] * shape[2][2] - shape[1][2] * shape[2][1])
        - shape[0][1] * (shape[1][0] * shape[2][2] - shape[1][2] * shape[2][0])
        + shape[0][2] * (shape[1][0] * shape[2][1] - shape[1][1] * shape[2][0])
    )
    radius = determinant ** (-1 / 6)
    matrix = [[radius * x for x in row] for row in root]

    calibration = BenchCalibration(
        offset=center,
        matrix=matrix,
        field_strength=radius,
    )
    deviations = [
        math.dist(calibration.apply(sample), (0, 0, 0)) - radius for sample in samples
    ]
    residual = math.sqrt(sum(d * d for d in deviations) / len(deviations))
    return BenchFitResult(calibration=calibration, residual=residual)
