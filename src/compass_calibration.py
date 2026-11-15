"""
Calibration of the magnetometers.

There are two layers, because the iron near the sensors falls into two groups.

The iron that travels with the sensor (the board, the enclosure, connectors and
screws) adds a constant field (hard iron) and distorts the earth's field depending on
its direction (soft iron). Seen from the sensor both turn with it, so they can be
measured once on the bench: turn the finished unit in every direction and fit an
ellipsoid to the readings. The correction maps that ellipsoid back to a sphere. See
``calibrate_compass.py``.

The boat's iron (engine, battery, fuel tank, fittings) turns with the sensor only
when the whole boat turns, so it can't be removed on the bench.
:py:class:`ContinuousCalibration` learns it from the turns the boat makes anyway. It
fits an ellipse (or a circle while the coverage is incomplete) to the horizontal
field after levelling, sorted into direction bins so that long straight runs don't
dominate the fit.
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


#: number of bins the horizontal field direction is sorted into
CALIBRATION_BINS = 36
#: filled bins needed for a circle fit (hard iron only)
MIN_BINS_CIRCLE = 12
#: filled bins needed for an ellipse fit (hard and soft iron)
MIN_BINS_ELLIPSE = 30
#: samples per bin after which older samples are slowly forgotten
MAX_BIN_WEIGHT = 50.0
#: samples a bin needs before it counts as filled
MIN_BIN_WEIGHT = 5.0
#: fraction of the change towards a new fit that is applied each time
DEFAULT_BLEND = 0.3
#: fits whose residual exceeds this fraction of the radius are rejected
MAX_RELATIVE_RESIDUAL = 0.1
#: soft iron correction is only accepted if the ellipse axes differ by less than this
MAX_AXIS_RATIO = 1.5


@dataclass(kw_only=True)
class _Bin:
    x: float
    y: float
    weight: float


@dataclass(kw_only=True)
class HorizontalCalibration:
    """Correction of the boat's iron, applied after the bench calibration."""

    #: hard iron offset in the unit's axes in µT (x forward, y right)
    offset: tuple[float, float] = (0.0, 0.0)
    #: soft iron correction of the levelled horizontal field
    soft_iron: Matrix = field(default_factory=lambda: identity(2))
    #: radius of the fitted circle, the horizontal field strength in µT
    radius: float | None = None

    def remove_offset(self, field_body: Vector3) -> Vector3:
        return (
            field_body[0] - self.offset[0],
            field_body[1] - self.offset[1],
            field_body[2],
        )

    def apply_soft_iron(self, x: float, y: float) -> tuple[float, float]:
        corrected = mat_vec(self.soft_iron, [x, y])
        return corrected[0], corrected[1]


@dataclass(frozen=True, kw_only=True)
class _Fit:
    center: tuple[float, float]
    soft_iron: Matrix
    radius: float
    residual: float


def _fit_circle(points: list[tuple[float, float]]) -> _Fit | None:
    # x² + y² = 2 cx x + 2 cy y + (r² - cx² - cy²)
    rows = [[2 * x, 2 * y, 1.0] for x, y in points]
    targets = [x * x + y * y for x, y in points]
    solution = least_squares(rows, targets)
    if solution is None:
        return None
    cx, cy, k = solution
    radius_squared = k + cx * cx + cy * cy
    if radius_squared <= 0:
        return None
    radius = math.sqrt(radius_squared)
    residual = math.sqrt(
        sum((math.hypot(x - cx, y - cy) - radius) ** 2 for x, y in points)
        / len(points),
    )
    return _Fit(
        center=(cx, cy),
        soft_iron=identity(2),
        radius=radius,
        residual=residual,
    )


def _fit_ellipse(points: list[tuple[float, float]]) -> _Fit | None:
    # a x² + 2b xy + c y² + 2d x + 2e y = 1
    rows = [[x * x, 2 * x * y, y * y, 2 * x, 2 * y] for x, y in points]
    solution = least_squares(rows, [1.0] * len(points))
    if solution is None:
        return None
    a, b, c, d, e = solution
    quadric = [[a, b], [b, c]]
    center_solution = solve(quadric, [-d, -e])
    if center_solution is None:
        return None
    cx, cy = center_solution
    scale = 1 + a * cx * cx + 2 * b * cx * cy + c * cy * cy
    if scale <= 0:
        return None
    shape = [[a / scale, b / scale], [b / scale, c / scale]]
    root = symmetric_power(shape, 0.5)
    if root is None:
        return None

    # semi axes are the inverse square roots of the eigenvalues of the shape
    determinant = shape[0][0] * shape[1][1] - shape[0][1] * shape[1][0]
    radius = determinant ** (-1 / 4)
    trace = shape[0][0] + shape[1][1]
    discriminant = math.sqrt(max(trace * trace / 4 - determinant, 0))
    largest = trace / 2 + discriminant
    smallest = trace / 2 - discriminant
    if smallest <= 0 or math.sqrt(largest / smallest) > MAX_AXIS_RATIO:
        return None

    soft_iron = [[radius * x for x in row] for row in root]
    residual = math.sqrt(
        sum(
            (math.hypot(*mat_vec(soft_iron, [x - cx, y - cy])) - radius) ** 2
            for x, y in points
        )
        / len(points),
    )
    return _Fit(center=(cx, cy), soft_iron=soft_iron, radius=radius, residual=residual)


class ContinuousCalibration:
    """
    Learns the boat's hard and soft iron from normal manoeuvring.

    Samples are levelled horizontal field vectors after the bench calibration, but
    before this calibration. They are sorted into bins by their direction around the
    current centre estimate, each bin keeps a slowly forgetting average, and the
    fit is repeated whenever enough bins are filled. Because the bins keep the
    uncorrected vectors, a new fit never depends on the previous one.
    """

    def __init__(self, *, blend: float = DEFAULT_BLEND) -> None:
        self.blend = blend
        self.calibration = HorizontalCalibration()
        self.fit_count = 0
        self._bins: dict[int, _Bin] = {}

    @property
    def filled_bins(self) -> int:
        return sum(1 for b in self._bins.values() if b.weight >= MIN_BIN_WEIGHT)

    @property
    def converged(self) -> bool:
        return self.fit_count > 0

    def _bin_index(self, x: float, y: float) -> int:
        cx, cy = self.calibration.offset
        angle = math.atan2(y - cy, x - cx) % math.tau
        return int(angle / math.tau * CALIBRATION_BINS) % CALIBRATION_BINS

    def add_sample(self, x: float, y: float) -> None:
        """
        Add a levelled horizontal field sample.

        :param x: field along the levelled forward direction in µT
        :param y: field towards the levelled right in µT
        """
        index = self._bin_index(x, y)
        current = self._bins.get(index)
        if current is None:
            self._bins[index] = _Bin(x=x, y=y, weight=1.0)
            return
        weight = min(current.weight + 1, MAX_BIN_WEIGHT)
        current.x += (x - current.x) / weight
        current.y += (y - current.y) / weight
        current.weight = weight

    def _rebin(self) -> None:
        """Sort the bins again after the centre has moved."""
        old_bins = list(self._bins.values())
        self._bins = {}
        for old in old_bins:
            index = self._bin_index(old.x, old.y)
            current = self._bins.get(index)
            if current is None:
                self._bins[index] = _Bin(x=old.x, y=old.y, weight=old.weight)
                continue
            total = current.weight + old.weight
            current.x = (current.x * current.weight + old.x * old.weight) / total
            current.y = (current.y * current.weight + old.y * old.weight) / total
            current.weight = min(total, MAX_BIN_WEIGHT)

    def _fit(self, points: list[tuple[float, float]]) -> _Fit | None:
        fit = None
        if len(points) >= MIN_BINS_ELLIPSE:
            fit = _fit_ellipse(points)
        if fit is None:
            fit = _fit_circle(points)
        if fit is None or fit.residual > MAX_RELATIVE_RESIDUAL * fit.radius:
            return None
        return fit

    def refit(self) -> bool:
        """
        Fit the calibration to the current bins.

        :return: whether the calibration changed
        """
        indices = [i for i, b in self._bins.items() if b.weight >= MIN_BIN_WEIGHT]
        if len(indices) < MIN_BINS_CIRCLE:
            return False
        points = [(self._bins[i].x, self._bins[i].y) for i in indices]
        fit = self._fit(points)
        if fit is None:
            logger.debug("Compass calibration fit rejected")
            return False

        # drop bins that don't fit, e.g. averaged while next to a steel hull, and fit
        # again without them
        outliers = [
            i
            for i, (x, y) in zip(indices, points, strict=True)
            if abs(
                math.hypot(
                    *mat_vec(fit.soft_iron, [x - fit.center[0], y - fit.center[1]]),
                )
                - fit.radius,
            )
            > max(3 * fit.residual, 0.02 * fit.radius)
        ]
        if outliers and len(indices) - len(outliers) >= MIN_BINS_CIRCLE:
            for i in outliers:
                del self._bins[i]
            logger.info("Dropped %d compass calibration bins", len(outliers))
            return self.refit()

        blend = 1.0 if self.fit_count == 0 else self.blend
        old = self.calibration
        offset = (
            old.offset[0] + blend * (fit.center[0] - old.offset[0]),
            old.offset[1] + blend * (fit.center[1] - old.offset[1]),
        )
        soft_iron = [
            [
                old.soft_iron[i][j]
                + blend * (fit.soft_iron[i][j] - old.soft_iron[i][j])
                for j in range(2)
            ]
            for i in range(2)
        ]
        radius = (
            fit.radius
            if old.radius is None
            else old.radius + blend * (fit.radius - old.radius)
        )
        self.calibration = HorizontalCalibration(
            offset=offset,
            soft_iron=soft_iron,
            radius=radius,
        )
        self.fit_count += 1
        logger.info(
            "Compass calibration updated from %d bins: offset %.2f/%.2f µT, "
            "horizontal field %.2f µT, residual %.2f µT",
            len(points),
            offset[0],
            offset[1],
            radius,
            fit.residual,
        )
        self._rebin()
        return True

    def state_dict(self) -> dict[str, Any]:
        return {
            "offset": list(self.calibration.offset),
            "soft_iron": self.calibration.soft_iron,
            "radius": self.calibration.radius,
            "fit_count": self.fit_count,
            "bins": [[i, b.x, b.y, b.weight] for i, b in self._bins.items()],
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        offset = state["offset"]
        self.calibration = HorizontalCalibration(
            offset=(float(offset[0]), float(offset[1])),
            soft_iron=[[float(x) for x in row] for row in state["soft_iron"]],
            radius=state.get("radius"),
        )
        self.fit_count = int(state.get("fit_count", 0))
        self._bins = {
            int(i): _Bin(x=float(x), y=float(y), weight=float(w))
            for i, x, y, w in state.get("bins", [])
        }
