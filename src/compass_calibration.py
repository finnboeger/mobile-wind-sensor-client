"""
Calibration of the magnetometers.

There are two layers, because the iron near the sensors falls into two groups.

The iron that travels with the sensor (the board, the enclosure, connectors and
screws) adds a constant field (hard iron) and distorts the earth's field depending on
its direction (soft iron). Seen from the sensor both turn with it, so they can be
measured once on the bench: turn the finished unit in every direction and fit an
ellipsoid to the readings. The correction maps that ellipsoid back to a sphere. With
more than one magnetometer the same rotation also gives the small angles between
the chips on the board, so every magnetometer's field can be turned into the axes of
the first one (:py:func:`fit_rotation`), and :py:class:`DriftTracker` can follow
how the others drift relative to it. The accelerometer read during the same
rotation gives the angle between the first magnetometer and the IMU
(:py:func:`fit_imu_alignment`), e.g. from a sensing coil that doesn't stand exactly
upright. See ``calibrate_compass.py``.

The boat's iron (engine, battery, fuel tank, fittings) turns with the sensor only
when the whole boat turns, so it can't be removed on the bench.
:py:class:`ContinuousCalibration` learns it from the turns the boat makes anyway. It
fits an ellipse (or a circle while the coverage is incomplete) to the horizontal
field after levelling, sorted into direction bins so that long straight runs don't
dominate the fit.

:py:class:`GnssAlignment` finally learns the constant offset between the compass
heading and the course over ground on straight runs at speed.
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
    cross,
    identity,
    least_squares,
    mat_mul,
    mat_vec,
    norm,
    solve,
    symmetric_eigen,
    symmetric_power,
)

logger = logging.getLogger(__name__)

#: minimum number of samples for a bench calibration
MIN_BENCH_SAMPLES = 50
#: the IMU alignment needs the samples to cover the directions at least this well:
#: the smallest spread of the cross products of field and down, relative to the
#: field strength squared,
#: about 0.04 for a rotation in every direction
MIN_ALIGNMENT_SPREAD = 0.01
#: Gauss-Newton steps of the IMU alignment, and the step in rad that counts as done
_ALIGNMENT_ITERATIONS = 10
_ALIGNMENT_TOLERANCE = 1e-8


def _circular_mean(angles: list[float]) -> float:
    return math.atan2(
        sum(math.sin(a) for a in angles),
        sum(math.cos(a) for a in angles),
    )


def _wrap(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


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


def fit_rotation(source: list[Vector3], target: list[Vector3]) -> Matrix | None:
    """
    Find the rotation that best turns the source vectors into the target vectors.

    This is Horn's method: the rotation, as a quaternion, is the eigenvector of the
    largest eigenvalue of a 4x4 matrix built from the cross covariance. Only the
    directions of the vectors are used.

    :return: rotation matrix R with ``target ≈ R @ source``, or None if there are
        too few pairs
    """
    pairs = [
        (a, b)
        for a, b in zip(source, target, strict=True)
        if norm(a) > 0 and norm(b) > 0
    ]
    if len(pairs) < MIN_BENCH_SAMPLES:
        return None
    # s[i][j] = mean of source_i * target_j
    s = [[0.0] * 3 for _ in range(3)]
    for a, b in pairs:
        length_a, length_b = norm(a), norm(b)
        for i in range(3):
            for j in range(3):
                s[i][j] += a[i] * b[j] / (length_a * length_b * len(pairs))
    (xx, xy, xz), (yx, yy, yz), (zx, zy, zz) = s
    horn = [
        [xx + yy + zz, yz - zy, zx - xz, xy - yx],
        [yz - zy, xx - yy - zz, xy + yx, zx + xz],
        [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
        [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
    ]
    values, vectors = symmetric_eigen(horn)
    largest = values.index(max(values))
    w, x, y, z = (vectors[row][largest] for row in range(4))
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def rotation_angle(rotation: Matrix) -> float:
    """Return the angle in rad of a rotation matrix."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    return math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))


def align_bench_calibration(
    calibration: BenchCalibration,
    rotation: Matrix,
) -> BenchCalibration:
    """Return the calibration followed by a rotation into another sensor's axes."""
    return BenchCalibration(
        offset=calibration.offset,
        matrix=mat_mul(rotation, calibration.matrix),
        field_strength=calibration.field_strength,
    )


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _scaled(vector: Vector3, factor: float) -> Vector3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def _turned(rotation: Matrix, vector: Vector3) -> Vector3:
    x, y, z = mat_vec(rotation, vector)
    return (x, y, z)


def _small_rotation(vector: Vector3) -> Matrix:
    """Return the rotation about the axis of the vector by its length in rad."""
    angle = norm(vector)
    if angle == 0:
        return identity(3)
    x, y, z = _scaled(vector, 1 / angle)
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


@dataclass(frozen=True, kw_only=True)
class ImuAlignmentResult:
    #: rotation R with ``field in the IMU's axes = R @ calibrated field``
    rotation: Matrix
    #: angle of the earth's field below the horizon in rad, about 69° near Hamburg
    inclination: float
    #: root mean square deviation of that angle over the samples in rad
    residual: float


def fit_imu_alignment(
    fields: list[Vector3],
    downs: list[Vector3],
) -> ImuAlignmentResult | None:
    """
    Find the small rotation from a calibrated magnetometer's axes to the IMU's.

    The axis mappings in the config get both sensors into the unit's axes, but not
    to the last degree: a chip can sit slightly tilted, and the bench fit of a
    magnetometer whose sensing axes aren't exactly square (e.g. a leaning Z coil on
    an RM3100) returns square axes turned by about half the error. Seen from the
    compass, that looks like a tilt error, which near Hamburg is amplified 2.6 times
    into the heading.

    While the unit is turned on the bench, the angle between the earth's field and
    gravity stays the same, whatever the orientation. So the rotation is the one that
    makes ``(R @ field) · down`` constant over all samples. It is found by
    Gauss-Newton steps on a linear least squares fit of a small rotation and that
    constant.

    :param fields: magnetometer fields after the bench calibration
    :param downs: the direction down from the accelerometer at the same times, taken
        only while the unit was held still
    :return: the rotation, or None if there are too few samples or they don't cover
        enough directions
    """
    pairs = [
        (measured, down)
        for measured, down in zip(fields, downs, strict=True)
        if norm(measured) > 0 and norm(down) > 0
    ]
    if len(pairs) < MIN_BENCH_SAMPLES:
        return None
    unit_downs = [_scaled(down, 1 / norm(down)) for _, down in pairs]
    strength = sum(norm(measured) for measured, _ in pairs) / len(pairs)

    rotation = identity(3)
    for _ in range(_ALIGNMENT_ITERATIONS):
        rows: list[list[float]] = []
        targets: list[float] = []
        for (measured, _), down in zip(pairs, unit_downs, strict=True):
            turned = _turned(rotation, measured)
            # turning by a small angle vector d changes turned · down by
            # d · (turned cross down)
            gradient = cross(turned, down)
            rows.append([*gradient, -1.0])
            targets.append(-_dot(turned, down))
        solution = least_squares(rows, targets)
        if solution is None:
            return None
        step = (solution[0], solution[1], solution[2])
        rotation = mat_mul(_small_rotation(step), rotation)
        if norm(step) < _ALIGNMENT_TOLERANCE:
            break

    turned_fields = [_turned(rotation, measured) for measured, _ in pairs]

    # the rotation is only determined if the gradients point in every direction
    gradients = [
        cross(turned, down)
        for turned, down in zip(turned_fields, unit_downs, strict=True)
    ]
    mean = [sum(g[i] for g in gradients) / len(gradients) for i in range(3)]
    spread = [
        [
            sum((g[i] - mean[i]) * (g[j] - mean[j]) for g in gradients) / len(gradients)
            for j in range(3)
        ]
        for i in range(3)
    ]
    values, _ = symmetric_eigen(spread)
    if min(values) < MIN_ALIGNMENT_SPREAD * strength * strength:
        return None

    angles = [
        math.asin(max(-1.0, min(1.0, _dot(turned, down) / norm(turned))))
        for turned, down in zip(turned_fields, unit_downs, strict=True)
    ]
    inclination = sum(angles) / len(angles)
    residual = math.sqrt(sum((a - inclination) ** 2 for a in angles) / len(angles))
    return ImuAlignmentResult(
        rotation=rotation,
        inclination=inclination,
        residual=residual,
    )


#: time constant in seconds of the drift tracking
DRIFT_TIME_CONSTANT = 60.0
#: differences further than this from the learned drift, in µT, are a disturbance
#: near one of the magnetometers rather than drift, and are not used
DRIFT_TOLERANCE = 3.0
#: if the difference stays outside the tolerance for this long, it is accepted as
#: the new drift, in milliseconds
DRIFT_RESET_AFTER_MS = 600_000


class DriftTracker:
    """
    Learns how a magnetometer's offset drifts relative to a reference magnetometer.

    Two magnetometers on the same board, both bench calibrated into the same axes,
    measure the same field: the earth's, plus the boat's iron, which is far enough
    away to be the same at both. Whatever differs between them comes from the
    sensors themselves. A MEMS sensor's offset changes with temperature and after
    a strong field has magnetised it, while the RM3100 has no offset to drift. So
    the slowly changing part of the difference is the MEMS sensor's drift.

    Unlike the continuous calibration this also works on a long straight run, and
    it includes the vertical part of the offset, which the horizontal fit can't see.
    """

    def __init__(self) -> None:
        #: drift of the offset in µT in the unit's axes
        self.offset: Vector3 = (0.0, 0.0, 0.0)
        self._weight = 0.0
        self._time: int | None = None
        self._rejected_since: int | None = None

    def remove(self, field: Vector3) -> Vector3:
        return (
            field[0] - self.offset[0],
            field[1] - self.offset[1],
            field[2] - self.offset[2],
        )

    def add(self, timestamp: int, difference: Vector3) -> None:
        """
        Add the difference of the two magnetometers' fields.

        :param difference: this magnetometer's field minus the reference's, both
            bench calibrated, in µT
        """
        dt = 0.0 if self._time is None else max(0, timestamp - self._time) / 1000
        self._time = timestamp
        if self._weight > 0 and math.dist(difference, self.offset) > DRIFT_TOLERANCE:
            if self._rejected_since is None:
                self._rejected_since = timestamp
            elif timestamp - self._rejected_since > DRIFT_RESET_AFTER_MS:
                logger.warning(
                    "Magnetometer drift jumped from %s to %s µT",
                    ", ".join(f"{x:.1f}" for x in self.offset),
                    ", ".join(f"{x:.1f}" for x in difference),
                )
                self.offset = difference
                self._rejected_since = None
            return
        self._rejected_since = None
        # a plain average until a time constant's worth of samples is in
        self._weight += 1
        gain = max(dt / DRIFT_TIME_CONSTANT, 1 / self._weight)
        x, y, z = (
            self.offset[i] + min(1.0, gain) * (difference[i] - self.offset[i])
            for i in range(3)
        )
        self.offset = (x, y, z)

    def state_dict(self) -> dict[str, Any]:
        return {"offset": list(self.offset), "weight": self._weight}

    def load_state_dict(self, state: dict[str, Any]) -> None:
        offset = state["offset"]
        self.offset = (float(offset[0]), float(offset[1]), float(offset[2]))
        self._weight = float(state.get("weight", 0.0))


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


#: number of heading sectors the offset is averaged over
ALIGNMENT_BINS = 12
#: samples per sector after which older samples are slowly forgotten
MAX_ALIGNMENT_WEIGHT = 100.0
#: samples a sector needs before it counts
MIN_ALIGNMENT_WEIGHT = 10.0
#: sectors needed before the offset is used
MIN_ALIGNMENT_BINS = 4
#: the sectors must be spread out: the mean of their unit vectors must be shorter
MAX_ALIGNMENT_CONCENTRATION = 0.5
#: offsets larger than this are assumed to be wrong and not applied
MAX_ALIGNMENT_OFFSET = math.radians(45)


class GnssAlignment:
    """
    Learns the offset from the compass heading to the course over ground.

    Only straight runs at speed are used, when the boat moves where it points. The
    offset is averaged per heading sector first, so that a current, which shifts the
    course by a different amount on each heading, averages out. It contains the
    error of the declination setting and what's left of the boat's iron.
    """

    def __init__(self) -> None:
        #: sector -> [sum of cos, sum of sin, weight]
        self._bins: dict[int, list[float]] = {}
        self.offset: float | None = None

    def add_sample(self, heading: float, course_over_ground: float) -> None:
        """
        Add a pair of heading and course over ground, both in radians.

        :param heading: heading of the bow from the compass, before this alignment
        :param course_over_ground: true course over ground
        """
        index = int((heading % math.tau) / math.tau * ALIGNMENT_BINS) % ALIGNMENT_BINS
        difference = _wrap(course_over_ground - heading)
        current = self._bins.setdefault(index, [0.0, 0.0, 0.0])
        if current[2] >= MAX_ALIGNMENT_WEIGHT:
            decay = (MAX_ALIGNMENT_WEIGHT - 1) / MAX_ALIGNMENT_WEIGHT
            current[0] *= decay
            current[1] *= decay
            current[2] *= decay
        current[0] += math.cos(difference)
        current[1] += math.sin(difference)
        current[2] += 1
        self._update_offset()

    def _update_offset(self) -> None:
        used = [(i, b) for i, b in self._bins.items() if b[2] >= MIN_ALIGNMENT_WEIGHT]
        if len(used) < MIN_ALIGNMENT_BINS:
            return
        sector_angles = [(i + 0.5) / ALIGNMENT_BINS * math.tau for i, _ in used]
        concentration = math.hypot(
            sum(math.cos(a) for a in sector_angles),
            sum(math.sin(a) for a in sector_angles),
        ) / len(used)
        if concentration > MAX_ALIGNMENT_CONCENTRATION:
            return
        offset = _circular_mean([math.atan2(b[1], b[0]) for _, b in used])
        if abs(offset) > MAX_ALIGNMENT_OFFSET:
            logger.warning(
                "Compass heading differs from the course over ground by %.1f°, "
                "not applying it. Check the sensor orientation settings.",
                math.degrees(offset),
            )
            return
        if self.offset is None:
            logger.info(
                "Compass aligned with the course over ground: %.1f°",
                math.degrees(offset),
            )
        self.offset = offset

    def state_dict(self) -> dict[str, Any]:
        return {
            "offset": self.offset,
            "bins": [[i, *b] for i, b in self._bins.items()],
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        self._bins = {
            int(i): [float(c), float(s), float(w)]
            for i, c, s, w in state.get("bins", [])
        }
        self.offset = state.get("offset")
