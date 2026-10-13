"""Small dense linear algebra helpers, so the compass doesn't need numpy."""

import math
from typing import TypeAlias

Vector3: TypeAlias = tuple[float, float, float]
Matrix: TypeAlias = list[list[float]]

#: pivots smaller than this are treated as zero when solving linear systems
_SINGULAR_THRESHOLD = 1e-12


def identity(size: int) -> Matrix:
    return [[1.0 if row == col else 0.0 for col in range(size)] for row in range(size)]


def transpose(matrix: Matrix) -> Matrix:
    return [list(column) for column in zip(*matrix, strict=True)]


def mat_mul(a: Matrix, b: Matrix) -> Matrix:
    columns = transpose(b)
    return [
        [sum(x * y for x, y in zip(row, column, strict=True)) for column in columns]
        for row in a
    ]


def mat_vec(matrix: Matrix, vector: list[float] | tuple[float, ...]) -> list[float]:
    return [sum(x * y for x, y in zip(row, vector, strict=True)) for row in matrix]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(vector: tuple[float, ...] | list[float]) -> float:
    return math.sqrt(sum(x * x for x in vector))


def solve(matrix: Matrix, vector: list[float]) -> list[float] | None:
    """
    Solve ``matrix @ x = vector`` with Gaussian elimination and partial pivoting.

    :return: the solution, or None if the matrix is (numerically) singular
    """
    size = len(vector)
    # augmented copy, so the inputs are left untouched
    rows = [[*matrix[i], vector[i]] for i in range(size)]
    scale = max((abs(x) for row in matrix for x in row), default=0.0)
    if scale == 0:
        return None

    for col in range(size):
        pivot = max(range(col, size), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) <= _SINGULAR_THRESHOLD * scale:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        for row in range(col + 1, size):
            factor = rows[row][col] / rows[col][col]
            if factor != 0:
                for k in range(col, size + 1):
                    rows[row][k] -= factor * rows[col][k]

    solution = [0.0] * size
    for row in range(size - 1, -1, -1):
        remainder = rows[row][size] - sum(
            rows[row][k] * solution[k] for k in range(row + 1, size)
        )
        solution[row] = remainder / rows[row][row]
    return solution


def least_squares(rows: list[list[float]], targets: list[float]) -> list[float] | None:
    """
    Solve the overdetermined system ``rows @ x ≈ targets`` via the normal equations.

    :return: the least squares solution, or None if it is not unique
    """
    if len(rows) == 0:
        return None
    size = len(rows[0])
    normal = [[0.0] * size for _ in range(size)]
    right = [0.0] * size
    for row, target in zip(rows, targets, strict=True):
        for i in range(size):
            right[i] += row[i] * target
            for j in range(size):
                normal[i][j] += row[i] * row[j]
    return solve(normal, right)


def symmetric_eigen(
    matrix: Matrix,
    max_sweeps: int = 50,
) -> tuple[list[float], Matrix]:
    """
    Eigen decomposition of a symmetric matrix with the cyclic Jacobi method.

    :return: eigenvalues and a matrix with the matching eigenvectors as columns
    """
    size = len(matrix)
    a = [list(row) for row in matrix]
    vectors = identity(size)
    for _ in range(max_sweeps):
        off_diagonal = sum(
            a[i][j] ** 2 for i in range(size) for j in range(size) if i != j
        )
        if off_diagonal < _SINGULAR_THRESHOLD**2:
            break
        for p in range(size - 1):
            for q in range(p + 1, size):
                if a[p][q] == 0:
                    continue
                theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
                t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta**2 + 1))
                c = 1 / math.sqrt(t * t + 1)
                s = t * c
                for k in range(size):
                    a_kp, a_kq = a[k][p], a[k][q]
                    a[k][p] = c * a_kp - s * a_kq
                    a[k][q] = s * a_kp + c * a_kq
                for k in range(size):
                    a_pk, a_qk = a[p][k], a[q][k]
                    a[p][k] = c * a_pk - s * a_qk
                    a[q][k] = s * a_pk + c * a_qk
                for k in range(size):
                    v_kp, v_kq = vectors[k][p], vectors[k][q]
                    vectors[k][p] = c * v_kp - s * v_kq
                    vectors[k][q] = s * v_kp + c * v_kq
    return [a[i][i] for i in range(size)], vectors


def symmetric_power(matrix: Matrix, exponent: float) -> Matrix | None:
    """
    Raise a symmetric positive definite matrix to a power, e.g. 0.5 for the root.

    :return: the result, or None if the matrix is not positive definite
    """
    values, vectors = symmetric_eigen(matrix)
    if min(values) <= 0:
        return None
    size = len(matrix)
    return [
        [
            sum(
                vectors[i][k] * values[k] ** exponent * vectors[j][k]
                for k in range(size)
            )
            for j in range(size)
        ]
        for i in range(size)
    ]
