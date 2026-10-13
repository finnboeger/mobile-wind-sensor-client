"""
Kalman filter for the boat's velocity over ground.

The GNSS receiver measures velocity directly (Doppler), which is accurate but only
arrives at the navigation rate (1-2 Hz on the NEO-M8U). This filter tracks the
north and east velocity together with the acceleration on each axis, so the
velocity can be evaluated at any point in time, in particular at the timestamp of
each apparent wind measurement.

Each axis is an independent two state filter ``[velocity, acceleration]``. The
acceleration follows a Singer model: it is random, but decays back towards zero with
a time constant, because a boat doesn't keep accelerating for long. The two axes are
independent because the measurements and the process noise are the same in every
horizontal direction.
"""

import logging
import math
from dataclasses import dataclass

from utils.vector import Vector2D

logger = logging.getLogger(__name__)

#: typical (one standard deviation) acceleration of the boat in m/s². Larger values
#: follow manoeuvres more quickly, smaller values smooth more.
DEFAULT_ACCELERATION_SIGMA = 0.3
#: time in seconds after which an acceleration has decayed to 1/e
DEFAULT_ACCELERATION_TIME_CONSTANT = 2.0
#: lower bound for the measurement standard deviation in m/s. The receiver's
#: speed accuracy estimate can be overly optimistic.
DEFAULT_MIN_MEASUREMENT_SIGMA = 0.05
#: measurements whose innovation exceeds this many standard deviations are rejected
DEFAULT_GATE_SIGMA = 5.0
#: after this many rejected measurements in a row the filter is reset to the next one
MAX_CONSECUTIVE_REJECTIONS = 3
#: maximum time in milliseconds the velocity is extrapolated using the acceleration
DEFAULT_MAX_EXTRAPOLATION_MS = 1000


@dataclass(kw_only=True)
class _Axis:
    """Two state Kalman filter ``[velocity, acceleration]`` for one axis."""

    velocity: float
    acceleration: float
    #: covariance matrix [[p_vv, p_va], [p_va, p_aa]]
    p_vv: float
    p_va: float
    p_aa: float

    def predict(
        self,
        dt: float,
        acceleration_sigma: float,
        time_constant: float,
    ) -> None:
        # x = F x with F = [[1, g], [0, phi]]
        phi = math.exp(-dt / time_constant)
        g = time_constant * (1 - phi)
        self.velocity += self.acceleration * g
        self.acceleration *= phi

        # P = F P F^T + Q
        p_vv = self.p_vv + 2 * g * self.p_va + g * g * self.p_aa
        p_va = phi * (self.p_va + g * self.p_aa)
        p_aa = phi * phi * self.p_aa
        # Q keeps the stationary acceleration variance at acceleration_sigma²,
        # the velocity terms are the usual approximation for integrated noise
        q_aa = acceleration_sigma**2 * (1 - phi * phi)
        self.p_vv = p_vv + q_aa * dt * dt / 3
        self.p_va = p_va + q_aa * dt / 2
        self.p_aa = p_aa + q_aa

    def innovation(self, measurement: float, variance: float) -> tuple[float, float]:
        """Return the innovation and its variance for a velocity measurement."""
        return measurement - self.velocity, self.p_vv + variance

    def update(self, residual: float, residual_variance: float) -> None:
        # H = [1, 0], K = P H^T / S
        k_v = self.p_vv / residual_variance
        k_a = self.p_va / residual_variance
        self.velocity += k_v * residual
        self.acceleration += k_a * residual

        # P = (I - K H) P
        p_vv, p_va, p_aa = self.p_vv, self.p_va, self.p_aa
        self.p_vv = p_vv - k_v * p_vv
        self.p_va = p_va - k_v * p_va
        self.p_aa = p_aa - k_a * p_va

    def velocity_after(self, dt: float, time_constant: float) -> float:
        return self.velocity + self.acceleration * time_constant * (
            1 - math.exp(-dt / time_constant)
        )


class VelocityKalmanFilter:
    """Kalman filter estimating the velocity over ground in north/east coordinates."""

    def __init__(
        self,
        *,
        acceleration_sigma: float = DEFAULT_ACCELERATION_SIGMA,
        acceleration_time_constant: float = DEFAULT_ACCELERATION_TIME_CONSTANT,
        min_measurement_sigma: float = DEFAULT_MIN_MEASUREMENT_SIGMA,
        gate_sigma: float = DEFAULT_GATE_SIGMA,
        max_extrapolation_ms: int = DEFAULT_MAX_EXTRAPOLATION_MS,
    ) -> None:
        self.acceleration_sigma = acceleration_sigma
        self.acceleration_time_constant = acceleration_time_constant
        self.min_measurement_sigma = min_measurement_sigma
        self.gate_sigma = gate_sigma
        self.max_extrapolation_ms = max_extrapolation_ms

        self._north: _Axis | None = None
        self._east: _Axis | None = None
        #: timestamp of the current state in milliseconds
        self._time: int | None = None
        self._consecutive_rejections = 0

        #: statistics for the dashboard
        self.accepted_count = 0
        self.rejected_count = 0
        self.reset_count = 0
        #: timestamp of the last measurement in milliseconds
        self.last_measurement_time: int | None = None

    @property
    def initialized(self) -> bool:
        return self._time is not None

    def reset(self) -> None:
        self._north = None
        self._east = None
        self._time = None
        self._consecutive_rejections = 0

    def _initialize(self, timestamp: int, velocity: Vector2D, variance: float) -> None:
        acceleration_variance = self.acceleration_sigma**2
        self._north = _Axis(
            velocity=velocity.x,
            acceleration=0.0,
            p_vv=variance,
            p_va=0.0,
            p_aa=acceleration_variance,
        )
        self._east = _Axis(
            velocity=velocity.y,
            acceleration=0.0,
            p_vv=variance,
            p_va=0.0,
            p_aa=acceleration_variance,
        )
        self._time = timestamp
        self._consecutive_rejections = 0

    def update(self, timestamp: int, velocity: Vector2D, sigma: float) -> bool:
        """
        Add a velocity measurement.

        :param timestamp: time of the measurement in milliseconds
        :param velocity: measured velocity, x is north and y is east, in m/s
        :param sigma: standard deviation of the measurement in m/s
        :return: whether the measurement was accepted
        """
        variance = max(sigma, self.min_measurement_sigma) ** 2
        self.last_measurement_time = timestamp

        if self._north is None or self._east is None or self._time is None:
            self._initialize(timestamp, velocity, variance)
            self.accepted_count += 1
            return True

        # Measurements arriving out of order are applied at the current state time
        dt = max(0, timestamp - self._time) / 1000
        if dt > 0:
            for axis in (self._north, self._east):
                axis.predict(
                    dt,
                    self.acceleration_sigma,
                    self.acceleration_time_constant,
                )
            self._time = timestamp

        residual_north, variance_north = self._north.innovation(velocity.x, variance)
        residual_east, variance_east = self._east.innovation(velocity.y, variance)

        normalized_squared = (
            residual_north**2 / variance_north + residual_east**2 / variance_east
        )
        if normalized_squared > 2 * self.gate_sigma**2:
            self._consecutive_rejections += 1
            self.rejected_count += 1
            if self._consecutive_rejections >= MAX_CONSECUTIVE_REJECTIONS:
                logger.info(
                    "Velocity filter rejected %d measurements in a row, resetting",
                    self._consecutive_rejections,
                )
                self._initialize(timestamp, velocity, variance)
                self.reset_count += 1
                return True
            logger.debug(
                "Rejected velocity measurement, innovation %.2f m/s north, "
                "%.2f m/s east",
                residual_north,
                residual_east,
            )
            return False

        self._consecutive_rejections = 0
        self._north.update(residual_north, variance_north)
        self._east.update(residual_east, variance_east)
        self.accepted_count += 1
        return True

    def velocity_at(self, timestamp: int) -> Vector2D | None:
        """
        Estimate the velocity at the given time without changing the filter state.

        The velocity is extrapolated with the estimated acceleration for at most
        ``max_extrapolation_ms`` in either direction.

        :param timestamp: time in milliseconds
        :return: velocity, x is north and y is east, in m/s, or None if the filter has
            not received a measurement yet
        """
        if self._north is None or self._east is None or self._time is None:
            return None

        offset = timestamp - self._time
        offset = max(-self.max_extrapolation_ms, min(self.max_extrapolation_ms, offset))
        dt = offset / 1000
        return Vector2D(
            x=self._north.velocity_after(dt, self.acceleration_time_constant),
            y=self._east.velocity_after(dt, self.acceleration_time_constant),
        )

    def acceleration(self) -> Vector2D | None:
        """Return the estimated acceleration, x is north and y is east, in m/s²."""
        if self._north is None or self._east is None:
            return None
        return Vector2D(x=self._north.acceleration, y=self._east.acceleration)

    def velocity_sigma(self) -> float | None:
        """Return the standard deviation of the current velocity estimate in m/s."""
        if self._north is None or self._east is None:
            return None
        return math.sqrt(max(self._north.p_vv, self._east.p_vv))
