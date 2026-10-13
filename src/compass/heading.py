"""
Kalman filter for the heading, from the gyro and the magnetometer.

The state is ``[heading, gyro bias]``. Each gyro sample predicts the heading with the
rate of turn around the vertical axis, and each magnetometer heading corrects it. The
result follows turns without lag and smooths out the magnetometer noise and the
heading errors caused by wave motion.
"""

import logging
import math

logger = logging.getLogger(__name__)

#: gyro noise in rad/s per square root of a second, how quickly the predicted heading
#: becomes uncertain
DEFAULT_GYRO_NOISE = math.radians(0.2)
#: how quickly the gyro bias may wander, in rad/s per square root of a second
DEFAULT_BIAS_NOISE = math.radians(0.01)
#: standard deviation of a single magnetometer heading in radians
DEFAULT_MAGNETIC_NOISE = math.radians(3)
#: magnetometer headings further than this from the prediction are rejected
DEFAULT_GATE = math.radians(25)
#: after the magnetometer has disagreed for this long, the filter jumps to it
DEFAULT_RESET_AFTER_MS = 3000
#: initial uncertainty of the gyro bias in rad/s
_INITIAL_BIAS_SIGMA = math.radians(0.5)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-π, π)``."""
    return (angle + math.pi) % math.tau - math.pi


class HeadingFilter:
    def __init__(
        self,
        *,
        gyro_noise: float = DEFAULT_GYRO_NOISE,
        bias_noise: float = DEFAULT_BIAS_NOISE,
        magnetic_noise: float = DEFAULT_MAGNETIC_NOISE,
        gate: float = DEFAULT_GATE,
        reset_after_ms: int = DEFAULT_RESET_AFTER_MS,
    ) -> None:
        self.gyro_noise = gyro_noise
        self.bias_noise = bias_noise
        self.magnetic_noise = magnetic_noise
        self.gate = gate
        self.reset_after_ms = reset_after_ms

        self.heading: float | None = None
        self.bias = 0.0
        self._p_hh = 0.0
        self._p_hb = 0.0
        self._p_bb = _INITIAL_BIAS_SIGMA**2
        self._rejected_since: int | None = None
        #: latest bias corrected rate of turn in rad/s
        self.rate = 0.0

    def predict(self, dt: float, rate: float) -> None:
        """
        Advance the heading with the gyro.

        :param dt: time step in seconds
        :param rate: rate of turn around the vertical axis in rad/s
        """
        self.rate = rate - self.bias
        if self.heading is None:
            return
        self.heading = (self.heading + self.rate * dt) % math.tau
        # P = F P F^T + Q with F = [[1, -dt], [0, 1]]
        p_hh = self._p_hh - 2 * dt * self._p_hb + dt * dt * self._p_bb
        p_hb = self._p_hb - dt * self._p_bb
        self._p_hh = p_hh + self.gyro_noise**2 * dt
        self._p_hb = p_hb
        self._p_bb += self.bias_noise**2 * dt

    def update(self, timestamp: int, magnetic_heading: float) -> bool:
        """
        Correct the heading with a magnetometer heading.

        :param timestamp: time of the measurement in milliseconds
        :param magnetic_heading: heading in radians
        :return: whether the measurement was used
        """
        variance = self.magnetic_noise**2
        if self.heading is None:
            self.heading = magnetic_heading % math.tau
            self._p_hh = variance
            self._p_hb = 0.0
            return True

        residual = wrap_angle(magnetic_heading - self.heading)
        residual_variance = self._p_hh + variance
        if abs(residual) > max(self.gate, 3 * math.sqrt(residual_variance)):
            if self._rejected_since is None:
                self._rejected_since = timestamp
            elif timestamp - self._rejected_since > self.reset_after_ms:
                logger.info(
                    "Magnetometer disagreed with the gyro for %d ms, resetting heading",
                    timestamp - self._rejected_since,
                )
                self.heading = magnetic_heading % math.tau
                self._p_hh = variance
                self._p_hb = 0.0
                self._rejected_since = None
                return True
            return False
        self._rejected_since = None

        k_h = self._p_hh / residual_variance
        k_b = self._p_hb / residual_variance
        self.heading = (self.heading + k_h * residual) % math.tau
        self.bias += k_b * residual
        p_hh, p_hb, p_bb = self._p_hh, self._p_hb, self._p_bb
        self._p_hh = p_hh - k_h * p_hh
        self._p_hb = p_hb - k_h * p_hb
        self._p_bb = p_bb - k_b * p_hb
        return True
