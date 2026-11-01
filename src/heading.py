"""
Kalman filter combining the gyro with the compass heading.

The compass is damped, so its heading lags in turns. The gyro measures turns
immediately but drifts. This filter integrates the heading rate from the gyro and
uses the compass to correct the heading and to learn the gyro's remaining bias about
the vertical. The state is ``[heading, heading rate bias]``.

The compass is modelled as a first order lag of the true heading. The filter keeps a
lagged copy of its own heading and compares the compass with that copy, so a turn
doesn't pull the heading back towards the old compass value. The lag model is only
approximate, so compass readings count less while the two copies differ.
"""

import math

#: standard deviation of a compass reading in rad
DEFAULT_COMPASS_SIGMA = math.radians(2.0)
#: angle random walk of the integrated gyro rate in rad/√s
DEFAULT_RATE_NOISE = math.radians(0.2)
#: random walk of the gyro bias in rad/s/√s
DEFAULT_BIAS_NOISE = math.radians(0.005)
#: initial standard deviation of the gyro bias in rad/s
INITIAL_BIAS_SIGMA = math.radians(0.5)
#: time constant in seconds of the compass damping, 0 if it doesn't lag
DEFAULT_COMPASS_LAG = 1.0
#: assumed relative error of the lag model
LAG_MODEL_ERROR = 0.5
#: maximum time in milliseconds the heading is extrapolated with the last rate
DEFAULT_MAX_EXTRAPOLATION_MS = 500
#: gaps between gyro samples longer than this in seconds are not integrated
MAX_TIME_STEP = 0.2
#: without gyro samples for this many milliseconds the heading follows the compass
GYRO_TIMEOUT_MS = 1000


def wrap_angle(angle: float) -> float:
    """Wrap an angle in rad to [-π, π)."""
    return (angle + math.pi) % math.tau - math.pi


class HeadingFilter:
    """Estimates the unit's heading from the gyro and the compass."""

    def __init__(
        self,
        *,
        compass_sigma: float = DEFAULT_COMPASS_SIGMA,
        rate_noise: float = DEFAULT_RATE_NOISE,
        bias_noise: float = DEFAULT_BIAS_NOISE,
        compass_lag: float = DEFAULT_COMPASS_LAG,
        max_extrapolation_ms: int = DEFAULT_MAX_EXTRAPOLATION_MS,
    ) -> None:
        self.compass_variance = compass_sigma**2
        self.rate_variance = rate_noise**2
        self.bias_variance = bias_noise**2
        self.compass_lag = compass_lag
        self.max_extrapolation_ms = max_extrapolation_ms

        #: heading in rad, None until the first compass reading
        self.heading: float | None = None
        #: bias of the heading rate in rad/s
        self.bias = 0.0
        self._p_hh = 0.0
        self._p_hb = 0.0
        self._p_bb = 0.0
        #: heading as the lagging compass should read it, in rad
        self._lagged_heading = 0.0
        self._time: int | None = None
        self._rate = 0.0

    @property
    def initialized(self) -> bool:
        return self.heading is not None

    def predict(self, timestamp: int, heading_rate: float) -> None:
        """
        Integrate a gyro sample.

        :param timestamp: time of the sample in milliseconds
        :param heading_rate: rate of change of the heading in rad/s, from the gyro
        """
        self._rate = heading_rate
        if self.heading is None or self._time is None:
            return

        dt = (timestamp - self._time) / 1000
        if dt <= 0:
            return
        self._time = timestamp
        if dt > MAX_TIME_STEP:
            # a gap in the gyro data: the compass has to carry the heading
            self._p_hh += self.compass_variance
            self._lagged_heading = self.heading
            return

        # x = F x with F = [[1, -dt], [0, 1]]
        self.heading = wrap_angle(self.heading + (heading_rate - self.bias) * dt)
        if self.compass_lag > 0:
            self._lagged_heading = wrap_angle(
                self._lagged_heading
                + wrap_angle(self.heading - self._lagged_heading)
                * (1 - math.exp(-dt / self.compass_lag)),
            )
        else:
            self._lagged_heading = self.heading
        p_hh = self._p_hh - 2 * dt * self._p_hb + dt * dt * self._p_bb
        p_hb = self._p_hb - dt * self._p_bb
        self._p_hh = p_hh + self.rate_variance * dt
        self._p_hb = p_hb
        self._p_bb += self.bias_variance * dt

    def update(self, timestamp: int, compass_heading: float) -> None:
        """
        Add a compass reading.

        :param timestamp: time of the reading in milliseconds
        :param compass_heading: heading in rad
        """
        if self.heading is None or self._time is None:
            self.heading = wrap_angle(compass_heading)
            self._lagged_heading = self.heading
            self.bias = 0.0
            self._p_hh = self.compass_variance
            self._p_hb = 0.0
            self._p_bb = INITIAL_BIAS_SIGMA**2
            self._time = timestamp
            return

        if timestamp - self._time > GYRO_TIMEOUT_MS:
            # the gyro stopped, restart from the compass but keep the learned bias
            self.heading = wrap_angle(compass_heading)
            self._lagged_heading = self.heading
            self._p_hh = self.compass_variance
            self._p_hb = 0.0
            self._time = timestamp
            self._rate = 0.0
            return

        # compare with the heading the lagging compass should show, both differ
        # by the same correction
        residual = wrap_angle(compass_heading - self._lagged_heading)
        lag_error = LAG_MODEL_ERROR * wrap_angle(self.heading - self._lagged_heading)
        residual_variance = self._p_hh + self.compass_variance + lag_error**2
        k_h = self._p_hh / residual_variance
        k_b = self._p_hb / residual_variance

        self.heading = wrap_angle(self.heading + k_h * residual)
        self._lagged_heading = wrap_angle(self._lagged_heading + k_h * residual)
        self.bias += k_b * residual

        p_hh, p_hb, p_bb = self._p_hh, self._p_hb, self._p_bb
        self._p_hh = p_hh - k_h * p_hh
        self._p_hb = p_hb - k_h * p_hb
        self._p_bb = p_bb - k_b * p_hb

    def heading_at(self, timestamp: int) -> float | None:
        """
        Estimate the heading at the given time without changing the filter state.

        :param timestamp: time in milliseconds
        :return: heading in rad in [0, 2π), or None before the first compass reading
        """
        if self.heading is None or self._time is None:
            return None
        offset = timestamp - self._time
        offset = max(-self.max_extrapolation_ms, min(self.max_extrapolation_ms, offset))
        return (self.heading + (self._rate - self.bias) * offset / 1000) % math.tau
