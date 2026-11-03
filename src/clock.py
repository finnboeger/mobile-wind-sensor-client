"""
Keep the system clock in line with GPS time.

The Pi has no real time clock, so without internet access (and thus NTP) its
clock can be far off after a boot. Wind data is stamped with the system clock and
positions with GPS time, so the two have to agree to be matched.
"""

import logging
import time

logger = logging.getLogger(__name__)

#: the system clock is only set when it is off by more than this many milliseconds,
#: so a working NTP sync (accurate to a few milliseconds) is never overridden
MAX_OFFSET_MS = 1000

_permission_warned = False


def sync_from_gps(gps_time_ms: int) -> bool:
    """
    Set the system clock to ``gps_time_ms`` if it is too far off.

    Setting the clock needs root (or ``CAP_SYS_TIME``).

    :return: whether the clock was set
    """
    global _permission_warned  # noqa: PLW0603
    offset_ms = gps_time_ms - int(time.time() * 1000)
    if abs(offset_ms) <= MAX_OFFSET_MS:
        return False
    try:
        time.clock_settime(time.CLOCK_REALTIME, gps_time_ms / 1000)
    except PermissionError:
        if not _permission_warned:
            logger.warning(
                "The system clock is off by %d ms but can't be set without root.",
                offset_ms,
            )
            _permission_warned = True
        return False
    logger.info("Set the system clock from GPS, it was off by %d ms", offset_ms)
    return True
