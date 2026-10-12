"""
I²C drivers for the magnetometer and the IMU.

Each driver returns readings in the sensor's own axes. :py:func:`parse_axes` maps them
to the boat's body frame (x forward, y starboard, z down), so the sensors can be
mounted in any of the 24 right angle orientations.
"""

import logging
import math
import time
from collections.abc import Callable
from typing import Protocol

from smbus2 import SMBus

from compass.attitude import GRAVITY
from utils.linalg import Vector3

logger = logging.getLogger(__name__)

#: number of axes of each sensor
_AXES = 3


class Magnetometer(Protocol):
    def read(self) -> Vector3:
        """Return the magnetic field in µT in the sensor's axes."""
        ...


class Imu(Protocol):
    def read(self) -> tuple[Vector3, Vector3]:
        """Return the specific force in m/s² and the angular rate in rad/s."""
        ...


def parse_axes(specification: str) -> Callable[[Vector3], Vector3]:
    """
    Parse an axis mapping like ``"x,-y,-z"``.

    The three entries are the sensor axes that point forward, to starboard and down,
    in that order. ``"x,-y,-z"`` suits a sensor lying flat, component side up, with
    its x axis towards the bow.
    """
    entries = [entry.strip().lower() for entry in specification.split(",")]
    if len(entries) != _AXES:
        msg = f"Axis mapping {specification!r} needs three entries"
        raise ValueError(msg)
    mapping: list[tuple[int, float]] = []
    for entry in entries:
        sign = -1.0 if entry.startswith("-") else 1.0
        name = entry.lstrip("+-")
        if name not in ("x", "y", "z"):
            msg = f"Unknown axis {entry!r} in {specification!r}"
            raise ValueError(msg)
        mapping.append(("xyz".index(name), sign))
    if sorted(index for index, _ in mapping) != [0, 1, 2]:
        msg = f"Axis mapping {specification!r} must use each axis once"
        raise ValueError(msg)

    def apply(vector: Vector3) -> Vector3:
        return (
            mapping[0][1] * vector[mapping[0][0]],
            mapping[1][1] * vector[mapping[1][0]],
            mapping[2][1] * vector[mapping[2][0]],
        )

    return apply


def _check_id(
    bus: SMBus,
    address: int,
    register: int,
    expected: int,
    name: str,
) -> None:
    found = bus.read_byte_data(address, register)
    if found != expected:
        logger.warning(
            "%s at 0x%02x reports id 0x%02x, expected 0x%02x",
            name,
            address,
            found,
            expected,
        )


def _wait_for(
    bus: SMBus,
    address: int,
    register: int,
    mask: int,
    timeout: float = 0.05,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if bus.read_byte_data(address, register) & mask:
            return True
        time.sleep(0.0005)
    return False


class MMC5983MA:
    """
    MEMSIC MMC5983MA AMR magnetometer.

    Every reading is taken twice, once after a SET and once after a RESET pulse. The
    field flips sign between the two while the sensor's own offset doesn't, so their
    difference cancels the offset and its drift with temperature.
    """

    DEFAULT_ADDRESS = 0x30
    _PRODUCT_ID = 0x2F
    _STATUS = 0x08
    _CONTROL_0 = 0x09
    _CONTROL_1 = 0x0A
    _MEASUREMENT_DONE = 0x01
    _TAKE_MEASUREMENT = 0x01
    _SET = 0x08
    _RESET = 0x10
    #: 18 bit output, zero field is in the middle of the range
    _NULL_FIELD = 1 << 17
    #: counts per µT in 18 bit mode (16384 counts per gauss)
    _COUNTS_PER_MICROTESLA = 163.84

    def __init__(self, bus: SMBus, address: int = DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address
        _check_id(bus, address, self._PRODUCT_ID, 0x30, "MMC5983MA")
        # 100 Hz bandwidth (8 ms per measurement), the lowest noise setting
        bus.write_byte_data(address, self._CONTROL_1, 0x00)

    def _measure(self, pulse: int) -> Vector3:
        self.bus.write_byte_data(self.address, self._CONTROL_0, pulse)
        time.sleep(0.001)
        self.bus.write_byte_data(self.address, self._CONTROL_0, self._TAKE_MEASUREMENT)
        if not _wait_for(self.bus, self.address, self._STATUS, self._MEASUREMENT_DONE):
            msg = "MMC5983MA measurement timed out"
            raise TimeoutError(msg)
        data = self.bus.read_i2c_block_data(self.address, 0x00, 7)
        x = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x3)
        y = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x3)
        z = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x3)
        return (
            (x - self._NULL_FIELD) / self._COUNTS_PER_MICROTESLA,
            (y - self._NULL_FIELD) / self._COUNTS_PER_MICROTESLA,
            (z - self._NULL_FIELD) / self._COUNTS_PER_MICROTESLA,
        )

    def read(self) -> Vector3:
        after_set = self._measure(self._SET)
        after_reset = self._measure(self._RESET)
        return (
            (after_set[0] - after_reset[0]) / 2,
            (after_set[1] - after_reset[1]) / 2,
            (after_set[2] - after_reset[2]) / 2,
        )


class RM3100:
    """PNI RM3100 magneto-inductive magnetometer on I²C."""

    DEFAULT_ADDRESS = 0x20
    _POLL = 0x00
    _CYCLE_COUNT = 0x04
    _MEASUREMENT = 0x24
    _STATUS = 0x34
    _REVISION = 0x36
    _DATA_READY = 0x80
    #: cycle count 200: 13 nT resolution, 75 counts per µT, up to 440 Hz single axis
    _CYCLES = 200
    _COUNTS_PER_MICROTESLA = 75.0

    def __init__(self, bus: SMBus, address: int = DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address
        _check_id(bus, address, self._REVISION, 0x22, "RM3100")
        cycles = [self._CYCLES >> 8, self._CYCLES & 0xFF]
        bus.write_i2c_block_data(address, self._CYCLE_COUNT, cycles * 3)

    def read(self) -> Vector3:
        # single measurement of all three axes
        self.bus.write_byte_data(self.address, self._POLL, 0x70)
        if not _wait_for(self.bus, self.address, self._STATUS, self._DATA_READY):
            msg = "RM3100 measurement timed out"
            raise TimeoutError(msg)
        data = self.bus.read_i2c_block_data(self.address, self._MEASUREMENT, 9)

        def counts(offset: int) -> int:
            value = (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2]
            return value - (1 << 24) if value & 0x800000 else value

        return (
            counts(0) / self._COUNTS_PER_MICROTESLA,
            counts(3) / self._COUNTS_PER_MICROTESLA,
            counts(6) / self._COUNTS_PER_MICROTESLA,
        )


class ISM330DHCX:
    """ST ISM330DHCX accelerometer and gyro."""

    DEFAULT_ADDRESS = 0x6A
    _WHO_AM_I = 0x0F
    _CTRL1_XL = 0x10
    _CTRL2_G = 0x11
    _CTRL3_C = 0x12
    _OUTX_L_G = 0x22
    #: 104 Hz, ±4 g
    _ACCELEROMETER_SETTING = 0x48
    #: 104 Hz, ±500 °/s
    _GYRO_SETTING = 0x44
    #: block data update and register address auto increment
    _CTRL3_SETTING = 0x44
    _G_PER_COUNT = 0.122e-3
    _DEGREES_PER_SECOND_PER_COUNT = 17.5e-3

    def __init__(self, bus: SMBus, address: int = DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address
        _check_id(bus, address, self._WHO_AM_I, 0x6B, "ISM330DHCX")
        bus.write_byte_data(address, self._CTRL3_C, self._CTRL3_SETTING)
        bus.write_byte_data(address, self._CTRL1_XL, self._ACCELEROMETER_SETTING)
        bus.write_byte_data(address, self._CTRL2_G, self._GYRO_SETTING)

    def read(self) -> tuple[Vector3, Vector3]:
        data = self.bus.read_i2c_block_data(self.address, self._OUTX_L_G, 12)
        values = [
            int.from_bytes(bytes(data[i : i + 2]), "little", signed=True)
            for i in range(0, 12, 2)
        ]
        rate_scale = math.radians(self._DEGREES_PER_SECOND_PER_COUNT)
        acceleration_scale = self._G_PER_COUNT * GRAVITY
        # the accelerometer measures the reaction to gravity, so a sensor lying flat
        # reads +1 g on its z axis, which points up
        return (
            (
                values[3] * acceleration_scale,
                values[4] * acceleration_scale,
                values[5] * acceleration_scale,
            ),
            (values[0] * rate_scale, values[1] * rate_scale, values[2] * rate_scale),
        )


MAGNETOMETERS: dict[str, type[MMC5983MA] | type[RM3100]] = {
    "mmc5983ma": MMC5983MA,
    "rm3100": RM3100,
}
IMUS: dict[str, type[ISM330DHCX]] = {"ism330dhcx": ISM330DHCX}
