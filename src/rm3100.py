"""
Driver for the PNI RM3100 magneto-inductive magnetometer on I2C.

The RM3100 measures the frequency of an LC oscillator around each sensor coil, so it
has no offset that drifts with temperature and needs no SET/RESET pulses.
"""

import time
from typing import Protocol

from ism330dhcx import I2CBus
from structs import Vector3

DEFAULT_ADDRESS = 0x20

_POLL = 0x00
_CYCLE_COUNT = 0x04
_MEASUREMENT = 0x24
_STATUS = 0x34
_REVISION = 0x36
_EXPECTED_REVISION = 0x22

#: POLL: measure x, y and z once
_POLL_XYZ = 0x70
#: STATUS: a measurement is ready
_DATA_READY = 0x80
#: cycle count 200: 13 nT resolution, 75 counts per µT, about 7 ms per measurement
_CYCLES = 200
_COUNTS_PER_MICROTESLA = 75.0
#: longest time a measurement may take, in seconds
_MEASUREMENT_TIMEOUT = 0.05


class RM3100Bus(I2CBus, Protocol):
    def write_i2c_block_data(
        self,
        i2c_addr: int,
        register: int,
        data: list[int],
    ) -> None: ...


def _signed_24_bit(high: int, middle: int, low: int) -> int:
    value = (high << 16) | (middle << 8) | low
    return value - (1 << 24) if value & 0x800000 else value


class RM3100:
    """PNI RM3100 magnetometer."""

    def __init__(self, bus: RM3100Bus, address: int = DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address

        revision = bus.read_byte_data(address, _REVISION)
        if revision != _EXPECTED_REVISION:
            msg = (
                f"No RM3100 at I2C address {address:#04x}: revision is "
                f"{revision:#04x}, expected {_EXPECTED_REVISION:#04x}"
            )
            raise OSError(msg)
        cycles = [_CYCLES >> 8, _CYCLES & 0xFF]
        bus.write_i2c_block_data(address, _CYCLE_COUNT, cycles * 3)

    def read(self) -> Vector3:
        """Return the magnetic field in µT in the sensor's axes."""
        self.bus.write_byte_data(self.address, _POLL, _POLL_XYZ)
        deadline = time.monotonic() + _MEASUREMENT_TIMEOUT
        while not self.bus.read_byte_data(self.address, _STATUS) & _DATA_READY:
            if time.monotonic() > deadline:
                msg = "RM3100 measurement timed out"
                raise TimeoutError(msg)
            time.sleep(0.0005)

        data = self.bus.read_i2c_block_data(self.address, _MEASUREMENT, 9)
        return (
            _signed_24_bit(*data[0:3]) / _COUNTS_PER_MICROTESLA,
            _signed_24_bit(*data[3:6]) / _COUNTS_PER_MICROTESLA,
            _signed_24_bit(*data[6:9]) / _COUNTS_PER_MICROTESLA,
        )
