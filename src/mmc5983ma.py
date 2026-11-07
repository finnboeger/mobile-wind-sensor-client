"""
Driver for the MEMSIC MMC5983MA AMR magnetometer on I2C.

Every reading is taken twice, once after a SET and once after a RESET pulse. The
pulses flip the sensor's magnetisation, so the field changes sign between the two
readings while the sensor's own offset doesn't. Half their difference is the field
without the offset, which would otherwise drift with temperature.
"""

import time

from ism330dhcx import I2CBus
from structs import Vector3

DEFAULT_ADDRESS = 0x30

_STATUS = 0x08
_CONTROL_0 = 0x09
_CONTROL_1 = 0x0A
_PRODUCT_ID = 0x2F
_EXPECTED_PRODUCT_ID = 0x30

#: STATUS: a measurement is done
_MEASUREMENT_DONE = 0x01
#: CONTROL_0: start a measurement, SET and RESET pulses
_TAKE_MEASUREMENT = 0x01
_SET = 0x08
_RESET = 0x10
#: CONTROL_1: 100 Hz bandwidth, 8 ms per measurement, the lowest noise setting
_BANDWIDTH_100_HZ = 0x00
#: the 18 bit output reads half the range at zero field
_NULL_FIELD = 1 << 17
#: counts per µT in 18 bit mode (16384 counts per gauss)
_COUNTS_PER_MICROTESLA = 163.84
#: time the sensor needs after a SET or RESET pulse, in seconds
_PULSE_TIME = 0.001
#: longest time a measurement may take, in seconds
_MEASUREMENT_TIMEOUT = 0.05


class MMC5983MA:
    """MEMSIC MMC5983MA magnetometer."""

    def __init__(self, bus: I2CBus, address: int = DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address

        product_id = bus.read_byte_data(address, _PRODUCT_ID)
        if product_id != _EXPECTED_PRODUCT_ID:
            msg = (
                f"No MMC5983MA at I2C address {address:#04x}: product id is "
                f"{product_id:#04x}, expected {_EXPECTED_PRODUCT_ID:#04x}"
            )
            raise OSError(msg)
        bus.write_byte_data(address, _CONTROL_1, _BANDWIDTH_100_HZ)

    def _measure(self, pulse: int) -> Vector3:
        self.bus.write_byte_data(self.address, _CONTROL_0, pulse)
        time.sleep(_PULSE_TIME)
        self.bus.write_byte_data(self.address, _CONTROL_0, _TAKE_MEASUREMENT)
        deadline = time.monotonic() + _MEASUREMENT_TIMEOUT
        while not self.bus.read_byte_data(self.address, _STATUS) & _MEASUREMENT_DONE:
            if time.monotonic() > deadline:
                msg = "MMC5983MA measurement timed out"
                raise TimeoutError(msg)
            time.sleep(0.0005)

        # 8 high bits, 8 middle bits of x, y, z, then the 2 low bits of each
        data = self.bus.read_i2c_block_data(self.address, 0x00, 7)
        x = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x3)
        y = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x3)
        z = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x3)
        return (
            (x - _NULL_FIELD) / _COUNTS_PER_MICROTESLA,
            (y - _NULL_FIELD) / _COUNTS_PER_MICROTESLA,
            (z - _NULL_FIELD) / _COUNTS_PER_MICROTESLA,
        )

    def read(self) -> Vector3:
        """Return the magnetic field in µT in the sensor's axes."""
        after_set = self._measure(_SET)
        after_reset = self._measure(_RESET)
        return (
            (after_set[0] - after_reset[0]) / 2,
            (after_set[1] - after_reset[1]) / 2,
            (after_set[2] - after_reset[2]) / 2,
        )
