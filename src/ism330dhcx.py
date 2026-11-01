"""Driver for the ST ISM330DHCX accelerometer and gyro on I2C."""

import math
import time
from typing import Protocol

from structs import Vector3

#: standard gravity in m/s²
GRAVITY = 9.80665

DEFAULT_ADDRESS = 0x6A

_WHO_AM_I = 0x0F
_EXPECTED_WHO_AM_I = 0x6B
_CTRL1_XL = 0x10
_CTRL2_G = 0x11
_CTRL3_C = 0x12
_OUTX_L_G = 0x22

#: CTRL3_C: software reset
_SOFTWARE_RESET = 0x01
#: CTRL3_C: block data update and register address auto increment
_BLOCK_DATA_UPDATE_AND_AUTO_INCREMENT = 0x44
#: time the sensor needs after a software reset, in seconds
_RESET_TIME = 0.05

#: output data rates in Hz and their ODR register codes, for both sensors
OUTPUT_DATA_RATES: dict[float, int] = {
    12.5: 0x1,
    26.0: 0x2,
    52.0: 0x3,
    104.0: 0x4,
    208.0: 0x5,
    416.0: 0x6,
}
#: accelerometer full scale ±4 g
_ACCELEROMETER_FULL_SCALE = 0b10 << 2
_G_PER_COUNT = 0.122e-3
#: gyro full scale ±500 °/s
_GYRO_FULL_SCALE = 0b01 << 2
_DEGREES_PER_SECOND_PER_COUNT = 17.5e-3


class I2CBus(Protocol):
    def read_byte_data(self, i2c_addr: int, register: int) -> int: ...

    def write_byte_data(self, i2c_addr: int, register: int, value: int) -> None: ...

    def read_i2c_block_data(
        self,
        i2c_addr: int,
        register: int,
        length: int,
    ) -> list[int]: ...


def output_data_rate(sample_rate_hz: float) -> float:
    """Return the lowest output data rate of the sensor at or above the given rate."""
    for rate in sorted(OUTPUT_DATA_RATES):
        if rate >= sample_rate_hz:
            return rate
    return max(OUTPUT_DATA_RATES)


class ISM330DHCX:
    """ST ISM330DHCX accelerometer and gyro."""

    def __init__(
        self,
        bus: I2CBus,
        address: int = DEFAULT_ADDRESS,
        sample_rate_hz: float = 104.0,
    ) -> None:
        self.bus = bus
        self.address = address

        who_am_i = bus.read_byte_data(address, _WHO_AM_I)
        if who_am_i != _EXPECTED_WHO_AM_I:
            msg = (
                f"No ISM330DHCX at I2C address {address:#04x}: "
                f"WHO_AM_I is {who_am_i:#04x}, expected {_EXPECTED_WHO_AM_I:#04x}"
            )
            raise OSError(msg)

        bus.write_byte_data(address, _CTRL3_C, _SOFTWARE_RESET)
        time.sleep(_RESET_TIME)
        bus.write_byte_data(address, _CTRL3_C, _BLOCK_DATA_UPDATE_AND_AUTO_INCREMENT)

        rate_code = OUTPUT_DATA_RATES[output_data_rate(sample_rate_hz)] << 4
        bus.write_byte_data(address, _CTRL1_XL, rate_code | _ACCELEROMETER_FULL_SCALE)
        bus.write_byte_data(address, _CTRL2_G, rate_code | _GYRO_FULL_SCALE)

    def read(self) -> tuple[Vector3, Vector3]:
        """
        Read the latest sample.

        :return: specific force in m/s² and angular rate in rad/s in the sensor's
            axes. A sensor lying flat reads +1 g on its z axis, which points up.
        """
        # gyro x, y, z followed by accelerometer x, y, z, little endian
        data = self.bus.read_i2c_block_data(self.address, _OUTX_L_G, 12)
        values = [
            int.from_bytes(bytes(data[i : i + 2]), "little", signed=True)
            for i in range(0, 12, 2)
        ]
        rate_scale = math.radians(_DEGREES_PER_SECOND_PER_COUNT)
        acceleration_scale = _G_PER_COUNT * GRAVITY
        return (
            (
                values[3] * acceleration_scale,
                values[4] * acceleration_scale,
                values[5] * acceleration_scale,
            ),
            (values[0] * rate_scale, values[1] * rate_scale, values[2] * rate_scale),
        )
