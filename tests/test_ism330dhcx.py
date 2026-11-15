import math

import pytest

import ism330dhcx


class _FakeBus:
    def __init__(self, who_am_i: int = 0x6B) -> None:
        self.registers: dict[int, int] = {0x0F: who_am_i}
        self.writes: list[tuple[int, int]] = []
        self.block = [0] * 12

    def read_byte_data(self, i2c_addr: int, register: int) -> int:
        assert i2c_addr == 0x6A
        return self.registers.get(register, 0)

    def write_byte_data(self, i2c_addr: int, register: int, value: int) -> None:
        assert i2c_addr == 0x6A
        self.writes.append((register, value))

    def read_i2c_block_data(
        self,
        i2c_addr: int,
        register: int,
        length: int,
    ) -> list[int]:
        assert i2c_addr == 0x6A
        assert register == 0x22
        assert length == 12
        return self.block


def _encode(*values: int) -> list[int]:
    data: list[int] = []
    for value in values:
        data.extend(value.to_bytes(2, "little", signed=True))
    return data


def test_configures_rate_and_full_scale() -> None:
    bus = _FakeBus()
    ism330dhcx.ISM330DHCX(bus, sample_rate_hz=100)
    # reset, then block data update + auto increment, 104 Hz ±4 g, 104 Hz ±500 °/s
    assert bus.writes == [(0x12, 0x01), (0x12, 0x44), (0x10, 0x48), (0x11, 0x44)]


def test_wrong_chip_is_reported() -> None:
    with pytest.raises(OSError, match="WHO_AM_I"):
        ism330dhcx.ISM330DHCX(_FakeBus(who_am_i=0x6C))


def test_converts_counts_to_si_units() -> None:
    bus = _FakeBus()
    sensor = ism330dhcx.ISM330DHCX(bus)
    # gyro 100 °/s about z, accelerometer 1 g on z
    bus.block = _encode(0, 0, round(100 / 17.5e-3), 0, -100, round(1 / 0.122e-3))

    acceleration, angular_rate = sensor.read()

    assert abs(acceleration[2] - ism330dhcx.GRAVITY) < 0.01
    assert acceleration[1] < 0
    assert abs(angular_rate[2] - math.radians(100)) < 1e-3


@pytest.mark.parametrize(
    ("requested", "expected"),
    [(10, 12.5), (50, 52.0), (104, 104.0), (150, 208.0), (1000, 416.0)],
)
def test_output_data_rate(requested: float, expected: float) -> None:
    assert ism330dhcx.output_data_rate(requested) == expected
