import pytest

import mmc5983ma
import rm3100


class _FakeBus:
    def __init__(self, address: int, registers: dict[int, int]) -> None:
        self.address = address
        self.registers = registers
        self.writes: list[tuple[int, int | list[int]]] = []
        self.blocks: list[list[int]] = []

    def read_byte_data(self, i2c_addr: int, register: int) -> int:
        assert i2c_addr == self.address
        return self.registers.get(register, 0)

    def write_byte_data(self, i2c_addr: int, register: int, value: int) -> None:
        assert i2c_addr == self.address
        self.writes.append((register, value))

    def write_i2c_block_data(
        self,
        i2c_addr: int,
        register: int,
        data: list[int],
    ) -> None:
        assert i2c_addr == self.address
        self.writes.append((register, data))

    def read_i2c_block_data(
        self,
        i2c_addr: int,
        register: int,
        length: int,
    ) -> list[int]:
        assert i2c_addr == self.address
        assert register in (0x00, 0x24)
        block = self.blocks.pop(0)
        assert len(block) == length
        return block


def _mmc_block(counts: tuple[int, int, int]) -> list[int]:
    x, y, z = (count + (1 << 17) for count in counts)
    return [
        x >> 10,
        (x >> 2) & 0xFF,
        y >> 10,
        (y >> 2) & 0xFF,
        z >> 10,
        (z >> 2) & 0xFF,
        ((x & 0x3) << 6) | ((y & 0x3) << 4) | ((z & 0x3) << 2),
    ]


def test_mmc5983ma_cancels_its_offset_with_set_and_reset() -> None:
    bus = _FakeBus(0x30, {0x2F: 0x30, 0x08: 0x01})
    sensor = mmc5983ma.MMC5983MA(bus)
    field = (20.0, -5.0, 45.0)
    offset = (3.0, 1.0, -2.0)
    # the field flips its sign between SET and RESET, the offset doesn't
    for sign in (1, -1):
        bus.blocks.append(
            _mmc_block(
                (
                    round((sign * field[0] + offset[0]) * 163.84),
                    round((sign * field[1] + offset[1]) * 163.84),
                    round((sign * field[2] + offset[2]) * 163.84),
                ),
            ),
        )

    measured = sensor.read()

    assert all(abs(m - f) < 0.01 for m, f in zip(measured, field, strict=True))
    assert (0x09, 0x08) in bus.writes
    assert (0x09, 0x10) in bus.writes


def test_mmc5983ma_wrong_chip_is_reported() -> None:
    with pytest.raises(OSError, match="product id"):
        mmc5983ma.MMC5983MA(_FakeBus(0x30, {0x2F: 0x00}))


def _rm_block(counts: tuple[int, int, int]) -> list[int]:
    data: list[int] = []
    for count in counts:
        data.extend((count & 0xFFFFFF).to_bytes(3, "big"))
    return data


def test_rm3100_sets_cycle_counts_and_converts_counts() -> None:
    bus = _FakeBus(0x20, {0x36: 0x22, 0x34: 0x80})
    sensor = rm3100.RM3100(bus)
    assert bus.writes == [(0x04, [0, 200] * 3)]

    bus.blocks.append(_rm_block((75 * 20, -75 * 5, 75 * 45)))
    assert sensor.read() == (20.0, -5.0, 45.0)
    assert (0x00, 0x70) in bus.writes


def test_rm3100_wrong_chip_is_reported() -> None:
    with pytest.raises(OSError, match="revision"):
        rm3100.RM3100(_FakeBus(0x20, {0x36: 0x00}))
