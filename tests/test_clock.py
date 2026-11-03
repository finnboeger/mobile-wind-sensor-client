import time

import pytest

import clock


@pytest.fixture
def calls(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    calls: list[float] = []
    monkeypatch.setattr(time, "time", lambda: 1_000.0)
    monkeypatch.setattr(time, "clock_settime", lambda _, t: calls.append(t))
    return calls


def test_small_offset_is_ignored(calls: list[float]) -> None:
    assert not clock.sync_from_gps(1_000_500)
    assert calls == []


def test_large_offset_sets_clock(calls: list[float]) -> None:
    assert clock.sync_from_gps(1_005_000)
    assert calls == [1_005.0]
    assert clock.sync_from_gps(995_000)


def test_missing_permission(monkeypatch: pytest.MonkeyPatch) -> None:
    def deny(*_: object) -> None:
        raise PermissionError

    monkeypatch.setattr(time, "time", lambda: 1_000.0)
    monkeypatch.setattr(time, "clock_settime", deny)
    assert not clock.sync_from_gps(1_005_000)
