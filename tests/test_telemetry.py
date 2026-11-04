import telemetry


def test_update_and_snapshot() -> None:
    telemetry.clear()
    telemetry.update("gps", satellites=7)
    telemetry.update("gps", valid=True)
    telemetry.increment("true_wind", "skipped_stale")
    telemetry.increment("true_wind", "skipped_stale")
    telemetry.set_values("true_wind", waiting_for=["up-to-date heading"])

    snapshot = telemetry.snapshot()
    assert snapshot["gps"]["satellites"] == 7
    assert snapshot["gps"]["valid"] is True
    assert "updated" in snapshot["gps"]
    assert snapshot["true_wind"]["skipped_stale"] == 2
    assert snapshot["true_wind"]["waiting_for"] == ["up-to-date heading"]
    assert "updated" not in snapshot["true_wind"]

    # the snapshot is a copy
    snapshot["gps"]["satellites"] = 0
    assert telemetry.snapshot()["gps"]["satellites"] == 7
