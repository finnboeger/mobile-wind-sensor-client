"""Motion of the measurement unit from the IMU and the compass."""

from attitude import AttitudeEstimator
from heading import HeadingFilter
from structs import HeadingData, ImuSample


class UnitMotion:
    """Tracks the unit's attitude and heading."""

    def __init__(self, heading_filter: HeadingFilter | None = None) -> None:
        self.attitude = AttitudeEstimator()
        self.heading = heading_filter or HeadingFilter()

    def add_imu(self, samples: list[ImuSample]) -> None:
        for sample in samples:
            self.attitude.update(sample)
            if self.attitude.initialized:
                self.heading.predict(
                    sample.timestamp,
                    self.attitude.heading_rate(sample),
                )

    def add_compass(self, heading: HeadingData) -> None:
        self.heading.update(heading.timestamp, heading.heading)

    def add(self, samples: list[ImuSample], headings: list[HeadingData]) -> None:
        """Add IMU samples and compass readings in the order they were measured."""
        events: list[ImuSample | HeadingData] = [*samples, *headings]
        # sorted() is stable, so samples with the same timestamp keep their order
        for event in sorted(events, key=lambda event: event.timestamp):
            if isinstance(event, ImuSample):
                self.add_imu([event])
            else:
                self.add_compass(event)

    def heading_at(self, timestamp: int) -> float | None:
        """Return the heading in rad at the given time, or None if not known yet."""
        return self.heading.heading_at(timestamp)
