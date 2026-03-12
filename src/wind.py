import logging
import math
from dataclasses import asdict
from queue import Queue

import n2k

import config
import log
from structs import (
    ApparentWindData,
    CorrectedApparentWindData,
    HeadingData,
    PositionData,
    TrueWindData,
    WindData,
)
from utils.vector import PolarCoordinates, Vector2D

logger = logging.getLogger(__name__)

#: offset to apply to the compass data in radians. positive values indicate that
#: the north of the measurement unit is offset clockwise from the compass north.
COMPASS_OFFSET = math.radians(config.Config().SENSOR.COMPASS_OFFSET_DEGREES)

#: amount of seconds to average wind data over
WIND_AVERAGING_SECONDS = 5


def read_position_queue(
    position_queue: Queue[PositionData],
    latest_position_time: int | None,
) -> tuple[list[Vector2D], int | None]:
    movement_buffer: list[Vector2D] = []
    while not position_queue.empty():
        position = position_queue.get()
        if position.true_course is None or position.speed is None:
            logger.debug("Discarding position data with missing course or speed.")
            continue
        movement_buffer.append(
            Vector2D.from_polar(
                PolarCoordinates(
                    angle=n2k.utils.deg_to_rad(position.true_course),
                    magnitude=n2k.utils.knots_to_meters_per_second(
                        position.speed,
                    ),
                ),
            ),
        )
        latest_position_time = position.timestamp

    return movement_buffer, latest_position_time


class WindBuffer:
    def __init__(self, max_age: int) -> None:
        self.buffer: list[tuple[Vector2D, int]] = []
        self.max_age = max_age

    def append(self, wind_data: tuple[Vector2D, int]) -> None:
        self.buffer.append(wind_data)

        # drop out-of-date wind data from the buffer
        self.buffer = [
            data for data in self.buffer if (data[1] > wind_data[1] - self.max_age)
        ]

    def ready(self) -> bool:
        return len(self.buffer) > 0

    def average(self) -> Vector2D:
        if not self.ready():
            msg = "WindBuffer is not ready, cannot compute average."
            raise ValueError(msg)

        return sum(
            (wind_vector for [wind_vector, _] in self.buffer),
            start=Vector2D(x=0, y=0),
        ) / len(self.buffer)

    def average_wind_data(self) -> WindData:
        average_polar = self.average().to_polar()
        return WindData(
            wind_angle=average_polar.angle,
            wind_speed=average_polar.magnitude,
            timestamp=self.buffer[-1][1],  # timestamp of the last entry
        )


def worker(
    position_queue: Queue[PositionData],
    heading_queue: Queue[HeadingData],
    wind_queue: Queue[ApparentWindData],
    consumers: list[Queue[tuple[CorrectedApparentWindData, TrueWindData]]],
) -> None:
    current_movement_vector: Vector2D | None = None
    latest_position_time: int | None = None
    current_heading: HeadingData | None = None
    # store the wind vectors of the last few seconds to compute an average
    true_wind_buffer = WindBuffer(max_age=WIND_AVERAGING_SECONDS)
    apparent_wind_buffer = WindBuffer(max_age=WIND_AVERAGING_SECONDS)

    while True:
        # get wind data, blocking
        current_apparent_wind = wind_queue.get()
        logger.debug(
            "Received wind data: %.2f m/s at %.1f°",
            current_apparent_wind.wind_speed,
            n2k.utils.rad_to_deg(current_apparent_wind.wind_angle),
        )

        # get every message in compass data, non-blocking
        while not heading_queue.empty():
            # store latest compass data
            current_heading = heading_queue.get()

        # get every message in position data, non-blocking
        movement_buffer, latest_position_time = read_position_queue(
            position_queue,
            latest_position_time,
        )

        # store average movement vector between wind updates
        movement_buffer_length = len(movement_buffer)
        if movement_buffer_length > 0:
            current_movement_vector = (
                sum(
                    movement_buffer,
                    start=Vector2D(x=0, y=0),
                )
                / movement_buffer_length
            )

        # assert that both position and compass data are not None
        if (
            current_movement_vector is None
            or latest_position_time is None
            or current_heading is None
        ):
            continue

        # assert that both position and compass data are up to date
        # (within 1 second of wind data)
        maximum_allowed_offset = 1000  # in milliseconds
        if (
            latest_position_time * 1000
            < current_apparent_wind.timestamp - maximum_allowed_offset
        ) or (
            current_heading.timestamp
            < (current_apparent_wind.timestamp - maximum_allowed_offset)
        ):
            logger.debug(
                "Position or heading data is too old, skipping wind update. "
                "Position time: %d, Heading time: %d, Wind time: %d",
                latest_position_time,
                current_heading.timestamp,
                current_apparent_wind.timestamp,
            )
            continue

        current_apparent_wind_vector = Vector2D.from_polar(
            PolarCoordinates(
                angle=current_apparent_wind.wind_angle
                + current_heading.heading
                + COMPASS_OFFSET,
                magnitude=current_apparent_wind.wind_speed,
            ),
        )

        current_true_wind_vector = (
            current_apparent_wind_vector - current_movement_vector
        )

        # send wind vector to output queues (mqtt, db, lora)
        true_wind_buffer.append(
            (current_true_wind_vector, current_apparent_wind.timestamp),
        )
        apparent_wind_buffer.append(
            (current_apparent_wind_vector, current_apparent_wind.timestamp),
        )

        # compute the average wind vector from the buffer
        if not true_wind_buffer.ready() or not apparent_wind_buffer.ready():
            continue

        average_true_wind = TrueWindData(**asdict(true_wind_buffer.average_wind_data()))
        average_apparent_wind = CorrectedApparentWindData(
            **asdict(apparent_wind_buffer.average_wind_data()),
        )

        log.data("true_wind", average_true_wind)

        if logger.getEffectiveLevel() <= logging.DEBUG:
            logger.debug(
                "Average True Wind (%d seconds): %.2fkts from %.1f°",
                WIND_AVERAGING_SECONDS,
                n2k.utils.meters_per_second_to_knots(average_true_wind.wind_speed),
                n2k.utils.rad_to_deg(average_true_wind.wind_angle),
            )
            current_true_wind_polar = current_true_wind_vector.to_polar()
            logger.debug(
                "Current True Wind: %.2fkts from %.1f°",
                n2k.utils.meters_per_second_to_knots(current_true_wind_polar.magnitude),
                n2k.utils.rad_to_deg(current_true_wind_polar.angle),
            )

            logger.debug(
                "Average Apparent Wind (%d seconds): %.2fkts from %.1f°",
                WIND_AVERAGING_SECONDS,
                n2k.utils.meters_per_second_to_knots(average_apparent_wind.wind_speed),
                n2k.utils.rad_to_deg(average_apparent_wind.wind_angle),
            )
            current_true_wind_polar = current_true_wind_vector.to_polar()
            logger.debug(
                "Current Apparent Wind: %.2fkts from %.1f°",
                n2k.utils.meters_per_second_to_knots(current_apparent_wind.wind_speed),
                n2k.utils.rad_to_deg(current_apparent_wind.wind_angle),
            )

        for consumer in consumers:
            consumer.put((average_apparent_wind, average_true_wind))
