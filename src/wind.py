import logging
import math
from queue import Queue

import n2k

import config
from structs import ApparentWindData, HeadingData, PositionData, TrueWindData
from utils.vector import PolarCoordinates, Vector2D

logger = logging.getLogger(__name__)

#: offset to apply to the compass data in radians. positive values indicate that
#: the north of the measurement unit is offset clockwise from the compass north.
COMPASS_OFFSET = math.radians(config.Config().SENSOR.COMPASS_OFFSET_DEGREES)


def read_position_queue(
    position_queue: Queue[PositionData],
    latest_position_time: int | None,
) -> tuple[list[Vector2D], int | None]:
    movement_buffer: list[Vector2D] = []
    while not position_queue.empty():
        position = position_queue.get()
        if position.true_course is None or position.speed is None:
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


def worker(
    position_queue: Queue[PositionData],
    heading_queue: Queue[HeadingData],
    wind_queue: Queue[ApparentWindData],
    consumers: list[Queue[TrueWindData]],
) -> None:
    current_movement_vector: Vector2D | None = None
    latest_position_time: int | None = None
    current_heading: HeadingData | None = None
    wind_buffer: list[tuple[Vector2D, int]] = []

    while True:
        # get wind data, blocking
        current_apparent_wind = wind_queue.get()

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

        apparent_wind_vector = Vector2D.from_polar(
            PolarCoordinates(
                angle=current_apparent_wind.wind_angle
                + current_heading.heading
                + COMPASS_OFFSET,
                magnitude=current_apparent_wind.wind_speed,
            ),
        )

        current_true_wind_vector = apparent_wind_vector - current_movement_vector

        # send true wind vector to output queues (mqtt, db, lora)
        wind_buffer.append((current_true_wind_vector, current_apparent_wind.timestamp))

        # drop wind data older than 10 seconds from the buffer
        wind_buffer = [
            wind_data
            for wind_data in wind_buffer
            if (wind_data[1] > current_apparent_wind.timestamp - 10)
        ]
        # compute the average true wind vector from the buffer
        if len(wind_buffer) == 0:
            continue

        average_true_wind_vector = sum(
            (wind_vector for [wind_vector, _] in wind_buffer),
            start=Vector2D(x=0, y=0),
        ) / len(wind_buffer)

        average_true_wind_polar = average_true_wind_vector.to_polar()
        average_true_wind = TrueWindData(
            wind_angle=average_true_wind_polar.angle,
            wind_speed=average_true_wind_polar.magnitude,
            timestamp=current_apparent_wind.timestamp,
        )

        if logger.getEffectiveLevel() <= logging.DEBUG:
            logger.debug(
                "Average True Wind (5seconds): %.2fkts from %.1f°",
                n2k.utils.meters_per_second_to_knots(average_true_wind_polar.magnitude),
                n2k.utils.rad_to_deg(average_true_wind_polar.angle),
            )
            current_true_wind_polar = current_true_wind_vector.to_polar()
            logger.debug(
                "Current True Wind: %.2fkts from %.1f°",
                n2k.utils.meters_per_second_to_knots(current_true_wind_polar.magnitude),
                n2k.utils.rad_to_deg(current_true_wind_polar.angle),
            )

        for consumer in consumers:
            consumer.put(average_true_wind)
