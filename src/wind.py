import logging
import math
from dataclasses import asdict
from queue import Queue

import n2k

import config
import log
from heading import HeadingFilter
from kalman import VelocityKalmanFilter
from motion import UnitMotion
from structs import (
    ApparentWindData,
    CorrectedApparentWindData,
    HeadingData,
    ImuSample,
    PositionData,
    TrueWindData,
    VelocityData,
    WindData,
    WindOutputQueue,
)
from utils.vector import PolarCoordinates, Vector2D

logger = logging.getLogger(__name__)

#: offset to apply to the compass data in radians. positive values indicate that
#: the north of the measurement unit is offset clockwise from the compass north.
COMPASS_OFFSET = math.radians(config.Config().SENSOR.COMPASS_OFFSET_DEGREES)

#: amount of milliseconds to average wind data over
WIND_AVERAGING_MILLISECONDS = 5 * 1000


#: speed accuracy in m/s assumed for logged positions that don't include one
DEFAULT_SPEED_ACCURACY = 0.5


def position_velocity(position: PositionData) -> Vector2D:
    """Return the velocity over ground of a position, x is north and y is east."""
    if position.velocity_north is not None and position.velocity_east is not None:
        return Vector2D(x=position.velocity_north, y=position.velocity_east)
    return Vector2D.from_polar(
        PolarCoordinates(
            angle=n2k.utils.deg_to_rad(position.true_course),
            magnitude=position.speed,
        ),
    )


def read_position_queue(
    position_queue: Queue[PositionData],
    latest_position_time: int | None,
    velocity_filter: VelocityKalmanFilter | None,
    motion: UnitMotion | None = None,
) -> tuple[list[Vector2D], int | None]:
    """
    Read all waiting positions and feed them into the velocity filter.

    :param motion: motion of the unit, to remove the sway of the GPS antenna
    :return: velocities of all read positions at the IMU and the timestamp of the
        latest one
    """
    movement_buffer: list[Vector2D] = []
    while not position_queue.empty():
        position = position_queue.get()
        velocity = position_velocity(position)
        if motion is not None:
            velocity -= motion.lever_arm_velocity(
                position.timestamp,
                motion.gps_antenna_position,
            )
        movement_buffer.append(velocity)
        latest_position_time = position.timestamp

        # HNR-PVT repeats or interpolates the GNSS solution, so only the GNSS
        # measurements are independent. Positions logged before the source was
        # recorded can't be told apart and are all used.
        if velocity_filter is not None and position.source in (None, "NAV-PVT"):
            velocity_filter.update(
                position.timestamp,
                velocity,
                position.speed_accuracy
                if position.speed_accuracy is not None
                else DEFAULT_SPEED_ACCURACY,
            )

    return movement_buffer, latest_position_time


def movement_at(
    velocity_filter: VelocityKalmanFilter | None,
    timestamp: int,
    movement_buffer: list[Vector2D],
) -> Vector2D | None:
    """
    Return the boat's velocity over ground at the time of a wind measurement.

    :param velocity_filter: velocity filter, or None to average the GPS samples
    :param timestamp: time of the wind measurement in milliseconds
    :param movement_buffer: velocities received since the last wind measurement
    :return: the velocity, or None if no new estimate is available
    """
    if velocity_filter is None:
        # average movement vector between wind updates
        if len(movement_buffer) == 0:
            return None
        return sum(movement_buffer, start=Vector2D(x=0, y=0)) / len(movement_buffer)

    # evaluate the filtered velocity at the time of the wind measurement
    velocity = velocity_filter.velocity_at(timestamp)
    if velocity is not None:
        log.data(
            "velocity",
            VelocityData(
                velocity_north=velocity.x,
                velocity_east=velocity.y,
                timestamp=timestamp,
            ),
        )
    return velocity


def read_heading_queues(
    heading_queue: Queue[HeadingData],
    imu_queue: Queue[list[ImuSample]] | None,
    motion: UnitMotion | None,
    current_heading: HeadingData | None,
) -> HeadingData | None:
    """
    Read all waiting compass readings and IMU samples and feed them into the motion.

    :return: the latest compass reading
    """
    headings: list[HeadingData] = []
    while not heading_queue.empty():
        headings.append(heading_queue.get())

    samples: list[ImuSample] = []
    while imu_queue is not None and not imu_queue.empty():
        samples.extend(imu_queue.get())

    if motion is not None:
        motion.add(samples, headings)

    return headings[-1] if headings else current_heading


def heading_at(
    motion: UnitMotion | None,
    timestamp: int,
    current_heading: HeadingData,
) -> float:
    """
    Return the unit's heading at the time of a wind measurement.

    :param motion: motion of the unit, or None to use the latest compass reading
    :param timestamp: time of the wind measurement in milliseconds
    :param current_heading: latest compass reading
    :return: heading in rad
    """
    heading = motion.heading_at(timestamp) if motion is not None else None
    if heading is None:
        return current_heading.heading
    log.data("unit_heading", HeadingData(heading=heading, timestamp=timestamp))
    return heading


def sensor_velocity(
    motion: UnitMotion | None,
    timestamp: int,
    boat_velocity: Vector2D,
) -> Vector2D:
    """Return the wind sensor's velocity: the boat's plus the sway of the mast."""
    if motion is None:
        return boat_velocity
    return boat_velocity + motion.lever_arm_velocity(
        timestamp,
        motion.wind_sensor_position,
    )


def create_motion() -> UnitMotion | None:
    """Return the unit's motion tracker if the IMU is enabled."""
    imu_config = config.Config().IMU
    if not imu_config.ENABLED:
        return None
    # the heading from the unit's own magnetometers isn't damped
    compass_lag = (
        0.0 if config.Config().MAGNETOMETER.ENABLED else imu_config.COMPASS_LAG_SECONDS
    )
    return UnitMotion(
        HeadingFilter(
            compass_sigma=math.radians(imu_config.COMPASS_SIGMA_DEGREES),
            compass_lag=compass_lag,
        ),
        wind_sensor_position=imu_config.WIND_SENSOR_POSITION,
        gps_antenna_position=imu_config.GPS_ANTENNA_POSITION,
    )


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
    consumers: list[WindOutputQueue],
    imu_queue: Queue[list[ImuSample]] | None = None,
) -> None:
    current_movement_vector: Vector2D | None = None
    latest_position_time: int | None = None
    current_heading: HeadingData | None = None
    # store the wind vectors of the last few seconds to compute an average
    true_wind_buffer = WindBuffer(max_age=WIND_AVERAGING_MILLISECONDS)
    apparent_wind_buffer = WindBuffer(max_age=WIND_AVERAGING_MILLISECONDS)
    filter_config = config.Config().FILTER
    velocity_filter = (
        VelocityKalmanFilter(
            acceleration_sigma=filter_config.ACCELERATION_SIGMA,
            acceleration_time_constant=filter_config.ACCELERATION_TIME_CONSTANT,
        )
        if filter_config.ENABLED
        else None
    )
    motion = create_motion() if imu_queue is not None else None

    while True:
        # get wind data, blocking
        current_apparent_wind = wind_queue.get()
        logger.debug(
            "Received wind data: %.2f m/s at %.1f°",
            current_apparent_wind.wind_speed,
            n2k.utils.rad_to_deg(current_apparent_wind.wind_angle),
        )

        # get every compass reading and IMU sample, non-blocking
        current_heading = read_heading_queues(
            heading_queue,
            imu_queue,
            motion,
            current_heading,
        )

        # get every message in position data, non-blocking
        movement_buffer, latest_position_time = read_position_queue(
            position_queue,
            latest_position_time,
            velocity_filter,
            motion,
        )

        new_movement_vector = movement_at(
            velocity_filter,
            current_apparent_wind.timestamp,
            movement_buffer,
        )
        if new_movement_vector is not None:
            current_movement_vector = new_movement_vector

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
            latest_position_time
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
                + heading_at(motion, current_apparent_wind.timestamp, current_heading)
                + COMPASS_OFFSET,
                magnitude=current_apparent_wind.wind_speed,
            ),
        )

        current_true_wind_vector = current_apparent_wind_vector - sensor_velocity(
            motion,
            current_apparent_wind.timestamp,
            current_movement_vector,
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
                WIND_AVERAGING_MILLISECONDS,
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
                WIND_AVERAGING_MILLISECONDS,
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
