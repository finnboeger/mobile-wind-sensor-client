import logging
import logging.handlers
import threading
from queue import Queue

import n2k

import gps
import nmea
import wind
from config import Config
from structs import CorrectedApparentWindData, PositionData, TrueWindData


def init_logging() -> None:
    # Begin initializing logging
    root_logger = logging.getLogger()
    logger = logging.getLogger(__name__)

    formatter = logging.Formatter(
        "%(asctime)s,%(msecs).03d :: %(levelname)s :: %(name)s ::  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    startup_handler = logging.StreamHandler()
    startup_handler.setFormatter(formatter)
    root_logger.addHandler(startup_handler)
    root_logger.setLevel(logging.INFO)

    # Load config
    config = Config()

    # Finish initializing logging
    logger.info("Setting log level to %d", config.LOGGING.LOG_LEVEL)
    root_logger.setLevel(config.LOGGING.LOG_LEVEL)

    if config.LOGGING.LOG_FILE is not None:
        logger.debug("Replacing stderr handler with file handler")
        handler = logging.handlers.RotatingFileHandler(
            config.LOGGING.LOG_FILE,
            encoding="utf-8",
            maxBytes=config.LOGGING.MAX_LOG_SIZE * 1000,
            backupCount=config.LOGGING.MAX_LOG_FILES,
        )
        handler.setFormatter(formatter)
        root_logger.removeHandler(startup_handler)
        root_logger.addHandler(handler)


def forward_position(
    n2k_node: n2k.Node,
    input_queue: Queue[PositionData],
    output_queue: Queue[PositionData],
) -> None:
    """
    Forward position data from the input to the NMEA2000 network and the output queue.

    Note: The Garmin display GMI20 only displays the GPS data if the COG/SOG message
    is sent as well, and also has both its fields set to a proper value (i.e. not None).

    :param n2k_node: NMEA2000 Node to send messages through.
    :param input_queue: Input queue containing position data.
    :param output_queue: Output queue to forward position data.
    """
    while True:
        position = input_queue.get()

        message = n2k.messages.create_n2k_gnss_data_message(
            n2k.messages.GNSSPositionData(
                days_since_1970=position.timestamp // 86400,  # 86400 seconds in a day
                seconds_since_midnight=position.timestamp % 86400,
                latitude=position.latitude,
                longitude=position.longitude,
                altitude=position.altitude,
                gnss_type=n2k.types.N2kGNSSType.GPS,
                gnss_method=n2k.types.N2kGNSSMethod.GNSS_fix,
                n_satellites=position.number_satellites_used,
                hdop=position.horizontal_dilution_of_precision,
                pdop=position.positional_dilution_of_precision,
                geoidal_separation=position.geoidal_separation,
                n_reference_stations=None,
                reference_station_type=None,
                reference_station_id=position.differential_reference_station_id,
                age_of_correction=position.differential_gps_data_age,
            ),
        )
        n2k_node.send_msg(message)

        message = n2k.messages.create_n2k_lat_long_rapid_message(
            n2k.messages.LatLonRapid(
                latitude=position.latitude,
                longitude=position.longitude,
            ),
        )
        n2k_node.send_msg(message)

        message = n2k.messages.create_n2k_cog_sog_rapid_message(
            n2k.messages.CogSogRapid(
                cog=(
                    n2k.utils.deg_to_rad(position.true_course)
                    if position.true_course is not None
                    # Default to 0 if None, as the Display won't show the data otherwise
                    else 0.0
                ),
                sog=n2k.utils.knots_to_meters_per_second(position.speed)
                if position.speed is not None
                else 0.0,
                heading_reference=n2k.types.N2kHeadingReference.true,
            ),
        )
        n2k_node.send_msg(message)

        output_queue.put(position)


if __name__ == "__main__":
    init_logging()

    position_queue = gps.init()
    n2k_node, heading_queue, wind_queue = nmea.init()

    position_queue2: Queue[PositionData] = Queue()

    # Forward the GPS data to the NMEA2000 network
    threading.Thread(
        target=forward_position,
        args=(n2k_node, position_queue, position_queue2),
        daemon=True,
    ).start()

    # TODO: init mqtt
    # TODO: potentially init local server

    forward_wind_to_nmea_queue: Queue[
        tuple[CorrectedApparentWindData, TrueWindData]
    ] = Queue()

    # Start the worker thread to compute the true wind and send it the the consumers
    worker_thread = threading.Thread(
        target=wind.worker,
        args=(position_queue2, heading_queue, wind_queue, [forward_wind_to_nmea_queue]),
    )
    worker_thread.start()

    # Forward the true and corrected apparent wind data to the NMEA2000 network
    forward_wind_to_nmea_thread = threading.Thread(
        target=nmea.forward_wind,
        args=(n2k_node, forward_wind_to_nmea_queue),
        daemon=True,
    )
    forward_wind_to_nmea_thread.start()

    worker_thread.join()
