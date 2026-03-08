import datetime
import logging
import math
import threading
import time
from queue import Queue
from typing import Literal, TypeVar

import pyubx2
import pyubx2.ubxtypes_core
import serial

import config
import log
from structs import PositionData
from ubx.hnr_pvt import UbxHnrPvt, parse_ubx_hnr_pvt_message
from ubx.nav_dgps import parse_ubx_nav_dgps_message
from ubx.nav_dop import parse_ubx_nav_dop_message
from ubx.nav_pvt import UbxNavPvt, parse_ubx_nav_pvt_message
from ubx.shared_types import GnssFixType
from utils.vector import Vector2D

SERIAL_PORT = "/dev/ttyACM0"

logger = logging.getLogger(__name__)

ENABLED_MESSAGES = [
    # roll, pitch, heading, accuracy r/p/h
    "NAV-ATT",
    # timestamp, valid, gpsFix info, num satellite, lon, lat, height,
    # speed 2d, heading 2d, pdop accuracy h/b/speed
    "NAV-PVT",
    "NAV-DGPS",  # age of DGPS data, DGPS station ID
    "NAV-DOP",  # gdop, pdop, tdop, vdop, hdop, ndop, edop
    "NAV-SAT",  # numSvs, (gnssId, svId, carrier-to-noise-ratio, elevation, azimuth)[]
    "ESF-ALG",  # yaw, pitch, roll
    "ESF-STATUS",  # fusionMode
    # timestamp, valid, gpsFix info, lon, lat, height
    # speed 2d/3d, heading 2d, accuracy h/v/speed
    "HNR-PVT",
]

T = TypeVar("T")
U = TypeVar("U")


def key_from_val(dictionary: dict[T, U], value: U) -> T:
    """
    Get dictionary key corresponding to (unique) value.

    :param dict dictionary: dictionary
    :param object value: unique dictionary value
    :return: dictionary key
    :rtype: str
    :raises: KeyError: if no key found for value

    """
    val = None
    for key, val in dictionary.items():
        if val == value:
            return key
    msg = f"No key found for value {value}"
    raise KeyError(msg)


def disable_nmea_messages(ubr: pyubx2.UBXReader) -> None:
    for message_type in pyubx2.UBX_MSGIDS:
        if message_type[0:1] in (b"\xf0", b"\xf1"):
            send_configuration_message(ubr, message_type, 0)


def enable_ubx_messages(ubr: pyubx2.UBXReader) -> None:
    for message_id in ENABLED_MESSAGES:
        message_type = key_from_val(pyubx2.ubxtypes_core.UBX_MSGIDS, message_id)
        send_configuration_message(ubr, message_type, 1)


def send_configuration_message(
    ubr: pyubx2.UBXReader,
    message_type: bytes,
    message_rate: Literal[0, 1],
) -> None:
    """
    Set rate for specified message type via CFG-MSG.

    :param str message_id: type of config message
        (two bytes, first is message class, second is message ID)
    :param int message_rate: message rate (i.e. every nth position solution)
    """
    message_class = int.from_bytes(message_type[0:1], "little", signed=False)
    message_id = int.from_bytes(message_type[1:2], "little", signed=False)

    # select which receiver ports to apply rate to
    rates = {}
    ports = config.Config().GPS.PORTS
    for port in ports:
        rates[port] = message_rate

    # create CFG-MSG command
    msg = pyubx2.UBXMessage(
        "CFG",
        "CFG-MSG",
        pyubx2.SET,
        msgClass=message_class,
        msgID=message_id,
        rateDDC=rates.get("I2C", 0),
        rateUART1=rates.get("UART1", 0),
        rateUART2=rates.get("UART2", 0),
        rateUSB=rates.get("USB", 0),
        rateSPI=rates.get("SPI", 0),
    )
    if not isinstance(ubr.datastream, serial.Serial):
        error_message = "UBXReader datastream is not a serial port."
        raise TypeError(error_message)
    ubr.datastream.write(msg.serialize())


def reader_thread(queue: Queue[pyubx2.UBXMessage]) -> None:
    with serial.Serial(SERIAL_PORT, baudrate=115200, timeout=0) as console:
        ubx_reader = pyubx2.UBXReader(
            console,
            protfilter=pyubx2.NMEA_PROTOCOL
            | pyubx2.UBX_PROTOCOL
            | pyubx2.RTCM3_PROTOCOL,
            quitonerror=pyubx2.ERR_LOG,
            msgmode=pyubx2.GET,
            errorhandler=None,
        )
        # We want to use the UBX protocol to be able to receive interpolated GPS data
        logger.debug("Disabling NMEA messages")
        disable_nmea_messages(ubx_reader)
        logger.debug(
            "Enabling UBX messages: "
            "NAV-ATT, NAV-PVT, NAV-DOP, NAV-SAT, ESF-ALG, ESF-STATUS, HNR-PVT",
        )
        enable_ubx_messages(ubx_reader)
        while True:
            raw_data, parsed_data = ubx_reader.read()
            # TODO: check pickle-ability of raw_data and parsed_data
            log.data("gps", raw_data)
            if not isinstance(parsed_data, pyubx2.UBXMessage):
                logger.warning("Received non-UBX message: %s", parsed_data)
                continue
            queue.put(parsed_data)


def wait_for_gps_fix(queue: Queue[pyubx2.UBXMessage]) -> None:
    start = time.time()

    while True:
        message = queue.get()

        if message.identity != "NAV-PVT":
            continue

        if parse_ubx_nav_pvt_message(message).fix_type in (
            GnssFixType.FIX_2D,
            GnssFixType.FIX_3D,
            GnssFixType.GNSS_AND_DEAD_RECKONING,
        ):
            logger.info(
                "GPS Fix found, took %.1f seconds",
                round(time.time() - start, ndigits=1),
            )
            return


def log_position_info(last_position: PositionData, position: PositionData) -> None:
    if last_position.valid and not position.valid:
        logger.info("GPS fix lost")
    elif not last_position.valid and position.valid:
        logger.info("GPS fix reacquired")
    elif (
        last_position.valid and position.valid
    ) and logger.getEffectiveLevel() <= logging.DEBUG:
        if not (
            last_position.latitude is not None
            and last_position.longitude is not None
            and position.latitude is not None
            and position.longitude is not None
        ):
            error_message = (
                "Current or last position has missing latitude or longitude."
            )
            raise TypeError(error_message)

        movement_vector = Vector2D(
            x=last_position.latitude - position.latitude,
            y=last_position.longitude - position.longitude,
        )
        logger.debug(
            "Old Position: %.7f, %.7f",
            last_position.latitude,
            last_position.longitude,
        )
        logger.debug(
            "New Position: %.7f, %.7f",
            position.latitude,
            position.longitude,
        )
        logger.debug(
            "Calculated Track angle, %.1f",
            math.degrees(movement_vector.to_polar().angle),
        )
        logger.debug(
            "Provided Movement: Speed: %.2f, Direction: %s",
            position.speed,
            f"{position.true_course:.2f}"
            if position.true_course is not None
            else "None",
        )


def worker(
    message_queue: Queue[pyubx2.UBXMessage],
    position_queue: Queue[PositionData],
) -> None:
    """
    Read the GPS position data from the serial console and forward it to the queue.

    :param position_queue: Queue to output GPS position data to.
    """
    # Wait for first GPS fix
    wait_for_gps_fix(message_queue)

    differential_gps_data_age: int | None = None
    differential_reference_station_id: int | None = None
    number_of_satellites_used: int | None = None
    positional_dilution_of_precision: float | None = None
    horizontal_dilution_of_precision: float | None = None
    last_position: PositionData | None = None

    def submit_position_data(message: UbxNavPvt | UbxHnrPvt) -> None:
        nonlocal last_position

        position = PositionData(
            timestamp=int(
                datetime.datetime(
                    year=message.year,
                    month=message.month,
                    day=message.day,
                    hour=message.hour,
                    minute=message.minute,
                    second=message.second,
                    tzinfo=datetime.UTC,
                ).timestamp(),
            ),
            valid=message.fix_type
            in (
                GnssFixType.FIX_2D,
                GnssFixType.FIX_3D,
                GnssFixType.GNSS_AND_DEAD_RECKONING,
            ),
            latitude=message.lat,
            longitude=message.lon,
            altitude=message.height,
            geoidal_separation=message.height - message.h_msl,
            speed=message.g_speed,
            true_course=message.head_mot,
            differential_gps_data_age=differential_gps_data_age,
            differential_reference_station_id=differential_reference_station_id,
            number_satellites_used=number_of_satellites_used,
            positional_dilution_of_precision=positional_dilution_of_precision,
            horizontal_dilution_of_precision=horizontal_dilution_of_precision,
        )

        if last_position is not None:
            log_position_info(last_position, position)
        last_position = position
        if position.valid:
            position_queue.put(position)

    while True:
        message = message_queue.get()
        if message.identity == "NAV-DGPS":
            nav_dgps_message = parse_ubx_nav_dgps_message(message)
            differential_gps_data_age = nav_dgps_message.age
            differential_reference_station_id = nav_dgps_message.base_id
        elif message.identity == "NAV-DOP":
            nav_dop_message = parse_ubx_nav_dop_message(message)
            positional_dilution_of_precision = nav_dop_message.p_dop
            horizontal_dilution_of_precision = nav_dop_message.h_dop
        elif message.identity == "NAV-PVT":
            message = parse_ubx_nav_pvt_message(message)
            number_of_satellites_used = message.num_sv
            positional_dilution_of_precision = message.p_dop
            submit_position_data(message)
        elif message.identity == "HNR-PVT":
            hnr_pvt_message = parse_ubx_hnr_pvt_message(message)
            submit_position_data(hnr_pvt_message)


def init() -> Queue[PositionData]:
    """
    Initialize the GPS module.

    :return: A queue that will contain GPS position data.
    """
    position_queue: Queue[PositionData] = Queue()
    message_queue: Queue[pyubx2.UBXMessage] = Queue()

    threading.Thread(
        target=reader_thread,
        args=(message_queue,),
        daemon=True,
    ).start()

    threading.Thread(
        target=worker,
        args=(
            message_queue,
            position_queue,
        ),
        daemon=True,
    ).start()

    return position_queue
