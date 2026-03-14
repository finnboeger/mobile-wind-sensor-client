import math
from queue import Queue

import can
import n2k

import log
from structs import (
    ApparentWindData,
    CorrectedApparentWindData,
    HeadingData,
    PositionData,
    TrueWindData,
    WindData,
)

MAX_WIND_SPEED = 200  # m/s, values above this are considered invalid


def init() -> tuple[n2k.Node, Queue[HeadingData], Queue[WindData]]:
    """
    Initialize the NMEA2000 Node.

    :return: _description_
    """
    bus = can.Bus("can0", interface="socketcan")

    device_information = n2k.DeviceInformation(
        unique_number=1,
        device_function=130,
        device_class=25,
        manufacturer_code=2046,
        industry_group=4,
    )
    node = n2k.Node(bus, device_information=device_information)

    # TODO: Set up the node with appropriate product and configuration information
    node.set_product_information("Test", "0.0.1", "Dev", "00000000001", 5)
    node.set_configuration_information()

    handler = Handler(node)
    node.attach_msg_handler(handler)

    # Create a notifier to handle incoming messages
    can.Notifier(bus, [node])

    return node, handler.heading_queue, handler.wind_queue


class Handler(n2k.MessageHandler):
    heading_queue: Queue[HeadingData]
    wind_queue: Queue[WindData]

    def __init__(
        self,
        node: n2k.Node,
    ) -> None:
        super().__init__(0, node)
        self.heading_queue = Queue()
        self.wind_queue = Queue()

    def handle_msg(self, msg: n2k.Message) -> None:
        if msg.pgn == n2k.PGN.WindSpeed:
            wind_data = n2k.messages.parse_n2k_wind_speed(msg)
            if wind_data.wind_reference != n2k.types.N2kWindReference.Apparent:
                return
            if wind_data.wind_speed is None or wind_data.wind_speed > MAX_WIND_SPEED:
                # disregard wind data above the maximum speed
                return
            if wind_data.wind_angle is None:
                # disregard wind data without a valid angle
                return
            wind_data = ApparentWindData(
                wind_angle=wind_data.wind_angle,
                wind_speed=wind_data.wind_speed,
                timestamp=msg.msg_time,
            )
            self.wind_queue.put(wind_data)
            log.data("wind", wind_data)
            return

        if msg.pgn == n2k.PGN.VesselHeading:
            heading = n2k.messages.parse_n2k_heading(msg).heading
            if heading is None or heading > math.tau or heading < -math.tau:
                # bad heading (radians)
                return
            heading_data = HeadingData(
                heading=heading % math.tau,
                timestamp=msg.msg_time,
            )
            self.heading_queue.put(heading_data)
            log.data("heading", heading_data)


def forward_position(
    n2k_node: n2k.Node,
    input_queue: Queue[PositionData],
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
        position = None
        while not input_queue.empty() or position is None:
            position = input_queue.get()

        timestamp_seconds = position.timestamp // 1000
        message = n2k.messages.create_n2k_gnss_data_message(
            n2k.messages.GNSSPositionData(
                days_since_1970=timestamp_seconds // 86400,  # 86400 seconds in a day
                seconds_since_midnight=timestamp_seconds % 86400,
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


def forward_wind(
    n2k_node: n2k.Node,
    queue: Queue[tuple[CorrectedApparentWindData, TrueWindData]],
) -> None:
    """
    Forward wind data from the input to the NMEA2000 network and the output queue.

    Both the corrected apparent wind (original apparent wind combined with heading data
    to calculate the apparent wind direction instead of angle) and the true wind data
    are sent as NMEA2000 messages.

    :param n2k_node: NMEA2000 Node to send messages through.
    :param input_queue: Input queue containing position data.
    :param output_queue: Output queue to forward position data.
    """
    while True:
        apparent_wind_data, true_wind_data = queue.get()
        msg = n2k.messages.create_n2k_wind_speed_message(
            n2k.messages.WindSpeed(
                wind_speed=true_wind_data.wind_speed,
                wind_angle=true_wind_data.wind_angle,
                wind_reference=n2k.types.N2kWindReference.TrueNorth,
            ),
        )
        n2k_node.send_msg(msg)
        msg = n2k.messages.create_n2k_wind_speed_message(
            n2k.messages.WindSpeed(
                wind_speed=apparent_wind_data.wind_speed,
                wind_angle=apparent_wind_data.wind_angle,
                wind_reference=n2k.types.N2kWindReference.Apparent,
            ),
        )
        n2k_node.send_msg(msg)
