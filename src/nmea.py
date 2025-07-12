import math
from queue import Queue

import can
import n2k

from structs import ApparentWindData, HeadingData

MAX_WIND_SPEED = 200  # m/s, values above this are considered invalid


def init() -> tuple[n2k.Node, Queue[HeadingData], Queue]:
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
    wind_queue: Queue

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
