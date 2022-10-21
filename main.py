# import sys
# sys.path.insert(0, "../n2k")
import math

import pynmea2
import serial
import n2k
import multiprocessing
import time
import queue
import can
import asyncio

import mqtt
import position


COMPASS_OFFSET = 0  # positive: north of the measurement unit is offset clockwise from the compass north


# TODO: rework init
def init_gps(control_console: serial.Serial, output_console: serial.Serial) -> None:
    start = time.time()
    # TODO: Hotstart
    command = "AT+CGNSPWR=1"
    control_console.write((command + "\r\n").encode())
    time.sleep(0.1)
    response = control_console.read(1024)
    # TODO: Handle initialisation errors
    assert response.decode() == command + "\r\r\n" + "OK\r\n"

    # Wait for first GPS fix
    while True:
        time.sleep(1)
        data = b""
        while data[-2:] != b"\r\n":
            data += output_console.read(1024)
        data = data.decode()
        parsed = [pynmea2.parse(d) for d in data.split("\r\n") if len(d) > 0]
        for sentence in parsed:
            if isinstance(sentence, pynmea2.RMC) and sentence.status == "A":
                print("GPS Fix found, took", round(time.time() - start), "seconds")
                # print(repr(sentence))
                return


async def gps_listener(console: serial.Serial, q: multiprocessing.Queue, ) -> None:
    time_ms = int(time.time() * 1000)

    bufsize = 1024
    sid = 0
    while True:
        # TODO: probably read all at once so that we can properly create the n2k messages :/
        # raw_data = console.read_until("\r\n")

        raw_data = b""
        while raw_data[-2:] != b"\r\n":
            raw_data += console.read(bufsize)

        try:
            data = raw_data.decode()
        except UnicodeDecodeError as e:
            print(e)
            continue
        data = [x for x in data.split("\r\n") if x is not None and len(x) > 0]
        parsed = [pynmea2.parse(d) for d in data]
        for sentence in parsed:
            if isinstance(sentence, pynmea2.GSV):
                # GNSS Satellites in view http://www.nmea.de/nmea0183datensaetze.html#gsv
                pass
            elif isinstance(sentence, pynmea2.GGA):
                # GNSS Position fix http://www.nmea.de/nmea0183datensaetze.html#gga
                # TODO: check if seconds since midnight is UTC or timezone specific
                # TODO: unknown which format timestamp is
                if sentence.timestamp is None:
                    continue
                msg = n2k.messages.set_n2k_gnss_data(
                    sid=sid,
                    days_since_1970=int(sentence.timestamp // (60*60*24)),
                    seconds_since_midnight=float(sentence.timestamp % 86400),
                    latitude=float(sentence.lat) * 1 if sentence.lat_dir == "N" else -1,
                    longitude=float(sentence.lon) * 1 if sentence.lon_dir == "E" else -1,
                    altitude=float(sentence.alt),
                    gnss_type=n2k.types.N2kGNSSType(0),
                    gnss_method=n2k.types.N2kGNSSMethod(1),
                    n_satellites=int(sentence.num_sats),
                    hdop=float(sentence.horizontal_dil),
                    pdop=float(0),
                    geoidal_separation=float(sentence.geo_sep),
                    n_reference_station=0,  # TODO: pass along reference station
                    reference_station_type=None,
                    reference_station_id=None,
                    age_of_correction=None,
                )
                q.put(msg)
            elif isinstance(sentence, pynmea2.VTG):
                # GNSS Speed over Ground http://www.nmea.de/nmea0183datensaetze.html#vtg
                # increase our sid every time we come across this kind of message as it seems to start each group and
                # is only send once
                sid = (sid + 1) % 250  # not sure if 255 is reserved therefore we roll over beforehand
            elif isinstance(sentence, pynmea2.RMC):
                # GNSS combined position, movement, time data,
                # also contains compass declination http://www.nmea.de/nmea0183datensaetze.html#rmc
                if sentence.true_course is None:
                    continue
                msg = n2k.messages.set_n2k_cog_sog_rapid(
                    sid=sid,
                    heading_reference=n2k.types.N2kHeadingReference(0),  # degrees true
                    cog=n2k.utils.deg_to_rad(float(sentence.true_course)),
                    sog=n2k.utils.knots_to_meters_per_second(float(sentence.spd_over_grnd)),
                )
                q.put(msg)
            elif isinstance(sentence, pynmea2.GSA):
                # GNSS Dilution of Precision data http://www.nmea.de/nmea0183datensaetze.html#gsa
                pass
            else:
                raise Exception("Unknown message:" + repr(sentence))


def combine_forces(angle1: float, force1: float, angle2: float, force2: float) -> (float, float):
    """

    :param angle1: in radians
    :param force1: unit-less
    :param angle2: in radians
    :param force2: same unit as force 1
    :return: angle (in radians), combined force
    """
    vector1 = (force1 * math.cos(angle1), force1 * math.sin(angle1))
    vector2 = (force2 * math.cos(angle2), force2 * math.sin(angle2))
    vector_r = (vector1[0] + vector2[0], vector1[1] + vector2[1])

    result_angle = (math.atan2(vector_r[1], vector_r[0])) % 360
    result_magnitude = math.sqrt(vector_r[0] ** 2 + vector_r[1] ** 2)
    return result_angle, result_magnitude


class Handler(n2k.MessageHandler):
    pos_send_queue: multiprocessing.Queue
    pos_recv_queue: multiprocessing.Queue
    compass_heading: float = 0.0
    speed: float = 0.0
    heading: float = 0.0

    def __init__(self,
                 node: n2k.Node,
                 pos_send_queue: multiprocessing.Queue,
                 pos_recv_queue: multiprocessing.Queue):
        super().__init__(0, node)
        self.pos_send_queue = pos_send_queue
        self.pos_recv_queue = pos_recv_queue

    def using_previous_data(self, msg: n2k.Message) -> None:
        wind_data = n2k.messages.parse_n2k_wind_speed(msg)
        twd, tws = combine_forces(wind_data.wind_angle, wind_data.wind_speed, self.heading, -self.speed)
        self._node.send_msg(n2k.messages.set_n2k_wind_speed(
            sid=wind_data.sid,
            wind_speed=tws,
            wind_angle=twd,
            wind_reference=n2k.types.N2kWindReference(0),
        ))

    def handle_msg(self, msg: n2k.Message) -> None:
        if not (msg.pgn == n2k.PGN.WindSpeed or msg.pgn == n2k.PGN.VesselHeading):
            return

        # TODO: basically everything, message flow is weird at the moment

        if msg.pgn == n2k.PGN.WindSpeed:
            try:
                speed_data = self.pos_recv_queue.get(False, 0.1)
            except Exception:
                self.using_previous_data(msg)
                return
            try:
                gnss_data = self.pos_recv_queue.get(False, 0.1)
            except Exception:
                self.using_previous_data(msg)
                return
            self._node.send_msg(speed_data)
            self._node.send_msg(gnss_data)
            if speed_data.pgn != n2k.PGN.CogSogRapid and gnss_data.pgn == n2k.PGN.CogSogRapid:
                speed_data, gnss_data = gnss_data, speed_data
            elif speed_data.pgn != n2k.PGN.CogSogRapid and gnss_data.pgn != n2k.PGN.CogSogRapid:
                print("both messages arent speed data")
                self.using_previous_data(msg)
                return
            wind_data = n2k.messages.parse_n2k_wind_speed(msg)
            movement_data = n2k.messages.parse_n2k_cog_sog_rapid(speed_data)
            twd, tws = combine_forces(wind_data.wind_angle, wind_data.wind_speed, movement_data.cog, -movement_data.sog)
            self._node.send_msg(n2k.messages.set_n2k_wind_speed(
                sid=wind_data.sid,
                wind_speed=tws,
                wind_angle=twd,
                wind_reference=n2k.types.N2kWindReference(0),
            ))

        if msg.pgn == n2k.PGN.VesselHeading:
            self.compass_heading = n2k.messages.parse_n2k_heading(msg).heading
            self.pos_send_queue.put(msg)


if __name__ == "__main__":
    with serial.Serial("/dev/ttyUSB1", 115200, timeout=0, rtscts=True, dsrdtr=True) as gps_console, \
            serial.Serial("/dev/ttyUSB2", 115200, timeout=0, rtscts=True, dsrdtr=True) as sim7000control:
        # Initialize connection with GPS module and wait for first fix
        init_gps(sim7000control, gps_console)

        # Setup position calculation
        position_send_queue = multiprocessing.Queue()
        position_recv_queue = multiprocessing.Queue()

        position_process = multiprocessing.Process(target=position.worker,
                                                   args=(position_send_queue, position_recv_queue))
        position_process.start()

        # Setup GPS input
        asyncio.run(gps_listener(gps_console, position_send_queue))

        # Setup mqtt sending
        # TODO: set max size -> check if queue is full before put and removing first item if so
        mqtt_queue = multiprocessing.Queue()

        mqtt_process = multiprocessing.Process(target=mqtt.worker, args=(mqtt_queue,))
        mqtt_process.start()

        # Setup N2k Node
        bus = can.Bus('can0', interface='socketcan')
        notifier = can.Notifier(bus, [])

        device_information = n2k.DeviceInformation()
        device_information.unique_number = 1
        device_information.device_function = 130
        device_information.device_class = 25
        device_information.manufacturer_code = 2046
        device_information.industry_group = 4

        n2k_node = n2k.Node(bus, device_information)
        n2k_node.set_product_information("Test", "0.0.1", "Dev", "00000000001", 5)
        n2k_node.set_configuration_information()

        handler = Handler(n2k_node, position_send_queue, position_recv_queue)
        n2k_node.attach_msg_handler(handler)
        notifier.add_listener(n2k_node)

        while True:
            time.sleep(0.1)
            # TODO: allow for graceful shutdown?
