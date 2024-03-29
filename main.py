# import sys
# sys.path.insert(0, "../n2k")
import math
import threading

import pynmea2
import serial
import n2k
from typings import MQTTMessage
from typing import List
import multiprocessing
import time
import queue
import can
import asyncio

import mqtt
import position
import os


COMPASS_OFFSET = -110  # positive: north of the measurement unit is offset clockwise from the compass north. In degrees
LOGFILE = "log.log"
i = 0
while os.path.exists(LOGFILE + str(i)):
    i += 1
LOG = open(LOGFILE + str(i), "w")


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

    print("Waiting for GPS")

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


def gps_listener(console: serial.Serial, q: multiprocessing.Queue, ) -> None:
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
            print(" ".join(map(str, [time.time(), sentence])), file=LOG, flush=True)
            if isinstance(sentence, pynmea2.GSV):
                # GNSS Satellites in view http://www.nmea.de/nmea0183datensaetze.html#gsv
                pass
            elif isinstance(sentence, pynmea2.GGA):
                # GNSS Position fix http://www.nmea.de/nmea0183datensaetze.html#gga
                # TODO: check if seconds since midnight is UTC or timezone specific
                if sentence.timestamp is None:
                    continue
                timestamp = (sentence.timestamp.hour * 60 + sentence.timestamp.minute) * 60 + sentence.timestamp.second
                msg = n2k.messages.set_n2k_gnss_data(
                    sid=sid,
                    days_since_1970=int(time.time() // (60*60*24)),
                    seconds_since_midnight=float(timestamp),
                    latitude=float(sentence.lat) / 100 * 1 if sentence.lat_dir == "N" else -1,
                    longitude=float(sentence.lon) / 100 * 1 if sentence.lon_dir == "E" else -1,
                    altitude=float(sentence.altitude),
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

    result_angle = (math.atan2(vector_r[1], vector_r[0])) % math.tau
    result_magnitude = math.sqrt(vector_r[0] ** 2 + vector_r[1] ** 2)
    # print("Boat Speed:", "{:.2f}".format(-force2), "Boat Heading:", "{:.2f}".format(math.degrees(angle2)))
    # print("Wind Speed:", "{:.2f}".format(force1), "Wind Angle:  ", "{:.2f}".format(math.degrees(angle1)))
    # print("Calc Speed:", "{:.2f}".format(result_magnitude), "Calc Angle:  ", "{:.2f}".format(math.degrees(result_angle)))
    # print("")

    return result_angle, result_magnitude

class WBuffer():
    _buffer: List
    max_length: int

    def __init__(self, max_length = 100):
        self._buffer = []
        self.max_length = max_length

    def append(self, timestamp, wd, ws):
        if len(self._buffer) >= self.max_length:
            self._buffer.pop(0) # remove first element so that we don't exceed its max size
        self._buffer.append((timestamp, wd, ws))

    def avg(self, timespan):
        if len(self._buffer) == 0:
            return 0
        last_timestamp = self._buffer[-1][0]
        values = [(v[1], v[2]) for v in self._buffer if v[0] >= last_timestamp - timespan]
        # combine forces and divide magnitude by len
        avg_wd = 0
        avg_ws = 0
        for wd, ws in values:
            avg_wd, avg_ws = combine_forces(avg_wd, avg_ws, wd, ws)
        return avg_wd, avg_ws

class Handler(n2k.MessageHandler):
    pos_send_queue: multiprocessing.Queue
    pos_recv_queue: multiprocessing.Queue
    ext_send_queue: multiprocessing.Queue
    compass_heading: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    tw_buffer: WBuffer
    aw_buffer: WBuffer

    def __init__(self,
                 node: n2k.Node,
                 pos_send_queue: multiprocessing.Queue,
                 pos_recv_queue: multiprocessing.Queue,
                 external_send_queue: multiprocessing.Queue):
        super().__init__(0, node)
        self.pos_send_queue = pos_send_queue
        self.pos_recv_queue = pos_recv_queue
        self.ext_send_queue = external_send_queue
        self.tw_buffer = WBuffer()
        self.aw_buffer = WBuffer()

    def send(self, sid, apparent_direction, apparent_speed):
        awd = apparent_direction
        aws = apparent_speed
        # TODO: smooth true wind over N seconds (with 0 being no smoothing) to avoid jumpy directions due to oscillating wind sensor
        twd, tws = combine_forces(apparent_direction, apparent_speed, self.heading, -self.speed)

        self.aw_buffer.append(int(time.time()), awd, aws)
        self.tw_buffer.append(int(time.time()), twd, tws)

        awd, aws = self.aw_buffer.avg(5)
        twd, tws = self.tw_buffer.avg(5)

        print(" ".join(map(str, [time.time(), "SENT -", "awd:", awd, "aws:", aws, "twd:", twd, "tws:", tws])), file=LOG, flush=True)

        true_wind_message = n2k.messages.set_n2k_wind_speed(
            sid=sid,
            wind_speed=tws,
            wind_angle=twd,
            wind_reference=n2k.types.N2kWindReference(0),
        )
        apparent_wind_message = n2k.messages.set_n2k_wind_speed(
            sid=sid,
            wind_speed=aws,
            wind_angle=awd,
            wind_reference=n2k.types.N2kWindReference(2),
        )

        self._node.send_msg(true_wind_message)
        self._node.send_msg(apparent_wind_message)

        self.ext_send_queue.put(MQTTMessage(
            time_ms=time.time() * 1000,
            apparent_wind_speed=aws,
            apparent_wind_direction=awd,
            true_wind_speed=tws,
            true_wind_direction=twd,
        ))

    def using_previous_data(self, msg: n2k.Message) -> None:
        wind_data = n2k.messages.parse_n2k_wind_speed(msg)
        awd = math.radians(math.degrees(wind_data.wind_angle + self.compass_heading) + COMPASS_OFFSET) % math.tau
        aws = wind_data.wind_speed
        self.send(wind_data.sid, awd, aws)

    def handle_msg(self, msg: n2k.Message) -> None:
        if not (msg.pgn == n2k.PGN.WindSpeed or msg.pgn == n2k.PGN.VesselHeading):
            return

        # TODO: basically everything, message flow is weird at the moment
        #       especially remove dependency on frequency / order of messages
        #       it shouldn't matter if more position or more wind data arrives and in which order the gps is

        if msg.pgn == n2k.PGN.WindSpeed:
            # TODO: discard NaN Values when parsing messages
            wind_data = n2k.messages.parse_n2k_wind_speed(msg)
            if wind_data.wind_reference != n2k.types.N2kWindReference(2):
                return
            if wind_data.wind_speed > 200:
                # obviously false value
                return
            data = None
            try:
                while True:
                    data = self.pos_recv_queue.get_nowait()
                    self._node.send_msg(data)
                    if data.pgn == n2k.PGN.CogSogRapid:
                        break
            except Exception:
                self.using_previous_data(msg)
                return
            if data is None or data.pgn != n2k.PGN.CogSogRapid:
                self.using_previous_data(msg)
                return
            movement_data = n2k.messages.parse_n2k_cog_sog_rapid(data)
            if None in [wind_data.wind_angle, wind_data.wind_speed, movement_data.cog, movement_data.sog]:
                return
            aws = wind_data.wind_speed
            # TODO: calculate true heading using magnetic deviation and use that
            awd = math.radians(math.degrees(wind_data.wind_angle + self.compass_heading) + COMPASS_OFFSET) % math.tau
            self.heading = movement_data.cog
            self.speed = movement_data.sog
            self.send(wind_data.sid, awd, aws)

        if msg.pgn == n2k.PGN.VesselHeading:
            heading = n2k.messages.parse_n2k_heading(msg).heading
            if heading > math.tau or heading < -math.tau:
                # bad heading (radians)
                return
            self.compass_heading = heading % math.tau
            print(" ".join(map(str, [time.time(), "COMPASS -", self.compass_heading])), file=LOG, flush=True)
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
        gps_thread = threading.Thread(target=gps_listener, args=(gps_console, position_send_queue))
        gps_thread.start()

        # Setup mqtt sending
        # TODO: set max size -> check if queue is full before put and removing first item if so
        mqtt_queue = multiprocessing.Queue()

        mqtt_process = multiprocessing.Process(target=mqtt.worker, args=(mqtt_queue,))
        mqtt_process.start()

        # Setup N2k Node
        bus = can.Bus('can0', interface='socketcan')
        notifier = can.Notifier(bus, [])

        device_information = n2k.DeviceInformation(
            unique_number=1,
            device_function=130,
            device_class=25,
            manufacturer_code=2046,
            industry_group=4,
        )

        n2k_node = n2k.Node(bus, device_information)
        n2k_node.set_product_information("Test", "0.0.1", "Dev", "00000000001", 5)
        n2k_node.set_configuration_information()

        handler = Handler(n2k_node, position_send_queue, position_recv_queue, mqtt_queue)
        n2k_node.attach_msg_handler(handler)
        notifier.add_listener(n2k_node)

        while True:
            time.sleep(0.1)
            # TODO: allow for graceful shutdown?
