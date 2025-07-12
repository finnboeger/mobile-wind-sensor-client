import datetime
import threading
from decimal import Decimal
from queue import Queue

import pynmea2
import serial

from structs import FAAMode, FixType, PositionData, Quality, Satellite

SERIAL_PORT = "/dev/ttyACM0"


def read_next_sentence(console: serial.Serial) -> pynmea2.NMEASentence:
    while True:
        data = console.readline().decode()
        if len(data) == 0:
            continue
        return pynmea2.parse(data)


def wait_for_gps_fix(console: serial.Serial) -> None:
    while True:
        sentence = read_next_sentence(console)
        if isinstance(sentence, pynmea2.GLL) and sentence.status == "A":
            return


def parse_lat_lon(val: str) -> float:
    """
    Parse latitude or longitude from a NMEA sentence.

    The value is expected to be in the format "(d)ddmm.mmmm...", where:
    - "(d)dd" is the degrees part (2-3 digits)
    - "mm.mmmm..." is the minutes part (2 digits and decimal)
    The function converts this to a float representing the value in decimal degrees.

    :param val: Latitude or longitude value in NMEA format.
    :return: Parsed value in degrees.
    """
    degrees = int(val.split(".")[0][:-2])
    minutes = float(val.split(".")[0][-2:] + "." + val.split(".")[1])
    return degrees + (minutes / 60)


def rmc_to_position_data(
    sentence: pynmea2.RMC,
) -> PositionData:
    """
    Initialize a PositionData object from an RMC sentence.

    :param sentence: RMC sentence to extract data from.
    :return: PositionData object with the extracted data.
    """
    # assert the sentence field instances to satisfy typechecking
    if not (
        isinstance(sentence.datestamp, datetime.date)
        and isinstance(sentence.timestamp, datetime.time)
        and sentence.status in ("A", "V")
        and sentence.lat is not None
        and sentence.lat_dir in ("N", "S", "")
        and sentence.lon is not None
        and sentence.lon_dir in ("E", "W", "")
        and (
            isinstance(sentence.spd_over_grnd, float) or sentence.spd_over_grnd is None
        )
        and (isinstance(sentence.true_course, float) or sentence.true_course is None)
        and sentence.mag_variation is not None
        and sentence.mag_var_dir in ("E", "W", "")
    ):
        error_message = "Error while parsing RMC sentence."
        raise TypeError(error_message)

    # Combine date and time into a single datetime object
    dt = datetime.datetime(
        year=sentence.datestamp.year,
        month=sentence.datestamp.month,
        day=sentence.datestamp.day,
        hour=sentence.timestamp.hour,
        minute=sentence.timestamp.minute,
        second=sentence.timestamp.second,
        tzinfo=sentence.timestamp.tzinfo,
    )
    timestamp = int(dt.timestamp())
    latitude = (
        (parse_lat_lon(sentence.lat) * 1 if sentence.lat_dir == "N" else -1)
        if sentence.lat != "" and sentence.lat_dir != ""
        else None
    )
    longitude = (
        (parse_lat_lon(sentence.lon) * 1 if sentence.lon_dir == "E" else -1)
        if sentence.lon != "" and sentence.lon_dir != ""
        else None
    )
    magnetic_variation = (
        (float(sentence.mag_variation) * 1 if sentence.mag_var_dir == "E" else -1)
        if sentence.mag_variation != "" and sentence.mag_var_dir != ""
        else None
    )
    return PositionData(
        timestamp=timestamp,
        status=sentence.status,
        latitude=latitude,
        longitude=longitude,
        speed=sentence.spd_over_grnd,
        true_course=sentence.true_course,
        magnetic_variation=magnetic_variation,
        faa_mode=FAAMode(sentence.mode_indicator),
        satellites_in_view=set(),
    )


def add_vtg_to_position_data(
    data: PositionData,
    sentence: pynmea2.VTG,
) -> None:
    if not (
        isinstance(sentence.mag_track, (float, Decimal)) or sentence.mag_track is None
    ):
        error_message = "Error while parsing VTG sentence."
        raise TypeError(error_message)
    data.magnetic_course = (
        float(sentence.mag_track) if sentence.mag_track is not None else None
    )


def add_gga_to_position_data(
    data: PositionData,
    sentence: pynmea2.GGA,
) -> None:
    """
    Add GGA sentence data to the PositionData object.

    :param data: PositionData object to update.
    :param sentence: GGA sentence to extract data from.
    """
    if not (
        isinstance(sentence.gps_qual, int)
        and sentence.num_sats is not None
        and (isinstance(sentence.altitude, float) or sentence.altitude is None)
        and sentence.horizontal_dil is not None
        and sentence.geo_sep is not None
        and sentence.age_gps_data is not None
        and sentence.ref_station_id is not None
    ):
        error_message = "Error while parsing GGA sentence."
        raise TypeError(error_message)
    data.quality = Quality(sentence.gps_qual)
    data.number_satellites_used = int(sentence.num_sats)
    data.horizontal_dilution_of_precision = float(sentence.horizontal_dil)
    data.altitude = sentence.altitude
    data.geoidal_separation = (
        float(sentence.geo_sep) if sentence.geo_sep != "" else None
    )
    if sentence.age_gps_data != "":
        data.differential_gps_data_age = int(sentence.age_gps_data)
    if sentence.ref_station_id != "":
        data.differential_reference_station_id = int(sentence.ref_station_id)


def add_gsa_to_position_data(
    data: PositionData,
    sentence: pynmea2.GSA,
) -> None:
    """
    Add GSA sentence data to the PositionData object.

    :param data: PositionData object to update.
    :param sentence: GSA sentence to extract data from.
    """
    if not (
        sentence.mode_fix_type is not None
        and sentence.pdop is not None
        and sentence.vdop is not None
    ):
        error_message = "Error while parsing GSA sentence."
        raise TypeError(error_message)

    data.fix_type = FixType(int(sentence.mode_fix_type))
    data.positional_dilution_of_precision = float(sentence.pdop)
    data.vertical_dilution_of_precision = float(sentence.vdop)

    used_satellites: list[int] = []
    fields = [
        sentence.sv_id01,
        sentence.sv_id02,
        sentence.sv_id03,
        sentence.sv_id04,
        sentence.sv_id05,
        sentence.sv_id06,
        sentence.sv_id07,
        sentence.sv_id08,
        sentence.sv_id09,
        sentence.sv_id10,
        sentence.sv_id11,
        sentence.sv_id12,
    ]
    for field in fields:
        if not isinstance(field, str):
            error_message = (
                "Error while parsing GSA sentence. expected field to contain string."
            )
            raise TypeError(error_message)
        if field == "":
            continue
        used_satellites.append(int(field))
    data.used_satellites = used_satellites


def add_gsv_to_satellites_in_view(
    satellites: set[Satellite],
    sentence: pynmea2.GSV,
) -> None:
    """
    Add the satellites from a GSV sentence to the satellites in view set.

    :param satellites: set of satellites to update.
    :param sentence: GSV sentence to extract data from.
    """

    def add_satellite(
        prn_num: str | None,
        elevation_deg: str | None,
        azimuth: str | None,
        snr: str | None,
    ) -> None:
        if prn_num == "" or prn_num is None or elevation_deg is None or azimuth is None:
            return
        satellites.add(
            Satellite(
                id=int(prn_num),
                elevation=int(elevation_deg),
                azimuth=int(azimuth),
                signal_to_noise_ratio=(
                    int(snr) if snr is not None and snr != "" else None
                ),
            ),
        )

    add_satellite(
        sentence.sv_prn_num_1,
        sentence.elevation_deg_1,
        sentence.azimuth_1,
        sentence.snr_1,
    )
    add_satellite(
        sentence.sv_prn_num_2,
        sentence.elevation_deg_2,
        sentence.azimuth_2,
        sentence.snr_2,
    )
    add_satellite(
        sentence.sv_prn_num_3,
        sentence.elevation_deg_3,
        sentence.azimuth_3,
        sentence.snr_3,
    )
    add_satellite(
        sentence.sv_prn_num_4,
        sentence.elevation_deg_4,
        sentence.azimuth_4,
        sentence.snr_4,
    )


def process_sentences(sentences: list[pynmea2.NMEASentence]) -> PositionData:
    if len(sentences) == 0:
        error_message = "No sentences provided."
        raise ValueError(error_message)
    if not isinstance(sentences[0], pynmea2.RMC):
        error_message = "First sentence must be RMC."
        raise TypeError(error_message)

    data = rmc_to_position_data(sentences[0])

    for sentence in sentences[1:]:
        if isinstance(sentence, pynmea2.RMC):
            # RMC: Recommended Minimum Navigation Information
            error_message = "RMC sentence found in the middle of a group of sentences, "
            raise TypeError(error_message)

        if isinstance(sentence, pynmea2.VTG):
            # VTG: Track made good and Ground speed

            add_vtg_to_position_data(data, sentence)
            continue

        if isinstance(sentence, pynmea2.GGA):
            # GGA: Global Positioning System Fix Data, Time, Position and
            # fix related data for a GPS receiver.

            add_gga_to_position_data(data, sentence)
            continue

        if isinstance(sentence, pynmea2.GSA):
            # GSA: GPS DOP and active satellites
            add_gsa_to_position_data(data, sentence)
            continue

        if isinstance(sentence, pynmea2.GSV):
            # GSV: Satellites in view  # noqa: ERA001
            add_gsv_to_satellites_in_view(data.satellites_in_view, sentence)
            continue

        if isinstance(sentence, pynmea2.GLL):
            # GLL: Geographic Position - Latitude/Longitude
            # This sentence only contains information we already possess
            continue

        error_message = f"Unsupported NMEA sentence type: {type(sentence)}"
        raise TypeError(error_message)

    return data



def worker(position_queue: Queue[PositionData]) -> None:
    """
    Read the GPS position data from the serial console and forward it to the queue.

    :param position_queue: Queue to output GPS position data to.
    """
    with serial.Serial(SERIAL_PORT, baudrate=115200, timeout=0) as console:
        # Wait for first GPS fix
        wait_for_gps_fix(console)

        sentences: list[pynmea2.NMEASentence] = []

        while True:
            sentence = read_next_sentence(console)
            if isinstance(sentence, pynmea2.RMC):
                # RMC: Recommended Minimum Navigation Information
                if len(sentences) > 0:
                    try:
                        position = process_sentences(sentences)

                        # only submit valid position data
                        if position.status == "A":
                            position_queue.put(position)
                    except Exception as e:
                        print(e)

                # reset data for the next group of sentences
                sentences = [sentence]
                continue

            if len(sentences) == 0:
                # Wait for first RMC sentence to initialize the data buffer, as it
                # indicates the start if each group of sentences, and is only sent once
                continue

            sentences.append(sentence)


def init() -> Queue[PositionData]:
    """
    Initialize the GPS module.

    :return: A queue that will contain GPS position data.
    """
    position_queue: Queue[PositionData] = Queue()

    threading.Thread(
        target=worker,
        args=(position_queue,),
        daemon=True,
    ).start()

    return position_queue
