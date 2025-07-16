# TODO: Replay functionality
# - TUI
#   - buttons: play / pause / speed up / slow down / step single line
#   - show current position, heading, wind, etc.
#   - show current time
#   - show current speed
#   - maybe: go back
#         (by caching the calculated results and replaying them, not by re-calculating)
#   - maybe: if gone back, clear data and start over from there
# - Required functions from the code:
#   - parse replay file
#   - populate corresponding queues with data
#   - extract calculated data to display in TUI

import argparse
import logging
import threading
import time
from pathlib import Path
from queue import Queue

import jsonpickle
import pynmea2

import gps
import wind
from log import DATA_SEPARATOR
from structs import HeadingData, PositionData, WindData

root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)
root_logger.addHandler(logging.StreamHandler())
logger = logging.getLogger(__name__)

# TODO: add TUI to display the replayed data
# +--------------------+ +----------------------------------------+
# | Replayed Logfile   | | Current Data                           |
# | (auto scrolling)   | |  - position, heading, speed, time,     |
# +--------------------+ |  - twd, tws, awd, aws, averages        |
# +--------------------+ |                                        |
# | Current debug log  | |                                        |
# +--------------------+ +----------------------------------------+
# +---------------------------------------------------------------+
# | Controls                                                      |
# | - play / pause                                                |
# | - speed up / slow                                             |
# | - step single line (disabled unless paused)                   |
# | - go back?                                                    |
# +---------------------------------------------------------------+


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay raw data from a file.")
    parser.add_argument("filename")
    parser.add_argument(
        "-s",
        "--speed",
        type=float,
        default=1.0,
        help="Speed of the replay, 1.0 is normal speed, 2.0 is double speed, etc.",
    )

    args = parser.parse_args()
    filename = str(args.filename)
    logger.debug("Replaying %s", filename)
    speed = float(args.speed)
    logger.debug("Speed set to %f", speed)

    position_queue: Queue[PositionData] = Queue()
    sentence_queue: Queue[pynmea2.NMEASentence] = Queue()
    heading_queue: Queue[HeadingData] = Queue()
    wind_queue: Queue[WindData] = Queue()

    threading.Thread(
        target=gps.worker,
        args=(sentence_queue, position_queue),
        daemon=True,
    ).start()

    threading.Thread(
        target=wind.worker,
        args=(
            position_queue,
            heading_queue,
            wind_queue,
            [],
        ),  # TODO: add consumer to extract data and display in TUI
        daemon=True,
    ).start()

    with Path(filename).open("r") as file:
        replay_start: float | None = None
        start_time = time.time()
        for line in file:
            raw_time, consumer, raw_data = line.strip().split(DATA_SEPARATOR, 2)
            time_s = float(raw_time)
            # TODO: handle pausing of the timer if we want to step though line by line
            # TODO: handle speed up and slow down
            elapsed_time = (time.time() - start_time) * speed
            if replay_start is None:
                replay_start = time_s
            if time_s > replay_start + elapsed_time:
                # wait until time_s is reached
                time.sleep((time_s - replay_start - elapsed_time) / speed)

            data = jsonpickle.decode(raw_data)  # noqa: S301 # nosec
            if consumer == "gps":
                if not isinstance(data, str):
                    msg = f"Expected GPS data to be a string, got {type(data)}"
                    raise TypeError(msg)
                sentence = pynmea2.parse(data)
                sentence_queue.put(sentence)
            elif consumer == "heading":
                if not isinstance(data, HeadingData):
                    msg = f"Expected HeadingData, got {type(data)}"
                    raise TypeError(msg)
                heading_queue.put(data)
            elif consumer == "wind":
                if not isinstance(data, WindData):
                    msg = f"Expected WindData, got {type(data)}"
                    raise TypeError(msg)
                wind_queue.put(data)
