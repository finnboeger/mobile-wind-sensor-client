import math
import threading
import time
from queue import Queue

import n2k

import nmea
from structs import HeadingData


class _Node:
    def __init__(self) -> None:
        self.messages: list[n2k.Message] = []

    def send_msg(self, msg: n2k.Message) -> None:
        self.messages.append(msg)


def test_heading_is_sent_for_the_bow_at_most_ten_times_a_second() -> None:
    node = _Node()
    queue: Queue[HeadingData] = Queue()
    for index in range(20):
        queue.put(HeadingData(heading=math.radians(100), timestamp=index * 50))
    threading.Thread(
        target=nmea.forward_heading,
        args=(node, queue),
        kwargs={"bow_angle": math.radians(-30), "true_heading": True},
        daemon=True,
    ).start()
    deadline = time.monotonic() + 2
    while not queue.empty() and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)

    assert len(node.messages) == 10
    heading = n2k.messages.parse_n2k_heading(node.messages[0])
    assert heading.heading is not None
    assert abs(math.degrees(heading.heading) - 70) < 0.01
    assert heading.ref == n2k.types.N2kHeadingReference.true
