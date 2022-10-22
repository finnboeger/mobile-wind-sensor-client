from multiprocessing import Queue

import n2k


def worker(recv: Queue, send: Queue) -> None:
    # TODO: setup imu
    # TODO: await first gps fix

    while True:
        # TODO: get complete fifo from imu
        # TODO: calculate attitude, heading, and filter positional data (position, speed, direction)
        #       send calculated data n times per second
        msg: n2k.Message = recv.get()
        if msg.pgn != n2k.PGN.VesselHeading:
            send.put(msg)
