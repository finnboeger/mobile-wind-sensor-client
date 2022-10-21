from multiprocessing import Queue


def worker(recv: Queue, send: Queue) -> None:
    # TODO: setup imu
    # TODO: await first gps fix

    while True:
        # TODO: get complete fifo from imu
        # TODO: calculate attitude, heading, and filter positional data (position, speed, direction)
        #       send calculated data n times per second
        send.put(recv.get())
