import threading

import gps
import nmea
import wind

if __name__ == "__main__":
    position_queue = gps.init()
    n2k_node, heading_queue, wind_queue = nmea.init()

    # TODO: forward position to n2k network
    # TODO: init mqtt
    # TODO: potentially init local server

    # Start the worker thread to compute the true wind and send it the the consumers
    worker_thread = threading.Thread(
        target=wind.worker,
        args=(position_queue, heading_queue, wind_queue, []),
    )
    worker_thread.start()
    worker_thread.join()
