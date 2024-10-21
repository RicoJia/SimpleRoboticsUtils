#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.discoverer import (
    Discoverer,
    DiscovererTypes,
    UDP_BROADCAST_TIMEOUT,
)
from multiprocessing import Process, Value
import time


# Create 1 publisher in the main process, and 5 subscriber in subprocesses
# In UDP_BROADCAST_TIMEOUT, the subscribers should all find the publisher
# Then send a sigint to kill the subscribers
TOPIC_NAME = "test_topic"


class TestDiscoverer:
    # the current implementation will kind of choke within a second
    def test_discoverer(self):
        reader_num = 7
        readers_processes = []
        recognized_writers = Value("i", 0)

        def reader_worker(recognized_writers):
            def update_recognized_writer():
                nonlocal recognized_writers
                recognized_writers.value += 1
                # TODO Remember to remove
                # print(f'got new writer')

            d = Discoverer(
                topic=TOPIC_NAME,
                discoverer_type=DiscovererTypes.READER,
                start_connection_callback=update_recognized_writer,
                debug=False,
            )
            d.start_discovery()
            time.sleep(UDP_BROADCAST_TIMEOUT * 5)

        for i in range(reader_num):
            readers_processes.append(
                Process(target=reader_worker, args=(recognized_writers,))
            )
            readers_processes[-1].start()
        writer = Discoverer(
            topic=TOPIC_NAME,
            discoverer_type=DiscovererTypes.WRITER,
            debug=False,
        )
        writer.start_discovery()
        for i in range(8):
            # TODO Remember to remove
            print(f"{i}: recognized_writers {recognized_writers.value}")
            time.sleep(UDP_BROADCAST_TIMEOUT * 1)
        [p.join() for p in readers_processes]
        # assert recognized_writers.value == reader_num
