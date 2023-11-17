#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import SharedMemoryPub, SharedMemorySub
from multiprocessing import Process, Value
import time

# Need to
class TestSharedMemoryPubSub:
    def test_shm_pub_sub(self):
        """
        spawn sub at 50hz in a new process
        Spawn pub in a new process
        start publishing at 50 hz
        stop the sub and the pub
        """
        num_message_received = Value('i', 0)
        TEST_FREQUENCY = 80
        NUM_TEST_MSG = 100
        def subscriber_reading_process(num_message_received):
            def increment_num_message_received(_):
                nonlocal num_message_received
                num_message_received.value += 1

            shm_sub = SharedMemorySub(
                topic="test",
                data_type=float,
                arr_size=2,
                read_frequency=TEST_FREQUENCY,
                # debug=True,
                callback= increment_num_message_received
            )
            time.sleep(int(NUM_TEST_MSG/TEST_FREQUENCY) + 1)

        def publisher_publishing_process():
            shm_pub = SharedMemoryPub(
                topic="test",
                data_type=float,
                arr_size=2,
                # debug=True
            )
            time.sleep(0.5)
            for i in range(NUM_TEST_MSG):
                shm_pub.publish([i, i]) 
                time.sleep(1/TEST_FREQUENCY)

        reader_proc = Process(
            target=subscriber_reading_process,
            args=(num_message_received, )
        )
        publisher_proc = Process(
            target=publisher_publishing_process,
            args=()
        )

        reader_proc.start()
        publisher_proc.start()
        reader_proc.join()
        publisher_proc.join()

        assert num_message_received.value == NUM_TEST_MSG
