#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemoryPub,
    SharedMemorySub,
)
from multiprocessing import Process, Value
import time


class TestSharedMemoryPubSub:
    def _test_shm_pub_sub(self, use_ros):
        """
        spawn sub at 50hz in a new process
        Spawn pub in a new process
        start publishing at 50 hz
        stop the sub and the pub
        """
        num_message_received = Value("i", 0)
        TEST_FREQUENCY = 40
        NUM_TEST_MSG = 1000

        def subscriber_reading_process(num_message_received, use_ros):
            def increment_num_message_received(msg):
                nonlocal num_message_received
                assert int(msg[1]) == num_message_received.value
                num_message_received.value += 1

            if use_ros:
                import rospy
                rospy.init_node("ros_test_sub")
            shm_sub = SharedMemorySub(
                topic="test",
                data_type=float,
                arr_size=2,
                read_frequency=TEST_FREQUENCY,
                # debug=True,
                callback=increment_num_message_received,
                use_ros=use_ros,
            )
            time.sleep(int(NUM_TEST_MSG / TEST_FREQUENCY) + 2.5)

        def publisher_publishing_process(use_ros):
            if use_ros:
                import rospy
                rospy.init_node("ros_test_pub")
            shm_pub = SharedMemoryPub(
                topic="test",
                data_type=float,
                arr_size=2,
                # debug=True,
                use_ros=use_ros,
            )
            time.sleep(0.5)
            for i in range(NUM_TEST_MSG):
                shm_pub.publish([i, i])
                time.sleep(1 / TEST_FREQUENCY)
        reader_proc = Process(
            target=subscriber_reading_process, args=(num_message_received, use_ros)
        )
        publisher_proc = Process(target=publisher_publishing_process, args=(use_ros,))

        reader_proc.start()
        publisher_proc.start()
        reader_proc.join()
        publisher_proc.join()

        assert num_message_received.value == NUM_TEST_MSG

    def test_shm_pub_sub(self):
        # self._test_shm_pub_sub(use_ros=False)
        self._test_shm_pub_sub(use_ros=True)