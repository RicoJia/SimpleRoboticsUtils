#!/usr/bin/env python3
from simple_robotics_python_utils.pubsub.shared_memory_pub_sub import (
    SharedMemoryPub,
    SharedMemorySub,
)
from multiprocessing import Process, Manager
import time

ARR_SIZE = 20


class TestSharedMemoryPubSub:
    def _test_shm_pub_sub(self, use_ros):
        """
        spawn sub at 50hz in a new process
        Spawn pub in a new process
        start publishing at 50 hz
        stop the sub and the pub
        """
        TEST_FREQUENCY = 200
        NUM_TEST_MSG = 10
        with Manager() as manager:
            messages_received_ls = manager.list()

            def subscriber_reading_process(messages_received_ls, use_ros):
                def increment_num_message_received(msg):
                    nonlocal messages_received_ls
                    messages_received_ls.append(msg)

                if use_ros:
                    import rospy

                    rospy.init_node("ros_test_sub")
                shm_sub = SharedMemorySub(
                    topic="test",
                    data_type=float,
                    arr_size=ARR_SIZE,
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
                    arr_size=ARR_SIZE,
                    # debug=True,
                    use_ros=use_ros,
                )
                time.sleep(0.5)
                for i in range(NUM_TEST_MSG):
                    shm_pub.publish([i for _ in range(ARR_SIZE)])
                    time.sleep(1 / TEST_FREQUENCY)

            reader_proc = Process(target=subscriber_reading_process, args=(messages_received_ls, use_ros))
            publisher_proc = Process(target=publisher_publishing_process, args=(use_ros,))

            reader_proc.start()
            publisher_proc.start()
            publisher_proc.join()
            reader_proc.join()

            for received_arr in messages_received_ls:
                assert all([int(i) == int(received_arr[0]) for i in received_arr])
                # assert int(first) == int(i)

    def test_shm_pub_sub(self):
        self._test_shm_pub_sub(use_ros=False)
        # Run this with rosmaster
        # self._test_shm_pub_sub(use_ros=True)
