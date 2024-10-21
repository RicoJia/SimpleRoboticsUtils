#!/usr/bin/env python3

"""High Level Summary
A shared memory based pub-sub framework could take half of CPU usage as the ROS topic one
That is very appealing for high frequency inter-process communication (IPC) on a small CPU, like Raspberry pi. 
However, there are some challenges:
    - For a given topic, its shared memory should be "unlinked" by the last node.

Some notes are:-
    - UDP and TCP (hence HTTP, which builds on top of it) cannot listen to the same port at the same time
    - UDP multicast: a multicast packet can be sent to multiple listening processes. TCP does not have that feature
    - ROS1 uses 11311 as the default port.

Quirks of this implementation
- PLEASE USE CAUTIOUSLY IN TIME CRITICAL CODE
    - Our test shows that there subscriber could miss messages over 40hz
- The lifttime of the discoverer, hence pubs and subs, are process time
- Instantiation of the pub and the sub must be in the same process because of the background discoverer thread
    - However, it can be shared within the same process
- This is a python implementation, there could be slowdowns!
- **Partner's heartbeats are lost in UDP multicast**
    - This leads to messages are not being published

Future work:
- For the UDP multicast issue, I thought about adding ack/nack. However, to make it robust,
we need a unicast port for each partner. 

References:
- In FastDDS, How it works (https://blog.yanjingang.com/?p=6809)
- Discovery phase: PDP (Participant Discovery Phase) and EDP (endpoint discovery phase)
    1. The Participant discovery phase
        - Each participant is a publisher/subscriber of a topic.
        - Simple Participant Discovery Phase. That's where UDP comes into play
    2. EDP: publisher and subscriber talk through a unicast port, to match their topic, and QoS. If matched, start pub/sub.
- you can send Ack/Nack messages as receipt / non-receipt of heartbeats
"""

import atexit
import ctypes
import mmap
import struct
import threading
import time
import typing

import posix_ipc
from simple_robotics_python_utils.pubsub.pub_sub_utils import spin, Rate
from simple_robotics_python_utils.common.logger import get_logger
from simple_robotics_python_utils.pubsub.discoverer import Discoverer, DiscovererTypes

import rospy
from std_msgs.msg import Float32MultiArray

###############################################
# Util Functions
###############################################


def _get_size(type: type) -> int:
    lookup = {float: ctypes.c_double, bool: ctypes.c_bool, int: ctypes.c_int}
    return ctypes.sizeof(lookup[type])


###############################################
# Shared Memory Pub SUb
###############################################
class SharedMemoryPubSubBase:
    """Notes
    1. daemon thread will be forcibly terminated when main thread joins. So for shared memory reads and writes, that's not proper
    2. So, use atexit.register(), in combination with threading.main_thread().is_alive()
    3. a python float is 24 bytes, including 8 byte C double (4 bytes on 32 bit machine); reference count;
    4. unlink is the process of removing a shared memory. It's one level below the file descriptor
        - If you unlink a process here, other processes wouldn't find it either.
    5. One solution for this distributed system, is to have:
        1. Enterance announcement: hey, I'm here!
        2. Heartbeat repeating the same announcement msg
            - While everybody takes notes of everybody's name. If your heart beat is missing, you are no longer here.
        3. Vaporation message: Hey, I'm gone
    6. To keep things simple, we do not need the above mechanism. Each topic is 8k in diskspace.
    """

    def __init__(
        self,
        topic: str,
        data_type: type,
        arr_size: int,
        discover_type: DiscovererTypes,
        start_connection_callback: typing.Callable[[], None] = None,
        no_connection_callback: typing.Callable[[], None] = None,
        debug=False,
    ):
        self.topic = topic
        self.data_type = data_type
        self.data_size = arr_size * _get_size(data_type)
        self.timestamp_size = _get_size(float)
        self.logger = get_logger(
            f"{self.topic}_{self.__class__.__name__}",
            print_level="DEBUG" if debug else "INFO",
        )
        self._init_shared_memory()
        struct_pack_lookup = {float: "d", bool: "?", int: "i"}
        self.struct_data_type_str = struct_pack_lookup[data_type] * arr_size
        atexit.register(self.__cleanup)

        self._discoverer = Discoverer(
            topic=self.topic,
            discoverer_type=discover_type,
            start_connection_callback=start_connection_callback,
            no_connection_callback=no_connection_callback,
            debug=debug,
        )
        self._discoverer.start_discovery()

    def _init_shared_memory(self):
        def init_shm(name: str, size):
            try:
                shm = posix_ipc.SharedMemory(name, flags=posix_ipc.O_CREX, size=size)
                self.logger.debug(f"{self.__class__.__name__} found existing shm for {name}")
            except posix_ipc.ExistentialError:
                shm = posix_ipc.SharedMemory(name)
            mmap_obj = mmap.mmap(shm.fd, shm.size)
            shm.close_fd()
            return shm, mmap_obj

        self.shm, self.mmap = init_shm(self.topic, self.data_size)
        self.timestamp_shm, self.timestamp_mmap = init_shm(f"{self.topic}_timestamp", self.timestamp_size)
        try:
            self.logger.debug(f"Found existing semaphore")
            self.sem = posix_ipc.Semaphore(self.topic)
        except posix_ipc.ExistentialError:
            self.logger.debug(f"creating new semaphore")
            self.sem = posix_ipc.Semaphore(self.topic, flags=posix_ipc.O_CREX, initial_value=1)
        self.logger.debug(f"Initiated shared memory")

    def _remove_shared_memory(self):
        """
        By design, this is only called when there's a shared memory error
        Not during clean up.
        """

        def unlink_shm_or_semaphore(entity):
            try:
                entity.unlink()
            except posix_ipc.ExistentialError:
                pass

        for entity in (self.shm, self.timestamp_shm, self.sem):
            unlink_shm_or_semaphore(self.shm)
        self.logger.debug(f"Removed shared memory")

    def _reset_shared_memory():
        """
        THIS FUNCTION IS VERY DANGEROUS: IT CAN ONLY BE USED WHEN YOU ARE THE ONLY SUBSCRIBER
        USE IT WITH GREAT CAUTION
        """
        self._remove_shared_memory()
        self._init_shared_memory()

    def __cleanup(self):
        self.logger.debug(f"{self.__class__.__name__} instance for topic {self.topic} is terminated")
        for mmap_obj in (self.mmap, self.timestamp_mmap):
            mmap_obj.close()


class SharedMemoryPub(SharedMemoryPubSubBase):
    def __init__(
        self,
        topic: str,
        data_type: type,
        arr_size: int,
        start_connection_callback: typing.Callable[[], None] = None,
        no_connection_callback: typing.Callable[[], None] = None,
        debug=False,
        # set to true because its speed performance is better
        use_ros=True,
    ):
        self.use_ros = use_ros
        if self.use_ros:
            self._ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        else:
            super().__init__(
                topic,
                data_type,
                arr_size,
                DiscovererTypes.WRITER,
                start_connection_callback,
                no_connection_callback,
                debug,
            )

    def publish(self, msg_arr: typing.List[typing.Any]):
        """Publish a message by writing to shared memory.

        If no subscriber is connected, this function terminates immediately

        Args:
            msg_arr (typing.List[typing.Any]): messsage of specified type in the constructor

        Raises:
            RuntimeError: if the message size does not match
        """
        if self.use_ros:
            msg = Float32MultiArray()
            msg.data = msg_arr
            self._ros_pub.publish(msg)
        else:
            # Timeout set to 0, so we just check if there's at least one subscriber
            # without waiting
            if self._discoverer.wait_to_proceed(timeout=0):
                packed_data = struct.pack(self.struct_data_type_str, *msg_arr)
                packed_timestamp = struct.pack("d", time.time())
                if len(packed_data) != self.data_size:
                    raise RuntimeError(
                        f"Published message array mismatching. Expected {self.data_size/_get_size(self.data_type)}, got {len(msg_arr)}"
                    )
                with self.sem:
                    try:
                        self.mmap[: len(packed_data)] = packed_data
                        self.timestamp_mmap[: len(packed_timestamp)] = packed_timestamp
                        self.logger.debug(f"publishing: {msg_arr}")
                    # This happens if the previous use of shared memory has a different size
                    except IndexError:
                        self._reset_shared_memory()
                        self.logger.warning("Previous use of the shared memory is incompatible. It's now cleaned")


class SharedMemorySub(SharedMemoryPubSubBase):
    def __init__(
        self,
        topic: str,
        data_type: type,
        arr_size: int,
        read_frequency: int,
        callback: typing.Callable[[tuple], None],
        start_connection_callback: typing.Callable[[], None] = None,
        no_connection_callback: typing.Callable[[], None] = None,
        debug=False,
        # set to true because its speed performance is better
        use_ros=True,
    ):
        self.use_ros = use_ros
        if self.use_ros:
            self._has_connection = False
            _start_connection_callback = start_connection_callback

            # get ros subscriber
            def ros_callback(msg):
                if not self._has_connection and _start_connection_callback:
                    _start_connection_callback()
                self._has_connection = True
                callback(msg.data)

            self._ros_sub = rospy.Subscriber(topic, Float32MultiArray, ros_callback)
        else:
            super().__init__(
                topic,
                data_type,
                arr_size,
                DiscovererTypes.READER,
                start_connection_callback,
                no_connection_callback,
                debug,
            )
            self.callback = callback
            self.rate = Rate(read_frequency)
            self._th = threading.Thread(target=self.__run, daemon=False)
            self._th.start()

    def __run(self):
        # make it properly take signals
        last_msg_timestamp: float = 0
        while threading.main_thread().is_alive():
            if self._discoverer.wait_to_proceed(timeout=3):
                connection_start_timestamp = self._discoverer.get_current_connection_start_time_time()
                with self.sem:
                    current_msg_timestamp = float(struct.unpack("d", self.timestamp_mmap[: self.timestamp_size])[0])
                    if current_msg_timestamp and current_msg_timestamp != last_msg_timestamp:
                        if current_msg_timestamp > connection_start_timestamp:
                            try:
                                unpacked_msg = struct.unpack(
                                    self.struct_data_type_str,
                                    self.mmap[: self.data_size],
                                )
                                last_msg_timestamp = current_msg_timestamp
                                self.callback(unpacked_msg)
                                self.logger.debug(f"unpacked_msg: {unpacked_msg}")
                            except struct.error as e:
                                self._reset_shared_memory()
                                print("WARNING: Previous use of the shared memory is incompatible. It's now cleaned")
                self.rate.sleep()

    def __cleanup(self):
        super().__cleanup()
        self._th.join()


if __name__ == "__main__":
    import argparse
    import rospy
    from std_msgs.msg import Float64MultiArray

    parser = argparse.ArgumentParser()
    parser.add_argument("spawn_type", type=str, choices=["pub", "sub", "ros_pub", "ros_sub"])
    args = parser.parse_args()
    msg_ls = list(range(360))
    if args.spawn_type == "pub":
        pub = SharedMemoryPub(topic="test_topic", data_type=int, arr_size=len(msg_ls), debug=False)
        for i in range(2000):
            pub.publish(msg_ls)
            time.sleep(0.02)
    elif args.spawn_type == "sub":
        sub = SharedMemorySub(
            topic="test_topic",
            data_type=int,
            arr_size=len(msg_ls),
            read_frequency=50,
            debug=False,
        )
        spin()
    elif args.spawn_type == "ros_sub":
        rospy.init_node("test_sub", anonymous=True)
        sub = rospy.Subscriber("test_topic", Float64MultiArray, lambda msg: msg.data)
        rospy.spin()
    elif args.spawn_type == "ros_pub":
        rospy.init_node("test_pub")
        pub = rospy.Publisher("test_topic", Float64MultiArray, queue_size=0)
        rate = rospy.Rate(50)
        for i in range(1000):
            msg = Float64MultiArray()
            msg.data = msg_ls
            pub.publish(msg)
            rate.sleep()
