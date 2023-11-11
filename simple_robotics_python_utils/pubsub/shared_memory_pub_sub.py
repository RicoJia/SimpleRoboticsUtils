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
"""
import atexit
import ctypes
import mmap
import struct
import threading
import time
import typing

import posix_ipc

from pub_sub_utils import Rate, spin

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
    So, the lack of heart beat meachanism means: we DO NOT DELETE SHARED MEMORY, once created, ASSUMING that
    wouldn't cause overflows
    """

    def __init__(self, topic: str, data_type: type, arr_size: int, verbose=False):
        self.topic = topic
        self.data_type = data_type
        self.data_size = arr_size * _get_size(data_type)
        self._verbose = verbose
        self._init_shared_memory()
        struct_pack_lookup = {float: "d", bool: "?", int: "i"}
        self.struct_data_type_str = struct_pack_lookup[data_type] * arr_size
        atexit.register(self.__cleanup)

    def _init_shared_memory(self):
        try:
            self.shm = posix_ipc.SharedMemory(self.topic)
            # TODO Remember to remove
            print(f"Found existing shared memory")
        except posix_ipc.ExistentialError:
            self.shm = posix_ipc.SharedMemory(
                self.topic, flags=posix_ipc.O_CREX, size=self.data_size
            )
            print(f"creating new shared memory")
        try:
            print(f"Found existing semaphore")
            self.sem = posix_ipc.Semaphore(self.topic)
        except posix_ipc.ExistentialError:
            print(f"creating new semaphore")
            self.sem = posix_ipc.Semaphore(
                self.topic, flags=posix_ipc.O_CREX, initial_value=1
            )
        self.mmap = mmap.mmap(self.shm.fd, self.shm.size)
        self.shm.close_fd()

    def _remove_shared_memory(self):
        """
        By design, this is only called when there's a shared memory error
        Not during clean up.
        """
        try:
            self.shm.unlink()
        except posix_ipc.ExistentialError:
            pass
        try:
            self.sem.unlink()
            print(f"Sem unlinked successfully")
        except posix_ipc.ExistentialError:
            pass

    def __cleanup(self):
        if self._verbose:
            print(
                f"{self.__class__.__name__} instance for topic {self.topic} is terminated"
            )
        self.mmap.close()


class SharedMemoryPub(SharedMemoryPubSubBase):
    def __init__(self, topic: str, data_type: type, arr_size: int, verbose=False):
        super().__init__(topic, data_type, arr_size, verbose)

    def publish(self, msg_arr: typing.List[typing.Any]):
        packed_data = struct.pack(self.struct_data_type_str, *msg_arr)
        if len(packed_data) != self.data_size:
            raise RuntimeError(
                f"Published message array mismatching. Expected {self.data_size/_get_size(self.data_type)}, got {len(msg_arr)}"
            )
        self.sem.acquire()
        try:
            self.mmap[: len(packed_data)] = packed_data
            if self._verbose:
                # TODO Remember to remove
                print(f"publishing: {msg_arr}")
        except IndexError:
            self._remove_shared_memory()
            self._init_shared_memory()
            print(
                "WARNING: Previous use of the shared memory is incompatible. It's now cleaned"
            )
        self.sem.release()


class SharedMemorySub(SharedMemoryPubSubBase):
    def __init__(
        self,
        topic: str,
        data_type: type,
        arr_size: int,
        read_frequency: int,
        verbose=False,
    ):
        super().__init__(topic, data_type, arr_size, verbose)
        self.rate = Rate(read_frequency)
        self._th = threading.Thread(target=self.__run, daemon=False)
        self._th.start()

    # How does python's __ mangling work? Can outsiders call this?
    def __run(self):
        # make it properly take signals
        while threading.main_thread().is_alive():
            self.sem.acquire()
            try:
                unpacked_msg = struct.unpack(
                    self.struct_data_type_str, self.mmap[: self.data_size]
                )
                if self._verbose:
                    print(f"unpacked_msg: {unpacked_msg}")
            except struct.error as e:
                self._remove_shared_memory()
                self._init_shared_memory()
                print(
                    "WARNING: Previous use of the shared memory is incompatible. It's now cleaned"
                )
            self.sem.release()
            self.rate.sleep()

    def __cleanup(self):
        super().__cleanup()
        self._th.join()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("spawn_type", type=str, choices=["pub", "sub"])
    args = parser.parse_args()
    msg_ls = [1, 2, 3, 4, 5]
    if args.spawn_type == "pub":
        pub = SharedMemoryPub(
            topic="test_topic", data_type=int, arr_size=len(msg_ls), verbose=True
        )
        for i in range(5):
            pub.publish([r + i for r in msg_ls])
            time.sleep(0.5)
    elif args.spawn_type == "sub":
        sub = SharedMemorySub(
            topic="test_topic",
            data_type=int,
            arr_size=len(msg_ls),
            read_frequency=10,
            verbose=True,
        )
        spin()
