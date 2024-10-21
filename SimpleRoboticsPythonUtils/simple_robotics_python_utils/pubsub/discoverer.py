#!/usr/bin/env python3
import atexit
import enum
import socket
import struct
import threading
import time
from collections import namedtuple
from datetime import datetime
from typing import Dict, Optional

from simple_robotics_python_utils.common.logger import get_logger
from simple_robotics_python_utils.pubsub.pub_sub_utils import spin

import typing


class DiscovererTypes(enum.Enum):
    WRITER = enum.auto()
    READER = enum.auto()


DISCOVERER_UDP_PORT = 5007
DISCOVERER_MULTICAST_GROUP_ADDR = "224.0.0.1"
UDP_BROADCAST_TIMEOUT = 1
CONNECTION_EXPIRATION_TIMEOUT = UDP_BROADCAST_TIMEOUT * 1000
HELLO = "h"
BYE = "b"


class Discoverer:
    """
    Workflow:
        Set up TCP port
        announce who you are over UDP
        For pub, start new thread saying hi on UDP. Then through TCP, listen for subs. If no subs, no publishing
        For sub, start new thread listening for new pubs, through UDP. Then through TCP, talk to pub; if no pub, no reading
        During cleanup:
        pub will say bye on TCP: "bye, <TOPIC>"
        sub will say bye to its pubs through TCP.
    """

    def __init__(
        self,
        topic: str,
        discoverer_type: DiscovererTypes,
        port: int = DISCOVERER_UDP_PORT,
        start_connection_callback: typing.Callable[[], None] = None,
        no_connection_callback: typing.Callable[[], None] = None,
        debug: bool = False,
    ):
        if not isinstance(discoverer_type, DiscovererTypes):
            raise TypeError(f"Discoverer type must be WRITER or READER. Instead we got {discoverer_type}")
        self.type = discoverer_type
        self.port = port
        self.no_connection_callback = no_connection_callback
        self.start_connection_callback = start_connection_callback
        self.partners: Dict[str, float] = {}
        self.logger = get_logger(name=self.__class__.__name__, print_level="DEBUG" if debug else "INFO")
        self._at_least_one_partner_event = threading.Event()
        self.topic = topic
        self.socket_path = f"/tmp/{self.topic.lstrip('/')}_{self.type.name}_{datetime.now().isoformat()}"
        self.th = threading.Thread(target=self._main, daemon=False)
        self._current_connection_start_time: Optional[float] = None
        atexit.register(self._cleanup)

    def start_discovery(self):
        self.th.start()

    def wait_to_proceed(self, timeout=None) -> bool:
        """wait until there is at least one partner, or a timeout is hit

        Args:
            timeout (int, optional): timeout to proceed. Defaults to 0.

        Returns:
            bool: if there is at leat one partner
        """
        self._at_least_one_partner_event.wait(timeout)
        return self._at_least_one_partner_event.isSet()

    def get_current_connection_start_time_time(self):
        return self._current_connection_start_time

    def _main(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            self._configure_socket_for_listening(udp_socket)
            self._create_and_send_msg(HELLO, self.socket_path, udp_socket)
            self._spin_and_manage_connection_with_other_type(udp_socket)

    def _configure_socket_for_listening(self, udp_socket):
        # Below code allows us to write to the UDP port
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # disable self messages looping back
        # udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        udp_socket.bind(("", self.port))
        udp_socket.settimeout(UDP_BROADCAST_TIMEOUT)
        # Teslling the socket to add to the multicast group
        multicast_req = struct.pack(
            "4sl",
            socket.inet_aton(DISCOVERER_MULTICAST_GROUP_ADDR),
            socket.INADDR_ANY,
        )
        # socket.IPPROTO_IP allows changes to IP layer, like multicast
        # now, the socket can receive multicast packets from the group, on local machine
        udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, multicast_req)
        # Below code allows us to send to the UDP port
        # Limit time to live (ttl) to 1,
        # so only max 1 hop (routers crossed) packets are allowed to make
        # Before getting discarded
        ttl = struct.pack("b", 1)
        udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    def _spin_and_manage_connection_with_other_type(self, udp_socket):
        other_type = {
            DiscovererTypes.READER: DiscovererTypes.WRITER,
            DiscovererTypes.WRITER: DiscovererTypes.READER,
        }[self.type]
        # Note: main thread is the thread where the python interpreter is launched,
        # not necessarily where the object gets created
        while threading.main_thread().is_alive():
            try:
                data, addr = udp_socket.recvfrom(1024)
                # Note: we can hear messages we multicast ourselves.
                # However, loop back doesn't apply to multicast because it will fail on others' multicast messages
                header, topic, type, socket_path = self._unpack_hello_msg(data.decode())
                if type == self.type.name:
                    continue
                self._prune_potential_gone_partners()
                if header == HELLO and type == other_type.name and topic == self.topic:
                    # update will automatically add / update
                    self._state_update(start=True, socket_path=socket_path, udp_socket=udp_socket)
                if header == BYE and type == other_type.name and topic == self.topic:
                    # pop will delete the socket path in partners. If key doesn't exist, it won't yell
                    self._state_update(start=False, socket_path=socket_path)
            except socket.timeout:
                self._create_and_send_msg(HELLO, self.socket_path, udp_socket)
        self._create_and_send_msg(BYE, self.socket_path, udp_socket)

    def _state_update(self, start: bool, socket_path: str, udp_socket=None):
        if start:
            if not self.partners:
                # This can make UDP socket sleep a bit longer
                self._create_and_send_msg(HELLO, self.socket_path, udp_socket)
                self._current_connection_start_time = time.time()
                if self.start_connection_callback:
                    self.start_connection_callback()
                self._at_least_one_partner_event.set()
            self.partners.update({socket_path: time.time() + CONNECTION_EXPIRATION_TIMEOUT})
            self.logger.debug(f"{self.type} Added a partner, now have parters: {len(self.partners)}")
        else:
            self.partners.pop(socket_path, None)
            if not self.partners:
                self._current_connection_start_time = None
                if self.no_connection_callback:
                    self.no_connection_callback()
                self._at_least_one_partner_event.clear()
            self.logger.debug(f"{self.type} Removed a partner, now have parters: {len(self.partners)}")

    def _prune_potential_gone_partners(self):
        prune_list = [socket_path for socket_path, expiration_time in self.partners.items() if expiration_time < time.time()]
        for socket_path in prune_list:
            self.partners.pop(socket_path, None)
        if not self.partners:
            self._at_least_one_partner_event.clear()
        self.logger.debug(f"{self.type} - Current partners: {len(self.partners)}")

    def _create_and_send_msg(self, header: str, socket_path: str, sock):
        message = f"{header},{self.topic},{self.type.name},{socket_path}"
        sock.sendto(message.encode(), (DISCOVERER_MULTICAST_GROUP_ADDR, self.port))

    def _unpack_hello_msg(self, msg: str):
        header, topic, type, socket_path = msg.split(",")
        return header, topic, type, socket_path

    def _cleanup(self):
        if self.th.is_alive():
            self.th.join()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("spawn_type", type=str)
    args = parser.parse_args()
    if args.spawn_type == "sub":
        sub_discoverer = Discoverer("/test_topic", DiscovererTypes.READER, debug=True)
        sub_discoverer.start_discovery()
        while True:
            should_proceed = sub_discoverer.wait_to_proceed(1)
            if should_proceed:
                print(f"Proceed")
            else:
                print(f"Not proceed")
    elif args.spawn_type == "pub":
        pub_discoverer = Discoverer("/test_topic", DiscovererTypes.WRITER, debug=True)
        pub_discoverer.start_discovery()
        # sleep so the publisher thread has time to announce
        time.sleep(6)
