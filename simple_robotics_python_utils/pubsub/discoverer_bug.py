#!/usr/bin/env python3
import threading
import time
import socket
import struct

UDP_BROADCAST_TIMEOUT = 1
DISCOVERER_MULTICAST_GROUP_ADDR = "224.0.0.1"

class FakeDiscoverer:
    def __init__(self) -> None:
        self.port = 5007
        self.socket_path = f"/tmp/test_topic"
        self.th = threading.Thread(target=self._udp_thread_func)
        self.th.start()
    
    def _udp_thread_func(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            self._configure_socket_for_listening(udp_socket)
            while threading.current_thread().is_alive():
                try:
                    print(f'before received')
                    data, addr = udp_socket.recvfrom(10)
                    print(f'post received, data: {data}')
                except socket.timeout:
                    self._create_and_send_msg("header", self.socket_path, udp_socket)

    def _create_and_send_msg(self, header: str, socket_path: str, sock):
        message = f"{header}"
        sock.sendto(message.encode(), (DISCOVERER_MULTICAST_GROUP_ADDR, self.port))

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
        udp_socket.setsockopt(
            socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, multicast_req
        )
        # Below code allows us to send to the UDP port
        # Limit time to live (ttl) to 1,
        # so only max 1 hop (routers crossed) packets are allowed to make
        # Before getting discarded
        ttl = struct.pack("b", 1)
        udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        
    def main_thread(self):
        while True:
            print("main")
            time.sleep(0.1)

if __name__ == "__main__":
    f = FakeDiscoverer()
    f.main_thread()