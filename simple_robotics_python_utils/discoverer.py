#!/usr/bin/env python3
import socket
import threading
import time 
from datetime import datetime
import atexit
import enum
import struct
import uuid
from pub_sub_utils import spin

class DiscovererTypes(enum.Enum):
    PUB = enum.auto()
    SUB = enum.auto()

DISCOVERER_UDP_PORT = 5007
DISCOVERER_MULTICAST_GROUP_ADDR = '224.0.0.1'

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
    def __init__(self, topic: str, discoverer_type: DiscovererTypes, port: int = DISCOVERER_UDP_PORT):
        self.type = discoverer_type
        self.port = port
        self.partners: set[str] = set()
        self.uuid = uuid.uuid4()
        if self.type == DiscovererTypes.PUB:
            self.th = threading.Thread(target=self.publisher_thread_func, daemon=False)
        elif self.type == DiscovererTypes.SUB:
            self.th = threading.Thread(target=self.subscriber_thread_func, daemon=False)
        else:
            raise TypeError(f"Discoverer type must be PUB or SUB. Instead we got {self.type}")
        self.topic = topic
        atexit.register(self._cleanup)
        self.th.start()

    def subscriber_thread_func(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as local_udp_sock:
            local_udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            local_udp_sock.bind((DISCOVERER_MULTICAST_GROUP_ADDR, self.port))
            # Teslling the socket to add to the multicast group
            multicast_req = struct.pack('4sl', socket.inet_aton(DISCOVERER_MULTICAST_GROUP_ADDR), socket.INADDR_ANY)
            # socket.IPPROTO_IP allows changes to IP layer, like multicast
            local_udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, multicast_req)
            # now, the socket can receive multicast packets from the group, on local machine
            
            print(f'Subscriber: started listening on UDP at port  {self.port}')
            while threading.main_thread().is_alive():
                # This is blocking
                data, addr = local_udp_sock.recvfrom(1024)
                
                print(f'Subscriber: received {data} from {addr}, going to say hi')
                topic, type, socket_path = self.unpack_hello_msg(data.decode())
                if type == DiscovererTypes.PUB.name:
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as tcp_socket_to_pubs:
                        tcp_socket_to_pubs.connect(socket_path)
                        tcp_socket_to_pubs.send(str(self.uuid).encode())


    def _create_hello_msg(self, socket_path: str):
        return f'{self.topic},{self.type.name},{socket_path}'

    def unpack_hello_msg(self, msg: str):
        topic, type, socket_path = msg.split(",")
        return topic, type, socket_path

    def publisher_thread_func(self):
        # 1. Set up Unix domain socket for TCP
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as tcp_listen_sock:
            # 0 means a random ephemeral port from the OS
            socket_path = f"/tmp/{self.topic.lstrip('/')}_{self.type.name}_{datetime.now().isoformat()}"
            tcp_listen_sock.bind(socket_path)
            # this is not blocking, simply sets up how many clients can be queued up
            tcp_listen_sock.listen()
            tcp_listen_sock.settimeout(2.0)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as local_udp_sock:
                # Limit time to live (ttl) to 1, 
                # so only max 1 hop (routers crossed) packets are allowed to make
                # Before getting discarded
                ttl = struct.pack('b', 1)
                local_udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
                message = self._create_hello_msg(socket_path)
                local_udp_sock.sendto(message.encode(), (DISCOVERER_MULTICAST_GROUP_ADDR, self.port))
            
            while threading.main_thread().is_alive():
                try:
                    conn, subscriber_tcp_addr = tcp_listen_sock.accept()
                    with conn:
                        subscriber_uuid = conn.recv(1024).decode()
                        self.partners.add(subscriber_uuid)
                        print(f'Rico: subscriber: {subscriber_uuid}, my subs: {self.partners}')
                except socket.timeout:
                    pass


    def _cleanup(self):
        if self.type == DiscovererTypes.PUB:
            pass
        elif self.type == DiscovererTypes.SUB:
            pass
        self.th.join()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("spawn_type", type=str)
    args = parser.parse_args()
    if args.spawn_type == "sub":
        sub_discoverer = Discoverer("/test_topic", DiscovererTypes.SUB)
        spin()
    elif args.spawn_type == "pub":
        pub_discoverer = Discoverer("/test_topic", DiscovererTypes.PUB)
        # sleep so the publisher thread has time to shake hands
        time.sleep(1)