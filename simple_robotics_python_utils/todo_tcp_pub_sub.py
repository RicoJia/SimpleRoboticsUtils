#!/usr/bin/env python3
import socket
import time


# TODO: not working yet!!
class SimpleTCPPublisher:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def publish(self, msg: str):
        # AF_INET is IPv4 address family; another one is AF_INET6; AF_UNIX / AF_LOCAL is local unix domain socket
        # SOCKET_STREAM means TCP; SOCKET_DGRAM is UDP
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            # will block until it hears someone
            s.listen()
            i = 0
            while True:
                conn, addr = s.accept()
                with conn:
                    conn.sendall(f"{msg}_{i}".encode("utf8"))
                # TODO Remember to remove
                print(f"Rico: {msg}_{i}")
                time.sleep(0.1)
                i += 1


class SimpleTCPSubscriber:
    """Notes
    1. TCP requires clients and servers to have port numbers. Client usually gets a random ephemeral port number
    from its operating system.
    """

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def subscribe(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            while True:
                # receive at most 1024 bytes at once
                # if publishers are down, data will be None
                data = s.recv(1024)
                time.sleep(0.1)
                print(f'Rico: {data.decode("utf8")}')


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("spawn", choices=["pub", "sub"])
    args = parser.parse_args()
    if args.spawn == "pub":
        SimpleTCPPublisher("localhost", 5000).publish("hello")
    elif args.spawn == "sub":
        SimpleTCPSubscriber("localhost", 5000).subscribe()
