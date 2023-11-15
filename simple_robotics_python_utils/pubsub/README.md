# Pub Sub Frameworks


## Introduction
1. Main files

## Design Notes
1. pure UDP framework since it's on local machine?
    - Then, when a new person comes in, everybody has to holler.
    - yeah. UDP have no flow control. So if receiving app is not fast enough, the receiver's socket buffer can overflow.
    - Sending faster than NIC?
    - OS could discard packet, if CPU or memory runs low.

2. A TCP port can only be used as a listner, or sender. Cannot be used for dual uses.

3. So for a discovery protocol, we want to minimize the use of UDP for it unreliability.
    - UDP port for broadcasting when one person comes in.
        - in multicast comm, packets need to be sent to a **multicast group address**, then network infrastructure (routers, network switches) duplicates the packetm delivers to all receivers. Do you must 
            1. Listen on the same port using multicast, same multicast address
                - `localhost (127.0.0.1)` is an IPv$ unicast address. We must use (`224.0.0.0` to `239.255.255.255`)
            2. join multicast group
        - UDP Multicast Loopback
            - loopback is every time after a node sends a message, it hears that message immediately.

    - When there's no more publisher, subscriber should stop checking the shared memory; When there's no more subscriber, publisher should stop writing to the shared memory.
        - for fast and reliable performance, after a pub/sub makes its announcement, it needs to open a TCP connection to listen for existing sub / pub
        - Then, when a new sub / pub joins, this node should have a TCP connection to send to the new node. Because a single TCP port is single-direction.
        - So the above method needs 2 threads, each in charge of a TCP port.


    - Publisher:
        - A unix domain TCP port. have TCP open, when hearing a new subscriber, go send namecard.
    - Subscriber:
        - address on unix domain is meaningless. that's why you see `conn, addr = recv()`

    - Profiling result:
        - a publisher and a subscriber roughly consumes the same amount of CPU: during steady state, each process consumes less than 0.1% of Rpi 4B CPU

4. integration:
    - Want shmpublisher or subscriber to start when notify, then stop when notify.
    - if go, we go. if not, we sleep on the event
    - but subscriber can still get some garbage values
        - when no subscriber, set the last values to null. That is bad.
        - Messages should have timestamps; Then, as long as subscriber is faster than publisher, you won't miss messages.


