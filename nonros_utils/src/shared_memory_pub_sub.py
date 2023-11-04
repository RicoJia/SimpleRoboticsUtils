#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from threading import Thread
import posix_ipc
import mmap
import struct
