#!/usr/bin/env python3
import time


class Rate:
    """Notes
    1. There's no convenient way to adaptively sleep for a certain amount of time.
    """

    def __init__(self, rate: float):
        self.sleep_time = 1.0 / rate
        self.next_wakeup_time = time.perf_counter() + self.sleep_time

    def sleep(self):
        time.sleep(max(0.0, self.next_wakeup_time - time.perf_counter()))
        self.next_wakeup_time = time.perf_counter() + self.sleep_time


def spin():
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
