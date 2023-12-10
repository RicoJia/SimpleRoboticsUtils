#! /usr/bin/env python3
import os
def try_remove_file(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
