#! /usr/bin/env python3
import os
import typing


def feature_flag_read(name: str) -> typing.Union[bool, str]:
    flag_val: str = os.environ.get(name).lower()
    BOOL_FLAGS = {"true": True, "false": False}
    if flag_val in BOOL_FLAGS:
        return BOOL_FLAGS[flag_val]
    else:
        return flag_val


def try_remove_file(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
