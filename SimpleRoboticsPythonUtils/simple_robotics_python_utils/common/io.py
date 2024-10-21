#! /usr/bin/env python3
import os
import typing
import enum


class ANSIColors(enum.Enum):
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    BRIGHT_RED = "\033[91m"
    BRIGHT_GREEN = "\033[92m"
    BRIGHT_YELLOW = "\033[93m"
    BRIGHT_BLUE = "\033[94m"
    BRIGHT_MAGENTA = "\033[95m"
    BRIGHT_CYAN = "\033[96m"
    BRIGHT_WHITE = "\033[97m"


def ask_user(
    msg: str, options: typing.List[str], msg_color: ANSIColors = ANSIColors.GREEN, options_color: ANSIColors = ANSIColors.BLUE
) -> str:
    if not isinstance(msg_color, ANSIColors) or not isinstance(options_color, ANSIColors):
        raise ValueError("color must be of type ANSIColors")
    print(f"{msg_color.value}{msg}{ANSIColors.RESET.value}")
    for i, opt in enumerate(options, 1):
        print(f"{options_color.value}{i}. {opt}{ANSIColors.RESET.value}")
    while True:
        try:
            choice = int(input("Please choose a valid option: "))
            if 0 <= choice - 1 <= len(options):
                return options[choice - 1]
            else:
                print(
                    f"{ANSIColors.RED.value}Invalid choice. Please enter a number between 1 and {len(options)}.{ANSIColors.RESET.value}"
                )
        except ValueError:
            print(f"{ANSIColors.RED.value}Invalid input. Please enter a number.{ANSIColors.RESET.value}")


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
