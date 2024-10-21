#!/usr/bin/env python3
import logging


class ColorCode:
    RED = "\033[91m"
    DARK_RED = "\033[31m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    RESET = "\033[0m"


class ColorFormatter(logging.Formatter):
    COLOR_MAP = {
        logging.DEBUG: ColorCode.BLUE,
        logging.INFO: ColorCode.GREEN,
        logging.WARNING: ColorCode.YELLOW,
        logging.ERROR: ColorCode.RED,
        logging.CRITICAL: ColorCode.DARK_RED,
    }

    def format(self, record):
        record.name = f"{ColorCode.CYAN}{record.name}{ColorCode.RESET}"
        record.levelname = f"{self.COLOR_MAP[record.levelno]}{record.levelname}{ColorCode.RESET}"
        msg = logging.Formatter.format(self, record)
        parts = msg.split(" - ", maxsplit=1)
        parts[0] = f"{ColorCode.MAGENTA}{parts[0]}{ColorCode.RESET}"
        msg = " - ".join(parts)
        return msg


def get_logger(name: str, print_level: str = "INFO"):
    logger = logging.getLogger(name)
    # Python caches loggers with the same name. If we get a cached logger but add
    # a new handler, messages will be printed twice
    if not logger.handlers:
        level = getattr(logging, print_level.upper())
        logger.setLevel(level)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        formatter = ColorFormatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    return logger


if __name__ == "__main__":
    l1 = get_logger("MY_LOGGER", print_level="DEBUG")
    l1.debug("hola")
    l = get_logger("MY_LOGGER", print_level="INFO")
    l.info("Holi")
