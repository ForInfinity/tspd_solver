import logging
import os
import re
import sys
from typing import Optional

_module_logger: Optional[logging.Logger] = None


class TermEscapeCodeFormatter(logging.Formatter):
    """A class to strip the escape codes from the """

    def __init__(self, fmt=None, datefmt=None, style='%', validate=True):
        super().__init__(fmt, datefmt, style, validate)

    def format(self, record):
        escape_re = re.compile(r'\x1b\[[\d;]*m')
        record.msg = re.sub(escape_re, "", str(record.msg))
        return super().format(record)

# setup logging
def setup_logging():
    global _module_logger
    os.makedirs('./logs', exist_ok=True)
    if _module_logger is not None:
        return _module_logger

    # logger
    _module_logger = logging.getLogger('tspd_solver')
    _module_logger.setLevel(logging.DEBUG)
    _module_logger.propagate = False

    # create formatters
    simple_formatter = logging.Formatter("%(name)s: %(levelname)s: %(message)s")
    file_log_formatter = TermEscapeCodeFormatter("%(asctime)s %(name)s[%(process)d]: %(levelname)s - %(message)s")

    # create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(simple_formatter)

    # create file handler

    file_handler = logging.FileHandler('./logs/tspd-solver.debug.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(file_log_formatter)

    # add handlers
    _module_logger.addHandler(console_handler)
    _module_logger.addHandler(file_handler)


def get_logger() -> logging.Logger:
    if _module_logger is None:
        setup_logging()
    return _module_logger
