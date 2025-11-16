# core/logger.py

import logging
import sys

def init_logger(debug=False):
    logger = logging.getLogger("ros1_server")
    logger.setLevel(logging.DEBUG if debug else logging.INFO)

    # Avoid duplicate handlers on re-run
    if logger.hasHandlers():
        logger.handlers.clear()

    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] %(message)s', "%H:%M:%S")
    handler.setFormatter(formatter)

    logger.addHandler(handler)
    return logger
