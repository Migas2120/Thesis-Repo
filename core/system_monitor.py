#!/usr/bin/env python3
"""
SystemMonitor
-------------
Tracks CPU and memory utilization of the current process in real time.
Logs both to console and (optionally) to a .jsonl file for later analysis.
"""

import psutil
import time
import json
import threading
import os




class SystemMonitor:
    def __init__(self, logger, log_file=None, interval=1.0):
        """
        Args:
            logger: Python logger instance for console output.
            log_file: Optional path to .jsonl file for storing logs.
            interval: Time (seconds) between samples.
        """
        self.logger = logger
        self.log_file = log_file
        self.interval = interval
        self.process = psutil.Process()
        self.running = True

    def start(self):
        """Start the background monitoring thread."""
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()

    def _loop(self):
        """Monitor CPU and memory usage at fixed intervals."""
        while self.running:
            try:
                # Get CPU usage (% per core, averaged across all cores)
                cpu = self.process.cpu_percent(interval=None)
                mem_info = self.process.memory_info()
                mem_mb = mem_info.rss / (1024 * 1024)  # RSS memory in MB

                # Log to console
                self.logger.debug(f"[SystemMonitor] CPU={cpu:.2f}% | Memory={mem_mb:.2f} MB")

                # Log to file if provided
                if self.log_file:
                    with open(self.log_file, "a") as f:
                        entry = {
                            "timestamp": time.time(),
                            "cpu_percent": cpu,
                            "memory_mb": mem_mb
                        }
                        f.write(json.dumps(entry) + "\n")

            except Exception as e:
                self.logger.error(f"[SystemMonitor] Error: {e}")

            time.sleep(self.interval)

    def stop(self):
        """Stop the background monitoring thread."""
        self.running = False
