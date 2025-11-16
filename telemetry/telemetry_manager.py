import time
import json
import threading
import datetime

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.pose_handler import PoseHandler
from telemetry.telemetry_handlers.battery_handler import BatteryHandler
from telemetry.telemetry_handlers.gps_handler import GPSHandler
from telemetry.telemetry_handlers.status_handler import StatusHandler

class TelemetryManager:

    def __init__(self, logger=None, telemetry_log_file=None, log_interval=1.0):
        self.logger = logger 
        self.telemetry_log_file = telemetry_log_file
        self.log_interval = log_interval  # Seconds between logs
        self._stop_flag = threading.Event()

        self.handlers = [
            PoseHandler(logger=logger),
            BatteryHandler(logger=logger),
            GPSHandler(logger=logger),
            StatusHandler(logger=logger)
        ]

        self.last_snapshot = {}
        self.pending_command = None
        self.last_command_time = None
        self.last_effect_time = None

        if telemetry_log_file:
            log_dir = os.path.dirname(telemetry_log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            self.logger.info(f"[TelemetryManager] Logging to {telemetry_log_file}")

        self.latency_log_file = os.path.join(
            os.path.dirname(self.telemetry_log_file),
            f"latency_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl"
        )
        #self.logger.info(f"[TelemetryManager] Latency logs → {self.latency_log_file}")
        
        self.thread = threading.Thread(target=self._run_logger, daemon=True)
        self.thread.start()
        self.logger.info(f"[TelemetryManager] Background telemetry logging started.")

    def _run_logger(self):
        while not self._stop_flag.is_set():
            self.log_once()
            time.sleep(self.log_interval)
            
    def get_all(self):
        data = {}
        for handler in self.handlers:
            serialized = handler.get_serialized()
            if serialized:
                data.update(serialized)
        return data

    def log_once(self):
        """
        Serializes and appends current telemetry snapshot to JSONL file.
        """
        if not self.telemetry_log_file:
            return

        data = self.get_all()
        if not data:
            return

        # ---- BEGIN LATENCY PROBE (VERBOSE) ----
        try:
            self.logger.debug(f"[LatencyProbe] tick start | pending={getattr(self, 'pending_command', None)} "
                            f"| last_cmd_time={getattr(self, 'last_command_time', None)}")

            if self.pending_command:
                status = data.get("status", {})
                pose   = data.get("pose", {})
                gps    = data.get("gps", {})

                # Unpack pose safely (matches your JSON: pose.position.{x,y,z})
                pose_now  = (pose or {}).get("position", {})
                pose_prev = (self.last_snapshot or {}).get("pose", {}).get("position", {})

                self.logger.debug("[LatencyProbe] snapshot check | "
                                f"pose_now_keys={list(pose_now.keys()) if pose_now else None} "
                                f"| pose_prev_keys={list(pose_prev.keys()) if pose_prev else None} "
                                f"| mode={status.get('mode')} | armed={status.get('armed')}")

                # 1) ARM
                if self.pending_command == "arm":
                    self.logger.debug(f"[LatencyProbe] arm branch | status.armed={status.get('armed')}")
                    if status.get("armed", False):
                        t3 = time.perf_counter()
                        delta_ms = (t3 - self.last_command_time) * 1000
                        #self.logger.info(f"[LatencyProbe] Command 'arm' latency = {delta_ms:.1f} ms")
                        self._record_latency("arm", delta_ms)
                        self.pending_command = None
                        self.last_effect_time = t3

                # 2) MODE
                elif self.pending_command == "mode":
                    self.logger.debug(f"[LatencyProbe] mode branch | status.mode={status.get('mode')}")
                    if status.get("mode") in ("GUIDED", "LAND", "RTL"):
                        t3 = time.perf_counter()
                        delta_ms = (t3 - self.last_command_time) * 1000
                        #self.logger.info(f"[LatencyProbe] Command 'mode' latency = {delta_ms:.1f} ms ({status.get('mode')})")
                        self._record_latency("mode", delta_ms)
                        self.pending_command = None
                        self.last_effect_time = t3

                # 3) MOVEMENT (pos / pos_global / vel)
                elif self.pending_command in ("pos", "pos_global", "vel"):
                    if pose_now and pose_prev:
                        dx = float(pose_now.get("x", 0.0)) - float(pose_prev.get("x", 0.0))
                        dy = float(pose_now.get("y", 0.0)) - float(pose_prev.get("y", 0.0))
                        dz = float(pose_now.get("z", 0.0)) - float(pose_prev.get("z", 0.0))
                        dist = (dx*dx + dy*dy + dz*dz) ** 0.5

                        """
                        self.logger.info(
                            f"[LatencyProbe] Δ=({dx:+.3f},{dy:+.3f},{dz:+.3f}) dist={dist:.3f} "
                            f"| cmd={self.pending_command}"
                        )
                        
                        """

                        # Threshold for movement detection (meters)
                        threshold_m = 0.15
                        if dist > threshold_m:
                            t3 = time.perf_counter()
                            raw_latency_ms = (t3 - self.last_command_time) * 1000

                            # --- NEW: estimate expected motion time ---
                            v = getattr(self, "last_command_velocity", None)
                            if v is None:
                                v = 0.0
                            expected_ms = (threshold_m / max(v, 1e-3)) * 1000  # prevent div-by-zero
                            adjusted_ms = max(0.0, raw_latency_ms - expected_ms)

                            """
                            self.logger.info(
                                f"[LatencyProbe] Command '{self.pending_command}' latency = {raw_latency_ms:.1f} ms "
                                f"(expected travel={expected_ms:.1f} ms → comm={adjusted_ms:.1f} ms)"
                            )
                            """
                            

                            # Record both metrics (optional: adjust to your JSON schema)
                            self._record_latency(self.pending_command, adjusted_ms)
                            self._record_latency(f"{self.pending_command}_raw", raw_latency_ms)

                            # Clear pending state
                            self.pending_command = None
                            self.last_effect_time = t3
                    else:
                        self.logger.debug(
                            f"[LatencyProbe] movement branch | "
                            f"pose_now={bool(pose_now)} pose_prev={bool(pose_prev)} (waiting for both)"
                        )
        except Exception as e:
            self.logger.error(f"[TelemetryManager] LatencyProbe error: {e}")
        # ---- END LATENCY PROBE (VERBOSE) ----

        except Exception as e:
            self.logger.error(f"[TelemetryManager] LatencyProbe error: {e}")

        snapshot = {
            "timestamp": time.time(),
            "telemetry": data
        }

        try:
            """
            with open(self.telemetry_log_file, "a") as f:
                f.write(json.dumps(snapshot) + "\n")

                self.last_snapshot = data
            """
        except Exception as e:
            self.logger.error(f"[TelemetryManager] Failed to write telemetry log: {e}")
    
    def get_all_latest_status(self):
        return self.get_all()

    def get_latest_pose(self):
        for handler in self.handlers:
            if hasattr(handler, "get_pose"):
                return handler.get_pose()
        return None
    
    def get_pose_offset(self):
        for handler in self.handlers:
            if hasattr(handler, "get_offset"):
                return handler.get_offset()
        return None

    def track_command(self, cmd_name: str):
        self.pending_command = cmd_name
        self.last_command_time = time.perf_counter()
        #self.logger.info(f"[LatencyProbe] Tracking command '{cmd_name}' at {self.last_command_time:.6f}")

    def _record_latency(self, command_name: str, delta_ms: float):
        """
        Append a latency record to latency_<timestamp>.jsonl
        """

        """
        try:
            entry = {
                "timestamp": time.time(),
                "command": command_name,
                "latency_ms": round(delta_ms, 2)
            }
            with open(self.latency_log_file, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception as e:
            self.logger.error(f"[TelemetryManager] Failed to record latency: {e}")
        """



    def shutdown(self):
        self._stop_flag.set()
        self.thread.join()
        #self.logger.info("[TelemetryManager] Logging thread stopped.")
