# drone_instance.py
MAX_PAUSED_MISSIONS = 5

from math import sqrt
import datetime
import time
import uuid
import json
import numpy as np
from collections import deque
import heapq
from typing import List, Tuple, Optional, Callable
import sys
import os
import numpy as np
import trimesh
import copy
from shapely.geometry import Polygon, Point

# Append ROS 1 package root directory to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

log_filename = f"telemetry_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl"
cli_filename = f"cli_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"

cli_log_file = os.path.join(package_root, "cli_logs", cli_filename)
telemetry_log_file = os.path.join(package_root, "logs", log_filename)

from node_wrapper.node_wrapper import NodeWrapper
from telemetry.telemetry_manager import TelemetryManager
from mission.mission import Mission, PrioritizedMission
from telemetry.health_monitor import TelemetryHealthMonitor, TelemetryHealth    
from geo.geo_converter import GeoConverter





class DroneInstance:
    """
    Represents a single drone running on a companion PC with its own mission logic.
    """

    def __init__(self, domain_id: int, logger=None, on_landing: Optional[Callable]=None, unsafe: bool = False):
        # ——————————————————————————————————————————————
        # Identification & Logging
        # ——————————————————————————————————————————————
        self.domain_id = domain_id
        self.node_name = f"drone_instance_{domain_id}"
        self.logger    = logger
        self.unsafe    = unsafe
        self.cli_log_file = cli_log_file

        if self.cli_log_file:
            cli_log_dir = os.path.dirname(self.cli_log_file)
            if cli_log_dir and not os.path.exists(cli_log_dir):
                os.makedirs(cli_log_dir)
        # ——————————————————————————————————————————————
        # ROS Node Wrapper & Communication
        # ——————————————————————————————————————————————
        self.node = NodeWrapper(logger=self.logger, drone_id=self.domain_id, unsafe=self.unsafe)

        # ——————————————————————————————————————————————
        # Mission Queues & Command Buffer
        # ——————————————————————————————————————————————
        self.mission_queue   : List[PrioritizedMission] = []
        self.paused_missions : List[Mission]            = []
        self.command_queue   = deque()
        self.active_mission  : Optional[Mission]        = None

        # ——————————————————————————————————————————————
        # Telemetry Management
        # ——————————————————————————————————————————————
        self.telemetry_manager = TelemetryManager(
            logger=self.logger,
            telemetry_log_file=telemetry_log_file,
            log_interval=1.0
        )

        self.telemetry_health = TelemetryHealthMonitor(
            window_sec=3.0,
            max_stale_sec_pose=0.5,
            max_stale_sec_gps=1.5,
            max_jump_m=2.0,
            max_vel_mps=5.0,
            jitter_m_mad_limit=0.4,
            mad_k=3.5
        )
        self.telemetry_degraded = False

        # ——————————————————————————————————————————————
        # Flight State Flags
        # ——————————————————————————————————————————————
        self.has_taken_off           = False
        self.waiting_to_start_mission = False
        self.is_holding               = False
        self.hold_start_time          = None
        self.landing_in_progress = False

        # ——————————————————————————————————————————————
        # Safety & Abort Locks
        # ——————————————————————————————————————————————
        self.abort_locked      = False
        self.mission_abort_lock = False
        self.returning_to_base  = False
        self.use_global_fallback = False
        
        self.geo = GeoConverter()

        # ——————————————————————————————————————————————
        # Manual Control Mode
        # ——————————————————————————————————————————————
        self.manual_mode = False

        # ——————————————————————————————————————————————
        # Incoming-Message Dispatcher
        # ——————————————————————————————————————————————
        self._msg_handlers = {
            ("command", "start_manual"):    lambda d: self._set_manual(True),
            ("command", "stop_manual"):     lambda d: self._set_manual(False),
            ("add_mission", None):          self._handle_add_mission,
            ("command", None):              self._handle_generic_command,
            ("map_area", None):             self._handle_map_area,
            ("resume_missions", None):      lambda d: self._handle_resume(),
            ("edit_mission", None):         self._handle_edit_mission,
            ("change_priority", None):      self._handle_change_priority,
            ("return_to_base", None):       lambda d: self._handle_return_to_base(),
            ("manual_control", None):       self._handle_manual_control,
            ("delete_mission", None):       self._handle_delete_mission,
        }


        # ——————————————————————————————————————————————
        # Landing Callback
        # ——————————————————————————————————————————————
        self.on_landing = on_landing

        mode = "UNSAFE" if self.unsafe else "SAFE"
        self._log("info", f"Initializing DroneInstance with domain ID {self.domain_id} [{mode} mode]")

    def tick(self):

        # ——————————————————————————————————————————————
        # TELEMETRY → Unity (always send, even if early return)
        # ——————————————————————————————————————————————
        try:
            pose = self.telemetry_manager.get_latest_pose()
            telemetry = self.telemetry_manager.get_all_latest_status()
            battery = telemetry.get("battery", {}) if telemetry else {}
            status = telemetry.get("status", {}) if telemetry else {}

            packet = {
                "type": "telemetry_update",
                "id": self.domain_id,
                "timestamp": time.time(),
                "active_mission": (
                    self.active_mission.mission_id if self.active_mission else None
                ),
                "battery": {
                    "percentage": battery.get("percentage"),
                    "voltage": battery.get("voltage"),
                },
                "position": {
                    "x": pose["x"] if pose else None,
                    "y": pose["y"] if pose else None,
                    "z": pose["z"] if pose else None,
                },
                "status": {
                    "mode": status.get("mode"),
                    "armed": status.get("armed"),
                }
            }

            self._log("debug", f"Telemetry : {packet}")

            self.node.publish_to_unity(packet)
        except Exception as e:
            self._log("error", f"Telemetry send failed: {e}")

        # ——————————————————————————————————————————————
        # NORMAL TICK FLOW
        # ——————————————————————————————————————————————
        
        if self.landing_in_progress:
            return

        if self.abort_locked:
            return

        # 1) Return‐to‐base handling
        if self._handle_returning_to_base():
            return
        
        # 2) Battery safety check
        if self._check_battery_fail_safe():
            return  
        
        """
        health = self._check_telemetry_health()
        if health and health.degraded and not self.use_global_fallback:
            # Telemetry bad and no GPS fallback → pause
            return
        """

        
        
        # 3) Manual mode bypass
        if self.manual_mode:
            return

        

        # 4) Only pick a new mission if no mission is currently active
        if self._try_start_next_mission():
            return

        # 5) Handle mission execution
        if self.active_mission:
            self._execute_active_mission()



    def _execute_active_mission(self):
        mission = self.active_mission

        telemetry = self.telemetry_manager.get_all_latest_status()
        pose = self.telemetry_manager.get_latest_pose()
        status = telemetry.get("status", {}) if telemetry else {}

        if not pose and not self.use_global_fallback:
            self._log("warn", "Mission execution halted: no pose and GPS fallback disabled.")
            return

        if not status and not self.unsafe:
            self._log("warn", "Mission execution halted: no status and not in unsafe mode.")
            return

        pose_z = None
        if pose and self.telemetry_manager.get_pose_offset():
            offset_z = self.telemetry_manager.get_pose_offset()["z"]
            pose_z = pose["z"] - offset_z

        if pose_z is not None and pose_z > 1.5:
            self.has_taken_off = True
        
        takeoff_altitude = 2.0 + self.telemetry_manager.get_pose_offset()["z"]


        # Only handle arming + GUIDED mode before takeoff
        if not self.has_taken_off:
            # 1. Arm if not armed
            if not status.get("armed", False):
                self._log("info", "Drone not armed. Sending arm command.")
                self.node.publish_from_unity(json.dumps({
                    "type": "command",
                    "command": "arm",
                    "id": self.domain_id
                }))
                return

            # 2. Switch to GUIDED mode if not already
            if status.get("mode") != "GUIDED":
                self._log("info", "Switching to GUIDED mode.")
                self.node.publish_from_unity(json.dumps({
                    "type": "command",
                    "command": "mode",
                    "mode": "GUIDED",
                    "id": self.domain_id
                }))
                return

            # 3. Take off if we haven't yet
            self._log("info", "Initiating takeoff to 2.0m.")
            self.node.publish_from_unity(json.dumps({
                "type": "command",
                "command": "takeoff",
                "altitude": takeoff_altitude,
                "id": self.domain_id
            }))
            self.has_taken_off = True
            self.waiting_to_start_mission = True
            return

        # 4. Wait until drone reaches takeoff altitude
        if self.waiting_to_start_mission:
            if pose["z"] >= takeoff_altitude - 0.5:
                self._log("info", "Reached takeoff altitude. Starting mission.")
                self.waiting_to_start_mission = False
            else:
                self._log("debug", f"Waiting to reach takeoff altitude: z={pose['z']:.2f}")
                return

        # 5. Proceed with mission (local pose or GPS fallback)
        if self._mission_is_complete(mission):
            self._finish_mission()
            return

        target_wp_data   = mission.waypoints[mission.current_wp_index]
        target_wp_coords = target_wp_data["coords"]

        if self.use_global_fallback and getattr(self, "geo", None) and self.geo.has_origin:
            cur_gps = self._get_current_gps()
            if not cur_gps:
                self._log("warn", "GPS fallback active but no current GPS fix; skipping tick")
                return
            tgt_lla = self._to_lla_cached(target_wp_data)
            dist3, horiz, dz = self._compute_distance_lla(tgt_lla, cur_gps)
            self._log("debug", f"[GPS] dist3={dist3:.2f}m (horiz={horiz:.2f}m, dz={dz:.2f}m)")

            # GPS reach criteria (tune as needed)
            reached = (horiz < 1.5 and abs(dz) < 1.0) or dist3 < 2.0
            if reached:
                self._complete_current_waypoint(mission)
            else:
                # reuse retry logic with GPS-based distance
                self._handle_unreached_waypoint(mission, target_wp_data, dist3)
        else:
            # normal local pose path
            dist = self._compute_distance(pose, target_wp_coords)
            self._log("debug", f"Distance to waypoint: {dist:.2f} meters")
            if self._has_reached_waypoint(dist):
                self._complete_current_waypoint(mission)
            else:
                self._handle_unreached_waypoint(mission, target_wp_data, dist)

    def _try_start_next_mission(self) -> bool:
        # ——————————————————————————————————————————————
        # If no mission is active and there are missions queued, pop the
        # highest‐priority one, respect its start_time, and dispatch it.
        # Returns True if a mission was started (so tick() should return).
        # ——————————————————————————————————————————————

        if self.active_mission is not None or not self.mission_queue:
            return False

        now = time.time()
        while self.mission_queue:
            prioritized = heapq.heappop(self.mission_queue)
            mission = prioritized.mission

            if not prioritized.valid:
                self._log("debug", f"Skipping invalidated mission: {prioritized.mission_id}")
                continue

            # Respect scheduled start_time
            if getattr(mission, "start_time", None) and now < mission.start_time:
                self._log(
                    "debug",
                    f"Mission '{mission.mission_id}' scheduled for later "
                    f"(start_time={mission.start_time}, now={now:.0f})"
                )
                heapq.heappush(self.mission_queue, prioritized)
                return False

            # Assign & dispatch
            self.active_mission = mission
            self._log("info", f"Starting mission: {mission.mission_id}")
            self.node.publish_to_unity({
                "type":       "mission",
                "id":          self.domain_id,
                "mission_id":  mission.mission_id,
                "status":      "started"
            })
            return True

        return False
    
    def _mission_is_complete(self, mission):
        if mission.current_wp_index >= len(mission.waypoints):
            if mission.mode == "patrol":
                mission.completed_loops += 1
                if mission.patrol_loops < 0 or mission.completed_loops < mission.patrol_loops:
                    self._log("info", f"Patrol loop {mission.completed_loops} complete — restarting.")
                    mission.current_wp_index = 0
                    return False
                else:
                    return True
            return True
        return False


    def _handle_returning_to_base(self) -> bool:
        # ——————————————————————————————————————————————
        # If we’re in return‐to‐base mode, check for home arrival,
        # land once there, clear the flag, and return True to stop tick().
        # ——————————————————————————————————————————————

        if not self.returning_to_base:
            return False

        pose = self.telemetry_manager.get_latest_pose()
        if not pose:
            return False

        x, y = pose["x"], pose["y"]
        # within 0.5 m of home?
        if abs(x) < 0.5 and abs(y) < 0.5:
            self._log("info", "Home reached — issuing LAND command")
            self.node.publish_from_unity(json.dumps({
                "type":    "command",
                "command": "mode",
                "mode":    "LAND",
                "id":      self.domain_id
            }))
            self.returning_to_base = False
            self.landing_in_progress = True

            # notify AppRunner to shut everything down
            if self.on_landing:
                self.on_landing()
        return True
    
    def _finish_mission(self):
        self._log("info", f"Mission '{self.active_mission.mission_id}' complete.")
        self.node.publish_to_unity({
            "type":       "mission",
            "id":          self.domain_id,
            "mission_id":  self.active_mission.mission_id,
            "status":      "completed"
        })
        self.active_mission = None

        if self.paused_missions:
            resumed = self.paused_missions.pop()
            self._log("info", f"Resuming paused mission '{resumed.mission_id}' from waypoint {resumed.start_index}.")

            resumed.waypoints = resumed.waypoints[resumed.start_index:]
            resumed.current_wp_index = 0
            self.active_mission = resumed
            
            self.node.publish_to_unity({
                "type":       "mission",
                "id":          self.domain_id,
                "mission_id":  resumed.mission_id,
                "status":      "resumed"
            })

            return

        if self.mission_queue:
            return  # let tick() pick the next one

        self._dispatch_queued_commands()

    def _compute_distance(self, pose, target):
        dx = target[0] - pose["x"]
        dy = target[1] - pose["y"]
        dz = ( target[2] + self.telemetry_manager.get_pose_offset()["z"]) - pose["z"] 
        return sqrt(dx**2 + dy**2 + dz**2)

    def _has_reached_waypoint(self, dist):
        return dist < 0.5

    def _complete_current_waypoint(self, mission):
        idx = mission.current_wp_index
        wp_data = mission.waypoints[idx]
        hold_duration = wp_data.get("hold", 0)

        # DEBUG: entry state
        self._log("debug", f"_complete_current_waypoint: idx={idx}, hold_duration={hold_duration}, "
                        f"is_holding={self.is_holding}, hold_start_time={self.hold_start_time}")

        # 1) start hold if needed
        if hold_duration > 0 and not self.is_holding:
            self._log("info",  f"Starting hold at waypoint #{idx+1} for {hold_duration}s")
            self.hold_start_time = time.time()
            self.is_holding       = True
            return

        # 2) continue or finish hold
        if self.is_holding:
            elapsed = time.time() - self.hold_start_time
            self._log("debug", f" Holding… elapsed={elapsed:.2f}s / target={hold_duration:.2f}s")
            if elapsed < hold_duration:
                return
            # hold complete
            self._log("info", f" Hold complete at waypoint #{idx+1} after {elapsed:.2f}s")
            self.is_holding       = False
            self.hold_start_time  = None

        # 3) advance to next waypoint
        new_idx = idx + 1
        self._log("debug", f" Advancing from waypoint #{idx+1} to #{new_idx+1}")
        mission.current_wp_index = new_idx
        mission.last_command_time = None
        mission.retry_count       = 0
        mission.last_distance     = None

    def _handle_unreached_waypoint(self, mission, target_wp_data, dist):
        now = time.time()

        if mission.last_command_time is None:
            self._send_waypoint_command(target_wp_data["coords"])
            mission.last_command_time = now
            mission.last_distance = dist
            mission.retry_count = 1
        elif (now - mission.last_command_time) > 5.0:
            if dist < mission.last_distance:
                self._log("debug", f"Still approaching target... {dist:.2f}m remaining")
                mission.last_distance = dist
            else:
                if mission.retry_count < 3:
                    self._log("warn", f"No progress. Retrying waypoint command ({mission.retry_count + 1}/3)")
                    self._send_waypoint_command(target_wp_data["coords"])
                    mission.last_command_time = now
                    mission.retry_count += 1
                else:
                    self._log("error", "Too many retries without progress. Skipping waypoint.")
                    mission.current_wp_index += 1
                    mission.last_command_time = None
                    mission.retry_count = 0
                    mission.last_distance = None

    def _send_waypoint_command(self, waypoint):
        """
        waypoint is (x,y,z) in local frame. If fallback is active and we have a geo origin,
        translate to LLA and publish a 'pos_global' command; otherwise publish local 'pos'.
        """
        if self.use_global_fallback and getattr(self, "geo", None) and self.geo.has_origin:
            try:
                lat, lon, alt = self.geo.enu_to_lla(waypoint[0], waypoint[1], waypoint[2])
                # --- Latency Probe Start ---
                send_time = time.perf_counter()
                if hasattr(self.telemetry_manager, "track_command"):
                    self.telemetry_manager.track_command("pos_global")
                self._log("debug", f"LatencyProbe | Sent pos_global command at {send_time:.6f}")
                # --- Latency Probe End ---
                self.node.publish_from_unity(json.dumps({
                    "type": "command",
                    "command": "pos_global",   # handled by your GlobalPositionCommand
                    "id": self.domain_id,
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "frame": "REL_ALT"         # change to "AMSL"/"TERRAIN" if needed
                }))
                self._log("info", f"GPS fallback: LLA setpoint lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}")
                return
            except Exception as e:
                self._log("error", f"Fallback LLA conversion failed, sending local pos. Err={e}")
        
        fake_unity_cmd = {
            "command": "pos",
            "x": waypoint[0],
            "y": waypoint[1],
            "z": waypoint[2]
        }
        # --- Latency Probe Start ---
        send_time = time.perf_counter()
        if hasattr(self.telemetry_manager, "track_command"):
            self.telemetry_manager.track_command("pos")
        self._log("debug", f"LatencyProbe | Sent pos command at {send_time:.6f}")
        # --- Latency Probe End ---

        self.node.publish_from_unity(json.dumps(fake_unity_cmd))

    def _handle_abort(self):
        self._log("warn", "Abort command received — stopping drone and aborting current mission.")

        current_pose = self.telemetry_manager.get_latest_pose()
        telemetry = self.telemetry_manager.get_all_latest_status()
        current_status = telemetry.get("status", {}) if telemetry else {}

        pose_position = current_pose.get("position") if current_pose else None
        if pose_position:
            hold_cmd = {
                "command": "pos",
                "x": pose_position["x"],
                "y": pose_position["y"],
                "z": pose_position["z"]
            }
            self.node.publish_from_unity(json.dumps(hold_cmd))
            self._log("info", "Issued hold position command after abort.")
        else:
            self._log("warn", "Cannot issue hold position — no pose available.")

        # Cancel the current mission
        self.active_mission = None
        self.command_queue.clear()
        self.mission_abort_lock = True

        # Only reset flight state if drone is on the ground (disarmed)
        if not current_status.get("armed", False):
            self._log("info", "Drone is disarmed — resetting takeoff state.")
            self.has_taken_off = False
            self.waiting_to_start_mission = False

    # --- GPS helpers ---
    def _get_current_gps(self):
        tel = self.telemetry_manager.get_all_latest_status() or {}
        gps = tel.get("gps") or {}
        lat = gps.get("lat") or gps.get("latitude")
        lon = gps.get("lon") or gps.get("longitude")
        alt = gps.get("alt") or gps.get("altitude")
        if None in (lat, lon, alt):
            return None
        return {"lat": float(lat), "lon": float(lon), "alt": float(alt)}

    def _haversine_m(self, lat1, lon1, lat2, lon2):
        from math import radians, sin, cos, sqrt, asin
        R = 6371000.0
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
        return 2 * R * asin(sqrt(a))

    def _compute_distance_lla(self, tgt_lla, cur_lla):
        horiz = self._haversine_m(cur_lla["lat"], cur_lla["lon"], tgt_lla[0], tgt_lla[1])
        dz = float(tgt_lla[2]) - float(cur_lla["alt"])
        dist3 = (horiz**2 + dz**2) ** 0.5
        return dist3, horiz, dz

    def _to_lla_cached(self, wp_data):
        # wp_data: {"coords": (x,y,z), ...}
        if "lla" in wp_data:
            return wp_data["lla"]
        if not (getattr(self, "geo", None) and self.geo.has_origin):
            raise RuntimeError("Geo origin not set; cannot convert to LLA")
        x, y, z = wp_data["coords"]
        lat, lon, alt = self.geo.enu_to_lla(x, y, z)
        wp_data["lla"] = (lat, lon, alt)  # cache
        return wp_data["lla"]

    def _check_battery_fail_safe(self) -> bool:
        #--————————————————————————————————————————————
        # Checks battery level and issues RTL if critical. Returns True if
        # fail-safe was triggered and handled (so tick() should return).
        # ——————————————————————————————————————————————

        if self.returning_to_base or self.landing_in_progress:
            return 
    
        telemetry = self.telemetry_manager.get_all_latest_status()

        if not telemetry:
            self._log("warn", "Battery check skipped — no telemetry available.")
            return False

        battery_data     = telemetry.get("battery", {})
        battery_percent  = battery_data.get("percentage")  # 0.0–1.0

        if battery_percent is None:
            self._log("warn", "Battery percentage unavailable.")
            return False

        pct = battery_percent * 100

        # CRITICAL
        if pct < 20.0:
            # Only issue RTL once
            if not self.returning_to_base:
                self._log("error", f"Battery critically low ({pct:.1f}%) — issuing RTL and aborting mission.")

                # 1) Command RTL
                self.node.publish_from_unity(json.dumps({
                    "type":    "command",
                    "command": "mode",
                    "mode":    "RTL",
                    "id":      self.domain_id
                }))

                # 2) Clean up mission state
                self.active_mission    = None
                self.command_queue.clear()
                self.mission_abort_lock = True

                # Enter returning-to-base mode so we don’t spam again
                self.returning_to_base = True

            # In either case, we handled the fail‐safe, so tick() should return
            return True

        # LOW (warning) but non‐critical
        if pct < 25.0:
            self._log("warn", f"Battery low ({pct:.1f}%).")

        return False

    def _check_telemetry_health(self) -> Optional[TelemetryHealth]:
        tel  = self.telemetry_manager.get_all_latest_status() or {}
        pose = self.telemetry_manager.get_latest_pose() or None
        gps  = tel.get("gps", {})  # expect keys: lat/lon/alt, hdop, satellites

        # Try to bind ENU↔LLA origin when we first have decent pose+gps
        if pose and gps:
            self._try_init_geo_origin(pose, gps)

        health = self.telemetry_health.update(pose=pose, gps=gps)
        self._maybe_toggle_fallback(health, gps)

        # If degraded, pause aggressive actions and hold position
        if health.degraded:
            if not self.telemetry_degraded:
                self._log("warn", f"Telemetry degraded: {health.reason} (HDOP={health.hdop}, NSats={health.nsats})")
                self.node.publish_to_unity({
                    "type": "telemetry_health",
                    "id": self.domain_id,
                    "status": "degraded",
                    "reason": health.reason,
                    "hdop": health.hdop,
                    "nsats": health.nsats
                })

            self.telemetry_degraded = True

            # If we still have pose, hold locally; otherwise switch LOITER
            if pose:
                self.node.publish_from_unity(json.dumps({
                    "type": "command", "command": "pos", "id": self.domain_id,
                    "x": pose["x"], "y": pose["y"], "z": pose["z"]
                }))
            else:
                self.node.publish_from_unity(json.dumps({
                    "type":"command","command":"mode","mode":"LOITER","id":self.domain_id
                }))
        else:
            if self.telemetry_degraded:
                self._log("info", "Telemetry recovered.")
                self.node.publish_to_unity({
                    "type": "telemetry_health",
                    "id": self.domain_id,
                    "status": "ok"
                })
            self.telemetry_degraded = False

        return health


    def _try_init_geo_origin(self, pose: dict, gps: dict):
        """
        Bind local (x,y,z) to first good GPS fix to define ENU<->LLA origin.
        Requires pose + gps with decent quality.
        """
        if self.geo.has_origin or not pose or not gps:
            return

        lat = gps.get("lat") or gps.get("latitude")
        lon = gps.get("lon") or gps.get("longitude")
        alt = gps.get("alt") or gps.get("altitude")
        nsats = gps.get("satellites") or gps.get("nsats")
        hdop  = gps.get("hdop") or gps.get("h_acc")

        if lat is None or lon is None or alt is None:
            return
        # minimally decent fix (tune as needed)
        if (nsats is not None and int(nsats) < 6) or (hdop is not None and float(hdop) > 2.0):
            return

        self.geo.set_origin(float(lat), float(lon), float(alt),
                            x0=float(pose["x"]), y0=float(pose["y"]), z0=float(pose["z"]))
        self._log("info", f"Geo origin set @ LLA=({float(lat):.7f},{float(lon):.7f},{float(alt):.1f}) "
                        f"⇄ local=({pose['x']:.2f},{pose['y']:.2f},{pose['z']:.2f})")

    def _maybe_toggle_fallback(self, health, gps: dict):
        """
        Enable GPS fallback when pose is bad but GPS is OK; disable when recovered.
        """
        if health.degraded:
            pose_bad = (health.pose_stale or health.jitter_high or health.jump_detected or health.vel_spike)
            gps_ok   = (not health.gps_stale) and (not health.gps_poor)
            if pose_bad and gps_ok and not self.use_global_fallback:
                self.use_global_fallback = True
                self._log("warn", "Enabling GPS GLOBAL setpoint fallback (pose degraded, GPS OK).")
            return

        # recovered
        if self.use_global_fallback:
            self.use_global_fallback = False
            self._log("info", "Disabling GPS fallback (telemetry recovered).")

    """
    Portion of the code that handles mission editing commands.
    This includes adding, removing, and moving waypoints in the active mission.
    """


    def _handle_edit_mission(self, data: dict):
        mission_id = data.get("mission_id")
        action = data.get("action")
        index = data.get("index")
        wp = data.get("waypoint")

        # --- find which mission to edit ---
        target_mission = None
        if self.active_mission and (not mission_id or self.active_mission.mission_id == mission_id):
            target_mission = self.active_mission
        else:
            for m in self.paused_missions:
                if m.mission_id == mission_id:
                    target_mission = m
                    break
            if target_mission is None:
                for item in self.mission_queue:
                    if item.mission_id == mission_id:
                        target_mission = item.mission
                        break

        if not target_mission:
            self._log("warn", f"Edit mission requested for unknown mission_id '{mission_id}'.")
            return

        try:
            if action == "remove":
                removed = target_mission.waypoints.pop(index)
                self._log("info", f"Removed waypoint at index {index}: {removed}")

                # Adjust pointer if this was the active mission
                if target_mission == self.active_mission and \
                index <= target_mission.current_wp_index and \
                target_mission.current_wp_index > 0:
                    target_mission.current_wp_index -= 1

            elif action == "add":
                if not wp or len(wp) != 3:
                    raise ValueError("Waypoint must be a 3-element list [x, y, z]")

                new_wp = {"coords": list(wp)}
                target_mission.waypoints.insert(index, new_wp)
                self._log("info", f"Added waypoint at index {index}: {new_wp}")

                if target_mission == self.active_mission and index <= target_mission.current_wp_index:
                    target_mission.current_wp_index += 1

            elif action == "move":
                if not wp or len(wp) != 3:
                    raise ValueError("Waypoint must be a 3-element list [x, y, z]")
                target_mission.waypoints[index] = {"coords": list(wp)}
                self._log("info", f"Updated waypoint at index {index} to: {wp}")

            else:
                self._log("warn", f"Unknown edit_mission action: {action}")

        except Exception as e:
            self._log("error", f"Failed to edit mission '{mission_id}': {e}")

    def _handle_add_mission(self, data: dict):
        try:
            mission_id = data["mission_id"]
            waypoints_raw = data["waypoints"]
            if not isinstance(waypoints_raw, list):
                raise ValueError("Waypoints must be a list of [x, y, z] or waypoint dicts.")

            mode = data.get("mode", "once")
            patrol_loops = data.get("patrol_loops", 1)
            priority = data.get("priority", 5)  # Default is mid-priority

            new_mission = Mission(
                mission_id=mission_id,
                waypoints=waypoints_raw,
                mode=mode,
                patrol_loops=patrol_loops
            )

            if data.get("preempt", False) and self.active_mission:
                self._log("warn", f"Preempting mission '{self.active_mission.mission_id}' with '{mission_id}'.")

                if len(self.paused_missions) >= MAX_PAUSED_MISSIONS:
                    self._log("error", f"Cannot pause mission '{self.active_mission.mission_id}' — paused mission stack full.")
                    return
                
                paused = self.active_mission.pause(current_index=self.active_mission.current_wp_index)
                self.paused_missions.append(paused)

                self.node.publish_to_unity({
                    "type": "mission",
                    "id": self.domain_id,
                    "mission_id": paused.mission_id,
                    "status": "paused"
                })

                paused_ids = [m.mission_id for m in self.paused_missions]
                self._log("debug", f"Paused mission chain: {paused_ids}")

                self.active_mission = new_mission

                self.node.publish_to_unity({
                    "type":       "mission",
                    "id":          self.domain_id,
                    "mission_id":  self.active_mission.mission_id,
                    "status":      "started"
                })

            else:
                heapq.heappush(self.mission_queue, PrioritizedMission(
                    priority=priority,
                    mission=new_mission,
                    mission_id=mission_id
                ))
                self._log("info", f"Mission '{mission_id}' added with priority {priority} and {len(new_mission.waypoints)} waypoints.")

            # Reset takeoff if on ground
            telemetry = self.telemetry_manager.get_all_latest_status()
            status = telemetry.get("status", {}) if telemetry else {}
            pose = self.telemetry_manager.get_latest_pose()
            if (status and not status.get("armed", False)) or (pose and pose["z"] < self.telemetry_manager.get_pose_offset()["z"] + 0.5):
                self.has_taken_off = False
                self.waiting_to_start_mission = False
                self._log("debug", "Resetting takeoff flags (drone on ground).")

        except Exception as e:
            self._log("error", f"Failed to add mission: {e}")

    def _handle_resume(self):
        if self.mission_abort_lock:
            self.mission_abort_lock = False
            self._log("info", "Mission processing resumed.")
        else:
            self._log("info", "Resume requested, but mission processing was not locked.")

    def _handle_skip_waypoint(self):
        if self.active_mission is None:
            self._log("warn", "Skip requested but no active mission.")
            return

        self._log("info", f"Skip requested — skipping waypoint {self.active_mission.current_wp_index + 1}")
        self._complete_current_waypoint(self.active_mission)

    def _handle_map_area(self, data: dict):
        """
        Robust version: given ANY 3D corner points,
        builds the closed hull, slices it into horizontal layers,
        grids each slice polygon footprint.
        """
        try:
            

            mission_id = data.get("mission_id", "map_area_auto")

            raw_pts = data.get("points", [])

            points = np.array([
                [pt["x"], pt["y"], pt["z"]]
                for pt in raw_pts
            ], dtype=float)

            # Build convex hull from user points
            hull = trimesh.convex.convex_hull(points)

            # Determine Z range
            z_min = np.min(points[:, 2])
            z_max = np.max(points[:, 2])

            vertical_step = data.get("vertical_step", 2.0)
            spacing = data.get("grid_spacing", 5.0)
            priority = data.get("priority", 5)
            preempt = data.get("preempt", False)

            z_layers = np.arange(z_min, z_max + vertical_step, vertical_step)

            waypoints = []

            for z in z_layers:
                # Slice the hull with plane Z = z
                slice = hull.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
                if slice is None:
                    continue

                # Convert to planar polygon in XY
                path2D, T = slice.to_2D()
                polygon_paths = path2D.polygons_full
                if not polygon_paths:
                    continue

                poly = polygon_paths[0]  # If multiple, take outer loop

                # Convert to Shapely Polygon for point-in-polygon tests
                shapely_poly = Polygon(list(poly.exterior.coords))

                minx, miny, maxx, maxy = shapely_poly.bounds
                y_values = np.arange(miny, maxy + spacing, spacing)

                for i, y in enumerate(y_values):
                    x_values = np.arange(minx, maxx + spacing, spacing)
                    row = []
                    for x in x_values:
                        if shapely_poly.contains(Point(x, y)):
                            point_local = np.array([x, y, 0, 1])
                            point_world = T @ point_local
                            row.append((point_world[0], point_world[1], point_world[2]))
                    if i % 2 == 1:
                        row.reverse()
                    waypoints.extend(row)

            # Pack as standard mission
            mission_payload = {
                "type": "add_mission",
                "mission_id": mission_id,
                "waypoints": waypoints,
                "mode": "once",
                "patrol_loops": 1,
                "priority": priority,
                "preempt": preempt
            }

            self._handle_add_mission(mission_payload)

            self._log("debug", f"{mission_payload}")

            self._log(
                "info",
                f"[3D Slicer] {len(waypoints)} waypoints generated from 3D hull "
                f"between Z={z_min:.2f}-{z_max:.2f} every {vertical_step}m."
            )

        except Exception as e:
            self._log("error", f"Failed to handle map_area command: {e}")

    def _handle_change_priority(self, data: dict):
        mission_id = data.get("mission_id")
        new_priority = data.get("priority")

        if mission_id is None or new_priority is None:
            self._log("warn", "Invalid change_priority command — 'mission_id' and 'priority' required.")
            return

        found = False
        for item in self.mission_queue:
            if item.mission_id == mission_id and item.valid:
                item.valid = False  # Invalidate old item
                heapq.heappush(self.mission_queue, PrioritizedMission(
                    priority=new_priority,
                    mission=item.mission,
                    mission_id=item.mission_id
                ))
                found = True
                self._log("info", f"Priority for mission '{mission_id}' updated to {new_priority}.")
                break

        if not found:
            self._log("warn", f"Mission '{mission_id}' not found in queue for priority update.")

    def _handle_manual_control(self, data: dict):
        if not self.manual_mode:
            self._log("warn", "Manual control received while not in manual_mode")
            return

        if self.returning_to_base:
            self._log("warn", "Manual control ignored—return‐to‐base in progress")
            return

        axes = data.get("axes", {})
        # map sticks → velocity
        x   =  axes.get("ly", 0.0) * 1.0   # forward/back
        y   = -axes.get("lx", 0.0) * 1.0   # left/right
        z   =  axes.get("ry", 0.0) * 0.5   # up/down
        yaw =  axes.get("rx", 0.0) * 0.5   # yaw rate

        # --- Latency Probe Start ---
        send_time = time.perf_counter()

        # Compute velocity magnitude for latency adjustment
        self.telemetry_manager.last_command_velocity = (
            (abs(x)**2 + abs(y)**2 + abs(z)**2) ** 0.5
        )

        if hasattr(self.telemetry_manager, "track_command"):
            self.telemetry_manager.track_command("vel")
        self._log("debug", f"LatencyProbe | Sent vel command at {send_time:.6f}")
        # --- Latency Probe End ---

        cmd = {
            "type":    "command",
            "command": "vel",      # must match whatever your BaseCommand registry uses
            "id":      self.domain_id,
            "x":       x,
            "y":       y,
            "z":       z,
            "yaw":     yaw
        }
        self._log("info", f"Manual VEL cmd: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")
        self.node.publish_from_unity(json.dumps(cmd))

    def _handle_generic_command(self, data: dict):
        cmd = data.get("command")
        
        if cmd == "abort":
            return self._handle_abort()
        if cmd == "skip_wp":
            return self._handle_skip_waypoint()
        if cmd == "pause":
            return self._handle_pause()
        if cmd == "resume":
            return self._handle_resume()

        # If we’re in a mission, queue it
        if self.active_mission:
            self._log("info", "Mission active — queuing command for later execution.")
            self.command_queue.append(data)
        else:
            send_time = time.perf_counter()
            data["_t_sent"] = send_time
            self._log("debug", f"LatencyProbe | Sent {data['command']} id={data.get('_id')} at {send_time:.6f}")

            # tell the TelemetryManager we’re tracking this command
            if hasattr(self.telemetry_manager, "track_command"):
                try:
                    self.telemetry_manager.track_command(data.get("command"))
                except Exception as e:
                    self._log("error", f"LatencyProbe | track_command failed: {e}")

            # now publish the command normally
            self.node.publish_from_unity(json.dumps(data))

    def _handle_return_to_base(self):
        """Handle Return-to-Base (RTL) command."""
        self._log("info", "Return-to-Base requested — cancelling mission and switching to RTL mode.")

        # 1. Cancel any active mission and clear the queue
        if self.active_mission:
            self._log("info", f"Aborting active mission '{self.active_mission.mission_id}' for RTL.")
            self.active_mission = None

        self.command_queue.clear()
        self.mission_queue.clear()
        self.paused_missions.clear()

        # --- Latency Probe Start ---
        send_time = time.perf_counter()
        if hasattr(self.telemetry_manager, "track_command"):
            self.telemetry_manager.track_command("mode")
        self._log("debug", f"LatencyProbe | Sent mode=RTL at {send_time:.6f}")
        # --- Latency Probe End ---

        # 2. Switch flight mode to RTL
        self.node.publish_from_unity(json.dumps({
            "type": "command",
            "command": "mode",
            "mode": "RTL",
            "id": self.domain_id
        }))

        # 3. Update flags to prevent new mission dispatch
        self.returning_to_base = True
        
        # 4. Notify Unity via telemetry
        self.node.publish_to_unity({
            "type": "mission",
            "id": self.domain_id,
            "mission_id": "return_to_base",
            "status": "started"
        })

        self._log("info", "Return-to-Base mode engaged.")

    def _set_manual(self, enabled: bool):
        self.manual_mode = enabled
        self._log("info", f"{'Entered' if enabled else 'Exited'} MANUAL mode")

        if not enabled:
            try:
                if hasattr(self.node.dispatcher, "vel_handler"):
                    self.node.dispatcher.vel_handler.stop()
                    self._log("info", "Stopped velocity streaming (manual mode off).")
            except Exception as e:
                self._log("error", f"Failed to stop velocity stream: {e}")

    def _handle_pause(self):
        """Pause the currently active mission and push it onto the paused stack."""
        if not self.active_mission:
            self._log("warn", "Pause command received but no active mission.")
            return

        paused = self.active_mission.pause(current_index=self.active_mission.current_wp_index)
        self.paused_missions.append(paused)

        self._log("info", f"Paused mission '{paused.mission_id}' at waypoint {paused.current_wp_index}.")
        self.node.publish_to_unity({
            "type": "mission",
            "id": self.domain_id,
            "mission_id": paused.mission_id,
            "status": "paused"
        })

        # Clear the active mission
        self.active_mission = None

    def _handle_resume(self):
        """Resume the most recently paused mission."""
        if not self.paused_missions:
            self._log("warn", "Resume command received but no paused missions available.")
            return

        resumed = self.paused_missions.pop()
        self._log("info", f"Resuming paused mission '{resumed.mission_id}' from waypoint {resumed.current_wp_index}.")

        # Restore mission progress and mark as active
        resumed.waypoints = resumed.waypoints[resumed.start_index:]
        resumed.current_wp_index = 0
        self.active_mission = resumed

        # Notify Unity
        self.node.publish_to_unity({
            "type": "mission",
            "id": self.domain_id,
            "mission_id": resumed.mission_id,
            "status": "resumed"
        })

    def _handle_delete_mission(self, data: dict):
        """
        Deletes a mission by mission_id from active, paused, or queued sets.
        If it's the currently active mission, it is aborted and cleared.
        """
        mission_id = data.get("mission_id")
        if not mission_id:
            self._log("warn", "delete_mission called without mission_id.")
            return

        removed_from = None

        # 1) Check active mission
        if self.active_mission and self.active_mission.mission_id == mission_id:
            self._log("info", f"Deleting active mission '{mission_id}'. Aborting mission.")
            self.active_mission = None
            removed_from = "active"

        # 2) Check paused stack
        else:
            before = len(self.paused_missions)
            self.paused_missions = [m for m in self.paused_missions if m.mission_id != mission_id]
            if len(self.paused_missions) < before:
                self._log("info", f"Deleted mission '{mission_id}' from paused stack.")
                removed_from = "paused"

        # 3) Check mission queue (heap)
        if not removed_from:
            old_queue_len = len(self.mission_queue)
            new_queue = []
            for item in self.mission_queue:
                if item.mission_id != mission_id:
                    new_queue.append(item)
            if len(new_queue) < old_queue_len:
                self.mission_queue = new_queue
                self._log("info", f"Deleted mission '{mission_id}' from queue.")
                removed_from = "queue"

        if not removed_from:
            self._log("warn", f"Mission '{mission_id}' not found in active, paused, or queue.")
        else:
            # Notify Unity
            self.node.publish_to_unity({
                "type": "mission",
                "id": self.domain_id,
                "mission_id": mission_id,
                "status": "deleted"
            })
            
    """
    System wide code
    """

    def publish_from_unity(self, data: dict):

        clean_data = copy.deepcopy(data)

        probe_meta = {
            "_t_received": time.perf_counter(),
            "_id": data.get("command_id", str(uuid.uuid4()))
        }
        self._log("debug", f"LatencyProbe | Received {data.get('command', 'N/A')} id={probe_meta['_id']} at {probe_meta['_t_received']:.6f}")

        self._log("info", f"Received message from Unity: {data}")

        msg_type = clean_data.get("type")
        cmd      = clean_data.get("command")

        handler = self._msg_handlers.get((msg_type, cmd)) \
            or self._msg_handlers.get((msg_type, None))

        if handler:
            handler(clean_data)
        else:
            self._log("warn", f"Unknown message type or structure: {data}")

    def _dispatch_queued_commands(self):
        if not self.command_queue:
            return

        self._log("info", f"Dispatching {len(self.command_queue)} queued command(s)...")
        while self.command_queue:
            cmd = self.command_queue.popleft()
            self.node.publish_from_unity(json.dumps(cmd))
            self._log("info", f"Executed queued command: {cmd}")

    def shutdown(self):
        self._log("info", "Shutting down DroneInstance...")
        if self.telemetry_manager:
            self.telemetry_manager.shutdown()

    def _log(self, level: str, msg: str):
        tag = f"[DroneInstance {self.domain_id}]"
        line = f"{tag} {msg}"
        
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(line)
        else:
            print(line)
        
        try:
            with open(self.cli_log_file, "a") as f:
                f.write(line + "\n")
        except Exception as e:
            # fallback, don't let logging crash the app
            print(f"{tag} Failed to write log file: {e}")
