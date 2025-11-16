# app_runner.py

import time
import threading
import json
import sys
import os
import datetime

# Add ROS package root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from tcp.server import TCPServer
from tcp.client import TCPClient
from drone_instance.drone_instance import DroneInstance


class AppRunner:
    """
    Lightweight ROS 1 runner for a single-drone companion computer.
    Accepts TCP input from Unity and forwards it to the drone's control interface.
    """

    def __init__(self, logger, unsafe: bool = False):
        self.logger = logger
        self.unsafe = unsafe
        self.tcp_server = None
        self.tcp_server_thread = None

        self.client: TCPClient | None = None

        self.drone = DroneInstance(domain_id=0, logger=self.logger, on_landing=self.shutdown, unsafe=self.unsafe)
        
        self._log("info", "AppRunner initialized with 1 DroneInstance (ID 0)")

    def start(self):
        self._log("info", "Starting AppRunner...")

        self.tcp_server = TCPServer(
            ros_node=self,
            logger=self.logger,
            host='0.0.0.0',
            port=65432
        )
        self.tcp_server_thread = threading.Thread(target=self.tcp_server.start, daemon=True)
        self.tcp_server_thread.start()

        self._log("info", "TCP Server is running. Waiting for Unity commands...")
        # Don’t start the outgoing client yet—it’ll be kicked off via a Unity message

    def tick(self):
        """
        Called every second from main loop. Delegates to drone tick.
        """
        self.drone.tick()

    def publish_from_unity(self, message_json):
        try:
            data = json.loads(message_json)
            self._log("debug", f"Received Unity message: {data}")
            # Forward straight to the drone logic
            self.drone.publish_from_unity(data)
        except Exception as e:
            self._log("error", f"Error processing Unity message: {e}")

    def publish_to_unity(self, message_dict):
        """Send a JSON message back to Unity, if the client exists."""
        if self.client:
            payload = json.dumps(message_dict)
            self.client.send(payload)
            self._log("debug", f"[AppRunner→Unity] {payload}")
        else:
            self._log("warn", "No Unity client; dropping message")

    def _init_client(self, host: str, port: int):
        """Create or re-create the TCPClient back to Unity."""
        if self.client:
            self._log("info", "Re-initializing existing TCP client...")
            self.client.stop()

        self._log("info", f"Initializing outgoing TCPClient to {host}:{port}")
        self.client = TCPClient(
            host=host,
            port=port,
            on_connected    = lambda: self._log("info", "Outgoing client connected"),
            on_disconnected = lambda: self._log("warn", "Outgoing client disconnected"),
            on_data         = lambda txt: self._log("debug", f"[Unity Drone] {txt}"),
            logger=self.logger
        )
        self.client.start()

        self.drone.node.set_tcp_client(self.client)

    def handle_unity_disconnect(self):
        self._log("warn", "Unity link lost → preserving active mission, clearing the rest, then queuing RTL→LAND")
        # 1) clear out any future missions
        self.drone.mission_queue.clear()

        # 2) queue up an RTL, then a LAND so that it happens immediately after the current mission finishes
        rtl_cmd = {
            "type":    "command",
            "command": "mode",
            "mode":    "RTL",
            "id":      self.drone.domain_id
        }
        self.drone.command_queue.append(rtl_cmd)
        self.drone.returning_to_base = True
        
    def shutdown(self):
        self._log("info", "Shutting down TCP and drone...")
        if self.tcp_server:
            self.tcp_server.stop()
        self.drone.shutdown()
        if self.client:
            self.client.stop()  
        
        exit(0)

    def _log(self, level: str, msg: str):
        tag = "[AppRunner]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
