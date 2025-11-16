#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import PoseStamped

import sys
import os
# Append ROS 1 package root directory to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from command_dispatcher.command_dispatcher import CommandDispatcher


class NodeWrapper:
    def __init__(self, logger=None, drone_id=11, unsafe: bool = False):
        self.logger = logger or rospy.loginfo
        self.drone_id = drone_id

        if not rospy.core.is_initialized():
            rospy.init_node("node_wrapper", anonymous=True)

        # Init dispatcher for command-based interaction
        self.dispatcher = CommandDispatcher(ros_handler=self, logger=self.logger)

        # Init publisher for mission-based position control
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self._log("info", f"Initialized NodeWrapper for drone {self.drone_id}")

    def set_tcp_client(self, client):
        self.tcp_client = client
        self._log("debug", "TCP client set in NodeWrapper")

    def publish_from_unity(self, json_data: str):
        try:
            data = json.loads(json_data)
            self._log("info", f"Dispatching message: {data}")
            self.dispatcher.dispatch(data)
        except Exception as e:
            self._log("error", f"Failed to parse JSON or dispatch command: {e}")

    def publish_to_unity(self, message: dict) -> None:
        """
        Sends a JSON message back to Unity through the linked TCP client.
        """
        if not hasattr(self, "tcp_client") or self.tcp_client is None:
            self._log("warn", "No TCP client linked; dropping telemetry.")
            return
        try:
            payload = json.dumps(message)
            self.tcp_client.send(payload)
            self._log("debug", f"[Unity ← NodeWrapper] {payload}")
        except Exception as e:
            self._log("error", f"Failed to send telemetry to Unity: {e}")

    
    def _log(self, level: str, msg: str):
        tag = f"[NodeWrapper {self.drone_id}]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
