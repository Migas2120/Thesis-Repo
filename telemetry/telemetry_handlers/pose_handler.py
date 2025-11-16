import rospy
from geometry_msgs.msg import PoseStamped

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.handler_base import TelemetryHandlerBase

class PoseHandler(TelemetryHandlerBase):
    def __init__(self, logger=None):
        super().__init__(logger)
        self._latest = None
        self._offset = None

        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._callback)
        self.logger.info("[PoseHandler] Subscribed to /mavros/local_position/pose")

    def _callback(self, msg: PoseStamped):
        if self._offset is None:
            # First message: take its position as offset
            p = msg.pose.position
            self._offset = {"x": p.x, "y": p.y, "z": p.z}
            self.logger.info(f"[PoseHandler] Offset initialized: {self._offset}")

        self._latest = msg

    def get_serialized(self):
        if not self._latest:
            return None
        pos = self._latest.pose.position
        ori = self._latest.pose.orientation
        return {
            "pose": {
                "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                "orientation": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w}
            }
        }

    def get_pose(self):
        if self._latest:
            p = self._latest.pose.position
            return {"x": p.x, "y": p.y, "z": p.z}
        return None

    def get_offset(self):
        """Return the initial offset (first reading), or None if not initialized yet."""
        return self._offset
