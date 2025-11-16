import os
import sys
import rospy
from mavros_msgs.msg import State

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.handler_base import TelemetryHandlerBase  # adjust if needed

class StatusHandler(TelemetryHandlerBase):
    def __init__(self, logger=None):
        super().__init__(logger)
        self._latest = None

        self.sub = rospy.Subscriber('/mavros/state', State, self._callback)
        self.logger.info("[StatusHandler] Subscribed to /mavros/state")

    def _callback(self, msg):
        self._latest = msg
        self.logger.debug(f"[StatusHandler] Received status: {msg}")

    def get_serialized(self):
        if not self._latest:
            return None
        return {
            "status": {
                "armed": self._latest.armed,
                "connected": self._latest.connected,
                "mode": self._latest.mode,
                "guided": self._latest.guided,
                "manual_input": self._latest.manual_input,
                "system_status": self._latest.system_status,
            }
        }
