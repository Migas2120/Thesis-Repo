import rospy
from sensor_msgs.msg import NavSatFix
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.handler_base import TelemetryHandlerBase

class GPSHandler(TelemetryHandlerBase):
    def __init__(self, logger=None):
        super().__init__(logger)
        self._latest = None

        self.sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._callback)
        self.logger.info("[GPSHandler] Subscribed to /mavros/global_position/global")

    def _callback(self, msg):
        self._latest = msg

    def get_serialized(self):
        if not self._latest:
            return None
        return {
            "gps": {
                "latitude": self._latest.latitude,
                "longitude": self._latest.longitude,
                "altitude": self._latest.altitude,
                "status": self._latest.status.status
            }
        }
