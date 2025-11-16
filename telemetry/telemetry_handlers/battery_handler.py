import rospy
from sensor_msgs.msg import BatteryState
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.handler_base import TelemetryHandlerBase

class BatteryHandler(TelemetryHandlerBase):
    def __init__(self, logger=None):
        super().__init__(logger)
        self._latest = None

        self.sub = rospy.Subscriber('/mavros/battery', BatteryState, self._callback)
        self.logger.info("[BatteryHandler] Subscribed to /mavros/battery")

    def _callback(self, msg):
        self._latest = msg

    def get_serialized(self):
        if not self._latest:
            return None
        return {
            "battery": {
                "voltage": self._latest.voltage,
                "current": self._latest.current,
                "percentage": self._latest.percentage,
                "power_supply_status": self._latest.power_supply_status
            }
        }
