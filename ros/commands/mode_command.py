import rospy
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import SetMode

class ModeCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            # Mode string required, e.g., "GUIDED", "AUTO", "STABILIZE"
            mode = data.get("mode", "").upper()
            if not mode:
                self.logger.error("[MODE] No 'mode' field provided in data.")
                return

            rospy.wait_for_service('/mavros/set_mode')
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_srv(custom_mode=mode, base_mode=0)  # base_mode often ignored in PX4/ArduPilot

            if response.mode_sent:
                self.logger.info(f"[MODE] Successfully set mode to: {mode}")
            else:
                self.logger.warn(f"[MODE] Mode change to '{mode}' was rejected")
        except Exception as e:
            self.logger.error(f"[MODE] Exception: {e}")
