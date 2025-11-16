import rospy
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import CommandTOL

class TakeoffCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            # Parse altitude and optional fields
            altitude = float(data.get("altitude", 2.0))
            latitude = float(data.get("lat", 0.0))
            longitude = float(data.get("lon", 0.0))
            min_pitch = float(data.get("min_pitch", 0.0))
            yaw = float(data.get("yaw", 0.0))

            rospy.wait_for_service('/mavros/cmd/takeoff')
            takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

            response = takeoff_srv(
                min_pitch=min_pitch,
                yaw=yaw,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude
            )

            if response.success:
                self.logger.info(f"[TAKEOFF] Takeoff initiated to {altitude}m.")
            else:
                self.logger.warn("[TAKEOFF] Takeoff failed.")
        except Exception as e:
            self.logger.error(f"[TAKEOFF] Exception: {e}")
