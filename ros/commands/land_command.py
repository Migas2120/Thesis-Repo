import rospy
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from mavros_msgs.srv import CommandTOL
from ros.commands.base_command import BaseCommand

class LandCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            rospy.wait_for_service('/mavros/cmd/land', timeout=5)
            land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

            # Call directly with keyword args (ROS 1 style)
            res = land_srv(
                min_pitch=0.0,
                yaw=float(data.get("yaw", 0.0)),
                latitude=float(data.get("latitude", 0.0)),
                longitude=float(data.get("longitude", 0.0)),
                altitude=float(data.get("altitude", 0.0))
            )

            if res.success:
                self.logger.info("[LAND] Land command accepted.")
            else:
                self.logger.error(f"[LAND] Command failed with result: {res.result}")

        except (rospy.ServiceException, rospy.ROSException) as e:
            self.logger.error(f"[LAND] Service call failed: {e}")
