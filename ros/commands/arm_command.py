import rospy

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import CommandBool

class ArmCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            arm = bool(data.get("arm", True))  # default to True
            rospy.wait_for_service('/mavros/cmd/arming')
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(arm)

            if response.success:
                self.logger.info(f"[ARM] Success: armed={arm}")
            else:
                self.logger.warn(f"[ARM] Failed to arm/disarm")
        except Exception as e:
            self.logger.error(f"[ARM] Exception: {e}")
