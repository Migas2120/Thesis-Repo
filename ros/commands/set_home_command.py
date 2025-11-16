# set_home_command.py

import rospy
from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import CommandHome

class SetHomeCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            use_current = bool(data.get("current_gps", True))
            latitude = float(data.get("lat", 0.0))
            longitude = float(data.get("lon", 0.0))
            altitude = float(data.get("alt", 0.0))

            rospy.wait_for_service('/mavros/cmd/set_home')
            set_home_srv = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

            response = set_home_srv(
                current_gps=use_current,
                yaw=0.0,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude
            )

            if response.success:
                self.logger.info(f"[HOME] Home position set. Current GPS: {use_current}")
            else:
                self.logger.warn("[HOME] Failed to set home position.")

        except Exception as e:
            self.logger.error(f"[HOME] Exception setting home: {e}")
