# clear_mission_command.py

import rospy
from ros.commands.base_command import BaseCommand
from std_srvs.srv import Empty

class ClearMissionCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            rospy.wait_for_service('/mavros/mission/clear')
            clear_srv = rospy.ServiceProxy('/mavros/mission/clear', Empty)

            clear_srv()  # No request data
            self.logger.info("[MISSION] Successfully cleared all waypoints.")

        except Exception as e:
            self.logger.error(f"[MISSION] Failed to clear waypoints: {e}")
