# set_current_mission_command.py

import rospy
from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import WaypointSetCurrent

class SetCurrentMissionCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            wp_index = int(data.get("index", 0))

            rospy.wait_for_service('/mavros/mission/set_current')
            set_current_srv = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)

            response = set_current_srv(wp_seq=wp_index)

            if response.success:
                self.logger.info(f"[MISSION] Set current waypoint to index {wp_index}.")
            else:
                self.logger.warn(f"[MISSION] Failed to set waypoint {wp_index}.")

        except Exception as e:
            self.logger.error(f"[MISSION] Exception in setting current waypoint: {e}")
