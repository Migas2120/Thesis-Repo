# push_mission_command.py

import rospy
from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

class PushMissionCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            # Extract waypoint list from the received data
            waypoints_data = data.get("waypoints", [])
            if not waypoints_data:
                self.logger.error("[MISSION] No waypoints provided.")
                return

            # Convert input dicts to mavros Waypoint messages
            waypoints = []
            for idx, wp in enumerate(waypoints_data):
                waypoint = Waypoint()
                waypoint.frame = wp.get("frame", 3)  # Default: MAV_FRAME_GLOBAL_REL_ALT
                waypoint.command = wp.get("command", 16)  # Default: NAV_WAYPOINT
                waypoint.is_current = wp.get("is_current", idx == 0)
                waypoint.autocontinue = wp.get("autocontinue", True)
                waypoint.param1 = wp.get("param1", 0)
                waypoint.param2 = wp.get("param2", 0)
                waypoint.param3 = wp.get("param3", 0)
                waypoint.param4 = wp.get("param4", 0)
                waypoint.x_lat = wp.get("x_lat", 0)
                waypoint.y_long = wp.get("y_long", 0)
                waypoint.z_alt = wp.get("z_alt", 2.0)
                waypoints.append(waypoint)

            rospy.wait_for_service('/mavros/mission/push')
            push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

            # Send waypoints to autopilot
            response = push_srv(start_index=0, waypoints=waypoints)

            if response.success:
                self.logger.info(f"[MISSION] Successfully pushed {response.wp_transfered} waypoints.")
            else:
                self.logger.warn("[MISSION] Waypoint push failed.")

        except Exception as e:
            self.logger.error(f"[MISSION] Exception: {e}")
