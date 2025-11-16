# pos_command.py
import os
import sys

from geometry_msgs.msg import PoseStamped

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
import rospy
import time

class PositionCommand(BaseCommand):
    def __init__(self, logger=None):
        super().__init__(logger)
        self.publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.last_publish_time = 0

    def execute(self, ros, data: dict):
        try:
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            z = float(data.get("z", 2.0))

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0  # Neutral orientation

            self.publisher.publish(pose)
            self.logger.info(f"[POSITION] Published target: ({x}, {y}, {z})")

        except Exception as e:
            self.logger.error(f"[POSITION] Exception: {e}")
