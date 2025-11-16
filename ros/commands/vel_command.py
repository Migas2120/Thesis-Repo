import rospy
import sys
import os
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
from geometry_msgs.msg import TwistStamped

class VelCommand(BaseCommand):
    def __init__(self, logger=None):
        super().__init__(logger)
        self.pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.last_msg = None
        self._thread = None
        self._running = False
        self.logger.info("[VEL] Initialized VelCommand")


    def execute(self, ros, data: dict):
        try:
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x = float(data.get("x", 0.0))
            msg.twist.linear.y = float(data.get("y", 0.0))
            msg.twist.linear.z = float(data.get("z", 0.0))
            msg.twist.angular.z = float(data.get("yaw", 0.0))

            self.last_msg = msg

            if not self._running:
                print("[VEL] Starting stream thread…")
                self._running = True
                self._thread = threading.Thread(target=self._stream_loop, daemon=True)
                self._thread.start()
                print(f"[VEL] Thread started? {self._thread.is_alive()}")
            else:
                print("[VEL] Stream already running, updating last_msg only.")
                

        except Exception as e:
            print(f"[VEL] Exception in execute(): {e}")

    def _stream_loop(self):
        print("[VEL] Stream loop entered.")
        rate = rospy.Rate(10)
        while self._running and not rospy.is_shutdown():
            if self.last_msg:
                self.last_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.last_msg)
            rate.sleep()
        print("[VEL] Stream loop exited.")

    def stop(self):
        """Stop continuous streaming."""
        self._running = False
