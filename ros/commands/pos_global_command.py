import os, sys, rospy, time
from mavros_msgs.msg import GlobalPositionTarget
from std_msgs.msg import Header

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand

class GlobalPositionCommand(BaseCommand):
    """
    Publish GPS (lat, lon, alt) setpoints via /mavros/setpoint_raw/global.

    Expected payload (floats):
      { "lat": 38.736946, "lon": -9.142685, "alt": 20.0,
        "yaw": 1.57,                     # optional (rad)
        "frame": "REL_ALT" }             # optional: REL_ALT | TERRAIN | AMSL
    """
    def __init__(self, logger=None):
        super().__init__(logger)
        self.pub = rospy.Publisher('/mavros/setpoint_raw/global',
                                   GlobalPositionTarget, queue_size=10)

    def execute(self, ros, data: dict):
        try:
            lat = float(data["lat"])
            lon = float(data["lon"])
            alt = float(data.get("alt", 10.0))
            yaw = data.get("yaw", None)
            frame = str(data.get("frame", "REL_ALT")).upper()

            msg = GlobalPositionTarget()
            msg.header = Header(stamp=rospy.Time.now(), frame_id="map")

            # Coordinate frame
            if frame == "TERRAIN":
                msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_TERRAIN_ALT
            elif frame == "AMSL":
                msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT  # AMSL
            else:
                msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT  # default

            # Type mask: position only; ignore vel/accel + yaw_rate
            IGN = GlobalPositionTarget
            type_mask = (IGN.IGNORE_VX | IGN.IGNORE_VY | IGN.IGNORE_VZ |
                         IGN.IGNORE_AFX | IGN.IGNORE_AFY | IGN.IGNORE_AFZ |
                         IGN.IGNORE_YAW_RATE)

            if yaw is None:
                type_mask |= IGN.IGNORE_YAW
            else:
                msg.yaw = float(yaw)

            msg.type_mask = type_mask

            # Position (LLA)
            msg.latitude  = lat
            msg.longitude = lon
            msg.altitude  = float(alt)  # meters; REL_ALT=above home, TERRAIN=AGL, AMSL=MSL

            self.pub.publish(msg)
            self.logger.info(f"[POS_GLOBAL] Published LLA target: "
                             f"lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} ({frame})")

        except KeyError as e:
            self.logger.error(f"[POS_GLOBAL] Missing key: {e}")
        except Exception as e:
            self.logger.error(f"[POS_GLOBAL] Exception: {e}")
