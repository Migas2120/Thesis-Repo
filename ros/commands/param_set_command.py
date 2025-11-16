# param_set_command.py

import rospy
from ros.commands.base_command import BaseCommand
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue

class ParamSetCommand(BaseCommand):
    def execute(self, ros, data: dict):
        try:
            param_id = data.get("param_id")
            param_value = data.get("value")

            if param_id is None or param_value is None:
                self.logger.error("[PARAM] param_id or value missing.")
                return

            rospy.wait_for_service('/mavros/param/set')
            param_set_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)

            # Auto-detect type: int or float
            value = ParamValue()
            if isinstance(param_value, int):
                value.integer = param_value
                value.real = float(param_value)
            elif isinstance(param_value, float):
                value.real = param_value
                value.integer = int(param_value)
            else:
                self.logger.error("[PARAM] Unsupported param type.")
                return

            response = param_set_srv(param_id=param_id, value=value)

            if response.success:
                self.logger.info(f"[PARAM] Set {param_id} = {param_value}")
            else:
                self.logger.warn(f"[PARAM] Failed to set {param_id}")

        except Exception as e:
            self.logger.error(f"[PARAM] Exception: {e}")
