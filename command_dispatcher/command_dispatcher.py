# ros1_server/ros/command_dispatcher.py

import logging

import sys
import os
# Append ROS 1 package root directory to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

# Import your ROS 1 command handlers here
from ros.commands.arm_command import ArmCommand
from ros.commands.mode_command import ModeCommand
from ros.commands.TakeoffCommand import TakeoffCommand
from ros.commands.land_command import LandCommand
from ros.commands.position_command import PositionCommand
from ros.commands.push_mission_command import PushMissionCommand
from ros.commands.clear_mission_command import ClearMissionCommand
from ros.commands.set_current_mission_command import SetCurrentMissionCommand
from ros.commands.set_home_command import SetHomeCommand
from ros.commands.param_set_command import ParamSetCommand
from ros.commands.pos_global_command import GlobalPositionCommand
from ros.commands.vel_command import VelCommand
class CommandDispatcher:
    """
    Routes a 'command' string from a Unity payload to its corresponding
    ROS 1 command handler (e.g., arm, takeoff, mode).
    """

    def __init__(self, ros_handler, logger=None):
        self.ros = ros_handler
        self.logger = logger or ros_handler.get_logger() if hasattr(ros_handler, "get_logger") else logging.getLogger(__name__)

        self.vel_handler = VelCommand(logger=self.logger)

        # Map of command strings to command handler instances
        self.command_map = {
            "arm": ArmCommand(logger=self.logger),
            "mode": ModeCommand(logger=self.logger),
            "takeoff": TakeoffCommand(logger=self.logger),
            "land": LandCommand(logger=self.logger),
            "pos": PositionCommand(logger=self.logger),
            "push_mission": PushMissionCommand(logger=self.logger),
            "clear_mission": ClearMissionCommand(logger=self.logger),
            "set_current_mission": SetCurrentMissionCommand(logger=self.logger),
            "set_home": SetHomeCommand(logger=self.logger),
            "param_set": ParamSetCommand(logger=self.logger),
            "pos_global": GlobalPositionCommand(logger=self.logger),
            "vel": self.vel_handler,
        }

        self.logger.info(f"[CommandDispatcher] Initialized with commands: {list(self.command_map.keys())}")

    def dispatch(self, data: dict):
        command = data.get("command", "")
        print(f"[Command Dispatcher] {command}")
        handler = self.command_map.get(command)

        if handler:
            try:

                if command == "takeoff":
                    # 1. Arm the drone
                    arm_handler = self.command_map.get("arm")
                    if arm_handler:
                        self.logger.info("[CommandDispatcher] Arming drone before takeoff...")
                        arm_handler.execute(self.ros, {"command": "arm", "arm": True})

                    # 2. Switch to GUIDED mode
                    mode_handler = self.command_map.get("mode")
                    if mode_handler:
                        self.logger.info("[CommandDispatcher] Switching mode to GUIDED before takeoff...")
                        mode_handler.execute(self.ros, {"command": "mode", "mode": "GUIDED"})

                print(f"[Command Dispatcher] Dispatching command '{command}' with data: {data}")
                handler.execute(self.ros, data)

                # Automatically start or stop continuous publishing for velocity
                if command == "vel" and hasattr(handler, "_stream_loop"):
                    # already handled inside execute(), no action here
                    pass
                elif command == "stop_vel" and hasattr(handler, "stop"):
                    handler.stop()

            except Exception as e:
                print(f"Error executing '{command}': {e}")
        else:
            print(f"Unknown command: '{command}'")
