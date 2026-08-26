from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from py_trees.common import Access
from spot_msgs.action import RobotCommand
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    RobotCommandActionBehaviour,
)
from py_trees.blackboard import Blackboard


class CloseGripperAction(RobotCommandActionBehaviour):
    def __init__(
        self,
        name="CloseGripperAction",
        robot_name="",
        robot_command_resources=None,
    ):
        super().__init__(name, robot_name, robot_command_resources)

    def _build_goal(self) -> RobotCommand.Goal:
        cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal
