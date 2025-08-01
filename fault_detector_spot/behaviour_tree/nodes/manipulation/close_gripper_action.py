from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from py_trees.common import Access
from spot_msgs.action import RobotCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction
from py_trees.blackboard import Blackboard

class CloseGripperAction(SimpleSpotAction):
    def __init__(self, name="CloseGripperAction", robot_name=""):
        super().__init__(name, robot_name)

    def _build_goal(self) -> RobotCommand.Goal:
        cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal
