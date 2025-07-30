from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from py_trees.common import Access
from spot_msgs.action import RobotCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction
from py_trees.blackboard import Blackboard

class ToggleGripperAction(SimpleSpotAction):
    def __init__(self, name="ToggleGripperAction", robot_name=""):
        super().__init__(name, robot_name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="gripper_open", access=Access.WRITE)
        self.blackboard.gripper_open = False

    def _build_goal(self) -> RobotCommand.Goal:
        open_fraction = 0.0 if self.blackboard.gripper_open else 1.0
        cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(open_fraction)
        self.blackboard.gripper_open = not self.blackboard.gripper_open

        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal
