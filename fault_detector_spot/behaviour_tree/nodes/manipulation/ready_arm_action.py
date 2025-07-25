# fault_detector_spot/behaviour_tree/nodes/ready_arm_action.py

from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from spot_msgs.action import RobotCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction

class ReadyArmActionSimple(SimpleSpotAction):
    def __init__(self, name="ReadyArmAction", robot_name=""):
        super().__init__(name, robot_name)

    def _build_goal(self) -> RobotCommand.Goal:
        ready_cmd = RobotCommandBuilder.arm_ready_command()
        goal = RobotCommand.Goal()
        convert(ready_cmd, goal.command)
        return goal
