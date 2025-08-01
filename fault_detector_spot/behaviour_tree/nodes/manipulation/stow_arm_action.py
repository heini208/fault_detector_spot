# fault_detector_spot/behaviour_tree/nodes/stow_arm_action.py

import py_trees
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction


class StowArmActionSimple(SimpleSpotAction):
    """
    Sends a Spot arm_stow command via RobotCommand action asynchronously.
    Returns RUNNING while in flight, SUCCESS on completion, FAILURE on error.
    """

    def __init__(self, name: str = "StowArmAction", robot_name: str = ""):
        super().__init__(name, robot_name)

    def _build_goal(self) -> RobotCommand.Goal:
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        goal = RobotCommand.Goal()
        convert(stow_cmd, goal.command)
        return goal
