import py_trees
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction


class StandUpActionSimple(SimpleSpotAction):
    """
    Sends a Spot stand command via RobotCommand action asynchronously.
    Returns RUNNING while in flight, SUCCESS on completion, FAILURE on error.
    """

    def __init__(self, name: str = "StandUpAction", robot_name: str = ""):
        super().__init__(name, robot_name)

    def _build_goal(self) -> RobotCommand.Goal:
        # Construct the synchro stand command goal
        stand_cmd = RobotCommandBuilder.synchro_stand_command()
        goal = RobotCommand.Goal()
        convert(stand_cmd, goal.command)
        return goal
