from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from spot_msgs.action import RobotCommand
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import SimpleSpotAction


class ReadyArmActionSimple(SimpleSpotAction):
    def __init__(
        self,
        name="ReadyArmAction",
        robot_name="",
        robot_command_resources=None,
    ):
        super().__init__(name, robot_name, robot_command_resources)

    def _build_goal(self) -> RobotCommand.Goal:
        ready_cmd = RobotCommandBuilder.arm_ready_command()
        goal = RobotCommand.Goal()
        convert(ready_cmd, goal.command)
        return goal
