from bosdyn.client.robot_command import RobotCommandBuilder

from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
from fault_detector_spot.behaviour_tree.nodes.utility.move_command_action import MoveCommandAction
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with


class ManipulatorMoveArmAction(MoveCommandAction):
    """
    Executes a Spot arm movement to the blackboard's goal_tag_pose via RobotCommand action.
    Uses ActionClientBehaviour for all lifecycle phases, customizing only client init and goal build.
    """

    def __init__(self, name="ManipulatorMoveArmAction", robot_name="", duration=3.0):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def _build_goal(self) -> RobotCommand.Goal:
        # Fetch and validate desired pose
        if not isinstance(self.blackboard.last_command, MoveCommand):
            raise RuntimeError(
                f"Expected ManipulatorMoveCommand on blackboard.last_command, got {type(self.blackboard.last_command).__name__}")

        target: PoseStamped = self.blackboard.last_command.compute_goal_pose(self.tf_listener)
        cmd = RobotCommandBuilder.arm_pose_command(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
            target.pose.orientation.w,
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            namespace_with(self.robot_name, target.header.frame_id),
            self.duration
        )
        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal