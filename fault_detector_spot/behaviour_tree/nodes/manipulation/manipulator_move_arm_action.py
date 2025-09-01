from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from fault_detector_spot.behaviour_tree.command_ids import CommandID
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.manipulator_move_command import ManipulatorMoveCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access
from spot_msgs.action import RobotCommand


class ManipulatorMoveArmAction(ActionClientBehaviour):
    """
    Executes a Spot arm movement to the blackboard's goal_tag_pose via RobotCommand action.
    Uses ActionClientBehaviour for all lifecycle phases, customizing only client init and goal build.
    """

    def __init__(self,
                 name: str = "ManipulatorMoveArmAction",
                 robot_name: str = "",
                 duration: float = 3.0):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration
        self.tf_listener: TFListenerWrapper = None
        # register blackboard keys
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def _init_client(self) -> bool:
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
            # Initialize TF listener
            self.tf_listener = TFListenerWrapper(self.node)
            self.initialized = True
            return True
        except Exception as e:
            self.logger.error(f"[{self.name}] init failed: {e}")
            return False

    def _build_goal(self) -> RobotCommand.Goal:
        # Fetch and validate desired pose
        if not isinstance(self.blackboard.last_command, ManipulatorMoveCommand):
            raise RuntimeError(
                f"Expected ManipulatorMoveCommand on blackboard.last_command, got {type(self.blackboard.last_command).__name__}")

        target: PoseStamped = self.blackboard.last_command.goal_pose
        cmd = RobotCommandBuilder.arm_pose_command(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
            target.pose.orientation.w,
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
            self.duration
        )
        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal
