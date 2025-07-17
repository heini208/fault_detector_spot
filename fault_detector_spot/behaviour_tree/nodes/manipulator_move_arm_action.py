import py_trees
from geometry_msgs.msg import PoseStamped
import rclpy
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME



class ManipulatorMoveArmAction(py_trees.behaviour.Behaviour):
    """
    Executes a Spot arm movement to the blackboard's goal_tag_pose via RobotCommand action,
    then clears the goal variables from the blackboard.
    """

    def __init__(self, name: str = "ManipulatorMoveArmAction", robot_name: str = ""):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.robot_command_client: ActionClientWrapper = None
        self.tf_listener: TFListenerWrapper = None
        self.robot_name = robot_name
        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.send_goal_future = None
        self.goal_handle       = None
        self.get_result_future = None
        self.sent              = False
        self.initialized = False
        self.node = None

    def setup(self, **kwargs):
        # Store the ROS node
        self.node = kwargs.get('node') or ros_scope.node()
        if self.node is None:
            raise RuntimeError("No ROS node provided to ManipulatorMoveArmAction")

        # Allow reading and deleting the pose & id on the blackboard
        self.blackboard.register_key(key="goal_tag_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_tag_id",   access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="last_command_id", access=py_trees.common.Access.READ)

    def initialize(self):
        """Lazy initialization of action client and TF listener."""
        try:
            self.logger = self.node.get_logger()

            self.robot_command_client = ActionClientWrapper(
                RobotCommand,
                namespace_with(self.robot_name, "robot_command"),
                self.node
            )

            self.tf_listener = TFListenerWrapper(self.node)
            self.initialized = True
            return True
        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"Initialization failed: {e}")
            return False

    def update(self) -> py_trees.common.Status:
        # First-call lazy setup
        if not self.initialized:
            if not self.initialize():
                self.feedback_message = "Waiting to initialize action client..."
                return py_trees.common.Status.RUNNING

        if self.sent:
            return py_trees.common.Status.SUCCESS

        if self.blackboard.last_command_id != "move_to_tag":
            self._cancel_inflight()
            self.feedback_message = "Cancelled by new command"
            return py_trees.common.Status.FAILURE

        if not self.check_goal_exists():
            self.feedback_message = "No goal_tag_pose on blackboard"
            return py_trees.common.Status.FAILURE

        try:
            if self.send_goal_future is None:
                self.send_async_goal()
                return py_trees.common.Status.RUNNING

            if self.goal_handle is None and self.send_goal_future.done():
                return self.check_goal_accepted()

            if not self.get_result_future.done():
                return py_trees.common.Status.RUNNING

            return self.check_action_result()

        except Exception as e:
            self.feedback_message = f"Error during arm move: {e}"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def send_async_goal(self):
            ps = self.blackboard.goal_tag_pose
            goal_msg = self.create_arm_command_as_message(ps, 2.0)
            self.send_goal_future = self.robot_command_client.send_goal_async(
                goal_msg
            )
            self.feedback_message = "Goal sent, waiting for acceptance"

    def check_goal_accepted(self):
        self.goal_handle = self.send_goal_future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "Goal rejected"
            return py_trees.common.Status.FAILURE
        self.get_result_future = self.goal_handle.get_result_async()
        # clear the BB now that the goal is safely with the server
        self.clear_blackboard()
        self.feedback_message = "Goal accepted; waiting for result"
        return py_trees.common.Status.RUNNING

    def check_action_result(self):
        result = self.get_result_future.result().result
        if result.success:  # or wrapper-specific status check
            self.feedback_message = "Arm move succeeded"
            self.sent = True
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Arm failed: {result}"
            return py_trees.common.Status.FAILURE

    def create_arm_command_as_message(self, ps, seconds=2):
        arm_command = RobotCommandBuilder.arm_pose_command(
                ps.pose.position.x,
                ps.pose.position.y,
                ps.pose.position.z,
                ps.pose.orientation.w,
                ps.pose.orientation.x,
                ps.pose.orientation.y,
                ps.pose.orientation.z,
                namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
                seconds,
            )
        action_goal = RobotCommand.Goal()
        convert(arm_command, action_goal.command)
        return action_goal


    def check_goal_exists(self):
        return self.blackboard.exists("goal_tag_pose") or self.blackboard.goal_tag_pose is not None

    def clear_blackboard(self):
        if self.blackboard.exists("goal_tag_pose"):
            self.blackboard.goal_tag_pose = None
        if self.blackboard.exists("goal_tag_id"):
            self.blackboard.goal_tag_id = None

    def _cancel_inflight(self):
        if self.goal_handle:
            self.robot_command_client._cancel_goal_async(self.goal_handle)
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self.sent = False

    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.robot_command_client._cancel_goal_async(self.goal_handle)
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self.sent = False
