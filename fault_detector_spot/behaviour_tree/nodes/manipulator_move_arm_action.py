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


class ManipulatorMoveArmAction(py_trees.behaviour.Behaviour):
    """
    Executes a Spot arm movement to the blackboard's goal_tag_pose via RobotCommand action.
    """

    def __init__(self, name: str = "ManipulatorMoveArmAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.robot_command_client: ActionClientWrapper = None
        self.tf_listener: TFListenerWrapper = None
        self.odom_frame_name: str = None
        self.grav_body_frame: str = None
        self.sent = False
        self.initialized = False
        self.node = None

    def setup(self, **kwargs):
        # Just store the node for later use
        self.node = kwargs.get('node') or ros_scope.node()
        if self.node is None:
            raise RuntimeError("No ROS node provided to ManipulatorMoveArmAction")

        # Register blackboard key
        self.blackboard.register_key(key="goal_tag_pose", access=py_trees.common.Access.READ)

    def initialize(self):
        """Lazy initialization of ROS clients"""
        try:
            # Set up logger
            self.logger = self.node.get_logger()

            # Action client on ~/robot_command
            robot_ns = ""  # fill in if using a namespace
            self.robot_command_client = ActionClientWrapper(
                RobotCommand,
                namespace_with(robot_ns, "robot_command"),
                self.node
            )

            # TF listener to get transforms
            self.tf_listener = TFListenerWrapper(self.node)
            self.odom_frame_name = namespace_with(robot_ns, ros_scope.ODOM_FRAME_NAME)
            self.grav_body_frame = namespace_with(robot_ns, ros_scope.GRAV_ALIGNED_BODY_FRAME_NAME)

            self.initialized = True
            return True
        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"Failed to initialize: {e}")
            return False

    def update(self) -> py_trees.common.Status:
        # Lazy initialization on first update
        if not self.initialized:
            if not self.initialize():
                self.feedback_message = "Failed to initialize action clients"
                return py_trees.common.Status.RUNNING  # Keep trying

        # Check we have a PoseStamped on the blackboard
        if not self.blackboard.exists("goal_tag_pose") or self.blackboard.goal_tag_pose is None:
            self.feedback_message = "No goal_tag_pose on blackboard"
            return py_trees.common.Status.FAILURE

        if not self.sent:
            try:
                # Transform the PoseStamped into Spot's odom frame
                ps: PoseStamped = self.blackboard.goal_tag_pose
                # Build a flat-body SE3Pose
                hand = geometry_pb2.SE3Pose(
                    position=geometry_pb2.Vec3(
                        x=ps.pose.position.x,
                        y=ps.pose.position.y,
                        z=ps.pose.position.z
                    ),
                    rotation=geometry_pb2.Quaternion(
                        w=ps.pose.orientation.w,
                        x=ps.pose.orientation.x,
                        y=ps.pose.orientation.y,
                        z=ps.pose.orientation.z
                    )
                )
                # Lookup odom_T_flat_body, convert to math_helpers.SE3Pose
                tf = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_body_frame)
                odom_flat = math_helpers.SE3Pose(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                    math_helpers.Quat(
                        tf.transform.rotation.w,
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                    )
                )
                odom_goal = odom_flat * math_helpers.SE3Pose.from_obj(hand)

                # Create the RobotCommand goal
                arm_cmd = RobotCommandBuilder.arm_pose_command(
                    odom_goal.x,
                    odom_goal.y,
                    odom_goal.z,
                    odom_goal.rot.w,
                    odom_goal.rot.x,
                    odom_goal.rot.y,
                    odom_goal.rot.z,
                    self.odom_frame_name,
                    2.0  # seconds
                )
                goal_msg = RobotCommand.Goal()
                convert(arm_cmd, goal_msg.command)

                # Send and wait
                self.logger.info(f"Sending Spot arm goal to x={ps.pose.position.x:.2f}, "
                                 f"y={ps.pose.position.y:.2f}, z={ps.pose.position.z:.2f}")
                result = self.robot_command_client.send_goal_and_wait("move_to_goal", goal_msg)
                if result.status != result.STATUS_SUCCEEDED:
                    self.feedback_message = f"Spot arm action failed: {result.status}"
                    return py_trees.common.Status.FAILURE

                self.sent = True
                self.feedback_message = "Spot arm moved successfully"
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"Error executing arm command: {e}"
                return py_trees.common.Status.FAILURE

        # If we've already sent a command, report success
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        # Reset sent flag if we're being invalidated
        if new_status == py_trees.common.Status.INVALID:
            self.sent = False