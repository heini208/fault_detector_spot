from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import tf_transformations as tf
from fault_detector_spot.behaviour_tree.commands.move_base_relative_command import MoveBaseRelativeCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from tf2_geometry_msgs import do_transform_pose_stamped


class BaseMoveRelativeAction(ActionClientBehaviour):
    """
    Moves the robot base to the tag with the given offset, using low speed limits.
    """

    def __init__(self, name="BaseMoveToTagAction", robot_name=""):
        super().__init__(name)
        self.robot_name = robot_name
        self.blackboard = self.attach_blackboard_client()
        self.tf_listener: TFListenerWrapper = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        self.tf_listener = TFListenerWrapper(self.node)

        # Blackboard keys: read visible tags, read/write last command
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def _init_client(self):
        action_ns = namespace_with(self.robot_name, "robot_command")
        self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
        self.initialized = True
        return True

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(self.blackboard.last_command, MoveBaseRelativeCommand):
            raise RuntimeError("Expected MoveBaseRelativeCommand on blackboard.last_command")

        cmd: MoveBaseRelativeCommand = self.blackboard.last_command
        target: PoseStamped = cmd.compute_goal_pose(self.tf_listener)

        # Build low-speed mobility params
        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(0.1, 0.1, 0.2).to_proto(),  # linear x, y and angular z
                min_vel=math_helpers.SE2Velocity(-0.1, -0.1, -0.2).to_proto(),
            )
        )

        if target.header.frame_id != ODOM_FRAME_NAME:
            tf_to_odom = self.tf_listener.lookup_a_tform_b(
                ODOM_FRAME_NAME, target.header.frame_id, timeout_sec=2
            )
            target = do_transform_pose_stamped(target, tf_to_odom)
            target.header.frame_id = ODOM_FRAME_NAME

        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=target.pose.position.x,
            goal_y=target.pose.position.y,
            frame_name=namespace_with(self.robot_name, target.header.frame_id),
            goal_heading=tf.euler_from_quaternion([
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ])[2],
            params=low_speed_params
        )

        # Step 5: wrap in RobotCommand.Goal
        goal = RobotCommand.Goal()
        from bosdyn_msgs.conversions import convert
        convert(cmd, goal.command)
        return goal