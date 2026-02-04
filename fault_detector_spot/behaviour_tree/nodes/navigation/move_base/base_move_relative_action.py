from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import tf_transformations as tf
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.base_move_relative_command import BaseMoveRelativeCommand
from fault_detector_spot.behaviour_tree.nodes.utility.move_command_action import MoveCommandAction
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
from tf2_geometry_msgs import do_transform_pose_stamped


class BaseMoveRelativeAction(MoveCommandAction):
    """
    Moves the robot base relative to a target using low speed limits.
    Inherits TF safety checks from MoveCommandAction.
    """

    def __init__(self, name="BaseMoveRelativeAction", robot_name=""):
        super().__init__(name)
        self.robot_name = robot_name
        # blackboard & tf_listener init handled in base/setup

    def setup(self, **kwargs):
        # Call base setup to initialize TF listener and blackboard
        super().setup(**kwargs)
        self.node = kwargs.get("node")

    def _init_client(self):
        action_ns = namespace_with(self.robot_name, "robot_command")
        self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
        self.initialized = True
        return True

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(self.blackboard.last_command, BaseMoveRelativeCommand):
            raise RuntimeError("Expected BaseMoveRelativeCommand on blackboard.last_command")

        # TF readiness is guaranteed by MoveCommandAction.update()
        target: PoseStamped = self.blackboard.last_command.compute_goal_pose(self.tf_listener)

        # Build low-speed mobility params
        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(0.1, 0.1, 0.2).to_proto(),  # linear x, y and angular z
                min_vel=math_helpers.SE2Velocity(-0.1, -0.1, -0.2).to_proto(),
            )
        )

        # Ensure target is in ODOM frame for base navigation
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

        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal