from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder

import tf_transformations as tf
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.base_to_tag_command import BaseToTagCommand
from fault_detector_spot.behaviour_tree.nodes.utility.move_command_action import MoveCommandAction
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand


class BaseMoveToTagAction(MoveCommandAction):
    """
    Moves the robot base to the tag with the given offset, using low speed limits.
    Inherits TF safety checks from MoveCommandAction.
    """

    def __init__(self, name="BaseMoveToTagAction", robot_name="", duration=3.0):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration
        # blackboard & tf_listener init handled in base/setup

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(self.blackboard.last_command, BaseToTagCommand):
            raise RuntimeError("Expected BaseToTagCommand on blackboard.last_command")

        # TF readiness is guaranteed by MoveCommandAction.update()
        target: PoseStamped = self.blackboard.last_command.compute_goal_pose(self.tf_listener)

        # Build low-speed mobility params
        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(0.15, 0.15, 0.2).to_proto(),  # linear x, y and angular z
                min_vel=math_helpers.SE2Velocity(-0.15, -0.15, -0.2).to_proto(),
            )
        )

        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=target.pose.position.x,
            goal_y=target.pose.position.y,
            goal_heading=tf.euler_from_quaternion([
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ])[2],
            frame_name=target.header.frame_id,
            params=low_speed_params,
        )

        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal