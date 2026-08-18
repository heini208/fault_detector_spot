from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand

from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.inspection.geometry.rotation import quaternion_to_rpy
from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.navigation.commands.base_to_tag_command import (
    BaseToTagCommand,
)


class BaseMoveToTagAction(MoveCommandAction):
    """Move the robot base to a tag-relative target at low speed."""

    def __init__(
        self,
        name="BaseMoveToTagAction",
        robot_name="",
        duration=3.0,
    ):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(self.blackboard.last_command, BaseToTagCommand):
            raise RuntimeError(
                "Expected BaseToTagCommand on blackboard.last_command"
            )

        target: PoseStamped = self.blackboard.last_command.compute_goal_pose(
            self.tf_listener
        )
        orientation = target.pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )

        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(
                    0.15,
                    0.15,
                    0.2,
                ).to_proto(),
                min_vel=math_helpers.SE2Velocity(
                    -0.15,
                    -0.15,
                    -0.2,
                ).to_proto(),
            )
        )

        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=target.pose.position.x,
            goal_y=target.pose.position.y,
            goal_heading=yaw,
            frame_name=target.header.frame_id,
            params=low_speed_params,
        )

        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal
