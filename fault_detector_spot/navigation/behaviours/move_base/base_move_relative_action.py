from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
from tf2_geometry_msgs import do_transform_pose_stamped

from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.inspection.geometry.rotation import quaternion_to_rpy
from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.navigation.commands.base_move_relative_command import (
    BaseMoveRelativeCommand,
)


class BaseMoveRelativeAction(MoveCommandAction):
    """Move the robot base relative to its current pose at low speed."""

    def __init__(self, name="BaseMoveRelativeAction", robot_name=""):
        super().__init__(name)
        self.robot_name = robot_name

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs.get("node")

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(
            self.blackboard.last_command,
            BaseMoveRelativeCommand,
        ):
            raise RuntimeError(
                "Expected BaseMoveRelativeCommand on blackboard.last_command"
            )

        target: PoseStamped = self.blackboard.last_command.compute_goal_pose(
            self.tf_listener
        )

        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(
                    0.1,
                    0.1,
                    0.2,
                ).to_proto(),
                min_vel=math_helpers.SE2Velocity(
                    -0.1,
                    -0.1,
                    -0.2,
                ).to_proto(),
            )
        )

        if target.header.frame_id != ODOM_FRAME_NAME:
            tf_to_odom = self.tf_listener.lookup_a_tform_b(
                ODOM_FRAME_NAME,
                target.header.frame_id,
                timeout_sec=2,
            )
            target = do_transform_pose_stamped(target, tf_to_odom)
            target.header.frame_id = ODOM_FRAME_NAME

        orientation = target.pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )

        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=target.pose.position.x,
            goal_y=target.pose.position.y,
            frame_name=namespace_with(
                self.robot_name,
                target.header.frame_id,
            ),
            goal_heading=yaw,
            params=low_speed_params,
        )

        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal
