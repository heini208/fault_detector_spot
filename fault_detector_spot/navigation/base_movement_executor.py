"""Centralized RobotCommand execution for Spot base movement."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import time

from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
from tf2_geometry_msgs import do_transform_pose_stamped

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)
RELATIVE_LINEAR_SPEED_MPS = 0.10
TAG_LINEAR_SPEED_MPS = 0.15
ANGULAR_SPEED_RAD_S = 0.20


class BaseMovementOutcome(Enum):
    """Typed outcome of one base executor lifecycle update."""

    RUNNING = "running"
    SUCCESS = "success"
    BUSY = "busy"
    ACTION_SERVER_UNAVAILABLE = "action_server_unavailable"
    GOAL_RESPONSE_TIMEOUT = "goal_response_timeout"
    GOAL_REJECTED = "goal_rejected"
    RESULT_TIMEOUT = "result_timeout"
    MOTION_FAILED = "motion_failed"
    EXECUTION_ERROR = "execution_error"


@dataclass(frozen=True)
class BaseMovementUpdate:
    """Current nonblocking base execution outcome and detail."""

    outcome: BaseMovementOutcome
    detail: str


class BaseMovementExecutor(MovementExecutor):
    """Build, submit, monitor, and cancel Spot base movements."""

    OUTCOME_TYPE = BaseMovementOutcome
    UPDATE_TYPE = BaseMovementUpdate
    MOVEMENT_NAME = "base"

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        action_client=None,
        goal_response_timeout_sec: float = (
            DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC
        ),
        result_timeout_sec: float = DEFAULT_RESULT_TIMEOUT_SEC,
        monotonic_clock=time.monotonic,
        logger=None,
    ):
        super().__init__(
            tf_listener,
            tag_state_source=tag_state_source,
            robot_name=robot_name,
            action_client=action_client,
            goal_response_timeout_sec=goal_response_timeout_sec,
            result_timeout_sec=result_timeout_sec,
            monotonic_clock=monotonic_clock,
            logger=logger,
        )

    def relative(self, command) -> BaseMovementUpdate:
        """Start a relative SE2 base movement."""
        return self._start_goal(
            lambda: self._build_relative_goal(command)
        )

    def tag(self, command) -> BaseMovementUpdate:
        """Start an SE2 base movement relative to a live visible tag."""
        return self._start_goal(
            lambda: self._build_tag_goal(command)
        )

    def stand(self) -> BaseMovementUpdate:
        """Start Spot's native stand command."""
        return self._start_goal(self._build_stand_goal)

    def sit(self) -> BaseMovementUpdate:
        """Start Spot's native sit command."""
        return self._start_goal(self._build_sit_goal)

    def _build_relative_goal(self, command) -> RobotCommand.Goal:
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative base movement requires a command with "
                "compute_goal_pose()"
            )

        command = self._prepare_move_command(
            command,
            ODOM_FRAME_NAME,
        )
        target = command.compute_goal_pose(self.tf_listener)
        target = self._normalize_to_odom(target)
        return self._build_se2_goal(
            target,
            RELATIVE_LINEAR_SPEED_MPS,
        )

    def _build_tag_goal(self, command) -> RobotCommand.Goal:
        if self.tag_state_source is None:
            raise RuntimeError(
                "Tag base movement requires a tag state source"
            )
        if command is None or not hasattr(command, "tag_id"):
            raise TypeError(
                "Tag base movement requires a command with tag_id"
            )
        if not callable(getattr(command, "compute_goal_pose", None)):
            raise TypeError(
                "Tag base movement requires compute_goal_pose()"
            )

        tag_id = int(command.tag_id)
        tag = self.tag_state_source.visible_snapshot().get(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently visible"
            )

        command.tag_pose = deepcopy(tag.pose)
        command = self._prepare_move_command(
            command,
            ODOM_FRAME_NAME,
        )
        target = command.compute_goal_pose(self.tf_listener)
        target = self._normalize_to_odom(target)
        return self._build_se2_goal(
            target,
            TAG_LINEAR_SPEED_MPS,
        )

    def _build_stand_goal(self) -> RobotCommand.Goal:
        command = RobotCommandBuilder.synchro_stand_command()
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal

    def _build_sit_goal(self) -> RobotCommand.Goal:
        command = RobotCommandBuilder.synchro_sit_command()
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal

    def _normalize_to_odom(
        self,
        target: PoseStamped,
    ) -> PoseStamped:
        if not isinstance(target, PoseStamped):
            raise TypeError("Base target must be a PoseStamped")

        source_frame = target.header.frame_id.strip()
        if not source_frame:
            raise ValueError("Base target frame must not be empty")

        if source_frame == ODOM_FRAME_NAME:
            return deepcopy(target)

        transform = self.tf_listener.lookup_a_tform_b(
            ODOM_FRAME_NAME,
            source_frame,
            timeout_sec=0.0,
        )
        normalized = do_transform_pose_stamped(
            target,
            transform,
        )
        normalized.header.frame_id = ODOM_FRAME_NAME
        return normalized

    def _build_se2_goal(
        self,
        target: PoseStamped,
        linear_speed_mps: float,
    ) -> RobotCommand.Goal:
        orientation = target.pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )

        speed = float(linear_speed_mps)
        velocity_limit = SE2VelocityLimit(
            max_vel=math_helpers.SE2Velocity(
                speed,
                speed,
                ANGULAR_SPEED_RAD_S,
            ).to_proto(),
            min_vel=math_helpers.SE2Velocity(
                -speed,
                -speed,
                -ANGULAR_SPEED_RAD_S,
            ).to_proto(),
        )
        params = RobotCommandBuilder.mobility_params()
        params.vel_limit.CopyFrom(velocity_limit)

        command = (
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=target.pose.position.x,
                goal_y=target.pose.position.y,
                goal_heading=yaw,
                frame_name=namespace_with(
                    self.robot_name,
                    target.header.frame_id,
                ),
                params=params,
            )
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal



__all__ = [
    "BaseMovementExecutor",
    "BaseMovementOutcome",
    "BaseMovementUpdate",
]
