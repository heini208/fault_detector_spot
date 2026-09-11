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
from fault_detector_spot.navigation.posture_state_source import (
    PostureState,
)
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)

BASE_READY_STATE_TIMEOUT_PARAMETER = "base.ready_state_timeout_sec"
BASE_READY_STANDING_TIMEOUT_PARAMETER = (
    "base.ready_standing_timeout_sec"
)

DEFAULT_BASE_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC = 2.0

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
    POSTURE_STATE_UNAVAILABLE = "posture_state_unavailable"
    POSTURE_STATE_STALE = "posture_state_stale"
    POSTURE_STATE_UNKNOWN = "posture_state_unknown"
    EXECUTION_ERROR = "execution_error"


@dataclass(frozen=True)
class BaseMovementUpdate:
    """Current nonblocking base execution outcome and detail."""

    outcome: BaseMovementOutcome
    detail: str


class _BaseOperation(Enum):
    MOVEMENT = "movement"
    MOVEMENT_STAND = "movement_stand"
    STAND = "stand"
    SIT = "sit"


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
        posture_state_source=None,
        ready_state_timeout_sec: float = (
            DEFAULT_BASE_READY_STATE_TIMEOUT_SEC
        ),
        ready_standing_timeout_sec: float = (
            DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC
        ),
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
        self.posture_state_source = posture_state_source
        self.ready_state_timeout_sec = self._positive_timeout(
            ready_state_timeout_sec,
            "Base ready state timeout",
        )
        self.ready_standing_timeout_sec = self._positive_timeout(
            ready_standing_timeout_sec,
            "Base ready standing timeout",
        )

        self._operation = None
        self._movement_goal_builder = None
        self._state_wait_started = None
        self._verification_started = None

    def relative(self, command) -> BaseMovementUpdate:
        """Start a relative SE2 base movement."""
        return self._start_movement_goal(
            lambda: self._build_relative_goal(command)
        )

    def tag(self, command) -> BaseMovementUpdate:
        """Start an SE2 base movement relative to a live visible tag."""
        return self._start_movement_goal(
            lambda: self._build_tag_goal(command)
        )

    def stand(self) -> BaseMovementUpdate:
        """Start Spot's native stand command directly."""
        if self.active:
            return self._busy_update()
        self._operation = _BaseOperation.STAND
        return super()._start_goal(self._build_stand_goal)

    def sit(self) -> BaseMovementUpdate:
        """Start Spot's native sit command directly."""
        if self.active:
            return self._busy_update()
        self._operation = _BaseOperation.SIT
        return super()._start_goal(self._build_sit_goal)

    def poll(self) -> BaseMovementUpdate:
        """Advance the active base operation without blocking."""
        if not self.active or self._operation is None:
            return BaseMovementUpdate(
                BaseMovementOutcome.EXECUTION_ERROR,
                "No base movement is active",
            )

        if self._verification_started is not None:
            return self._poll_standing_confirmation()

        if self._send_goal_future is None:
            if (
                self._operation is _BaseOperation.MOVEMENT
                and self._pending_goal_builder is None
            ):
                return self._advance_movement_start()
            return super().poll()

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def _start_movement_goal(self, goal_builder) -> BaseMovementUpdate:
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _BaseOperation.MOVEMENT
        self._movement_goal_builder = goal_builder
        return self._advance_movement_start()

    def _advance_movement_start(self) -> BaseMovementUpdate:
        state = self._fresh_posture_state()
        if state is None or state is PostureState.UNKNOWN:
            return self._wait_for_posture_state()

        self._state_wait_started = None
        if state is PostureState.SITTING:
            self._operation = _BaseOperation.MOVEMENT_STAND
            return self._submit_goal(self._build_stand_goal)

        return self._submit_movement_goal()

    def _submit_movement_goal(self) -> BaseMovementUpdate:
        goal_builder = self._movement_goal_builder
        if goal_builder is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Base movement has no pending goal builder",
            )

        self._operation = _BaseOperation.MOVEMENT
        self._pending_goal_builder = goal_builder
        return self._submit_goal(goal_builder)

    def _handle_successful_result(self, result):
        if self._operation is _BaseOperation.MOVEMENT_STAND:
            self._reset_goal_lifecycle()
            self._verification_started = self._monotonic_clock()
            return self._poll_standing_confirmation()
        return super()._handle_successful_result(result)

    def _poll_standing_confirmation(self) -> BaseMovementUpdate:
        state = self._fresh_posture_state()
        if state is PostureState.STANDING:
            self._verification_started = None
            return self._submit_movement_goal()

        if not self._deadline_expired(
            self._verification_started,
            self.ready_standing_timeout_sec,
        ):
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Waiting for Spot to report standing",
            )

        outcome = self._posture_state_failure_outcome(state)
        if (
            outcome is BaseMovementOutcome.POSTURE_STATE_UNKNOWN
            and state is not PostureState.UNKNOWN
        ):
            outcome = BaseMovementOutcome.MOTION_FAILED
        return self._finish(
            outcome,
            "Stand movement completed, but Spot did not report "
            "STANDING within "
            f"{self.ready_standing_timeout_sec:.1f} s",
        )

    def _wait_for_posture_state(self) -> BaseMovementUpdate:
        now = self._monotonic_clock()
        if self._state_wait_started is None:
            self._state_wait_started = now

        if now - self._state_wait_started < self.ready_state_timeout_sec:
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Waiting for fresh base posture before movement",
            )

        state = self._fresh_posture_state()
        return self._finish(
            self._posture_state_failure_outcome(state),
            "Fresh base posture was unavailable for "
            f"{self.ready_state_timeout_sec:.1f} s",
        )

    def _fresh_posture_state(self):
        if self.posture_state_source is None:
            return None
        return self.posture_state_source.posture()

    def _posture_state_failure_outcome(self, state):
        source = self.posture_state_source
        if source is None:
            return BaseMovementOutcome.POSTURE_STATE_UNAVAILABLE
        if getattr(source, "last_received_at", None) is None:
            return BaseMovementOutcome.POSTURE_STATE_UNAVAILABLE
        if source.is_stale():
            return BaseMovementOutcome.POSTURE_STATE_STALE
        if state is PostureState.UNKNOWN:
            return BaseMovementOutcome.POSTURE_STATE_UNKNOWN
        return BaseMovementOutcome.POSTURE_STATE_UNKNOWN

    def _reset_operation(self) -> None:
        super()._reset_operation()
        self._operation = None
        self._movement_goal_builder = None
        self._state_wait_started = None
        self._verification_started = None

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
