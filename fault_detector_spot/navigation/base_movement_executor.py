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


DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC = 2.0
DEFAULT_RESULT_TIMEOUT_SEC = 30.0
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


class BaseMovementExecutor:
    """Build, submit, monitor, and cancel Spot base movements."""

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
        if tf_listener is None:
            raise ValueError(
                "BaseMovementExecutor requires a TF listener"
            )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.tf_listener = tf_listener
        self.tag_state_source = tag_state_source
        self.robot_name = robot_name
        self.action_client = action_client
        self.goal_response_timeout_sec = self._positive_timeout(
            goal_response_timeout_sec,
            "Goal response timeout",
        )
        self.result_timeout_sec = self._positive_timeout(
            result_timeout_sec,
            "Action result timeout",
        )
        self._monotonic_clock = monotonic_clock
        self._logger = logger

        self._active = False
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None

    @property
    def active(self) -> bool:
        return self._active

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

    def poll(self) -> BaseMovementUpdate:
        """Advance the active base operation without blocking."""
        if not self.active:
            return BaseMovementUpdate(
                BaseMovementOutcome.EXECUTION_ERROR,
                "No base movement is active",
            )

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def cancel(self) -> None:
        """Request cancellation of the active base operation."""
        if not self.active:
            return
        self._request_cancel()
        self._reset()

    def shutdown(self) -> None:
        self.cancel()

    def _start_goal(self, goal_builder) -> BaseMovementUpdate:
        if self.active:
            return BaseMovementUpdate(
                BaseMovementOutcome.BUSY,
                "Another base movement is already active",
            )

        self._active = True
        if self.action_client is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Base movement executor has no RobotCommand action client",
            )

        try:
            server_ready = self.action_client.wait_for_server(
                timeout_sec=0.0
            )
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"RobotCommand action server check failed: {exception}",
            )

        if not server_ready:
            action_name = namespace_with(
                self.robot_name,
                "robot_command",
            )
            return self._finish(
                BaseMovementOutcome.ACTION_SERVER_UNAVAILABLE,
                f"Action server '{action_name}' unavailable",
            )

        try:
            goal = goal_builder()
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Base goal preparation failed: {exception}",
            )

        try:
            future = self.action_client.send_goal_async(goal)
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Base goal submission failed: {exception}",
            )

        if future is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "RobotCommand action client returned no goal future",
            )

        self._send_goal_future = future
        self._goal_sent_monotonic = self._monotonic_clock()
        return BaseMovementUpdate(
            BaseMovementOutcome.RUNNING,
            "Goal sent",
        )

    def _poll_goal_response(self) -> BaseMovementUpdate:
        if self._send_goal_future is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Active base movement has no goal future",
            )

        if not self._send_goal_future.done():
            if self._deadline_expired(
                self._goal_sent_monotonic,
                self.goal_response_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    BaseMovementOutcome.GOAL_RESPONSE_TIMEOUT,
                    "Action goal response timed out after "
                    f"{self.goal_response_timeout_sec:.1f} s",
                )
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Waiting for goal acceptance",
            )

        try:
            goal_handle = self._send_goal_future.result()
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Base goal submission failed: {exception}",
            )

        if goal_handle is None or not goal_handle.accepted:
            return self._finish(
                BaseMovementOutcome.GOAL_REJECTED,
                "Action goal was rejected",
            )

        self._goal_handle = goal_handle
        try:
            self._result_future = goal_handle.get_result_async()
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Action result request failed: {exception}",
            )

        if self._result_future is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "RobotCommand goal returned no result future",
            )

        self._result_started_monotonic = self._monotonic_clock()
        return BaseMovementUpdate(
            BaseMovementOutcome.RUNNING,
            "Goal accepted",
        )

    def _poll_result(self) -> BaseMovementUpdate:
        if self._result_future is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Accepted base goal has no result future",
            )

        if not self._result_future.done():
            if self._deadline_expired(
                self._result_started_monotonic,
                self.result_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    BaseMovementOutcome.RESULT_TIMEOUT,
                    "Action result timed out after "
                    f"{self.result_timeout_sec:.1f} s",
                )
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Base movement in progress",
            )

        try:
            result_wrapper = self._result_future.result()
            result = result_wrapper.result
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Action result failed: {exception}",
            )

        if bool(getattr(result, "success", False)):
            return self._finish(
                BaseMovementOutcome.SUCCESS,
                "Succeeded",
            )

        return self._finish(
            BaseMovementOutcome.MOTION_FAILED,
            self._command_failure_detail(result),
        )

    def _build_relative_goal(self, command) -> RobotCommand.Goal:
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative base movement requires a command with "
                "compute_goal_pose()"
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

    def _request_cancel(self) -> None:
        handle = self._goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exception:
                self._log_error(
                    f"Base goal cancellation failed: {exception}"
                )
            return

        future = self._send_goal_future
        if future is None or future.done():
            return

        def cancel_when_accepted(done_future):
            try:
                accepted_handle = done_future.result()
                if (
                    accepted_handle is not None
                    and accepted_handle.accepted
                ):
                    accepted_handle.cancel_goal_async()
            except Exception as exception:
                self._log_error(
                    "Pending base goal cancellation failed: "
                    f"{exception}"
                )

        future.add_done_callback(cancel_when_accepted)

    def _finish(
        self,
        outcome: BaseMovementOutcome,
        detail: str,
    ) -> BaseMovementUpdate:
        update = BaseMovementUpdate(
            outcome,
            str(detail).strip(),
        )
        self._reset()
        return update

    def _reset(self) -> None:
        self._active = False
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None

    def _deadline_expired(
        self,
        started,
        timeout_sec: float,
    ) -> bool:
        if started is None:
            return False
        return self._monotonic_clock() - started >= timeout_sec

    @staticmethod
    def _positive_timeout(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized

    @staticmethod
    def _command_failure_detail(result) -> str:
        detail = str(
            getattr(result, "detail", "")
            or getattr(result, "message", "")
        ).strip()
        if detail:
            return detail
        return f"Robot command failed: {result}"

    def _log_error(self, message: str) -> None:
        if self._logger is None:
            return
        log = getattr(self._logger, "error", None)
        if callable(log):
            log(message)


__all__ = [
    "BaseMovementExecutor",
    "BaseMovementOutcome",
    "BaseMovementUpdate",
]
