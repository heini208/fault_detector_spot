"""Centralized Cartesian Spot arm movement execution."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import time

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
import tf2_geometry_msgs

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import (
    transform_to_pose_data,
)


DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC = 2.0
DEFAULT_RESULT_TIMEOUT_SEC = 30.0


class ArmMovementOutcome(Enum):
    """Typed outcome of one executor lifecycle update."""

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
class ArmMovementUpdate:
    """Current nonblocking execution outcome and diagnostic detail."""

    outcome: ArmMovementOutcome
    detail: str


class ArmMovementExecutor:
    """Resolve, submit, monitor, and cancel Cartesian arm movements."""

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        speed_policy=None,
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
                "ArmMovementExecutor requires a TF listener"
            )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.tf_listener = tf_listener
        self.tag_state_source = tag_state_source
        self.robot_name = robot_name
        self.speed_policy = (
            speed_policy
            if speed_policy is not None
            else ArmMotionSpeedPolicy()
        )
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

        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None

    @property
    def active(self) -> bool:
        return self._send_goal_future is not None

    def relative(
        self,
        command,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start a hand-relative movement."""
        return self._start_goal(
            lambda: self._build_relative_goal(command, speed)
        )

    def pose(
        self,
        target: PoseStamped,
        execution_frame: str = "",
        speed=None,
    ) -> ArmMovementUpdate:
        """Start an absolute hand-pose movement."""
        return self._start_goal(
            lambda: self._build_hand_pose_goal(
                target,
                execution_frame,
                speed,
            )
        )

    def probe_pose(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start an absolute active-probe movement."""
        return self._start_goal(
            lambda: self._build_probe_pose_goal(
                probe_target,
                motion_sensor_id,
                speed,
            )
        )

    def tag_probe(
        self,
        command,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start a probe movement to a target relative to a live tag."""
        return self._start_goal(
            lambda: self._build_tag_probe_goal(command, speed)
        )

    def probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start a movement relative to the current active probe frame."""
        return self._start_goal(
            lambda: self._build_probe_relative_goal(
                offset,
                motion_sensor_id,
                speed,
            )
        )

    def poll(self) -> ArmMovementUpdate:
        """Advance the active RobotCommand lifecycle without blocking."""
        if self._send_goal_future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "No arm movement is active",
            )

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def cancel(self) -> None:
        """Request cancellation of the active movement and release it."""
        if not self.active:
            return
        self._request_cancel()
        self._reset_lifecycle()

    def shutdown(self) -> None:
        self.cancel()

    def _start_goal(self, goal_builder) -> ArmMovementUpdate:
        if self.active:
            return ArmMovementUpdate(
                ArmMovementOutcome.BUSY,
                "Another arm movement is already active",
            )
        if self.action_client is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Arm movement executor has no RobotCommand action client",
            )

        try:
            server_ready = self.action_client.wait_for_server(
                timeout_sec=0.0
            )
        except Exception as exception:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"RobotCommand action server check failed: {exception}",
            )
        if not server_ready:
            action_name = namespace_with(
                self.robot_name,
                "robot_command",
            )
            return ArmMovementUpdate(
                ArmMovementOutcome.ACTION_SERVER_UNAVAILABLE,
                f"Action server '{action_name}' unavailable",
            )

        try:
            goal = goal_builder()
        except Exception as exception:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Arm goal preparation failed: {exception}",
            )

        try:
            future = self.action_client.send_goal_async(goal)
        except Exception as exception:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Arm goal submission failed: {exception}",
            )
        if future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "RobotCommand action client returned no goal future",
            )

        self._send_goal_future = future
        self._goal_sent_monotonic = self._monotonic_clock()
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        )

    def _poll_goal_response(self) -> ArmMovementUpdate:
        if not self._send_goal_future.done():
            if self._deadline_expired(
                self._goal_sent_monotonic,
                self.goal_response_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    ArmMovementOutcome.GOAL_RESPONSE_TIMEOUT,
                    "Action goal response timed out after "
                    f"{self.goal_response_timeout_sec:.1f} s",
                )
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for goal acceptance",
            )

        try:
            goal_handle = self._send_goal_future.result()
        except Exception as exception:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Arm goal submission failed: {exception}",
            )

        if goal_handle is None or not goal_handle.accepted:
            return self._finish(
                ArmMovementOutcome.GOAL_REJECTED,
                "Action goal was rejected",
            )

        self._goal_handle = goal_handle
        try:
            self._result_future = goal_handle.get_result_async()
        except Exception as exception:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Action result request failed: {exception}",
            )

        if self._result_future is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "RobotCommand goal returned no result future",
            )

        self._result_started_monotonic = self._monotonic_clock()
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal accepted",
        )

    def _poll_result(self) -> ArmMovementUpdate:
        if self._result_future is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Accepted arm goal has no result future",
            )

        if not self._result_future.done():
            if self._deadline_expired(
                self._result_started_monotonic,
                self.result_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    ArmMovementOutcome.RESULT_TIMEOUT,
                    "Action result timed out after "
                    f"{self.result_timeout_sec:.1f} s",
                )
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Arm movement in progress",
            )

        try:
            result_wrapper = self._result_future.result()
            result = result_wrapper.result
        except Exception as exception:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Action result failed: {exception}",
            )

        if bool(getattr(result, "success", False)):
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Succeeded",
            )

        return self._finish(
            ArmMovementOutcome.MOTION_FAILED,
            self._command_failure_detail(result),
        )

    def _request_cancel(self) -> None:
        handle = self._goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exception:
                self._log_error(
                    f"Arm goal cancellation failed: {exception}"
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
                    "Pending arm goal cancellation failed: "
                    f"{exception}"
                )

        future.add_done_callback(cancel_when_accepted)

    def _finish(
        self,
        outcome: ArmMovementOutcome,
        detail: str,
    ) -> ArmMovementUpdate:
        update = ArmMovementUpdate(
            outcome,
            str(detail).strip(),
        )
        self._reset_lifecycle()
        return update

    def _reset_lifecycle(self) -> None:
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None

    def _build_relative_goal(
        self,
        command,
        speed=None,
    ) -> RobotCommand.Goal:
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative arm movement requires a command with "
                "compute_goal_pose()"
            )

        relative_target = command.compute_goal_pose(
            self.tf_listener
        )
        source_frame = relative_target.header.frame_id.strip()
        if not source_frame:
            raise ValueError(
                "Relative arm target frame must not be empty"
            )

        source_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            source_frame,
            timeout_sec=0.0,
        )
        target_hand = tf2_geometry_msgs.do_transform_pose_stamped(
            relative_target,
            source_to_execution,
        )

        if source_frame == HAND_FRAME_NAME:
            current_hand = self._pose_from_transform(
                source_to_execution,
                GRAV_ALIGNED_BODY_FRAME_NAME,
            )
        else:
            current_hand = self._current_pose(
                GRAV_ALIGNED_BODY_FRAME_NAME,
                HAND_FRAME_NAME,
            )

        return self._build_motion_goal(
            current_hand,
            target_hand,
            speed,
        )

    def _build_hand_pose_goal(
        self,
        target: PoseStamped,
        execution_frame: str = "",
        speed=None,
    ) -> RobotCommand.Goal:
        target_hand = self._normalize_target(
            target,
            execution_frame,
        )
        current_hand = self._current_pose(
            target_hand.header.frame_id,
            HAND_FRAME_NAME,
        )
        return self._build_motion_goal(
            current_hand,
            target_hand,
            speed,
        )

    def _build_probe_pose_goal(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> RobotCommand.Goal:
        if not isinstance(probe_target, PoseStamped):
            raise TypeError("Probe target must be a PoseStamped")

        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Probe movement requires attachment geometry"
            )

        if sensor_id == BARE_HAND_MOTION_ID:
            return self._build_hand_pose_goal(
                probe_target,
                speed=speed,
            )

        target_frame = probe_target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Probe target frame must not be empty")

        current_probe = self._current_pose(
            target_frame,
            sensor_probe_frame(sensor_id),
        )
        return self._build_probe_motion_goal(
            current_probe,
            probe_target,
            sensor_id,
            speed,
        )

    def _build_tag_probe_goal(
        self,
        command,
        speed=None,
    ) -> RobotCommand.Goal:
        if self.tag_state_source is None:
            raise RuntimeError(
                "Tag probe movement requires a tag state source"
            )
        if command is None or not hasattr(command, "tag_id"):
            raise TypeError(
                "Tag probe movement requires a command with tag_id"
            )
        if not callable(getattr(command, "compute_goal_pose", None)):
            raise TypeError(
                "Tag probe movement requires compute_goal_pose()"
            )

        tag_id = int(command.tag_id)
        tag = self.tag_state_source.reachable_tag(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently reachable"
            )

        command.tag_pose = deepcopy(tag.pose)
        probe_target = command.compute_goal_pose(
            self.tf_listener
        )
        return self._build_probe_pose_goal(
            probe_target,
            command.motion_sensor_id,
            speed,
        )

    def _build_probe_relative_goal(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> RobotCommand.Goal:
        if not isinstance(offset, PoseStamped):
            raise TypeError(
                "Probe-relative offset must be a PoseStamped"
            )

        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Probe-relative movement requires attachment geometry"
            )

        probe_frame = sensor_probe_frame(sensor_id)
        offset_frame = offset.header.frame_id.strip()
        if offset_frame != probe_frame:
            raise ValueError(
                "Probe-relative offset must be expressed in the "
                f"active probe frame '{probe_frame}'"
            )

        probe_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        current_probe = self._pose_from_transform(
            probe_to_execution,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        target_probe = tf2_geometry_msgs.do_transform_pose_stamped(
            offset,
            probe_to_execution,
        )

        if sensor_id == BARE_HAND_MOTION_ID:
            return self._build_motion_goal(
                current_probe,
                target_probe,
                speed,
            )

        return self._build_probe_motion_goal(
            current_probe,
            target_probe,
            sensor_id,
            speed,
        )

    def _build_probe_motion_goal(
        self,
        current_probe: PoseStamped,
        target_probe: PoseStamped,
        sensor_id: str,
        speed=None,
    ) -> RobotCommand.Goal:
        duration_sec = self.speed_policy.duration_between(
            current_probe.pose,
            target_probe.pose,
            speed=speed,
        )
        hand_target = self._probe_target_to_hand_target(
            target_probe,
            sensor_id,
        )
        return self._build_pose_goal(
            hand_target,
            duration_sec,
        )

    def _probe_target_to_hand_target(
        self,
        probe_target: PoseStamped,
        sensor_id: str,
    ) -> PoseStamped:
        probe_frame = sensor_probe_frame(sensor_id)
        hand_to_probe = self.tf_listener.lookup_a_tform_b(
            HAND_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        hand_to_probe_pose = pose_data_to_pose(
            transform_to_pose_data(hand_to_probe)
        )

        hand_target = deepcopy(probe_target)
        hand_target.pose = compose_poses(
            probe_target.pose,
            inverse_pose(hand_to_probe_pose),
        )
        return hand_target

    def _build_motion_goal(
        self,
        current_hand: PoseStamped,
        target_hand: PoseStamped,
        speed=None,
    ) -> RobotCommand.Goal:
        duration_sec = self.speed_policy.duration_between(
            current_hand.pose,
            target_hand.pose,
            speed=speed,
        )
        return self._build_pose_goal(
            target_hand,
            duration_sec,
        )

    def _normalize_target(
        self,
        target: PoseStamped,
        execution_frame: str = "",
    ) -> PoseStamped:
        if not isinstance(target, PoseStamped):
            raise TypeError("Arm pose target must be a PoseStamped")

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        normalized_frame = execution_frame.strip()
        if not normalized_frame or target_frame == normalized_frame:
            return target

        transform = self.tf_listener.lookup_a_tform_b(
            normalized_frame,
            target_frame,
            timeout_sec=0.0,
        )
        return tf2_geometry_msgs.do_transform_pose_stamped(
            target,
            transform,
        )

    def _current_pose(
        self,
        target_frame: str,
        controlled_frame: str,
    ) -> PoseStamped:
        transform = self.tf_listener.lookup_a_tform_b(
            target_frame,
            controlled_frame,
            timeout_sec=0.0,
        )
        return self._pose_from_transform(
            transform,
            target_frame,
        )

    @staticmethod
    def _pose_from_transform(
        transform,
        frame_id: str,
    ) -> PoseStamped:
        current = PoseStamped()
        current.header.frame_id = frame_id
        current.pose = pose_data_to_pose(
            transform_to_pose_data(transform)
        )
        return current

    def _build_pose_goal(
        self,
        target: PoseStamped,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Translate the internal speed result to Spot's duration API."""
        duration = float(duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError(
                "Arm movement duration must be positive and finite"
            )

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        pose = target.pose
        command = RobotCommandBuilder.arm_pose_command(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            namespace_with(self.robot_name, target_frame),
            duration,
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal

    def _deadline_expired(self, started, timeout_sec) -> bool:
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
    "ArmMovementExecutor",
    "ArmMovementOutcome",
    "ArmMovementUpdate",
]
