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
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import (
    transform_to_pose_data,
)
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)

READY_LIFT_DISTANCE_PARAMETER = "arm.ready_lift_distance_m"
READY_STATE_TIMEOUT_PARAMETER = "arm.ready_state_timeout_sec"
READY_TF_TIMEOUT_PARAMETER = "arm.ready_tf_timeout_sec"
READY_DEPLOYED_TIMEOUT_PARAMETER = "arm.ready_deployed_timeout_sec"
STOW_STATE_TIMEOUT_PARAMETER = "arm.stow_state_timeout_sec"

DEFAULT_READY_LIFT_DISTANCE_M = 0.10
DEFAULT_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_READY_TF_TIMEOUT_SEC = 2.0
DEFAULT_READY_DEPLOYED_TIMEOUT_SEC = 2.0
DEFAULT_STOW_STATE_TIMEOUT_SEC = 2.0


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
    ARM_STATE_UNAVAILABLE = "arm_state_unavailable"
    ARM_STATE_STALE = "arm_state_stale"
    ARM_STATE_UNKNOWN = "arm_state_unknown"
    EXECUTION_ERROR = "execution_error"


@dataclass(frozen=True)
class ArmMovementUpdate:
    """Current nonblocking execution outcome and diagnostic detail."""

    outcome: ArmMovementOutcome
    detail: str


class _ArmOperation(Enum):
    MOVEMENT = "movement"
    PREPARE = "prepare"
    STOW = "stow"


class ArmMovementExecutor(MovementExecutor):
    """Resolve, submit, monitor, and cancel Cartesian arm movements."""

    OUTCOME_TYPE = ArmMovementOutcome
    UPDATE_TYPE = ArmMovementUpdate
    MOVEMENT_NAME = "arm"

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        speed_policy=None,
        action_client=None,
        arm_state_source=None,
        ready_lift_distance_m: float = DEFAULT_READY_LIFT_DISTANCE_M,
        ready_state_timeout_sec: float = DEFAULT_READY_STATE_TIMEOUT_SEC,
        ready_tf_timeout_sec: float = DEFAULT_READY_TF_TIMEOUT_SEC,
        ready_deployed_timeout_sec: float = (
            DEFAULT_READY_DEPLOYED_TIMEOUT_SEC
        ),
        stow_state_timeout_sec: float = DEFAULT_STOW_STATE_TIMEOUT_SEC,
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
        self.speed_policy = (
            speed_policy
            if speed_policy is not None
            else ArmMotionSpeedPolicy()
        )
        self.arm_state_source = arm_state_source
        self.ready_lift_distance_m = self._positive_timeout(
            ready_lift_distance_m,
            "Ready arm lift distance",
        )
        self.ready_state_timeout_sec = self._positive_timeout(
            ready_state_timeout_sec,
            "Ready arm state timeout",
        )
        self.ready_tf_timeout_sec = self._positive_timeout(
            ready_tf_timeout_sec,
            "Ready arm TF timeout",
        )
        self.ready_deployed_timeout_sec = self._positive_timeout(
            ready_deployed_timeout_sec,
            "Ready arm deployed timeout",
        )
        self.stow_state_timeout_sec = self._positive_timeout(
            stow_state_timeout_sec,
            "Stow arm state timeout",
        )

        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None

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

    def prepare(
        self,
        speed=None,
    ) -> ArmMovementUpdate:
        """Prepare a stowed arm with a short controlled upward motion."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.PREPARE
        self._operation_speed = speed
        return self._advance_prepare_start()

    def stow(self) -> ArmMovementUpdate:
        """Stow a deployed arm through Spot's native stow command."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.STOW
        return self._advance_stow_start()

    def poll(self) -> ArmMovementUpdate:
        """Advance the active arm operation without blocking."""
        if not self.active or self._operation is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "No arm movement is active",
            )

        if self._verification_started is not None:
            return self._poll_state_confirmation()

        if self._send_goal_future is None:
            if self._operation is _ArmOperation.PREPARE:
                return self._advance_prepare_start()
            if self._operation is _ArmOperation.STOW:
                return self._advance_stow_start()
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Active arm movement has no RobotCommand goal",
            )

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def _start_goal(self, goal_builder) -> ArmMovementUpdate:
        if self.active:
            return self._busy_update()
        self._operation = _ArmOperation.MOVEMENT
        return super()._start_goal(goal_builder)

    def _handle_successful_result(self, result):
        if self._operation in (
            _ArmOperation.PREPARE,
            _ArmOperation.STOW,
        ):
            self._reset_goal_lifecycle()
            self._verification_started = self._monotonic_clock()
            return self._poll_state_confirmation()
        return super()._handle_successful_result(result)

    def _reset_operation(self) -> None:
        super()._reset_operation()
        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None

    def _advance_prepare_start(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.ready_state_timeout_sec,
                "Waiting for manipulator stow state",
            )

        self._state_wait_started = None
        if state is ArmStowState.DEPLOYED:
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Arm is already deployed",
            )

        try:
            hand_transform = self.tf_listener.lookup_a_tform_b(
                GRAV_ALIGNED_BODY_FRAME_NAME,
                HAND_FRAME_NAME,
                timeout_sec=0.0,
            )
        except Exception as exception:
            return self._wait_for_ready_transform(exception)

        self._tf_wait_started = None
        current_hand = self._pose_from_transform(
            hand_transform,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        target_hand = deepcopy(current_hand)
        target_hand.pose.position.z += self.ready_lift_distance_m

        return self._submit_goal(
            lambda: self._build_motion_goal(
                current_hand,
                target_hand,
                self._operation_speed,
            )
        )

    def _advance_stow_start(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.stow_state_timeout_sec,
                "Waiting for manipulator stow state",
            )

        self._state_wait_started = None
        if state is ArmStowState.STOWED:
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Arm is already stowed",
            )

        return self._submit_goal(self._build_stow_goal)

    def _poll_state_confirmation(self) -> ArmMovementUpdate:
        if self._operation is _ArmOperation.PREPARE:
            expected = ArmStowState.DEPLOYED
            timeout_sec = self.ready_deployed_timeout_sec
            success_detail = "Arm deployed"
            failure_detail = (
                "Ready arm movement completed, but Spot did not report "
                f"DEPLOYED within {timeout_sec:.1f} s"
            )
        elif self._operation is _ArmOperation.STOW:
            expected = ArmStowState.STOWED
            timeout_sec = self.stow_state_timeout_sec
            success_detail = "Arm stowed"
            failure_detail = (
                "Stow arm movement completed, but Spot did not report "
                f"STOWED within {timeout_sec:.1f} s"
            )
        else:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Arm state verification has no matching operation",
            )

        state = self._fresh_arm_state()
        if state is expected:
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                success_detail,
            )

        if not self._deadline_expired(
            self._verification_started,
            timeout_sec,
        ):
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                f"Waiting for Spot to report arm {expected.value}",
            )

        outcome = self._arm_state_failure_outcome(state)
        if (
            outcome is ArmMovementOutcome.ARM_STATE_UNKNOWN
            and state is not ArmStowState.UNKNOWN
        ):
            outcome = ArmMovementOutcome.MOTION_FAILED
        return self._finish(outcome, failure_detail)

    def _wait_for_arm_state(
        self,
        timeout_sec: float,
        detail: str,
    ) -> ArmMovementUpdate:
        now = self._monotonic_clock()
        if self._state_wait_started is None:
            self._state_wait_started = now

        if now - self._state_wait_started < timeout_sec:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                detail,
            )

        state = self._fresh_arm_state()
        outcome = self._arm_state_failure_outcome(state)
        return self._finish(
            outcome,
            "Fresh manipulator stow state was unavailable for "
            f"{timeout_sec:.1f} s",
        )

    def _wait_for_ready_transform(
        self,
        exception: Exception,
    ) -> ArmMovementUpdate:
        now = self._monotonic_clock()
        if self._tf_wait_started is None:
            self._tf_wait_started = now

        if now - self._tf_wait_started < self.ready_tf_timeout_sec:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for ready-arm hand pose transform "
                f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME}",
            )

        return self._finish(
            ArmMovementOutcome.EXECUTION_ERROR,
            "Ready arm hand transform "
            f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME} "
            f"was unavailable for {self.ready_tf_timeout_sec:.1f} s: "
            f"{exception}",
        )

    def _fresh_arm_state(self):
        if self.arm_state_source is None:
            return None
        return self.arm_state_source.stow_state()

    def _arm_state_failure_outcome(self, state):
        source = self.arm_state_source
        if source is None:
            return ArmMovementOutcome.ARM_STATE_UNAVAILABLE
        if getattr(source, "last_received_at", None) is None:
            return ArmMovementOutcome.ARM_STATE_UNAVAILABLE
        if source.is_stale():
            return ArmMovementOutcome.ARM_STATE_STALE
        if state is ArmStowState.UNKNOWN:
            return ArmMovementOutcome.ARM_STATE_UNKNOWN
        return ArmMovementOutcome.ARM_STATE_UNKNOWN

    def _build_stow_goal(self) -> RobotCommand.Goal:
        stow_command = RobotCommandBuilder.arm_stow_command()
        goal = RobotCommand.Goal()
        convert(stow_command, goal.command)
        return goal

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



__all__ = [
    "ArmMovementExecutor",
    "ArmMovementOutcome",
    "ArmMovementUpdate",
]
