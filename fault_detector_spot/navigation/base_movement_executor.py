"""Centralized RobotCommand execution for Spot base movement."""

from dataclasses import dataclass
from enum import Enum
import math
import time

from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.navigation.base_motion_planner import (
    BaseMotionPlanner,
    BaseMovementPlan,
)
from fault_detector_spot.navigation.posture_state_source import (
    PostureState,
)
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)

from fault_detector_spot.navigation.base_goal_verifier import (
    BaseGoalVerifier, BaseGoalVerificationConfig,
)

from fault_detector_spot.navigation.walking_profile import GAITS, WalkingProfiles

BASE_READY_STATE_TIMEOUT_PARAMETER = "base.ready_state_timeout_sec"
BASE_READY_STANDING_TIMEOUT_PARAMETER = (
    "base.ready_standing_timeout_sec"
)

DEFAULT_BASE_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC = 2.0


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
        goal_verification_config=None,
        ros_time_sec=time.time,
        walking_profiles=None,
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
        self.goal_verification_config = (
            goal_verification_config or BaseGoalVerificationConfig()
        )
        self.walking_profiles = walking_profiles or WalkingProfiles()
        self.motion_planner = BaseMotionPlanner(
            tf_listener,
            self.walking_profiles,
        )
        self._ros_time_sec = ros_time_sec
        self.posture_state_source = posture_state_source
        self.ready_state_timeout_sec = self._positive_timeout(
            ready_state_timeout_sec,
            "Base ready state timeout",
        )
        self.ready_standing_timeout_sec = self._positive_timeout(
            ready_standing_timeout_sec,
            "Base ready standing timeout",
        )

        self._goal_verifier = None
        self._movement_plan = None
        self._correction_attempts = 0
        self._previous_correction_error = None
        self._operation = None
        self._movement_plan_builder = None
        self._state_wait_started = None
        self._verification_started = None

    def relative(self, command) -> BaseMovementUpdate:
        """Start a relative SE2 base movement."""
        return self._start_verified_base_movement(
            lambda: self.motion_planner.resolve_relative(command)
        )

    def tag(self, command) -> BaseMovementUpdate:
        """Start an SE2 base movement relative to a live visible tag."""
        return self._start_verified_base_movement(
            lambda: self.motion_planner.resolve_tag(
                command,
                self.tag_state_source,
            )
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

        if self._goal_verifier is not None:
            return self._poll_goal_verification()

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

    def _start_verified_base_movement(self, plan_builder) -> BaseMovementUpdate:
        """Start one movement whose plan is resolved after readiness."""
        if self.active:
            return self._busy_update()
        if not callable(plan_builder):
            raise TypeError("Base movement requires a plan builder")
        self._active = True
        self._operation = _BaseOperation.MOVEMENT
        self._movement_plan_builder = plan_builder
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
        plan_builder = self._movement_plan_builder
        if plan_builder is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Base movement has no pending plan builder",
            )

        def build_goal():
            plan = plan_builder()
            if not isinstance(plan, BaseMovementPlan):
                raise TypeError(
                    "Base movement plan builder must return BaseMovementPlan"
                )
            self._movement_plan = plan
            return self._build_absolute_base_goal(plan)

        self._operation = _BaseOperation.MOVEMENT
        self._pending_goal_builder = build_goal
        return self._submit_goal(build_goal)

    def _handle_successful_result(self, result):
        if self._operation is _BaseOperation.MOVEMENT_STAND:
            self._reset_goal_lifecycle()
            self._verification_started = self._monotonic_clock()
            return self._poll_standing_confirmation()
        if self._operation is _BaseOperation.MOVEMENT:
            plan = self._movement_plan
            if plan is None:
                return self._finish(
                    BaseMovementOutcome.EXECUTION_ERROR,
                    "Base movement has no plan for endpoint verification",
                )
            self._goal_verifier = BaseGoalVerifier(
                self._planar_target(plan),
                self.goal_verification_config,
                self._monotonic_clock(),
            )
            return self._poll_goal_verification()
        return super()._handle_successful_result(result)

    def _poll_goal_verification(self):
        verifier = self._goal_verifier
        pose, stamp = None, None
        try:
            transform = self.tf_listener.lookup_a_tform_b(
                ODOM_FRAME_NAME, BODY_FRAME_NAME, timeout_sec=0.0,
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            _, _, yaw = quaternion_to_rpy(QuaternionData(
                x=rotation.x, y=rotation.y, z=rotation.z, w=rotation.w,
            ))
            pose = (translation.x, translation.y, yaw)
            stamp = (transform.header.stamp.sec
                     + transform.header.stamp.nanosec * 1e-9)
        except Exception:
            pass
        outcome = verifier.update(
            pose, stamp, self._ros_time_sec(), self._monotonic_clock(),
        )
        if outcome is True:
            return self._finish(
                BaseMovementOutcome.SUCCESS,
                f"Base goal verified and settled; {verifier.detail}",
            )
        if outcome is False:
            correction = self._correct_absolute_base_goal(verifier)
            if correction is not None:
                return correction
            self._request_cancel()
            return self._finish(
                BaseMovementOutcome.MOTION_FAILED,
                f"Base endpoint verification timed out; {verifier.detail}",
            )
        return BaseMovementUpdate(BaseMovementOutcome.RUNNING, verifier.detail)

    def _correct_absolute_base_goal(self, verifier):
        """Retry only a fresh, out-of-tolerance endpoint at the frozen goal."""
        c = self.goal_verification_config
        error = verifier.current_error
        if error is None:
            return None
        score = max(error[0] / c.position_tolerance_m,
                    error[1] / c.yaw_tolerance_rad)
        if score <= 1.0:
            return None
        if self._correction_attempts >= c.maximum_correction_attempts:
            verifier.detail += "; correction attempt limit reached"
            return None
        previous = self._previous_correction_error
        if previous is not None and score > previous * (1 - c.minimum_correction_progress_ratio):
            verifier.detail += "; corrections made insufficient progress"
            return None
        plan = self._movement_plan
        if plan is None:
            return None
        self._previous_correction_error = score
        self._correction_attempts += 1
        self._goal_verifier = None
        self._reset_goal_lifecycle()
        update = self._submit_goal(
            lambda: self._build_absolute_base_goal(plan)
        )
        if update.outcome is BaseMovementOutcome.RUNNING:
            return BaseMovementUpdate(
                update.outcome,
                f"Correcting base goal, attempt {self._correction_attempts}/"
                f"{c.maximum_correction_attempts}; {verifier.detail}",
            )
        return update

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
        self._goal_verifier = None
        self._movement_plan = None
        self._correction_attempts = 0
        self._previous_correction_error = None
        self._operation = None
        self._movement_plan_builder = None
        self._state_wait_started = None
        self._verification_started = None

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

    @staticmethod
    def _planar_target(plan: BaseMovementPlan):
        target = plan.target
        orientation = target.pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )
        return (
            target.pose.position.x,
            target.pose.position.y,
            yaw,
        )

    def _build_absolute_base_goal(
        self,
        plan: BaseMovementPlan,
    ) -> RobotCommand.Goal:
        if not isinstance(plan, BaseMovementPlan):
            raise TypeError(
                "Absolute base goal requires a BaseMovementPlan"
            )

        target = plan.target
        x, y, yaw = self._planar_target(plan)
        profile = plan.profile
        speed = float(plan.linear_speed_mps)
        velocity_limit = SE2VelocityLimit(
            max_vel=math_helpers.SE2Velocity(
                speed,
                speed,
                profile.angular_speed_rad_s,
            ).to_proto(),
            min_vel=math_helpers.SE2Velocity(
                -speed,
                -speed,
                -profile.angular_speed_rad_s,
            ).to_proto(),
        )
        params = RobotCommandBuilder.mobility_params(
            locomotion_hint=GAITS[profile.gait],
        )
        params.vel_limit.CopyFrom(velocity_limit)

        command = (
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=x,
                goal_y=y,
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
