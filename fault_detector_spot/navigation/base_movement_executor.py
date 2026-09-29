"""Centralized RobotCommand execution for Spot base movement."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import time

from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from spot_msgs.action import RobotCommand

from fault_detector_spot.navigation.base_correction_policy import (
    BaseCorrectionDecision,
    BaseCorrectionPolicy,
)
from fault_detector_spot.navigation.base_goal_verifier import (
    BaseGoalVerifier,
    BaseGoalVerificationConfig,
)
from fault_detector_spot.navigation.base_motion_planner import (
    BaseMotionPlanner,
    BaseMovementPlan,
)
from fault_detector_spot.navigation.base_pose_source import BasePoseSource
from fault_detector_spot.navigation.posture_state_source import (
    PostureState,
)
from fault_detector_spot.navigation.walking_profile import WalkingProfiles
from fault_detector_spot.sensing.observations.tag_observation_stability import (
    StableTagObservationTracker,
    TagObservationStabilityConfig,
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
    STAND = "stand"
    SIT = "sit"


class _BaseTargetStrategy(Enum):
    FROZEN_TARGET = "frozen_target"
    FRESH_TAG_TARGET = "fresh_tag_target"


class _BasePhase(Enum):
    IDLE = "idle"
    WAITING_FOR_POSTURE = "waiting_for_posture"
    EXECUTING_STAND = "executing_stand"
    CONFIRMING_STANDING = "confirming_standing"
    EXECUTING_MOVEMENT = "executing_movement"
    VERIFYING_ENDPOINT = "verifying_endpoint"
    WAITING_FOR_FRESH_TAG = "waiting_for_fresh_tag"
    CORRECTING = "correcting"
    CANCELLING = "cancelling"
    EXECUTING_SIT = "executing_sit"


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
        base_pose_source=None,
        correction_policy=None,
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
        tag_stability_config=None,
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
            goal_verification_config
            or BaseGoalVerificationConfig()
        )
        self.correction_policy = (
            correction_policy or BaseCorrectionPolicy()
        )
        if not isinstance(
            self.correction_policy,
            BaseCorrectionPolicy,
        ):
            raise TypeError(
                "BaseMovementExecutor requires BaseCorrectionPolicy"
            )

        self.walking_profiles = walking_profiles or WalkingProfiles()
        self.tag_stability_config = (
            tag_stability_config or TagObservationStabilityConfig()
        )
        self._tag_observation_tracker = StableTagObservationTracker(
            self.tag_stability_config
        )
        self.motion_planner = BaseMotionPlanner(
            tf_listener,
            self.walking_profiles,
        )
        self.base_pose_source = (
            base_pose_source or BasePoseSource(tf_listener)
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
        self._operation = None
        self._phase = _BasePhase.IDLE
        self._phase_started = None
        self._movement_plan_builder = None
        self._target_strategy = None
        self._semantic_tag_command = None
        self._tag_settle_boundary_stamp = None

    def relative(self, command) -> BaseMovementUpdate:
        """Start a relative SE2 base movement."""
        return self._start_verified_base_movement(
            lambda: self.motion_planner.resolve_relative(command),
            _BaseTargetStrategy.FROZEN_TARGET,
        )

    def tag(self, command) -> BaseMovementUpdate:
        """Start an SE2 base movement relative to a live visible tag."""
        if self.active:
            return self._busy_update()
        semantic_command = deepcopy(command)
        return self._start_verified_base_movement(
            lambda: self.motion_planner.resolve_tag(
                semantic_command,
                self.tag_state_source,
            ),
            _BaseTargetStrategy.FRESH_TAG_TARGET,
            semantic_tag_command=semantic_command,
        )

    def stand(self) -> BaseMovementUpdate:
        """Start Spot's native stand command directly."""
        if self.active:
            return self._busy_update()
        self._operation = _BaseOperation.STAND
        self._set_phase(_BasePhase.EXECUTING_STAND)
        return super()._start_goal(self._build_stand_goal)

    def sit(self) -> BaseMovementUpdate:
        """Start Spot's native sit command directly."""
        if self.active:
            return self._busy_update()
        self._operation = _BaseOperation.SIT
        self._set_phase(_BasePhase.EXECUTING_SIT)
        return super()._start_goal(self._build_sit_goal)

    def cancel(self) -> None:
        """Cancel an active base command without releasing ownership early."""
        if not self.active:
            return
        if self._phase is _BasePhase.CANCELLING:
            return
        if (
            self._send_goal_future is None
            or (
                self._result_future is not None
                and self._result_future.done()
            )
        ):
            self._reset_operation()
            return

        self._pending_goal_builder = None
        self._goal_verifier = None
        self._set_phase(_BasePhase.CANCELLING)
        self._begin_cancellation()

    def poll(self) -> BaseMovementUpdate:
        """Advance the active base operation without blocking."""
        if not self.active or self._operation is None:
            return BaseMovementUpdate(
                BaseMovementOutcome.EXECUTION_ERROR,
                "No base movement is active",
            )

        if self._phase is _BasePhase.WAITING_FOR_POSTURE:
            return self._advance_movement_start()

        if self._phase is _BasePhase.CONFIRMING_STANDING:
            return self._poll_standing_confirmation()

        if self._phase is _BasePhase.VERIFYING_ENDPOINT:
            return self._poll_goal_verification()

        if self._phase is _BasePhase.WAITING_FOR_FRESH_TAG:
            return self._poll_fresh_tag_target()

        if self._phase is _BasePhase.CANCELLING:
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Cancelling base movement",
            )

        if self._phase in {
            _BasePhase.EXECUTING_STAND,
            _BasePhase.EXECUTING_MOVEMENT,
            _BasePhase.CORRECTING,
            _BasePhase.EXECUTING_SIT,
        }:
            return super().poll()

        return self._finish(
            BaseMovementOutcome.EXECUTION_ERROR,
            f"Unexpected active base phase '{self._phase.value}'",
        )

    def _start_verified_base_movement(
        self,
        plan_builder,
        target_strategy: _BaseTargetStrategy,
        semantic_tag_command=None,
    ) -> BaseMovementUpdate:
        """Start one movement whose plan is resolved after readiness."""
        if self.active:
            return self._busy_update()
        if not callable(plan_builder):
            raise TypeError("Base movement requires a plan builder")
        if not isinstance(target_strategy, _BaseTargetStrategy):
            raise TypeError(
                "Base movement requires an explicit target strategy"
            )
        if (
            target_strategy is _BaseTargetStrategy.FRESH_TAG_TARGET
            and semantic_tag_command is None
        ):
            raise ValueError(
                "Fresh tag movement requires its semantic command"
            )

        self.correction_policy.reset()
        self._tag_observation_tracker.reset()
        self._active = True
        self._operation = _BaseOperation.MOVEMENT
        self._movement_plan_builder = plan_builder
        self._target_strategy = target_strategy
        self._semantic_tag_command = semantic_tag_command
        self._set_phase(_BasePhase.WAITING_FOR_POSTURE)
        return self._advance_movement_start()

    def _advance_movement_start(self) -> BaseMovementUpdate:
        state = self._fresh_posture_state()
        if state is None or state is PostureState.UNKNOWN:
            return self._wait_for_posture_state()

        if state is PostureState.SITTING:
            self._set_phase(_BasePhase.EXECUTING_STAND)
            return self._submit_goal(self._build_stand_goal)

        return self._submit_movement_goal()

    def _submit_movement_goal(self) -> BaseMovementUpdate:
        plan_builder = self._movement_plan_builder
        if plan_builder is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Base movement has no pending plan builder",
            )
        return self._submit_movement_plan(
            plan_builder,
            _BasePhase.EXECUTING_MOVEMENT,
        )

    def _submit_movement_plan(
        self,
        plan_builder,
        phase: _BasePhase,
    ) -> BaseMovementUpdate:
        """Resolve and submit one direct planar movement plan."""
        if not callable(plan_builder):
            raise TypeError(
                "Base movement plan submission requires a plan builder"
            )
        if phase not in {
            _BasePhase.EXECUTING_MOVEMENT,
            _BasePhase.CORRECTING,
        }:
            raise ValueError(
                "Base movement plan submission requires an execution phase"
            )

        def build_goal():
            plan = plan_builder()
            if not isinstance(plan, BaseMovementPlan):
                raise TypeError(
                    "Base movement plan builder must return BaseMovementPlan"
                )
            self._movement_plan = plan
            return self.motion_planner.build_goal(
                plan,
                self.robot_name,
            )

        self._set_phase(phase)
        self._pending_goal_builder = build_goal
        return self._submit_goal(build_goal)

    def _handle_successful_result(self, result):
        if self._phase is _BasePhase.EXECUTING_STAND:
            if self._operation is not _BaseOperation.MOVEMENT:
                return super()._handle_successful_result(result)

            self._reset_goal_lifecycle()
            self._set_phase(_BasePhase.CONFIRMING_STANDING)
            return self._poll_standing_confirmation()

        if self._phase in {
            _BasePhase.EXECUTING_MOVEMENT,
            _BasePhase.CORRECTING,
        }:
            plan = self._movement_plan
            if plan is None:
                return self._finish(
                    BaseMovementOutcome.EXECUTION_ERROR,
                    "Base movement has no plan for endpoint verification",
                )
            self._goal_verifier = BaseGoalVerifier(
                self.motion_planner.planar_target(plan),
                self.goal_verification_config,
                self._monotonic_clock(),
            )
            self._tag_observation_tracker.reset()
            self._set_phase(_BasePhase.VERIFYING_ENDPOINT)
            return self._poll_goal_verification()

        return super()._handle_successful_result(result)

    def _poll_goal_verification(self):
        verifier = self._goal_verifier
        if verifier is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Base endpoint verification has no verifier",
            )

        sample = self.base_pose_source.sample()
        pose = sample.planar_pose if sample is not None else None
        stamp = sample.stamp_sec if sample is not None else None
        outcome = verifier.update(
            pose,
            stamp,
            self._ros_time_sec(),
            self._monotonic_clock(),
        )

        if self._target_strategy is _BaseTargetStrategy.FRESH_TAG_TARGET:
            if verifier.settled:
                boundary = verifier.settled_stamp
                if boundary is None:
                    return self._finish(
                        BaseMovementOutcome.EXECUTION_ERROR,
                        "Settled base verification has no timestamp",
                    )
                self._tag_settle_boundary_stamp = boundary
                self._goal_verifier = None
                self._tag_observation_tracker.reset()
                self._set_phase(_BasePhase.WAITING_FOR_FRESH_TAG)
                return self._poll_fresh_tag_target()
            if outcome is False:
                return self._finish(
                    BaseMovementOutcome.MOTION_FAILED,
                    "Base did not settle before fresh tag verification; "
                    f"{verifier.detail}",
                )
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                verifier.detail,
            )

        if outcome is True:
            return self._finish(
                BaseMovementOutcome.SUCCESS,
                f"Base goal verified and settled; {verifier.detail}",
            )
        if outcome is False:
            correction = self._correct_frozen_base_goal(verifier)
            if correction is not None:
                return correction
            self._request_cancel()
            return self._finish(
                BaseMovementOutcome.MOTION_FAILED,
                f"Base endpoint verification timed out; {verifier.detail}",
            )
        return BaseMovementUpdate(
            BaseMovementOutcome.RUNNING,
            verifier.detail,
        )

    def _poll_fresh_tag_target(self):
        command = self._semantic_tag_command
        source = self.tag_state_source
        boundary = self._tag_settle_boundary_stamp
        if command is None or boundary is None:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Fresh tag correction is missing execution state",
            )
        lookup = getattr(source, "visible_tag_after", None)
        if not callable(lookup):
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Tag state source cannot provide post-settle observations",
            )

        tag_id = int(command.tag_id)
        observation = lookup(tag_id, boundary)
        stable = self._tag_observation_tracker.update(
            observation,
            boundary,
        )
        if stable is None:
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Waiting for stable post-settle "
                f"tag {tag_id} observation",
            )

        sample = self.base_pose_source.sample()
        pose = sample.planar_pose if sample is not None else None
        if pose is None:
            return BaseMovementUpdate(
                BaseMovementOutcome.RUNNING,
                "Waiting for measured base pose before tag re-planning",
            )

        fresh_plan = self.motion_planner.resolve_tag_observation(
            command,
            stable,
        )
        fresh_target = self.motion_planner.planar_target(fresh_plan)
        error = BaseGoalVerifier.errors(pose, fresh_target)
        config = self.goal_verification_config
        if (
            error[0] <= config.position_tolerance_m
            and error[1] <= config.yaw_tolerance_rad
        ):
            return self._finish(
                BaseMovementOutcome.SUCCESS,
                "Fresh tag-relative target verified after settling; "
                f"error {error[0]:.4f} m, "
                f"{error[1]:.4f} rad",
            )

        result = self.correction_policy.decide(
            error,
            config.position_tolerance_m,
            config.yaw_tolerance_rad,
        )
        if result.decision is not BaseCorrectionDecision.CORRECT:
            detail = f"; {result.detail}" if result.detail else ""
            return self._finish(
                BaseMovementOutcome.MOTION_FAILED,
                "Fresh tag-relative target remains outside tolerance"
                f"{detail}",
            )

        self._tag_observation_tracker.reset()
        self._reset_goal_lifecycle()
        update = self._submit_movement_plan(
            lambda: fresh_plan,
            _BasePhase.CORRECTING,
        )
        if update.outcome is BaseMovementOutcome.RUNNING:
            return BaseMovementUpdate(
                update.outcome,
                "Correcting toward fresh tag-relative target, attempt "
                f"{result.attempt}/"
                f"{self.correction_policy.config.maximum_attempts}",
            )
        return update

    def _correct_frozen_base_goal(self, verifier):
        """Apply correction policy and retry the current frozen plan."""
        plan = self._movement_plan
        if plan is None:
            return None

        config = self.goal_verification_config
        result = self.correction_policy.decide(
            verifier.current_error,
            config.position_tolerance_m,
            config.yaw_tolerance_rad,
        )
        if result.detail:
            verifier.detail += f"; {result.detail}"
        if result.decision is not BaseCorrectionDecision.CORRECT:
            return None

        self._goal_verifier = None
        self._reset_goal_lifecycle()
        update = self._submit_movement_plan(
            lambda: plan,
            _BasePhase.CORRECTING,
        )
        if update.outcome is BaseMovementOutcome.RUNNING:
            return BaseMovementUpdate(
                update.outcome,
                "Correcting base goal, attempt "
                f"{result.attempt}/"
                f"{self.correction_policy.config.maximum_attempts}; "
                f"{verifier.detail}",
            )
        return update

    def _begin_cancellation(self) -> None:
        send_future = self._send_goal_future
        if send_future is None:
            self._reset_operation()
            return

        if self._goal_handle is not None:
            self._cancel_accepted_goal(self._goal_handle)
            return

        if send_future.done():
            self._cancel_after_goal_response(send_future)
        else:
            send_future.add_done_callback(
                self._cancel_after_goal_response
            )

    def _cancel_after_goal_response(self, send_future) -> None:
        if (
            not self.active
            or self._phase is not _BasePhase.CANCELLING
            or self._send_goal_future is not send_future
        ):
            return

        try:
            handle = send_future.result()
        except Exception as exception:
            self._log_error(
                "Base goal submission finished during cancellation with "
                f"an error: {exception}"
            )
            self._reset_operation()
            return

        if handle is None or not handle.accepted:
            self._reset_operation()
            return

        self._goal_handle = handle
        self._cancel_accepted_goal(handle)

    def _cancel_accepted_goal(self, handle) -> None:
        if (
            not self.active
            or self._phase is not _BasePhase.CANCELLING
            or self._goal_handle is not handle
        ):
            return

        result_future = self._result_future
        if result_future is None:
            try:
                result_future = handle.get_result_async()
            except Exception as exception:
                self._log_error(
                    "Base result request failed during cancellation: "
                    f"{exception}"
                )
                result_future = None
            self._result_future = result_future

        try:
            handle.cancel_goal_async()
        except Exception as exception:
            self._log_error(
                "Base goal cancellation failed: "
                f"{exception}"
            )

        if result_future is None:
            return

        if result_future.done():
            self._complete_cancellation(result_future)
        else:
            result_future.add_done_callback(
                self._complete_cancellation
            )

    def _complete_cancellation(self, result_future) -> None:
        if (
            not self.active
            or self._phase is not _BasePhase.CANCELLING
            or self._result_future is not result_future
        ):
            return
        self._reset_operation()

    def _poll_standing_confirmation(self) -> BaseMovementUpdate:
        state = self._fresh_posture_state()
        if state is PostureState.STANDING:
            return self._submit_movement_goal()

        if not self._deadline_expired(
            self._phase_started,
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
        if not self._deadline_expired(
            self._phase_started,
            self.ready_state_timeout_sec,
        ):
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

    def _set_phase(self, phase: _BasePhase) -> None:
        if not isinstance(phase, _BasePhase):
            raise TypeError("Base execution phase must be a _BasePhase")
        self._phase = phase
        self._phase_started = (
            None
            if phase is _BasePhase.IDLE
            else self._monotonic_clock()
        )

    def _reset_operation(self) -> None:
        super()._reset_operation()
        self.correction_policy.reset()
        self._goal_verifier = None
        self._movement_plan = None
        self._operation = None
        self._phase = _BasePhase.IDLE
        self._phase_started = None
        self._movement_plan_builder = None
        self._target_strategy = None
        self._semantic_tag_command = None
        self._tag_settle_boundary_stamp = None
        self._tag_observation_tracker.reset()

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



__all__ = [
    "BaseMovementExecutor",
    "BaseMovementOutcome",
    "BaseMovementUpdate",
]
