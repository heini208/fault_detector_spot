"""Centralized RobotCommand execution for Spot base movement."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import time

from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.navigation.body_height import (
    DEPLOYED_ARM_HEIGHT_SPEED_MPS, validate_body_height,
)
from fault_detector_spot.navigation.body_height_readiness import (
    BodyHeightReadiness, BodyHeightSource,
)

from bosdyn.api import trajectory_pb2
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration
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
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryUnavailable,
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
BASE_TAG_OBSERVATION_TIMEOUT_PARAMETER = (
    "base.tag_correction.observation_timeout_sec"
)

DEFAULT_BASE_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC = 2.0
DEFAULT_BASE_TAG_OBSERVATION_TIMEOUT_SEC = 5.0


class BaseMovementOutcome(Enum):
    """Typed outcome of one base executor lifecycle update."""

    RUNNING = "running"
    SUCCESS = "success"
    BUSY = "busy"
    ACTION_SERVER_UNAVAILABLE = "action_server_unavailable"
    GOAL_RESPONSE_TIMEOUT = "goal_response_timeout"
    GOAL_REJECTED = "goal_rejected"
    RESULT_TIMEOUT = "result_timeout"
    TAG_OBSERVATION_TIMEOUT = "tag_observation_timeout"
    MOTION_FAILED = "motion_failed"
    HEIGHT_STATE_UNAVAILABLE = "height_state_unavailable"
    HEIGHT_RESET_TIMEOUT = "height_reset_timeout"
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
    PREPARE = "prepare"
    STAND = "stand"
    CHANGE_HEIGHT = "change_height"
    SIT = "sit"


class _BaseTargetStrategy(Enum):
    FROZEN_TARGET = "frozen_target"
    FRESH_TAG_TARGET = "fresh_tag_target"


class _BasePhase(Enum):
    IDLE = "idle"
    WAITING_FOR_POSTURE = "waiting_for_posture"
    EXECUTING_STAND = "executing_stand"
    CHECKING_HEIGHT = "checking_height"
    CONFIRMING_HEIGHT = "confirming_height"
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
        height_readiness=None,
        height_reset_timeout_sec=5.0,
        tag_stability_config=None,
        tag_observation_timeout_sec: float = (
            DEFAULT_BASE_TAG_OBSERVATION_TIMEOUT_SEC
        ),
        arm_state_source=None,
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
        self.height_readiness = height_readiness or BodyHeightReadiness(
            BodyHeightSource(tf_listener, robot_name)
        )
        self.height_reset_timeout_sec = self._positive_timeout(
            height_reset_timeout_sec, "Height reset confirmation timeout",
        )
        self.posture_state_source = posture_state_source
        self.arm_state_source = arm_state_source
        # Command-space estimate, not measured body height. Startup assumes
        # nominal height; all subsequent successful stand/walk commands own it.
        self._commanded_height_m = 0.0
        self._pending_commanded_height_m = None
        self.ready_state_timeout_sec = self._positive_timeout(
            ready_state_timeout_sec,
            "Base ready state timeout",
        )
        self.ready_standing_timeout_sec = self._positive_timeout(
            ready_standing_timeout_sec,
            "Base ready standing timeout",
        )
        self.tag_observation_timeout_sec = self._positive_timeout(
            tag_observation_timeout_sec,
            "Fresh tag observation timeout",
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
        self._cancellation_terminal_update = None
        self._cancellation_complete = False

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
        def build_initial_plan():
            prepared = self.motion_planner.prepare_tag_request(semantic_command)
            plan = self.motion_planner.resolve_tag(prepared, self.tag_state_source)
            self._semantic_tag_command = prepared
            return plan

        return self._start_verified_base_movement(
            build_initial_plan,
            _BaseTargetStrategy.FRESH_TAG_TARGET,
            semantic_tag_command=semantic_command,
        )

    def prepare_for_navigation(self) -> BaseMovementUpdate:
        """Complete walking-height readiness before dispatching a Nav2 goal."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _BaseOperation.PREPARE
        self._set_phase(_BasePhase.WAITING_FOR_POSTURE)
        return self._advance_movement_start()

    def stand(self) -> BaseMovementUpdate:
        """Stand at zero offset and restore the commanded-height estimate."""
        if self.active:
            return self._busy_update()
        self._operation = _BaseOperation.STAND
        self._set_phase(_BasePhase.EXECUTING_STAND)
        return super()._start_goal(self._build_stand_goal)

    def change_height(self, body_height_m: float) -> BaseMovementUpdate:
        """Send a command-local stand height; never change walking defaults."""
        if self.active:
            return self._busy_update()
        height = validate_body_height(body_height_m)
        self.height_readiness.require_reset()
        self._operation = _BaseOperation.CHANGE_HEIGHT
        self._pending_commanded_height_m = height
        self._set_phase(_BasePhase.EXECUTING_STAND)
        return super()._start_goal(lambda: self._build_height_goal(height))

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
            self._cancellation_terminal_update = None
            if self._cancellation_complete:
                self._reset_operation()
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

        if self._phase is _BasePhase.CHECKING_HEIGHT:
            return self._check_walking_height()

        if self._phase is _BasePhase.CONFIRMING_HEIGHT:
            return self._confirm_walking_height()

        if self._phase is _BasePhase.CONFIRMING_STANDING:
            return self._poll_standing_confirmation()

        if self._phase is _BasePhase.VERIFYING_ENDPOINT:
            return self._poll_goal_verification()

        if self._phase is _BasePhase.WAITING_FOR_FRESH_TAG:
            return self._poll_fresh_tag_target()

        if self._phase is _BasePhase.CANCELLING:
            return self._poll_cancellation()

        if self._phase in {
            _BasePhase.EXECUTING_STAND,
            _BasePhase.EXECUTING_MOVEMENT,
            _BasePhase.CORRECTING,
            _BasePhase.EXECUTING_SIT,
        }:
            timeout = self._begin_timeout_cancellation_if_needed()
            if timeout is not None:
                return timeout
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

        self._set_phase(_BasePhase.CHECKING_HEIGHT)
        return self._check_walking_height()

    def _check_walking_height(self):
        sample = self.height_readiness.sample(self._ros_time_sec)
        if sample is None:
            if self._deadline_expired(self._phase_started, self.ready_state_timeout_sec):
                return self._finish(
                    BaseMovementOutcome.HEIGHT_STATE_UNAVAILABLE,
                    "Walking height unavailable: need fresh feet_center-to-body TF; "
                    + self.height_readiness.last_error,
                )
            return BaseMovementUpdate(BaseMovementOutcome.RUNNING,
                                      "Waiting for measured body height")
        state = self._fresh_posture_state()
        if state is not PostureState.STANDING:
            return self._finish(
                self._posture_state_failure_outcome(state),
                "Standing posture was lost while checking walking height",
            )
        if self.height_readiness.at_nominal_height(sample):
            return self._walking_height_ready()
        self.height_readiness.require_reset()
        self._set_phase(_BasePhase.EXECUTING_STAND)
        return self._submit_goal(self._build_stand_goal)

    def _confirm_walking_height(self):
        if (self._fresh_posture_state() is PostureState.STANDING
                and self.height_readiness.confirm_reset(self._ros_time_sec)):
            return self._walking_height_ready()
        if self._deadline_expired(self._phase_started, self.height_reset_timeout_sec):
            return self._finish(
                BaseMovementOutcome.HEIGHT_RESET_TIMEOUT,
                "Default-height stand did not produce fresh, settled walking height",
            )
        return BaseMovementUpdate(BaseMovementOutcome.RUNNING,
                                  "Waiting for measured walking height to settle")

    def _walking_height_ready(self):
        self._commanded_height_m = 0.0
        if self._operation is _BaseOperation.PREPARE:
            return self._finish(BaseMovementOutcome.SUCCESS, "Walking height ready")
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

    def _submit_goal(self, goal_builder):
        if self._operation is not _BaseOperation.CHANGE_HEIGHT:
            # Stand/reset, sit, and every walk use nominal standing offset.
            self._pending_commanded_height_m = 0.0
        return super()._submit_goal(goal_builder)

    def _handle_successful_result(self, result):
        if self._pending_commanded_height_m is not None:
            self._commanded_height_m = self._pending_commanded_height_m
            self._pending_commanded_height_m = None
        if self._phase is _BasePhase.EXECUTING_STAND:
            if self._operation not in {
                _BaseOperation.MOVEMENT, _BaseOperation.PREPARE,
            }:
                return super()._handle_successful_result(result)

            self._reset_goal_lifecycle()
            # The action reports completion of the stand trajectory. Require
            # newer measured samples too; a cached standing flag is insufficient.
            self.height_readiness.begin_confirmation(self._ros_time_sec())
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
                motion_timeout_sec=self.result_timeout_sec,
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
        try:
            observation = lookup(tag_id, boundary)
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                "Fresh tag observation lookup failed: "
                f"{exception}",
            )

        if observation is None:
            return self._wait_for_fresh_tag(
                f"Waiting for post-settle tag {tag_id} observation",
                BaseMovementOutcome.TAG_OBSERVATION_TIMEOUT,
            )

        try:
            observation = deepcopy(observation)
            observation.pose = self.motion_planner.observation_in_odom(
                observation.pose,
            )
        except MovementGeometryUnavailable as exception:
            return self._wait_for_fresh_tag(
                f"Waiting for capture-time tag transform: {exception}",
                BaseMovementOutcome.TAG_OBSERVATION_TIMEOUT,
            )
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.EXECUTION_ERROR,
                f"Tag stability transform failed: {exception}",
            )

        stable = self._tag_observation_tracker.update(
            observation,
            boundary,
        )
        if stable is None:
            return self._wait_for_fresh_tag(
                "Waiting for stable post-settle "
                f"tag {tag_id} observation "
                f"({self._tag_observation_tracker.sample_count}/"
                f"{self.tag_stability_config.required_samples} samples); "
                f"{self._tag_observation_tracker.detail}",
                BaseMovementOutcome.TAG_OBSERVATION_TIMEOUT,
            )

        sample = self.base_pose_source.sample()
        pose = sample.planar_pose if sample is not None else None
        stamp = sample.stamp_sec if sample is not None else None
        if not BaseGoalVerifier.sample_is_fresh(
            pose,
            stamp,
            self._ros_time_sec(),
            self.goal_verification_config.maximum_pose_age_sec,
        ):
            return self._wait_for_fresh_tag(
                "Waiting for fresh measured base pose before "
                "tag re-planning",
                BaseMovementOutcome.MOTION_FAILED,
            )
        try:
            fresh_plan = self.motion_planner.resolve_tag_observation(
                command,
                stable,
            )
        except MovementGeometryUnavailable as exception:
            return self._wait_for_fresh_tag(
                f"Waiting for tag re-planning geometry: {exception}",
                BaseMovementOutcome.MOTION_FAILED,
            )
        except Exception as exception:
            return self._finish(
                BaseMovementOutcome.MOTION_FAILED,
                f"Fresh tag re-planning failed: {exception}",
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

    def _wait_for_fresh_tag(self, detail, timeout_outcome):
        if self._deadline_expired(
            self._phase_started,
            self.tag_observation_timeout_sec,
        ):
            return self._finish(
                timeout_outcome,
                f"{detail}; timed out after "
                f"{self.tag_observation_timeout_sec:.1f} s",
            )
        return BaseMovementUpdate(
            BaseMovementOutcome.RUNNING,
            detail,
        )

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

    def _begin_timeout_cancellation_if_needed(self):
        if (
            self._goal_handle is None
            and self._send_goal_future is not None
            and not self._send_goal_future.done()
            and self._deadline_expired(
                self._goal_sent_monotonic,
                self.goal_response_timeout_sec,
            )
        ):
            return self._begin_timeout_cancellation(
                BaseMovementOutcome.GOAL_RESPONSE_TIMEOUT,
                "Action goal response timed out after "
                f"{self.goal_response_timeout_sec:.1f} s",
            )

        if (
            self._goal_handle is not None
            and self._result_future is not None
            and not self._result_future.done()
            and self._deadline_expired(
                self._result_started_monotonic,
                self.result_timeout_sec,
            )
        ):
            return self._begin_timeout_cancellation(
                BaseMovementOutcome.RESULT_TIMEOUT,
                "Action result timed out after "
                f"{self.result_timeout_sec:.1f} s",
            )
        return None

    def _begin_timeout_cancellation(self, outcome, detail):
        self._cancellation_terminal_update = BaseMovementUpdate(
            outcome,
            detail,
        )
        self._cancellation_complete = False
        self._pending_goal_builder = None
        self._goal_verifier = None
        self._set_phase(_BasePhase.CANCELLING)
        self._begin_cancellation()
        return self._poll_cancellation()

    def _poll_cancellation(self):
        terminal = self._cancellation_terminal_update
        if self._cancellation_complete and terminal is not None:
            return self._finish(
                terminal.outcome,
                terminal.detail,
            )
        return BaseMovementUpdate(
            BaseMovementOutcome.RUNNING,
            "Cancelling base movement",
        )

    def _complete_cancellation_lifecycle(self) -> None:
        if self._cancellation_terminal_update is None:
            self._reset_operation()
            return
        self._cancellation_complete = True

    def _begin_cancellation(self) -> None:
        send_future = self._send_goal_future
        if send_future is None:
            self._complete_cancellation_lifecycle()
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
            self._complete_cancellation_lifecycle()
            return

        if handle is None or not handle.accepted:
            self._complete_cancellation_lifecycle()
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
        self._complete_cancellation_lifecycle()

    def _poll_standing_confirmation(self) -> BaseMovementUpdate:
        state = self._fresh_posture_state()
        if state is PostureState.STANDING:
            self._set_phase(_BasePhase.CONFIRMING_HEIGHT)
            return self._confirm_walking_height()

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

    def _finish(self, outcome, detail):
        if outcome is BaseMovementOutcome.GOAL_REJECTED:
            # A rejected goal cannot have changed the height.
            self._pending_commanded_height_m = None
        return super()._finish(outcome, detail)

    def _reset_operation(self) -> None:
        if (self._pending_commanded_height_m is not None
                and self._send_goal_future is not None):
            # Cancellation, timeout, or failure can leave an intermediate pose.
            self._commanded_height_m = None
        self._pending_commanded_height_m = None
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
        self._cancellation_terminal_update = None
        self._cancellation_complete = False
        self._tag_observation_tracker.reset()

    def _build_height_goal(self, body_height_m) -> RobotCommand.Goal:
        state = (
            self.arm_state_source.stow_state()
            if self.arm_state_source is not None else None
        )
        # Only a freshly confirmed stowed arm permits the default fast change.
        if state is ArmStowState.STOWED:
            return self._build_stand_goal(body_height_m)
        current_offset = self._commanded_height_m
        if current_offset is None:
            raise ValueError(
                "Height command was interrupted; complete Stand to restore "
                "the commanded-height reference"
            )
        duration_sec = (
            abs(body_height_m - current_offset) / DEPLOYED_ARM_HEIGHT_SPEED_MPS
        )
        return self._build_stand_goal(body_height_m, duration_sec, current_offset)

    def _build_stand_goal(
        self, body_height_m=0.0, duration_sec=0, start_height_m=None,
    ) -> RobotCommand.Goal:
        params = RobotCommandBuilder.mobility_params(body_height=body_height_m)
        if duration_sec:
            if start_height_m is None:
                raise ValueError("Timed body height trajectory requires a starting height")
            trajectory = params.body_control.base_offset_rt_footprint
            target = trajectory.points.add()
            target.CopyFrom(trajectory.points[0])
            trajectory.points[0].pose.position.z = start_height_m
            target.time_since_reference.CopyFrom(seconds_to_duration(duration_sec))
            trajectory.pos_interpolation = trajectory_pb2.POS_INTERP_LINEAR
        command = RobotCommandBuilder.synchro_stand_command(
            params=params,
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal

    def _build_sit_goal(self) -> RobotCommand.Goal:
        command = RobotCommandBuilder.synchro_sit_command()
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal



__all__ = [
    "BASE_TAG_OBSERVATION_TIMEOUT_PARAMETER",
    "DEFAULT_BASE_TAG_OBSERVATION_TIMEOUT_SEC",
    "BaseMovementExecutor",
    "BaseMovementOutcome",
    "BaseMovementUpdate",
]
