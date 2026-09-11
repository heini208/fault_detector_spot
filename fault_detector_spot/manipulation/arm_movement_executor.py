"""Centralized Spot arm movement execution."""

from copy import deepcopy
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
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
)
from fault_detector_spot.manipulation.guarded_probe_execution import (
    GuardedProbeExecution,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlanner,
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
FORCE_STALE_TIMEOUT_PARAMETER = "arm.contact.force_stale_timeout_sec"
CONTACT_RETREAT_DISTANCE_PARAMETER = "arm.contact.retreat_distance_m"
CONTACT_RETREAT_SPEED_PARAMETER = "arm.contact.retreat_speed_mps"

DEFAULT_READY_LIFT_DISTANCE_M = 0.10
DEFAULT_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_READY_TF_TIMEOUT_SEC = 2.0
DEFAULT_READY_DEPLOYED_TIMEOUT_SEC = 2.0
DEFAULT_STOW_STATE_TIMEOUT_SEC = 2.0
DEFAULT_FORCE_STALE_TIMEOUT_SEC = 0.25
DEFAULT_CONTACT_RETREAT_DISTANCE_M = 0.010
DEFAULT_CONTACT_RETREAT_SPEED_MPS = 0.010


class _ArmOperation:
    MOVEMENT = "movement"
    GUARDED_MOVEMENT = "guarded_movement"
    GUARDED_PREPARE = "guarded_prepare"
    PREPARE = "prepare"
    STOW = "stow"


class ArmMovementExecutor(MovementExecutor):
    """Coordinate readiness, guarded probe motion, and low-level probe motion."""

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
        settling_detector=None,
        force_baseline_sampler=None,
        force_contact_policy=None,
        force_stale_timeout_sec: float = DEFAULT_FORCE_STALE_TIMEOUT_SEC,
        contact_retreat_distance_m: float = (
            DEFAULT_CONTACT_RETREAT_DISTANCE_M
        ),
        contact_retreat_speed_mps: float = DEFAULT_CONTACT_RETREAT_SPEED_MPS,
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
        self.force_contact_policy = force_contact_policy
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
        self.probe_motion_planner = ProbeMotionPlanner(
            tf_listener=tf_listener,
            speed_policy=self.speed_policy,
            build_pose_goal=self._build_pose_goal,
        )

        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None
        self._guarded_plan_builder = None
        self._guarded_force_threshold_n = None

        self.guarded_probe_execution = None
        if (
            arm_state_source is not None
            and settling_detector is not None
            and force_baseline_sampler is not None
            and force_contact_policy is not None
        ):
            self.guarded_probe_execution = GuardedProbeExecution(
                arm_state_source=arm_state_source,
                settling_detector=settling_detector,
                force_baseline_sampler=force_baseline_sampler,
                force_contact_policy=force_contact_policy,
                start_goal=self._guard_start_goal,
                poll_goal=self._guard_poll_goal,
                cancel_goal=self._guard_cancel_goal,
                current_hand_pose=self.probe_motion_planner.current_hand_pose,
                build_motion_goal=self.probe_motion_planner.build_motion_goal,
                default_angular_speed_rad_s=(
                    self.speed_policy.default_speed.angular_speed_rad_s
                ),
                force_stale_timeout_sec=force_stale_timeout_sec,
                retreat_distance_m=contact_retreat_distance_m,
                retreat_speed_mps=contact_retreat_speed_mps,
                monotonic_clock=monotonic_clock,
            )

    def relative(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a relative hand target and execute it through the guard."""
        if self.probe_motion_planner.relative_command_is_noop(command):
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Skipped zero arm movement",
            )
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_relative(command),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def pose(
        self,
        target: PoseStamped,
        execution_frame: str = "",
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Execute an absolute bare-hand target through the guard."""
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_absolute(
                target,
                execution_frame,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def probe_pose(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Compatibility entry for guarded absolute probe motion."""
        return self.guarded_probe(
            probe_target,
            motion_sensor_id,
            speed,
            force_threshold_n=force_threshold_n,
        )

    def tag_probe(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a live tag target and execute it through the guard."""
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_tag(
                command,
                self.tag_state_source,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a probe-relative target and execute it through the guard."""
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_probe_relative(
                offset,
                motion_sensor_id,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def guarded_probe(
        self,
        probe_target,
        motion_sensor_id: str = "",
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Start the single guarded entry point for probe motion."""
        if callable(probe_target):
            if str(motion_sensor_id).strip():
                raise ValueError(
                    "Guarded probe target builders must resolve their "
                    "own motion sensor ID"
                )
            target_builder = probe_target
        else:
            target = deepcopy(probe_target)
            sensor_id = str(motion_sensor_id).strip()
            target_builder = lambda: (target, sensor_id)
        return self._start_guarded_probe(
            target_builder,
            speed,
            force_threshold_n,
        )

    def probe(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start one low-level unguarded probe movement."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.MOVEMENT
        return self._submit_probe(
            probe_target,
            motion_sensor_id,
            speed,
        )

    def prepare(
        self,
        speed=None,
    ) -> ArmMovementUpdate:
        """Prepare a stowed arm with an unguarded controlled lift."""
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

        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            return self._poll_guarded_probe()

        if self._send_goal_future is None:
            if self._operation in (
                _ArmOperation.PREPARE,
                _ArmOperation.GUARDED_PREPARE,
            ):
                return self._advance_prepare_start()
            if self._operation == _ArmOperation.STOW:
                return self._advance_stow_start()
            return super().poll()

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def cancel(self) -> None:
        if (
            self._operation == _ArmOperation.GUARDED_MOVEMENT
            and self.guarded_probe_execution is not None
        ):
            self.guarded_probe_execution.cancel()
        super().cancel()

    def _start_guarded_probe(
        self,
        target_builder,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        if self.active:
            return self._busy_update()
        if self.guarded_probe_execution is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement is not configured",
            )
        if self.force_contact_policy is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement requires a force contact policy",
            )

        self._active = True
        self._operation = _ArmOperation.GUARDED_MOVEMENT
        self._operation_speed = speed
        self._guarded_force_threshold_n = force_threshold_n
        self._guarded_plan_builder = lambda: (
            self.probe_motion_planner.build_plan(
                target_builder,
                speed,
            )
        )
        return self._advance_guarded_readiness()

    def _advance_guarded_readiness(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.ready_state_timeout_sec,
                "Waiting for manipulator stow state before guarded movement",
            )

        self._state_wait_started = None
        if state is ArmStowState.STOWED:
            self._operation = _ArmOperation.GUARDED_PREPARE
            self._operation_speed = None
            return self._advance_prepare_start()

        return self._begin_guarded_probe()

    def _begin_guarded_probe(self) -> ArmMovementUpdate:
        self._operation = _ArmOperation.GUARDED_MOVEMENT
        builder = self._guarded_plan_builder
        if builder is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no pending target",
            )
        update = self.guarded_probe_execution.start(
            builder,
            force_threshold_n=self._guarded_force_threshold_n,
        )
        return self._finish_guarded_update(update)

    def _poll_guarded_probe(self) -> ArmMovementUpdate:
        update = self.guarded_probe_execution.poll()
        return self._finish_guarded_update(update)

    def _finish_guarded_update(
        self,
        update: ArmMovementUpdate,
    ) -> ArmMovementUpdate:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        if not self.active:
            return update
        return self._finish(update.outcome, update.detail)

    def _guard_start_goal(self, goal) -> ArmMovementUpdate:
        return self._submit_goal(lambda: goal)

    def _guard_poll_goal(self) -> ArmMovementUpdate:
        if self._send_goal_future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no active RobotCommand goal",
            )
        if self._goal_handle is None:
            return self._poll_goal_response()
        return self._poll_result()

    def _guard_cancel_goal(self) -> None:
        self._request_cancel()
        self._reset_goal_lifecycle()

    def _handle_successful_result(self, result):
        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            self._reset_goal_lifecycle()
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Succeeded",
            )

        if self._operation in (
            _ArmOperation.PREPARE,
            _ArmOperation.GUARDED_PREPARE,
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
        self._guarded_plan_builder = None
        self._guarded_force_threshold_n = None
        if self.guarded_probe_execution is not None:
            self.guarded_probe_execution.reset()

    def _advance_prepare_start(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.ready_state_timeout_sec,
                "Waiting for manipulator stow state",
            )

        self._state_wait_started = None
        if state is ArmStowState.DEPLOYED:
            if self._operation == _ArmOperation.GUARDED_PREPARE:
                return self._begin_guarded_probe()
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Arm is already deployed",
            )

        try:
            current_hand = self.probe_motion_planner.current_hand_pose()
        except Exception as exception:
            return self._wait_for_ready_transform(exception)

        self._tf_wait_started = None
        target_hand = deepcopy(current_hand)
        target_hand.pose.position.z += self.ready_lift_distance_m

        return self._submit_probe(
            target_hand,
            BARE_HAND_MOTION_ID,
            self._operation_speed,
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
        if self._operation in (
            _ArmOperation.PREPARE,
            _ArmOperation.GUARDED_PREPARE,
        ):
            expected = ArmStowState.DEPLOYED
            timeout_sec = self.ready_deployed_timeout_sec
            success_detail = "Arm deployed"
            failure_detail = (
                "Ready arm movement completed, but Spot did not report "
                f"DEPLOYED within {timeout_sec:.1f} s"
            )
        elif self._operation == _ArmOperation.STOW:
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
            if self._operation == _ArmOperation.GUARDED_PREPARE:
                self._verification_started = None
                return self._begin_guarded_probe()
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

    def _submit_probe(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        return self._submit_goal(
            lambda: self.probe_motion_planner.build_probe_goal(
                probe_target,
                motion_sensor_id,
                speed,
            )
        )

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

    def _build_pose_goal(
        self,
        target: PoseStamped,
        duration_sec: float,
    ) -> RobotCommand.Goal:
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
    "CONTACT_RETREAT_DISTANCE_PARAMETER",
    "CONTACT_RETREAT_SPEED_PARAMETER",
    "DEFAULT_CONTACT_RETREAT_DISTANCE_M",
    "DEFAULT_CONTACT_RETREAT_SPEED_MPS",
    "DEFAULT_FORCE_STALE_TIMEOUT_SEC",
    "DEFAULT_READY_DEPLOYED_TIMEOUT_SEC",
    "DEFAULT_READY_LIFT_DISTANCE_M",
    "DEFAULT_READY_STATE_TIMEOUT_SEC",
    "DEFAULT_READY_TF_TIMEOUT_SEC",
    "DEFAULT_STOW_STATE_TIMEOUT_SEC",
    "FORCE_STALE_TIMEOUT_PARAMETER",
    "READY_DEPLOYED_TIMEOUT_PARAMETER",
    "READY_LIFT_DISTANCE_PARAMETER",
    "READY_STATE_TIMEOUT_PARAMETER",
    "READY_TF_TIMEOUT_PARAMETER",
    "STOW_STATE_TIMEOUT_PARAMETER",
]
