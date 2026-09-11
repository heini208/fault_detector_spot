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
import tf2_geometry_msgs

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
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
    GuardedProbePlan,
)
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryResolver,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import (
    transform_to_pose_data,
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
        self.geometry_resolver = MovementGeometryResolver(tf_listener)

        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None
        self._guarded_plan_builder = None

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
                start_goal=self._guard_start_goal,
                poll_goal=self._guard_poll_goal,
                cancel_goal=self._guard_cancel_goal,
                current_hand_pose=self._current_hand_pose,
                build_motion_goal=self._build_motion_goal,
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
        if self._relative_command_is_noop(command):
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Skipped zero arm movement",
            )
        return self.guarded_probe(
            lambda: self._resolve_relative_probe(command),
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
            lambda: (
                self._normalize_target(
                    target,
                    execution_frame.strip()
                    or GRAV_ALIGNED_BODY_FRAME_NAME,
                ),
                BARE_HAND_MOTION_ID,
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
            lambda: self._resolve_tag_probe(command),
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
            lambda: self._resolve_probe_relative(
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
        self._guarded_plan_builder = lambda: self._build_guarded_probe_plan(
            target_builder,
            speed,
            force_threshold_n,
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
        update = self.guarded_probe_execution.start(builder)
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
            lambda: self._build_probe_pose_goal(
                probe_target,
                motion_sensor_id,
                speed,
            )
        )

    def _build_guarded_probe_plan(
        self,
        target_builder,
        speed=None,
        force_threshold_n=None,
    ) -> GuardedProbePlan:
        probe_target, motion_sensor_id = target_builder()
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Guarded probe movement requires attachment geometry"
            )

        target_probe = self._normalize_target(
            probe_target,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        current_probe = self._current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )
        effective_speed = self._effective_guarded_speed(speed)
        duration_sec = self.speed_policy.duration_between(
            current_probe.pose,
            target_probe.pose,
            speed=effective_speed,
        )

        if sensor_id == BARE_HAND_MOTION_ID:
            current_hand = deepcopy(current_probe)
            target_hand = deepcopy(target_probe)
        else:
            current_hand = self._current_hand_pose()
            target_hand = self._probe_target_to_hand_target(
                target_probe,
                sensor_id,
            )

        probe_rotation = self.speed_policy._rotation_angle(
            current_probe.pose,
            target_probe.pose,
        )
        start = current_hand.pose.position
        target = target_hand.pose.position
        dx = float(target.x) - float(start.x)
        dy = float(target.y) - float(start.y)
        dz = float(target.z) - float(start.z)
        hand_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        motion_required = (
            hand_distance > 1e-6
            or probe_rotation > 1e-6
        )
        if not motion_required:
            return GuardedProbePlan(
                goal=None,
                current_hand=deepcopy(current_hand),
                target_hand=deepcopy(target_hand),
                direction_x=0.0,
                direction_y=0.0,
                direction_z=0.0,
                linear_speed_mps=0.0,
                force_threshold_n=None,
                contact_consecutive_samples=(
                    self.force_contact_policy.consecutive_samples
                ),
                motion_required=False,
                force_guard_enabled=False,
            )

        goal = self._build_pose_goal(target_hand, duration_sec)
        if hand_distance <= 1e-6:
            return GuardedProbePlan(
                goal=goal,
                current_hand=deepcopy(current_hand),
                target_hand=deepcopy(target_hand),
                direction_x=0.0,
                direction_y=0.0,
                direction_z=0.0,
                linear_speed_mps=0.0,
                force_threshold_n=None,
                contact_consecutive_samples=(
                    self.force_contact_policy.consecutive_samples
                ),
                motion_required=True,
                force_guard_enabled=False,
            )

        actual_speed = hand_distance / duration_sec
        if force_threshold_n is None:
            threshold = self.force_contact_policy.threshold_for(
                actual_speed
            )
        else:
            threshold = float(force_threshold_n)
            if not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "Guarded probe force threshold override must be "
                    "positive and finite"
                )

        return GuardedProbePlan(
            goal=goal,
            current_hand=deepcopy(current_hand),
            target_hand=deepcopy(target_hand),
            direction_x=dx / hand_distance,
            direction_y=dy / hand_distance,
            direction_z=dz / hand_distance,
            linear_speed_mps=actual_speed,
            force_threshold_n=threshold,
            contact_consecutive_samples=(
                self.force_contact_policy.consecutive_samples
            ),
            motion_required=True,
            force_guard_enabled=True,
        )

    @staticmethod
    def _relative_command_is_noop(command) -> bool:
        offset = getattr(command, "offset", None)
        if not isinstance(offset, PoseStamped):
            return False

        position = offset.pose.position
        translation = math.sqrt(
            float(position.x) * float(position.x)
            + float(position.y) * float(position.y)
            + float(position.z) * float(position.z)
        )
        if translation > 1e-6:
            return False

        orientation = offset.pose.orientation
        values = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-12:
            return False
        x, y, z, w = (value / norm for value in values)
        return (
            abs(x) <= 1e-6
            and abs(y) <= 1e-6
            and abs(z) <= 1e-6
            and abs(abs(w) - 1.0) <= 1e-6
        )

    def _effective_guarded_speed(self, speed):
        if speed is None:
            return self.speed_policy.default_speed
        if not isinstance(speed, ArmMotionSpeed):
            raise TypeError(
                "Guarded probe speed must be an ArmMotionSpeed"
            )
        return speed

    def _resolve_relative_probe(self, command):
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative arm movement requires a command with "
                "compute_goal_pose()"
            )

        command = self.geometry_resolver.prepare_move_command(
            command,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        relative_target = command.compute_goal_pose(self.tf_listener)
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
        return target_hand, BARE_HAND_MOTION_ID

    def _resolve_tag_probe(self, command):
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
        command = self.geometry_resolver.prepare_move_command(
            command,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        probe_target = command.compute_goal_pose(self.tf_listener)
        return probe_target, command.motion_sensor_id

    def _resolve_probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
    ):
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
        if offset.header.frame_id.strip() != probe_frame:
            raise ValueError(
                "Probe-relative offset must be expressed in the "
                f"active probe frame '{probe_frame}'"
            )

        probe_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        target_probe = tf2_geometry_msgs.do_transform_pose_stamped(
            offset,
            probe_to_execution,
        )
        return target_probe, sensor_id

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

    def _current_hand_pose(self) -> PoseStamped:
        return self._current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            HAND_FRAME_NAME,
        )

    def _build_stow_goal(self) -> RobotCommand.Goal:
        stow_command = RobotCommandBuilder.arm_stow_command()
        goal = RobotCommand.Goal()
        convert(stow_command, goal.command)
        return goal

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

        target_probe = self._normalize_target(
            probe_target,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        current_probe = self._current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
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
        if sensor_id == BARE_HAND_MOTION_ID:
            hand_target = target_probe
        else:
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
            return deepcopy(target)

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
