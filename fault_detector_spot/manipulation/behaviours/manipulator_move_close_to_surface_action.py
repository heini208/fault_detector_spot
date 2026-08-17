"""Behavior-tree action for a standalone force-guarded surface approach."""

import math
import time

import py_trees
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from py_trees.common import Access, Status
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.geometry.rotation import rotation_distance_rad
from fault_detector_spot.inspection.execution.probe_surface_runtime_state import (
    ProbeSurfaceRuntimeStateSource,
)
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.sensing.end_effector_force import (
    estimate_force_baseline,
    project_probe_force_delta,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    aggregate_surface_distance_samples,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)


class ManipulatorMoveCloseToSurfaceAction(py_trees.behaviour.Behaviour):
    """Move the active probe to a requested live surface stand-off."""

    def __init__(
        self,
        name: str = "MoveCloseToSurface",
        robot_name: str = "",
        motion_duration_sec: float = 2.0,
        settle_sec: float = 0.5,
        sample_timeout_sec: float = 3.0,
        force_baseline_timeout_sec: float = 2.0,
        maximum_step_m: float = 0.010,
        tolerance_m: float = 0.005,
        maximum_travel_m: float = 0.400,
        maximum_approach_steps: int = 40,
        minimum_surface_samples: int = 5,
        minimum_surface_span_sec: float = 1.0,
        surface_stability_tolerance_m: float = 0.005,
        force_contact_threshold_n: float = 5.0,
        force_contact_consecutive_samples: int = 2,
        force_stale_timeout_sec: float = 0.5,
        maximum_contact_retries: int = 3,
        recovery_step_m: float = 0.040,
        maximum_recovery_steps: int = 20,
        maximum_lateral_drift_m: float = 0.010,
        maximum_axis_error_rad: float = math.radians(5.0),
        minimum_step_progress_ratio: float = 0.25,
    ):
        super().__init__(name)
        self.robot_name = robot_name
        self.motion_duration_sec = float(motion_duration_sec)
        self.settle_sec = float(settle_sec)
        self.sample_timeout_sec = float(sample_timeout_sec)
        self.force_baseline_timeout_sec = float(force_baseline_timeout_sec)
        self.maximum_step_m = float(maximum_step_m)
        self.tolerance_m = float(tolerance_m)
        self.maximum_travel_m = float(maximum_travel_m)
        self.maximum_approach_steps = int(maximum_approach_steps)
        self.minimum_surface_samples = int(minimum_surface_samples)
        self.minimum_surface_span_sec = float(minimum_surface_span_sec)
        self.surface_stability_tolerance_m = float(
            surface_stability_tolerance_m
        )
        self.force_contact_threshold_n = float(force_contact_threshold_n)
        self.force_contact_consecutive_samples = int(
            force_contact_consecutive_samples
        )
        self.force_stale_timeout_sec = float(force_stale_timeout_sec)
        self.maximum_contact_retries = int(maximum_contact_retries)
        self.recovery_step_m = float(recovery_step_m)
        self.maximum_recovery_steps = int(maximum_recovery_steps)
        self.maximum_lateral_drift_m = float(maximum_lateral_drift_m)
        self.maximum_axis_error_rad = float(maximum_axis_error_rad)
        self.minimum_step_progress_ratio = float(minimum_step_progress_ratio)
        self._validate_configuration()

        self.node = None
        self._client = None
        self._state_source = None
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)
        self.blackboard.register_key(
            key="command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            key="command_failure_detail",
            access=Access.WRITE,
        )
        self._clear_runtime()

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if self.node is None:
            raise RuntimeError(
                "ManipulatorMoveCloseToSurfaceAction requires a ROS node"
            )
        if self._state_source is None:
            self._state_source = ProbeSurfaceRuntimeStateSource(self.node)
        if self._client is None:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self._client = ActionClientWrapper(
                RobotCommand,
                action_ns,
                self.node,
                wait_for_server=False,
            )

    def shutdown(self):
        self._cancel_active_goal()
        if self._state_source is not None:
            self._state_source.close()
            self._state_source = None
        self._client = None

    def initialise(self):
        self._clear_runtime()
        self._phase = "acquire"
        self._phase_started = time.monotonic()
        self.feedback_message = "Resolving active probe attachment"

    def update(self) -> Status:
        try:
            command = self._command_from_blackboard()
            if self._command is None:
                self._command = command
                self.blackboard.command_failure_request_id = command.request_id
                self.blackboard.command_failure_detail = ""
            elif command is not self._command:
                return self._fail("Close-surface command changed during execution")

            if self._phase == "acquire":
                return self._update_acquire()
            if self._phase == "sampling":
                return self._update_sampling()
            if self._phase == "force_baseline":
                return self._update_force_baseline()
            if self._phase == "moving":
                return self._update_moving()
            if self._phase == "settling":
                return self._update_settling()
            if self._phase == "cancelling_for_recovery":
                return self._update_cancelling_for_recovery()
            if self._phase == "recovery_prepare":
                return self._update_recovery_prepare()
            if self._phase == "recovering":
                return self._update_recovering()
            return self._fail(f"Unknown close-surface phase: {self._phase}")
        except Exception as exception:
            if (
                self._phase in {"moving", "settling"}
                and self._recovery_hand_pose is not None
            ):
                return self._begin_motion_recovery(
                    str(exception),
                    retryable=False,
                )
            return self._fail(str(exception))

    def terminate(self, new_status: Status):
        if new_status == Status.INVALID:
            self._cancel_active_goal()
        self._clear_goal_state()

    def _update_acquire(self) -> Status:
        try:
            sensor_id, revision = self._state_source.active_attachment()
            recovery_pose = self._state_source.current_hand_pose_execution()
        except Exception as exception:
            if time.monotonic() - self._phase_started < self.sample_timeout_sec:
                self.feedback_message = f"Waiting for probe runtime state: {exception}"
                return Status.RUNNING
            return self._fail(str(exception))

        self._sensor_id = sensor_id
        self._attachment_revision = revision
        self._recovery_hand_pose = recovery_pose
        self._start_sampling()
        return Status.RUNNING

    def _update_sampling(self) -> Status:
        self._require_attachment_unchanged()
        now = time.monotonic()
        last_error = None
        try:
            fresh = self._state_source.surface_distance_samples(
                self._sensor_id,
                receipt_not_before=self._sample_receipt_not_before,
                minimum_samples=1,
            )
            for sample in fresh:
                self._surface_samples[sample.stamp_seconds] = sample
            aggregate = aggregate_surface_distance_samples(
                tuple(self._surface_samples.values()),
                self._command.target_surface_distance_m,
                self.maximum_step_m,
                tolerance_m=self.tolerance_m,
                minimum_samples=self.minimum_surface_samples,
                minimum_span_sec=self.minimum_surface_span_sec,
                stability_tolerance_m=self.surface_stability_tolerance_m,
            )
            current_probe = self._state_source.current_probe_pose_execution(
                self._sensor_id
            )
            if aggregate.verified:
                self.feedback_message = (
                    "Surface stand-off already reached: "
                    f"{aggregate.distance_m:.4f} m"
                )
                return Status.SUCCESS
            if aggregate.surface_plane_probe is None:
                raise RuntimeError(
                    "Stable surface sampling did not produce a fitted plane"
                )
            self._plan = freeze_probe_surface_approach(
                current_probe_pose_execution=current_probe,
                surface_plane_probe=aggregate.surface_plane_probe,
                target_distance_m=self._command.target_surface_distance_m,
                maximum_travel_m=self.maximum_travel_m,
            )
            if (
                self._plan.initial_axis_error_rad
                > self.maximum_axis_error_rad
            ):
                raise ValueError(
                    "Probe axis is not aligned with the fitted surface: "
                    f"{math.degrees(self._plan.initial_axis_error_rad):.2f} deg "
                    f"> {math.degrees(self.maximum_axis_error_rad):.2f} deg"
                )
            self._previous_probe_pose = current_probe
            self._baseline_receipt_not_before = time.monotonic()
            self._phase_started = self._baseline_receipt_not_before
            self._phase = "force_baseline"
            self.feedback_message = "Collecting stationary force baseline"
            return Status.RUNNING
        except Exception as exception:
            last_error = exception

        if now - self._phase_started >= self.sample_timeout_sec:
            return self._fail(
                "Unable to establish stable surface distance: "
                f"{last_error}"
            )
        self.feedback_message = f"Collecting stable surface distance: {last_error}"
        return Status.RUNNING

    def _update_force_baseline(self) -> Status:
        self._require_attachment_unchanged()
        try:
            samples = self._state_source.end_effector_force_samples(
                receipt_not_before=self._baseline_receipt_not_before,
                maximum_age_sec=self.force_baseline_timeout_sec,
            )
            baseline = estimate_force_baseline(samples)
        except Exception as exception:
            if (
                time.monotonic() - self._phase_started
                < self.force_baseline_timeout_sec
            ):
                self.feedback_message = f"Collecting force baseline: {exception}"
                return Status.RUNNING
            return self._fail(
                "Unable to establish stationary end-effector force baseline: "
                f"{exception}"
            )

        self._force_baseline = baseline
        self._force_last_receipt = max(
            sample.receipt_time for sample in samples
        )
        self._force_contact_count = 0
        self._probe_axis_hand = self._state_source.probe_axis_hand(
            self._sensor_id
        )
        return self._prepare_next_approach_step()

    def _prepare_next_approach_step(self) -> Status:
        current_probe = self._state_source.current_probe_pose_execution(
            self._sensor_id
        )
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.maximum_step_m,
            tolerance_m=self.tolerance_m,
        )
        self._validate_pose_guard(evaluation)
        if evaluation.reached:
            self.feedback_message = (
                "Reached requested surface stand-off: "
                f"{evaluation.estimated_distance_m:.4f} m"
            )
            return Status.SUCCESS

        if self._approach_steps >= self.maximum_approach_steps:
            return self._begin_motion_recovery(
                "Surface approach exceeded the maximum step count",
                retryable=False,
            )
        self._previous_probe_pose = current_probe
        self._requested_step_m = evaluation.requested_step_m
        hand_pose = self._state_source.current_hand_pose_execution()
        inward = self._plan.inward_direction()
        target = PoseData(
            position=Vector3Data(
                x=hand_pose.position.x + inward.x * self._requested_step_m,
                y=hand_pose.position.y + inward.y * self._requested_step_m,
                z=hand_pose.position.z + inward.z * self._requested_step_m,
            ),
            orientation=hand_pose.orientation,
        )
        self._approach_steps += 1
        self._send_pose_goal(target, self.motion_duration_sec)
        self._phase = "moving"
        self.feedback_message = (
            "Moving toward surface by "
            f"{self._requested_step_m:.4f} m"
        )
        return Status.RUNNING

    def _update_moving(self) -> Status:
        self._require_attachment_unchanged()
        force_error = self._check_force_guard()
        if force_error is not None:
            return self._begin_motion_recovery(
                force_error,
                retryable=force_error.startswith("Possible contact:"),
            )

        result = self._poll_goal()
        if result is None:
            return Status.RUNNING
        if result is False:
            return self._begin_motion_recovery(
                "Surface approach robot command failed",
                retryable=False,
            )

        self._settle_deadline = time.monotonic() + self.settle_sec
        self._phase = "settling"
        self.feedback_message = "Surface step reached; waiting to settle"
        return Status.RUNNING

    def _update_settling(self) -> Status:
        self._require_attachment_unchanged()
        force_error = self._check_force_guard()
        if force_error is not None:
            return self._begin_motion_recovery(
                force_error,
                retryable=force_error.startswith("Possible contact:"),
            )
        if time.monotonic() < self._settle_deadline:
            return Status.RUNNING

        current_probe = self._state_source.current_probe_pose_execution(
            self._sensor_id
        )
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.maximum_step_m,
            tolerance_m=self.tolerance_m,
        )
        try:
            self._validate_pose_guard(evaluation)
            self._validate_progress(current_probe)
        except Exception as exception:
            return self._begin_motion_recovery(
                f"Possible contact: {exception}",
                retryable=True,
            )
        return self._prepare_next_approach_step()

    def _begin_motion_recovery(self, detail: str, retryable: bool) -> Status:
        self._recovery_detail = detail
        self._recovery_retryable = bool(retryable)
        self._cancel_active_goal()
        if self._goal_active_or_pending():
            self._phase = "cancelling_for_recovery"
        else:
            self._phase = "recovery_prepare"
        self.feedback_message = f"{detail}; returning to aligned start pose"
        return Status.RUNNING

    def _update_cancelling_for_recovery(self) -> Status:
        result = self._poll_goal(allow_cancelled=True)
        if result is None:
            return Status.RUNNING
        self._phase = "recovery_prepare"
        return Status.RUNNING

    def _update_recovery_prepare(self) -> Status:
        current = self._state_source.current_hand_pose_execution()
        remaining = self._translation_between(
            current,
            self._recovery_hand_pose,
        )
        distance = self._norm(remaining)
        if distance <= 0.005:
            orientation_error = rotation_distance_rad(
                current.orientation,
                self._recovery_hand_pose.orientation,
            )
            if orientation_error > self.maximum_axis_error_rad:
                return self._fail(
                    "Surface recovery reached the start position with an "
                    "excessive orientation error of "
                    f"{math.degrees(orientation_error):.2f} deg"
                )
            return self._finish_recovery()

        if self._recovery_steps >= self.maximum_recovery_steps:
            return self._fail(
                "Surface recovery could not reach the original aligned "
                "pre-approach pose within the recovery step limit"
            )

        scale = min(1.0, self.recovery_step_m / distance)
        target = PoseData(
            position=Vector3Data(
                x=current.position.x + remaining.x * scale,
                y=current.position.y + remaining.y * scale,
                z=current.position.z + remaining.z * scale,
            ),
            orientation=self._recovery_hand_pose.orientation,
        )
        self._recovery_steps += 1
        self._send_pose_goal(target, self.motion_duration_sec)
        self._phase = "recovering"
        self.feedback_message = (
            "Recovering to aligned start pose, step "
            f"{self._recovery_steps}/{self.maximum_recovery_steps}"
        )
        return Status.RUNNING

    def _update_recovering(self) -> Status:
        result = self._poll_goal()
        if result is None:
            return Status.RUNNING
        if result is False:
            return self._fail("Surface recovery robot command failed")
        self._phase = "recovery_prepare"
        return Status.RUNNING

    def _finish_recovery(self) -> Status:
        detail = self._recovery_detail
        if (
            self._recovery_retryable
            and self._retries_used < self.maximum_contact_retries
        ):
            self._retries_used += 1
            self._recovery_steps = 0
            self._start_sampling()
            self.feedback_message = (
                f"Recovered after possible contact; retry "
                f"{self._retries_used}/{self.maximum_contact_retries}"
            )
            return Status.RUNNING
        if self._recovery_retryable:
            return self._fail(
                "Possible surface contact persisted after "
                f"{self.maximum_contact_retries} retries. Last detail: {detail}"
            )
        return self._fail(detail)

    def _start_sampling(self):
        self._surface_samples = {}
        self._sample_receipt_not_before = time.monotonic()
        self._phase_started = self._sample_receipt_not_before
        self._plan = None
        self._previous_probe_pose = None
        self._force_baseline = None
        self._probe_axis_hand = None
        self._force_contact_count = 0
        self._requested_step_m = 0.0
        self._approach_steps = 0
        self._phase = "sampling"
        self.feedback_message = "Collecting live surface distance"

    def _command_from_blackboard(self) -> MoveCloseToSurfaceCommand:
        if not self.blackboard.exists("last_command"):
            raise RuntimeError("No command is available on the blackboard")
        command = self.blackboard.last_command
        if not isinstance(command, MoveCloseToSurfaceCommand):
            raise RuntimeError(
                "Expected MoveCloseToSurfaceCommand, got "
                f"{type(command).__name__}"
            )
        return command

    def _require_attachment_unchanged(self):
        sensor_id, revision = self._state_source.active_attachment()
        if (
            sensor_id != self._sensor_id
            or revision != self._attachment_revision
        ):
            raise RuntimeError(
                "Sensor attachment changed during close-surface movement"
            )

    def _check_force_guard(self):
        now = time.monotonic()
        try:
            sample = self._state_source.latest_end_effector_force()
        except Exception as exception:
            if now - self._force_last_receipt > self.force_stale_timeout_sec:
                return (
                    "End-effector force became stale during surface approach: "
                    f"{exception}"
                )
            return None

        if sample.receipt_time <= self._force_last_receipt + 1e-12:
            if now - self._force_last_receipt > self.force_stale_timeout_sec:
                return "End-effector force stopped updating during surface approach"
            return None

        self._force_last_receipt = sample.receipt_time
        delta = project_probe_force_delta(
            sample,
            self._force_baseline,
            self._probe_axis_hand,
        )
        if delta.total_force_n >= self.force_contact_threshold_n:
            self._force_contact_count += 1
        else:
            self._force_contact_count = 0
        if self._force_contact_count < self.force_contact_consecutive_samples:
            return None
        return (
            "Possible contact: end-effector force delta "
            f"{delta.total_force_n:.2f} N total, "
            f"{delta.axial_force_n:+.2f} N axial, "
            f"{delta.lateral_force_n:.2f} N lateral"
        )

    def _validate_pose_guard(self, evaluation):
        if evaluation.lateral_offset_m > self.maximum_lateral_drift_m:
            raise RuntimeError(
                "Surface approach lateral drift exceeded the safety limit: "
                f"{evaluation.lateral_offset_m:.4f} m"
            )
        if evaluation.axis_error_rad > self.maximum_axis_error_rad:
            raise RuntimeError(
                "Probe axis rotated away from the frozen surface normal by "
                f"{math.degrees(evaluation.axis_error_rad):.2f} deg"
            )

    def _validate_progress(self, current_probe: PoseData):
        inward = self._plan.inward_direction()
        previous = self._previous_probe_pose.position
        current = current_probe.position
        delta = Vector3Data(
            x=current.x - previous.x,
            y=current.y - previous.y,
            z=current.z - previous.z,
        )
        achieved = (
            delta.x * inward.x
            + delta.y * inward.y
            + delta.z * inward.z
        )
        minimum = self._requested_step_m * self.minimum_step_progress_ratio
        if achieved + 1e-9 < minimum:
            raise RuntimeError(
                "surface approach did not achieve enough inward progress: "
                f"requested {self._requested_step_m:.4f} m, "
                f"achieved {achieved:.4f} m"
            )

    def _send_pose_goal(self, pose: PoseData, duration_sec: float):
        pose.validate()
        if not self._client.wait_for_server(timeout_sec=0.0):
            raise RuntimeError("Spot robot_command action server is unavailable")
        arm_command = RobotCommandBuilder.arm_pose_command(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
            float(duration_sec),
        )
        goal = RobotCommand.Goal()
        convert(arm_command, goal.command)
        self._clear_goal_state()
        self._send_goal_future = self._client.send_goal_async(goal)

    def _poll_goal(self, allow_cancelled=False):
        if self._goal_handle is None:
            if self._send_goal_future is None or not self._send_goal_future.done():
                return None
            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self._clear_goal_state()
                return False
            self._result_future = self._goal_handle.get_result_async()
            return None
        if self._result_future is None or not self._result_future.done():
            return None
        result_wrapper = self._result_future.result()
        result = result_wrapper.result
        success = bool(getattr(result, "success", False))
        if allow_cancelled and not success:
            success = True
        self._clear_goal_state()
        return success

    def _cancel_active_goal(self):
        if self._client is None:
            return
        if self._goal_handle is not None:
            self._client._cancel_goal_async(self._goal_handle)
            return
        future = self._send_goal_future
        if future is None or future.done():
            return

        def cancel_when_accepted(done_future):
            try:
                goal_handle = done_future.result()
            except Exception:
                return
            if goal_handle is not None and goal_handle.accepted:
                self._client._cancel_goal_async(goal_handle)

        future.add_done_callback(cancel_when_accepted)

    def _goal_active_or_pending(self):
        return (
            self._send_goal_future is not None
            or self._goal_handle is not None
            or self._result_future is not None
        )

    def _clear_goal_state(self):
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None

    def _fail(self, detail: str) -> Status:
        detail = str(detail).strip() or "Close-surface movement failed"
        self.feedback_message = detail
        request_id = getattr(self._command, "request_id", "")
        self.blackboard.command_failure_request_id = request_id
        self.blackboard.command_failure_detail = detail
        if self.node is not None:
            logger = self.node.get_logger()
            logger.error(f"[{self.name}] {detail}")
        self._cancel_active_goal()
        return Status.FAILURE

    def _clear_runtime(self):
        self._command = None
        self._phase = "idle"
        self._phase_started = 0.0
        self._sensor_id = ""
        self._attachment_revision = -1
        self._recovery_hand_pose = None
        self._surface_samples = {}
        self._sample_receipt_not_before = 0.0
        self._baseline_receipt_not_before = 0.0
        self._plan = None
        self._previous_probe_pose = None
        self._force_baseline = None
        self._probe_axis_hand = None
        self._force_last_receipt = 0.0
        self._force_contact_count = 0
        self._requested_step_m = 0.0
        self._approach_steps = 0
        self._settle_deadline = 0.0
        self._recovery_detail = ""
        self._recovery_retryable = False
        self._recovery_steps = 0
        self._retries_used = 0
        self._clear_goal_state()

    def _validate_configuration(self):
        positive = (
            self.motion_duration_sec,
            self.settle_sec,
            self.sample_timeout_sec,
            self.force_baseline_timeout_sec,
            self.maximum_step_m,
            self.tolerance_m,
            self.maximum_travel_m,
            self.minimum_surface_span_sec,
            self.surface_stability_tolerance_m,
            self.force_contact_threshold_n,
            self.force_stale_timeout_sec,
            self.recovery_step_m,
            self.maximum_lateral_drift_m,
            self.maximum_axis_error_rad,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("Close-surface configuration values must be positive")
        if self.maximum_approach_steps < 1:
            raise ValueError("Maximum surface approach steps must be positive")
        if self.maximum_step_m > 0.010 + 1e-12:
            raise ValueError("Surface approach step must not exceed 0.010 m")
        reachable_travel_m = (
            self.maximum_step_m * self.maximum_approach_steps
        )
        if self.maximum_travel_m > reachable_travel_m + 1e-12:
            raise ValueError(
                "Maximum surface approach travel exceeds the configured "
                "step-count limit"
            )
        if self.minimum_surface_samples < 3:
            raise ValueError("Surface sampling requires at least three samples")
        if self.force_contact_consecutive_samples < 1:
            raise ValueError("Force contact sample count must be positive")
        if self.maximum_contact_retries < 0:
            raise ValueError("Maximum contact retries must not be negative")
        if self.maximum_recovery_steps < 1:
            raise ValueError("Maximum recovery steps must be positive")
        if self.recovery_step_m > 0.040 + 1e-12:
            raise ValueError("Surface recovery step must not exceed 0.040 m")
        if not 0.0 < self.minimum_step_progress_ratio <= 1.0:
            raise ValueError(
                "Minimum surface-step progress ratio must be in (0, 1]"
            )

    @staticmethod
    def _translation_between(current: PoseData, target: PoseData) -> Vector3Data:
        return Vector3Data(
            x=target.position.x - current.position.x,
            y=target.position.y - current.position.y,
            z=target.position.z - current.position.z,
        )

    @staticmethod
    def _norm(vector: Vector3Data) -> float:
        return math.sqrt(
            vector.x * vector.x
            + vector.y * vector.y
            + vector.z * vector.z
        )

__all__ = ["ManipulatorMoveCloseToSurfaceAction"]
