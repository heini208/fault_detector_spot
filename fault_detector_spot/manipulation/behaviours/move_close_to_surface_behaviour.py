"""Behavior-tree workflow for repeated guarded surface approach movements."""

from copy import deepcopy
from dataclasses import dataclass, fields
import math
import time

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.geometry.rotation import rotation_distance_rad
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    aggregate_surface_distance_samples,
)
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
)
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.shared.geometry.transforms import (
    pose_data_to_pose,
    pose_to_pose_data,
)


CONTACT_MODE_PLANNING_DISTANCE_M = 0.001


@dataclass(frozen=True)
class MoveCloseToSurfaceConfig:
    settle_sec: float = 0.5
    sample_timeout_sec: float = 3.0
    maximum_step_m: float = 0.010
    tolerance_m: float = 0.005
    maximum_travel_m: float = 0.400
    maximum_approach_steps: int = 60
    minimum_surface_samples: int = 5
    minimum_surface_span_sec: float = 1.0
    surface_stability_tolerance_m: float = 0.005
    force_contact_threshold_n: float = 5.0
    force_near_target_threshold_n: float = 3.0
    force_near_target_distance_m: float = 0.020
    approach_far_speed_mps: float = 0.005
    approach_near_speed_mps: float = 0.001
    approach_slowdown_distance_m: float = 0.050
    contact_search_overtravel_m: float = 0.005
    recovery_step_m: float = 0.040
    recovery_speed_mps: float = 0.020
    maximum_recovery_steps: int = 20
    maximum_lateral_drift_m: float = 0.010
    maximum_axis_error_rad: float = math.radians(5.0)
    minimum_step_progress_ratio: float = 0.25

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        integer_fields = {
            "maximum_approach_steps",
            "minimum_surface_samples",
            "maximum_recovery_steps",
        }
        for field in fields(cls):
            name = f"close_surface.{field.name}"
            default = getattr(defaults, field.name)
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            value = node.get_parameter(name).value
            values[field.name] = (
                int(value) if field.name in integer_fields else float(value)
            )
        return cls(**values)


class MoveCloseToSurfaceBehaviour(ArmMovementBehaviour):
    """Chain guarded arm movements until stand-off or contact is reached."""

    def __init__(
        self,
        name: str = "MoveCloseToSurfaceBehaviour",
        robot_name=None,
        robot_command_resources=None,
        surface_source=None,
        config=None,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            robot_name=str(robot_name or ""),
            robot_command_resources=robot_command_resources,
        )
        self._configured_robot_name = robot_name
        self.surface_source = surface_source
        self._owns_surface_source = False
        self.config = config
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        self._clock = monotonic_clock
        self._initialise_error = ""
        if self.config is not None:
            self._validate_configuration()
        self._clear_runtime()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        if self._configured_robot_name is None:
            if not self.node.has_parameter("close_surface.robot_name"):
                self.node.declare_parameter("close_surface.robot_name", "")
            self.robot_name = str(
                self.node.get_parameter("close_surface.robot_name").value
            ).strip()
        self._ensure_executor()
        if self.surface_source is None:
            self.surface_source = (
                self.robot_command_resources.get_probe_surface_source(
                    self.node
                )
            )
        if self.config is None:
            self.config = MoveCloseToSurfaceConfig.from_node(self.node)
        self._validate_configuration()

    def initialise(self):
        super().initialise()
        self._clear_runtime()
        self._initialise_error = ""
        try:
            command = self._last_command()
            if not isinstance(command, MoveCloseToSurfaceCommand):
                raise RuntimeError(
                    "Expected MoveCloseToSurfaceCommand, got "
                    f"{type(command).__name__}"
                )
            self._command = command
            self._phase = "acquire"
            self._phase_started = self._clock()
            self._started = True
            self.feedback_message = "Resolving active probe attachment"
        except Exception as exception:
            self._initialise_error = str(exception)

    def update(self) -> Status:
        try:
            if self._initialise_error:
                return self._fail_workflow(self._initialise_error)
            self._ensure_executor()
            if self.surface_source is None:
                return self._fail_workflow(
                    "Close-surface surface source is not configured"
                )
            if self.config is None:
                return self._fail_workflow(
                    "Close-surface configuration is not available"
                )

            if self._phase == "acquire":
                return self._update_acquire()
            if self._phase == "sampling":
                return self._update_sampling()
            if self._phase == "approach":
                return self._handle_approach_update(self.executor.poll())
            if self._phase == "settling":
                return self._update_settling()
            if self._phase == "recovery_prepare":
                return self._update_recovery_prepare()
            if self._phase == "recovering":
                return self._handle_recovery_update(self.executor.poll())
            return self._fail_workflow(
                f"Unknown close-surface phase: {self._phase}"
            )
        except Exception as exception:
            if self._approach_steps > 0 and self._recovery_hand_pose is not None:
                return self._begin_recovery(str(exception))
            return self._fail_workflow(str(exception))

    def terminate(self, new_status: Status):
        if new_status is Status.INVALID and self.executor is not None:
            if self.executor.active:
                self.executor.cancel()
        self._started = False

    def shutdown(self):
        if self.executor is not None and self.executor.active:
            self.executor.cancel()
        if self._owns_surface_source and self.surface_source is not None:
            self.surface_source.destroy()
            self.surface_source = None
        self._started = False
        self._initialise_error = ""

    @property
    def contact_mode(self) -> bool:
        return (
            self._command is not None
            and self._command.target_surface_distance_m == 0.0
        )

    def _update_acquire(self) -> Status:
        try:
            sensor_id, revision = self.surface_source.active_attachment()
            recovery_pose = self._current_hand_pose()
        except Exception as exception:
            if self._clock() - self._phase_started < self.config.sample_timeout_sec:
                self.feedback_message = (
                    f"Waiting for probe runtime state: {exception}"
                )
                return Status.RUNNING
            return self._fail_workflow(str(exception))

        self._sensor_id = sensor_id
        self._attachment_revision = revision
        self._recovery_hand_pose = deepcopy(recovery_pose)
        self._start_sampling()
        return Status.RUNNING

    def _start_sampling(self) -> None:
        self._surface_samples = {}
        self._sample_receipt_not_before = self._clock()
        self._phase_started = self._sample_receipt_not_before
        self._plan = None
        self._previous_probe_pose = None
        self._approach_steps = 0
        self._phase = "sampling"
        self.feedback_message = "Collecting initial live surface distance"

    def _update_sampling(self) -> Status:
        self._require_attachment_unchanged()
        now = self._clock()
        last_error = None

        try:
            fresh = self.surface_source.surface_distance_samples(
                self._sensor_id,
                receipt_not_before=self._sample_receipt_not_before,
                maximum_age_sec=max(
                    self.config.sample_timeout_sec,
                    self.config.minimum_surface_span_sec,
                ),
                minimum_samples=self.config.minimum_surface_samples,
            )
            for sample in fresh:
                self._surface_samples[sample.stamp_seconds] = sample

            planning_target = self._planning_target_distance()
            aggregate = aggregate_surface_distance_samples(
                tuple(self._surface_samples.values()),
                planning_target,
                self.config.maximum_step_m,
                tolerance_m=self._surface_tolerance(),
                minimum_samples=self.config.minimum_surface_samples,
                minimum_span_sec=self.config.minimum_surface_span_sec,
                stability_tolerance_m=(
                    self.config.surface_stability_tolerance_m
                ),
            )
            current_probe = self._current_probe_pose()

            if not self.contact_mode and aggregate.verified:
                return self._success(
                    "Surface stand-off already reached from live measurement: "
                    f"{aggregate.distance_m:.4f} m"
                )
            if aggregate.surface_plane_probe is None:
                raise RuntimeError(
                    "Stable surface sampling did not produce surface geometry"
                )

            self._plan = freeze_probe_surface_approach(
                current_probe_pose_execution=current_probe,
                surface_plane_probe=aggregate.surface_plane_probe,
                target_distance_m=planning_target,
                maximum_travel_m=self.config.maximum_travel_m,
            )
            if (
                self._plan.initial_axis_error_rad
                > self.config.maximum_axis_error_rad
            ):
                raise ValueError(
                    "Probe axis is not aligned with the estimated surface: "
                    f"{math.degrees(self._plan.initial_axis_error_rad):.2f} deg "
                    f"> {math.degrees(self.config.maximum_axis_error_rad):.2f} deg"
                )

            self._previous_probe_pose = deepcopy(current_probe)
            self._aligned_probe_orientation = deepcopy(
                current_probe.orientation
            )
            mode = "contact" if self.contact_mode else "stand-off"
            self.feedback_message = (
                f"Frozen {mode} approach from live surface estimate at "
                f"{aggregate.distance_m:.4f} m"
            )
            return self._prepare_next_approach_step()
        except Exception as exception:
            last_error = exception

        detail = str(last_error)
        if self._surface_samples:
            detail += (
                f"; {len(self._surface_samples)} distinct valid frame(s) "
                "accumulated"
            )

        if now - self._phase_started >= self.config.sample_timeout_sec:
            return self._fail_workflow(
                "Unable to establish stable surface distance: "
                f"{detail}"
            )
        self.feedback_message = (
            "Collecting initial surface distance: "
            f"{detail}"
        )
        return Status.RUNNING

    def _prepare_next_approach_step(self) -> Status:
        self._require_attachment_unchanged()
        current_probe = self._current_probe_pose()
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.config.maximum_step_m,
            tolerance_m=self._surface_tolerance(),
        )
        self._validate_axis_guard(evaluation)

        if evaluation.reached and not self.contact_mode:
            return self._success(
                "Reached requested surface stand-off from frozen estimate: "
                f"{evaluation.estimated_distance_m:.4f} m"
            )
        if self._approach_steps >= self.config.maximum_approach_steps:
            return self._begin_recovery(
                "Surface approach exceeded the maximum step count"
            )

        requested_step_m = self._requested_step(evaluation)
        if requested_step_m <= 0.0:
            return self._begin_recovery(
                "Contact-seeking approach reached its travel limit without "
                "detecting contact"
            )

        self._previous_probe_pose = deepcopy(current_probe)
        self._requested_step_m = requested_step_m
        threshold_n = self._force_threshold_for(
            evaluation.remaining_inward_travel_m
        )
        speed = self._approach_speed_for(
            evaluation.estimated_distance_m
        )

        inward = self._plan.inward_direction()
        target = PoseData(
            position=Vector3Data(
                x=current_probe.position.x + inward.x * requested_step_m,
                y=current_probe.position.y + inward.y * requested_step_m,
                z=current_probe.position.z + inward.z * requested_step_m,
            ),
            orientation=deepcopy(self._aligned_probe_orientation),
        )
        self._approach_steps += 1
        self._phase = "approach"
        update = self.executor.guarded_probe(
            self._pose_stamped(target),
            self._sensor_id,
            speed=speed,
            force_threshold_n=threshold_n,
        )
        self.feedback_message = (
            "Approaching surface by "
            f"{requested_step_m:.4f} m at {speed.linear_speed_mps:.4f} m/s "
            f"with {threshold_n:.2f} N guard "
            f"(step {self._approach_steps}/"
            f"{self.config.maximum_approach_steps})"
        )
        return self._handle_approach_update(update)

    def _handle_approach_update(self, update) -> Status:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return Status.RUNNING
        if update.outcome is ArmMovementOutcome.CONTACT:
            if self.contact_mode:
                return self._success(
                    "Surface contact detected; shared arm guard completed "
                    "the local snap retreat"
                )
            return self._begin_recovery(
                "Unexpected contact before the requested stand-off was "
                f"reached: {update.detail}"
            )
        if update.outcome is ArmMovementOutcome.SUCCESS:
            self._settle_deadline = self._clock() + self.config.settle_sec
            self._phase = "settling"
            self.feedback_message = "Surface step completed; waiting to settle"
            return Status.RUNNING
        return self._begin_recovery(
            "Surface approach movement failed: "
            f"{update.outcome.value}: {update.detail}"
        )

    def _update_settling(self) -> Status:
        self._require_attachment_unchanged()
        if self._clock() < self._settle_deadline:
            return Status.RUNNING

        current_probe = self._current_probe_pose()
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.config.maximum_step_m,
            tolerance_m=self._surface_tolerance(),
        )
        self._validate_axis_guard(evaluation)
        achieved, lateral = self._validate_step_motion(current_probe)
        self.feedback_message = (
            f"Step settled: inward {achieved:.4f} m, lateral "
            f"{lateral:.4f} m; estimated surface distance "
            f"{evaluation.estimated_distance_m:.4f} m"
        )
        return self._prepare_next_approach_step()

    def _begin_recovery(self, detail: str) -> Status:
        self._recovery_detail = str(detail).strip()
        if self.executor is not None and self.executor.active:
            self.executor.cancel()
        if self._recovery_hand_pose is None or self._approach_steps == 0:
            return self._fail_workflow(self._recovery_detail)
        self._phase = "recovery_prepare"
        self.feedback_message = (
            f"{self._recovery_detail}; returning to the original "
            "pre-approach hand pose"
        )
        self._log_warning(self.feedback_message)
        return self._update_recovery_prepare()

    def _update_recovery_prepare(self) -> Status:
        current = self._current_hand_pose()
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
            if orientation_error > self.config.maximum_axis_error_rad:
                return self._fail_workflow(
                    "Surface recovery reached the start position with an "
                    "excessive orientation error of "
                    f"{math.degrees(orientation_error):.2f} deg"
                )
            return self._fail_workflow(self._recovery_detail)

        if self._recovery_steps >= self.config.maximum_recovery_steps:
            return self._fail_workflow(
                "Surface recovery could not reach the original aligned "
                "pre-approach pose within the recovery step limit. "
                f"Original failure: {self._recovery_detail}"
            )

        scale = min(1.0, self.config.recovery_step_m / distance)
        target = PoseData(
            position=Vector3Data(
                x=current.position.x + remaining.x * scale,
                y=current.position.y + remaining.y * scale,
                z=current.position.z + remaining.z * scale,
            ),
            orientation=deepcopy(self._recovery_hand_pose.orientation),
        )
        self._recovery_steps += 1
        self._phase = "recovering"
        update = self.executor.probe(
            self._pose_stamped(target),
            BARE_HAND_MOTION_ID,
            speed=self._speed(self.config.recovery_speed_mps),
        )
        self.feedback_message = (
            "Recovering to original pre-approach pose, step "
            f"{self._recovery_steps}/{self.config.maximum_recovery_steps}"
        )
        return self._handle_recovery_update(update)

    def _handle_recovery_update(self, update) -> Status:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return Status.RUNNING
        if update.outcome is ArmMovementOutcome.SUCCESS:
            self._phase = "recovery_prepare"
            return Status.RUNNING
        return self._fail_workflow(
            "Surface recovery movement failed: "
            f"{update.outcome.value}: {update.detail}. Original failure: "
            f"{self._recovery_detail}"
        )

    def _current_probe_pose(self) -> PoseData:
        current = self.executor.probe_motion_planner.current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(self._sensor_id),
        )
        return pose_to_pose_data(current)

    def _current_hand_pose(self) -> PoseData:
        return pose_to_pose_data(
            self.executor.probe_motion_planner.current_hand_pose(
                GRAV_ALIGNED_BODY_FRAME_NAME
            )
        )

    def _requested_step(self, evaluation) -> float:
        if not self.contact_mode:
            return float(evaluation.requested_step_m)

        traveled = max(0.0, float(evaluation.traveled_inward_m))
        remaining_guard = self.config.maximum_travel_m - traveled
        if remaining_guard <= 1e-9:
            return 0.0

        remaining = max(0.0, float(evaluation.remaining_inward_travel_m))
        if evaluation.reached or remaining <= self.config.maximum_step_m:
            desired = max(
                self.config.contact_search_overtravel_m,
                remaining + self.config.contact_search_overtravel_m,
            )
            return min(
                self.config.maximum_step_m,
                remaining_guard,
                desired,
            )
        return min(float(evaluation.requested_step_m), remaining_guard)

    def _approach_speed_for(self, estimated_surface_distance_m: float):
        distance = max(0.0, float(estimated_surface_distance_m))
        fraction = min(
            1.0,
            distance / self.config.approach_slowdown_distance_m,
        )
        linear_speed = (
            self.config.approach_near_speed_mps
            + (
                self.config.approach_far_speed_mps
                - self.config.approach_near_speed_mps
            )
            * fraction
        )
        return self._speed(linear_speed)

    def _speed(self, linear_speed_mps: float) -> ArmMotionSpeed:
        default = self.executor.speed_policy.default_speed
        return ArmMotionSpeed(
            linear_speed_mps=float(linear_speed_mps),
            angular_speed_rad_s=default.angular_speed_rad_s,
        )

    def _force_threshold_for(self, remaining_inward_travel_m: float) -> float:
        remaining = max(0.0, float(remaining_inward_travel_m))
        fraction = min(
            1.0,
            remaining / self.config.force_near_target_distance_m,
        )
        return (
            self.config.force_near_target_threshold_n
            + (
                self.config.force_contact_threshold_n
                - self.config.force_near_target_threshold_n
            )
            * fraction
        )

    def _planning_target_distance(self) -> float:
        target = float(self._command.target_surface_distance_m)
        if target > 0.0:
            return target
        return CONTACT_MODE_PLANNING_DISTANCE_M

    def _surface_tolerance(self) -> float:
        requested = float(
            getattr(self._command, "surface_tolerance_m", 0.0)
        )
        if requested > 0.0:
            return requested
        return float(self.config.tolerance_m)

    def _require_attachment_unchanged(self) -> None:
        sensor_id, revision = self.surface_source.active_attachment()
        if (
            sensor_id != self._sensor_id
            or revision != self._attachment_revision
        ):
            raise RuntimeError(
                "Sensor attachment changed during close-surface movement"
            )

    def _validate_axis_guard(self, evaluation) -> None:
        if evaluation.axis_error_rad > self.config.maximum_axis_error_rad:
            raise RuntimeError(
                "Probe axis rotated away from the frozen surface normal by "
                f"{math.degrees(evaluation.axis_error_rad):.2f} deg"
            )

    def _validate_step_motion(self, current_probe: PoseData):
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
        lateral_vector = Vector3Data(
            x=delta.x - inward.x * achieved,
            y=delta.y - inward.y * achieved,
            z=delta.z - inward.z * achieved,
        )
        lateral = self._norm(lateral_vector)
        if lateral > self.config.maximum_lateral_drift_m:
            raise RuntimeError(
                "per-step lateral drift exceeded the safety limit: "
                f"{lateral:.4f} m > "
                f"{self.config.maximum_lateral_drift_m:.4f} m"
            )
        minimum = (
            self._requested_step_m
            * self.config.minimum_step_progress_ratio
        )
        if achieved + 1e-9 < minimum:
            raise RuntimeError(
                "surface approach did not achieve enough inward progress: "
                f"requested {self._requested_step_m:.4f} m, "
                f"achieved {achieved:.4f} m"
            )
        return achieved, lateral

    def _success(self, detail: str) -> Status:
        self.feedback_message = str(detail).strip()
        self._started = False
        return Status.SUCCESS

    def _fail_workflow(self, detail: str) -> Status:
        return super()._fail(detail)

    def _clear_runtime(self) -> None:
        self._command = None
        self._phase = "idle"
        self._phase_started = 0.0
        self._sensor_id = ""
        self._attachment_revision = -1
        self._recovery_hand_pose = None
        self._aligned_probe_orientation = None
        self._surface_samples = {}
        self._sample_receipt_not_before = 0.0
        self._plan = None
        self._previous_probe_pose = None
        self._requested_step_m = 0.0
        self._approach_steps = 0
        self._settle_deadline = 0.0
        self._recovery_detail = ""
        self._recovery_steps = 0

    def _validate_configuration(self) -> None:
        c = self.config
        positive = (
            c.settle_sec,
            c.sample_timeout_sec,
            c.maximum_step_m,
            c.tolerance_m,
            c.maximum_travel_m,
            c.minimum_surface_span_sec,
            c.surface_stability_tolerance_m,
            c.force_contact_threshold_n,
            c.force_near_target_threshold_n,
            c.force_near_target_distance_m,
            c.approach_far_speed_mps,
            c.approach_near_speed_mps,
            c.approach_slowdown_distance_m,
            c.contact_search_overtravel_m,
            c.recovery_step_m,
            c.recovery_speed_mps,
            c.maximum_lateral_drift_m,
            c.maximum_axis_error_rad,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("Close-surface configuration values must be positive")
        if c.force_near_target_threshold_n > c.force_contact_threshold_n:
            raise ValueError(
                "Near-target force threshold must not exceed the normal threshold"
            )
        if c.approach_near_speed_mps > c.approach_far_speed_mps:
            raise ValueError(
                "Near-surface approach speed must not exceed far approach speed"
            )
        if c.maximum_approach_steps < 1:
            raise ValueError("Maximum surface approach steps must be positive")
        if c.maximum_step_m > 0.010 + 1e-12:
            raise ValueError("Surface approach step must not exceed 0.010 m")
        if c.contact_search_overtravel_m > c.maximum_step_m + 1e-12:
            raise ValueError(
                "Contact search overtravel must not exceed the maximum step"
            )
        if c.maximum_travel_m > (
            c.maximum_step_m * c.maximum_approach_steps + 1e-12
        ):
            raise ValueError(
                "Maximum surface approach travel exceeds the configured "
                "step-count limit"
            )
        if c.minimum_surface_samples < 3:
            raise ValueError("Surface sampling requires at least three samples")
        if c.maximum_recovery_steps < 1:
            raise ValueError("Maximum recovery steps must be positive")
        if c.recovery_step_m > 0.040 + 1e-12:
            raise ValueError("Surface recovery step must not exceed 0.040 m")
        if not 0.0 < c.minimum_step_progress_ratio <= 1.0:
            raise ValueError(
                "Minimum surface-step progress ratio must be in (0, 1]"
            )

    @staticmethod
    def _pose_stamped(data: PoseData) -> PoseStamped:
        data.validate()
        stamped = PoseStamped()
        stamped.header.frame_id = GRAV_ALIGNED_BODY_FRAME_NAME
        stamped.pose = pose_data_to_pose(data)
        return stamped

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

    def _log_warning(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().warning(f"[{self.name}] {message}")


__all__ = [
    "CONTACT_MODE_PLANNING_DISTANCE_M",
    "MoveCloseToSurfaceBehaviour",
    "MoveCloseToSurfaceConfig",
]
